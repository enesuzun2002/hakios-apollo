/*
 * The speaker amplifiers of the Xiaomi Mi 10T (apollo), step 4 of apollo_audio=N (bsd/dev/hakios.apollo_sound.c calls these
 * as hakios.sound.c calls hakios_speaker_* for its Awinic ones).
 *
 * Two Cirrus CS35L41 (revision B2) on I2C: the serial engine 3 of the first QUPv3 wrapper (0x98c000; GPIO 119 SDA and
 * 120 SCL, function qup3), at 0x40 (the speaker) and 0x42 (the earpiece, Android's "RCV"), both on reset GPIO 114.
 * They are fed by the tertiary TDM interface: 8 slots of 32 bits, the speaker's channel in slot 1 and the earpiece's
 * in slot 0 (as Android leaves their registers: .device-local/apollo/audio/NOTES-amps.txt), here at 48 kHz.
 *
 * Set up as Linux's cs35l41 driver sets one up (sound/soc/codecs/cs35l41.c, cs35l41-lib.c) for its internal boost:
 * reset, the OTP boot awaited, the revision B2 errata and the OTP's trims unpacked under the test key, the boost from
 * the tree's 1 uH / 15 uF / 4 A (what Android leaves: 0x2424, 0x7500, 0x40), the serial port as TDM (DSP_A), clocks
 * from the SoC, 24-bit samples in 32-bit slots, the PLL from the bit clock (12.288 MHz: REFCLK 0x33), 48 kHz, and the
 * DAC fed from the serial port's first channel (ASPRX1) -- not through the Cirrus DSP, whose speaker protection
 * firmware Android runs and this does not. In its place: the analog gain at 12 of Android's 18, the digital volume
 * muted until hakios.apollo_sound.c asks for a level, and never above what it asks (its default: 20 dB under Android's).
 *
 * Boot-arg apollo_amps_dsp=1 or 2 also loads (and with 2 starts) the speaker protection firmware, still without routing the
 * sound through it: see "the speaker protection firmware" below. Default 0: as before.
 * Boot-arg apollo_amps=1 the speaker only, 2 the earpiece only, 3 both (default).
 * I2C through the serial engine as Linux's i2c-qcom-geni.c drives it: FIFO mode, polled, 400 kHz.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/* into the loader's log as well (as the other apollo drivers) */
extern void loader_log_putc(char c);

static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[300];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	return printf("%s", line);
}

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static inline void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
}

static void
sleep_us(uint32_t us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

/* ---- pins (TLMM, north tile: one page per pin) --------------------------------------------------------- */

#define TLMM_NORTH_PHYS         0x0f900000UL
#define PIN_RESET               114
#define PIN_SDA                 119
#define PIN_SCL                 120
#define FUNCTION_QUP3           1

static vm_offset_t pin_reset, pins_i2c;         /* pin 114's page; 119 and 120's two */

static vm_offset_t
pin(unsigned n)
{
	return n == PIN_RESET ? pin_reset : pins_i2c + 0x1000UL * (n - PIN_SDA);
}

static void
pin_out(unsigned n, bool high)
{
	wr(pin(n) + 4, high ? 2 : 0);
	wr(pin(n), (2 / 2 - 1) << 6 | 1u << 9);         /* gpio, no pull, 2 mA, output */
}

/* ---- clocks (GCC, kona) ------------------------------------------------------------------------------- */

#define GCC_PHYS                0x00100000UL
#define GCC_SIZE                0x60000UL
#define GCC_VOTE                0x52008
#define VOTE_QUP0               ((1u << 6) | (1u << 7) | (1u << 8) | (1u << 9))        /* m_ahb, s_ahb, core, core_2x */
#define VOTE_QUP0_SE3           (1u << 13)
#define QUP0_M_AHB_CBCR         0x17004
#define QUP0_S_AHB_CBCR         0x17008
#define QUP0_SE3_CBCR           0x1739c
#define QUP0_SE3_RCG            0x173a0         /* cmd; cfg +4; DFS: +0x14, levels at +0x1c, +0x5c, +0x9c */
#define CLOCK_OFF               (1u << 31)
#define CLOCK_HW_CTL            (1u << 1)
#define XO_HZ                   19200000u

static vm_offset_t gcc;

static uint32_t
rcg_rate(uint32_t cfg, uint32_t m, uint32_t not_n_minus_m)
{
	static const uint32_t parent[8] = { XO_HZ, 600000000, 0, 0, 0, 0, 300000000, 0 };
	uint64_t hz = parent[cfg >> 8 & 7];
	uint32_t divider = cfg & 0x1f, mode = cfg >> 12 & 3;

	if (divider) {
		hz = hz * 2 / (divider + 1);
	}
	if (mode) {
		uint32_t n = (~not_n_minus_m & 0xffff) + (m & 0xffff);

		if (n) {
			hz = hz * (m & 0xffff) / n;
		}
	}
	return (uint32_t)hz;
}

/* the serial engine's clock: XO's 19.2 MHz, the rate i2c-qcom-geni.c's table is for; which preset gives it */
static int
clocks_on(uint32_t *select)
{
	uint32_t s_ahb;

	apollo_gcc_vote(gcc + GCC_VOTE, VOTE_QUP0 | VOTE_QUP0_SE3);    /* (locked: the other drivers vote in the same register) */
	*select = 0;
	if (rd(gcc + QUP0_SE3_RCG + 0x14) & 1) {
		int found = -1;

		for (uint32_t level = 0; level < 8 && found < 0; level++) {
			if (rcg_rate(rd(gcc + QUP0_SE3_RCG + 0x1c + 4 * level), rd(gcc + QUP0_SE3_RCG + 0x5c + 4 * level),
			    rd(gcc + QUP0_SE3_RCG + 0x9c + 4 * level)) == XO_HZ) {
				found = (int)level;
			}
		}
		if (found < 0) {
			say("apollo cs35l41: serial engine 3 has no 19.2 MHz preset\n");
			return -1;
		}
		*select = (uint32_t)found;
	} else {
		wr(gcc + QUP0_SE3_RCG + 4, (rd(gcc + QUP0_SE3_RCG + 4) & ~0x371fu) | 1);
		wr(gcc + QUP0_SE3_RCG, rd(gcc + QUP0_SE3_RCG) | 1);
		for (int i = 0; i < 500 && (rd(gcc + QUP0_SE3_RCG) & 1); i++) {
			sleep_us(10);
		}
	}
	for (int i = 0; i < 1000; i++) {
		s_ahb = rd(gcc + QUP0_S_AHB_CBCR);
		s_ahb = (s_ahb & CLOCK_HW_CTL) ? s_ahb & ~CLOCK_OFF : s_ahb;       /* gated by the hardware: idle, not off */
		if (!((rd(gcc + QUP0_M_AHB_CBCR) | s_ahb | rd(gcc + QUP0_SE3_CBCR)) & CLOCK_OFF)) {
			return 0;
		}
		sleep_us(10);
	}
	say("apollo cs35l41: the clocks stay off: %#x %#x %#x\n", rd(gcc + QUP0_M_AHB_CBCR), rd(gcc + QUP0_S_AHB_CBCR),
	    rd(gcc + QUP0_SE3_CBCR));
	return -1;
}

/* ---- I2C through the serial engine (i2c-qcom-geni.c) ------------------------------------------------------- */

#define SE_PHYS                 0x0098c000UL    /* the fourth serial engine of the first QUPv3 wrapper */
#define SE_SIZE                 0x4000UL
#define GENI_FORCE_DEFAULT      0x020
#define GENI_OUTPUT_CTRL        0x024
#define GENI_CGC_CTRL           0x028
#define GENI_STATUS             0x040
#define GENI_SER_M_CLK_CFG      0x048
#define GENI_IF_DISABLE_RO      0x064
#define GENI_FW_REVISION_RO     0x068           /* [15:8] protocol: 3 = I2C */
#define GENI_CLK_SEL            0x07c
#define GENI_BYTE_GRAN          0x254
#define GENI_DMA_MODE_EN        0x258
#define GENI_TX_PACKING_CFG0    0x260
#define GENI_TX_PACKING_CFG1    0x264
#define I2C_TX_TRANS_LEN        0x26c
#define I2C_RX_TRANS_LEN        0x270
#define I2C_SCL_COUNTERS        0x278
#define GENI_RX_PACKING_CFG0    0x284
#define GENI_RX_PACKING_CFG1    0x288
#define GENI_M_CMD0             0x600
#define GENI_M_CMD_CTRL         0x604
#define GENI_M_IRQ_STATUS       0x610
#define GENI_M_IRQ_EN           0x614
#define GENI_M_IRQ_CLEAR        0x618
#define GENI_S_IRQ_CLEAR        0x648
#define GENI_TX_FIFO            0x700
#define GENI_RX_FIFO            0x780
#define GENI_TX_FIFO_STATUS     0x800
#define GENI_RX_FIFO_STATUS     0x804
#define GENI_TX_WATERMARK       0x80c
#define GENI_RX_WATERMARK       0x810
#define GENI_RX_RFR_WATERMARK   0x814
#define SE_DMA_TX_IRQ_CLR       0xc44
#define SE_DMA_RX_IRQ_CLR       0xd44
#define SE_GSI_EVENT_EN         0xe18
#define SE_IRQ_EN               0xe1c
#define SE_HW_PARAM_0           0xe24
#define SE_DMA_GENERAL_CFG      0xe30

#define I2C_WRITE               1u
#define I2C_READ                2u
#define I2C_STOP_STRETCH        (1u << 2)       /* no stop at the end: a repeated start follows */
#define M_CMD_DONE              (1u << 0)
#define M_CMD_CANCEL            (1u << 4)
#define M_CMD_ABORT             (1u << 5)
#define M_NACK                  (1u << 10)      /* M_GP_IRQ_1: the address or a byte was not acknowledged */
#define M_ERRORS                ((1u << 1) | (1u << 2) | (1u << 3) | M_NACK | (1u << 11) | (1u << 12) | (1u << 13) | \
	                         (1u << 24) | (1u << 25) | (1u << 28) | (1u << 29))

static vm_offset_t se;
static uint32_t fifo_depth;

static const char *
i2c_init(uint32_t select)
{
	if ((rd(se + GENI_FW_REVISION_RO) >> 8 & 0xff) != 3) {
		return "serial engine 3 has no I2C firmware";
	}
	if (rd(se + GENI_IF_DISABLE_RO) & 1) {
		return "serial engine 3's FIFO interface is disabled";
	}
	fifo_depth = rd(se + SE_HW_PARAM_0) >> 16 & 0x3f;
	if (fifo_depth < 4) {
		return "serial engine 3 reports no FIFO";
	}
	wr(se + SE_GSI_EVENT_EN, 0);
	wr(se + GENI_M_IRQ_CLEAR, 0xffffffff);
	wr(se + GENI_S_IRQ_CLEAR, 0xffffffff);
	wr(se + SE_DMA_TX_IRQ_CLR, 0xffffffff);
	wr(se + SE_DMA_RX_IRQ_CLR, 0xffffffff);
	wr(se + GENI_CGC_CTRL, rd(se + GENI_CGC_CTRL) | 0x7f);
	wr(se + SE_DMA_GENERAL_CFG, rd(se + SE_DMA_GENERAL_CFG) | 0xf);
	wr(se + GENI_OUTPUT_CTRL, 0x7f);
	wr(se + GENI_FORCE_DEFAULT, 1);
	wr(se + GENI_DMA_MODE_EN, rd(se + GENI_DMA_MODE_EN) & ~1u);
	wr(se + GENI_RX_WATERMARK, fifo_depth - 3);
	wr(se + GENI_RX_RFR_WATERMARK, fifo_depth - 2);
	wr(se + GENI_TX_WATERMARK, 0);
	wr(se + SE_IRQ_EN, rd(se + SE_IRQ_EN) | 0xf);
	wr(se + GENI_M_IRQ_EN, rd(se + GENI_M_IRQ_EN) | 0x7fc0007f);
	wr(se + GENI_TX_PACKING_CFG0, 0x0007f8fe);      /* bytes, the first in the low byte of a word (as hakios.apollo_touch.c) */
	wr(se + GENI_TX_PACKING_CFG1, 0x000ffefe);
	wr(se + GENI_RX_PACKING_CFG0, 0x0007f8fe);
	wr(se + GENI_RX_PACKING_CFG1, 0x000ffefe);
	wr(se + GENI_BYTE_GRAN, 0);
	/* 400 kHz from 19.2 MHz (i2c-qcom-geni.c's geni_i2c_clk_map): divider 2, high 5, low 12, cycle 24 */
	wr(se + GENI_CLK_SEL, select & 7);
	wr(se + GENI_SER_M_CLK_CFG, 2u << 4 | 1);
	wr(se + I2C_SCL_COUNTERS, 5u << 20 | 12u << 10 | 24u);
	return NULL;
}

static void
i2c_recover(void)
{
	wr(se + GENI_M_CMD_CTRL, 1u << 2);
	for (int i = 0; i < 200 && !(rd(se + GENI_M_IRQ_STATUS) & M_CMD_CANCEL); i++) {
		sleep_us(50);
	}
	if (!(rd(se + GENI_M_IRQ_STATUS) & M_CMD_CANCEL)) {
		wr(se + GENI_M_CMD_CTRL, 1u << 1);
		for (int i = 0; i < 200 && !(rd(se + GENI_M_IRQ_STATUS) & M_CMD_ABORT); i++) {
			sleep_us(50);
		}
	}
	for (int i = 0; i < 64 && (rd(se + GENI_RX_FIFO_STATUS) & 0x01ffffff); i++) {
		(void)rd(se + GENI_RX_FIFO);
	}
	wr(se + GENI_M_IRQ_CLEAR, 0xffffffff);
}

static unsigned i2c_failures;

/* One transfer: write `length` bytes of `tx`, or read `length` into `rx`; `stop`: a stop at the end */
static bool
i2c_xfer(uint8_t address, const uint8_t *tx, uint8_t *rx, uint32_t length, bool stop)
{
	uint32_t sent = 0, received = 0, status = 0, i;

	for (i = 0; i < 200 && (rd(se + GENI_STATUS) & 1); i++) {
		sleep_us(10);
	}
	wr(se + GENI_M_IRQ_CLEAR, 0xffffffff);
	wr(se + (tx != NULL ? I2C_TX_TRANS_LEN : I2C_RX_TRANS_LEN), length);
	wr(se + GENI_M_CMD0, (tx != NULL ? I2C_WRITE : I2C_READ) << 27 | (uint32_t)address << 9 | (stop ? 0 : I2C_STOP_STRETCH));
	for (i = 0; i < 4000; i++) {                    /* 20 ms at most: four bytes take 0.1 ms at 400 kHz */
		while (tx != NULL && sent < length && (rd(se + GENI_TX_FIFO_STATUS) & 0x0fffffff) + 1 < fifo_depth) {
			uint32_t word = 0;

			for (int j = 0; j < 4 && sent < length; j++, sent++) {
				word |= (uint32_t)tx[sent] << (8 * j);
			}
			wr(se + GENI_TX_FIFO, word);
		}
		if (rx != NULL) {
			uint32_t words = rd(se + GENI_RX_FIFO_STATUS) & 0x01ffffff;

			while (words--) {
				uint32_t word = rd(se + GENI_RX_FIFO);

				for (int j = 0; j < 4 && received < length; j++) {
					rx[received++] = (uint8_t)(word >> (8 * j));
				}
			}
		}
		status = rd(se + GENI_M_IRQ_STATUS);
		if (status & M_ERRORS) {
			break;
		}
		if ((status & M_CMD_DONE) && (rx == NULL || received == length)) {
			wr(se + GENI_M_IRQ_CLEAR, status);
			return true;
		}
		sleep_us(5);
	}
	if (i2c_failures++ < 8) {
		say("apollo cs35l41: I2C %s of %u bytes at %#x failed: status %#x%s (sent %u, received %u)\n", tx ? "write" : "read",
		    length, address, status, (status & M_NACK) ? ", NOT ACKNOWLEDGED" : "", sent, received);
	}
	i2c_recover();
	return false;
}

/* ---- the amplifiers' registers: 32-bit addresses and values, big-endian --------------------------------------- */

static bool
reg_read(uint8_t amp, uint32_t reg, uint32_t *value)
{
	uint8_t a[4] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg }, v[4];

	if (!i2c_xfer(amp, a, NULL, 4, false) || !i2c_xfer(amp, NULL, v, 4, true)) {
		return false;
	}
	*value = (uint32_t)v[0] << 24 | (uint32_t)v[1] << 16 | (uint32_t)v[2] << 8 | v[3];
	return true;
}

static bool
reg_write(uint8_t amp, uint32_t reg, uint32_t value)
{
	uint8_t b[8] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg,
		         (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)value };

	return i2c_xfer(amp, b, NULL, 8, true);
}

static bool
reg_update(uint8_t amp, uint32_t reg, uint32_t mask, uint32_t value)
{
	uint32_t old;

	return reg_read(amp, reg, &old) && reg_write(amp, reg, (old & ~mask) | (value & mask));
}

#define REG_DEVID               0x00000000
#define REG_REVID               0x00000004
#define REG_OTPID               0x00000010
#define REG_TEST_KEY            0x00000040
#define REG_OTP_MEM0            0x00000400
#define REG_PWR_CTRL1           0x00002014      /* bit 0 GLOBAL_EN */
#define REG_PWR_CTRL2           0x00002018      /* bit 0 AMP_EN, [5:4] BST_EN */
#define REG_PUP_PDN             0x00002084      /* the power-up / power-down patch's register */
#define REG_PLL_CLK_CTRL        0x00002c04      /* [2:0] source (0 SCLK), bit 4 enable, [10:5] REFCLK_FREQ, bit 11 open loop */
#define REG_GLOBAL_CLK_CTRL     0x00002c0c      /* [4:0] GLOBAL_FS: 0x03 48 kHz */
#define REG_TST_FS_MON0         0x00002d10
#define REG_BST_PEAK_CUR        0x00003808
#define REG_BST_COEFF           0x00003810
#define REG_BST_SLOPE_LBST      0x00003814
#define REG_BST_DCM_CTRL        0x0000381c
#define REG_VIMON_SPKMON_RESYNC 0x00004100
#define REG_VPVBST_FS_SEL       0x00004400
#define REG_SP_ENABLES          0x00004800      /* bit 16 ASP_RX1_EN */
#define REG_SP_FORMAT           0x00004808
#define REG_SP_HIZ_CTRL         0x0000480c
#define REG_SP_FRAME_RX_SLOT    0x00004820
#define REG_SP_TX_WL            0x00004830
#define REG_SP_RX_WL            0x00004840
#define REG_DAC_PCM1_SRC        0x00004c00      /* 0x08 ASPRX1 */
#define REG_ASP_TX3_SRC         0x00004c28
#define REG_ASP_TX4_SRC         0x00004c2c
#define REG_AMP_DIG_VOL_CTRL    0x00006000      /* [13:3] PCM volume: 0 = 0 dB, 0.125 dB steps, 11-bit two's complement */
#define REG_AMP_GAIN_CTRL       0x00006c04      /* [9:5] PCM gain */
#define REG_IRQ1_STATUS1        0x00010010
#define REG_IRQ1_STATUS3        0x00010018
#define REG_IRQ1_STATUS4        0x0001001c
#define REG_DSP1_CCM_CORE_CTRL  0x02bc1000
#define REG_DSP1_XM_ACCEL_PL0   0x02bc2020
#define REG_DSP1_YM_ACCEL_PL0   0x02bc20e0

#define PUP_DONE                0x01000000
#define PDN_DONE                0x00800000
#define IRQ1_ERRORS             (0x80000000u | 0x00020000u | 0x00000040u | 0x00000080u | 0x00000100u)   /* short, temp, boost */
#define IRQ1_TEMP_WARN          0x00008000u     /* CS35L41_TEMP_WARN (include/sound/cs35l41.h): not a fault, a level cut (monitor) */
#define READ_FAILURES_MOST      3               /* IRQ1_STATUS1 unreadable so many looks in a row: unhealthy */
#define TEMP_WARN_CUT           (6 * 8)         /* eighths of a dB the guard takes off once TEMP_WARN has been seen */
#define VOL_MUTE                0x4cfu
#define GAIN                    12              /* of Android's 18 */

struct otp_element {
	uint32_t reg;
	uint8_t shift, size;
};

static const struct otp_element otp_map_1[] = {    /* cs35l41-lib.c, 99 entries: address, shift, size */
	{ 0x2030, 0, 4 }, { 0x2030, 7, 1 }, { 0x208c, 24, 6 }, { 0x2090, 14, 4 }, { 0x2090, 10, 4 }, { 0x300c, 11, 4 },
	{ 0x394c, 23, 2 }, { 0x3950, 0, 7 }, { 0x3950, 8, 7 }, { 0x3950, 16, 8 }, { 0x3950, 24, 8 }, { 0x3954, 0, 7 },
	{ 0x3954, 8, 7 }, { 0x3954, 16, 8 }, { 0x3954, 24, 8 }, { 0x3958, 0, 7 }, { 0x3958, 8, 7 }, { 0x3958, 16, 8 },
	{ 0x3958, 24, 8 }, { 0x395c, 0, 7 }, { 0x395c, 8, 7 }, { 0x395c, 16, 8 }, { 0x395c, 24, 8 }, { 0x416c, 0, 8 },
	{ 0x4160, 0, 7 }, { 0x416c, 8, 8 }, { 0x4160, 16, 10 }, { 0x416c, 16, 12 }, { 0x416c, 28, 1 }, { 0x4170, 0, 6 },
	{ 0x4170, 6, 1 }, { 0x4170, 8, 6 }, { 0x4170, 14, 1 }, { 0x4170, 16, 9 }, { 0x4360, 0, 5 }, { 0x4360, 6, 9 },
	{ 0x4448, 0, 8 }, { 0x4448, 8, 8 }, { 0x4448, 16, 8 }, { 0x4448, 24, 8 }, { 0x444c, 0, 3 }, { 0x6e30, 0, 5 },
	{ 0x6e30, 8, 5 }, { 0x6e30, 16, 5 }, { 0x6e30, 24, 5 }, { 0x6e34, 0, 5 }, { 0x6e34, 8, 5 }, { 0x6e34, 16, 5 },
	{ 0x6e34, 24, 5 }, { 0x6e38, 0, 5 }, { 0x6e38, 8, 5 }, { 0x6e38, 16, 5 }, { 0x6e38, 24, 5 }, { 0x6e3c, 0, 5 },
	{ 0x6e3c, 8, 5 }, { 0x6e3c, 16, 5 }, { 0x6e3c, 24, 5 }, { 0x6e40, 0, 5 }, { 0x6e40, 8, 5 }, { 0x6e40, 16, 5 },
	{ 0x6e40, 24, 5 }, { 0x6e44, 0, 5 }, { 0x6e48, 0, 10 }, { 0x6e48, 10, 10 }, { 0x6e48, 20, 10 }, { 0x6e4c, 0, 10 },
	{ 0x6e4c, 10, 10 }, { 0x6e4c, 20, 10 }, { 0x6e50, 0, 10 }, { 0x6e50, 10, 10 }, { 0x6e50, 20, 10 },
	{ 0x6e54, 0, 10 }, { 0x6e54, 10, 10 }, { 0x6e54, 20, 10 }, { 0x6e58, 0, 10 }, { 0x6e58, 10, 10 },
	{ 0x6e58, 20, 10 }, { 0x6e5c, 0, 10 }, { 0x6e5c, 10, 10 }, { 0x6e5c, 20, 10 }, { 0x6e60, 0, 10 },
	{ 0x6e60, 10, 10 }, { 0x6e60, 20, 10 }, { 0x6e64, 0, 10 }, { 0x7418, 7, 5 }, { 0x741c, 0, 5 }, { 0x741c, 11, 4 },
	{ 0x741c, 19, 4 }, { 0x7434, 17, 1 }, { 0x7434, 18, 7 }, { 0x7068, 0, 9 }, { 0x410c, 7, 1 }, { 0x400c, 0, 7 },
	{ 0x0, 0, 1 }, { 0x17040, 0, 8 }, { 0x17040, 8, 8 }, { 0x17040, 16, 8 }, { 0x17040, 24, 8 }, { 0x17044, 0, 24 },
};
static const struct otp_element otp_map_2[] = {    /* cs35l41-lib.c, 99 entries: address, shift, size */
	{ 0x2030, 0, 4 }, { 0x2030, 7, 1 }, { 0x208c, 24, 6 }, { 0x2090, 14, 4 }, { 0x2090, 10, 4 }, { 0x300c, 11, 4 },
	{ 0x394c, 23, 2 }, { 0x3950, 0, 7 }, { 0x3950, 8, 7 }, { 0x3950, 16, 8 }, { 0x3950, 24, 8 }, { 0x3954, 0, 7 },
	{ 0x3954, 8, 7 }, { 0x3954, 16, 8 }, { 0x3954, 24, 8 }, { 0x3958, 0, 7 }, { 0x3958, 8, 7 }, { 0x3958, 16, 8 },
	{ 0x3958, 24, 8 }, { 0x395c, 0, 7 }, { 0x395c, 8, 7 }, { 0x395c, 16, 8 }, { 0x395c, 24, 8 }, { 0x416c, 0, 8 },
	{ 0x4160, 0, 7 }, { 0x416c, 8, 8 }, { 0x4160, 16, 10 }, { 0x416c, 16, 12 }, { 0x416c, 28, 1 }, { 0x4170, 0, 6 },
	{ 0x4170, 6, 1 }, { 0x4170, 8, 6 }, { 0x4170, 14, 1 }, { 0x4170, 16, 9 }, { 0x4360, 0, 5 }, { 0x4360, 6, 9 },
	{ 0x4448, 0, 8 }, { 0x4448, 8, 8 }, { 0x4448, 16, 8 }, { 0x4448, 24, 8 }, { 0x444c, 0, 3 }, { 0x6e30, 0, 5 },
	{ 0x6e30, 8, 5 }, { 0x6e30, 16, 5 }, { 0x6e30, 24, 5 }, { 0x6e34, 0, 5 }, { 0x6e34, 8, 5 }, { 0x6e34, 16, 5 },
	{ 0x6e34, 24, 5 }, { 0x6e38, 0, 5 }, { 0x6e38, 8, 5 }, { 0x6e38, 16, 5 }, { 0x6e38, 24, 5 }, { 0x6e3c, 0, 5 },
	{ 0x6e3c, 8, 5 }, { 0x6e3c, 16, 5 }, { 0x6e3c, 24, 5 }, { 0x6e40, 0, 5 }, { 0x6e40, 8, 5 }, { 0x6e40, 16, 5 },
	{ 0x6e40, 24, 5 }, { 0x6e44, 0, 5 }, { 0x6e48, 0, 10 }, { 0x6e48, 10, 10 }, { 0x6e48, 20, 10 }, { 0x6e4c, 0, 10 },
	{ 0x6e4c, 10, 10 }, { 0x6e4c, 20, 10 }, { 0x6e50, 0, 10 }, { 0x6e50, 10, 10 }, { 0x6e50, 20, 10 },
	{ 0x6e54, 0, 10 }, { 0x6e54, 10, 10 }, { 0x6e54, 20, 10 }, { 0x6e58, 0, 10 }, { 0x6e58, 10, 10 },
	{ 0x6e58, 20, 10 }, { 0x6e5c, 0, 10 }, { 0x6e5c, 10, 10 }, { 0x6e5c, 20, 10 }, { 0x6e60, 0, 10 },
	{ 0x6e60, 10, 10 }, { 0x6e60, 20, 10 }, { 0x6e64, 0, 10 }, { 0x7418, 7, 5 }, { 0x741c, 0, 5 }, { 0x741c, 11, 4 },
	{ 0x741c, 19, 4 }, { 0x7434, 17, 1 }, { 0x7434, 18, 7 }, { 0x7068, 0, 9 }, { 0x410c, 7, 1 }, { 0x400c, 0, 7 },
	{ 0x4000, 11, 1 }, { 0x17040, 0, 8 }, { 0x17040, 8, 8 }, { 0x17040, 16, 8 }, { 0x17040, 24, 8 }, { 0x17044, 0, 24 },
};

static const struct {
	uint8_t address;
	uint8_t slot;           /* its channel's slot in the frame */
	const char *name;
	uint32_t cal_r;         /* the speaker's own measured impedance: Android's CSPL CAL_R while playing (amps-playing-table.txt) */
	uint8_t rx2_slot;       /* SP_FRAME_RX_SLOT's RX2 slot on Android: 0x0401 (slot 4) and 0x0200 (slot 2) */
	bool rx2_asprx2;        /* Android: only the earpiece's DSP RX2 comes from ASPRX2 (with ASP_RX2_EN) and its ASP TX1 carries DSPTX1;
	                         * the speaker's DSP RX2 is ASPRX1 and its ASP TX2 carries DSPTX1 */
} amps[2] = { { 0x40, 1, "the speaker (0x40)", 0x25cd, 4, false }, { 0x42, 0, "the earpiece (0x42)", 0x2039, 2, true } };

static bool amp_used[2], amp_ready[2], amp_on[2];
static bool prepared;
static uint32_t read_failures[2];       /* IRQ1_STATUS1 reads that failed in a row (apollo_cs35l41_healthy()) */
static bool temp_warned;                /* TEMP_WARN seen at some point of the run: the level stays cut for the rest of it */
/*
 * The public entry points that use I2C hold this: the sound's watcher may be in the middle of a sequence (a fade, a
 * power-down) when the run's end stops the amplifiers from another thread. Not apollo_cs35l41_quiesce(): a panic's
 * way out takes no lock (it only drops the reset line).
 */
static LCK_GRP_DECLARE(amps_lock_group, "apollo-cs35l41");
static LCK_MTX_DECLARE(amps_lock, &amps_lock_group);
static void amps_stop(void);
/* the digital volume last written, eighths of a dB under 0 dB; 8 * 102: muted. Muted again whenever they power down
 * or up: a start after a stop must write its level, not find it "already set" (that left them muted, heard on the phone) */
static uint32_t volume_at = 8 * 102;

static bool
test_key(uint8_t amp, bool unlock)
{
	return unlock ? reg_write(amp, REG_TEST_KEY, 0x55) && reg_write(amp, REG_TEST_KEY, 0xaa) :
	       reg_write(amp, REG_TEST_KEY, 0xcc) && reg_write(amp, REG_TEST_KEY, 0x33);
}

/* cs35l41_otp_unpack(): the factory trims, from the OTP's 32 words into their registers (under the test key) */
static bool
otp_unpack(uint8_t amp)
{
	const struct otp_element *map;
	uint32_t id, mem[32], entries, bit = 16, word = 2;

	if (!reg_read(amp, REG_OTPID, &id)) {
		return false;
	}
	if (id == 0x01 || id == 0x08) {
		map = otp_map_1, entries = sizeof(otp_map_1) / sizeof(otp_map_1[0]);
	} else if (id == 0x02 || id == 0x03 || id == 0x06) {
		map = otp_map_2, entries = sizeof(otp_map_2) / sizeof(otp_map_2[0]);
	} else {
		say("apollo cs35l41: %#x: no OTP map for id %#x\n", amp, id);
		return false;
	}
	for (uint32_t i = 0; i < 32; i++) {
		if (!reg_read(amp, REG_OTP_MEM0 + 4 * i, &mem[i])) {
			return false;
		}
	}
	for (uint32_t i = 0; i < entries; i++) {
		uint32_t value = 0, size = map[i].size;

		if (size == 0) {
			continue;
		}
		if (bit + size > 32) {
			value = mem[word] >> bit;
			word++;
			value |= (mem[word] & ((1u << (bit + size - 32)) - 1)) << (32 - bit);
			bit = bit + size - 32;
		} else {
			value = (mem[word] >> bit) & (size == 32 ? ~0u : (1u << size) - 1);
			bit += size;
		}
		if (bit == 32) {
			bit = 0;
			word++;
		}
		if (word >= 32) {
			break;
		}
		if (map[i].reg != 0) {
			uint32_t mask = (size == 32 ? ~0u : ((1u << size) - 1)) << map[i].shift;

			if (!reg_update(amp, map[i].reg, mask, value << map[i].shift)) {
				return false;
			}
		}
	}
	say("apollo cs35l41: %#x: OTP id %#x, %u trims unpacked\n", amp, id, entries);
	return true;
}

/* One amplifier, after the reset: found, trimmed, set up for the frame, powered down and muted */
static bool
amp_setup(unsigned a)
{
	uint8_t amp = amps[a].address;
	uint32_t id = 0, rev = 0, status = 0;
	int i;

	for (i = 0; i < 100; i++) {
		if (reg_read(amp, REG_IRQ1_STATUS4, &status) && (status & 0x2)) {
			break;
		}
		sleep_ms(1);
	}
	if (i == 100) {
		say("apollo cs35l41: %s: the OTP boot is not done (%#x)\n", amps[a].name, status);
		return false;
	}
	if (!reg_read(amp, REG_IRQ1_STATUS3, &status) || (status & 0x80000000u)) {
		say("apollo cs35l41: %s: the OTP boot failed (%#x)\n", amps[a].name, status);
		return false;
	}
	if (!reg_read(amp, REG_DEVID, &id) || !reg_read(amp, REG_REVID, &rev) || (id != 0x35a40 && id != 0x35b40)) {
		say("apollo cs35l41: %s: not a CS35L41 (id %#x, revision %#x)\n", amps[a].name, id, rev);
		return false;
	}
	/* the errata (revision B0 and B2 alike), the DSP core stopped, the trims: under the test key */
	if (!test_key(amp, true) ||
	    !reg_write(amp, REG_VIMON_SPKMON_RESYNC, 0) || !reg_write(amp, 0x00004310, 0) ||
	    !reg_write(amp, REG_VPVBST_FS_SEL, 0) || !reg_write(amp, REG_BST_DCM_CTRL, 0x51) ||
	    !reg_write(amp, REG_DSP1_YM_ACCEL_PL0, 0) || !reg_write(amp, REG_DSP1_XM_ACCEL_PL0, 0) ||
	    !reg_write(amp, REG_PWR_CTRL2, 0) || !reg_write(amp, REG_AMP_GAIN_CTRL, 0) ||
	    !reg_write(amp, REG_ASP_TX3_SRC, 0) || !reg_write(amp, REG_ASP_TX4_SRC, 0) ||
	    !reg_write(amp, REG_DSP1_CCM_CORE_CTRL, 0) || !otp_unpack(amp) ||
	    /* the frame monitor for a bit clock over 6.144 MHz (cs35l41_dai_set_sysclk) */
	    !reg_write(amp, REG_TST_FS_MON0, 0x10 | 0x24u << 12) || !test_key(amp, false)) {
		say("apollo cs35l41: %s: the errata and trims could not be written\n", amps[a].name);
		return false;
	}
	/* boost (the tree's 1 uH, 15 uF, 4 A: Android's values), the serial port, the clock, the DAC's source, muted */
	if (!reg_update(amp, REG_BST_COEFF, 0xffff, 0x2424) || !reg_update(amp, REG_BST_SLOPE_LBST, 0xff03, 0x7500) ||
	    !reg_update(amp, REG_BST_PEAK_CUR, 0x7f, 0x40) || !reg_update(amp, REG_PWR_CTRL2, 0x30, 0x20) ||
	    !reg_update(amp, REG_SP_HIZ_CTRL, 0x3, 0x3) ||
	    !reg_write(amp, REG_SP_FORMAT, 0x20200000) ||              /* 32-bit slots both ways, DSP_A, clocks from the SoC */
	    !reg_update(amp, REG_SP_RX_WL, 0x3f, 24) || !reg_update(amp, REG_SP_TX_WL, 0x3f, 24) ||
	    !reg_update(amp, REG_SP_FRAME_RX_SLOT, 0x3f, amps[a].slot) ||
	    !reg_update(amp, REG_GLOBAL_CLK_CTRL, 0x1f, 0x03) ||
	    /* the PLL from the bit clock, 12.288 MHz (cs35l41_set_sysclk's order) */
	    !reg_update(amp, REG_PLL_CLK_CTRL, 1u << 11, 1u << 11) ||
	    !reg_update(amp, REG_PLL_CLK_CTRL, 0x3f << 5, 0x33u << 5) ||
	    !reg_update(amp, REG_PLL_CLK_CTRL, 1u << 4, 0) ||
	    !reg_update(amp, REG_PLL_CLK_CTRL, 0x7, 0) ||
	    !reg_update(amp, REG_PLL_CLK_CTRL, 1u << 11, 0) ||
	    !reg_update(amp, REG_PLL_CLK_CTRL, 1u << 4, 1u << 4) ||
	    !reg_update(amp, REG_DAC_PCM1_SRC, 0x7f, 0x08) ||
	    !reg_update(amp, REG_AMP_DIG_VOL_CTRL, 0x7ffu << 3, VOL_MUTE << 3) ||
	    !reg_update(amp, REG_AMP_GAIN_CTRL, 0x1f << 5, GAIN << 5)) {
		say("apollo cs35l41: %s: could not be set up\n", amps[a].name);
		return false;
	}
	say("apollo cs35l41: %s: CS35L41 %#x revision %#x, set up: TDM slot %u of 8, 24 bits at 48 kHz, the PLL from the bit "
	    "clock, gain %u, muted, powered down\n", amps[a].name, id, rev, amps[a].slot, GAIN);
	return true;
}

/* ---- the speaker protection firmware (boot-arg apollo_amps_dsp=N) --------------------------------------------------
 *
 * Android runs cs35l41-dsp1-spk-prot-revb2.wmfw (the HALO core's program and its initial data) and a tuning file per
 * amplifier (.bin: -revb2.bin for the speaker, -RCV-revb2.bin for the earpiece) through Linux's cs_dsp
 * (drivers/firmware/cirrus/cs_dsp.c: cs_dsp_load, cs_dsp_halo_setup_algs, cs_dsp_load_coeff, cs_dsp_run). Done here the
 * same way; verified against Android's load log (.device-local/apollo/android-wifi/dmesg-boot.txt, "DSP1:" lines of 1-0040):
 * every block's address, the algorithm's "ID cd v29.33.0 XM@94 YM@e", "Writing 4044 bytes at 2c0015c", "Writing 8 bytes at 34016e0".
 *
 *   apollo_amps_dsp=0 (default): nothing of this runs.
 *   =1 (stages 1 and 2): the three files are parsed and written (the core stopped), then read back and compared;
 *   =2 (stage 3): the MPU unlocked, the rate slots set, the monitor inputs set, the core started at boot WITHOUT requiring
 *      its heartbeat (the DSP runs from the PLL, which needs the bit clock: none at boot, run 221026). At each playback start
 *      (bit clock running, amplifier not yet powered up) the heartbeat is checked, the core restarted once if it does not count;
 *      after power-up the mailbox gets RESUME (PAUSE at stop). Healthy or not, the audio stays PCM (DAC_PCM1_SRC = ASPRX1).
 *   =3 (stage 4): as 2, and at the playback start the DAC is fed from the DSP (DAC_PCM1_SRC = DSPTX1, DSP RX1/RX2 = ASPRX1,
 *      DRE on); at stop the DAC goes back to ASPRX1. Only at the rate the tuning was made for (Android's 96 kHz: AMP_RATE_HZ
 *      must equal DSP_TUNING_RATE_HZ) and only for an amplifier whose calibration took; otherwise it runs as 2 (said in the log):
 *      the protection algorithm fed at the wrong rate or with the wrong impedance would protect nothing it claims to.
 * On any failure (loading, no heartbeat at playback, no RESUME) the cores are stopped and both amplifiers are reset and set up
 * again as before this existed.
 *
 * Memory windows (cs35l41-lib.c cs35l41_dsp1_regions, include/sound/cs35l41.h): XM packed 0x02000000, XM unpacked (24-bit)
 * 0x02800000, YM packed 0x02c00000, YM unpacked 0x03400000, PM packed 0x03800000. cs_dsp_halo_region_to_reg(): packed XM/YM
 * at (base + 3 * word) & ~3, PM at base + 5 * word, unpacked at base + 4 * word. The registers' format as everywhere here: a
 * 32-bit address, then data, the address counting up by the byte (regmap_raw_write), big-endian words.
 */
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

/*
 * Data bytes per I2C transaction (plus the 4 of the address): ~6 ms at 400 kHz, under i2c_xfer()'s 20 ms. A multiple of 12:
 * the packed memories take three 32-bit registers for four 24-bit DSP words (cs_dsp.c: cs_dsp_halo_region_to_reg, packed
 * = base + 3 * word), and with 256 the second transaction of the first big XM block -- which began inside such a group --
 * read back as zeros while the first was right (25 October). 240 = 20 whole groups of XM/YM, 12 of PM.
 */
#define DSP_CHUNK               240             /* and of 20: packed PM takes five registers for four 40-bit words (cs_dsp.c:
	                                         * base + 5 * word); 252 broke the PM block at +2048 (25 October) */
#define DSP_VERIFY_FULL         8192u           /* blocks up to this size are read back whole, larger ones in windows */
#define DSP_VERIFY_STRIDE       2040u           /* a multiple of 60, as DSP_CHUNK: windows start on whole packed groups */
#define DSP_BASE                0x02b80000u     /* cs35l41_configure_cs_dsp: dsp->base = CS35L41_DSP1_CTRL_BASE */
#define DSP_CORE_SOFT_RESET     (DSP_BASE + 0x00010)
#define DSP_CORE_CONTROL        (DSP_BASE + 0x41000)    /* HALO_CCM_CORE_CONTROL (= REG_DSP1_CCM_CORE_CTRL) */
#define DSP_CORE_RESET          0x00000200u
#define DSP_CORE_EN             0x00000001u
#define DSP_MPU_LOCK_CONFIG     (DSP_BASE + 0x43140)
#define DSP_MPU_ACCESS          (DSP_BASE + 0x43000)    /* four groups of 0x18: XMEM, YMEM, WINDOW, XREG, (gap), YREG */
#define DSP_RX1_RATE            (DSP_BASE + 0x00080)    /* eight, 8 apart; TX1_RATE at +0x280 */
#define DSP_TX1_RATE            (DSP_BASE + 0x00280)
#define DSP_MBOX_STATUS         0x00013004u     /* CS35L41_DSP_MBOX_2: CSPL status, 0 RUNNING, 1 PAUSED (cs35l41.h) */
#define DSP_MBOX_CMD            0x00013020u     /* CS35L41_DSP_VIRT1_MBOX_1: where cs35l41_set_cspl_mbox_cmd() writes the command */
#define MBOX_CMD_PAUSE          1u              /* enum cs35l41_cspl_mbox_cmd (cs35l41.h): PAUSE 1 (status PAUSED 1), */
#define MBOX_CMD_RESUME         2u              /* RESUME 2 (status RUNNING 0), SPK_OUT_ENABLE 7 (external boost only) */
#define REG_DSP1_RX1_SRC        0x00004c40u     /* then RX2 .. RX8, 4 apart */
#define REG_IRQ1_RAW_STATUS3    0x00010098      /* the live PLL state (IRQ1_STATUS3 latches events: its bit 29 is not the lock) */
#define REG_IRQ1_STATUS3_PLL    0x00000002u     /* CS35L41_PLL_LOCK in IRQ1_STATUS3 */
#define SRC_ASPRX1              0x08u
#define SRC_DSP_TX1             0x32u           /* CS35L41_INPUT_DSP_TX1: DAC_PCM1_SRC "DSP" is 0x32 (cs35l41_pcm_source_values) */
#define SRC_VPMON               0x28u           /* cs35l41_dsp_init: DSP1RX5 VPMON (internal boost), RX6 VBSTMON, RX7 TEMPMON, RX8 RSVD */
#define SRC_VBSTMON             0x29u
#define SRC_TEMPMON             0x3au
#define SRC_RSVD                0x3bu
#define PWR_CTRL3_DRE           (1u << 20)      /* "DRE Switch": SOC_DAPM_SINGLE(PWR_CTRL3, 20); Android's PWR_CTRL3 0x01101000 has it set */
#define REG_PWR_CTRL3           0x0000201c
/* What Android leaves while music plays (.device-local/apollo/audio/amps-playing-table.txt), applied at apollo_amps_dsp >= 3.
 * Bit names from include/sound/cs35l41.h and cs35l41.c's DAPM widgets. */
#define REG_CTRL_OVRRIDE        0x00002020u     /* CS35L41_CTRL_OVRRIDE: Android 0x2 (no field names upstream) */
#define PWR_CTRL2_MONITORS      0x00003700u     /* VPMON bit 8, VBSTMON 9, TEMPMON 10, VMON 12, IMON 13 (the "VMON".."TEMPMON" supplies);
	                                         * Android's 0x3721 = these + BST_EN 2 (bits 5:4, set at setup) + AMP_EN (bit 0, set at power-up) */
#define PWR_CTRL3_ANDROID       0x01101010u     /* bit 4 CLASSH_EN ("CLASS H" ADC), bit 20 DRE, bit 24 (default 0x01000010 has it), bit 12 (not
	                                         * named upstream) */
#define REG_BSTCVRT_VCTRL1      0x00003800u     /* Android 0 */
#define REG_BSTCVRT_VCTRL2      0x00003804u     /* BST_CTL_SEL: 0 register (VCTRL1), 1 Class-H tracking, 2 MDSYNC (cs35l41.h CS35L41_BST_CTL_SEL_*); Android 1 */
#define REG_BSTCVRT_SFT_RAMP    0x0000380cu     /* Android 3 */
#define REG_BSTCVRT_SW_FREQ     0x00003818u     /* Android 0x01008000 */
#define REG_BSTCVRT_DCM_FORCE   0x00003820u     /* Android 0 */
#define REG_ASP_TX1_SRC         0x00004c20u
#define REG_ASP_TX2_SRC         0x00004c24u
#define SP_EN_RX2               (1u << 17)      /* ASP_RX2_EN */
#define SP_EN_TX1               (1u << 0)
#define SP_EN_TX2               (1u << 1)
#define SRC_ASPRX2              0x09u
#define SRC_VMON                0x18u
#define SRC_IMON                0x19u
#define SRC_CLASSH              0x21u           /* Android's DSP RX6 (Linux's cs35l41_dsp_init writes VBSTMON there) */
#define AMP_DIG_VOL_RAMP_MASK   0x7u            /* AMP_DIG_VOL_CTRL [2:0] volume ramp; Android 4 */
#define AMP_DIG_VOL_RAMP        4u
#define CAL_R_ADDR              0x0280026cu     /* CSPL "CAL_R", "CAL_STATUS", "CAL_CHECKSUM" (XM unpacked, as Android's dump has them) */
#define CAL_STATUS_ADDR         0x02800274u
#define CAL_CHECKSUM_ADDR       0x02800278u
#define DSP_XM_UNPACKED         0x02800000u
#define DSP_YM_UNPACKED         0x03400000u
#define DSP_XM_PACKED           0x02000000u
#define DSP_YM_PACKED           0x02c00000u
#define DSP_PM_PACKED           0x03800000u
#define AMP_RATE_HZ             48000u          /* what this runs the amplifiers at (GLOBAL_FS 3) */
#define DSP_TUNING_RATE_HZ      96000u          /* what the .bin tuning is for: Android's rate (GLOBAL_FS 4) */
#define FW_ID_HALO_CSPL         0x400a4u        /* the firmware's own id (XM header) */
#define ALG_ID_CSPL             0xcdu           /* the speaker protection algorithm */
#define ALG_CSPL_XM_BASE        0x94u           /* what Android's log shows for it: "XM@94 YM@e" */
#define ALG_CSPL_YM_BASE        0x0eu
#define DSP_MAX_ALGS            4

#define WMFW_PM_PACKED          0x10u
#define WMFW_XM_PACKED          0x11u
#define WMFW_YM_PACKED          0x12u
#define WMFW_XM_UNPACKED        0x05u           /* WMFW_ADSP2_XM */
#define WMFW_YM_UNPACKED        0x06u           /* WMFW_ADSP2_YM */
#define WMFW_ALGORITHM_DATA     0xf2u
#define WMFW_INFO_TEXT          0xffu
#define WMFW_NAME_TEXT          0xfeu

struct dsp_info {                               /* the XM header the firmware leaves in the DSP (wmfw_halo_id_hdr) */
	uint32_t fw_id, fw_ver, vendor, xm_base, ym_base, n_algs;
	struct {
		uint32_t id, ver, xm_base, ym_base;
	} alg[DSP_MAX_ALGS];
};

static uint32_t dsp_level;                      /* apollo_amps_dsp */
static struct dsp_info dsp_info[2];
/* The firmware's controls (from the .wmfw's algorithm data), kept for the playback start; valid = loaded and the core started */
static struct {
	bool valid, routed, feedback, calibrated;
	uint32_t cal_r, cal_status, cal_checksum;
	uint32_t halo_state, heartbeat, enable, command, state, errorno, temperature;
} dsp_ctl[2];

static inline uint32_t
le32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static inline uint32_t
le16(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8;
}

static inline uint32_t
be32(const uint8_t *p)
{
	return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

static uint64_t
now_ms(void)
{
	uint64_t abs = mach_absolute_time(), ns;

	absolutetime_to_nanoseconds(abs, &ns);
	return ns / 1000000;
}

/* `len` bytes at the register `reg`, in transactions of up to DSP_CHUNK bytes (the address counts up by itself) */
static bool
dsp_write(uint8_t amp, uint32_t reg, const uint8_t *data, uint32_t len)
{
	uint8_t b[4 + DSP_CHUNK];

	if ((reg & 3) != 0 || (len & 3) != 0) {
		say("apollo cs35l41: %#x: the write of %u bytes at %#x is not word aligned\n", amp, len, reg);
		return false;
	}
	while (len > 0) {
		uint32_t n = len < DSP_CHUNK ? len : DSP_CHUNK;

		b[0] = (uint8_t)(reg >> 24), b[1] = (uint8_t)(reg >> 16), b[2] = (uint8_t)(reg >> 8), b[3] = (uint8_t)reg;
		memcpy(b + 4, data, n);
		if (!i2c_xfer(amp, b, NULL, 4 + n, true)) {
			say("apollo cs35l41: %#x: the write of %u bytes at %#x failed\n", amp, n, reg);
			return false;
		}
		reg += n, data += n, len -= n;
	}
	return true;
}

static bool
dsp_read(uint8_t amp, uint32_t reg, uint8_t *out, uint32_t len)
{
	while (len > 0) {
		uint32_t n = len < DSP_CHUNK ? len : DSP_CHUNK;
		uint8_t a[4] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg };

		if (!i2c_xfer(amp, a, NULL, 4, false) || !i2c_xfer(amp, NULL, out, n, true)) {
			return false;
		}
		reg += n, out += n, len -= n;
	}
	return true;
}

static uint32_t dsp_bytes_checked;

static bool
dsp_verify_window(uint8_t amp, uint32_t reg, const uint8_t *data, uint32_t len, uint32_t off, const char *what)
{
	uint8_t back[DSP_CHUNK];
	uint32_t n = len - off < DSP_CHUNK ? len - off : DSP_CHUNK, i = 0;

	if (!dsp_read(amp, reg + off, back, n)) {
		say("apollo cs35l41: %#x: %s: the readback of %u bytes at %#x failed\n", amp, what, n, reg + off);
		return false;
	}
	if (memcmp(back, data + off, n) != 0) {
		while (back[i] == data[off + i]) {
			i++;
		}
		uint32_t bad = 0, w0 = 0, wl = 0, k;
		uint8_t again[DSP_CHUNK];
		bool ok;

		for (k = 0; k < n; k++) {
			bad += back[k] != data[off + k];
		}
		say("apollo cs35l41: %#x: %s: READBACK MISMATCH at %#x (+%u of the block's %u): wrote %02x, read %02x (%u of the window's %u bytes differ, I2C failures so far %u)\n",
		    amp, what, reg + off + i, off + i, len, data[off + i], back[i], bad, n, i2c_failures);
		/* diagnosis: re-read the window's first and last word, then write it again one word per transaction */
		ok = reg_read(amp, reg + off, &w0) && reg_read(amp, reg + off + n - 4, &wl);
		say("apollo cs35l41: %#x: re-read: first word %#x (wanted %#x), last word %#x (wanted %#x)%s\n", amp, w0, be32(data + off), wl,
		    be32(data + off + n - 4), ok ? "" : " (a read failed)");
		for (k = 0; k < n; k += 4) {
			uint8_t one[8] = { (uint8_t)((reg + off + k) >> 24), (uint8_t)((reg + off + k) >> 16), (uint8_t)((reg + off + k) >> 8),
				           (uint8_t)(reg + off + k), data[off + k], data[off + k + 1], data[off + k + 2], data[off + k + 3] };

			if (!i2c_xfer(amp, one, NULL, 8, true)) {
				say("apollo cs35l41: %#x: the word-by-word rewrite failed at %#x\n", amp, reg + off + k);
				return false;
			}
		}
		if (!dsp_read(amp, reg + off, again, n)) {
			say("apollo cs35l41: %#x: the readback after the word-by-word rewrite failed\n", amp);
			return false;
		}
		if (memcmp(again, data + off, n) != 0) {
			for (k = 0; again[k] == data[off + k]; k++) {
			}
			say("apollo cs35l41: %#x: still wrong after word-by-word writes (+%u: read %02x): the region is not writable\n", amp,
			    off + k, again[k]);
			return false;
		}
		say("apollo cs35l41: %#x: word-by-word writes of the window took: the burst write failed, not the region (continuing)\n", amp);
	}
	dsp_bytes_checked += n;
	return true;
}

/* Read back what was just written and compare it: all of a block up to DSP_VERIFY_FULL, of a larger one a window of
 * DSP_CHUNK bytes every DSP_VERIFY_STRIDE and the last window */
static bool
dsp_verify(uint8_t amp, uint32_t reg, const uint8_t *data, uint32_t len, const char *what)
{
	uint32_t stride = len <= DSP_VERIFY_FULL ? DSP_CHUNK : DSP_VERIFY_STRIDE, off;

	for (off = 0; off < len; off += stride) {
		if (!dsp_verify_window(amp, reg, data, len, off, what)) {
			return false;
		}
	}
	if (len > DSP_CHUNK && (off - stride) + DSP_CHUNK < len) {                  /* the tail: the last window ends at the end */
		/*
		 * ...from a start on a whole packed group (60 bytes: 12 for XM/YM, 20 for PM): reads that begin inside a group
		 * came back as zeros where the writes were right (the PM block at +2048, 25 October)
		 */
		return dsp_verify_window(amp, reg, data, len, (len - DSP_CHUNK) / 60 * 60, what);
	}
	return true;
}

/* The register of word `words` of a memory type of the file formats (cs_dsp_halo_region_to_reg) */
static bool
dsp_region_reg(uint32_t type, uint32_t words, uint32_t *reg)
{
	switch (type) {
	case WMFW_XM_PACKED:
		*reg = (DSP_XM_PACKED + words * 3) & ~3u;
		return true;
	case WMFW_YM_PACKED:
		*reg = (DSP_YM_PACKED + words * 3) & ~3u;
		return true;
	case WMFW_PM_PACKED:
		*reg = DSP_PM_PACKED + words * 5;
		return true;
	case WMFW_XM_UNPACKED:
		*reg = DSP_XM_UNPACKED + words * 4;
		return true;
	case WMFW_YM_UNPACKED:
		*reg = DSP_YM_UNPACKED + words * 4;
		return true;
	}
	return false;
}

static const char *
dsp_region_name(uint32_t type)
{
	switch (type) {
	case WMFW_XM_PACKED: return "XM_PACKED";
	case WMFW_YM_PACKED: return "YM_PACKED";
	case WMFW_PM_PACKED: return "PM_PACKED";
	case WMFW_XM_UNPACKED: return "XM";
	case WMFW_YM_UNPACKED: return "YM";
	}
	return "?";
}

/* The .wmfw (format 3, HALO): the 40-byte header, then blocks of { type << 24 | offset (words), length, data } with no
 * padding between them (cs_dsp_load). Memory blocks are written; the information text and the algorithm data (the
 * controls' names, read by dsp_control()) are not. */
static bool
wmfw_load(uint8_t amp, const uint8_t *fw, uint64_t size, uint32_t *blocks, uint32_t *bytes)
{
	uint64_t pos;

	if (size < 40 || memcmp(fw, "WMFW", 4) != 0 || fw[11] != 3 || fw[10] != 4) {
		say("apollo cs35l41: the .wmfw is not a format 3 file for a HALO core\n");
		return false;
	}
	pos = le32(fw + 4);                             /* header length: 40 */
	while (pos + 8 <= size) {
		uint32_t word = le32(fw + pos), len = le32(fw + pos + 4), type = word >> 24, offset = word & 0xffffff, reg = 0;

		if (len > size - pos - 8) {
			say("apollo cs35l41: the .wmfw's block at %llu overflows the file\n", pos);
			return false;
		}
		if (type == WMFW_INFO_TEXT || type == WMFW_NAME_TEXT || type == WMFW_ALGORITHM_DATA) {
			pos += 8 + len;
			continue;
		}
		if (!dsp_region_reg(type, offset, &reg)) {
			say("apollo cs35l41: the .wmfw's block at %llu has the unknown type %#x\n", pos, type);
			return false;
		}
		if (!dsp_write(amp, reg, fw + pos + 8, len) || !dsp_verify(amp, reg, fw + pos + 8, len, dsp_region_name(type))) {
			return false;
		}
		(*blocks)++;
		*bytes += len;
		pos += 8 + len;
	}
	return pos == size;
}

/* A string field of the algorithm data: a length of `width` bytes, the characters, padded to 4 (cs_dsp_coeff_parse_string) */
static bool
alg_string(const uint8_t *b, uint32_t length, uint32_t *pos, unsigned width, const uint8_t **text, uint32_t *n)
{
	uint32_t l, total;

	if (length - *pos < 4) {
		return false;
	}
	l = width == 1 ? b[*pos] : le16(b + *pos);
	total = (l + width + 3) & ~3u;
	if (total > length - *pos) {
		return false;
	}
	if (text != NULL) {
		*text = b + *pos + width;
	}
	if (n != NULL) {
		*n = l;
	}
	*pos += total;
	return true;
}

/* The register of the control `name` of algorithm `alg` (an 0xf2 block: id, name, description, count, then per control
 * { u16 offset, u16 memory type, u32 size of the rest } name, description strings, ctl type, flags, length), given where the
 * firmware put the algorithm (`info`). cs_dsp_parse_coeff and cs_dsp_coeff_base_reg: the algorithm's base plus the offset, in
 * the unpacked memory */
static bool
dsp_control(const uint8_t *fw, uint64_t size, const struct dsp_info *info, uint32_t alg, const char *name, uint32_t *reg)
{
	uint64_t pos = le32(fw + 4);
	uint32_t xm = 0, ym = 0;
	bool known = false;

	if (info->fw_id == alg) {
		xm = info->xm_base, ym = info->ym_base, known = true;
	}
	for (uint32_t i = 0; i < info->n_algs && i < DSP_MAX_ALGS; i++) {
		if (info->alg[i].id == alg) {
			xm = info->alg[i].xm_base, ym = info->alg[i].ym_base, known = true;
		}
	}
	if (!known) {
		return false;
	}
	while (pos + 8 <= size) {
		uint32_t word = le32(fw + pos), len = le32(fw + pos + 4), p = 0, count;
		const uint8_t *b = fw + pos + 8;

		if (len > size - pos - 8) {
			return false;
		}
		pos += 8 + len;
		if ((word >> 24) != WMFW_ALGORITHM_DATA || len < 8 || le32(b) != alg) {
			continue;
		}
		p = 4;
		if (!alg_string(b, len, &p, 1, NULL, NULL) || !alg_string(b, len, &p, 2, NULL, NULL) || len - p < 4) {
			return false;
		}
		count = le32(b + p);
		p += 4;
		for (uint32_t i = 0; i < count; i++) {
			uint32_t offset, type, rest, q, n = 0, end;
			const uint8_t *text = NULL;

			if (len - p < 8) {
				return false;
			}
			offset = le16(b + p), type = le16(b + p + 2), rest = le32(b + p + 4);
			if (rest > len - p - 8) {
				return false;
			}
			end = p + 8 + rest;
			q = p + 8;
			if (!alg_string(b, len, &q, 1, &text, &n) || !alg_string(b, len, &q, 1, NULL, NULL) ||
			    !alg_string(b, len, &q, 2, NULL, NULL)) {
				return false;
			}
			if (strlen(name) == n && memcmp(text, name, n) == 0) {
				if (type == WMFW_XM_UNPACKED) {
					*reg = DSP_XM_UNPACKED + (xm + offset) * 4;
				} else if (type == WMFW_YM_UNPACKED) {
					*reg = DSP_YM_UNPACKED + (ym + offset) * 4;
				} else {
					return false;
				}
				return true;
			}
			p = end;
		}
	}
	return false;
}

/* The XM header (cs_dsp_halo_setup_algs): 10 words (core id, block rev, vendor, firmware id, version, XM base and size, YM base
 * and size, algorithm count), then 6 per algorithm (id, version, XM base, XM size, YM base, YM size); read from the DSP's
 * unpacked XM, so it shows that the packed writes landed */
static bool
dsp_header(uint8_t amp, struct dsp_info *info)
{
	uint8_t raw[4 * (10 + 6 * DSP_MAX_ALGS)];
	uint32_t n;

	memset(info, 0, sizeof(*info));
	if (!dsp_read(amp, DSP_XM_UNPACKED, raw, 40)) {
		return false;
	}
	info->vendor = be32(raw + 8), info->fw_id = be32(raw + 12), info->fw_ver = be32(raw + 16);
	info->xm_base = be32(raw + 20), info->ym_base = be32(raw + 28), info->n_algs = n = be32(raw + 36);
	if (n == 0 || n > DSP_MAX_ALGS || !dsp_read(amp, DSP_XM_UNPACKED + 40, raw + 40, 24 * n)) {
		return false;
	}
	for (uint32_t i = 0; i < n; i++) {
		const uint8_t *e = raw + 40 + 24 * i;

		info->alg[i].id = be32(e), info->alg[i].ver = be32(e + 4), info->alg[i].xm_base = be32(e + 8);
		info->alg[i].ym_base = be32(e + 16);
	}
	return true;
}

static bool
dsp_alg_base(const struct dsp_info *info, uint32_t id, uint32_t *xm, uint32_t *ym)
{
	if (id == info->fw_id) {
		*xm = info->xm_base, *ym = info->ym_base;
		return true;
	}
	for (uint32_t i = 0; i < info->n_algs && i < DSP_MAX_ALGS; i++) {
		if (info->alg[i].id == id) {
			*xm = info->alg[i].xm_base, *ym = info->alg[i].ym_base;
			return true;
		}
	}
	return false;
}

/* The .bin (WMDR, cs_dsp_load_coeff): a header of `len` (at +4), then items of { u16 offset, u16 type, u32 algorithm id, u32
 * version, u32 offset32, u32 length, data }, each padded to 4. The data of a memory type goes to the algorithm's base in that
 * memory plus `offset` (bytes): 0x12 YM_PACKED, 0x11 XM_PACKED, 0x06 YM, 0x05 XM. Texts (0xfe00) and metadata (0xfc00) skipped. */
static bool
bin_load(uint8_t amp, const struct dsp_info *info, const uint8_t *bin, uint64_t size, uint32_t *items, uint32_t *bytes)
{
	uint64_t pos;

	if (size < 16 || memcmp(bin, "WMDR", 4) != 0) {
		say("apollo cs35l41: the .bin is not a WMDR file\n");
		return false;
	}
	pos = le32(bin + 4);
	while (pos + 20 <= size) {
		uint32_t offset = le16(bin + pos), type = le16(bin + pos + 2), id = le32(bin + pos + 4), len = le32(bin + pos + 16);
		uint32_t xm, ym, base, reg;

		if (len > size - pos - 20) {
			say("apollo cs35l41: the .bin's item at %llu overflows the file\n", pos);
			return false;
		}
		if (type == 0xfe00 || type == 0xfc00 || type == 0xff00) {
			pos += (20 + (uint64_t)len + 3) & ~3ull;
			continue;
		}
		if ((type != WMFW_XM_PACKED && type != WMFW_YM_PACKED && type != WMFW_XM_UNPACKED && type != WMFW_YM_UNPACKED) ||
		    !dsp_alg_base(info, id, &xm, &ym)) {
			say("apollo cs35l41: the .bin's item at %llu: type %#x for algorithm %#x is not one this knows\n", pos, type, id);
			return false;
		}
		base = (type == WMFW_XM_PACKED || type == WMFW_XM_UNPACKED) ? xm : ym;
		if (!dsp_region_reg(type, base, &reg)) {
			return false;
		}
		reg += offset;
		say("apollo cs35l41: %#x: .bin item: %u bytes in %s for algorithm %#x v%u.%u.%u at %#x\n", amp, len, dsp_region_name(type), id,
		    le32(bin + pos + 8) >> 16 & 0xff, le32(bin + pos + 8) >> 8 & 0xff, le32(bin + pos + 8) & 0xff, reg);
		if (!dsp_write(amp, reg, bin + pos + 20, len) || !dsp_verify(amp, reg, bin + pos + 20, len, "the .bin")) {
			return false;
		}
		(*items)++;
		*bytes += len;
		pos += (20 + (uint64_t)len + 3) & ~3ull;
	}
	return true;
}

static void
dsp_stop_core(uint8_t amp)
{
	reg_update(amp, DSP_CORE_CONTROL, DSP_CORE_EN, 0);              /* cs_dsp_halo_stop_core */
	reg_update(amp, DSP_CORE_SOFT_RESET, 1, 1);
}

/* Stages 1 and 2 for one amplifier: the .wmfw and the .bin written (the core stopped), read back, the header checked */
static bool
dsp_load(unsigned a, const uint8_t *fw, uint64_t fw_size, const uint8_t *bin, uint64_t bin_size)
{
	uint8_t amp = amps[a].address;
	uint32_t blocks = 0, bytes = 0, items = 0, bin_bytes = 0, xm = 0, ym = 0, reg;
	struct dsp_info *info = &dsp_info[a];
	uint64_t start = now_ms(), mid;

	dsp_bytes_checked = 0;
	if (!reg_write(amp, DSP_CORE_CONTROL, 0) || !wmfw_load(amp, fw, fw_size, &blocks, &bytes)) {
		return false;
	}
	mid = now_ms();
	if (!dsp_header(amp, info)) {
		say("apollo cs35l41: %s: the firmware's header cannot be read back from the DSP's XM\n", amps[a].name);
		return false;
	}
	say("apollo cs35l41: %s: firmware %#x vendor %#x v%u.%u.%u, XM@%#x YM@%#x, %u algorithm(s)", amps[a].name, info->fw_id, info->vendor,
	    info->fw_ver >> 16 & 0xff, info->fw_ver >> 8 & 0xff, info->fw_ver & 0xff, info->xm_base, info->ym_base, info->n_algs);
	for (uint32_t i = 0; i < info->n_algs && i < DSP_MAX_ALGS; i++) {
		say(", [%u] ID %x v%u.%u.%u XM@%x YM@%x", i, info->alg[i].id, info->alg[i].ver >> 16 & 0xff, info->alg[i].ver >> 8 & 0xff,
		    info->alg[i].ver & 0xff, info->alg[i].xm_base, info->alg[i].ym_base);
	}
	say("\n");
	if (info->fw_id != FW_ID_HALO_CSPL || !dsp_alg_base(info, ALG_ID_CSPL, &xm, &ym) || xm != ALG_CSPL_XM_BASE || ym != ALG_CSPL_YM_BASE) {
		say("apollo cs35l41: %s: NOT the expected firmware (%#x with the algorithm %#x at XM@%#x YM@%#x)\n", amps[a].name,
		    FW_ID_HALO_CSPL, ALG_ID_CSPL, ALG_CSPL_XM_BASE, ALG_CSPL_YM_BASE);
		return false;
	}
	if (!bin_load(amp, info, bin, bin_size, &items, &bin_bytes)) {
		return false;
	}
	if (!dsp_control(fw, fw_size, info, ALG_ID_CSPL, "CSPL_STATE", &reg) || reg != DSP_XM_UNPACKED + (ALG_CSPL_XM_BASE + 2) * 4) {
		say("apollo cs35l41: %s: the CSPL_STATE control is not where Android reads it (%#x)\n", amps[a].name, DSP_XM_UNPACKED +
		    (ALG_CSPL_XM_BASE + 2) * 4);
		return false;
	}
	say("apollo cs35l41: %s: LOADED: .wmfw %u blocks, %u bytes; .bin %u items, %u bytes; %u bytes read back and equal; %llu ms "
	    "(%llu for the .wmfw)\n", amps[a].name, blocks, bytes, items, bin_bytes, dsp_bytes_checked, now_ms() - start, mid - start);
	return true;
}

/* cs_dsp_halo_start_core */
static bool
dsp_core_start(uint8_t amp)
{
	return reg_update(amp, DSP_CORE_CONTROL, DSP_CORE_RESET | DSP_CORE_EN, DSP_CORE_RESET | DSP_CORE_EN) &&
	       reg_update(amp, DSP_CORE_CONTROL, DSP_CORE_RESET, 0);
}

/* The CSPL mailbox (cs35l41_set_cspl_mbox_cmd): the command to VIRT1_MBOX_1, then MBOX_2 read up to 5 times, 1 ms apart, for the
 * status that command gives (RESUME: RUNNING 0, PAUSE: PAUSED 1; 0xffffffff and 0x00ffffff are errors) */
static bool
dsp_mbox(unsigned a, uint32_t cmd, uint32_t want)
{
	uint8_t amp = amps[a].address;
	uint32_t sts = 0xdead;

	if (!reg_write(amp, DSP_MBOX_CMD, cmd)) {
		return false;
	}
	for (int i = 0; i < 5; i++) {
		sleep_ms(1);
		if (reg_read(amp, DSP_MBOX_STATUS, &sts)) {
			if (sts == 0xffffffffu || sts == 0x00ffffffu) {
				break;
			}
			if (sts == want) {
				return true;
			}
		}
	}
	say("apollo cs35l41: %s: mailbox command %u: status %#x, wanted %#x\n", amps[a].name, cmd, sts, want);
	return false;
}

/* The state of the firmware, for the log */
static void
dsp_report(unsigned a, const char *when)
{
	uint8_t amp = amps[a].address;
	uint32_t hs = 0xdead, beat = 0xdead, en = 0xdead, cmd = 0xdead, st = 0xdead, err = 0xdead, temp = 0xdead, mbox = 0xdead;
	uint32_t pll = 0xdead, pllc = 0xdead, core = 0xdead;

	reg_read(amp, dsp_ctl[a].halo_state, &hs), reg_read(amp, dsp_ctl[a].heartbeat, &beat), reg_read(amp, dsp_ctl[a].enable, &en);
	reg_read(amp, dsp_ctl[a].command, &cmd), reg_read(amp, dsp_ctl[a].state, &st), reg_read(amp, dsp_ctl[a].errorno, &err);
	reg_read(amp, dsp_ctl[a].temperature, &temp), reg_read(amp, DSP_MBOX_STATUS, &mbox);
	reg_read(amp, REG_IRQ1_RAW_STATUS3, &pll), reg_read(amp, REG_PLL_CLK_CTRL, &pllc), reg_read(amp, DSP_CORE_CONTROL, &core);
	say("apollo cs35l41: %s: DSP %s: HALO_STATE %#x HEARTBEAT %#x | CSPL ENABLE %#x COMMAND %#x STATE %#x ERRORNO %#x TEMPERATURE %#x "
	    "| mailbox status %#x | core control %#x PLL_CLK_CTRL %#x raw IRQ1_STATUS3 %#x (PLL %s)\n", amps[a].name, when, hs, beat, en, cmd,
	    st, err, temp, mbox, core, pllc, pll, (pll & REG_IRQ1_STATUS3_PLL) ? "locked" : "NOT locked");
}

/* apollo_amps_dsp >= 3, before GLOBAL_EN: what Android has set while music plays and the protection algorithm needs: the
 * voltage/current/temperature monitors on (PWR_CTRL2), the DSP's eight inputs, Class-H tracking boost, the ASP slots the
 * feedback uses, the ramp. The output is not routed yet (DAC_PCM1_SRC stays ASPRX1 until the firmware is judged). */
static bool
dsp_feedback(unsigned a)
{
	uint8_t amp = amps[a].address;
	/* DSP1 RX1..RX8 as Android: ASPRX1, ASPRX1 (speaker) or ASPRX2 (earpiece), VMON, IMON, VPMON, CLASSH, TEMPMON, RSVD */
	const uint32_t rx[8] = { SRC_ASPRX1, amps[a].rx2_asprx2 ? SRC_ASPRX2 : SRC_ASPRX1, SRC_VMON, SRC_IMON, SRC_VPMON, SRC_CLASSH,
	                         SRC_TEMPMON, SRC_RSVD };

	for (unsigned i = 0; i < 8; i++) {
		if (!reg_write(amp, REG_DSP1_RX1_SRC + 4 * i, rx[i])) {
			return false;
		}
	}
	if (!reg_update(amp, REG_PWR_CTRL2, PWR_CTRL2_MONITORS, PWR_CTRL2_MONITORS) ||
	    !reg_update(amp, REG_PWR_CTRL3, PWR_CTRL3_ANDROID, PWR_CTRL3_ANDROID) || !reg_write(amp, REG_CTRL_OVRRIDE, 2) ||
	    !reg_write(amp, REG_BSTCVRT_VCTRL1, 0) || !reg_write(amp, REG_BSTCVRT_SFT_RAMP, 3) ||
	    !reg_write(amp, REG_BSTCVRT_SW_FREQ, 0x01008000) || !reg_write(amp, REG_BSTCVRT_DCM_FORCE, 0) ||
	    !reg_write(amp, REG_BSTCVRT_VCTRL2, 1) ||
	    !reg_update(amp, REG_AMP_DIG_VOL_CTRL, AMP_DIG_VOL_RAMP_MASK, AMP_DIG_VOL_RAMP)) {
		return false;
	}
	/* the DSP's output also on the ASP TX the way Android has it (earpiece TX1, speaker TX2: SP_FRAME_TX_SLOT 0x03020100) and the
	 * earpiece's second RX slot; SP_ENABLES: speaker 0x00010002, earpiece 0x00030001 (bit 16 is set at power-up) */
	if (!reg_write(amp, amps[a].rx2_asprx2 ? REG_ASP_TX1_SRC : REG_ASP_TX2_SRC, SRC_DSP_TX1) ||
	    !reg_update(amp, REG_SP_FRAME_RX_SLOT, 0x3f00, (uint32_t)amps[a].rx2_slot << 8) ||
	    !reg_update(amp, REG_SP_ENABLES, SP_EN_RX2 | SP_EN_TX1 | SP_EN_TX2,
	                amps[a].rx2_asprx2 ? SP_EN_RX2 | SP_EN_TX1 : SP_EN_TX2)) {
		return false;
	}
	dsp_ctl[a].feedback = true;
	say("apollo cs35l41: %s: Android's playing setup: monitors on (PWR_CTRL2 +%#x), PWR_CTRL3 %#x, DSP RX1..8 = ASPRX1/%s/VMON/IMON/VPMON/CLASSH/"
	    "TEMPMON/RSVD, boost Class-H tracking, ASP TX%u = DSPTX1, ramp %u\n", amps[a].name, PWR_CTRL2_MONITORS, PWR_CTRL3_ANDROID,
	    amps[a].rx2_asprx2 ? "ASPRX2" : "ASPRX1", amps[a].rx2_asprx2 ? 1 : 2, AMP_DIG_VOL_RAMP);
	return true;
}

/* The three calibration words as the DSP memory holds them (24-bit values in 32-bit unpacked XM words), read back */
static bool
dsp_cal_read(unsigned a, uint32_t *r, uint32_t *st, uint32_t *sum)
{
	uint8_t amp = amps[a].address;

	return reg_read(amp, dsp_ctl[a].cal_r, r) && reg_read(amp, dsp_ctl[a].cal_status, st) &&
	       reg_read(amp, dsp_ctl[a].cal_checksum, sum);
}

/* cs_amp_write_cal_coeffs: CAL_R, CAL_STATUS, CAL_CHECKSUM (= CAL_R + CAL_STATUS), each a big-endian 32-bit word at its control's
 * unpacked-XM address (cs_dsp_coeff_write_ctrl -> regmap_raw_write); written with dsp_write, the path the firmware image took */
static bool
dsp_cal_write(unsigned a)
{
	uint8_t amp = amps[a].address, w[4];
	const uint32_t reg[3] = { dsp_ctl[a].cal_r, dsp_ctl[a].cal_status, dsp_ctl[a].cal_checksum };
	const uint32_t val[3] = { amps[a].cal_r, 1, amps[a].cal_r + 1 };

	for (unsigned i = 0; i < 3; i++) {
		w[0] = (uint8_t)(val[i] >> 24), w[1] = (uint8_t)(val[i] >> 16), w[2] = (uint8_t)(val[i] >> 8), w[3] = (uint8_t)val[i];
		if (!dsp_write(amp, reg[i], w, 4)) {
			return false;
		}
	}
	return true;
}

/* The speaker's own measured impedance as Android's CSPL has it (CAL_R, CAL_STATUS 1, CAL_CHECKSUM = CAL_R + CAL_STATUS), read
 * back. Run 232258 read zeros after the plain writes, with the firmware running: so the memory around it is logged first (the words
 * Android's dump has non-zero at 0x2800250..0x2800278 show whether the firmware's own state reads at all), and a failed readback is
 * retried with the firmware PAUSED (a running core may own or rewrite these words), then RESUMEd. */
static bool
dsp_calibrate(unsigned a)
{
	uint8_t amp = amps[a].address, raw[0x30];
	uint32_t r = 0, st = 0, sum = 0;
	bool ok;

	if (dsp_ctl[a].cal_r == 0) {
		return true;
	}
	if (dsp_read(amp, DSP_XM_UNPACKED + 0x250, raw, sizeof raw)) {
		say("apollo cs35l41: %s: XM words 0x250..0x27c before the calibration: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
		    amps[a].name, be32(raw), be32(raw + 4), be32(raw + 8), be32(raw + 12), be32(raw + 16), be32(raw + 20), be32(raw + 24),
		    be32(raw + 28), be32(raw + 32), be32(raw + 36), be32(raw + 40), be32(raw + 44));
	}
	if (!dsp_cal_write(a)) {
		return false;
	}
	sleep_ms(3);
	if (!dsp_cal_read(a, &r, &st, &sum)) {
		return false;
	}
	say("apollo cs35l41: %s: calibration CAL_R %#x STATUS %#x CHECKSUM %#x (wanted %#x, 1, %#x)\n", amps[a].name, r, st, sum,
	    amps[a].cal_r, amps[a].cal_r + 1);
	ok = r == amps[a].cal_r && st == 1 && sum == amps[a].cal_r + 1;
	if (!ok && dsp_mbox(a, MBOX_CMD_PAUSE, 1)) {
		bool wrote = dsp_cal_write(a);

		sleep_ms(3);
		if (wrote && dsp_cal_read(a, &r, &st, &sum)) {
			say("apollo cs35l41: %s: calibration with the firmware paused: CAL_R %#x STATUS %#x CHECKSUM %#x\n", amps[a].name, r, st, sum);
			ok = r == amps[a].cal_r && st == 1 && sum == amps[a].cal_r + 1;
		}
		if (!dsp_mbox(a, MBOX_CMD_RESUME, 0)) {
			dsp_report(a, "RESUME after the paused calibration FAILED");
			return false;
		}
		sleep_ms(3);
		if (dsp_cal_read(a, &r, &st, &sum)) {
			ok = r == amps[a].cal_r && st == 1 && sum == amps[a].cal_r + 1;
			say("apollo cs35l41: %s: calibration after RESUME: CAL_R %#x STATUS %#x CHECKSUM %#x (%s)\n", amps[a].name, r, st, sum,
			    ok ? "kept" : "LOST");
		}
	}
	return ok;
}

/* Stage 3 for one amplifier: cs_dsp_run (the MPU unlocked, the core started), cs35l41_write_fs_errata (the rate slots), the
 * DSP's monitor inputs (cs35l41_dsp_init). The core is started here, at boot, without a bit clock: the firmware is NOT awaited
 * (run 221026: no heartbeat then). Linux starts the core when the playback path powers up (the "DSP1 Preload" widget's POST_PMU
 * runs wm_adsp_event -> cs_dsp_run) with the PLL running; dsp_playback_check() looks for the heartbeat at the first playback. */
static bool
dsp_run(unsigned a, const uint8_t *fw, uint64_t fw_size)
{
	uint8_t amp = amps[a].address;
	const struct dsp_info *info = &dsp_info[a];
	uint32_t halo_state_reg = 0, heartbeat_reg = 0, enable_reg = 0, command_reg = 0, state_reg = 0, error_reg = 0, temperature_reg = 0;
	uint32_t v;

	if (!dsp_control(fw, fw_size, info, FW_ID_HALO_CSPL, "HALO_STATE", &halo_state_reg) ||
	    !dsp_control(fw, fw_size, info, FW_ID_HALO_CSPL, "HALO_HEARTBEAT", &heartbeat_reg) ||
	    !dsp_control(fw, fw_size, info, ALG_ID_CSPL, "CSPL_ENABLE", &enable_reg) ||
	    !dsp_control(fw, fw_size, info, ALG_ID_CSPL, "CSPL_COMMAND", &command_reg) ||
	    !dsp_control(fw, fw_size, info, ALG_ID_CSPL, "CSPL_STATE", &state_reg) ||
	    !dsp_control(fw, fw_size, info, ALG_ID_CSPL, "CSPL_ERRORNO", &error_reg) ||
	    !dsp_control(fw, fw_size, info, ALG_ID_CSPL, "CSPL_TEMPERATURE", &temperature_reg)) {
		say("apollo cs35l41: %s: the firmware's HALO_STATE, HALO_HEARTBEAT or CSPL_* controls are not in the .wmfw\n", amps[a].name);
		return false;
	}
	say("apollo cs35l41: %s: controls (from the .wmfw's algorithm data): HALO_STATE %#x, HALO_HEARTBEAT %#x, CSPL_ENABLE %#x, "
	    "CSPL_COMMAND %#x, CSPL_STATE %#x, CSPL_ERRORNO %#x, CSPL_TEMPERATURE %#x\n", amps[a].name, halo_state_reg, heartbeat_reg,
	    enable_reg, command_reg, state_reg, error_reg, temperature_reg);
	dsp_ctl[a].halo_state = halo_state_reg, dsp_ctl[a].heartbeat = heartbeat_reg, dsp_ctl[a].enable = enable_reg;
	dsp_ctl[a].command = command_reg, dsp_ctl[a].state = state_reg, dsp_ctl[a].errorno = error_reg;
	dsp_ctl[a].temperature = temperature_reg;

	/* the calibration controls (apollo_amps_dsp >= 3): by name from the .wmfw, Android's addresses if the names are not there */
	if (dsp_level >= 3) {
		uint32_t r = 0, st = 0, sum = 0;

		if (dsp_control(fw, fw_size, info, ALG_ID_CSPL, "CAL_R", &r) && dsp_control(fw, fw_size, info, ALG_ID_CSPL, "CAL_STATUS", &st) &&
		    dsp_control(fw, fw_size, info, ALG_ID_CSPL, "CAL_CHECKSUM", &sum)) {
			say("apollo cs35l41: %s: calibration controls from the .wmfw: CAL_R %#x CAL_STATUS %#x CAL_CHECKSUM %#x (Android: %#x %#x %#x)\n",
			    amps[a].name, r, st, sum, CAL_R_ADDR, CAL_STATUS_ADDR, CAL_CHECKSUM_ADDR);
		} else {
			r = CAL_R_ADDR, st = CAL_STATUS_ADDR, sum = CAL_CHECKSUM_ADDR;
			say("apollo cs35l41: %s: CAL_* not found in the .wmfw: Android's addresses\n", amps[a].name);
		}
		dsp_ctl[a].cal_r = r, dsp_ctl[a].cal_status = st, dsp_ctl[a].cal_checksum = sum;
	}

	/* the calibration with the core still stopped (as Linux writes it before cs_dsp_run), read back, and again after the core has
	 * run 20 ms: tells whether the memory takes the words and whether the firmware's start clears them */
	if (dsp_level >= 3 && dsp_ctl[a].cal_r != 0 && dsp_cal_write(a)) {
		uint32_t cr = 0, cs = 0, ck = 0;

		if (dsp_cal_read(a, &cr, &cs, &ck)) {
			say("apollo cs35l41: %s: calibration written with the core stopped: CAL_R %#x STATUS %#x CHECKSUM %#x\n", amps[a].name, cr, cs, ck);
		}
	}
	/* cs_dsp_halo_configure_mpu with lock_regions = 0xffffffff (cs35l41_configure_cs_dsp): nothing locked */
	if (!reg_write(amp, DSP_MPU_LOCK_CONFIG, 0x5555) || !reg_write(amp, DSP_MPU_LOCK_CONFIG, 0xaaaa)) {
		return false;
	}
	for (uint32_t g = 0; g < 4; g++) {
		static const uint32_t groups[5] = { 0x00, 0x04, 0x08, 0x0c, 0x14 };         /* XMEM, YMEM, WINDOW, XREG, YREG */

		for (uint32_t k = 0; k < 5; k++) {
			if (!reg_write(amp, DSP_MPU_ACCESS + 0x18 * g + groups[k], 0xffffffffu)) {
				return false;
			}
		}
	}
	if (!reg_write(amp, DSP_MPU_LOCK_CONFIG, 0)) {
		return false;
	}
	for (uint32_t s = 0; s < 8; s++) {              /* the slots' rates, 0x1 (cs35l41_fs_errata_patch) */
		if (!reg_write(amp, DSP_RX1_RATE + 8 * s, 1) || !reg_write(amp, DSP_TX1_RATE + 8 * s, 1)) {
			return false;
		}
	}
	/* the monitor inputs of cs35l41_dsp_init (not the reset defaults for RX5 and RX6): VPMON, VBSTMON, TEMPMON, reserved */
	if (!reg_write(amp, REG_DSP1_RX1_SRC + 0x10, SRC_VPMON) || !reg_write(amp, REG_DSP1_RX1_SRC + 0x14, SRC_VBSTMON) ||
	    !reg_write(amp, REG_DSP1_RX1_SRC + 0x18, SRC_TEMPMON) || !reg_write(amp, REG_DSP1_RX1_SRC + 0x1c, SRC_RSVD)) {
		return false;
	}
	if (!dsp_core_start(amp)) {
		return false;
	}
	sleep_ms(20);
	dsp_report(a, "core started at boot (no bit clock yet: the heartbeat is not required here)");
	if (dsp_level >= 3 && dsp_ctl[a].cal_r != 0) {
		uint32_t cr = 0, cs = 0, ck = 0;

		if (dsp_cal_read(a, &cr, &cs, &ck)) {
			say("apollo cs35l41: %s: calibration after the core's first 20 ms: CAL_R %#x STATUS %#x CHECKSUM %#x\n", amps[a].name, cr, cs, ck);
		}
	}
	if (reg_read(amp, 0x02bc3104, &v)) {                /* HALO_MPU_XM_VIO_STATUS: a violation would show here */
		if (v & 0x007e0000u) {
			say("apollo cs35l41: %s: MPU XM violation status %#x\n", amps[a].name, v);
		}
	}
	dsp_ctl[a].valid = true;
	dsp_ctl[a].routed = false;
	say("apollo cs35l41: %s: the firmware is loaded and its core started; it is judged at the first playback\n", amps[a].name);
	return true;
}

/* The PLL locked (CS35L41_PLL_LOCK, BIT(1) of IRQ1_STATUS3; the raw register is the live level, the latched one is accepted too).
 * Up to 500 ms, polled every 5 ms; the time to lock is logged. */
static bool
dsp_wait_pll(unsigned a)
{
	uint8_t amp = amps[a].address;
	uint32_t raw = 0, latched = 0;
	uint64_t start = now_ms();

	for (int i = 0; i < 100; i++) {
		if (reg_read(amp, REG_IRQ1_RAW_STATUS3, &raw) && (raw & REG_IRQ1_STATUS3_PLL)) {
			say("apollo cs35l41: %s: PLL locked after %llu ms (raw IRQ1 status 3 %#x)\n", amps[a].name, now_ms() - start, raw);
			return true;
		}
		reg_read(amp, REG_IRQ1_STATUS3, &latched);
		if (latched & REG_IRQ1_STATUS3_PLL) {
			say("apollo cs35l41: %s: PLL lock latched after %llu ms (raw %#x, latched %#x)\n", amps[a].name, now_ms() - start, raw,
			    latched);
			return true;
		}
		sleep_ms(5);
	}
	say("apollo cs35l41: %s: PLL NOT locked after %llu ms (raw IRQ1 status 3 %#x, latched %#x)\n", amps[a].name, now_ms() - start,
	    raw, latched);
	return false;
}

/* What the firmware's CSPL_ERRORNO says: neither upstream Linux, the .wmfw nor Cirrus's public files give the codes, so only the
 * zero is known (no error) */
static const char *
dsp_errorno_text(uint32_t err)
{
	return err == 0 ? "no error" : "meaning not documented in Linux, the .wmfw or Cirrus's public files";
}

/* The firmware's check at a playback start, after the amplifier powered up and the PLL locked (Linux: the core runs at POST_PMU,
 * wm_adsp_event -> cs_dsp_run): the core is stopped and started again (cs_dsp_halo_start_core), and the heartbeat must change
 * twice within 300 ms. CSPL_STATE and CSPL_ERRORNO are read and logged. */
static bool
dsp_playback_check(unsigned a)
{
	uint8_t amp = amps[a].address;
	uint32_t beat = 0, last = 0, beats = 0, state = 0xdead, err = 0xdead;

	/*
	 * No restart: the core started at boot runs by itself once the PLL locks -- its heartbeat had counted to 0xd (and the
	 * earpiece's to 0x213) by the time this looked, and the restart is what stopped it (HEARTBEAT 1, CSPL_STATE 1,
	 * CSPL_ERRORNO 2 after it; 25 October). Linux too starts the core once (cs_dsp_run at POST_PMU) and never again.
	 */
	dsp_report(a, "at the playback start, amplifier powered, PLL locked");
	for (int i = 0; i < 30; i++) {
		if (!reg_read(amp, dsp_ctl[a].heartbeat, &beat)) {
			return false;
		}
		if (i > 0 && beat != last) {
			beats++;
		}
		last = beat;
		if (beats >= 2) {
			break;
		}
		sleep_ms(10);
	}
	reg_read(amp, dsp_ctl[a].state, &state), reg_read(amp, dsp_ctl[a].errorno, &err);
	say("apollo cs35l41: %s: CSPL_STATE %#x, CSPL_ERRORNO %#x (%s)\n", amps[a].name, state, err, dsp_errorno_text(err));
	if (beats >= 2) {
		say("apollo cs35l41: %s: the firmware's heartbeat counts at the playback start (%u changes, now %#x)\n", amps[a].name, beats,
		    beat);
		dsp_report(a, "at the playback start, heartbeat counting");
		return true;
	}
	say("apollo cs35l41: %s: no heartbeat at the playback start (%u changes, %#x)\n", amps[a].name, beats, beat);
	dsp_report(a, "at the playback start, NO heartbeat");
	return false;
}

/* Both amplifiers reset together (they share the line) and set up again, without the firmware */
static void
amps_plain_pcm(void)
{
	pin_out(PIN_RESET, false);
	sleep_ms(2);
	pin_out(PIN_RESET, true);
	sleep_ms(2);
	for (unsigned a = 0; a < 2; a++) {
		amp_ready[a] = amp_used[a] && amp_setup(a);
	}
}

/* Cores stopped, both amplifiers reset and set up again as plain PCM, the firmware given up for this run */
static void
dsp_failsafe(void)
{
	for (unsigned a = 0; a < 2; a++) {
		if (amp_used[a]) {
			dsp_stop_core(amps[a].address);
		}
		dsp_ctl[a].valid = false;
		dsp_ctl[a].routed = false;
	}
	dsp_level = 0;
	amps_plain_pcm();
}

static void
dsp_stage(void)
{
	static const char *const bins[2] = { "cs35l41-dsp1-spk-prot-revb2.bin", "cs35l41-dsp1-spk-prot-RCV-revb2.bin" };
	uint64_t fw_size = 0, start = now_ms();
	const uint8_t *fw = hakios_image_file("fw", "cs35l41-dsp1-spk-prot-revb2.wmfw", &fw_size, NULL);     /* (never freed: it can
	                                                                                                       * point into the image) */
	bool ok = fw != NULL;

	if (!ok) {
		say("apollo cs35l41: apollo_amps_dsp=%u: cs35l41-dsp1-spk-prot-revb2.wmfw is not in the image's fw/ (fw-audio-apollo/): plain PCM\n",
		    dsp_level);
		return;
	}
	for (unsigned a = 0; a < 2 && ok; a++) {
		uint64_t bin_size = 0;
		const uint8_t *bin;

		if (!amp_ready[a]) {
			continue;
		}
		bin = hakios_image_file("fw", bins[a], &bin_size, NULL);
		if (bin == NULL) {
			say("apollo cs35l41: %s is not in the image's fw/\n", bins[a]);
			ok = false;
		} else {
			ok = dsp_load(a, fw, fw_size, bin, bin_size) && (dsp_level < 2 || dsp_run(a, fw, fw_size));
		}
	}
	if (ok) {
		say("apollo cs35l41: apollo_amps_dsp=%u: the firmware is loaded%s on every amplifier in use after %llu ms; the audio stays PCM "
		    "until the first playback judges it\n", dsp_level, dsp_level >= 2 ? " and its core started" : "", now_ms() - start);
		return;
	}
	say("apollo cs35l41: apollo_amps_dsp=%u FAILED: both amplifiers reset and set up as plain PCM\n", dsp_level);
	dsp_failsafe();
}

static bool
map_all(void)
{
	if (gcc != 0) {
		return true;
	}
	pin_reset = ml_io_map(TLMM_NORTH_PHYS + 0x1000UL * PIN_RESET, 0x1000);
	pins_i2c = ml_io_map(TLMM_NORTH_PHYS + 0x1000UL * PIN_SDA, 0x2000);
	gcc = ml_io_map(GCC_PHYS, GCC_SIZE);
	se = ml_io_map(SE_PHYS, SE_SIZE);
	return pin_reset && pins_i2c && gcc && se;
}

static bool
amps_prepare(void)
{
	uint32_t which = 3, select = 0;
	const char *problem;
	bool any = false;

	PE_parse_boot_argn("apollo_amps", &which, sizeof(which));
	dsp_level = 0;
	PE_parse_boot_argn("apollo_amps_dsp", &dsp_level, sizeof(dsp_level));
	if (dsp_level >= 3 && AMP_RATE_HZ != DSP_TUNING_RATE_HZ) {
		say("apollo cs35l41: apollo_amps_dsp=%u REFUSED: the tuning is for %u Hz and this runs %u Hz; stage 2 instead (the firmware "
		    "loaded and run, the audio PCM)\n", dsp_level, DSP_TUNING_RATE_HZ, AMP_RATE_HZ);
		dsp_level = 2;
	}
	amp_used[0] = (which & 1) != 0;
	amp_used[1] = (which & 2) != 0;
	if (!map_all()) {
		say("apollo cs35l41: cannot map the registers\n");
		return false;
	}
	if (clocks_on(&select) != 0) {
		return false;
	}
	wr(pin(PIN_SDA), FUNCTION_QUP3 << 2 | (2 / 2 - 1) << 6);
	wr(pin(PIN_SCL), FUNCTION_QUP3 << 2 | (2 / 2 - 1) << 6);
	problem = i2c_init(select);
	if (problem != NULL) {
		say("apollo cs35l41: %s\n", problem);
		return false;
	}
	memset(dsp_ctl, 0, sizeof(dsp_ctl));
	/* both out of reset at once: they share the line (cs35l41_probe: low 2 ms, high, 2 ms) */
	pin_out(PIN_RESET, false);
	sleep_ms(2);
	pin_out(PIN_RESET, true);
	sleep_ms(2);
	prepared = true;
	for (unsigned a = 0; a < 2; a++) {
		amp_ready[a] = amp_used[a] && amp_setup(a);
		any |= amp_ready[a];
	}
	if (any && dsp_level >= 1) {
		dsp_stage();
		any = amp_ready[0] || amp_ready[1];
	}
	return any;
}

/* Powered up (the clock runs: hakios.apollo_sound.c has started the port), unmuted only by apollo_cs35l41_volume() */
static bool
amps_start(void)
{
	bool any = false, dsp_failed = false;

	for (unsigned a = 0; a < 2; a++) {
		uint8_t amp = amps[a].address;
		uint32_t status = 0;
		int i;

		if (!amp_ready[a] || amp_on[a]) {
			any |= amp_on[a];
			continue;
		}
		if (dsp_level >= 3 && dsp_ctl[a].valid && !dsp_feedback(a)) {
			say("apollo cs35l41: %s: the feedback setup failed\n", amps[a].name);
			dsp_failed = true;
			continue;
		}
		if (!reg_update(amp, REG_SP_ENABLES, 1u << 16, 1u << 16) ||
		    !test_key(amp, true) || !reg_write(amp, REG_PUP_PDN, 0x002f1aa0) || !test_key(amp, false) ||
		    !reg_update(amp, REG_PWR_CTRL1, 1, 1)) {
			continue;
		}
		for (i = 0; i < 100; i++) {
			if (reg_read(amp, REG_IRQ1_STATUS1, &status) && (status & PUP_DONE)) {
				break;
			}
			sleep_ms(1);
		}
		reg_write(amp, REG_IRQ1_STATUS1, PUP_DONE);
		if (i == 100 || !reg_update(amp, REG_PWR_CTRL2, 1, 1)) {
			say("apollo cs35l41: %s: did not power up (status %#x)\n", amps[a].name, status);
			continue;
		}
		amp_on[a] = true;
		any = true;
		read_failures[a] = 0;
		volume_at = 8 * 102;            /* (powered up muted) */
		say("apollo cs35l41: %s: POWERED UP, muted\n", amps[a].name);
	}
	/* The firmware, after the amplifiers powered up (muted) with the bit clock running (Linux's order: the PLL from the clock, the
	 * core run at POST_PMU, then RESUME): the PLL locked, the core restarted with a counting heartbeat, RESUME answered with
	 * status 0. Only then the DAC may take the DSP's output (apollo_amps_dsp >= 3). */
	if (dsp_level >= 3) {
		say("apollo cs35l41: NOTE the rate: Android runs these amplifiers at 96 kHz (GLOBAL_FS 4, PLL_CLK_CTRL 0x10770, SCLK 24.576 MHz) with "
		    "the protection firmware; this runs 48 kHz (GLOBAL_FS 3, 12.288 MHz)\n");
	}
	for (unsigned a = 0; a < 2 && dsp_level >= 2 && !dsp_failed; a++) {
		if (!amp_on[a] || !dsp_ctl[a].valid) {
			continue;
		}
		/*
		 * RESUME before the heartbeat is looked at: the previous playback's end left the firmware PAUSED (mailbox status 1), and
		 * a paused firmware's heartbeat stands still -- it had counted to 0x16d0 and the check called it dead (25 October).
		 */
		if (!dsp_wait_pll(a)) {
			dsp_failed = true;
		} else if (!dsp_mbox(a, MBOX_CMD_RESUME, 0)) {      /* cs35l41_dsp_audio_ev POST_PMU (internal boost: no SPK_OUT_ENABLE) */
			dsp_report(a, "RESUME FAILED");
			dsp_failed = true;
		} else if (!dsp_playback_check(a)) {
			dsp_failed = true;
		} else if (dsp_level >= 3 && (dsp_ctl[a].cal_r == 0 || !dsp_calibrate(a))) {
			/*
			 * Not a failure: written before RESUME the values read back as zeros (25 October), so they are tried with the
			 * firmware running. But the DAC is not given the firmware's output without them (stage 2 for this amplifier):
			 * its tuning's default impedance is not this speaker's.
			 */
			dsp_ctl[a].calibrated = false;
			say("apollo cs35l41: %s: the calibration did not take: the audio stays PCM (stage 2), the firmware not routed\n",
			    amps[a].name);
		} else {
			dsp_ctl[a].calibrated = dsp_level >= 3;
			dsp_report(a, "after RESUME, heartbeat counting");
		}
	}
	for (unsigned a = 0; a < 2 && dsp_level >= 3; a++) {
		uint8_t amp = amps[a].address;

		if (!amp_on[a] || !dsp_ctl[a].valid || dsp_ctl[a].routed) {
			continue;
		}
		if (AMP_RATE_HZ != DSP_TUNING_RATE_HZ || !dsp_ctl[a].calibrated) {
			say("apollo cs35l41: %s: NOT routed through the firmware (%s): PCM, as stage 2\n", amps[a].name,
			    AMP_RATE_HZ != DSP_TUNING_RATE_HZ ? "not at the tuning's rate" : "not calibrated");
			continue;
		}
		/* Android's "speaker" path (mixer_paths_overlay_static.xml): PCM Source DSP, DSP RX1 and RX2 from ASPRX1 (the channel's own
		 * slot, SP_FRAME_RX_SLOT), the DRE switch on; "Boost Class-H Tracking" and "Class-H Head Room" are Cirrus's Android controls
		 * whose registers are not in upstream Linux nor in Android's register dump, so they are left as they are. */
		if (!reg_update(amp, REG_PWR_CTRL3, PWR_CTRL3_DRE, PWR_CTRL3_DRE) || !reg_update(amp, REG_DAC_PCM1_SRC, 0x7f, SRC_DSP_TX1)) {
			dsp_failed = true;
			break;
		}
		dsp_ctl[a].routed = true;
		say("apollo cs35l41: %s: DAC_PCM1_SRC = DSPTX1, DSP RX1 and RX2 = ASPRX1, DRE on: the audio goes through the firmware\n",
		    amps[a].name);
	}
	if (dsp_failed) {
		for (unsigned a = 0; a < 2; a++) {
			uint32_t s1 = 0xdead, s2 = 0xdead, s3 = 0xdead, s4 = 0xdead;

			if (amp_on[a] && dsp_ctl[a].valid) {
				reg_read(amps[a].address, REG_IRQ1_STATUS1, &s1), reg_read(amps[a].address, REG_IRQ1_STATUS1 + 4, &s2);
				reg_read(amps[a].address, REG_IRQ1_STATUS3, &s3), reg_read(amps[a].address, REG_IRQ1_STATUS3 + 4, &s4);
				say("apollo cs35l41: %s: failure state: IRQ1_STATUS1 %#x STATUS2 %#x STATUS3 %#x STATUS4 %#x\n", amps[a].name, s1, s2, s3, s4);
				dsp_report(a, "at the failure");
			}
		}
		say("apollo cs35l41: THE FIRMWARE IS NOT HEALTHY (PLL, heartbeat, RESUME or routing): amplifiers powered down, reset, plain PCM\n");
		amps_stop();
		dsp_failsafe();
		return amps_start();                    /* (dsp_level is 0 now: no second pass through this) */
	}
	return any;
}

/* `below` eighths of a dB under Android's level (0 dB of digital volume): stepped there, 6 dB a step, step_ms apart */
static bool
amps_volume(uint32_t below, uint32_t step_ms)
{
	uint32_t at = volume_at;
	bool ok = true;

	if (below > 8 * 102) {
		below = 8 * 102;
	}
	while (at != below) {
		at = at > below ? (at - below > 48 ? at - 48 : below) : (below - at > 48 ? at + 48 : below);
		for (unsigned a = 0; a < 2; a++) {
			if (amp_on[a]) {
				uint32_t vol = at >= 8 * 102 ? VOL_MUTE : ((uint32_t)(-(int32_t)at) & 0x7ff);

				ok &= reg_update(amps[a].address, REG_AMP_DIG_VOL_CTRL, 0x7ffu << 3, vol << 3);
			}
		}
		volume_at = at;
		if (step_ms) {
			sleep_ms(step_ms);
		}
	}
	return ok;
}

/*
 * No fault reported (an amplifier's short, its temperature, its boost); on one, both are powered down. A status that
 * cannot be read is not taken for a healthy one: READ_FAILURES_MOST failed reads in a row, and both are powered down too
 * (a bus that stopped answering would otherwise hide every fault while they play). TEMP_WARN is noted for the monitor.
 */
static bool
amps_healthy(void)
{
	for (unsigned a = 0; a < 2; a++) {
		uint32_t status = 0;

		if (!amp_on[a]) {
			continue;
		}
		if (!reg_read(amps[a].address, REG_IRQ1_STATUS1, &status)) {
			if (++read_failures[a] >= READ_FAILURES_MOST) {
				say("apollo cs35l41: %s: ITS STATUS CANNOT BE READ (%u times in a row): both powered down\n", amps[a].name,
				    read_failures[a]);
				amps_stop();
				return false;
			}
			continue;
		}
		read_failures[a] = 0;
		if (status & IRQ1_ERRORS) {
			say("apollo cs35l41: %s REPORTS A FAULT (%#x): both powered down\n", amps[a].name, status);
			amps_stop();
			return false;
		}
		if ((status & IRQ1_TEMP_WARN) && !temp_warned) {
			temp_warned = true;
			say("apollo cs35l41: %s WARNS OF ITS TEMPERATURE (%#x): the level is cut by %u.%03u dB for the rest of the run\n",
			    amps[a].name, status, TEMP_WARN_CUT / 8, TEMP_WARN_CUT % 8 * 125);
		}
	}
	return true;
}

/* Muted, powered down (cs35l41_global_enable(0), the power-down patch) */
static void
amps_stop(void)
{
	for (unsigned a = 0; a < 2; a++) {
		uint8_t amp = amps[a].address;
		uint32_t status = 0;

		if (!amp_on[a]) {
			continue;
		}
		reg_update(amp, REG_AMP_DIG_VOL_CTRL, 0x7ffu << 3, VOL_MUTE << 3);
		if (dsp_ctl[a].valid) {
			dsp_mbox(a, MBOX_CMD_PAUSE, 1);                 /* cs35l41_dsp_audio_ev PRE_PMD (a failure only logs) */
		}
		reg_update(amp, REG_PWR_CTRL2, 1, 0);
		reg_update(amp, REG_PWR_CTRL1, 1, 0);
		for (int i = 0; i < 100; i++) {
			if (reg_read(amp, REG_IRQ1_STATUS1, &status) && (status & PDN_DONE)) {
				break;
			}
			sleep_ms(1);
		}
		reg_write(amp, REG_IRQ1_STATUS1, PDN_DONE);
		test_key(amp, true);
		reg_write(amp, REG_PUP_PDN, 0x002f1aa3);
		test_key(amp, false);
		reg_update(amp, REG_SP_ENABLES, 1u << 16, 0);
		if (dsp_ctl[a].feedback) {                                  /* the monitors and the extra ASP slots off again */
			reg_update(amp, REG_SP_ENABLES, SP_EN_RX2 | SP_EN_TX1 | SP_EN_TX2, 0);
			reg_update(amp, REG_PWR_CTRL2, PWR_CTRL2_MONITORS, 0);
			dsp_ctl[a].feedback = false;
		}
		if (dsp_ctl[a].routed) {                                    /* the DAC back on the ASP: PCM until the next start routes it */
			reg_update(amp, REG_DAC_PCM1_SRC, 0x7f, SRC_ASPRX1);
			dsp_ctl[a].routed = false;
		}
		amp_on[a] = false;
		volume_at = 8 * 102;            /* (muted on the way down) */
		say("apollo cs35l41: %s: powered down\n", amps[a].name);
	}
}

/* On a panic's way out: both held in reset at once (no I2C, no sleep) */
void
apollo_cs35l41_quiesce(void)
{
	if (prepared && pin_reset != 0) {
		wr(pin_reset + 4, 0);
	}
}

/*
 * The further reduction the guard is to make (eighths of a dB): TEMP_WARN (IRQ1_STATUS1 bit 15, latched, never cleared
 * here), once either amplifier has shown it, holds the level TEMP_WARN_CUT down for the rest of the run -- Android's
 * protection firmware would turn it down by itself; this has no measure of how far, so it stays cut. Nothing else yet
 * (Android's cold-amplifier and battery tables are not carried over).
 */
static uint32_t
amps_monitor(bool tell)
{
	for (unsigned a = 0; a < 2 && !temp_warned; a++) {
		uint32_t status = 0;

		if (amp_on[a] && reg_read(amps[a].address, REG_IRQ1_STATUS1, &status) && (status & IRQ1_TEMP_WARN)) {
			temp_warned = true;
			say("apollo cs35l41: %s WARNS OF ITS TEMPERATURE (%#x): the level is cut by %u.%03u dB for the rest of the run\n",
			    amps[a].name, status, TEMP_WARN_CUT / 8, TEMP_WARN_CUT % 8 * 125);
		}
	}
	if (tell && temp_warned) {
		say("apollo cs35l41: the temperature warning holds the level %u.%03u dB down\n", TEMP_WARN_CUT / 8, TEMP_WARN_CUT % 8 * 125);
	}
	return temp_warned ? TEMP_WARN_CUT : 0;
}

/* ---- the entry points (hakios.apollo_sound.c): one at a time, under amps_lock ------------------------------------- */

bool
apollo_cs35l41_prepare(__unused const uint8_t *config, __unused uint64_t size)
{
	bool ok;

	lck_mtx_lock(&amps_lock);
	ok = amps_prepare();
	lck_mtx_unlock(&amps_lock);
	return ok;
}

bool
apollo_cs35l41_start(void)
{
	bool ok;

	lck_mtx_lock(&amps_lock);
	ok = amps_start();
	lck_mtx_unlock(&amps_lock);
	return ok;
}

bool
apollo_cs35l41_volume(uint32_t below, uint32_t step_ms)
{
	bool ok;

	lck_mtx_lock(&amps_lock);
	ok = amps_volume(below, step_ms);
	lck_mtx_unlock(&amps_lock);
	return ok;
}

bool
apollo_cs35l41_healthy(void)
{
	bool ok;

	lck_mtx_lock(&amps_lock);
	ok = amps_healthy();
	lck_mtx_unlock(&amps_lock);
	return ok;
}

void
apollo_cs35l41_stop(void)
{
	lck_mtx_lock(&amps_lock);
	amps_stop();
	lck_mtx_unlock(&amps_lock);
}

uint32_t
apollo_cs35l41_monitor(bool tell)
{
	uint32_t below;

	lck_mtx_lock(&amps_lock);
	below = amps_monitor(tell);
	lck_mtx_unlock(&amps_lock);
	return below;
}
