/*
 * The vibration motor of the Xiaomi Mi 10T (apollo): a linear resonant actuator driven by an Awinic AW8697 (chip ID
 * 0x97 in register 0), Android's "aw8697_haptic" (Xiaomi's driver drivers/input/misc/aw8697_haptic/aw8697.c of the
 * apollo-q-oss kernel). This is hakios.haptic.c's service (what iOS plays on its "Actuator" device arrives at
 * /dev/apollo-haptic as samples and is played as Android's waveforms), for another chip on another bus. From the
 * phone's own device tree (//soc/i2c@a8c000/aw8697_haptic@5A):
 *
 *   I2C        the serial engine 11 = the sixth of the second QUPv3 wrapper (qupv3_se11_i2c at 0xa8c000; the
 *              wrapper's core at 0xac0000): function qup11 (1) on GPIO 60 data and 61 clock, 2 mA, no pull, address
 *              0x5a; 400 kHz through the serial engine's FIFO as in hakios.apollo_cs35l41.c
 *   reset      GPIO 130, high = running (Android's hw_reset: low 5 ms, high 5 ms)
 *   interrupt  GPIO 129, pulled up (not used: polled)
 *   effects    Android's RAM waveforms, aw8697_haptic.bin (fw/ of the small image): 1..10 short (Android's effects
 *              0..9: vib_effect_id_boundary 0xa), 11 the one Android repeats forever for a long vibration
 *
 * Nothing here is done unless the boot-arg apollo_haptic=N asks for it (N as redwood's haptic=N):
 *
 *   1  a look, reading only: the pins, and whether the chip answers (its ID, 0x97). Out of the firmware the reset
 *      pin is an input with a pull-down: the chip is held in reset and does not answer.
 *   2  ... and the reset released as Android's driver releases it, and the chip's registers read (nothing written)
 *   3  ... and Android's configuration written (aw8697_haptic_init), Android's waveforms loaded into the chip's RAM
 *      and read back; then standby. Nothing moves.
 *   4  ... and one short effect played (apollo_haptic_effect=N, default 1: Android's effect 0)
 *   5  ... configured and loaded as for 3, and the motor for iOS until the run ends: /dev/apollo-haptic (mode 0666)
 *      takes int16 mono 48 kHz samples as /dev/hakios-haptic does; a waveform of iOS's (a stretch of samples after
 *      a pause) is a tap (waveform 1), one that goes on for longer than a tap the looping waveform (11) until it
 *      ends -- HAPTIC_MAX_MS at most, then the motor rests as long. The gain follows the loudness (gain_for()).
 *      Taps and vibrations together drive the motor HAPTIC_MAX_MS at most in any DUTY_WINDOW_MS (2 s): past that
 *      it is stopped and rests HAPTIC_MAX_MS, and nothing starts until the window has room again.
 *
 * Limits, whatever asks: Android's own values only (gain 0x80 at the most, its boost settings, its waveforms); no
 * effect longer than HAPTIC_MAX_MS, no more than HAPTIC_MAX_MS of driving in any DUTY_WINDOW_MS; any fault the chip
 * reports (boost short circuit or over-voltage, under-voltage, over-current, over-temperature), a byte nobody acknowledges or a RAM that does not read back, and the chip is put
 * into reset for the rest of the run. At the run's end and on a panic's way out the chip goes back into reset.
 *
 * Taken from Android's driver (aw8697.c, apollo-q-oss), function by function:
 *   aw8697_hw_reset            reset low 5 ms, high 5 ms
 *   aw8697_read_chipid         REG_ID (0x00) = 0x97, then aw8697_haptic_softreset: REG_ID <- 0xaa, 1 ms
 *   aw8697_haptic_init         standby; PWMDBG[6:5] = 2 (24 kHz); BSTDBG1..3 <- DT vib_bstdbg 0x30 0xeb 0xd4; TSET <-
 *                              0x12; R_SPARE <- 0x68; ANADBG[3:2] = 3; BSTCFG[2:0] = 5 (3.5 A); motor protection off
 *                              (DETCTRL[5] = 1, PWMPRC[7] = 0, PRLVL[7] = 0); BST_AUTO[2] = 0; offset calibration;
 *                              ADCTEST[6] = 1 (vbat hardware compensation); BEMF_VTH <- DT vib_bemf_config
 *   aw8697_container_update    RAMINIT (SYSCTRL[5]) on; BASE_ADDRH/L <- the file's bytes 2, 3; FIFO_AE/AF from the
 *                              base address; RAMADDRH/L <- the same; the file from byte 4 on into RAMDATA; RAMINIT off
 *   aw8697_ram_loaded          the file's first two bytes: the sum of all the others (16 bits)
 *   aw8697_haptic_play_effect_seq / _play_mode / _play_go / _set_wav_seq / _set_wav_loop /
 *   _set_repeat_wav_seq / _set_gain / _set_bst_vol
 *                              WAVSEQ1 (0x07) <- waveform, WAVSEQ2 <- 0, WAVLOOP1 (0x0f) <- 0 (once) or 0xf0 (forever),
 *                              SYSCTRL: RAM mode, active, boost (once) or bypass (looped); DATDBG (0x39) <- gain;
 *                              BSTDBG4[5:1] <- DT vib_bst_vol_ram 0x11; GO (0x05) <- 1
 *   aw8697_haptic_stop         GO <- 0, until GLB_STATE (0x46) & 0xf is 0, then SYSCTRL[0] = 1 (standby)
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <miscfs/devfs/devfs.h>
#include <sys/conf.h>
#include <sys/systm.h>
#include <sys/uio.h>
#include <sys/hakios.apollo_input.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern void loader_log_putc(char c);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

static uint32_t stage;
static bool broken, released, leaving, prepared;

/* every line to the loader's log as well (and to the console when that may be written) */
extern bool apollo_rpmh_bcm_vote(const char *name, uint32_t ab, uint32_t ib);    /* hakios.apollo_cpufreq.c */
extern void apollo_cmd_db_dump(void);
static bool bringing_up;                /* the chip's bring-up: every line waits pause_ms */
static uint32_t pause_ms = 400;
static uint32_t start_after_ms = 15000;  /* (the start's flood of lines over first: apollo_haptic_after=MS) */
static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[300];
	va_list list;
	int length;

	va_start(list, format);
	length = vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	if (!leaving && ml_get_interrupts_enabled()) {
		printf("%s", line);
		/*
		 * The first runs reset the phone without a word (25 October): while the chip is being brought up, every line
		 * is given time to reach the loader before the next access (apollo_haptic_pause=MS, 0: none).
		 */
		if (bringing_up && pause_ms) {
			delay_for_interval(pause_ms, 1000 * NSEC_PER_USEC);
		}
	}
	return length;
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
spin_us(uint32_t us)
{
	uint64_t until;

	nanoseconds_to_absolutetime((uint64_t)us * NSEC_PER_USEC, &until);
	until += mach_absolute_time();
	while (mach_absolute_time() < until) {
	}
}

static void
sleep_us(uint32_t us)
{
	if (leaving) {
		spin_us(us);
	} else {
		delay_for_interval(us, NSEC_PER_USEC);
	}
}

static void
wait_ms(uint32_t ms)
{
	if (leaving) {
		spin_us(ms * 1000);
	} else {
		delay_for_interval(ms, NSEC_PER_MSEC);
	}
}

/* ---- pins (TLMM, south tile: one page per pin, mapped one pair at a time) ------------------------------------ */

#define TLMM_SOUTH_PHYS         0x0f500000UL
#define PIN_SDA                 60
#define PIN_SCL                 61
#define PIN_INT                 129
#define PIN_RESET               130
#define FUNCTION_QUP11          1       /* pinctrl-kona.c: GPIO 60 and 61, the function after gpio */

static vm_offset_t pins_i2c;            /* pages of GPIO 60 and 61 */
static vm_offset_t pins_rst;            /* pages of GPIO 129 and 130 */

static vm_offset_t
pin(unsigned n)
{
	return n < PIN_INT ? pins_i2c + 0x1000UL * (n - PIN_SDA) : pins_rst + 0x1000UL * (n - PIN_INT);
}

static void
show_pins(void)
{
	static const struct {
		unsigned pin;
		const char *what;
	} pins[] = { { PIN_SDA, "I2C data" }, { PIN_SCL, "I2C clock" }, { PIN_RESET, "reset" }, { PIN_INT, "interrupt" } };

	for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
		uint32_t cfg, io;

		say("apollo haptic: about to read gpio%u's configuration at %#lx\n", pins[i].pin,
		    TLMM_SOUTH_PHYS + 0x1000UL * pins[i].pin);
		cfg = rd(pin(pins[i].pin));
		io = rd(pin(pins[i].pin) + 4);

		say("apollo haptic: gpio%u (%s): function %u, pull %u, %u mA, %s, reads %u\n", pins[i].pin, pins[i].what,
		    (cfg >> 2) & 15, cfg & 3, (((cfg >> 6) & 7) + 1) * 2, (cfg & (1u << 9)) ? (io & 2 ? "drives high" : "drives low") :
		    "input", io & 1);
	}
}

/* the reset pin as an output of the GPIO function, driving `high' (2 mA, no pull) */
static void
reset_pin(bool high)
{
	wr(pin(PIN_RESET) + 4, high ? 2 : 0);
	wr(pin(PIN_RESET), 1u << 9);
	released = high;
}

static bool
reset_is_high(void)
{
	uint32_t cfg = rd(pin(PIN_RESET)), io = rd(pin(PIN_RESET) + 4);

	return (cfg & (1u << 9)) && (io & 2);
}

/* ---- clocks (GCC, kona; the second QUPv3 wrapper's sixth serial engine) -------------------------------------- */

#define GCC_PHYS                0x00100000UL
#define GCC_SIZE                0x60000UL
#define GCC_VOTE                0x52008         /* gcc-kona.c: enable_reg of all the wrapper 1 clocks below */
#define VOTE_QUP1_CORE_2X       (1u << 18)      /* gcc_qupv3_wrap1_core_2x_clk: halt 0x23140 */
#define VOTE_QUP1_CORE          (1u << 19)      /* gcc_qupv3_wrap1_core_clk: halt 0x23138 */
#define VOTE_QUP1_M_AHB         (1u << 20)      /* gcc_qupv3_wrap_1_m_ahb_clk: halt 0x18004 */
#define VOTE_QUP1_S_AHB         (1u << 21)      /* gcc_qupv3_wrap_1_s_ahb_clk: halt 0x18008 (hwcg bit 1) */
#define VOTE_QUP1_S3            (1u << 25)      /* gcc_qupv3_wrap1_s3_clk: halt 0x1839c */
#define QUP1_CORE_2X_CBCR       0x23140
#define QUP1_CORE_CBCR          0x23138
#define QUP1_M_AHB_CBCR         0x18004
#define QUP1_S_AHB_CBCR         0x18008
#define QUP1_S3_CBCR            0x1839c
#define QUP1_S3_RCG             0x183a0         /* gcc_qupv3_wrap1_s3_clk_src cmd_rcgr; cfg +4; DFS: +0x14, levels at +0x1c, +0x5c, +0x9c */
/*
 * (The first version enabled wrapper 1's serial engine *5* -- 0x185fc, vote bit 27 -- for serial engine 11 = 0xa8c000, the
 * fourth of the wrapper's engines, S3: its own clock stayed off. The phone's DT agrees with S3: i2c@a8c000's clocks are
 * GCC 112 = GCC_QUPV3_WRAP1_S3_CLK, 134 = WRAP_1_M_AHB, 135 = WRAP_1_S_AHB, in the kona clock header of apollo-q-oss.)
 */
#define WRAPPER1_PHYS           0x00ac0000UL    /* qupv3_1_geni_se@ac0000 */
#define WRAPPER_HW_VER          0x04            /* qcom-geni-se.c: QUP_HW_VER_REG */
#define CLOCK_OFF               (1u << 31)
#define CLOCK_HW_CTL            (1u << 1)
#define XO_HZ                   19200000u

static vm_offset_t gcc, wrapper;

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

/* a branch's CLK_OFF bit (a branch the hardware gates, by its HW_CTL bit, is idle then, not off) */
static bool
branch_on(uint32_t cbcr)
{
	uint32_t v = rd(gcc + cbcr);

	return !(v & CLOCK_OFF) || (cbcr == QUP1_S_AHB_CBCR && (v & CLOCK_HW_CTL));
}

/* the wrapper's bus and core clocks (m_ahb, s_ahb, core, core_2x): voted, then each read back (up to 10 ms) */
static int
bus_clocks_on(void)
{
	apollo_gcc_vote(gcc + GCC_VOTE, VOTE_QUP1_CORE_2X | VOTE_QUP1_CORE | VOTE_QUP1_M_AHB | VOTE_QUP1_S_AHB);    /* (locked: shared register) */
	for (int i = 0; i < 1000; i++) {
		if (branch_on(QUP1_CORE_2X_CBCR) && branch_on(QUP1_CORE_CBCR) && branch_on(QUP1_M_AHB_CBCR) && branch_on(QUP1_S_AHB_CBCR)) {
			return 0;
		}
		sleep_us(10);
	}
	say("apollo haptic: the wrapper's clocks stay off: core_2x %#x core %#x m_ahb %#x s_ahb %#x (vote reads %#x)\n",
	    rd(gcc + QUP1_CORE_2X_CBCR), rd(gcc + QUP1_CORE_CBCR), rd(gcc + QUP1_M_AHB_CBCR), rd(gcc + QUP1_S_AHB_CBCR),
	    rd(gcc + GCC_VOTE));
	return -1;
}

/* the serial engine's clock: XO's 19.2 MHz, the rate i2c-qcom-geni.c's table is for; which preset gives it */
static int
se_clock_on(uint32_t *select)
{
	*select = 0;
	if (rd(gcc + QUP1_S3_RCG + 0x14) & 1) {
		int found = -1;

		for (uint32_t lvl = 0; lvl < 8 && found < 0; lvl++) {
			if (rcg_rate(rd(gcc + QUP1_S3_RCG + 0x1c + 4 * lvl), rd(gcc + QUP1_S3_RCG + 0x5c + 4 * lvl),
			    rd(gcc + QUP1_S3_RCG + 0x9c + 4 * lvl)) == XO_HZ) {
				found = (int)lvl;
			}
		}
		if (found < 0) {
			say("apollo haptic: serial engine 11 has no 19.2 MHz preset\n");
			return -1;
		}
		*select = (uint32_t)found;
	} else {
		wr(gcc + QUP1_S3_RCG + 4, (rd(gcc + QUP1_S3_RCG + 4) & ~0x371fu) | 1);
		wr(gcc + QUP1_S3_RCG, rd(gcc + QUP1_S3_RCG) | 1);
		for (int i = 0; i < 500 && (rd(gcc + QUP1_S3_RCG) & 1); i++) {
			sleep_us(10);
		}
		if (rd(gcc + QUP1_S3_RCG) & 1) {
			say("apollo haptic: the serial engine's clock does not take its configuration (%#x)\n", rd(gcc + QUP1_S3_RCG));
			return -1;
		}
	}
	apollo_gcc_vote(gcc + GCC_VOTE, VOTE_QUP1_S3);
	for (int i = 0; i < 1000; i++) {
		if (branch_on(QUP1_S3_CBCR)) {
			return 0;
		}
		sleep_us(10);
	}
	say("apollo haptic: serial engine 11's clock stays off: %#x\n", rd(gcc + QUP1_S3_CBCR));
	return -1;
}

/* ---- I2C through the serial engine (i2c-qcom-geni.c; as in hakios.apollo_cs35l41.c) ----------------------------------- */

#define SE_PHYS                 0x00a8c000UL    /* the sixth serial engine of the second QUPv3 wrapper */
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
		return "serial engine 11 has no I2C firmware";
	}
	if (rd(se + GENI_IF_DISABLE_RO) & 1) {
		return "serial engine 11's FIFO interface is disabled";
	}
	fifo_depth = rd(se + SE_HW_PARAM_0) >> 16 & 0x3f;
	if (fifo_depth < 4) {
		return "serial engine 11 reports no FIFO";
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
	wr(se + GENI_TX_PACKING_CFG0, 0x0007f8fe);      /* bytes, the first in the low byte of a word */
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
	for (i = 0; i < 8000; i++) {                    /* 40 ms at most: 33 bytes take 0.8 ms at 400 kHz */
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
		say("apollo haptic: I2C %s of %u bytes at %#x failed: status %#x%s (sent %u, received %u)\n", tx ? "write" : "read",
		    length, address, status, (status & M_NACK) ? ", NOT ACKNOWLEDGED" : "", sent, received);
	}
	i2c_recover();
	return false;
}

/* ---- the chip's registers: 8 bits, the address counts up (but for the RAM's data port) ---------------------------- */

#define AW_ADDRESS              0x5a

static bool
regs_read(uint8_t reg, uint8_t *values, unsigned count)
{
	return i2c_xfer(AW_ADDRESS, &reg, NULL, 1, false) && i2c_xfer(AW_ADDRESS, NULL, values, count, true);
}

static bool
regs_write(uint8_t reg, const uint8_t *values, unsigned count)
{
	uint8_t buffer[40];

	if (count > sizeof(buffer) - 1) {
		return false;
	}
	buffer[0] = reg;
	memcpy(buffer + 1, values, count);
	return i2c_xfer(AW_ADDRESS, buffer, NULL, count + 1, true);
}

/* ---- the chip (aw8697_reg.h) ------------------------------------------------------------------------------- */

#define AW_ID                   0x00            /* read: 0x97; write 0xaa: a soft reset */
#define AW_SYSST                0x01
#define AW_SYSINT               0x02            /* read: cleared; [7] boost, [6] over-voltage, [5] UVLO, [2] OC, [1] OT */
#define AW_SYSINTM              0x03            /* [5] UVLO masked */
#define AW_SYSCTRL              0x04            /* [5] RAMINIT, [3:2] play mode (0 RAM), [1] boost (0 bypass), [0] standby */
#define AW_GO                   0x05
#define AW_WAVSEQ1              0x07
#define AW_WAVSEQ2              0x08
#define AW_WAVLOOP1             0x0f            /* [7:4] how often the first is repeated, 15: forever */
#define AW_PWMPRC               0x2d            /* [7] the motor's protection */
#define AW_PWMDBG               0x2e            /* [6:5] the PWM rate: 2 is 24 kHz */
#define AW_BSTDBG1              0x31
#define AW_BSTDBG2              0x32
#define AW_BSTDBG3              0x33
#define AW_BSTCFG               0x34            /* [2:0] the boost's peak current: 5 is 3.5 A */
#define AW_ANADBG               0x35            /* [3:2] the over-current limit: 3 is 4.65 A */
#define AW_DATDBG               0x39            /* the gain */
#define AW_BSTDBG4              0x3a            /* [5:1] the boost's voltage */
#define AW_PRLVL                0x3e            /* [7] the protection */
#define AW_BASEADDRH            0x21
#define AW_BASEADDRL            0x22
#define AW_FIFO_AEH             0x23
#define AW_FIFO_AEL             0x24
#define AW_FIFO_AFH             0x25
#define AW_FIFO_AFL             0x26
#define AW_RAMADDRH             0x40
#define AW_RAMADDRL             0x41
#define AW_RAMDATA              0x42
#define AW_GLB_STATE            0x46            /* [3:0] 0: standby */
#define AW_BST_AUTO             0x47            /* [2] automatic boost */
#define AW_TSET                 0x4d
#define AW_R_SPARE              0x5d
#define AW_DETCTRL              0x5f            /* [5] protection: no action, [0] the offset diagnosis */
#define AW_ADCTEST              0x66            /* [6] vbat compensation in hardware */
#define AW_BEMF_VTHH_H          0x74
#define AW_CHIPID               0x97

#define SYSINT_FAULTS           0xe6
#define HAPTIC_MAX_MS           1000
#define DUTY_WINDOW_MS          2000            /* at most HAPTIC_MAX_MS of driving (taps and vibrations) in any window this long */
#define DUTY_SLOT_MS            100             /* the window kept as so many slots of driving time; one more than it takes, so */
#define DUTY_SLOTS              (DUTY_WINDOW_MS / DUTY_SLOT_MS + 1)     /* that the slots always cover the whole window */
#define ANDROID_GAIN            0x80            /* aw8697_haptic_effect_strength(): the strongest level */
#define BST_VOL_RAM             0x11            /* DT vib_bst_vol_ram */

static void
give_up(const char *why)
{
	broken = true;
	say("apollo haptic: %s: THE CHIP IS PUT INTO RESET (gpio%u low) and nothing more is done\n", why, PIN_RESET);
	if (pins_rst != 0) {
		reset_pin(false);
	}
}

static bool
get(uint8_t reg, uint8_t *value)
{
	if (!regs_read(reg, value, 1)) {
		say("apollo haptic: %#04x NOT READ\n", reg);
		return false;
	}
	return true;
}

/* One register written and read back; a value that reads back differently is said (some bits may not be the host's), not fatal. */
static bool
put(uint8_t reg, uint8_t value)
{
	uint8_t after = 0;

	if (broken) {
		return false;
	}
	if (!regs_write(reg, &value, 1)) {
		say("apollo haptic: %#04x <- %#04x NOT ACKNOWLEDGED\n", reg, value);
		give_up("a write was not acknowledged");
		return false;
	}
	if (!regs_read(reg, &after, 1)) {
		give_up("a register could not be read back");
		return false;
	}
	say("apollo haptic: %#04x <- %#04x, reads %#04x%s\n", reg, value, after, after == value ? "" : ": DIFFERENT");
	return true;
}

/* A register's bits `mask' set to `value' (the others as they are), as aw8697_i2c_write_bits(). */
static bool
upd(uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t old = 0;

	if (broken || !get(reg, &old)) {
		give_up("a register could not be read");
		return false;
	}
	return put(reg, (uint8_t)((old & ~mask) | (value & mask)));
}

/* The chip's interrupt flags (reading clears them): false, and the chip given up on, if one is a fault. */
static bool
healthy(const char *when)
{
	uint8_t flags = 0;

	if (!get(AW_SYSINT, &flags)) {
		give_up("the flags could not be read");
		return false;
	}
	/*
	 * Under-voltage alone is not a fault: Xiaomi's aw8697.c masks the UVLO interrupt in standby (aw8697_haptic_play_mode(),
	 * SYSINTM_UVLO_OFF) and only logs it otherwise ("chip uvlo int error"); the chip showed it right after its
	 * configuration, at rest (25 October).
	 */
	if (flags & 0x20) {
		say("apollo haptic: %s: the chip reports under-voltage (%#04x): told, not a fault (as Xiaomi's driver)\n", when, flags);
	}
	/*
	 * Nor a boost error: Xiaomi's aw8697.c switches that interrupt off (aw8697_interrupt_setup(): SYSINTM BSTERR_OFF) and
	 * never looks at it; the chip raised it on the first tap of a run (25 October) and the motor was given up on for nothing.
	 */
	if (flags & 0x80) {
		static unsigned told;

		if (told++ < 4) {
			say("apollo haptic: %s: the chip reports a boost error (%#04x): told, not a fault (Xiaomi's driver masks it)\n", when,
			    flags);
		}
	}
	if (flags & SYSINT_FAULTS & ~0xa0) {
		say("apollo haptic: %s: the chip reports %#04x:%s%s%s%s%s\n", when, flags, flags & 0x80 ? " BOOST ERROR" : "",
		    flags & 0x40 ? " OVER-VOLTAGE" : "", flags & 0x20 ? " UNDER-VOLTAGE" : "", flags & 0x04 ? " OVER-CURRENT" : "",
		    flags & 0x02 ? " OVER-TEMPERATURE" : "");
		give_up("a fault");
		return false;
	}
	return true;
}

static bool
chip_answers(void)
{
	uint8_t id = 0;

	if (!regs_read(AW_ID, &id, 1)) {
		say("apollo haptic: nobody answers at %#x on the bus of gpio%u/%u\n", AW_ADDRESS, PIN_SDA, PIN_SCL);
		return false;
	}
	say("apollo haptic: the chip at %#x says %#04x%s\n", AW_ADDRESS, id, id == AW_CHIPID ? ", an AW8697" : ": NOT THE AW8697 EXPECTED");
	return id == AW_CHIPID;
}

/* the registers as they are (not the interrupt flags, the RTP and RAM data ports: reading them has effects), nothing written */
static void
show_registers(void)
{
	for (unsigned base = 0; base < 0x80; base += 8) {
		uint8_t values[8];
		bool ok = true;

		for (unsigned i = 0; i < 8; i++) {
			unsigned reg = base + i;

			values[i] = 0;
			if (reg == AW_SYSINT || reg == 0x06 || reg == AW_RAMDATA) {
				continue;
			}
			ok &= get((uint8_t)reg, &values[i]);
		}
		say("apollo haptic: %#04x: %02x %02x %02x %02x %02x %02x %02x %02x\n", base, values[0], values[1], values[2], values[3],
		    values[4], values[5], values[6], values[7]);
		if (!ok) {
			return;
		}
	}
}

/* aw8697_hw_reset(): low 5 ms, high 5 ms */
static void
hardware_reset(void)
{
	/* aw8697_hw_reset() with pinctrl: aw8697_reset_active (output high), 5 ms, aw8697_reset_reset (output low), 5 ms, active */
	say("apollo haptic: the reset is released: gpio%u high, low for 5 ms, then high\n", PIN_RESET);
	wr(pin(PIN_INT), 3);                    /* aw8697_gpio_int: gpio, 2 mA, pull-up, input */
	reset_pin(true);
	wait_ms(5);
	reset_pin(false);
	wait_ms(5);
	reset_pin(true);
	wait_ms(5);
}

/* aw8697_haptic_play_mode(STANDBY) */
static bool
to_standby(void)
{
	return upd(AW_SYSINTM, 0x20, 0x20) && upd(AW_SYSCTRL, 0x01, 0x01);
}

/* aw8697_haptic_init(): Android's configuration */
static bool
configure(void)
{
	static const uint8_t bemf[4] = { 0x00, 0x08, 0x03, 0xf8 };     /* DT vib_bemf_config */
	uint8_t id = 0;

	say("apollo haptic: Android's configuration is written\n");
	if (!regs_write(AW_ID, (const uint8_t[]){ 0xaa }, 1)) {         /* aw8697_haptic_softreset() */
		give_up("the soft reset was not acknowledged");
		return false;
	}
	wait_ms(2);
	if (!get(AW_ID, &id) || id != AW_CHIPID) {
		give_up("the chip does not answer after its soft reset");
		return false;
	}
	if (!to_standby() || !upd(AW_PWMDBG, 0x60, 2 << 5) ||
	    !put(AW_BSTDBG1, 0x30) || !put(AW_BSTDBG2, 0xeb) || !put(AW_BSTDBG3, 0xd4) ||    /* DT vib_bstdbg */
	    !put(AW_TSET, 0x12) || !put(AW_R_SPARE, 0x68) ||                                 /* DT vib_tset, vib_r_spare */
	    !upd(AW_ANADBG, 0x0c, 3 << 2) || !upd(AW_BSTCFG, 0x07, 5) ||
	    !upd(AW_DETCTRL, 0x20, 0x20) || !upd(AW_PWMPRC, 0x80, 0) || !upd(AW_PRLVL, 0x80, 0) ||  /* the motor protection off */
	    !upd(AW_BST_AUTO, 0x04, 0)) {
		return false;
	}
	/* aw8697_haptic_offset_calibration(): RAMINIT on, the diagnosis run, RAMINIT off */
	if (!upd(AW_SYSCTRL, 0x20, 0x20) || !upd(AW_DETCTRL, 0x01, 0x01)) {
		return false;
	}
	for (unsigned i = 0; i < 500; i++) {
		uint8_t value = 1;

		if (!get(AW_DETCTRL, &value) || !(value & 1)) {
			break;
		}
		wait_ms(1);
	}
	if (!upd(AW_SYSCTRL, 0x20, 0) || !upd(AW_ADCTEST, 0x40, 0x40)) {
		return false;
	}
	for (unsigned i = 0; i < 4; i++) {
		if (!put((uint8_t)(AW_BEMF_VTHH_H + i), bemf[i])) {
			return false;
		}
	}
	return upd(AW_BSTDBG4, 0x3e, BST_VOL_RAM << 1) && healthy("after the configuration");
}

static uint32_t states_seen;           /* the GLB_STATE states seen by the last wait, a bit each */

/* until the chip says it is in standby (GLB_STATE 0), at most `ms' */
static bool
wait_standby(unsigned ms)
{
	states_seen = 0;
	for (unsigned waited = 0; waited <= ms; waited += 2) {
		uint8_t state = 0xff;

		if (!get(AW_GLB_STATE, &state)) {
			return false;
		}
		states_seen |= 1u << (state & 0x0f);
		if ((state & 0x0f) == 0) {
			return true;
		}
		wait_ms(2);
	}
	return false;
}

/*
 * Android's waveforms into the chip's RAM (aw8697_ram_loaded, aw8697_container_update): the file is a checksum (16
 * bits, of all that follows), the address the RAM's contents begin at, and those contents from byte 4 on -- a
 * version byte, the start and end address of each waveform, the waveforms. Read back the same way and compared.
 */
static bool
load_waveforms(void)
{
	uint64_t size = 0;
	const uint8_t *file = hakios_image_file("fw", "aw8697_haptic.bin", &size, NULL);
	uint16_t sum = 0;
	uint32_t base, length, ae, af;
	uint8_t address[2], back[32];
	bool ok = false;

	if (file == NULL || size < 8) {
		say("apollo haptic: no aw8697_haptic.bin in fw/ of the small image\n");
		return false;
	}
	for (uint64_t i = 2; i < size; i++) {
		sum = (uint16_t)(sum + file[i]);
	}
	if (sum != (uint16_t)(file[0] << 8 | file[1])) {
		say("apollo haptic: aw8697_haptic.bin does not add up (%#x, it says %#x): not loaded\n", sum, file[0] << 8 | file[1]);
		goto out;
	}
	base = (uint32_t)(file[2] << 8 | file[3]);
	length = (uint32_t)size - 4;
	ae = base >> 2;
	af = base - (base >> 2);
	say("apollo haptic: Android's waveforms: %u bytes from %#x on\n", length, base);
	if (!wait_standby(250) || !upd(AW_SYSCTRL, 0x20, 0x20)) {       /* RAMINIT */
		give_up("the RAM could not be opened");
		goto out;
	}
	address[0] = file[2];
	address[1] = file[3];
	if (!put(AW_BASEADDRH, address[0]) || !put(AW_BASEADDRL, address[1]) ||
	    !put(AW_FIFO_AEH, (uint8_t)(ae >> 8)) || !put(AW_FIFO_AEL, (uint8_t)ae) ||
	    !put(AW_FIFO_AFH, (uint8_t)(af >> 8)) || !put(AW_FIFO_AFL, (uint8_t)af) ||
	    !regs_write(AW_RAMADDRH, address, 2)) {
		give_up("the RAM's address could not be set");
		goto out;
	}
	for (uint32_t at = 0; at < length; at += 32) {
		unsigned n = length - at < 32 ? length - at : 32;

		if (!regs_write(AW_RAMDATA, file + 4 + at, n)) {
			give_up("the RAM could not be written");
			goto out;
		}
	}
	if (!regs_write(AW_RAMADDRH, address, 2)) {
		give_up("the RAM's address could not be set again");
		goto out;
	}
	for (uint32_t at = 0; at < length; at += sizeof(back)) {
		unsigned n = length - at < sizeof(back) ? length - at : sizeof(back);

		if (!regs_read(AW_RAMDATA, back, n) || memcmp(back, file + 4 + at, n) != 0) {
			say("apollo haptic: the RAM reads back differently at %#x\n", base + at);
			give_up("the waveforms did not load");
			goto out;
		}
	}
	if (!upd(AW_SYSCTRL, 0x20, 0)) {
		goto out;
	}
	say("apollo haptic: Android's waveforms are in the chip's RAM and read back as written\n");
	ok = healthy("after the waveforms");
out:
	/*
	 * Not freed: hakios_image_file() can hand back the file where it lies in the image itself, not a copy -- freeing
	 * that panicked the kernel ("kfree: addr ..., size 3861 not found in any zone", 25 October; and with the Wi-Fi's
	 * shutdown hanging on the way out, it looked like a freeze). 4 KB kept for the run is nothing.
	 */
	return ok;
}

/* GO off, the chip awaited until it is in standby, then put there (aw8697_haptic_stop()) */
static bool
halt(void)
{
	/* (one that cannot be told -- given up on already, or not answering -- is stopped by its reset line instead) */
	if (broken || !regs_write(AW_GO, (const uint8_t[]){ 0 }, 1) || !wait_standby(200)) {
		give_up(broken ? "it cannot be told to stop" : "it does not stop");
		return false;
	}
	return to_standby();            /* (a failure there has put it into reset already: put(), upd()) */
}

/* A waveform started without waiting for it: once, or (`forever') repeated until halted. */
static bool
start_waveform(unsigned waveform, bool forever, uint8_t gain)
{
	/*
	 * The first time iOS played (25 October) the kernel hung on the spot: before each waveform, the serial engine's clocks
	 * are read back (the GCC's registers are always safe to read), so that an engine whose clocks someone has turned
	 * off is never touched -- the bus would hang on it.
	 */
	if (!branch_on(QUP1_CORE_2X_CBCR) || !branch_on(QUP1_CORE_CBCR) || !branch_on(QUP1_M_AHB_CBCR) || !branch_on(QUP1_S_AHB_CBCR) ||
	    !branch_on(QUP1_S3_CBCR)) {
		say("apollo haptic: THE SERIAL ENGINE'S CLOCKS ARE OFF before a waveform (core_2x %#x core %#x m_ahb %#x s_ahb %#x s3 %#x, vote "
		    "%#x): the engine is not touched\n", rd(gcc + QUP1_CORE_2X_CBCR), rd(gcc + QUP1_CORE_CBCR), rd(gcc + QUP1_M_AHB_CBCR),
		    rd(gcc + QUP1_S_AHB_CBCR), rd(gcc + QUP1_S3_CBCR), rd(gcc + GCC_VOTE));
		give_up("the serial engine's clocks are off");  /* (the reset line only: the engine is not touched) */
		return false;
	}
	{
		static unsigned told;

		if (told++ < 3) {
			say("apollo haptic: waveform %u (%s, gain %#04x): clocks on, the chip is told\n", waveform, forever ? "repeated" : "once", gain);
		}
	}
	if (!put(AW_WAVSEQ1, (uint8_t)waveform) || !put(AW_WAVSEQ2, 0) || !put(AW_WAVLOOP1, forever ? 0xf0 : 0x00) ||
	    !upd(AW_SYSCTRL, 0x0f, forever ? 0x00 : 0x02) ||                     /* RAM mode, active, boost (bypass for a loop) */
	    !get(AW_SYSINT, &(uint8_t){ 0 }) || !upd(AW_SYSINTM, 0x20, 0) ||    /* (the flags cleared, UVLO watched) */
	    !put(AW_DATDBG, gain)) {
		return false;
	}
	if (!regs_write(AW_GO, (const uint8_t[]){ 1 }, 1)) {
		give_up("the start was not acknowledged");
		return false;
	}
	return true;
}

/* One waveform played once (Android's effect `effect' - 1), stopped at HAPTIC_MAX_MS at the latest. */
static bool
play_effect(unsigned effect)
{
	uint64_t began = mach_absolute_time(), ended;
	bool done;

	say("apollo haptic: waveform %u is played once, at Android's gain %#x\n", effect, ANDROID_GAIN);
	if (!wait_standby(250) || !start_waveform(effect, false, ANDROID_GAIN)) {
		return false;
	}
	wait_ms(2);
	done = wait_standby(HAPTIC_MAX_MS);
	absolutetime_to_nanoseconds(mach_absolute_time() - began, &ended);
	if (!done) {
		say("apollo haptic: STILL PLAYING after %u ms: stopped\n", HAPTIC_MAX_MS);
	}
	if (!halt()) {
		return false;
	}
	say("apollo haptic: the waveform has played: back in standby after %llu ms (states seen meanwhile: %#x, a bit each; 0 is "
	    "standby)\n", ended / NSEC_PER_MSEC, states_seen);
	return healthy("after playing");
}

/* ---- apollo_haptic=5: the motor for iOS (hakios.haptic.c's service) ------------------------------------------- */

#define ACTIVE_US               30000           /* a write that was not silence counts for this long */
#define SILENT_BELOW            256             /* of 32768: samples no larger are silence */
#define TAP_US                  60000           /* longer than this, and it is a vibration, not a tap */
#define SETTLE_US               8000            /* how long iOS's waveform is listened to before a tap: its loudness */
#define FOLLOW_US               20000           /* how often a vibration's strength follows iOS's */

#define TOLD_EVENTS             24              /* the first so many taps and vibrations are each said */

static volatile uint64_t heard_us;              /* when the last write that was not silence came */
static volatile uint64_t quiet_us;              /* when the last write that was silence came */
static volatile uint64_t onset_us;              /* when iOS's last waveform began: the first write not silence after a pause */
static volatile uint32_t heard_peak;            /* the largest sample of that waveform, or since the service last looked */
static uint64_t written_frames;
static unsigned taps, loops, capped;            /* what has been played, for the run's end */

static uint64_t
now_us(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / NSEC_PER_USEC;
}

/* From the audio server's I/O thread: never waits. */
static int
haptic_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	int16_t samples[256];
	int error = 0;

	while (uio_resid(uio) >= 2) {
		user_ssize_t bytes = uio_resid(uio);
		uint32_t peak = 0;

		if (bytes > (user_ssize_t)sizeof(samples)) {
			bytes = sizeof(samples);
		}
		bytes &= ~(user_ssize_t)1;
		if ((error = uiomove((caddr_t)samples, (int)bytes, uio)) != 0) {
			break;
		}
		for (unsigned i = 0; i < bytes / 2; i++) {
			uint32_t v = samples[i] < 0 ? (uint32_t)-(int32_t)samples[i] : (uint32_t)samples[i];

			peak = v > peak ? v : peak;
		}
		written_frames += (uint64_t)bytes / 2;
		if (peak > SILENT_BELOW) {
			uint64_t now = now_us();

			if (heard_us == 0 || quiet_us > heard_us || now - heard_us >= ACTIVE_US) {
				onset_us = now;
				heard_peak = 0;
			}
			heard_us = now;
			if (peak > heard_peak) {
				heard_peak = peak;
			}
		} else {
			quiet_us = now_us();
		}
	}
	return error;
}

static int
haptic_open(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	return 0;
}

static const struct cdevsw haptic_cdevsw = {
	.d_open = haptic_open, .d_close = haptic_open, .d_read = eno_rdwrt, .d_write = haptic_write,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

/* the chip's state (GLB_STATE: 0 standby), 0xff if it cannot be read */
static unsigned
chip_state(void)
{
	uint8_t state = 0xff;

	return get(AW_GLB_STATE, &state) ? state & 0x0f : 0xff;
}

/*
 * How strong to play what iOS plays (hakios.haptic.c's reasoning): Android's strongest gain, 0x80, from a third of
 * full scale up, and below that down to five eighths of it (0x50, Android's medium level) for the softest.
 */
static uint8_t
gain_for(uint32_t peak)
{
	uint32_t top = ANDROID_GAIN, low = top * 5 / 8, third = 32767 / 3;

	return (uint8_t)(peak >= third ? top : low + (top - low) * peak / third);
}

static void
serve(void)
{
	enum { IDLE, TAP, LOOP, REST } state = IDLE;
	uint64_t since = 0, rest_until = 0, last_look = 0, tap_onset = 0, followed = 0;
	uint8_t playing_gain = 0;
	unsigned follows = 0;                   /* how often the vibration playing now has changed strength */
	uint32_t driven[DUTY_SLOTS] = { 0 };    /* us the motor was driven (a tap or a vibration under way) per DUTY_SLOT_MS */
	uint64_t duty_slot = now_us() / (DUTY_SLOT_MS * 1000), duty_last = now_us();
	unsigned duty_cuts = 0;
	int major = cdevsw_add(-1, &haptic_cdevsw);

	if (major < 0 || devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "apollo-haptic") == NULL) {
		say("apollo haptic: /dev/apollo-haptic cannot be made\n");
		return;
	}
	say("apollo haptic: THE MOTOR PLAYS WHAT IOS PLAYS ON ITS ACTUATOR (/dev/apollo-haptic) from here on, until the run ends\n");
	while (!broken) {
		uint64_t now = now_us();
		bool active = heard_us != 0 && now - heard_us < ACTIVE_US;
		uint32_t peak = heard_peak;
		uint8_t gain = gain_for(peak);
		uint64_t slot = now / (DUTY_SLOT_MS * 1000), window = 0;

		/*
		 * The duty budget, across taps and vibrations alike: the time since the last look counts as driving if a tap or a
		 * vibration was under way (a tap's whole TAP state, more than its waveform: on the safe side); the slots older
		 * than the window are emptied. Over HAPTIC_MAX_MS in the window: stopped, a rest of HAPTIC_MAX_MS, and nothing
		 * starts again until the window has room (a single cap per vibration does not stop a stream of short taps).
		 */
		for (uint64_t s = duty_slot + 1; s <= slot && s <= duty_slot + DUTY_SLOTS; s++) {
			driven[s % DUTY_SLOTS] = 0;
		}
		duty_slot = slot;
		if (state == TAP || state == LOOP) {
			uint64_t spent = now - duty_last;

			driven[slot % DUTY_SLOTS] += spent > DUTY_WINDOW_MS * 1000ull ? DUTY_WINDOW_MS * 1000u : (uint32_t)spent;
		}
		duty_last = now;
		for (unsigned i = 0; i < DUTY_SLOTS; i++) {
			window += driven[i];
		}
		if ((state == TAP || state == LOOP) && window >= HAPTIC_MAX_MS * 1000ull) {
			if (!halt()) {
				break;
			}
			capped++;
			rest_until = now + HAPTIC_MAX_MS * 1000ull;
			if (duty_cuts++ < TOLD_EVENTS) {
				say("apollo haptic: %llu ms of driving in the last %u ms: the motor is STOPPED and rests %u ms (%u taps, %u "
				    "vibrations so far)\n", window / 1000, DUTY_WINDOW_MS, HAPTIC_MAX_MS, taps, loops);
			}
			state = REST;
			wait_ms(4);
			continue;
		}

		switch (state) {
		case IDLE:
		case TAP:
			/*
			 * A waveform of iOS's that has begun (after a pause: a tap of a series is a tap of its own) is
			 * a tap -- once its first milliseconds are heard: a waveform begins softly, and they say how
			 * loud it is; one that goes on without a pause, a vibration.
			 */
			if (active && onset_us != tap_onset && now - onset_us >= SETTLE_US) {
				uint64_t began = onset_us;

				if (window >= HAPTIC_MAX_MS * 1000ull) {
					tap_onset = began;      /* (no room in the duty window: this tap is left out, not played late) */
					break;
				}
				if (state == TAP && !wait_standby(0) && !halt()) {
					break;
				}
				if (!start_waveform(1, false, gain ? gain : 1)) {
					break;
				}
				state = TAP;
				since = now;
				tap_onset = began;
				taps++;
				if (taps + loops <= TOLD_EVENTS) {
					wait_ms(4);
					say("apollo haptic: tap %u: iOS's waveform peaks at %u of 32768: Android's waveform 1 at gain %#04x; the "
					    "chip's state %u\n", taps, peak, gain ? gain : 1, chip_state());
				}
				break;
			}
			if (state == IDLE) {
				break;
			}
			if (active && quiet_us < tap_onset && now - since > TAP_US) {
				if (halt() && start_waveform(11, true, gain ? gain : 1)) {
					state = LOOP;
					since = followed = now;
					playing_gain = gain ? gain : 1;
					follows = 0;
					loops++;
					heard_peak = 0;
					if (taps + loops <= TOLD_EVENTS) {
						wait_ms(4);
						say("apollo haptic: vibration %u: iOS goes on playing (peak %u): Android's waveform 11 over and over at "
						    "gain %#04x, %u ms at most; the chip's state %u\n", loops, peak, gain ? gain : 1,
						    HAPTIC_MAX_MS, chip_state());
					}
				}
			} else if (!active && now - since > TAP_US) {
				state = halt() ? IDLE : state;          /* (the tap is over: the chip back into standby) */
			}
			break;
		case LOOP:
			/* as strong as iOS's waveform is now (Android's gain at full scale, never more) */
			if (active && now - followed >= FOLLOW_US) {
				followed = now;
				heard_peak = 0;
				if (peak != 0 && (gain > playing_gain + 8 || gain + 8 < playing_gain)) {
					if (!put(AW_DATDBG, gain)) {
						break;
					}
					playing_gain = gain;
					follows++;
				}
			}
			if (!active || now - since > HAPTIC_MAX_MS * 1000ull) {
				bool cut = now - since > HAPTIC_MAX_MS * 1000ull;

				if (cut) {
					capped++;
					rest_until = now + HAPTIC_MAX_MS * 1000ull;
				}
				if (halt()) {
					state = rest_until > now ? REST : IDLE;
					if (taps + loops <= TOLD_EVENTS + 1) {
						say("apollo haptic: vibration %u: %s after %llu ms (its strength changed %u times, last to gain %#04x); "
						    "the chip is in standby\n", loops, cut ? "CUT at the limit" : "iOS stopped, and so does the motor",
						    (now - since) / 1000, follows, playing_gain);
					}
				}
			}
			break;
		case REST:
			if (now >= rest_until) {        /* (and what starts next waits for room in the duty window) */
				state = IDLE;
			}
			break;
		}
		if (state != IDLE && state != REST && now - last_look > 50000) {
			last_look = now;
			if (!healthy("while playing")) {
				break;
			}
		}
		wait_ms(4);
	}
	say("apollo haptic: the motor is given up on (%u taps and %u vibrations played, %u cut at %u ms)\n", taps, loops, capped,
	    HAPTIC_MAX_MS);
}

static bool
prepare(void)
{
	uint32_t select = 0;
	const char *problem;

	pins_i2c = ml_io_map(TLMM_SOUTH_PHYS + 0x1000UL * PIN_SDA, 0x2000);
	pins_rst = ml_io_map(TLMM_SOUTH_PHYS + 0x1000UL * PIN_INT, 0x2000);
	gcc = ml_io_map(GCC_PHYS, GCC_SIZE);
	se = ml_io_map(SE_PHYS, SE_SIZE);
	wrapper = ml_io_map(WRAPPER1_PHYS, 0x1000);
	if (!pins_i2c || !pins_rst || !gcc || !se || !wrapper) {
		say("apollo haptic: cannot map the registers\n");
		return false;
	}
	show_pins();
	/*
	 * Wrapper 1's core clock is RPMh's on this SoC: Linux' geni-se driver votes the interconnect path "qup-core"
	 * (qup1_core_master -> qup1_core_slave), and in sm8250.c's icc tables that is the BCM "QUP0": it holds the core
	 * masters of all three wrappers (bcm_qup0.nodes = qup0/1/2_core_master). cmd-db has no "QUP1" on this phone.
	 */
	say("apollo haptic: about to vote for the QUP core clocks in RPMh (BCM \"QUP0\", which covers wrapper 1 too)\n");
	if (!apollo_rpmh_bcm_vote("QUP0", 1, 1)) {
		say("apollo haptic: RPMh does not take the \"QUP0\" vote: not touching serial engine 11\n");
		apollo_cmd_db_dump();
		return false;
	}
	say("apollo haptic: about to turn on wrapper 1's bus and core clocks (gcc vote %#x, bits 18-21), then read each back\n", GCC_VOTE);
	if (bus_clocks_on() != 0) {
		return false;
	}
	say("apollo haptic: wrapper 1's clocks are on (core_2x %#x core %#x m_ahb %#x s_ahb %#x): about to read its version at %#lx\n",
	    rd(gcc + QUP1_CORE_2X_CBCR), rd(gcc + QUP1_CORE_CBCR), rd(gcc + QUP1_M_AHB_CBCR), rd(gcc + QUP1_S_AHB_CBCR),
	    WRAPPER1_PHYS + WRAPPER_HW_VER);
	say("apollo haptic: wrapper 1's version is %#x\n", rd(wrapper + WRAPPER_HW_VER));
	say("apollo haptic: about to turn on serial engine 11's clock (gcc_qupv3_wrap1_s3_clk, vote bit 25, cbcr %#x)\n", QUP1_S3_CBCR);
	if (se_clock_on(&select) != 0) {
		return false;
	}
	say("apollo haptic: serial engine 11's clock is on (cbcr %#x)\n", rd(gcc + QUP1_S3_CBCR));
	say("apollo haptic: about to give gpio%u and gpio%u to qup11\n", PIN_SDA, PIN_SCL);
	wr(pin(PIN_SDA), FUNCTION_QUP11 << 2 | (2 / 2 - 1) << 6);     /* qup11, 2 mA, no pull */
	wr(pin(PIN_SCL), FUNCTION_QUP11 << 2 | (2 / 2 - 1) << 6);
	say("apollo haptic: about to read the serial engine at %#lx (its firmware revision), then set it up\n", SE_PHYS);
	problem = i2c_init(select);
	if (problem != NULL) {
		say("apollo haptic: %s\n", problem);
		return false;
	}
	prepared = true;
	return true;
}

static void
haptic_thread(__unused void *parameter, __unused wait_result_t result)
{
	delay_for_interval(start_after_ms, 1000 * NSEC_PER_USEC);
	bringing_up = true;
	say("apollo haptic: stage %u: %s\n", stage, stage == 1 ? "a look, reading only" : stage == 2 ? "the reset released, registers read" :
	    stage == 3 ? "the chip configured and its waveforms loaded, nothing played" : stage == 4 ? "one effect played" :
	    "the motor for iOS");
	if (!prepare()) {
		goto done;
	}
	if (stage >= 2 && !reset_is_high()) {
		hardware_reset();
	}
	if (!chip_answers()) {
		say("apollo haptic: (the reset pin %s)\n", reset_is_high() ? "drives high: the chip should answer" :
		    "does not drive high: the chip is held in reset or not powered");
		if (released) {
			give_up("the chip does not answer");
		}
	} else {
		if (stage >= 2) {
			show_registers();
		}
		if (stage >= 3 && configure() && load_waveforms() && to_standby() && stage == 4) {
			uint32_t effect = 1;

			PE_parse_boot_argn("apollo_haptic_effect", &effect, sizeof(effect));
			if (effect < 1 || effect > 10) {
				effect = 1;     /* (the short ones only) */
			}
			(void)play_effect(effect);
		} else if (stage == 5 && !broken) {
			bringing_up = false;
			serve();
		}
	}
done:
	say("apollo haptic: done\n");
	thread_terminate(current_thread());
}

void
apollo_haptic_init(void)
{
	thread_t thread;

	if (!PE_parse_boot_argn("apollo_haptic", &stage, sizeof(stage)) || stage == 0) {
		return;
	}
	if (stage > 5) {
		stage = 5;
	}
	PE_parse_boot_argn("apollo_haptic_pause", &pause_ms, sizeof(pause_ms));
	PE_parse_boot_argn("apollo_haptic_after", &start_after_ms, sizeof(start_after_ms));
	if (kernel_thread_start(haptic_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

/* At the run's end and on a panic's way out: the chip back into reset, if it was let out (no I2C, no lock, no sleep). */
void
apollo_haptic_quiesce(void)
{
	if (prepared && pins_rst != 0 && (released || reset_is_high())) {
		leaving = true;
		reset_pin(false);
		if (stage == 5) {
			say("apollo haptic: at the run's end: %u taps and %u vibrations played (%u cut at %u ms), from %llu frames iOS "
			    "wrote; the chip is held in reset\n", taps, loops, capped, HAPTIC_MAX_MS, written_frames);
		}
	}
}
