/*
 * The touch screen of the Xiaomi Mi 10T (apollo): a Novatek NT36675 ("j3s") in the display's panel, on the
 * QUPv3 serial engine 4, as a source of touches for /dev/hakios-touch (sys/hakios.input.h). Only with
 * boot-arg apollo_touch=1 on apollo (bsd/dev/hakios.apollo_input.c): every address, clock and pin below belongs to that phone.
 *
 * There is no clock, pin, regulator or SPI framework in this kernel, so the driver does all of it itself, in
 * the order the vendor's Linux drivers do:
 *   clocks   the serial engine's clocks, by vote in the global clock controller (kona GCC)
 *   pins     four lines to the serial engine, plus reset, interrupt and the chip's I/O supply
 *   SPI      the serial engine ("GENI") in FIFO mode, polled; or, if the engine cannot be used, the same
 *            lines driven by hand
 *   chip     Novatek: power the I/O supply on, reset the chip, then read the touch reports from its event
 *            buffer, through the Novatek's own read framing (the "set page" register and the address bits of
 *            the data byte), and keep reading them
 * No interrupt is used: a thread reads the report every poll interval.
 *
 * Register layouts: Linux drivers/spi/spi-geni-qcom.c, drivers/soc/qcom/{qcom-geni-se,rpmh-rsc,cmd-db}.c,
 * drivers/clk/qcom/gcc-kona.c, drivers/pinctrl/qcom/pinctrl-kona.c, and the phone's own device tree (the
 * /soc/spi@990000/touch_spi@0 serial engine, and /soc/ts_novatek for the chip itself).
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>           /* assert_wait, thread_block, thread_wakeup */
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_input.h>
#include <sys/hakios.input.h>          /* hakios_touch_post(): the queue backboardd reads, shared */
#include <sys/systm.h>

extern void hakios_tl(const char *format, ...) __printflike(1, 2);

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);

/*
 * Every line goes to the loader's log directly as well (as in hakios.buttons.c): the kernel's own
 * messages reach that log only when a run ends cleanly, and a phone that resets keeps only these.
 */
extern void loader_log_putc(char c);

static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[400];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	return printf("%s", line);
}
#define printf say

static vm_offset_t tlmm_north, tlmm_south_38, tlmm_south_69, gcc, se;

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

/* ---- pins (TLMM) -------------------------------------------------------------------------------- */

/* The QUPv3 wrappers of the SC chip give serial engine 4 its four lines at gpio 8, 9, 10 and 11 (miso, mosi, clk,
 * chip select); the chip's interrupt is gpio 39, its reset is gpio 38, and it takes its I/O supply on gpio 69. */
/* The TLMM window is 16 MiB, twice the kernel's whole io_submap (ml_io_map of it panics): only the pages of
 * the pins used are mapped, each pin's registers being one page of its tile. */
#define TLMM_PHYS               0x0f000000UL
#define TLMM_NORTH              0x00900000UL
#define TLMM_SOUTH              0x00500000UL
#define TLMM_PIN_PHYS(tile, n)  (TLMM_PHYS + (tile) + 0x1000UL * (n))
#define GPIO_CFG(n)             pin_base(n)
#define GPIO_IO(n)              (GPIO_CFG(n) + 4) /* bit 0 in, bit 1 out */
#define GPIO_INTR_CFG(n)        (GPIO_CFG(n) + 8) /* pinctrl-kona.c: intr_cfg_reg */
#define GPIO_INTR_STATUS(n)     (GPIO_CFG(n) + 0xc) /* bit 0: an edge was seen; written 0 to clear */
#define INTR_ENABLE             (1u << 0)        /* on to the interrupt controller: left off */
#define INTR_POLARITY_HIGH      (1u << 1)
#define INTR_DETECT_EDGE        (1u << 2)        /* [3:2] 1: edge */
#define INTR_DETECT_MASK        (3u << 2)
#define INTR_RAW_STATUS         (1u << 4)        /* the status latches even with the enable bit off */
#define PULL_NONE               0
#define PULL_UP                 3
#define PULL_DOWN              2

#define PIN_MISO                8
#define PIN_MOSI                9
#define PIN_CLK                 10
#define PIN_CS                  11
#define PIN_INT                 39
#define PIN_RESET               38
#define PIN_VDDIO               69
#define PIN_FUNCTION_SPI        1       /* "qup4" */

static vm_offset_t
pin_base(unsigned pin)
{
	if (pin >= PIN_MISO && pin <= PIN_CS) {
		return tlmm_north + 0x1000UL * (pin - PIN_MISO);
	}
	if (pin == PIN_RESET || pin == PIN_INT) {
		return tlmm_south_38 + 0x1000UL * (pin - PIN_RESET);
	}
	return tlmm_south_69;   /* PIN_VDDIO, the only other pin used */
}

static void
gpio_configure(unsigned pin, unsigned function, unsigned pull, unsigned ma, bool output)
{
	wr(GPIO_CFG(pin), pull | function << 2 | (ma / 2 - 1) << 6 | (output ? 1u << 9 : 0));
}

static void
gpio_set(unsigned pin, bool high)
{
	wr(GPIO_IO(pin), high ? 2 : 0);
}

static void
gpio_output(unsigned pin, bool high, unsigned pull, unsigned ma)
{
	gpio_set(pin, high);            /* the level first, so that the pin never drives the wrong one */
	gpio_configure(pin, 0, pull, ma, true);
}

/* ---- clocks (GCC, kona) -------------------------------------------------------------------------------- */

#define GCC_PHYS                0x00100000UL
#define GCC_SIZE                0x60000UL
#define GCC_VOTE                (gcc + 0x52008)         /* the application processor's votes */
/* spi@990000 is serial engine 4 of the first QUPv3 wrapper (qupv3_0_geni_se@9c0000): its clocks are wrap0's */
#define VOTE_QUP0_CORE_2X       (1u << 9)
#define VOTE_QUP0_CORE          (1u << 8)
#define VOTE_QUP0_M_AHB         (1u << 6)
#define VOTE_QUP0_S_AHB         (1u << 7)
#define VOTE_QUP0_SE4           (1u << 14)             /* serial engine 4 is the fifth of the first group */
#define GCC_QUP0_M_AHB_CBCR     (gcc + 0x17004)         /* bit 31: the clock is off */
#define GCC_QUP0_S_AHB_CBCR     (gcc + 0x17008)
#define GCC_QUP0_SE4_CBCR       (gcc + 0x174cc)
#define GCC_QUP0_SE4_CMD        (gcc + 0x174d0)         /* bit 0: take the new configuration */
#define GCC_QUP0_SE4_CFG        (gcc + 0x174d4)         /* [4:0] 2*divider-1, [10:8] source, [13:12] mode */
#define GCC_QUP0_SE4_DFSR       (gcc + 0x174e4)         /* bit 0: the engine chooses among preset rates itself */
#define GCC_QUP0_SE4_PERF_CFG(l) (gcc + 0x174ec + 4 * (l))
#define GCC_QUP0_SE4_PERF_M(l)  (gcc + 0x1752c + 4 * (l))
#define GCC_QUP0_SE4_PERF_N(l)  (gcc + 0x1756c + 4 * (l))
#define CLOCK_OFF               (1u << 31)
#define CLOCK_HW_CTL            (1u << 1)               /* gated by the hardware: CLOCK_OFF says only "idle now" */
#define XO_HZ                   19200000

static uint32_t se_source_hz, se_clock_select;

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

/* A branch's status, with CLOCK_OFF left out of a branch the hardware gates (Linux skips its check too) */
static uint32_t
branch_status(vm_offset_t cbcr)
{
	uint32_t value = rd(cbcr);

	return (value & CLOCK_HW_CTL) ? value & ~CLOCK_OFF : value;
}

static bool
clocks_on(void)
{
	apollo_gcc_vote(GCC_VOTE, VOTE_QUP0_CORE_2X | VOTE_QUP0_CORE | VOTE_QUP0_M_AHB | VOTE_QUP0_S_AHB);
	if (rd(GCC_QUP0_SE4_DFSR) & 1) {
		for (uint32_t level = 0; level < 8; level++) {
			uint32_t hz = rcg_rate(rd(GCC_QUP0_SE4_PERF_CFG(level)), rd(GCC_QUP0_SE4_PERF_M(level)),
			    rd(GCC_QUP0_SE4_PERF_N(level)));

			if (hz >= 4000000 && (se_source_hz == 0 || hz < se_source_hz)) {
				se_source_hz = hz;
				se_clock_select = level;
			}
		}
		if (se_source_hz == 0) {
			printf("apollo: no usable preset clock rate\n");
			return false;
		}
	} else {
		wr(GCC_QUP0_SE4_CFG, (rd(GCC_QUP0_SE4_CFG) & ~0x371fu) | 1);
		wr(GCC_QUP0_SE4_CMD, rd(GCC_QUP0_SE4_CMD) | 1);
		for (int i = 0; i < 500 && (rd(GCC_QUP0_SE4_CMD) & 1); i++) {
			sleep_us(10);
		}
		if (rd(GCC_QUP0_SE4_CMD) & 1) {
			printf("apollo: the serial engine's clock does not take its configuration (%#x)\n", rd(GCC_QUP0_SE4_CMD));
			return false;
		}
		se_source_hz = XO_HZ;
		se_clock_select = 0;
	}
	apollo_gcc_vote(GCC_VOTE, VOTE_QUP0_SE4);
	for (int i = 0; i < 1000; i++) {
		if (!((rd(GCC_QUP0_M_AHB_CBCR) | branch_status(GCC_QUP0_S_AHB_CBCR) | rd(GCC_QUP0_SE4_CBCR)) & CLOCK_OFF)) {
			return true;
		}
		sleep_us(10);
	}
	printf("apollo: clocks stay off: %#x %#x %#x\n", rd(GCC_QUP0_M_AHB_CBCR), rd(GCC_QUP0_S_AHB_CBCR),
	    rd(GCC_QUP0_SE4_CBCR));
	return false;
}

/* ---- SPI through the serial engine (GENI) ---------------------------------------------------------- */

#define SE_PHYS                 0x00990000UL     /* the fifth serial engine of the first QUPv3 group */
#define SE_SIZE                 0x4000UL
#define GENI_FORCE_DEFAULT      (se + 0x020)
#define GENI_OUTPUT_CTRL        (se + 0x024)
#define GENI_CGC_CTRL           (se + 0x028)
#define GENI_STATUS             (se + 0x040)    /* bit 0: a command is running */
#define GENI_SER_M_CLK_CFG      (se + 0x048)    /* bit 0 enable, [15:4] divider */
#define GENI_IF_DISABLE_RO      (se + 0x064)    /* bit 0: no FIFO interface */
#define GENI_FW_REVISION_RO     (se + 0x068)    /* [15:8] protocol: 1 = SPI */
#define GENI_CLK_SEL            (se + 0x07c)
#define SPI_CPHA                (se + 0x224)
#define SPI_LOOPBACK            (se + 0x22c)
#define SPI_CPOL                (se + 0x230)
#define SPI_DEMUX_OUTPUT_INV    (se + 0x24c)
#define SPI_DEMUX_SEL           (se + 0x250)
#define GENI_BYTE_GRAN          (se + 0x254)
#define GENI_DMA_MODE_EN        (se + 0x258)
#define SPI_TRANS_CFG           (se + 0x25c)
#define GENI_TX_PACKING_CFG0    (se + 0x260)
#define GENI_TX_PACKING_CFG1    (se + 0x264)
#define SPI_WORD_LEN            (se + 0x268)
#define SPI_TX_TRANS_LEN        (se + 0x26c)
#define SPI_RX_TRANS_LEN        (se + 0x270)
#define SPI_DELAY_COUNTERS      (se + 0x278)
#define GENI_RX_PACKING_CFG0    (se + 0x284)
#define GENI_RX_PACKING_CFG1    (se + 0x288)
#define GENI_M_CMD0             (se + 0x600)    /* [31:27] operation */
#define GENI_M_CMD_CTRL         (se + 0x604)    /* bit 2 cancel, bit 1 abort */
#define GENI_M_IRQ_STATUS       (se + 0x610)
#define GENI_M_IRQ_EN           (se + 0x614)
#define GENI_M_IRQ_CLEAR        (se + 0x618)
#define GENI_S_IRQ_CLEAR        (se + 0x648)
#define GENI_TX_FIFO            (se + 0x700)
#define GENI_RX_FIFO            (se + 0x780)
#define GENI_TX_FIFO_STATUS     (se + 0x800)    /* [27:0] words */
#define GENI_RX_FIFO_STATUS     (se + 0x804)    /* bit 31 last, [30:28] bytes of the last word, [24:0] words */
#define GENI_TX_WATERMARK       (se + 0x80c)
#define GENI_RX_WATERMARK       (se + 0x810)
#define GENI_RX_RFR_WATERMARK   (se + 0x814)
#define SE_DMA_TX_IRQ_CLR       (se + 0xc44)
#define SE_DMA_RX_IRQ_CLR       (se + 0xd44)
#define SE_GSI_EVENT_EN         (se + 0xe18)
#define SE_IRQ_EN               (se + 0xe1c)
#define SE_HW_PARAM_0           (se + 0xe24)    /* [21:16] FIFO depth in words */
#define SE_DMA_GENERAL_CFG      (se + 0xe30)

#define M_CMD_DONE              (1u << 0)
#define M_CMD_CANCEL            (1u << 4)
#define M_CMD_ABORT             (1u << 5)
#define M_ERRORS                ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 24) | (1u << 25) | (1u << 28) | (1u << 29))
#define SPI_FULL_DUPLEX         (3u << 27)

static uint32_t fifo_depth;

static void geni_show_state(const char *when);

static const char *
geni_spi_init(uint32_t hz)
{
	uint32_t divider;

	if ((rd(GENI_FW_REVISION_RO) >> 8 & 0xff) != 1) {
		return "the serial engine has no SPI firmware";
	}
	if (rd(GENI_IF_DISABLE_RO) & 1) {
		return "the serial engine's FIFO interface is disabled";
	}
	fifo_depth = rd(SE_HW_PARAM_0) >> 16 & 0x3f;
	if (fifo_depth < 4) {
		return "the serial engine reports no FIFO";
	}
	wr(SE_GSI_EVENT_EN, 0);
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
	wr(GENI_S_IRQ_CLEAR, 0xffffffff);
	wr(SE_DMA_TX_IRQ_CLR, 0xffffffff);
	wr(SE_DMA_RX_IRQ_CLR, 0xffffffff);
	wr(GENI_CGC_CTRL, rd(GENI_CGC_CTRL) | 0x7f);
	wr(SE_DMA_GENERAL_CFG, rd(SE_DMA_GENERAL_CFG) | 0xf);
	wr(GENI_OUTPUT_CTRL, 0x7f);
	wr(GENI_FORCE_DEFAULT, 1);
	wr(GENI_DMA_MODE_EN, rd(GENI_DMA_MODE_EN) & ~1u);
	wr(GENI_RX_WATERMARK, fifo_depth - 3);
	wr(GENI_RX_RFR_WATERMARK, fifo_depth - 2);
	wr(GENI_TX_WATERMARK, 0);
	wr(SE_IRQ_EN, rd(SE_IRQ_EN) | 0xf);
	wr(GENI_M_IRQ_EN, rd(GENI_M_IRQ_EN) | 0x7fc0007f);
	wr(SPI_LOOPBACK, 0);
	wr(SPI_DEMUX_SEL, 0);
	wr(SPI_CPHA, 0);
	wr(SPI_CPOL, 0);
	wr(SPI_DEMUX_OUTPUT_INV, 0);
	wr(SPI_TRANS_CFG, rd(SPI_TRANS_CFG) & ~2u);
	wr(SPI_DELAY_COUNTERS, 0);
	wr(GENI_TX_PACKING_CFG0, 0x0007f8fe);
	wr(GENI_TX_PACKING_CFG1, 0x000ffefe);
	wr(GENI_RX_PACKING_CFG0, 0x0007f8fe);
	wr(GENI_RX_PACKING_CFG1, 0x000ffefe);
	wr(GENI_BYTE_GRAN, 0);
	wr(SPI_WORD_LEN, 8 - 4);
	divider = (se_source_hz + hz - 1) / hz;
	wr(GENI_CLK_SEL, se_clock_select & 7);
	wr(GENI_SER_M_CLK_CFG, (divider ? divider : 1) << 4 | 1);
	geni_show_state("set up");
	return NULL;
}

/* What the serial engine says of itself: enough to tell a stuck command, a full FIFO and the wrong mode apart */
static void
geni_show_state(const char *when)
{
	printf("apollo: engine, %s: status %#x, m_irq %#x/%#x, cmd %#x, tx fifo %#x, rx fifo %#x, fifo depth %u "
	    "(hw param %#x), fw %#x, if-disable %#x, dma mode %#x, clk cfg %#x sel %#x\n", when, rd(GENI_STATUS),
	    rd(GENI_M_IRQ_STATUS), rd(GENI_M_IRQ_EN), rd(GENI_M_CMD0), rd(GENI_TX_FIFO_STATUS), rd(GENI_RX_FIFO_STATUS),
	    fifo_depth, rd(SE_HW_PARAM_0), rd(GENI_FW_REVISION_RO), rd(GENI_IF_DISABLE_RO), rd(GENI_DMA_MODE_EN),
	    rd(GENI_SER_M_CLK_CFG), rd(GENI_CLK_SEL));
}

static void
geni_drain(uint8_t *rx, uint32_t length, uint32_t *received)
{
	uint32_t status = rd(GENI_RX_FIFO_STATUS), words = status & 0x01ffffff, bytes = words * 4;

	if (status & 0x80000000u) {
		uint32_t valid = status >> 28 & 7;

		if (valid && valid < 4) {
			bytes -= 4 - valid;
		}
	}
	for (uint32_t i = 0; i < words; i++) {
		uint32_t word = rd(GENI_RX_FIFO);

		for (int j = 0; j < 4 && bytes; j++, bytes--) {
			if (*received < length) {
				if (rx) {
					rx[*received] = (uint8_t)(word >> (8 * j));
				}
				(*received)++;
			}
		}
	}
}

static void
geni_recover(void)
{
	wr(GENI_M_CMD_CTRL, 1u << 2);
	for (int i = 0; i < 2000 && !(rd(GENI_M_IRQ_STATUS) & M_CMD_CANCEL); i++) {
		sleep_us(100);
	}
	if (!(rd(GENI_M_IRQ_STATUS) & M_CMD_CANCEL)) {
		wr(GENI_M_CMD_CTRL, 1u << 1);
		for (int i = 0; i < 2000 && !(rd(GENI_M_IRQ_STATUS) & M_CMD_ABORT); i++) {
			sleep_us(100);
		}
	}
	for (int i = 0; i < 64 && (rd(GENI_RX_FIFO_STATUS) & 0x01ffffff); i++) {
		(void)rd(GENI_RX_FIFO);
	}
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
}

/* One transfer with the chip selected throughout: `length` bytes out (zeros without tx) and as many in. */
static bool
geni_transfer(const uint8_t *tx, uint8_t *rx, uint32_t length)
{
	static unsigned complaints;
	uint32_t sent = 0, received = 0, status = 0;
	uint64_t deadline;

	for (int i = 0; rd(GENI_STATUS) & 1; i++) {
		if (i > 2000) {
			geni_recover();
			break;
		}
		sleep_us(100);
	}
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
	wr(SPI_TX_TRANS_LEN, length);
	wr(SPI_RX_TRANS_LEN, length);
	wr(GENI_M_CMD0, SPI_FULL_DUPLEX);
	clock_interval_to_deadline(200, NSEC_PER_MSEC, &deadline);
	while (mach_absolute_time() < deadline) {
		while (sent < length && (rd(GENI_TX_FIFO_STATUS) & 0x0fffffff) + 1 < fifo_depth) {
			uint32_t word = 0;

			for (int j = 0; j < 4 && sent < length; j++, sent++) {
				word |= (uint32_t)(tx ? tx[sent] : 0) << (8 * j);
			}
			wr(GENI_TX_FIFO, word);
		}
		status = rd(GENI_M_IRQ_STATUS);
		geni_drain(rx, length, &received);
		if (status & M_ERRORS) {
			break;
		}
		if (status & M_CMD_DONE) {
			geni_drain(rx, length, &received);
			wr(GENI_M_IRQ_CLEAR, status);
			if (received == length) {
				return true;
			}
			break;
		}
	}
	if (complaints == 0) {
		geni_show_state("first failed transfer");
	}
	if (complaints++ < 8) {
		printf("apollo: SPI transfer of %u bytes failed: status %#x, sent %u, received %u\n", length, status, sent,
		    received);
	}
	geni_recover();
	return false;
}

static bool (*spi_transfer)(const uint8_t *tx, uint8_t *rx, uint32_t length);

#define NVT_EVENT_BUF_ADDR          0x22d00
#define NVT_EVENT_FWINFO           0x78
#define NVT_EVENT_RESET_COMPLETE   0x60
#define NVT_EVENT_PROJECTID        0x9a
#define NVT_EVENT_MAP_HOST_CMD     0x50
#define NVT_EVENT_MAP_HANDSHAKING 0x51
#define NVT_RESET_STATE_INIT      0xa0
#define NVT_RESET_STATE_MAX       0xaf
#define NVT_POINT_DATA_LEN        65
#define NVT_MAX_TOUCHES           10
#define NVT_SW_RST_ADDR           0x3f0fe
#define NVT_SPI_RD_FAST_ADDR      0x3f310

/* one transfer's bytes: a whole touch report (NVT_POINT_DATA_LEN + 1) and its two framing bytes fit */
static uint8_t spi_tx[80], spi_rx[80];
static uint32_t max_x = 10800, max_y = 24000;
static uint32_t nvt_abs_x_max, nvt_abs_y_max;
static bool apollo_touch_verbose = true;

static bool
nvt_set_page(uint32_t addr)
{
	uint8_t buffer[3];

	buffer[0] = 0xff;
	buffer[1] = (addr >> 15) & 0xff;
	buffer[2] = (addr >> 7) & 0xff;
	for (uint32_t i = 0; i < 3; i++) {
		spi_tx[i] = buffer[i];
	}
	return spi_transfer(spi_tx, spi_rx, 3);
}

/*
 * data[0] is the first byte after the address: Linux's buffers keep the address in [0], so its buf[n] is data[n - 1]
 * here.
 */
static bool
nvt_read(uint32_t addr, uint8_t *data, uint32_t length)
{
	uint32_t frames = length + 2;

	for (uint32_t i = 0; i < frames; i++) {
		spi_tx[i] = (uint8_t)((i == 0) ? (addr & 0x7f) : 0);
	}
	if (!spi_transfer(spi_tx, spi_rx, frames)) {
		return false;
	}
	for (uint32_t i = 0; i < length; i++) {
		data[i] = spi_rx[i + 2];
	}
	return true;
}

static struct {
	uint32_t polls, reports, touches, bad, last_x, last_y, edges, released, wdt, esd, reloads;
} seen;

static struct {
	bool down, seen;
	uint32_t x, y;
} contact[NVT_MAX_TOUCHES];
static uint64_t fingers_at;     /* when a report last had a finger in it */

/* every finger still down is lifted: for when the reports that would have said so did not come */
static void
apollo_release_all(void)
{
	for (uint32_t id = 0; id < NVT_MAX_TOUCHES; id++) {
		if (contact[id].down) {
			contact[id].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
			seen.released++;
		}
	}
}

static bool
apollo_any_down(void)
{
	for (uint32_t id = 0; id < NVT_MAX_TOUCHES; id++) {
		if (contact[id].down) {
			return true;
		}
	}
	return false;
}
static uint32_t flip;   /* boot-arg touch_flip: 1 = mirror left-right, 2 = mirror top-bottom */

static void
apollo_nvt_report(const uint8_t *data, uint32_t blocks)
{
	static unsigned shown;
	bool moved = false;

	/* a finger that this report leaves out has been lifted (as in Linux's nvt_ts_work_func) */
	for (uint32_t id = 0; id < NVT_MAX_TOUCHES; id++) {
		contact[id].seen = false;
	}
	for (uint32_t i = 0; i < blocks; i++) {
		const uint8_t *t = data + 6 * i;
		uint32_t id = t[0] >> 3, status = t[0] & 0x7;

		if (id == 0 || id > NVT_MAX_TOUCHES) {
			continue;
		}
		if (status != 1 && status != 2) {
			if (contact[id - 1].down) {
				contact[id - 1].down = false;
				hakios_touch_post(HAKIOS_TOUCH_UP, id - 1, contact[id - 1].x, contact[id - 1].y);
			}
			continue;
		}
		uint32_t x = ((uint32_t)t[1] << 4) + (uint32_t)(t[3] >> 4);
		uint32_t y = ((uint32_t)t[2] << 4) + (uint32_t)(t[3] & 0xf);

		if (x > nvt_abs_x_max || y > nvt_abs_y_max) {
			continue;
		}
		uint32_t sx = x >= max_x ? 65535 : x * 65535 / max_x;
		uint32_t sy = y >= max_y ? 65535 : y * 65535 / max_y;
		if (flip & 1) {
			sx = 65535 - sx;
		}
		if (flip & 2) {
			sy = 65535 - sy;
		}
		seen.touches++;
		fingers_at = mach_absolute_time();
		seen.last_x = sx;
		seen.last_y = sy;
		contact[id - 1].seen = true;
		if (!contact[id - 1].down) {
			contact[id - 1].down = true;
			contact[id - 1].x = sx;
			contact[id - 1].y = sy;
			hakios_touch_post(HAKIOS_TOUCH_DOWN, id - 1, sx, sy);
			/* (a second finger: whether the chip reports one at all; the first few are told) */
			if (shown < 20) {
				uint32_t down = 0;

				for (uint32_t n = 0; n < NVT_MAX_TOUCHES; n++) {
					down += contact[n].down;
				}
				if (down > 1 || id > 1) {
					shown++;
					say("apollo touch: finger %u down (%u down now), report byte %#x\n", id, down, t[0]);
				}
			}
		} else if (contact[id - 1].x != sx || contact[id - 1].y != sy) {
			hakios_touch_post(HAKIOS_TOUCH_MOVE, id - 1, sx, sy);
			moved = true;
			contact[id - 1].x = sx;
			contact[id - 1].y = sy;
		}
	}
	for (uint32_t id = 0; id < NVT_MAX_TOUCHES; id++) {
		if (contact[id].down && !contact[id].seen) {
			contact[id].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
		}
	}
	apollo_hitch_touch(apollo_any_down(), moved);   /* (apollo_hitch=1: a gap between frames counts only while a finger moves) */
}

/* ---- the chip's firmware -------------------------------------------------------------------------- */

/*
 * The NT36675 has no flash: it waits in its boot code until the host has put its firmware in its memory, after
 * every reset of the panel (Android's driver does it at each screen-on). This is Linux's
 * nvt_download_firmware_hw_crc (Xiaomi's apollo kernel, drivers/input/touchscreen/nt36672c/nt36xxx_fw_update.c),
 * with the NT36675's memory map (nt36xxx_mem_map.h: hw_crc 2). The file is the phone's own, built into its
 * Android kernel; it comes in fw/ of the small image (fw-touch/ when the image is built).
 */
#define NVT_FW_NAME             "novatek_nt36675_j3s_fw01.bin"
#define NVT_BOOT_RDY_ADDR       0x3f10d
#define NVT_ILM_LENGTH_ADDR     0x3f118
#define NVT_DLM_LENGTH_ADDR     0x3f130
#define NVT_ILM_DES_ADDR        0x3f128
#define NVT_DLM_DES_ADDR        0x3f12c
#define NVT_G_ILM_CHECKSUM_ADDR 0x3f100
#define NVT_G_DLM_CHECKSUM_ADDR 0x3f104
#define NVT_R_ILM_CHECKSUM_ADDR 0x3f120
#define NVT_R_DLM_CHECKSUM_ADDR 0x3f124
#define NVT_BLD_CRC_EN_ADDR     0x3f30e
#define NVT_BLD_ILM_DLM_CRC_ADDR 0x3f133
#define NVT_RESET_STATE_REK     0xa1
#define NVT_MAX_PARTITIONS      32
#define NVT_WRITE_CHUNK         1024

extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

static uint8_t nvt_write_buffer[NVT_WRITE_CHUNK + 1];

/* `length` bytes to the chip's memory at `addr`: the address byte with its write bit, then the data */
static bool
nvt_write(uint32_t addr, const uint8_t *data, uint32_t length)
{
	while (length > 0) {
		uint32_t n = length < NVT_WRITE_CHUNK ? length : NVT_WRITE_CHUNK;

		if (!nvt_set_page(addr)) {
			return false;
		}
		nvt_write_buffer[0] = (uint8_t)((addr & 0x7f) | 0x80);
		memcpy(nvt_write_buffer + 1, data, n);
		if (!spi_transfer(nvt_write_buffer, NULL, n + 1)) {
			return false;
		}
		addr += n;
		data += n;
		length -= n;
	}
	return true;
}

static bool
nvt_write_byte(uint32_t addr, uint8_t value)
{
	return nvt_write(addr, &value, 1);
}

static bool
nvt_read_at(uint32_t addr, uint8_t *data, uint32_t length)
{
	return nvt_set_page(addr) && nvt_read(addr, data, length);
}

static uint32_t
le32(const uint8_t *p)
{
	return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static struct {
	uint32_t bin, sram, size, crc;
} nvt_part[NVT_MAX_PARTITIONS];
static uint32_t nvt_parts;

/* nvt_bin_header_parser: ILM and DLM first, then the info sections from 0x30, then the DLM's overlays */
static const char *
nvt_parse_firmware(const uint8_t *fw, uint64_t size)
{
	uint32_t end, info, overlays;

	if (size < 0x20000 || memcmp(fw + 0x20000 - 3, "NVT", 3) != 0) {
		return "has no \"NVT\" end flag at 128 KiB";
	}
	if (fw[0x20000 - 0x1000] + fw[0x20000 - 0x1000 + 1] != 0xff) {
		return "has a broken version";
	}
	/* (the info sections run from 0x30 to `end`: within the file, or the file is not believed) */
	end = le32(fw);
	if (end > size) {
		return "has a section table beyond its end";
	}
	info = end > 0x30 ? (uint32_t)(((uint64_t)end - 0x30 + 0xf) / 0x10) : 0;
	overlays = (fw[0x28] & 0x10) ? (fw[0x28] & 0x0f) : 0;
	nvt_parts = 2 + info + overlays;
	if (nvt_parts > NVT_MAX_PARTITIONS) {
		return "has too many sections";
	}
	for (uint32_t i = 0; i < nvt_parts; i++) {
		if (i < 2) {
			nvt_part[i].bin = le32(fw + 12 * i);
			nvt_part[i].sram = le32(fw + 12 * i + 4);
			nvt_part[i].size = le32(fw + 12 * i + 8);
			nvt_part[i].crc = le32(fw + 0x18 + 4 * i);
		} else {
			/* (in 64 bits: an overlay's place comes from DLM's offset, which the file says, sized or not) */
			uint64_t pos = i < 2 + info ? 0x30 + 0x10 * (uint64_t)(i - 2) :
			    (uint64_t)nvt_part[1].bin + 0x10 * (uint64_t)(i - 2 - info);

			if (pos + 16 > size) {
				return "has a section table beyond its end";
			}
			nvt_part[i].sram = le32(fw + pos);
			nvt_part[i].size = le32(fw + pos + 4);
			nvt_part[i].bin = le32(fw + pos + 8);
			nvt_part[i].crc = le32(fw + pos + 12);
		}
		/* the sections are written one byte longer than they say (nvt_write_firmware) */
		if (nvt_part[i].size != 0 && (uint64_t)nvt_part[i].bin + nvt_part[i].size + 1 > size) {
			return "has a section beyond its end";
		}
	}
	return NULL;
}

static bool
nvt_reset_state(uint8_t at_least, int tries, uint8_t *seen)
{
	uint8_t status[6];

	for (int i = 0; i < tries; i++) {
		if (!nvt_read_at(NVT_EVENT_BUF_ADDR | NVT_EVENT_RESET_COMPLETE, status, 6)) {
			return false;
		}
		*seen = status[0];
		if (status[0] >= at_least && status[0] <= NVT_RESET_STATE_MAX) {
			return true;
		}
		sleep_ms(10);
	}
	return false;
}

/* the address, length and expected checksum of ILM or DLM, for the boot code's own check (nvt_set_bld_crc_bank) */
static bool
nvt_crc_bank(uint32_t des, uint32_t length_addr, uint32_t checksum_addr, uint32_t part)
{
	uint8_t a[3] = { (uint8_t)nvt_part[part].sram, (uint8_t)(nvt_part[part].sram >> 8), (uint8_t)(nvt_part[part].sram >> 16) };
	uint8_t l[3] = { (uint8_t)nvt_part[part].size, (uint8_t)(nvt_part[part].size >> 8), (uint8_t)(nvt_part[part].size >> 16 & 1) };
	uint8_t c[4] = { (uint8_t)nvt_part[part].crc, (uint8_t)(nvt_part[part].crc >> 8), (uint8_t)(nvt_part[part].crc >> 16),
		         (uint8_t)(nvt_part[part].crc >> 24) };

	return nvt_write(des, a, 3) && nvt_write(length_addr, l, 3) && nvt_write(checksum_addr, c, 4);
}

static bool
nvt_download_firmware(const uint8_t *fw)
{
	uint8_t state = 0, flag;

	for (int attempt = 1; attempt <= 3; attempt++) {
		/* the boot code's reset (nvt_bootloader_reset), with the fast read mode off */
		if (!nvt_write_byte(NVT_SW_RST_ADDR, 0x69)) {
			return false;
		}
		sleep_ms(5);
		nvt_write_byte(NVT_SPI_RD_FAST_ADDR, 0x00);

		for (uint32_t i = 0; i < nvt_parts; i++) {
			if (nvt_part[i].size != 0 && !nvt_write(nvt_part[i].sram, fw + nvt_part[i].bin, nvt_part[i].size + 1)) {
				printf("apollo: touch firmware: writing section %u failed\n", i);
				return false;
			}
		}
		nvt_crc_bank(NVT_ILM_DES_ADDR, NVT_ILM_LENGTH_ADDR, NVT_G_ILM_CHECKSUM_ADDR, 0);
		nvt_crc_bank(NVT_DLM_DES_ADDR, NVT_DLM_LENGTH_ADDR, NVT_G_DLM_CHECKSUM_ADDR, 1);
		/* the boot code checks ILM and DLM itself (nvt_bld_crc_enable) */
		if (nvt_read_at(NVT_BLD_CRC_EN_ADDR, &flag, 1)) {
			nvt_write_byte(NVT_BLD_CRC_EN_ADDR, flag | 0x80);
		}
		/* the firmware's reset status cleared, its own check on (nvt_fw_crc_enable) */
		nvt_write_byte(NVT_EVENT_BUF_ADDR | NVT_EVENT_RESET_COMPLETE, 0x00);
		nvt_write_byte(NVT_EVENT_BUF_ADDR | NVT_EVENT_MAP_HOST_CMD, 0xae);
		/* and go (nvt_boot_ready) */
		nvt_write_byte(NVT_BOOT_RDY_ADDR, 1);
		sleep_ms(5);
		if (nvt_reset_state(NVT_RESET_STATE_INIT, 11, &state)) {
			printf("apollo: touch firmware loaded (%u sections; attempt %d), reset state %#x\n", nvt_parts, attempt, state);
			return true;
		}
		printf("apollo: touch firmware: no reset state after attempt %d (%#x)\n", attempt, state);
	}
	{
		uint8_t crc_flag = 0, r[8] = { 0 };

		nvt_read_at(NVT_BLD_ILM_DLM_CRC_ADDR, &crc_flag, 1);
		nvt_read_at(NVT_R_ILM_CHECKSUM_ADDR, r, 8);
		printf("apollo: touch firmware: crc done %u, ilm %u, dlm %u; ilm %#x (file %#x), dlm %#x (file %#x)\n",
		    crc_flag >> 2 & 1, crc_flag & 1, crc_flag >> 1 & 1, le32(r), nvt_part[0].crc, le32(r + 4), nvt_part[1].crc);
	}
	return false;
}

static const uint8_t *nvt_file;    /* kept for a reload (Linux's ESD and watchdog recoveries) */

static bool
apollo_novatek_probe(void)
{
	uint8_t fw[17], state = 0;
	uint64_t size = 0;
	const uint8_t *file = hakios_image_file("fw", NVT_FW_NAME, &size, NULL);

	nvt_file = file;
	const char *problem;

	if (file == NULL) {
		printf("apollo: no fw/" NVT_FW_NAME " in the small image (fw-touch/ when it is built)\n");
		return false;
	}
	problem = nvt_parse_firmware(file, size);
	if (problem != NULL) {
		printf("apollo: " NVT_FW_NAME " %s\n", problem);
		return false;
	}
	if (!nvt_download_firmware(file)) {
		return false;
	}

	if (!nvt_read_at(NVT_EVENT_BUF_ADDR | NVT_EVENT_FWINFO, fw, 17)) {
		return false;
	}
	if ((fw[0] + fw[1]) != 0xff) {
		if (apollo_touch_verbose) {
			printf("apollo: the touch firmware version read %02x %02x\n", fw[0], fw[1]);
		}
		return false;
	}
	nvt_abs_x_max = ((uint32_t)fw[4] << 8) | fw[5];
	nvt_abs_y_max = ((uint32_t)fw[6] << 8) | fw[7];
	max_x = nvt_abs_x_max;
	max_y = nvt_abs_y_max;
	if (apollo_touch_verbose) {
		printf("apollo: touch firmware version %02x, geometry %u x %u\n", fw[0], max_x, max_y);
	}

	/* the baseline (nvt_ts_resume waits for it too), then the page of the reports stays selected */
	if (!nvt_reset_state(NVT_RESET_STATE_REK, 50, &state)) {
		printf("apollo: the touch firmware has no baseline yet (reset state %#x)\n", state);
	}
	return nvt_set_page(NVT_EVENT_BUF_ADDR);
}

static void
apollo_touch_show_state(const char *when)
{
	printf("apollo: %s: votes %#x, clocks %#x %#x %#x, rate cfg %#x dfs %#x; pins cs/miso/mosi/clk cfg %#x %#x %#x %#x,"
	    " reset %#x/%u, int %#x/%u, vddio %#x/%u\n", when, rd(GCC_VOTE), rd(GCC_QUP0_M_AHB_CBCR),
	    rd(GCC_QUP0_S_AHB_CBCR), rd(GCC_QUP0_SE4_CBCR), rd(GCC_QUP0_SE4_CFG), rd(GCC_QUP0_SE4_DFSR), rd(GPIO_CFG(PIN_CS)),
	    rd(GPIO_CFG(PIN_MISO)), rd(GPIO_CFG(PIN_MOSI)), rd(GPIO_CFG(PIN_CLK)), rd(GPIO_CFG(PIN_RESET)),
	    rd(GPIO_IO(PIN_RESET)), rd(GPIO_CFG(PIN_INT)), rd(GPIO_IO(PIN_INT)), rd(GPIO_CFG(PIN_VDDIO)), rd(GPIO_IO(PIN_VDDIO)));
}

/*
 * The screen follows iOS (apollo_backlight_power -> apollo_touch_screen): the caller only records the wanted state and
 * wakes the thread, which does the SPI work. A request that comes before the thread has started (or without a touch
 * thread at all) is kept: the thread's loop looks at the wanted state first, so a screen switched off before the touch
 * was up puts the chip to sleep as soon as it is.
 */
static volatile bool touch_screen_wanted = true;

void
apollo_touch_screen(bool on)
{
	touch_screen_wanted = on;
	thread_wakeup((event_t)&touch_screen_wanted);
}

/* nvt_ts_suspend: the deep-sleep command, then no more reports are read until the screen is on again */
static void
apollo_touch_sleep(void)
{
	apollo_release_all();
	sleep_ms(10);                           /* nvt_ts_suspend: mdelay(10) before the command */
	/* nvt_ts_suspend (no gesture wake): buf[0] = EVENT_MAP_HOST_CMD, buf[1] = 0x11 "deep sleep mode" */
	bool ok = nvt_write_byte(NVT_EVENT_BUF_ADDR | NVT_EVENT_MAP_HOST_CMD, 0x11);

	sleep_ms(10);                           /* nvt_ts_suspend: mdelay(10) after it */
	say("apollo touch: the screen is off, the chip is put to sleep%s\n", ok ? "" : " (the command failed)");
}

/*
 * nvt_ts_resume: the reset line high (the panel's reset has reset the chip), the firmware downloaded again
 * (nvt_update_firmware, which starts with the boot code's reset), then the wait for the baseline (RESET_STATE_REK).
 */
static void
apollo_touch_wake(void)
{
	uint8_t state = 0;
	bool ok = false;

	for (int attempt = 1; attempt <= 2 && !ok; attempt++) {
		/*
		 * The vendor only raises the line: on Android the panel's own power cycle has reset the chip by then. Here the panel
		 * stays powered with the light off, so a second attempt pulses the line low first.
		 */
		if (attempt == 2) {
			gpio_output(PIN_RESET, false, PULL_UP, 8);
			sleep_ms(10);
		}
		gpio_output(PIN_RESET, true, PULL_UP, 8);       /* nvt_ts_resume: gpio_set_value(ts->reset_gpio, 1) */
		if (attempt == 2) {
			sleep_ms(10);
		}
		if (nvt_file == NULL || !nvt_download_firmware(nvt_file)) {
			say("apollo touch: the firmware could not be loaded on wake (attempt %d)\n", attempt);
			continue;
		}
		/* nvt_ts_resume: nvt_check_fw_reset_state(RESET_STATE_REK) */
		if (!nvt_reset_state(NVT_RESET_STATE_REK, 50, &state)) {
			say("apollo touch: no baseline after the wake (reset state %#x, attempt %d)\n", state, attempt);
			continue;
		}
		ok = true;
	}
	nvt_set_page(NVT_EVENT_BUF_ADDR);       /* the page of the reports stays selected */
	say("apollo touch: the screen is on, the chip is %s\n", ok ? "awake" : "not confirmed awake; polling anyway");
}

static void
apollo_touch_thread(__unused void *parameter, __unused wait_result_t result)
{
	static uint8_t report[NVT_POINT_DATA_LEN + 1];
	uint32_t poll_us = 8000;

	PE_parse_boot_argn("touch_poll_us", &poll_us, sizeof(poll_us));
	if (poll_us < 250 || poll_us > 100000) {
		poll_us = 8000;
	}
	printf("apollo: touch thread started: map the pins, the clock controller and serial engine 4\n");
	tlmm_north = ml_io_map(TLMM_PIN_PHYS(TLMM_NORTH, PIN_MISO), 0x4000);           /* pins 8 to 11 */
	tlmm_south_38 = ml_io_map(TLMM_PIN_PHYS(TLMM_SOUTH, PIN_RESET), 0x2000);       /* pins 38 and 39 */
	tlmm_south_69 = ml_io_map(TLMM_PIN_PHYS(TLMM_SOUTH, PIN_VDDIO), 0x1000);
	gcc = ml_io_map(GCC_PHYS, GCC_SIZE);
	se = ml_io_map(SE_PHYS, SE_SIZE);
	if (tlmm_north == 0 || tlmm_south_38 == 0 || tlmm_south_69 == 0 || gcc == 0 || se == 0) {
		printf("apollo: cannot map TLMM/GCC/GENI registers\n");
		thread_terminate(current_thread());
		return;
	}

	bool clocks_ok = clocks_on();
	if (!clocks_ok) {
		printf("apollo: the clocks cannot be turned on\n");
		thread_terminate(current_thread());
		return;
	}
	apollo_touch_show_state("as found");
	/* the reset line high and left so: the panel's own reset has reset the chip (nvt_ts_resume does no more) */
	gpio_output(PIN_VDDIO, true, PULL_NONE, 2);
	sleep_ms(20);
	gpio_output(PIN_RESET, true, PULL_UP, 8);
	sleep_ms(10);
	apollo_touch_show_state("powered");
	gpio_configure(PIN_MISO, PIN_FUNCTION_SPI, PULL_NONE, 6, false);
	gpio_configure(PIN_MOSI, PIN_FUNCTION_SPI, PULL_NONE, 6, false);
	gpio_configure(PIN_CLK, PIN_FUNCTION_SPI, PULL_NONE, 6, false);
	gpio_configure(PIN_CS, PIN_FUNCTION_SPI, PULL_NONE, 6, true);
	const char *problem = geni_spi_init(10000000);         /* touch_spi@0's spi-max-frequency */
	if (problem != NULL) {
		printf("apollo: %s\n", problem);
		thread_terminate(current_thread());
		return;
	}
	spi_transfer = geni_transfer;
	if (apollo_novatek_probe()) {
		/* the chip is up */
	} else {
		printf("apollo: no Novatek touch screen\n");
		thread_terminate(current_thread());
		return;
	}
	/*
	 * The chip has a report ready when its interrupt line rises (the tree's irq-gpio: 39, rising edge). The TLMM
	 * latches that edge for us without passing an interrupt on (raw status on, the enable bit off), and the latch
	 * is looked at every millisecond: a report is read only then, never while the chip is writing the next one.
	 * Until a first edge has been seen a report is read every 100 ms all the same; touch_int=0 reads on a timer.
	 */
	uint32_t use_int = 1;
	uint64_t last_read = mach_absolute_time(), fallback, fallback_down;

	PE_parse_boot_argn("touch_int", &use_int, sizeof(use_int));
	nanoseconds_to_absolutetime(100 * NSEC_PER_MSEC, &fallback);
	nanoseconds_to_absolutetime(15 * NSEC_PER_MSEC, &fallback_down);
	if (use_int) {
		uint32_t cfg = rd(GPIO_INTR_CFG(PIN_INT));

		cfg &= ~(INTR_ENABLE | INTR_DETECT_MASK | INTR_POLARITY_HIGH);
		wr(GPIO_INTR_CFG(PIN_INT), cfg | INTR_RAW_STATUS | INTR_DETECT_EDGE | INTR_POLARITY_HIGH);
		wr(GPIO_INTR_STATUS(PIN_INT), 0);
		printf("apollo: Novatek touch screen ready; a report is read when its interrupt line rises (cfg %#x)\n",
		    rd(GPIO_INTR_CFG(PIN_INT)));
	} else {
		printf("apollo: Novatek touch screen ready; the report is read every %u us\n", poll_us);
	}

	uint64_t now, lifted, esd_quiet, status_every, status_at = mach_absolute_time(), edge_at = status_at, gap_long;
	struct {
		uint64_t since;                 /* the last report passed on */
		bool down;                      /* a finger was down after it */
		uint32_t reads, empty, failed, no_edge, edges, told;
	} gap = { .since = status_at };

	nanoseconds_to_absolutetime(100 * NSEC_PER_MSEC, &gap_long);
	unsigned wdt_run = 0;
	bool esd_pending = false;

	/*
	 * Measured (touch_lat=1, 25 October): while a finger is on the glass a report comes every 8 ms or so, and the one
	 * that would say it has lifted does not come at all -- every tap of a run ended on this timer. At 150 ms a quick
	 * second tap landed while the first was still down, as the same finger moving: the keyboard took the first key
	 * as dragged to the second and wrote neither. 45 ms is five reports missed.
	 */
	nanoseconds_to_absolutetime(45 * NSEC_PER_MSEC, &lifted);
	nanoseconds_to_absolutetime(1500 * NSEC_PER_MSEC, &esd_quiet);
	nanoseconds_to_absolutetime(10 * NSEC_PER_SEC, &status_every);
	for (;;) {
		if (!touch_screen_wanted) {
			apollo_touch_sleep();
			/* no polling and no SPI traffic until the screen is on */
			for (;;) {
				assert_wait((event_t)&touch_screen_wanted, THREAD_UNINT);
				if (touch_screen_wanted) {
					clear_wait(current_thread(), THREAD_AWAKENED);
					break;
				}
				thread_block(THREAD_CONTINUE_NULL);
			}
			apollo_touch_wake();
			wdt_run = 0;
			esd_pending = false;
			if (use_int) {
				wr(GPIO_INTR_STATUS(PIN_INT), 0);
			}
			last_read = edge_at = mach_absolute_time();
		}
		now = mach_absolute_time();
		if (use_int) {
			bool edge = rd(GPIO_INTR_STATUS(PIN_INT)) & 1;

			/*
			 * Without an edge, a look every 100 ms all the same: a missed edge must not leave a finger down. While a
			 * finger is down, every 15 ms: measured (touch_lat=1), one tap in 17 was cut in two when a missed edge left
			 * a gap of more than the 45 ms after which a finger counts as lifted.
			 */
			if (!edge && now - last_read < (apollo_any_down() ? fallback_down : fallback)) {
				/* the chip reports all the time a finger is on the glass: none for 45 ms, and it is gone */
				if (apollo_any_down() && now - fingers_at > lifted) {
					apollo_release_all();
				}
				sleep_us(1000);
				continue;
			}
			if (!edge) {
				gap.no_edge++;
			}
			if (edge) {
				gap.edges++;
				wr(GPIO_INTR_STATUS(PIN_INT), 0);       /* before the read: an edge during it is the next report */
				edge_at = now;
				if (seen.edges++ == 0) {
					printf("apollo: the first edge of the touch interrupt line: reports follow it from now on\n");
				}
			}
			last_read = now;
		}
		seen.polls++;
		/*
		 * A gap between reports while a finger is down (the hitch tracer, 28 October: 409-479 ms without a report while a
		 * finger moved, and no frame from iOS meanwhile): what the reads in it saw, said once the next report comes.
		 */
		gap.reads++;
		if (!nvt_read(NVT_EVENT_BUF_ADDR, report, NVT_POINT_DATA_LEN + 1)) {
			gap.failed++;
			seen.bad++;
			if (seen.bad < 8) {
				printf("apollo: the touch report cannot be read\n");
			}
		} else {
			bool wdt = true, esd = true;

			seen.reports++;
			/* Linux's two recoveries look at the first six bytes: 0xfd/0xfe the firmware's watchdog, 0x77 ESD */
			for (int i = 0; i < 6; i++) {
				wdt = wdt && (report[i] == 0xfd || report[i] == 0xfe);
				esd = esd && report[i] == 0x77;
			}
			if (wdt) {
				if (++wdt_run > 10) {
					seen.wdt++;
					printf("apollo: the touch firmware's watchdog has reset it: loading it again\n");
					goto reload;
				}
			} else {
				wdt_run = 0;
			}
			if (esd) {
				if (!esd_pending) {
					seen.esd++;
					printf("apollo: the touch firmware asks for its ESD check\n");
				}
				esd_pending = true;
			} else if (report[0] != 0xff || report[1] != 0xff) {
				uint64_t since = now - gap.since;

				if (gap.down && since > gap_long && gap.told < 64) {
					uint64_t ns;

					absolutetime_to_nanoseconds(since, &ns);
					gap.told++;
					say("apollo touch: %llu ms without a report while a finger was down: %u reads (%u empty, %u failed, "
					    "%u with no edge), %u edges, the interrupt line %u now\n", ns / 1000000, gap.reads, gap.empty,
					    gap.failed, gap.no_edge, gap.edges, rd(GPIO_IO(PIN_INT)) & 1);
				}
				apollo_nvt_report(report, NVT_POINT_DATA_LEN / 6);
				gap.since = now;
				gap.down = apollo_any_down();
				gap.reads = gap.empty = gap.failed = gap.no_edge = gap.edges = 0;
			} else {
				gap.empty++;
			}
		}
		/* (nvt_esd_check_func) the firmware asked, and the line has been quiet for 1.5 s: load it again */
		if (esd_pending && now - edge_at > esd_quiet) {
			printf("apollo: no touch report for 1.5 s after the ESD request: loading the firmware again\n");
			goto reload;
		}
		goto status;
reload:
		apollo_release_all();
		wdt_run = 0;
		esd_pending = false;
		seen.reloads++;
		if (nvt_file == NULL || !nvt_download_firmware(nvt_file) || !nvt_set_page(NVT_EVENT_BUF_ADDR)) {
			printf("apollo: the touch firmware could not be loaded again\n");
		}
		if (use_int) {
			wr(GPIO_INTR_STATUS(PIN_INT), 0);
		}
		edge_at = mach_absolute_time();
status:
		if (mach_absolute_time() - status_at > status_every) {
			status_at = mach_absolute_time();
			printf("apollo: touch: %u reports in %u reads (%u bad), %u edges, %u finger positions, %u lifted by the "
			    "driver, %u watchdog and %u ESD recoveries (%u reloads); int line %u\n", seen.reports, seen.polls,
			    seen.bad, seen.edges, seen.touches, seen.released, seen.wdt, seen.esd, seen.reloads,
			    rd(GPIO_IO(PIN_INT)) & 1);
		}
		if (!use_int) {
			sleep_us(poll_us);
		}
	}
}

void
apollo_touch_init(void)
{
	uint32_t enabled = 0;

	if (!PE_parse_boot_argn("apollo_touch", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	PE_parse_boot_argn("touch_flip", &flip, sizeof(flip));
	thread_t thread;

	if (kernel_thread_start(apollo_touch_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "apollo touch");
		thread_deallocate(thread);
	}
}
