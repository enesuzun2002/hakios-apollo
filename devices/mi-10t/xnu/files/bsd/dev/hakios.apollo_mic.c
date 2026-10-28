/*
 * The Xiaomi Mi 10T's (apollo) main microphone, through the audio DSP. A copy of bsd/dev/hakios.mic.c (the POCO X5
 * Pro's, brought up on that phone), kept apart from it, with apollo's numbers: from Android's own live device tree
 * (the legacy APR stack's bolero-cdc under /soc/qcom,msm-audio-apr/qcom,q6core-audio), Xiaomi's mixer paths
 * (.device-local/apollo/audio/) and Xiaomi's kernel (apollo-q-oss, kernel-apollo/: techpack/audio at LA.UM.8.12:
 * asoc/codecs/bolero/, asoc/codecs/wcd938x/, soc/swr-mstr-ctrl.c, soc/pinctrl-lpi.c, dsp/q6afe.c,
 * asoc/codecs/audio-ext-clk-up.c, asoc/kona.c, asoc/kona-port-config.h; drivers/pinctrl/qcom/pinctrl-kona.c;
 * arch/arm64/boot/dts/venor/qcom/kona-lpi.dtsi, kona-pinctrl.dtsi). Nothing here happens unless boot-arg
 * apollo_mic=N asks for it, with apollo_audio=N (hakios.apollo_adsp.c), which calls apollo_mic_dsp_up() once the
 * DSP's audio services are up. (The POCO's driver needs boot-arg mic=, which apollo never sets: the two never run
 * together, and /dev/hakios-mic is this one's on apollo.)
 *
 * What differs from the POCO, and why it matters here:
 *   - the digital codec, "bolero", is 1.x on kona (bolero-cdc.c gives it 1.2 for its four macros, unless the VA
 *     macro's core ID says 2.1): no SoundWire mic clock dividers anywhere (va_macro_init() and the TX macro's
 *     SWR_MIC_CTL writes are 2.0/2.1 only), the "TX SMIC MUX" inputs are ADC0..3 and SWR_DMIC0..7 (smic_mux_text),
 *     and every register access of a 1.x macro is made under the macro's clock (__bolero_reg_read()), which for
 *     the TX and the VA macro is TX_CORE_CLK (the tree's va-macro qcom,default-clk-id 0)
 *   - the TX SoundWire master is version 1.5.1 (the tree's qcom,swr-master-version 0x1050001): it has no HCTL
 *     workaround (that is >= 1.6, and the POCO's 0x32a90a8 is not touched here), its link status is not looked at
 *     (swrm_check_link_status() returns true at once for <= 1.5.1), and -- what matters most -- every access to its
 *     registers is made with the TX macro's SoundWire clock running (is_swr_clk_needed(): <= 1.5.1, so
 *     swrm_ahb_read()/swrm_ahb_write() call swrm_clk_request() around each access). apollo_mic=1 therefore reads
 *     the master only if that clock is found running already; apollo_mic=2 reads it once the clock is started
 *   - the frame and the ports are kona's own: swr-mstr-ctrl.c of this techpack copies the master's port tables to
 *     the codec's ports as they are (no table of the devices' own), from kona-port-config.h's
 *     tx_frame_params_default (asoc/kona.c gives sm_port_map to this phone: no wcd937x, no lito v2, not the
 *     "xrfusionult" card)
 *   - the codec's reset is TLMM GPIO 32 on the south tile, the pins are LPI GPIOs 0, 1 and 2 only
 *
 * The main microphone under Android (Xiaomi's "voice-rec-mic" path, which a one-channel recording uses, with
 * "audio-record" = MultiMedia1 <- TX_CDC_DMA_TX_3): the analog microphone at ADC1 of the WCD938x codec ("AMIC1",
 * biased by "MIC BIAS1": the tree's qcom,audio-routing), whose samples go over SoundWire (the TX master of bolero)
 * into decimator 0 of the TX macro ("TX DEC0 MUX" SWR_MIC, "TX SMIC MUX0" ADC0: bolero's ADC0 is the codec's ADC1,
 * "TX SWR_ADC0" <- "ADC1_OUTPUT" in the tree's routing), out of it through the codec DMA interface
 * TX_CDC_DMA_TX_3, the AFE's port, into a stream that is read (ASM). Decimator 0, not 1: "voice-rec-mic" is
 * Xiaomi's one-channel recording from ADC1 alone and uses DEC0 (its "handset-mic" puts ADC1 on DEC1 only because
 * DEC0 carries the second microphone, ADC4, there); with DEC0 the port's one channel is its first (mask 1), and
 * DEC0 is also the one decimator tx_macro_init() sets up (tx_macro_reg_init: TX0_TX_PATH_SEC7). Before any of it:
 * the LPASS's own clocks, which the DSP gives out (Android's audio-ref-clk nodes):
 *   lpass_core_hw_vote   AFE_CMD_REMOTE_LPASS_CORE_HW_VOTE_REQUEST, block 3 ("LPASS_HW_MACRO"): the tree's
 *                        vote_lpass_core_hw, qcom,codec-ext-clk-src 9 = AUDIO_EXT_CLK_LPASS_CORE_HW_VOTE, which
 *                        lpass_hw_vote_prepare() (audio-ext-clk-up.c) votes as AFE_LPASS_CORE_HW_MACRO_BLOCK
 *                        (q6afe-v2.h: NONE 0, RSVD 1, AVTIMER 2, MACRO 3, DCODEC 4)
 *   lpass_audio_hw_vote  the same, block 4 ("LPASS_HW_DCODEC"): ext-clk-src 11 = AUDIO_EXT_CLK_LPASS_AUDIO_HW_VOTE
 *   tx_core_clk          AFE clock 0x30c at 19.2 MHz (the tree's tx_core_clk: qcom,codec-lpass-clk-id, -ext-clk-freq)
 *   tx_npl_clk           AFE clock 0x30d at 19.2 MHz (bolero_clk_rsc_mux0_clk_request() starts the NPL clock with
 *                        the core one). (The VA macro's own va_core 0x30b and va_npl 0x310 are not asked for:
 *                        the TX path's clock request is TX_CORE_CLK for TX_CORE_CLK, tx_macro_mclk_enable().)
 * Without them the macros' registers are not to be touched at all: an access to a block without its clock may
 * hang the bus.
 *
 *   apollo_mic=1  a look: the votes and the two clocks asked for, then -- only if all four were given -- the
 *                 registers of the TX and VA macros, of the LPASS pins the master uses (and their slew rates)
 *                 and of the codec's reset pin read (nothing written to any of them); the TX SoundWire master's
 *                 too, but only if the TX macro's SoundWire clock is found running (see above); then the clocks
 *                 and the votes given back
 *   apollo_mic=2  ... and the codec brought up far enough to answer on SoundWire, nothing of it set: its supplies
 *                 as the boot chain leaves them (PM8150 S4, 1.8 V, always on, and the PM8150A's BoB for the mic
 *                 bias: nothing of the PMICs is touched), its reset released (GPIO 32 low 20 us, then high, 16 mA:
 *                 wcd938x_reset() with msm-cdc-pinctrl's two states), the master's pins to their active state
 *                 (LPASS GPIOs 0, 1, 2: function 1, 2 mA, slew rate 1; the clock without a pull, the data with a
 *                 keeper), the TX macro's clocks as tx_macro_tx_va_mclk_enable() starts them (bolero's
 *                 "fs-gen-sequence" in the VA macro, FREQ_MCLK, MCLK_CONTROL, FS_CNT_CONTROL, the SoundWire clock
 *                 with a reset), the master started as swrm_runtime_resume() and swrm_master_init() start it (the
 *                 frame 50 rows x 2 columns, auto-enumeration, its interrupts' masks), the codec's TX device waited
 *                 for (ID 0x000d01170223, the tree's wcd938x-tx-slave) and its chip ID read over SoundWire. Then
 *                 all of it undone, in reverse: the codec back in reset, the pins and registers as they were found
 *   apollo_mic=3  ... and the main microphone listened to for apollo_mic_secs seconds (default 10, at most 600)
 *                 the way Android records from it: the codec set up as wcd938x_bind() and wcd938x_init_reg() set
 *                 it up (its interrupts as edges, the mic biases' 2750 mV, its bias and sleep registers); the
 *                 DSP's side (a stream read through our memory, ASM session 2; a device path for recording, ADM;
 *                 the codec DMA port TX_CDC_DMA_TX_3, AFE port 0xB037, 48 kHz, 16 bits, one channel: decimator
 *                 0); the TX macro's decimator 0 fed from ADC0 ("TX SMIC MUX0" ADC0, "TX DEC0 MUX" SWR_MIC,
 *                 "TX_DEC0 Volume" 87, and tx_macro_enable_dec()); "MIC BIAS1" and ADC1 in its default ("normal")
 *                 mode -- Xiaomi's paths set no "TX0 MODE" -- with "ADC1 Volume" 12 (Xiaomi's "voice-rec-mic");
 *                 the SoundWire port as kona's tables have it (see datapath_on()); then the decimator unmuted after
 *                 100 ms and its filter settled after 300 ms, as tx-macro's delayed work does. Each second's level
 *                 goes to the log. Every register write is read back (the few whose reading back is not known on
 *                 this phone are said so, with what they read); any mismatch or a DSP command refused stops it
 *                 all. Tests: apollo_mic_input=0 (decimator 0 fed from ZERO: exact silence), apollo_mic_adc=0 (ADC1
 *                 left off), apollo_mic_test=1 (no mic bias), apollo_mic_test=2 (ADC1's input held)
 *   apollo_mic=4  the main microphone for iOS: /dev/hakios-mic, with the POCO's contract (see below)
 *
 * What apollo_mic=3 and 4 record is apollo_mic_path=N (see port_config()'s comment): 1, the default, Android's
 * ordinary recording -- Xiaomi's "handset-mic", both microphones (the main one, ADC1, on decimator 1; the top
 * one, ADC4 through "ADC4 MUX" INP5 with "MIC BIAS4", on decimator 0), the port's two channels, and Xiaomi's
 * topology 0x10010315 with its ACDB calibration in the device path; the one channel iOS gets is made here
 * (apollo_mic_mix: 0 the average, 1 the main microphone, 2 the top one). 0: the "voice-rec-mic" path described
 * above (ADC1 alone, decimator 0, no processing), as first brought up; path 1 falls back to it if any of it fails.
 *
 * What is asked for is given back at the end, and before the DSP is shut down (apollo_mic_dsp_down()).
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <libkern/OSAtomic.h>
#include <kern/kalloc.h>
#include <mach/mach_types.h>
#include <miscfs/devfs/devfs.h>
#include <pexpert/pexpert.h>
#include <sys/conf.h>
#include <sys/hakios.apollo_input.h>
#include <sys/systm.h>
#include <sys/uio.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern void loader_log_putc(char c);

static LCK_GRP_DECLARE(apollo_mic_lock_group, "apollo-mic");
static LCK_MTX_DECLARE(mic_lock, &apollo_mic_lock_group);      /* what is asked for, and given back */

static uint32_t level;
static bool running;

static int say(const char *format, ...) __printflike(1, 2);

/*
 * Every line is kept as well, and all of them said again at the run's end (apollo_mic_quiesce()):
 * the log of a run keeps its beginning and its end, and this happens in between.
 */
#define KEPT_LINES              400
static char kept[KEPT_LINES][168];
static unsigned kept_count;

static int
say(const char *format, ...)
{
	char line[400];
	va_list list;
	int length;

	va_start(list, format);
	length = vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	printf("%s", line);
	strlcpy(kept[kept_count++ % KEPT_LINES], line, sizeof(kept[0]));
	return length;
}

/* A line that says all went as it should: not said again in iOS's later recordings (apollo_mic=4), where it is the same each time. */
static bool quiet;
#define note(...)       do { if (!quiet) { say(__VA_ARGS__); } } while (0)

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

static uint32_t __attribute__((noinline))
rd(vm_offset_t address)
{
	uint32_t value;

	__asm__ volatile ("ldr %w0, [%1]" : "=r" (value) : "r" (address) : "memory");
	return value;
}

static void __attribute__((noinline))
wr(vm_offset_t address, uint32_t value)
{
	__asm__ volatile ("str %w0, [%1]" : : "r" (value), "r" (address) : "memory");
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

static bool broken;                     /* a write that did not read back: nothing more is written, all is undone */
static char last_write[120] = "nothing written";

struct block {
	const char *name;
	uint32_t phys, size;
	vm_offset_t base;
};

static uint32_t
get(struct block *b, const char *name, uint32_t offset)
{
	uint32_t value;

	if (b->base == 0) {
		b->base = ml_io_map(b->phys, b->size);
	}
	value = rd(b->base + offset);
	note("apollo mic: %s %s (%#x) = %#x\n", b->name, name, b->phys + offset, value);
	return value;
}

/*
 * A register written (and read back, unless `check' is 0: some bits clear themselves, and a few registers
 * are not known to read back on this phone -- what they read is said). A write that does not read back as
 * written (in the bits of `check') stops the bring-up.
 */
static bool
put(struct block *b, const char *name, uint32_t offset, uint32_t value, uint32_t check)
{
	uint32_t back;

	if (broken) {
		return false;
	}
	if (b->base == 0) {
		b->base = ml_io_map(b->phys, b->size);
	}
	snprintf(last_write, sizeof(last_write), "%#x to %s %s (%#x)", value, b->name, name, b->phys + offset);
	wr(b->base + offset, value);
	back = rd(b->base + offset);
	if ((back & check) != (value & check)) {
		say("apollo mic: %s %s (%#x) <- %#x, but it reads %#x: STOPPING HERE\n", b->name, name, b->phys + offset, value,
		    back);
		broken = true;
		return false;
	}
	if (check) {
		note("apollo mic: %s %s (%#x) <- %#x, reads back\n", b->name, name, b->phys + offset, value);
	} else {
		note("apollo mic: %s %s (%#x) <- %#x (not checked: it reads %#x)\n", b->name, name, b->phys + offset, value, back);
	}
	return true;
}

/* only the bits of `mask' changed */
static bool
put_bits(struct block *b, const char *name, uint32_t offset, uint32_t mask, uint32_t value)
{
	uint32_t now;

	if (b->base == 0) {
		b->base = ml_io_map(b->phys, b->size);
	}
	now = rd(b->base + offset);
	return put(b, name, offset, (now & ~mask) | (value & mask), mask);
}

/* ---- the DSP: votes and clocks ------------------------------------------------------------------- */

#define APR_SVC_AFE                             4
#define APR_BASIC_RSP_RESULT                    0x000110e8u
#define APR_WAIT_MS                             1000
#define AFE_SVC_CMD_SET_PARAM                   0x000100f3u
#define AFE_MODULE_CLOCK_SET                    0x0001028fu
#define AFE_PARAM_ID_CLOCK_SET                  0x00010290u
#define AFE_CMD_REMOTE_LPASS_CORE_HW_VOTE_REQUEST       0x000100f4u     /* (q6afe-v2.h) */
#define AFE_CMD_RSP_REMOTE_LPASS_CORE_HW_VOTE_REQUEST   0x000100f5u
#define AFE_CMD_REMOTE_LPASS_CORE_HW_DEVOTE_REQUEST     0x000100f6u
#define LPASS_HW_MACRO_BLOCK                    3       /* AFE_LPASS_CORE_HW_MACRO_BLOCK (q6afe-v2.h) */
#define LPASS_HW_DCODEC_BLOCK                   4       /* AFE_LPASS_CORE_HW_DCODEC_BLOCK */
#define CLOCK_TX_CORE                           0x30c   /* the tree's tx_core_clk, qcom,codec-lpass-clk-id */
#define CLOCK_TX_NPL                            0x30d   /* tx_npl_clk */
#define CLOCK_HZ                                19200000        /* qcom,codec-lpass-ext-clk-freq 0x124f800 */
#define CLOCK_COUPLE_NO                         1       /* Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO, audio-ext-clk-up.c's default */

static const char *
dsp_error(uint32_t status)
{
	static const char *const names[] = { "ok", "failed", "bad parameter", "unsupported", "version", "unexpected", "panic",
		                             "no resource", "bad handle", "already", "not ready", "pending", "busy", "aborted",
		                             "preempted", "continue", "immediate", "not implemented", "need more", "no memory",
		                             "not there" };

	return status < sizeof(names) / sizeof(names[0]) ? names[status] : "an error this does not know";
}

static bool
command(const char *what, uint8_t service, uint32_t opcode, const void *payload, uint32_t length, uint32_t wanted,
    struct hakios_apr_answer *answer)
{
	struct hakios_apr_answer local;

	if (answer == NULL) {
		answer = &local;
	}
	if (!apollo_adsp_apr(service, 0, 0, opcode, payload, length, APR_WAIT_MS, answer)) {
		say("apollo mic: %s (opcode %#x): NO ANSWER\n", what, opcode);
		return false;
	}
	if (answer->opcode == APR_BASIC_RSP_RESULT && answer->words[0] == opcode) {
		if (answer->words[1] == 0) {
			note("apollo mic: %s: done\n", what);
		} else {
			say("apollo mic: %s: REFUSED: %s\n", what, dsp_error(answer->words[1]));
		}
		return answer->words[1] == 0 && wanted == 0;
	}
	if (wanted != 0 && answer->opcode == wanted) {
		return true;
	}
	say("apollo mic: %s: an answer not expected: opcode %#x, %u bytes: %#x %#x\n", what, answer->opcode, answer->length,
	    answer->words[0], answer->words[1]);
	return false;
}

struct afe_param {
	uint32_t module, param;
	uint16_t size, reserved;
} __attribute__((packed));

struct afe_clock_set {
	uint32_t version, id, hz;
	uint16_t attribute, root;
	uint32_t enable;
} __attribute__((packed));

static bool
afe_service_param(const char *what, uint32_t module, uint32_t param, const void *data, uint16_t size)
{
	uint8_t packet[16 + sizeof(struct afe_param) + 64];
	uint32_t *head = (uint32_t *)packet;
	struct afe_param *p = (struct afe_param *)(packet + 16);

	bzero(packet, sizeof(packet));
	head[0] = (uint32_t)sizeof(*p) + size;          /* payload size; address 0 and handle 0: in band */
	p->module = module;
	p->param = param;
	p->size = size;
	memcpy(packet + 16 + sizeof(*p), data, size);
	return command(what, APR_SVC_AFE, AFE_SVC_CMD_SET_PARAM, packet, 16 + (uint32_t)sizeof(*p) + size, 0, NULL);
}

/* (audio-ext-clk-up.c: Q6AFE_LPASS_CLK_CONFIG_API_VERSION 1, the id and rate of the tree, couple-no, root 0) */
static bool
afe_clock(const char *what, uint32_t id, bool on)
{
	struct afe_clock_set clock = { 1, id, CLOCK_HZ, CLOCK_COUPLE_NO, 0, on };

	return afe_service_param(what, AFE_MODULE_CLOCK_SET, AFE_PARAM_ID_CLOCK_SET, &clock, sizeof(clock));
}

static uint32_t vote_handle[5];
static bool voted[5], core_clock, npl_clock;

static bool
vote(uint32_t block, const char *client)
{
	struct {
		uint32_t block;
		char client[8];
	} __attribute__((packed)) request = { block, { 0 } };
	struct hakios_apr_answer answer;
	char what[80];

	strlcpy(request.client, client, sizeof(request.client));
	snprintf(what, sizeof(what), "the LPASS's vote for block %u (\"%s\")", block, client);
	if (!command(what, APR_SVC_AFE, AFE_CMD_REMOTE_LPASS_CORE_HW_VOTE_REQUEST, &request, sizeof(request),
	    AFE_CMD_RSP_REMOTE_LPASS_CORE_HW_VOTE_REQUEST, &answer)) {
		return false;
	}
	vote_handle[block] = answer.words[0];
	voted[block] = true;
	say("apollo mic: %s: given, handle %u\n", what, vote_handle[block]);
	return true;
}

static void
devote(uint32_t block)
{
	uint32_t request[2] = { block, vote_handle[block] };
	char what[64];

	if (!voted[block]) {
		return;
	}
	snprintf(what, sizeof(what), "the LPASS's vote for block %u, given back", block);
	(void)command(what, APR_SVC_AFE, AFE_CMD_REMOTE_LPASS_CORE_HW_DEVOTE_REQUEST, request, sizeof(request), 0, NULL);
	voted[block] = false;
}

static bool
clocks_on(void)
{
	if (!vote(LPASS_HW_MACRO_BLOCK, "LPASS_HW_MACRO") || !vote(LPASS_HW_DCODEC_BLOCK, "LPASS_HW_DCODEC")) {
		return false;
	}
	core_clock = afe_clock("the TX macro's core clock (0x30c) at 19.2 MHz, on", CLOCK_TX_CORE, true);
	if (!core_clock) {
		return false;
	}
	npl_clock = afe_clock("the TX macro's NPL clock (0x30d) at 19.2 MHz, on", CLOCK_TX_NPL, true);
	return npl_clock;
}

static void
clocks_off(void)
{
	if (npl_clock) {
		(void)afe_clock("the TX macro's NPL clock, off", CLOCK_TX_NPL, false);
		npl_clock = false;
	}
	if (core_clock) {
		(void)afe_clock("the TX macro's core clock, off", CLOCK_TX_CORE, false);
		core_clock = false;
	}
	devote(LPASS_HW_DCODEC_BLOCK);
	devote(LPASS_HW_MACRO_BLOCK);
}

/* ---- the blocks, read ------------------------------------------------------------------------------ */

/* (the tree: tx-macro@3220000, va-macro@3370000, tx_swr_master swrm-io-base 0x3230000, lpi_pinctrl@33c0000) */
static struct block tx_macro = { "the TX macro", 0x03220000, 0x1000 };
static struct block va_macro = { "the VA macro", 0x03370000, 0x1000 };
static struct block tx_swr = { "the TX SoundWire master", 0x03230000, 0x2000 };
static struct block lpi = { "the LPASS pins", 0x033c0000, 0x3000 };
/* GPIO 32: TLMM 0x0f000000 + SOUTH 0x500000 + 32 x REG_SIZE 0x1000 (pinctrl-kona.c: PINGROUP(32, SOUTH, ...)) */
static struct block tlmm_32 = { "the pin controller", 0x0f520000, 0x1000 };

/* bolero-cdc-registers.h: the TX macro's from TX_START_OFFSET 0, the VA macro's from VA_START_OFFSET 0x3000 */
#define TX_MCLK_CONTROL         0x000
#define TX_FS_CNT_CONTROL       0x004
#define TX_SWR_CONTROL          0x008
#define TX_FREQ_MCLK            0x090
#define VA_MCLK_CONTROL         0x000
#define VA_FS_CNT_CONTROL       0x004
#define VA_TOP_CFG0             0x080
#define VA_CORE_ID_0            0x0c0
#define VA_CORE_ID_1            0x0c4

/* the master's registers to look at (swrm_registers.h) */
static const struct { const char *name; uint32_t offset; } swr_look[] = {
	{ "COMP_HW_VERSION", 0x000 }, { "COMP_CFG", 0x004 }, { "COMP_STATUS", 0x014 }, { "COMP_PARAMS", 0x100 },
	{ "COMP_MASTER_ID", 0x104 }, { "INTERRUPT_STATUS", 0x200 }, { "INTERRUPT_MASK", 0x204 }, { "ENUMERATOR_CFG", 0x500 },
	{ "MCP_FRAME_CTRL_BANK0", 0x101c }, { "MCP_FRAME_CTRL_BANK1", 0x105c }, { "MCP_BUS_CTRL", 0x1044 },
	{ "MCP_CFG", 0x1048 }, { "MCP_STATUS", 0x104c }, { "MCP_SLV_STATUS", 0x1090 },
};

static void
master_look(void)
{
	for (size_t i = 0; i < sizeof(swr_look) / sizeof(swr_look[0]); i++) {
		(void)get(&tx_swr, swr_look[i].name, swr_look[i].offset);
	}
}

static void
look(void)
{
	static const struct { const char *name; uint32_t offset; } tx[] = {
		{ "MCLK_CONTROL", 0x000 }, { "FS_CNT_CONTROL", 0x004 }, { "SWR_CONTROL", 0x008 }, { "TOP_CFG0", 0x080 },
		{ "SWR_CTRL", 0x088 }, { "FREQ_MCLK", 0x090 }, { "SWR_AMIC0_CTL", 0x0d0 }, { "SWR_AMIC1_CTL", 0x0d4 },
		{ "ADC_MUX0_CFG0", 0x100 }, { "ADC_MUX0_CFG1", 0x104 }, { "TX0_TX_PATH_CTL", 0x400 }, { "TX0_TX_PATH_CFG0", 0x404 },
		{ "TX0_TX_VOL_CTL", 0x40c }, { "TX0_TX_PATH_SEC2", 0x418 }, { "TX0_TX_PATH_SEC7", 0x42c },
	}, va[] = {
		{ "MCLK_CONTROL", 0x000 }, { "FS_CNT_CONTROL", 0x004 }, { "TOP_CFG0", 0x080 }, { "CORE_ID_0", 0x0c0 },
		{ "CORE_ID_1", 0x0c4 },
	};
	static const unsigned pins[] = { 0, 1, 2 };
	struct block slew = { "the LPASS pins' slew rates", 0x0355a000, 0x1000, 0 };

	for (size_t i = 0; i < sizeof(tx) / sizeof(tx[0]); i++) {
		(void)get(&tx_macro, tx[i].name, tx[i].offset);
	}
	for (size_t i = 0; i < sizeof(va) / sizeof(va[0]); i++) {
		(void)get(&va_macro, va[i].name, va[i].offset);
	}
	for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
		char name[48];

		snprintf(name, sizeof(name), "gpio%u config (SoundWire TX)", pins[i]);
		(void)get(&lpi, name, 0x1000 * pins[i]);
		snprintf(name, sizeof(name), "gpio%u in/out", pins[i]);
		(void)get(&lpi, name, 0x1000 * pins[i] + 4);
	}
	(void)get(&slew, "slew rates (gpio0: bits 0-1, gpio1: 2-3, gpio2: 4-5)", 0);
	(void)get(&tlmm_32, "gpio32 config (the codec's reset)", 0);
	(void)get(&tlmm_32, "gpio32 in/out", 4);
	/*
	 * The master last, and only with its clock running: a 1.5.1 master is not accessed without the TX
	 * macro's SoundWire clock (is_swr_clk_needed(), swrm_ahb_read()), and starting that clock is a write
	 */
	if (rd(tx_macro.base + TX_SWR_CONTROL) & 1) {
		say("apollo mic: the TX macro's SoundWire clock is running: the master is read\n");
		master_look();
	} else {
		say("apollo mic: the TX macro's SoundWire clock is off: the master (version 1.5.1) is NOT read without it; "
		    "apollo_mic=2 reads it once the clock is started\n");
	}
}

/* ---- the codec's reset, the master's pins ------------------------------------------------------------ */

#define TLMM_CFG                0x0
#define TLMM_IO                 0x4
static uint32_t found_reset_cfg, found_reset_io;
static bool reset_taken;

/*
 * wcd938x_reset(): msm-cdc-pinctrl's sleep state (wcd938x_reset_sleep: 16 mA, no pull, output low), 20 us,
 * its active state (wcd938x_reset_active: 16 mA, output high), 20 us. Both states name the function "func2",
 * which kona's TLMM does not have (pinctrl-kona.c: gpio32's are gpio, qup12, NA, atest_usb03, ...): the
 * pinctrl core drops that mux setting and keeps the configuration, so under Android the pin is a GPIO (function
 * 0), as it is written here. [1:0] pull, [5:2] function, [8:6] drive (mA / 2 - 1), [9] output enable.
 */
static bool
reset_release(void)
{
	if (tlmm_32.base == 0) {
		tlmm_32.base = ml_io_map(tlmm_32.phys, tlmm_32.size);
	}
	found_reset_cfg = rd(tlmm_32.base + TLMM_CFG);
	found_reset_io = rd(tlmm_32.base + TLMM_IO);
	reset_taken = true;
	if (!put(&tlmm_32, "gpio32 out (the codec's reset)", TLMM_IO, 0, 2) ||
	    !put(&tlmm_32, "gpio32 config: gpio, 16 mA, no pull, output", TLMM_CFG, 0x3c0, 0x3ff)) {
		return false;
	}
	spin_us(30);
	if (!put(&tlmm_32, "gpio32 out: high (the codec out of reset)", TLMM_IO, 2, 2)) {
		return false;
	}
	spin_us(30);
	return true;
}

static void
reset_restore(void)
{
	if (!reset_taken) {
		return;
	}
	broken = false;         /* (putting back is done whatever went before) */
	(void)put(&tlmm_32, "gpio32 out: low (the codec in reset)", TLMM_IO, 0, 2);
	(void)put(&tlmm_32, "gpio32 config, as it was found", TLMM_CFG, found_reset_cfg, 0x3ff);
	(void)put(&tlmm_32, "gpio32 out, as it was found", TLMM_IO, found_reset_io & 2, 2);
	reset_taken = false;
}

/*
 * kona-lpi.dtsi: tx_swr_clk_active (gpio0: func1, 2 mA, slew-rate 1, bias-disable), tx_swr_data1_active
 * (gpio1: func1, 2 mA, slew-rate 1, bias-bus-hold), tx_swr_data2_active (gpio2: the same); the slew register
 * qcom,slew-reg 0x355a000, the pins' places in it qcom,lpi-slew-offset-tbl (0, 2, 4), two bits each
 * (pinctrl-lpi.c: LPI_SLEW_BITS_SIZE). The register layout is pinctrl-lpi.c's (LPI_GPIO_REG_*), as on the POCO.
 */
static struct block lpi_slew = { "the LPASS pins' slew rates", 0x0355a000, 0x1000 };
static const struct { unsigned pin, slew_bit; uint32_t pull; const char *name; } swr_pins[] = {
	{ 0, 0, 0, "gpio0 (SoundWire TX clock)" }, { 1, 2, 2, "gpio1 (data 1)" }, { 2, 4, 2, "gpio2 (data 2)" },
};
#define SWR_PINS                (sizeof(swr_pins) / sizeof(swr_pins[0]))
#define SLEW_MASK               0x3fu   /* the three pins' fields */
static uint32_t found_pin_cfg[SWR_PINS], found_pin_dir[SWR_PINS], found_slew;
static bool pins_taken;

static bool
pins_on(void)
{
	uint32_t slew;

	if (lpi.base == 0) {
		lpi.base = ml_io_map(lpi.phys, lpi.size);
	}
	if (lpi_slew.base == 0) {
		lpi_slew.base = ml_io_map(lpi_slew.phys, lpi_slew.size);
	}
	for (unsigned i = 0; i < SWR_PINS; i++) {
		found_pin_cfg[i] = rd(lpi.base + 0x1000 * swr_pins[i].pin);
		found_pin_dir[i] = rd(lpi.base + 0x1000 * swr_pins[i].pin + 4);
	}
	found_slew = rd(lpi_slew.base);
	pins_taken = true;
	slew = found_slew;
	for (unsigned i = 0; i < SWR_PINS; i++) {
		slew = (slew & ~(3u << swr_pins[i].slew_bit)) | (1u << swr_pins[i].slew_bit);
	}
	if (!put(&lpi_slew, "slew rates: 1 for the three", 0, slew, SLEW_MASK)) {
		return false;
	}
	for (unsigned i = 0; i < SWR_PINS; i++) {
		/* [1:0] pull, [5:2] function, [8:6] drive (mA / 2 - 1), [9] output: function 1, 2 mA, not an output */
		uint32_t cfg = (found_pin_cfg[i] & ~0x3ffu) | swr_pins[i].pull | 1u << 2;

		if (!put(&lpi, swr_pins[i].name, 0x1000 * swr_pins[i].pin, cfg, 0x3ff) ||
		    !put(&lpi, "its direction: in", 0x1000 * swr_pins[i].pin + 4, 0, 2)) {
			return false;
		}
	}
	return true;
}

static void
pins_restore(void)
{
	if (!pins_taken) {
		return;
	}
	broken = false;
	for (unsigned i = 0; i < SWR_PINS; i++) {
		(void)put(&lpi, "a pin, as it was found", 0x1000 * swr_pins[i].pin, found_pin_cfg[i], 0x3ff);
		(void)put(&lpi, "its direction, as it was found", 0x1000 * swr_pins[i].pin + 4, found_pin_dir[i], 2);
	}
	(void)put(&lpi_slew, "slew rates, as they were found", 0, found_slew, SLEW_MASK);
	pins_taken = false;
}

/* ---- the TX macro's clocks, the SoundWire master ----------------------------------------------------- */

#define SWRM_COMP_HW_VERSION            0x0000
#define SWRM_COMP_CFG                   0x0004
#define SWRM_COMP_SW_RESET              0x0008
#define SWRM_COMP_STATUS                0x0014
#define SWRM_COMP_PARAMS                0x0100
#define SWRM_INTERRUPT_MASK             0x0204          /* (SWRM_INTERRUPT_MASK_ADDR: written with the interrupts that are on) */
#define SWRM_INTERRUPT_CLEAR            0x0208
#define SWRM_CPU_INTERRUPT_EN           0x0210          /* SWR_MSTR_RX_SWRM_CPU_INTERRUPT_EN */
#define SWRM_CMD_FIFO_WR_CMD            0x0300
#define SWRM_CMD_FIFO_RD_CMD            0x0304
#define SWRM_CMD_FIFO_CMD               0x0308
#define SWRM_CMD_FIFO_STATUS            0x030c
#define SWRM_CMD_FIFO_CFG               0x0314
#define SWRM_CMD_FIFO_RD_FIFO           0x0318
#define SWRM_ENUMERATOR_CFG             0x0500
#define SWRM_ENUMERATOR_SLAVE_DEV_ID_1(m) (0x0530 + 0x8 * (m))
#define SWRM_ENUMERATOR_SLAVE_DEV_ID_2(m) (0x0534 + 0x8 * (m))
#define SWRM_MCP_FRAME_CTRL_BANK(m)     (0x101c + 0x40 * (m))
#define SWRM_MCP_BUS_CTRL               0x1044
#define SWRM_MCP_CFG                    0x1048
#define SWRM_MCP_STATUS                 0x104c
#define SWRM_MCP_SLV_STATUS             0x1090
#define SWRM_INTERRUPT_STATUS_MASK      0x1fdfd         /* (swr-mstr-ctrl.c) */
#define SWRM_VERSION_1_5_1              0x01050001u     /* (swr-mstr-ctrl.h; the tree's qcom,swr-master-version) */
#define SWR_BUS_HZ                      9600000         /* MCLK_FREQ (swr-wcd.h): swrm_probe()'s mclk_freq and bus_clk */
#define SWR_ROWS                        50              /* SWR_ROW_50 (control value 1) */
#define SWR_FRAME_SYNC_HZ               4000            /* SWRM_FRAME_SYNC_SEL */
#define CODEC_TX_ID                     0x000d01170223ull       /* the tree's wcd938x-tx-slave, reg <0xd 0x1170223> */
#define SWRS_SCP_INT_STATUS_MASK_1      0x41

static bool tx_clocks_taken, master_taken;
static uint32_t found_freq_mclk;
static uint32_t wr_depth = 8;           /* the master's command FIFO's depth (COMP_PARAMS: swrm_probe()'s wr_fifo_depth) */
static uint8_t read_id = 0;

/*
 * tx_macro_tx_va_mclk_enable() for TX_MCLK (the pins first: pins_on()), with tx_macro_mclk_enable() in it:
 * bolero_clk_rsc_fs_gen_request() runs the tree's qcom,fs-gen-sequence <0x3000 1, 0x3004 1, 0x3080 2> as
 * regmap_update_bits(register, mask, mask) -- VA MCLK_CONTROL bit 0, FS_CNT_CONTROL bit 0, TOP_CFG0 bit 1 (the
 * POCO's tree has FS_CNT's two bits, and a toggle; this one has bit 0 alone) --; then "9.6MHz MCLK" (FREQ_MCLK
 * bit 0), MCLK_CONTROL and FS_CNT_CONTROL on; then, the first time (reset_swr), the SoundWire clock with a reset.
 */
static bool
tx_clocks_on(void)
{
	if (tx_macro.base == 0) {
		tx_macro.base = ml_io_map(tx_macro.phys, tx_macro.size);
	}
	found_freq_mclk = rd(tx_macro.base + TX_FREQ_MCLK);
	tx_clocks_taken = true;
	return put_bits(&va_macro, "MCLK_CONTROL (fs-gen)", VA_MCLK_CONTROL, 0x01, 0x01) &&
	       put_bits(&va_macro, "FS_CNT_CONTROL (fs-gen)", VA_FS_CNT_CONTROL, 0x01, 0x01) &&
	       put_bits(&va_macro, "TOP_CFG0 (fs-gen)", VA_TOP_CFG0, 0x02, 0x02) &&
	       /* (the POCO's reads 0 back whatever is written; on this phone it is not known: said, not checked) */
	       put(&tx_macro, "FREQ_MCLK: 9.6 MHz", TX_FREQ_MCLK, (found_freq_mclk & ~1u) | 1, 0) &&
	       put_bits(&tx_macro, "MCLK_CONTROL: on", TX_MCLK_CONTROL, 0x01, 0x01) &&
	       put_bits(&tx_macro, "FS_CNT_CONTROL: on", TX_FS_CNT_CONTROL, 0x01, 0x01) &&
	       put_bits(&tx_macro, "SWR_CONTROL: reset", TX_SWR_CONTROL, 0x02, 0x02) &&
	       put_bits(&tx_macro, "SWR_CONTROL: clock on", TX_SWR_CONTROL, 0x01, 0x01) &&
	       put_bits(&tx_macro, "SWR_CONTROL: reset over", TX_SWR_CONTROL, 0x02, 0x00);
}

static void
tx_clocks_off(void)
{
	if (!tx_clocks_taken) {
		return;
	}
	broken = false;
	(void)put_bits(&tx_macro, "SWR_CONTROL: clock off", TX_SWR_CONTROL, 0x01, 0x00);
	(void)put_bits(&tx_macro, "FS_CNT_CONTROL: off", TX_FS_CNT_CONTROL, 0x01, 0x00);
	(void)put_bits(&tx_macro, "MCLK_CONTROL: off", TX_MCLK_CONTROL, 0x01, 0x00);
	(void)put(&tx_macro, "FREQ_MCLK: as it was found", TX_FREQ_MCLK, found_freq_mclk, 0);
	(void)put_bits(&va_macro, "TOP_CFG0 (fs-gen off)", VA_TOP_CFG0, 0x02, 0x00);
	(void)put_bits(&va_macro, "FS_CNT_CONTROL (fs-gen off)", VA_FS_CNT_CONTROL, 0x01, 0x00);
	(void)put_bits(&va_macro, "MCLK_CONTROL (fs-gen off)", VA_MCLK_CONTROL, 0x01, 0x00);
	tx_clocks_taken = false;
}

static uint32_t
swr_rd(uint32_t offset)
{
	if (tx_swr.base == 0) {
		tx_swr.base = ml_io_map(tx_swr.phys, tx_swr.size);
	}
	return rd(tx_swr.base + offset);
}

/*
 * swrm_runtime_resume()'s branch for a master that is not woken from a clock stop (the SSR one: two software
 * resets, then swrm_master_init()); this techpack's master_init() for 1.5.1: no HCTL workaround (>= 1.6 only),
 * the frame 50 rows x 2 columns, ssp period 9.6 MHz x 2 / (50 x 2 x 4000) = 48, auto-enumeration, 31 commands
 * without a ping, 3 retries, the bus clock started, the interrupts as pulses and masked as swr-mstr-ctrl.c masks
 * them, the master on; no link status looked at (<= 1.5.1); then "continue execute on command ignore" (>= 1.5.1).
 */
static bool
master_on(void)
{
	uint32_t ssp = SWR_BUS_HZ * 2 / (SWR_ROWS * 2 * SWR_FRAME_SYNC_HZ), cfg, version;

	version = swr_rd(SWRM_COMP_HW_VERSION);
	wr_depth = (swr_rd(SWRM_COMP_PARAMS) & 0x7c00) >> 10;
	say("apollo mic: the TX SoundWire master is version %#x (the tree says 0x1050001); its write FIFO holds %u commands\n",
	    version, wr_depth);
	if (version != SWRM_VERSION_1_5_1) {
		say("apollo mic: NOT the master this was written for: STOPPING HERE (nothing of it written)\n");
		broken = true;
		return false;
	}
	if (wr_depth == 0) {
		wr_depth = 8;
	}
	master_taken = true;
	if (!put(&tx_swr, "COMP_SW_RESET", SWRM_COMP_SW_RESET, 1, 0) || !put(&tx_swr, "COMP_SW_RESET", SWRM_COMP_SW_RESET, 1, 0)) {
		return false;
	}
	cfg = (swr_rd(SWRM_MCP_CFG) & ~0x3e0000u) | 0x1fu << 17;
	if (!put(&tx_swr, "MCP_FRAME_CTRL_BANK0: 50 rows, 2 columns", SWRM_MCP_FRAME_CTRL_BANK(0), 1u << 3 | 0u | (ssp - 1) << 16,
	    0xff07ff) ||
	    !put(&tx_swr, "ENUMERATOR_CFG: auto", SWRM_ENUMERATOR_CFG, 1, 1) ||
	    !put(&tx_swr, "MCP_CFG: 31 pings", SWRM_MCP_CFG, cfg, 0x3e0000) ||
	    !put(&tx_swr, "CMD_FIFO_CFG: 3 retries", SWRM_CMD_FIFO_CFG, 3, 0x7) ||
	    !put(&tx_swr, "MCP_BUS_CTRL: the clock started", SWRM_MCP_BUS_CTRL, 2, 0) ||
	    !put(&tx_swr, "COMP_CFG: interrupts as pulses", SWRM_COMP_CFG, 2, 0) ||
	    !put(&tx_swr, "INTERRUPT_CLEAR", SWRM_INTERRUPT_CLEAR, 0xffffffff, 0) ||
	    !put(&tx_swr, "INTERRUPT_MASK", SWRM_INTERRUPT_MASK, SWRM_INTERRUPT_STATUS_MASK, 0) ||
	    !put(&tx_swr, "CPU_INTERRUPT_EN", SWRM_CPU_INTERRUPT_EN, SWRM_INTERRUPT_STATUS_MASK, 0) ||
	    !put(&tx_swr, "COMP_CFG: on", SWRM_COMP_CFG, 3, 0)) {
		return false;
	}
	spin_us(105);   /* ("wait for hw enumeration to complete", swrm_runtime_resume()) */
	say("apollo mic: the master is on (COMP_STATUS %#x: not looked at for 1.5.1, the codec's enumeration says)\n",
	    swr_rd(SWRM_COMP_STATUS));
	return put(&tx_swr, "CMD_FIFO_CFG: | 0x80000000", SWRM_CMD_FIFO_CFG, swr_rd(SWRM_CMD_FIFO_CFG) | 0x80000000u, 0);
}

static void
master_off(void)
{
	if (!master_taken) {
		return;
	}
	broken = false;
	(void)put(&tx_swr, "COMP_CFG: off", SWRM_COMP_CFG, 0, 0);
	(void)put(&tx_swr, "INTERRUPT_MASK: none", SWRM_INTERRUPT_MASK, 0, 0);
	(void)put(&tx_swr, "CPU_INTERRUPT_EN: none", SWRM_CPU_INTERRUPT_EN, 0, 0);
	(void)put(&tx_swr, "COMP_SW_RESET", SWRM_COMP_SW_RESET, 1, 0);
	master_taken = false;
}

/* A command to a device on the bus (SWR_REG_VAL_PACK): a register written (swrm_cmd_fifo_wr_cmd()). */
static bool
swr_write(uint8_t device, uint16_t reg, uint8_t value, uint8_t id)
{
	uint32_t packed = reg | (uint32_t)id << 16 | (uint32_t)device << 20 | (uint32_t)value << 24;

	for (int i = 0; i < 30 && ((swr_rd(SWRM_CMD_FIFO_STATUS) >> 8) & 0x1f) >= wr_depth; i++) {
		spin_us(510);
	}
	snprintf(last_write, sizeof(last_write), "SoundWire: device %u register %#x <- %#x", device, reg, value);
	wr(tx_swr.base + SWRM_CMD_FIFO_WR_CMD, packed);
	spin_us(155);
	return true;
}

/* ... and one read (swrm_cmd_fifo_rd_cmd()): false if the answer does not come (or is not this command's) */
static bool
swr_read(uint8_t device, uint16_t reg, uint8_t *value)
{
	uint32_t answer = 0, packed;

	read_id = read_id < 14 ? read_id + 1 : 0;
	packed = reg | (uint32_t)read_id << 16 | (uint32_t)device << 20 | 1u << 24;
	for (int i = 0; i < 30 && ((swr_rd(SWRM_CMD_FIFO_STATUS) >> 8) & 0x1f) >= wr_depth; i++) {
		spin_us(510);
	}
	spin_us(105);
	wr(tx_swr.base + SWRM_CMD_FIFO_RD_CMD, packed);
	spin_us(255);
	for (int tries = 0; tries < 4; tries++) {
		for (int i = 0; i < 30 && ((swr_rd(SWRM_CMD_FIFO_STATUS) >> 16) & 0x1f) == 0; i++) {
			spin_us(510);
		}
		answer = swr_rd(SWRM_CMD_FIFO_RD_FIFO);
		if (((answer >> 8) & 0xf) == read_id) {
			*value = (uint8_t)answer;
			return true;
		}
		spin_us(505);
		if (tries == 2) {
			/* (as swrm_cmd_fifo_rd_cmd() does before its last try: the FIFO flushed, the read sent again) */
			wr(tx_swr.base + SWRM_CMD_FIFO_CMD, 1);
			wr(tx_swr.base + SWRM_CMD_FIFO_RD_CMD, packed);
			spin_us(255);
		}
	}
	say("apollo mic: SoundWire: device %u register %#x: no answer (fifo %#x, last %#x)\n", device, reg,
	    swr_rd(SWRM_CMD_FIFO_STATUS), answer);
	return false;
}

static uint8_t codec_device;

/* the codec's TX device, enumerated: its device number, 0 if it did not show up (swrm_get_logical_dev_num()) */
static uint8_t
codec_wait(void)
{
	for (int waited = 0; waited < 200; waited++) {
		for (uint8_t d = 1; d <= 11; d++) {
			uint64_t id = (uint64_t)swr_rd(SWRM_ENUMERATOR_SLAVE_DEV_ID_2(d)) << 32 | swr_rd(SWRM_ENUMERATOR_SLAVE_DEV_ID_1(d));
			uint32_t status = (swr_rd(SWRM_MCP_SLV_STATUS) >> (2 * d)) & 3;

			if ((id & 0xffffffffffffull) == CODEC_TX_ID && (status == 1 || status == 2)) {
				say("apollo mic: the codec's TX device is device %u on the bus (ID %#llx, status %u) after %d ms\n", d,
				    id & 0xffffffffffffull, status, waited);
				return d;
			}
		}
		sleep_ms(1);
	}
	say("apollo mic: the codec's TX device did not show up; the master has seen:\n");
	for (uint8_t d = 1; d <= 11; d++) {
		uint64_t id = (uint64_t)swr_rd(SWRM_ENUMERATOR_SLAVE_DEV_ID_2(d)) << 32 | swr_rd(SWRM_ENUMERATOR_SLAVE_DEV_ID_1(d));

		if (id != 0) {
			say("apollo mic:   device %u: ID %#llx\n", d, id);
		}
	}
	say("apollo mic:   slave status %#x, COMP_STATUS %#x, interrupt status %#x\n", swr_rd(SWRM_MCP_SLV_STATUS),
	    swr_rd(SWRM_COMP_STATUS), swr_rd(0x200));
	return 0;
}

static void
codec_look(void)
{
	static const struct { const char *name; uint16_t reg; } regs[] = {
		{ "DIGITAL_CHIP_ID0", 0x3401 }, { "DIGITAL_CHIP_ID1", 0x3402 }, { "DIGITAL_CHIP_ID2", 0x3403 },
		{ "DIGITAL_CHIP_ID3", 0x3404 }, { "SCP DEVICE_ID_0", 0x0050 }, { "SCP DEVICE_ID_1", 0x0051 },
	};

	for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
		uint8_t value;

		if (swr_read(codec_device, regs[i].reg, &value)) {
			say("apollo mic: the codec's %s (%#x) = %#x\n", regs[i].name, regs[i].reg, value);
		}
	}
}

/* ---- apollo_mic=3: the codec set up as Android sets it up for the main microphone ------------------------- */

/* wcd938x-registers.h (WCD938X_BASE_ADDRESS 0x3000): the same chip as the POCO's, the same registers */
#define WCD_ANA_BIAS                    0x3001
#define WCD_ANA_TX_CH1                  0x300e
#define WCD_ANA_TX_CH2                  0x300f
#define WCD_ANA_MICB1                   0x3022
#define WCD_ANA_MICB2                   0x3023
#define WCD_ANA_MICB3                   0x3025
#define WCD_ANA_MICB4                   0x3026
#define WCD_BIAS_VBG_FINE_ADJ           0x3029
#define WCD_MICB1_TEST_CTL_1            0x306b
#define WCD_MICB1_TEST_CTL_2            0x306c
#define WCD_MICB2_TEST_CTL_1            0x306e
#define WCD_MICB2_TEST_CTL_2            0x306f
#define WCD_MICB3_TEST_CTL_1            0x3071
#define WCD_MICB3_TEST_CTL_2            0x3072
#define WCD_MICB4_TEST_CTL_1            0x3074
#define WCD_MICB4_TEST_CTL_2            0x3075
#define WCD_TX_3_4_TEST_BLK_EN2         0x308f
#define WCD_HPH_REFBUFF_UHQA_CTL        0x30dd
#define WCD_HPH_SURGE_HPHLR_SURGE_EN    0x30e2
#define WCD_SLEEP_CTL                   0x3103
#define WCD_LDORXTX_CONFIG              0x312a
#define WCD_HPH_NEW_INT_RDAC_GAIN_CTL   0x3132
#define WCD_HPH_NEW_INT_RDAC_HD2_CTL_L  0x3140
#define WCD_HPH_NEW_INT_RDAC_HD2_CTL_R  0x3141
#define WCD_TXFE_ICTRL_STG2MAIN_ULP     0x31df
#define WCD_TXFE_ICTRL_STG2CASC_ULP     0x31e1
#define WCD_SWR_TX_CLK_RATE             0x3405
#define WCD_CDC_ANA_CLK_CTL             0x3408
#define WCD_CDC_DIG_CLK_CTL             0x3409
#define WCD_CDC_TX_ANA_MODE_0_1         0x3410
#define WCD_CDC_ANA_TX_CLK_CTL          0x3417
#define WCD_CDC_REQ_CTL                 0x3457
#define WCD_CDC_DMIC_CTL                0x345b
#define WCD_INTR_LEVEL_0                0x3474
#define WCD_EFUSE_REG_30                0x34ce
#define WCD_TX_REQ_FB_CTL_0             0x34d0
#define SWRS_SCP_CONTROL                0x0044
#define SWRS_SCP_FRAME_CTRL_BANK(m)     (0x0060 + 0x10 * (m))
#define SWRS_SCP_HOST_CLK_DIV2_CTL_BANK(m) (0x00e0 + 0x10 * (m))
#define SWRS_DP_CHANNEL_ENABLE_BANK(n, m) (0x120 + 0x100 * (n) + 0x10 * (m))
#define SWRS_DP_SAMPLE_CONTROL_1_BANK(n, m) (0x122 + 0x100 * (n) + 0x10 * (m))
#define SWRS_DP_OFFSET_CONTROL_1_BANK(n, m) (0x124 + 0x100 * (n) + 0x10 * (m))
#define SWRS_DP_OFFSET_CONTROL_2_BANK(n, m) (0x125 + 0x100 * (n) + 0x10 * (m))
#define SWRS_DP_LANE_CONTROL_BANK(n, m) (0x128 + 0x100 * (n) + 0x10 * (m))
/* swrm_registers.h: port n (1-based) */
#define SWRM_DP_PORT_CTRL_BANK(n, m)    (0x1124 + 0x100 * ((n) - 1) + 0x40 * (m))
#define SWRM_DP_PORT_CTRL_2_BANK(n, m)  (0x1128 + 0x100 * ((n) - 1) + 0x40 * (m))
#define SWRM_DP_PORT_HCTRL_BANK(n, m)   (0x1134 + 0x100 * ((n) - 1) + 0x40 * (m))

static uint8_t write_id;

/* a register of the codec's (device codec_device on the bus) written and, unless `check' is 0, read back */
static bool
codec_put(uint16_t reg, uint8_t value, uint8_t check)
{
	uint8_t back = 0;

	if (broken) {
		return false;
	}
	write_id = write_id < 14 ? write_id + 1 : 0;
	swr_write(codec_device, reg, value, write_id);
	if (check == 0) {
		note("apollo mic: the codec's %#06x <- %#04x\n", reg, value);
		return true;
	}
	if (!swr_read(codec_device, reg, &back) || (back & check) != (value & check)) {
		say("apollo mic: the codec's %#06x <- %#04x, but it reads %#04x: STOPPING HERE\n", reg, value, back);
		broken = true;
		return false;
	}
	note("apollo mic: the codec's %#06x <- %#04x, reads back\n", reg, value);
	return true;
}

static bool
codec_bits(uint16_t reg, uint8_t mask, uint8_t value)
{
	uint8_t now = 0;

	if (broken || !swr_read(codec_device, reg, &now)) {
		broken = true;
		return false;
	}
	return (now & mask) == (value & mask) || codec_put(reg, (uint8_t)((now & ~mask) | (value & mask)), mask);
}

/* a command to all devices on the bus (device 15, command id 15: Android's broadcasts) */
static void
broadcast(uint16_t reg, uint8_t value)
{
	swr_write(0xf, reg, value, 0xf);
	note("apollo mic: SoundWire, to all: %#06x <- %#04x\n", reg, value);
	/*
	 * Android waits for the master's "broadcast" completion here, the interrupt it does not have under this
	 * kernel (swrm_cmd_fifo_wr_cmd(), at most 200 ms); 10 ms, as on the POCO, then the bank is looked at
	 */
	sleep_ms(10);
}

/* to all devices, with an ordinary command id (swrm_cmd_fifo_wr_cmd(..., 0xF, 0x00, ...)) */
static void
to_all(uint16_t reg, uint8_t value)
{
	write_id = write_id < 14 ? write_id + 1 : 0;
	swr_write(0xf, reg, value, write_id);
	note("apollo mic: SoundWire, to all: %#06x <- %#04x\n", reg, value);
}

/* wcd938x_bind(), wcd938x_set_micbias_data(), wcd938x_init_reg() (this techpack's) */
static bool
codec_init(void)
{
	uint8_t efuse = 0;
	/* the tree's qcom,cdc-micbias1..4-mv: 2750 mV, wcd938x_get_micb_vout_ctl_val(): (2750 - 1000) / 50 */
	const uint8_t vout = (2750 - 1000) / 50;

	if (!codec_put(WCD_INTR_LEVEL_0, 0, 0xff) || !codec_put(WCD_INTR_LEVEL_0 + 1, 0, 0xff) ||
	    !codec_put(WCD_INTR_LEVEL_0 + 2, 0, 0xff) ||
	    !codec_bits(WCD_ANA_MICB1, 0x3f, vout) || !codec_bits(WCD_ANA_MICB2, 0x3f, vout) ||
	    !codec_bits(WCD_ANA_MICB3, 0x3f, vout) || !codec_bits(WCD_ANA_MICB4, 0x3f, vout) ||
	    !codec_bits(WCD_SLEEP_CTL, 0x0e, 0x0e) || !codec_bits(WCD_SLEEP_CTL, 0x80, 0x80)) {
		return false;
	}
	sleep_ms(1);
	if (!codec_bits(WCD_SLEEP_CTL, 0x40, 0x40)) {
		return false;
	}
	sleep_ms(1);
	if (!codec_bits(WCD_LDORXTX_CONFIG, 0x10, 0x00) || !codec_bits(WCD_BIAS_VBG_FINE_ADJ, 0xf0, 0x80) ||
	    !codec_bits(WCD_ANA_BIAS, 0x80, 0x80) || !codec_bits(WCD_ANA_BIAS, 0x40, 0x40)) {
		return false;
	}
	sleep_ms(10);
	if (!codec_bits(WCD_ANA_BIAS, 0x40, 0x00) || !codec_bits(WCD_HPH_NEW_INT_RDAC_GAIN_CTL, 0xf0, 0x00) ||
	    !codec_bits(WCD_HPH_NEW_INT_RDAC_HD2_CTL_L, 0x1f, 0x15) || !codec_bits(WCD_HPH_NEW_INT_RDAC_HD2_CTL_R, 0x1f, 0x15) ||
	    !codec_bits(WCD_HPH_REFBUFF_UHQA_CTL, 0xc0, 0x80) || !codec_bits(WCD_CDC_DMIC_CTL, 0x02, 0x02) ||
	    !codec_bits(WCD_TXFE_ICTRL_STG2CASC_ULP, 0xff, 0x14) || !codec_bits(WCD_TXFE_ICTRL_STG2MAIN_ULP, 0x1f, 0x08) ||
	    !codec_bits(WCD_TX_REQ_FB_CTL_0, 0xff, 0x55) || !codec_bits(WCD_TX_REQ_FB_CTL_0 + 1, 0xff, 0x44) ||
	    !codec_bits(WCD_TX_REQ_FB_CTL_0 + 2, 0xff, 0x11) || !codec_bits(WCD_TX_REQ_FB_CTL_0 + 3, 0xff, 0x00) ||
	    !codec_bits(WCD_TX_REQ_FB_CTL_0 + 4, 0xff, 0x00) ||
	    !codec_bits(WCD_MICB1_TEST_CTL_1, 0xe0, 0xe0) || !codec_bits(WCD_MICB2_TEST_CTL_1, 0xe0, 0xe0) ||
	    !codec_bits(WCD_MICB3_TEST_CTL_1, 0xe0, 0xe0) || !codec_bits(WCD_MICB4_TEST_CTL_1, 0xe0, 0xe0) ||
	    !codec_bits(WCD_TX_3_4_TEST_BLK_EN2, 0x01, 0x00) || !swr_read(codec_device, WCD_EFUSE_REG_30, &efuse) ||
	    !codec_bits(WCD_SLEEP_CTL, 0x0e, (uint8_t)((efuse & 0x07) << 1)) ||
	    /* (this techpack's wcd938x_init_reg() ends with the surge detection on: 0xC0; the POCO's with it off) */
	    !codec_bits(WCD_HPH_SURGE_HPHLR_SURGE_EN, 0xc0, 0xc0)) {
		return false;
	}
	say("apollo mic: the codec is set up as wcd938x_init_reg() sets it up (efuse 30: %#x)\n", efuse);
	return true;
}

/*
 * Two ways to record, boot-arg apollo_mic_path=N:
 *
 *   1 (the default) Android's own ordinary recording (Xiaomi's ACDB "Tutu" and configs, the phone's getprop: no
 *     Fluence for it; the HAL's SND_DEVICE_IN_HANDSET_MIC, mixer path "handset-mic", ACDB device 4, app type
 *     0x11132, 48 kHz): both microphones -- the main one (ADC1, AMIC1, MIC BIAS1) on decimator 1, the top one
 *     (ADC4 through "ADC4 MUX" INP5, AMIC5, MIC BIAS4) on decimator 0, "ADC1/ADC4 Volume" 6, the decimators at
 *     0 dB --, the port's two channels (channel 0 the top microphone, channel 1 the main one), and the DSP's
 *     processing in the device path: Xiaomi's topology 0x10010315 (mic gain, EANS -- off by its calibration --,
 *     an IIR treble cut, MBDRC) with its calibration. The two channels come up to us as they are, and the one
 *     channel iOS gets is made here (apollo_mic_mix: 0 their average, 1 the main one, 2 the top one). If the
 *     DSP refuses any of it (or anything else of this path fails), it is all undone and path 0 is set up in
 *     its place, for the rest of the run.
 *   0 Xiaomi's "voice-rec-mic": ADC1 alone on decimator 0, "ADC1 Volume" 12, "TX_DEC0 Volume" 87, no processing
 *     (topology none): what was brought up first, kept as it was.
 *
 * The bus and the data ports as Android has them. ADC1 is SoundWire port type ADC1 = 18 and ADC4 is ADC4 = 21
 * (dt-bindings/sound/audio-codec-port-types.h). The tree's qcom,tx_swr_ch_map (wcd938x_parse_port_mapping():
 * port, type, channel mask, rate, master type) puts ADC1 on the codec's port 0 (its DP1), channel mask 1, and
 * ADC4 on the codec's port 1 (its DP2), channel mask 2; the master's qcom,swr-port-mapping puts type 0x12 on
 * its port 2, channel mask 1, and type 0x15 on its port 3, channel mask 2 (swrm_get_master_port()). With no
 * "TX0 MODE"/"TX3 MODE" in Xiaomi's paths the ADCs' mode is 0 (ADC_MODE_INVALID): wcd938x_get_clk_rate() gives
 * each port 9.6 MHz and the codec's TX clock rate is "9.6 MHz" (0, wcd938x_set_swr_clk_rate()); the bus runs at
 * MCLK / 1 (divider 0), the frame 50 rows x 16 columns. The ssp period is swrm_get_ssp_period() of what
 * swrm_update_bus_clk() adds up: one port, 9.6 MHz: 9.6 MHz x 2 / (50 x 16 x 4000) = 6; two ports, 19.2 MHz
 * (swrm_get_clk_div_rate() leaves a sum above 9.6 MHz as it is, and swrm_get_clk_div() of 9.6 / 19.2 is 0):
 * 12 -- as Android computes it for handset-mic. The ports' parameters are kona-port-config.h's
 * tx_frame_params_default by the master's port (index port - 1): port 2 "TX1" si 1, offset1 1, offset2 0, lane
 * 0; port 3 "TX2" si 1, offset1 0, offset2 0, lane 1 (the second data line, LPI GPIO 2); no hstart and hstop.
 * They are written to the codec's ports and to the master's as swrm_copy_data_port_config() writes them, one
 * master port after the other (the codec's with command id 0; no block or word-length controls: those are 0xFF
 * there). Into the bank not in use, then the banks switched, then the other bank.
 */
struct swr_port {
	uint8_t master_port, codec_port, master_mask, codec_mask, si, offset1, offset2, lane;
};
static const struct swr_port swr_ports[2] = {
	{ 2, 0, 0x1, 0x1, 1, 1, 0, 0 },         /* ADC1 (the main microphone) */
	{ 3, 1, 0x2, 0x2, 1, 0, 0, 1 },         /* ADC4 (the top one): path 1 only */
};
static uint32_t mic_path = 1;           /* boot-arg apollo_mic_path=N: 1 Android's handset-mic and its processing, 0 voice-rec-mic */
static uint32_t mic_mix;                /* boot-arg apollo_mic_mix=N (path 1): 0 the two channels' average, 1 the main one, 2 the top one */
static uint32_t pan_setting;            /* boot-arg apollo_mic_pan=N (path 1): the IIR's pan, in place of Xiaomi's 1 (see pp_params()) */
static uint32_t ns_setting;             /* boot-arg apollo_mic_ns=1 (path 1): Xiaomi's noise suppressor on (see pp_params()) */
static uint32_t drc_setting = 1;        /* boot-arg apollo_mic_drc=0 (path 1): Xiaomi's multiband DRC off (see pp_params()) */
static unsigned channels = 1;           /* what the port, the path and the stream carry: 2 on path 1 */
static uint8_t bank_now;
static uint32_t adc_on = 1;             /* boot-arg apollo_mic_adc=0 (a test): the ADCs and their mic biases left off, the ports alone */
static uint32_t test_bits;              /* boot-arg apollo_mic_test=N (tests): 1 no mic bias, 2 the ADCs' inputs held */

static bool
port_config(uint8_t bank, bool first)
{
	/* the codec's side (swrm_apply_port_config() first: the devices' clock halved in that bank, to all, id 0) */
	if (first) {
		to_all(SWRS_SCP_HOST_CLK_DIV2_CTL_BANK(bank), 0x01);
	}
	for (unsigned i = 0; i < (mic_path == 1 ? 2u : 1u); i++) {
		const struct swr_port *p = &swr_ports[i];
		char name[80];

		swr_write(codec_device, SWRS_DP_CHANNEL_ENABLE_BANK(p->codec_port, bank), p->codec_mask, 0);
		swr_write(codec_device, SWRS_DP_SAMPLE_CONTROL_1_BANK(p->codec_port, bank), p->si, 0);
		swr_write(codec_device, SWRS_DP_OFFSET_CONTROL_1_BANK(p->codec_port, bank), p->offset1, 0);
		swr_write(codec_device, SWRS_DP_OFFSET_CONTROL_2_BANK(p->codec_port, bank), p->offset2, 0);
		swr_write(codec_device, SWRS_DP_LANE_CONTROL_BANK(p->codec_port, bank), p->lane, 0);
		note("apollo mic: the codec's data port %u, bank %u: channels %#x on, sample interval %u, offsets %u %u, lane %u\n",
		    p->codec_port + 1, bank, p->codec_mask, p->si, p->offset1, p->offset2, p->lane);
		/* the master's port */
		snprintf(name, sizeof(name), "DP_PORT_CTRL (port %u): channels %#x, offsets %u %u, interval %u", p->master_port,
		    p->master_mask, p->offset1, p->offset2, p->si);
		if (!put(&tx_swr, name, SWRM_DP_PORT_CTRL_BANK(p->master_port, bank),
		    (uint32_t)p->master_mask << 24 | (uint32_t)p->offset2 << 16 | (uint32_t)p->offset1 << 8 | p->si, 0xffffffff)) {
			return false;
		}
		snprintf(name, sizeof(name), "DP_PORT_CTRL_2 (port %u): lane %u", p->master_port, p->lane);
		if (!put(&tx_swr, name, SWRM_DP_PORT_CTRL_2_BANK(p->master_port, bank), p->lane, 0xff)) {
			return false;
		}
		/* (SWR_HSTOP_MAX_VAL << 4 | SWR_HSTART_MIN_VAL; on the POCO it reads 0 back whatever is written: said, not checked) */
		snprintf(name, sizeof(name), "DP_PORT_HCTRL (port %u): hstop 15, hstart 0", p->master_port);
		if (!put(&tx_swr, name, SWRM_DP_PORT_HCTRL_BANK(p->master_port, bank), 0xf0, 0)) {
			return false;
		}
	}
	return true;
}

static bool
datapath_on(void)
{
	uint8_t current = 0, bank;
	uint32_t frame, ssp = mic_path == 1 ? 12 : 6;

	/* wcd938x_tx_swr_ctrl(): the codec's TX clock rate ("9.6 MHz": 0) in the bank about to be used */
	if (!swr_read(codec_device, SWRS_SCP_CONTROL, &current)) {
		return false;
	}
	bank = (current & 0x40) ? 0 : 1;
	if (!codec_bits(WCD_SWR_TX_CLK_RATE, bank ? 0xf0 : 0x0f, 0x00)) {
		return false;
	}
	/* swrm_slvdev_datapath_control() */
	bank = (swr_rd(SWRM_MCP_STATUS) & 1) ? 0 : 1;
	if (!put(&tx_swr, "CPU_INTERRUPT_EN", SWRM_CPU_INTERRUPT_EN, SWRM_INTERRUPT_STATUS_MASK, 0) || !port_config(bank, true)) {
		return false;
	}
	/* 50 rows (1), 16 columns (7), MCLK / 1 (divider 0), the ssp period */
	frame = (swr_rd(SWRM_MCP_FRAME_CTRL_BANK(bank)) & ~0xff07ffu) | 1u << 3 | 7u | 0u << 8 | (ssp - 1) << 16;
	if (!put(&tx_swr, ssp == 12 ? "MCP_FRAME_CTRL (the bank to come): 50 x 16, MCLK / 1, ssp 12" :
	    "MCP_FRAME_CTRL (the bank to come): 50 x 16, MCLK / 1, ssp 6", SWRM_MCP_FRAME_CTRL_BANK(bank), frame, 0xff07ff)) {
		return false;
	}
	broadcast(SWRS_SCP_FRAME_CTRL_BANK(bank), 1u << 3 | 7u);       /* (enable_bank_switch(): the switch itself) */
	bank_now = (swr_rd(SWRM_MCP_STATUS) & 1);
	say("apollo mic: the banks switched: the master is in bank %u now (asked for %u)\n", bank_now, bank);
	if (bank_now != bank || !port_config(bank ? 0 : 1, false)) {
		return false;
	}
	/* and the codec's rate in the other bank */
	if (!codec_bits(WCD_SWR_TX_CLK_RATE, bank ? 0x0f : 0xf0, 0x00)) {
		return false;
	}
	/* (what the codec's ports say they are set to, both banks; its interrupts) */
	for (unsigned i = 0; i < (mic_path == 1 ? 2u : 1u); i++) {
		uint8_t port = swr_ports[i].codec_port;

		for (uint8_t b = 0; b < 2; b++) {
			uint8_t v[6] = { 0 };

			(void)swr_read(codec_device, SWRS_DP_CHANNEL_ENABLE_BANK(port, b), &v[0]);
			(void)swr_read(codec_device, SWRS_DP_SAMPLE_CONTROL_1_BANK(port, b), &v[1]);
			(void)swr_read(codec_device, SWRS_DP_OFFSET_CONTROL_1_BANK(port, b), &v[2]);
			(void)swr_read(codec_device, SWRS_DP_LANE_CONTROL_BANK(port, b), &v[3]);
			(void)swr_read(codec_device, SWRS_SCP_FRAME_CTRL_BANK(b), &v[4]);
			(void)swr_read(codec_device, SWRS_SCP_HOST_CLK_DIV2_CTL_BANK(b), &v[5]);
			note("apollo mic: the codec's DP%u, bank %u: channels %#x, interval %u, offset %u, lane %u; frame %#x, div2 %#x\n",
			    port + 1, b, v[0], v[1], v[2], v[3], v[4], v[5]);
		}
	}
	{
		uint8_t status = 0, control = 0, dp_int = 0, dp_ctl = 0;

		(void)swr_read(codec_device, 0x0040, &status);
		(void)swr_read(codec_device, SWRS_SCP_CONTROL, &control);
		(void)swr_read(codec_device, 0x0100, &dp_int);
		(void)swr_read(codec_device, 0x0102, &dp_ctl);
		note("apollo mic: the codec's SCP int status %#x, control/status %#x; DP1 int status %#x, port control %#x; the "
		    "master's interrupt status %#x\n", status, control, dp_int, dp_ctl, swr_rd(0x200));
	}
	return true;
}

#define WCD_ANA_TX_CH4                  0x3011
#define WCD_CDC_TX_ANA_MODE_2_3         0x3411
#define WCD_TX_NEW_AMIC_MUX_CFG         0x3127

/*
 * "MIC BIAS1" (wcd938x_micbias_control(MICB_ENABLE)), "ADC1 Volume" (12 on path 0, Xiaomi's "voice-rec-mic";
 * 6 on path 1, its "handset-mic"), then ADC1 as wcd938x_codec_enable_adc() and wcd938x_enable_req() set it
 * up: its mode the default one (ADC_MODE_VAL_NORMAL, 3), and its input held only for the moment its mode is
 * written (this techpack's enable_adc() lets go of it at once: wcd938x_tx_channel_config(..., 1), then (..., 0))
 */
static bool
adc1_on(void)
{
	return codec_bits(WCD_CDC_DIG_CLK_CTL, 0xf0, 0xf0) && codec_bits(WCD_CDC_ANA_CLK_CTL, 0x10, 0x10) &&
	       codec_bits(WCD_CDC_ANA_TX_CLK_CTL, 0x01, 0x01) && codec_bits(WCD_MICB1_TEST_CTL_2, 0x01, 0x01) &&
	       codec_bits(WCD_MICB2_TEST_CTL_2, 0x01, 0x01) && codec_bits(WCD_MICB3_TEST_CTL_2, 0x01, 0x01) &&
	       codec_bits(WCD_MICB4_TEST_CTL_2, 0x01, 0x01) && codec_bits(WCD_ANA_MICB1, 0xc0, (test_bits & 1) ? 0x00 : 0x40) &&
	       /* "ADC1 Volume": WCD938X_ANA_TX_CH1 bits 4:0 (SOC_SINGLE_TLV, max 20) */
	       codec_bits(WCD_ANA_TX_CH1, 0x1f, mic_path == 1 ? 6 : 12) &&
	       codec_bits(WCD_CDC_ANA_CLK_CTL, 0x08, 0x08) && codec_bits(WCD_CDC_ANA_CLK_CTL, 0x10, 0x10) &&
	       codec_bits(WCD_CDC_REQ_CTL, 0x02, 0x02) && codec_bits(WCD_CDC_REQ_CTL, 0x01, 0x00) &&
	       codec_bits(WCD_ANA_TX_CH2, 0x40, 0x40) && codec_bits(WCD_CDC_TX_ANA_MODE_0_1, 0x0f, 0x03) &&
	       codec_bits(WCD_CDC_DIG_CLK_CTL, 0x10, 0x10) && codec_bits(WCD_ANA_TX_CH2, 0x40, (test_bits & 2) ? 0x40 : 0x00);
}

/*
 * Path 1: the top microphone. "MIC BIAS4" (the tree's routing: AMIC5 <- MIC BIAS4; the bias registers'
 * common part is already on), "ADC4 MUX" INP5 (WCD938X_TX_NEW_AMIC_MUX_CFG bit 5: 0), "ADC4 Volume" 6
 * (WCD938X_ANA_TX_CH4 bits 4:0), then ADC4 as wcd938x_codec_enable_adc() sets up its channel 3:
 * CDC_ANA_CLK_CTL, its hold (WCD938X_ANA_TX_CH4 bit 5, wcd938x_tx_channel_config()), its mode in
 * TX_ANA_MODE_2_3 bits 7:4 (3, normal), its digital clock (CDC_DIG_CLK_CTL bit 7), the hold let go.
 */
static bool
adc4_on(void)
{
	return codec_bits(WCD_ANA_MICB4, 0xc0, (test_bits & 1) ? 0x00 : 0x40) &&
	       codec_bits(WCD_TX_NEW_AMIC_MUX_CFG, 0x20, 0x00) &&
	       codec_bits(WCD_ANA_TX_CH4, 0x1f, 6) &&
	       codec_bits(WCD_CDC_ANA_CLK_CTL, 0x08, 0x08) && codec_bits(WCD_CDC_ANA_CLK_CTL, 0x10, 0x10) &&
	       codec_bits(WCD_ANA_TX_CH4, 0x20, 0x20) && codec_bits(WCD_CDC_TX_ANA_MODE_2_3, 0xf0, 0x30) &&
	       codec_bits(WCD_CDC_DIG_CLK_CTL, 0x80, 0x80) && codec_bits(WCD_ANA_TX_CH4, 0x20, (test_bits & 2) ? 0x20 : 0x00);
}

/* ---- the TX macro's decimators ---------------------------------------------------------------------------- */

/* bolero-cdc-registers.h: decimator d's at 0x400 + 0x80 d (TX_MACRO_TX_PATH_OFFSET), its input mux at 0x100 + 8 d */
#define TX_ADC_MUX_CFG0(d)              (0x100 + 0x8 * (d))
#define TX_ADC_MUX_CFG1(d)              (0x104 + 0x8 * (d))
#define TX_PATH_CTL(d)                  (0x400 + 0x80 * (d))
#define TX_PATH_CFG0(d)                 (0x404 + 0x80 * (d))
#define TX_VOL_CTL(d)                   (0x40c + 0x80 * (d))
#define TX_PATH_SEC2(d)                 (0x418 + 0x80 * (d))
#define TX_PATH_SEC7(d)                 (0x42c + 0x80 * (d))
#define SMIC_ADC0                       1       /* smic_mux_text: "ZERO", "ADC0", "ADC1", "ADC2", "ADC3", ... */
#define SMIC_ADC3                       4
#define DEC0_VOLUME                     3       /* "TX_DEC0 Volume" 87 (path 0): SOC_SINGLE_SX_TLV(..., -84, 40): 87 - 84 = +3 dB */

static uint32_t found_dec[2][7];
static uint32_t smic_input = SMIC_ADC0; /* boot-arg apollo_mic_input=N (path 0): TX SMIC MUX0's input (1 ADC0, Android's; 0 ZERO, a test) */
static bool dec_taken[2];
static uint8_t dec_input[2];            /* what feeds each decimator in use (its SMIC MUX value) */
static uint8_t hpf_cut_off[2];          /* the decimators' high-pass filters as they were found */

static uint32_t
dec_reg(unsigned d, unsigned i)
{
	static const uint32_t r[7] = { 0x100, 0x104, 0x400, 0x404, 0x40c, 0x418, 0x42c };

	return i < 2 ? r[i] + 0x8 * d : r[i] + 0x80 * d;
}

/*
 * Which bolero this is: its core ID, as bolero_soc_codec_probe() reads it. This techpack takes 02 0E for
 * 2.1 and otherwise keeps the version it gave by the macros' count (1.2 here); what kona's own reads is not
 * known (said). A 2.1 is not what this file was written for: it stops.
 */
static bool
bolero_look(void)
{
	uint32_t id0 = get(&va_macro, "CORE_ID_0", VA_CORE_ID_0) & 0xff, id1 = get(&va_macro, "CORE_ID_1", VA_CORE_ID_1) & 0xff;

	if (id0 == 0x02 && id1 == 0x0e) {
		say("apollo mic: the digital codec says it is bolero 2.1 (core ID %02x %02x): not kona's 1.x: STOPPING HERE\n", id0, id1);
		broken = true;
		return false;
	}
	say("apollo mic: the digital codec's core ID is %02x %02x: bolero 1.x, as this techpack takes it on kona\n", id0, id1);
	return true;
}

/*
 * Decimator d fed from `input' at `volume' (signed dB): tx_macro_init()'s tx_macro_reg_init (TX0_TX_PATH_SEC7,
 * decimator 0's only), the mixer controls' writes (tx_macro_put_dec_enum(): "TX SMIC MUXd" ADC0..3 -- below 5,
 * an analog mic: CFG0 bit 7 cleared --; "TX DECd MUX" SWR_MIC: CFG1's 1; "TX_DECd Volume"),
 * tx_macro_hw_params()' rate (48 kHz: 4), and tx_macro_enable_dec()'s PRE_PMU and POST_PMU.
 */
static bool
dec_on(unsigned d, uint8_t input, uint8_t volume)
{
	char name[96];
	uint32_t cfg0;

	for (unsigned i = 0; i < 7; i++) {
		found_dec[d][i] = get(&tx_macro, "(as found)", dec_reg(d, i));
	}
	dec_taken[d] = true;
	dec_input[d] = input;
	if (d == 0 && !put_bits(&tx_macro, "TX0_TX_PATH_SEC7 (tx_macro_reg_init)", TX_PATH_SEC7(0), 0x3f, 0x0a)) {
		return false;
	}
	snprintf(name, sizeof(name), "ADC_MUX%u_CFG0: %s", d, input == SMIC_ADC0 ? "ADC0 (the main microphone)" :
	    input == SMIC_ADC3 ? "ADC3 (the top microphone)" : "ANOTHER INPUT (apollo_mic_input)");
	if (!put_bits(&tx_macro, "TX_PATH_CFG0: an analog mic", TX_PATH_CFG0(d), 0x80, 0x00) ||
	    !put_bits(&tx_macro, name, TX_ADC_MUX_CFG0(d), 0x0f, input & 0x0f) ||
	    !put_bits(&tx_macro, "ADC_MUX_CFG1: SWR_MIC", TX_ADC_MUX_CFG1(d), 0x03, 0x01)) {
		return false;
	}
	snprintf(name, sizeof(name), "TX%u_TX_VOL_CTL: %+d dB (TX_DEC%u Volume %d)", d, (int)(int8_t)volume, d, 84 + (int)(int8_t)volume);
	if (!put(&tx_macro, name, TX_VOL_CTL(d), volume, 0xff) ||
	    !put_bits(&tx_macro, "TX_PATH_CTL: 48 kHz", TX_PATH_CTL(d), 0x0f, 0x04) ||
	    /* PRE_PMU: the decimator's mode (0), its PGA muted */
	    !put_bits(&tx_macro, "TX_PATH_CFG0: mode 0", TX_PATH_CFG0(d), 0x06, 0x00) ||
	    !put_bits(&tx_macro, "TX_PATH_CTL: muted", TX_PATH_CTL(d), 0x10, 0x10) ||
	    /* POST_PMU: on (an analog mic: the gate not cleared first); the high-pass filter at 150 Hz while it settles */
	    !put_bits(&tx_macro, "TX_PATH_CTL: on", TX_PATH_CTL(d), 0x20, 0x20)) {
		return false;
	}
	cfg0 = get(&tx_macro, "TX_PATH_CFG0", TX_PATH_CFG0(d));
	hpf_cut_off[d] = (cfg0 & 0x60) >> 5;
	if (hpf_cut_off[d] != 2 && !put_bits(&tx_macro, "TX_PATH_CFG0: high-pass 150 Hz", TX_PATH_CFG0(d), 0x60, 2u << 5)) {
		return false;
	}
	/* the filter's gate: 2, then (an analog mic) 1; "6ms delay is required as per HW spec" */
	if (!put_bits(&tx_macro, "TX_PATH_SEC2: filter gate", TX_PATH_SEC2(d), 0x03, 0x02) ||
	    !put_bits(&tx_macro, "TX_PATH_SEC2", TX_PATH_SEC2(d), 0x03, 0x01)) {
		return false;
	}
	sleep_ms(6);
	/* "apply gain after decimator is enabled": the gain written again */
	return put(&tx_macro, "TX_VOL_CTL: written again", TX_VOL_CTL(d), get(&tx_macro, "TX_VOL_CTL", TX_VOL_CTL(d)), 0xff);
}

/* after TX_MACRO_AMIC_UNMUTE_DELAY_MS (100; tx_unmute_delay's 40 is less): the PGA unmuted */
static bool
dec_unmute(unsigned d)
{
	return !dec_taken[d] || put_bits(&tx_macro, "TX_PATH_CTL: unmuted", TX_PATH_CTL(d), 0x10, 0x00);
}

/*
 * after TX_MACRO_AMIC_HPF_DELAY_MS (300), tx_macro_tx_hpf_corner_freq_callback() for an analog mic (queued only
 * if the filter was not at 150 Hz already): the codec's hold on the ADC let go (bolero_clear_amic_tx_hold(), the
 * codec's TX_CH_HOLD_CLEAR: ADC1's ANA_TX_CH2 bit 6, ADC4's ANA_TX_CH4 bit 5), the filter back to where it was,
 * its gate 2, 20 us at 48 kHz, 1
 */
static bool
dec_settled(unsigned d)
{
	bool top = dec_input[d] == SMIC_ADC3;

	if (!dec_taken[d] || hpf_cut_off[d] == 2) {
		return true;
	}
	return codec_bits(top ? WCD_ANA_TX_CH4 : WCD_ANA_TX_CH2, top ? 0x20 : 0x40, (test_bits & 2) ? 0xff : 0x00) &&
	       put_bits(&tx_macro, "TX_PATH_CFG0: high-pass as it was", TX_PATH_CFG0(d), 0x60, (uint32_t)hpf_cut_off[d] << 5) &&
	       put_bits(&tx_macro, "TX_PATH_SEC2: filter gate", TX_PATH_SEC2(d), 0x03, 0x02) &&
	       (spin_us(21), put_bits(&tx_macro, "TX_PATH_SEC2", TX_PATH_SEC2(d), 0x03, 0x01));
}

/* PRE_PMD (muted), POST_PMD (off, mode 0, the mute bit back), then all as it was found */
static void
dec_off(unsigned d)
{
	if (!dec_taken[d]) {
		return;
	}
	broken = false;
	(void)put_bits(&tx_macro, "TX_PATH_CTL: muted", TX_PATH_CTL(d), 0x10, 0x10);
	(void)put_bits(&tx_macro, "TX_PATH_CTL: off", TX_PATH_CTL(d), 0x20, 0x00);
	(void)put_bits(&tx_macro, "TX_PATH_CFG0: mode 0", TX_PATH_CFG0(d), 0x06, 0x00);
	for (unsigned i = 0; i < 7; i++) {
		(void)put(&tx_macro, "(put back as found)", dec_reg(d, i), found_dec[d][i], 0);
	}
	dec_taken[d] = false;
}

/* the decimators of the path: 0 (path 0: ADC0 at +3 dB; path 1: ADC3, the top microphone, at 0 dB), 1 (path 1: ADC0 at 0 dB) */
static bool
decimators_on(void)
{
	if (mic_path != 1) {
		return dec_on(0, (uint8_t)smic_input, DEC0_VOLUME);
	}
	return dec_on(0, SMIC_ADC3, 0) && dec_on(1, SMIC_ADC0, 0);
}

/* ---- the DSP: the codec DMA port, a device path for recording, a stream that is read ------------------ */

#define APR_SVC_ADSP_CORE                       3
#define APR_SVC_ASM                             7
#define APR_SVC_ADM                             8
#define AFE_PORT_CMD_DEVICE_START               0x000100e5u
#define AFE_PORT_CMD_DEVICE_STOP                0x000100e6u
#define AFE_PORT_CMD_SET_PARAM_V2               0x000100efu
#define AFE_MODULE_AUDIO_DEV_INTERFACE          0x0001020cu
#define AFE_PARAM_ID_CODEC_DMA_CONFIG           0x000102b8u     /* (apr_audio-v2.h) */
#define MIC_PORT                                0xb037          /* AFE_PORT_ID_TX_CODEC_DMA_TX_3 */
#define ADM_CMD_SHARED_MEM_MAP_REGIONS          0x00010322u
#define ADM_CMDRSP_SHARED_MEM_MAP_REGIONS       0x00010323u
#define ADM_CMD_SHARED_MEM_UNMAP_REGIONS        0x00010324u
#define ADM_CMD_ADD_TOPOLOGIES                  0x00010335u
#define ADM_CMD_DEVICE_OPEN_V5                  0x00010326u
#define ADM_CMDRSP_DEVICE_OPEN_V5               0x00010329u
#define ADM_CMD_DEVICE_CLOSE_V5                 0x00010327u
#define ADM_CMD_MATRIX_MAP_ROUTINGS_V5          0x00010325u
#define ADM_CMD_SET_PP_PARAMS_V6                0x0001035du
#define ADM_PATH_LIVE_RECORDING                 2
#define ADM_MATRIX_AUDIO_TX                     1
#define ADM_TOPOLOGY_NONE                       0x00010312u
#define ADM_TOPOLOGY_XIAOMI_TX                  0x10010315u     /* "AUDIO_TX_MONO_COPP_MBDRC_V4", Xiaomi's custom one */
#define AVCS_CMD_SHARED_MEM_MAP_REGIONS         0x00012924u     /* (q6core.h) */
#define AVCS_CMDRSP_SHARED_MEM_MAP_REGIONS      0x00012925u
#define AVCS_CMD_SHARED_MEM_UNMAP_REGIONS       0x00012926u
#define AVCS_CMD_REGISTER_TOPOLOGIES            0x00012923u
#define ASM_CMD_SHARED_MEM_MAP_REGIONS          0x00010d92u
#define ASM_CMDRSP_SHARED_MEM_MAP_REGIONS       0x00010d93u
#define ASM_CMD_SHARED_MEM_UNMAP_REGIONS        0x00010d94u
#define ASM_STREAM_CMD_OPEN_READ_V3             0x00010db4u
#define ASM_STREAM_CMD_SET_ENCDEC_PARAM         0x00010c10u
#define ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2      0x00010da3u
#define ASM_SESSION_CMD_RUN_V2                  0x00010daau
#define ASM_DATA_CMD_READ_V2                    0x00010dacu
#define ASM_DATA_EVENT_READ_DONE_V2             0x00010d9au
#define ASM_STREAM_CMD_CLOSE                    0x00010bcdu
#define ASM_TOPOLOGY_NONE                       0x00010c68u
#define ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2      0x00010da5u
#define ASM_SHMEM_4K_POOL                       3               /* (ADSP_MEMORY_MAP_SHMEM8_4K_POOL, for all three services) */
#define MIC_SESSION                             2               /* (the speakers' is 1, hakios.apollo_sound.c) */
#define MIC_SESSION_FALLBACK                    3               /* path 0 after a failed path 1: a session nothing was routed to */
#define MIC_STREAM_PORT                         ((uint16_t)(mic_session << 8 | 1))
#define ADM_CMD_SET_PP_PARAMS_V5                0x00010328u
#define AVCS_CMD_GET_FWK_VERSION                0x0001292cu     /* (apr_audio-v2.h) */
#define AVCS_CMDRSP_GET_FWK_VERSION             0x0001292du
#define READ_TOKEN                              0x40000000u     /* | the period */
#define PCM_CHANNEL_FL                          1
#define PCM_CHANNEL_FR                          2
#define PCM_CHANNEL_FC                          3
#define MIC_RATE                                48000
#define MIC_PERIODS                             8
#define MIC_PERIOD_FRAMES                       960             /* 20 ms; one channel (path 0) or two (path 1) of 16 bits */
#define MIC_PERIOD_BYTES                        (MIC_PERIOD_FRAMES * 2 * channels)
#define TOPOLOGY_PAGE                           MIC_PERIODS     /* the page of our DSP memory behind the periods: the topology */
#define CAL_PAGE                                (MIC_PERIODS + 1)       /* ... and the one behind it: the device path's calibration */
#define APR_PAYLOAD_MOST                        256             /* apollo_adsp_apr()'s cap: a packet of sizeof(struct apr_header) + 256 */

/*
 * Xiaomi's calibration data, from its ACDB (/vendor/etc/acdbdata/Tutu/Tutu_Global_cal.acdb and Tutu_Handset_cal.acdb
 * on the phone). It is Xiaomi's, so it is not in this repository: devices/mi-10t/pull-firmware.sh pulls the two
 * files and devices/mi-10t/tools/acdb-mic.py writes these four out of them into fw-audio-apollo/, which goes into
 * the image's fw/ (userland/firmware.sh). They are read at path 1's start (mic_fw_load()), checked, and kept for
 * the run; if any is missing or not what it should be, path 0 is used.
 *   mic_topology_v2.bin      global property 0x132ac: the definition of topology 0x10010315 alone (count 1), the
 *                            AVCS REGISTER_TOPOLOGIES v2 form: version 2, the topology, config 1, 4 modules with
 *                            their instance IDs: 0x10c35 (mic gain), 0x1091f (EANS), 0x10c02 instance 0x8000 (IIR),
 *                            0x10943 (MBDRC v4); 52 bytes
 *   mic_topology_v1.bin      global property 0x131a7: the same, v1 form (no instance IDs); 36 bytes
 *   mic_topology_legacy.bin  global property 0x12e47: the same, ADM_CMD_ADD_TOPOLOGIES' form (16 module slots of 6
 *                            words); 396 bytes
 *   mic_audproc.bin          Tutu_Handset_cal.acdb, the ADM's processing calibration for device 4 ("HANDSET_MIC"),
 *                            app type 0x11132 (GENERAL_RECORDING), 48 kHz: 9 parameters, each param_hdr_v3 (module,
 *                            instance, reserved, parameter, size) and its data, as ADM_CMD_SET_PP_PARAMS_V6 takes
 *                            them; 504 bytes
 * (No audio volume calibration for device 4, no AFE calibration for the TX port.)
 */
#define MIC_FW_MOST                             3072    /* what any of them may be (the calibration goes into one page) */
struct mic_fw {
	const char *name;
	const uint8_t *data;
	uint64_t size;
};
static struct mic_fw fw_topology_v2 = { "mic_topology_v2.bin", NULL, 0 }, fw_topology_v1 = { "mic_topology_v1.bin", NULL, 0 },
    fw_topology_legacy = { "mic_topology_legacy.bin", NULL, 0 }, fw_audproc = { "mic_audproc.bin", NULL, 0 };
static bool fw_loaded;
static uintptr_t mic_memory;
static uint64_t mic_address;
static bool memory_taken, mic_mapped, mic_copp_open, mic_stream_open, mic_port_on, topology_registered;
static uint32_t mic_session = MIC_SESSION;
static int topology_version;            /* the definition the DSP took: 2 (with instance IDs), 1 (without), 0 (the ADM's legacy one) */
static bool fwk_asked;
static bool cal_mapped;                 /* the calibration's page mapped for the ADM: unmapped when the path is taken down */
static uint32_t cal_handle;
static bool restart_port;               /* path 0 after a failed path 1: the port stopped once more before it is set up */
static volatile uint32_t first_read_ms, first_read_status, first_read_bytes, first_error_status;
static uint64_t listen_began;
static uint32_t mic_map_handle, mic_copp;
static volatile uint32_t read_ready, read_errors, reads_done;
static volatile uint32_t read_sizes[MIC_PERIODS];

static bool
mic_events(uint8_t service, __unused uint16_t port, uint32_t opcode, uint32_t token, const uint32_t *payload, uint32_t length)
{
	if (service != APR_SVC_ASM || !(token & READ_TOKEN) || (token & 0x3fffffff) >= MIC_PERIODS) {
		return false;
	}
	if (reads_done == 0 && first_read_ms == 0) {
		uint64_t ns;

		/* (the first answer to a read, whatever it is: said by listen()) */
		absolutetime_to_nanoseconds(mach_absolute_time() - listen_began, &ns);
		first_read_ms = (uint32_t)(ns / NSEC_PER_MSEC) + 1;
		first_read_status = opcode == ASM_DATA_EVENT_READ_DONE_V2 ? (length >= 4 ? payload[0] : 0xffffffffu) :
		    (length >= 8 ? payload[1] : 0xffffffffu);
		first_read_bytes = opcode == ASM_DATA_EVENT_READ_DONE_V2 && length >= 24 ? payload[4] : 0;
	}
	if (opcode == ASM_DATA_EVENT_READ_DONE_V2) {
		if (length < 24 || payload[0] != 0) {
			if (read_errors == 0) {
				first_error_status = length >= 4 ? payload[0] : 0xffffffffu;
			}
			OSIncrementAtomic((volatile SInt32 *)&read_errors);
		}
		read_sizes[token & 0x3fffffff] = length >= 24 ? payload[4] : 0;
		OSIncrementAtomic((volatile SInt32 *)&reads_done);
		OSBitOrAtomic(1u << (token & 31), &read_ready);
		return true;
	}
	if (opcode == APR_BASIC_RSP_RESULT) {
		if (read_errors == 0) {
			first_error_status = length >= 8 ? payload[1] : 0xffffffffu;
		}
		OSIncrementAtomic((volatile SInt32 *)&read_errors);
		OSBitOrAtomic(1u << (token & 31), &read_ready);
		read_sizes[token & 0x3fffffff] = 0;
		return true;
	}
	return false;
}

static bool
mic_read(unsigned period)
{
	uint32_t read[5] = { (uint32_t)(mic_address + 4096 * period), (uint32_t)((mic_address + 4096 * period) >> 32),
		             mic_map_handle, MIC_PERIOD_BYTES, period };

	return apollo_adsp_apr(APR_SVC_ASM, MIC_STREAM_PORT, READ_TOKEN | period, ASM_DATA_CMD_READ_V2, read, sizeof(read), 0, NULL);
}

static bool
afe_port_param(uint32_t module, uint32_t param, const void *data, uint16_t size, const char *what)
{
	uint8_t packet[16 + sizeof(struct afe_param) + 64];
	struct afe_param *p = (struct afe_param *)(packet + 16);

	bzero(packet, sizeof(packet));
	*(uint16_t *)packet = MIC_PORT;
	*(uint16_t *)(packet + 2) = (uint16_t)(sizeof(*p) + size);
	p->module = module;
	p->param = param;
	p->size = size;
	memcpy(packet + 16 + sizeof(*p), data, size);
	return command(what, APR_SVC_AFE, AFE_PORT_CMD_SET_PARAM_V2, packet, 16 + (uint32_t)sizeof(*p) + size, 0, NULL);
}

/*
 * A command to a service's port, its basic answer looked at: the ADM's commands to a device path go to the
 * path's COPP as the port (q6adm.c: dest_port = the COPP's id). true only if done.
 */
static bool
port_command(const char *what, uint8_t service, uint16_t port, uint32_t opcode, const void *payload, uint32_t length)
{
	struct hakios_apr_answer answer;

	if (!apollo_adsp_apr(service, port, 0, opcode, payload, length, APR_WAIT_MS, &answer)) {
		say("apollo mic: %s (opcode %#x): NO ANSWER\n", what, opcode);
		return false;
	}
	if (answer.opcode == APR_BASIC_RSP_RESULT && answer.words[0] == opcode) {
		if (answer.words[1] == 0) {
			note("apollo mic: %s: done\n", what);
			return true;
		}
		say("apollo mic: %s: REFUSED: %s\n", what, dsp_error(answer.words[1]));
		return false;
	}
	say("apollo mic: %s: an answer not expected: opcode %#x, %u bytes: %#x %#x\n", what, answer.opcode, answer.length,
	    answer.words[0], answer.words[1]);
	return false;
}

/*
 * One of the four files of Xiaomi's data: fw/ of the small image (hakios_image_file(): the image's own memory), else
 * /fw on the root (apollo_adsp_fw_file(): ours); kept for the run either way.
 */
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);
static bool
mic_fw_file(struct mic_fw *f)
{
	f->data = hakios_image_file("fw", f->name, &f->size, NULL);
	if (f->data == NULL) {
		f->data = apollo_adsp_fw_file(f->name, &f->size);
	}
	if (f->data == NULL) {
		say("apollo mic: fw/%s is NOT THERE (neither in the image's fw/ nor in /fw): it comes from the phone's ACDB -- "
		    "devices/mi-10t/pull-firmware.sh, then userland/firmware.sh\n", f->name);
		return false;
	}
	if (f->size < 8 || f->size > MIC_FW_MOST || (f->size & 3) != 0) {
		say("apollo mic: fw/%s: %llu bytes, NOT A SIZE IT CAN HAVE\n", f->name, f->size);
		return false;
	}
	return true;
}

/* a topology definition file: one topology (count 1), 0x10010315, whose definition ends exactly at the file's end */
static bool
mic_fw_topology(const struct mic_fw *f, int kind)
{
	const uint32_t *w = (const uint32_t *)f->data;
	uint32_t words = (uint32_t)(f->size / 4), topology, end;

	if (kind == 0) {                /* legacy: count, topology, modules, 16 slots of 6 words */
		topology = w[1];
		end = 1 + 2 + 16 * 6;
	} else {                        /* v1 / v2: count, version, topology, config, modules, modules' words */
		topology = words >= 5 ? w[2] : 0;
		end = words >= 5 ? 5 + (kind == 2 ? 2 : 1) * w[4] : ~0u;
	}
	if (w[0] != 1 || topology != ADM_TOPOLOGY_XIAOMI_TX || end != words) {
		say("apollo mic: fw/%s: NOT ONE DEFINITION OF TOPOLOGY 0x10010315 (count %u, topology %#x, %u bytes for %u): not used\n",
		    f->name, w[0], topology, (unsigned)f->size, end == ~0u ? 0 : 4 * end);
		return false;
	}
	return true;
}

/* the calibration file: param_hdr_v3 headers and their data, walked exactly to the file's end */
static bool
mic_fw_audproc(const struct mic_fw *f)
{
	uint32_t at = 0, count = 0;

	while (at + 16 <= f->size) {
		uint32_t size = *(const uint32_t *)(f->data + at + 12);

		if ((size & 3) != 0 || size > f->size - at - 16) {
			break;
		}
		at += 16 + size;
		count++;
	}
	if (at != f->size || count == 0) {
		say("apollo mic: fw/%s: its parameters do not walk to its end (%u parameters, %u of %u bytes): not used\n", f->name,
		    count, at, (unsigned)f->size);
		return false;
	}
	say("apollo mic: fw/%s: %u parameters, %u bytes\n", f->name, count, at);
	return true;
}

/* All four, once a run: false (and said) if any is missing or not what it should be: path 1 is then not taken. */
static bool
mic_fw_load(void)
{
	if (fw_loaded) {
		return true;
	}
	fw_loaded = mic_fw_file(&fw_topology_v2) && mic_fw_topology(&fw_topology_v2, 2) &&
	    mic_fw_file(&fw_topology_v1) && mic_fw_topology(&fw_topology_v1, 1) &&
	    mic_fw_file(&fw_topology_legacy) && mic_fw_topology(&fw_topology_legacy, 0) &&
	    mic_fw_file(&fw_audproc) && mic_fw_audproc(&fw_audproc);
	if (fw_loaded) {
		say("apollo mic: Xiaomi's topology and calibration for the device path are in fw/ (from the phone's ACDB)\n");
	}
	return fw_loaded;
}

/*
 * One way of making topology 0x10010315 known to the DSP, out of band: the definition copied into the page of our
 * DSP memory behind the periods (cleaned out of our caches: the DSP reads memory), that page mapped for the
 * service (pool 3, as q6core_map_memory_regions() and the ADM's remap_cal_data() map it), the registration
 * pointing at it, the page unmapped again (q6core_send_custom_topologies(), send_adm_custom_topology()).
 */
static bool
topology_try(const char *how, uint8_t service, uint32_t map_op, uint32_t map_rsp, uint32_t register_op, uint32_t unmap_op,
    const uint8_t *blob, uint32_t size)
{
	struct {
		uint16_t pool, regions;
		uint32_t property, low, high, size;
	} __attribute__((packed)) map = { ASM_SHMEM_4K_POOL, 1, 0, 0, 0, 4096 };
	uint64_t address = mic_address + 4096 * TOPOLOGY_PAGE;
	uint32_t registration[4], handle;
	struct hakios_apr_answer answer;
	uint8_t *page = (uint8_t *)(mic_memory + 4096 * TOPOLOGY_PAGE);
	char what[120];
	bool done;

	bzero(page, 4096);
	memcpy(page, blob, size);
	for (vm_offset_t line = (vm_offset_t)page; line < (vm_offset_t)page + 4096; line += 64) {
		__asm__ volatile ("dc cvac, %0" : : "r" (line));        /* the DSP reads memory, not our caches */
	}
	__asm__ volatile ("dsb sy");
	map.low = (uint32_t)address;
	map.high = (uint32_t)(address >> 32);
	snprintf(what, sizeof(what), "the topology's page, mapped for the %s", how);
	if (!command(what, service, map_op, &map, sizeof(map), map_rsp, &answer)) {
		return false;
	}
	handle = answer.words[0];
	registration[0] = (uint32_t)address;
	registration[1] = (uint32_t)(address >> 32);
	registration[2] = handle;
	registration[3] = size;
	snprintf(what, sizeof(what), "topology 0x10010315 registered: %s, %u bytes at %#llx, handle %#x", how, size, address, handle);
	done = port_command(what, service, 0, register_op, registration, sizeof(registration));
	snprintf(what, sizeof(what), "the topology's page, unmapped for the %s", how);
	(void)command(what, service, unmap_op, &handle, sizeof(handle), 0, NULL);
	return done;
}

/* Xiaomi's topology made known to the DSP once a run: AVCS v2, else v1, else the ADM's legacy command. */
static bool
topology_register(void)
{
	if (topology_registered) {
		return true;
	}
	topology_version = 2;
	topology_registered =
	    topology_try("core service (AVCS, v2 definition)", APR_SVC_ADSP_CORE, AVCS_CMD_SHARED_MEM_MAP_REGIONS,
	    AVCS_CMDRSP_SHARED_MEM_MAP_REGIONS, AVCS_CMD_REGISTER_TOPOLOGIES, AVCS_CMD_SHARED_MEM_UNMAP_REGIONS, fw_topology_v2.data,
	    (uint32_t)fw_topology_v2.size) ||
	    (topology_version = 1, topology_try("core service (AVCS, v1 definition)", APR_SVC_ADSP_CORE, AVCS_CMD_SHARED_MEM_MAP_REGIONS,
	    AVCS_CMDRSP_SHARED_MEM_MAP_REGIONS, AVCS_CMD_REGISTER_TOPOLOGIES, AVCS_CMD_SHARED_MEM_UNMAP_REGIONS, fw_topology_v1.data,
	    (uint32_t)fw_topology_v1.size)) ||
	    (topology_version = 0, topology_try("ADM (legacy ADD_TOPOLOGIES)", APR_SVC_ADM, ADM_CMD_SHARED_MEM_MAP_REGIONS, ADM_CMDRSP_SHARED_MEM_MAP_REGIONS,
	    ADM_CMD_ADD_TOPOLOGIES, ADM_CMD_SHARED_MEM_UNMAP_REGIONS, fw_topology_legacy.data,
	    (uint32_t)fw_topology_legacy.size));
	say("apollo mic: Xiaomi's topology 0x10010315 %s\n", topology_registered ? "IS KNOWN TO THE DSP" :
	    "COULD NOT BE REGISTERED in any of the three ways");
	return topology_registered;
}

/*
 * What the DSP says it is (q6core_send_get_avcs_fwk_ver_cmd(): AVCS_CMD_GET_FWK_VERSION, no payload; its answer
 * avcs_get_fwk_version, then each service's ID and API version), asked once a run and said, for the log.
 */
static void
fwk_version(void)
{
	struct hakios_apr_answer answer;
	uint32_t services;

	if (fwk_asked) {
		return;
	}
	fwk_asked = true;
	if (!command("the DSP's framework version (AVCS_CMD_GET_FWK_VERSION)", APR_SVC_ADSP_CORE, AVCS_CMD_GET_FWK_VERSION, NULL, 0,
	    AVCS_CMDRSP_GET_FWK_VERSION, &answer)) {
		return;
	}
	services = answer.words[4];
	say("apollo mic: the DSP's AVS build %u.%u.%u.%u, %u services:\n", answer.words[0], answer.words[1], answer.words[2],
	    answer.words[3], services);
	for (uint32_t i = 0; i < services && 5 + 3 * i + 2 < 64 && 4 * (5 + 3 * i + 3) <= answer.length; i++) {
		say("apollo mic:   service %#x: API version %u (branch %u)\n", answer.words[5 + 3 * i], answer.words[6 + 3 * i],
		    answer.words[7 + 3 * i]);
	}
}

/*
 * Xiaomi's calibration of the device path (Android's send_adm_cal() after the routing), in band, one parameter a
 * command, each answered and its status said. adm_set_pp_params() sends ADM_CMD_SET_PP_PARAMS_V6 with
 * param_hdr_v3 headers (module, instance, reserved, parameter, size) when instance IDs are supported
 * (q6common_is_instance_id_supported(), which Android's HAL sets through the mixer control "Instance ID
 * Support"), else ADM_CMD_SET_PP_PARAMS_V5 with param_hdr_v1 headers (module, parameter, size in 16 bits,
 * reserved; q6common_pack_pp_params()). Here: V6 if the DSP took the topology's v2 definition (the one with
 * instance IDs), V5 otherwise -- and V5 also if V6 is refused. The command: a memory header of zeros (address
 * low, high, handle: in band), the parameters' size (the header's 16 bytes not counted), the parameters; to
 * the COPP as the port (dest_port = the COPP's id), its answer APR_BASIC_RSP_RESULT { opcode, status }.
 *
 * The one change: the IIR's pan (asm_iir_filter_config_params: the stages' count, their 5 coefficients each,
 * then each stage's shift factor and pan, 16 bits each). Xiaomi's is 1, which puts its treble cut on one channel
 * only; Android records that way and the HAL keeps one channel, but here both channels are kept to the end and
 * averaged into iOS's one channel (apollo_mic_mix=0), and a filter on one of the two would make the average two
 * different microphones' sounds through two different responses. So it is 0 (apollo_mic_pan=N: another),
 * which in Qualcomm's IIR is both channels -- not confirmed by any source here: said in the log, and
 * apollo_mic_pan=1 gives Xiaomi's own back.
 */
static int
pp_one(bool v6, const uint8_t *param, uint32_t number)
{
	uint32_t module = *(const uint32_t *)param, instance = *(const uint16_t *)(param + 4),
	    id = *(const uint32_t *)(param + 8), size = *(const uint32_t *)(param + 12);
	uint32_t packet[APR_PAYLOAD_MOST / 4], length;
	struct hakios_apr_answer answer;
	uint32_t opcode = v6 ? ADM_CMD_SET_PP_PARAMS_V6 : ADM_CMD_SET_PP_PARAMS_V5;

	length = (v6 ? 16 : 12) + size;
	if (16 + length > APR_PAYLOAD_MOST) {
		say("apollo mic: calibration parameter %u (module %#x parameter %#x, %u bytes) does not fit one command\n", number,
		    module, id, size);
		return -1;
	}
	bzero(packet, sizeof(packet));
	packet[3] = length;                     /* (packet[0..2]: address and handle 0: in band) */
	if (v6) {
		memcpy(&packet[4], param, 16 + size);
	} else {
		packet[4] = module;
		packet[5] = id;
		packet[6] = size & 0xffff;      /* (size, 16 bits; reserved 0) */
		memcpy(&packet[7], param + 16, size);
	}
	if (!apollo_adsp_apr(APR_SVC_ADM, (uint16_t)mic_copp, 0, opcode, packet, 16 + length, APR_WAIT_MS, &answer)) {
		say("apollo mic: calibration %s, parameter %u (module %#x instance %#x parameter %#x, %u bytes): NO ANSWER (or not "
		    "sent: the kernel's log says which)\n", v6 ? "V6" : "V5", number, module, instance, id, size);
		return -1;
	}
	if (answer.opcode != APR_BASIC_RSP_RESULT || answer.words[0] != opcode) {
		say("apollo mic: calibration %s, parameter %u: an answer not expected: opcode %#x, %u bytes: %#x %#x\n", v6 ? "V6" : "V5",
		    number, answer.opcode, answer.length, answer.words[0], answer.words[1]);
		return -1;
	}
	say("apollo mic: calibration %s, parameter %u (module %#x instance %#x parameter %#x, %u bytes): %s\n", v6 ? "V6" : "V5",
	    number, module, instance, id, size, answer.words[1] == 0 ? "done" : dsp_error(answer.words[1]));
	return (int)answer.words[1];
}

/*
 * The calibration as Android sends it (send_adm_cal(): out of band): the whole of it in a page of our DSP memory
 * (the one behind the topology's), that page mapped for the ADM (adm_memory_map_regions():
 * ADM_CMD_SHARED_MEM_MAP_REGIONS, pool 3, its handle in ADM_CMDRSP_SHARED_MEM_MAP_REGIONS), then one
 * SET_PP_PARAMS with the memory header { address low, high, handle } and the parameters' size, nothing in the
 * packet itself: a command of 32 bytes, which fits the DSP's receive intents (the in-band one of 172 bytes did
 * not: GLINK had no intent of the DSP's that size). V6 with the v3 headers as the ACDB has them; V5 with each
 * header made v1 (module, parameter, 16-bit size, reserved). The page stays mapped until the path is taken down.
 */
static bool
pp_out_of_band(const uint8_t *blob, uint32_t size, bool v6)
{
	struct {
		uint16_t pool, regions;
		uint32_t property, low, high, size;
	} __attribute__((packed)) map = { ASM_SHMEM_4K_POOL, 1, 0, 0, 0, 4096 };
	uint64_t address = mic_address + 4096 * CAL_PAGE;
	uint8_t *page = (uint8_t *)(mic_memory + 4096 * CAL_PAGE);
	uint32_t command_words[4], length = 0;
	struct hakios_apr_answer answer;
	uint32_t opcode = v6 ? ADM_CMD_SET_PP_PARAMS_V6 : ADM_CMD_SET_PP_PARAMS_V5;
	char what[160];

	bzero(page, 4096);
	if (v6) {
		memcpy(page, blob, size);
		length = size;
	} else {
		for (uint32_t at = 0; at + 16 <= size; at += 16 + *(const uint32_t *)(blob + at + 12)) {
			uint32_t one = *(const uint32_t *)(blob + at + 12);

			*(uint32_t *)(page + length) = *(const uint32_t *)(blob + at);
			*(uint32_t *)(page + length + 4) = *(const uint32_t *)(blob + at + 8);
			*(uint32_t *)(page + length + 8) = one & 0xffff;
			memcpy(page + length + 12, blob + at + 16, one);
			length += 12 + one;
		}
	}
	for (vm_offset_t line = (vm_offset_t)page; line < (vm_offset_t)page + 4096; line += 64) {
		__asm__ volatile ("dc cvac, %0" : : "r" (line));        /* the DSP reads memory, not our caches */
	}
	__asm__ volatile ("dsb sy");
	map.low = (uint32_t)address;
	map.high = (uint32_t)(address >> 32);
	if (!command("the calibration's page, mapped for the ADM", APR_SVC_ADM, ADM_CMD_SHARED_MEM_MAP_REGIONS, &map, sizeof(map),
	    ADM_CMDRSP_SHARED_MEM_MAP_REGIONS, &answer)) {
		return false;
	}
	cal_mapped = true;
	cal_handle = answer.words[0];
	command_words[0] = (uint32_t)address;
	command_words[1] = (uint32_t)(address >> 32);
	command_words[2] = cal_handle;
	command_words[3] = length;
	snprintf(what, sizeof(what), "Xiaomi's calibration of COPP %u, out of band (%s, %u bytes at %#llx, handle %#x)", mic_copp,
	    v6 ? "SET_PP_PARAMS_V6" : "SET_PP_PARAMS_V5", length, address, cal_handle);
	if (!apollo_adsp_apr(APR_SVC_ADM, (uint16_t)mic_copp, 0, opcode, command_words, sizeof(command_words), APR_WAIT_MS, &answer)) {
		say("apollo mic: %s: NO ANSWER (or not sent: the kernel's log says which)\n", what);
		return false;
	}
	if (answer.opcode != APR_BASIC_RSP_RESULT || answer.words[0] != opcode) {
		say("apollo mic: %s: an answer not expected: opcode %#x, %u bytes: %#x %#x\n", what, answer.opcode, answer.length,
		    answer.words[0], answer.words[1]);
		return false;
	}
	say("apollo mic: %s: %s\n", what, answer.words[1] == 0 ? "DONE" : dsp_error(answer.words[1]));
	return answer.words[1] == 0;
}

static bool
pp_params(void)
{
	static uint8_t blob_space[MIC_FW_MOST];         /* (Xiaomi's calibration, copied to be changed: pan, noise suppressor, DRC) */
	uint8_t *blob = blob_space;
	const uint32_t blob_size = (uint32_t)fw_audproc.size;
	uint32_t at = 0, number = 0;
	bool v6 = topology_version == 2;

	memcpy(blob, fw_audproc.data, blob_size);
	while (at + 16 <= blob_size) {
		uint32_t module = *(uint32_t *)(blob + at), param = *(uint32_t *)(blob + at + 8), size = *(uint32_t *)(blob + at + 12);

		if (module == 0x10c02 && param == 0x10c05 && size >= 4) {
			uint32_t stages = *(uint16_t *)(blob + at + 16);
			uint32_t pan_at = at + 16 + 4 + 20 * stages + 2 * stages;

			for (uint32_t s = 0; s < stages && pan_at + 2 * s + 2 <= at + 16 + size; s++) {
				say("apollo mic: the IIR's stage %u: pan %u, made %u\n", s, *(uint16_t *)(blob + pan_at + 2 * s), pan_setting);
				*(uint16_t *)(blob + pan_at + 2 * s) = (uint16_t)pan_setting;
			}
		}
		/*
		 * apollo_mic_ns=1: the noise suppressor in Xiaomi's chain (EANS, module 0x1091f) switched on, with Xiaomi's own tuning
		 * (parameter 0x10920, sent as it is). Their calibration for ordinary recording has it off (ADM_PARAM_ID_EANS_ENABLE
		 * 0x10c4b = 0): Android hides the microphones' hiss (about 30 dB under speech here, both microphones alike, 28 October)
		 * later, in the apps. Tuned for calls: music may come out a little watery.
		 */
		if (module == 0x1091f && param == 0x10c4b && size >= 4 && ns_setting) {
			say("apollo mic: the noise suppressor (EANS): enable %u, made 1 (apollo_mic_ns=1)\n", *(uint32_t *)(blob + at + 16));
			*(uint32_t *)(blob + at + 16) = 1;
		}
		/*
		 * apollo_mic_drc=0: Xiaomi's multiband DRC (module 0x10943, its enable 0x10944 { mode, enable }) off. With the
		 * suppressor on, the hiss came back in every pause of speech (28 October): the DRC's make-up gain lifts what is
		 * quiet, the room's hiss with it.
		 */
		if (module == 0x10943 && param == 0x10944 && size >= 8 && !drc_setting) {
			say("apollo mic: the multiband DRC: enable %u, made 0 (apollo_mic_drc=0)\n", *(uint32_t *)(blob + at + 20));
			*(uint32_t *)(blob + at + 20) = 0;
		}
		at += 16 + size;
	}
	if (pp_out_of_band(blob, blob_size, v6)) {
		return true;
	}
	say("apollo mic: Xiaomi's calibration of the device path (COPP %u), in band instead: %s, one parameter a command\n", mic_copp,
	    v6 ? "ADM_CMD_SET_PP_PARAMS_V6 (the topology's v2 definition was taken: instance IDs)" :
	    "ADM_CMD_SET_PP_PARAMS_V5 (no instance IDs)");
	for (at = 0; at + 16 <= blob_size; at += 16 + *(uint32_t *)(blob + at + 12)) {
		int status = pp_one(v6, blob + at, ++number);

		if (status != 0 && v6 && number == 1 && status > 0) {
			/* refused (not lost): this DSP may not take V6: V5 for all of them */
			say("apollo mic: V6 refused: the calibration sent as V5 instead\n");
			v6 = false;
			status = pp_one(v6, blob + at, number);
		}
		if (status != 0) {
			return false;
		}
	}
	say("apollo mic: Xiaomi's calibration of the device path is in: %u parameters, %u bytes\n", number, (unsigned)blob_size);
	return true;
}

/*
 * the stream and the path, opened, mapped and routed (msm-pcm-q6 and q6adm/q6asm at a recording's open and
 * prepare), in our memory behind the speakers' periods (apollo_audio_dsp_memory(): 16 pages, the SMMU's
 * stream 0x1801, set up by whichever of the two comes first): the periods in the first 8, the topology's
 * definition (path 1) in the 9th
 */
static bool
dsp_path_on(void)
{
	struct {
		uint16_t pool, regions;
		uint32_t property, low, high, size;
	} __attribute__((packed)) map = { ASM_SHMEM_4K_POOL, 1, 0, 0, 0, 0 };
	struct {
		uint16_t flags, mode, endpoint1, endpoint2;
		uint32_t topology;
		uint16_t channels, bits;
		uint32_t rate;
		uint8_t map[8];
	} __attribute__((packed)) open = { 0, ADM_PATH_LIVE_RECORDING, MIC_PORT, 0xffff, ADM_TOPOLOGY_NONE, 1, 16, MIC_RATE,
		                           { PCM_CHANNEL_FC } };
	struct {
		uint32_t mode, endpoint, topology, format;
		uint16_t bits, reserved;
	} __attribute__((packed)) reading = { 0, 0, ASM_TOPOLOGY_NONE, ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2, 16, 0 };
	struct {
		uint32_t param, size, frames_per_buffer, block_size;
		uint16_t channels, bits;
		uint32_t rate;
		uint16_t is_signed, reserved;
		uint8_t map[8];
	} __attribute__((packed)) encoder = { ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2, 28, 0, 20, 1, 16, MIC_RATE, 1, 0, { PCM_CHANNEL_FC } };
	struct {
		uint32_t matrix, sessions;
		uint16_t session, copps;
		uint16_t copp[2];
	} __attribute__((packed)) route = { ADM_MATRIX_AUDIO_TX, 1, (uint16_t)mic_session, 1, { 0, 0 } };
	struct hakios_apr_answer answer;
	uint32_t bytes = 0;

	if (mic_path == 1) {
		/* two channels, left and right: the port's channel 0 (decimator 0, the top microphone) and 1 (the main one) */
		open.topology = ADM_TOPOLOGY_XIAOMI_TX;
		open.channels = 2;
		open.map[0] = PCM_CHANNEL_FL;
		open.map[1] = PCM_CHANNEL_FR;
		encoder.channels = 2;
		encoder.map[0] = PCM_CHANNEL_FL;
		encoder.map[1] = PCM_CHANNEL_FR;
	}
	if (!apollo_audio_dsp_memory(&mic_memory, &mic_address, &bytes)) {
		say("apollo mic: no memory the DSP can see\n");
		return false;
	}
	memory_taken = true;
	if (bytes < (CAL_PAGE + 1) * 4096) {
		say("apollo mic: the memory the DSP can see is too small (%u bytes)\n", bytes);
		return false;
	}
	map.low = (uint32_t)mic_address;
	map.high = (uint32_t)(mic_address >> 32);
	map.size = MIC_PERIODS * 4096;
	apollo_adsp_apr_events_second(mic_events);
	if (!command("our periods for the microphone, mapped for the DSP", APR_SVC_ASM, ASM_CMD_SHARED_MEM_MAP_REGIONS, &map,
	    sizeof(map), ASM_CMDRSP_SHARED_MEM_MAP_REGIONS, &answer)) {
		return false;
	}
	mic_mapped = true;
	mic_map_handle = answer.words[0];
	say("apollo mic: our %u KB at %#llx are the DSP's handle %#x\n", MIC_PERIODS * 4, mic_address, mic_map_handle);
	if (!apollo_adsp_apr(APR_SVC_ASM, MIC_STREAM_PORT, 0, ASM_STREAM_CMD_OPEN_READ_V3, &reading, sizeof(reading), APR_WAIT_MS,
	    &answer) || answer.opcode != APR_BASIC_RSP_RESULT || answer.words[1] != 0) {
		say("apollo mic: the stream, opened for reading: %s (%#x %#x)\n", "REFUSED OR NO ANSWER", answer.opcode, answer.words[1]);
		return false;
	}
	mic_stream_open = true;
	say("apollo mic: the stream (session %u), opened for reading 16-bit linear PCM, no processing: done\n", mic_session);
	if (!apollo_adsp_apr(APR_SVC_ASM, MIC_STREAM_PORT, 0, ASM_STREAM_CMD_SET_ENCDEC_PARAM, &encoder, sizeof(encoder), APR_WAIT_MS,
	    &answer) || answer.opcode != APR_BASIC_RSP_RESULT || answer.words[1] != 0) {
		say("apollo mic: the stream's format: REFUSED OR NO ANSWER (%#x %#x)\n", answer.opcode, answer.words[1]);
		return false;
	}
	say("apollo mic: the stream's format: %u channel%s of 16 bits at 48 kHz: done\n", channels, channels == 1 ? "" : "s");
	if (mic_path == 1) {
		fwk_version();
		if (!topology_register()) {
			return false;
		}
	}
	if (!apollo_adsp_apr(APR_SVC_ADM, MIC_PORT, 0, ADM_CMD_DEVICE_OPEN_V5, &open, sizeof(open), APR_WAIT_MS, &answer) ||
	    answer.opcode != ADM_CMDRSP_DEVICE_OPEN_V5 || answer.words[0] != 0) {
		say("apollo mic: a device path from the port (topology %#x): REFUSED OR NO ANSWER (%#x %#x%s%s)\n", open.topology,
		    answer.opcode, answer.words[0], answer.opcode == ADM_CMDRSP_DEVICE_OPEN_V5 ? ": " : "",
		    answer.opcode == ADM_CMDRSP_DEVICE_OPEN_V5 ? dsp_error(answer.words[0]) : "");
		return false;
	}
	mic_copp_open = true;
	mic_copp = answer.words[1] & 0xffff;
	route.copp[0] = (uint16_t)mic_copp;
	say("apollo mic: the device path from port %#x is open: COPP %u (live recording, topology %#x, %u channel%s, 16 bits, 48 kHz)\n",
	    MIC_PORT, mic_copp, open.topology, channels, channels == 1 ? "" : "s");
	if (!command(mic_session == MIC_SESSION ? "the device path routed to session 2" : "the device path routed to session 3", APR_SVC_ADM, ADM_CMD_MATRIX_MAP_ROUTINGS_V5, &route, sizeof(route), 0,
	    NULL)) {
		return false;
	}
	return mic_path != 1 || pp_params();
}

/*
 * the codec DMA port: TX_CDC_DMA_TX_3 at 48 kHz, 16 bits; the active channels' mask is the TX macro's for the
 * AIF (tx_macro_get_channel_map(): active_ch_mask, the decimators' bits): path 0 decimator 0 (1), path 1
 * decimators 0 and 1 (3: channel 0 the top microphone, channel 1 the main one)
 */
static bool
dsp_port_on(void)
{
	struct {
		uint32_t version, rate;
		uint16_t bits, format, channels, mask;
	} __attribute__((packed)) config = { 1, MIC_RATE, 16, 0, 1, 1u << 0 };
	uint16_t start[2] = { MIC_PORT, 0 };

	if (mic_path == 1) {
		config.channels = 2;
		config.mask = 0x3;
	}
	if (restart_port) {
		/* (path 1 had it, with two channels: stopped again whatever undo() was told, then set up anew for one) */
		(void)command("the port 0xb037, stopped once more before path 0 sets it up", APR_SVC_AFE, AFE_PORT_CMD_DEVICE_STOP, start,
		    sizeof(start), 0, NULL);
		restart_port = false;
	}
	if (!afe_port_param(AFE_MODULE_AUDIO_DEV_INTERFACE, AFE_PARAM_ID_CODEC_DMA_CONFIG, &config, sizeof(config),
	    mic_path == 1 ? "the port TX_CDC_DMA_TX_3 (0xb037): 2 channels of 16 bits at 48 kHz, decimators 0 and 1" :
	    "the port TX_CDC_DMA_TX_3 (0xb037): 1 channel of 16 bits at 48 kHz, decimator 0")) {
		return false;
	}
	mic_port_on = command("the port 0xb037, started", APR_SVC_AFE, AFE_PORT_CMD_DEVICE_START, start, sizeof(start), 0, NULL);
	return mic_port_on;
}

static void
dsp_off(void)
{
	uint16_t stop[2] = { MIC_PORT, 0 };

	/* (each answer said: after a command the DSP never answered, these show whether it still listens) */
	if (mic_stream_open) {
		(void)port_command(mic_session == MIC_SESSION ? "the stream (session 2), closed" : "the stream (session 3), closed",
		    APR_SVC_ASM, MIC_STREAM_PORT, ASM_STREAM_CMD_CLOSE, NULL, 0);
		mic_stream_open = false;
	}
	if (mic_copp_open) {
		char what[48];

		snprintf(what, sizeof(what), "the device path (COPP %u), closed", mic_copp);
		(void)port_command(what, APR_SVC_ADM, (uint16_t)mic_copp, ADM_CMD_DEVICE_CLOSE_V5, NULL, 0);
		mic_copp_open = false;
	}
	if (cal_mapped) {
		(void)command("the calibration's page, unmapped for the ADM", APR_SVC_ADM, ADM_CMD_SHARED_MEM_UNMAP_REGIONS, &cal_handle,
		    sizeof(cal_handle), 0, NULL);
		cal_mapped = false;
	}
	if (mic_port_on) {
		(void)command("the port 0xb037, stopped", APR_SVC_AFE, AFE_PORT_CMD_DEVICE_STOP, stop, sizeof(stop), 0, NULL);
		mic_port_on = false;
	}
	if (mic_mapped) {
		(void)command("our periods, unmapped", APR_SVC_ASM, ASM_CMD_SHARED_MEM_UNMAP_REGIONS, &mic_map_handle,
		    sizeof(mic_map_handle), 0, NULL);
		mic_mapped = false;
	}
	apollo_adsp_apr_events_second(NULL);
	if (memory_taken) {
		apollo_audio_dsp_memory_put();
		memory_taken = false;
	}
}

/* ---- apollo_mic=4: the microphone for iOS -------------------------------------------------------------- */

/*
 * What the microphone hears goes into a ring, and /dev/hakios-mic gives it to iOS's audio server
 * (HakiosAudio.driver's device "Codec", userland/src/audio_plugin.c: codec_open() opens it read-only and
 * non-blocking, codec_read() reads what is there): 16-bit samples, one channel, 48 kHz -- the POCO's device,
 * the same contract. The path is up only while the device is open (the audio server opens it when its input
 * starts and closes it when it stops), and a read never waits: it gives what has come, keeping the reader
 * about 40 ms behind the DSP (it comes in 20 ms periods), and nothing until that much is there; the audio
 * server makes up the rest with silence.
 */
#define RING_FRAMES             16384           /* 341 ms; a power of two */
#define RING_KEEP               1920            /* 40 ms: how far behind the DSP a reader is kept */
#define RING_MOST               4800            /* 100 ms: more than this waiting, and the oldest is skipped */

static int16_t *ring;
static volatile uint64_t ring_written;          /* frames put into the ring since the device was made */
static uint64_t ring_read;                      /* frames taken from it (one reader: the audio server) */
static bool ring_primed;                        /* the reader is RING_KEEP behind; false after it ran dry */
static uint64_t ring_skipped, ring_dry;         /* frames skipped, reads that found too little: said when iOS stops */
static volatile uint32_t ios_wants;             /* the device is open */
static volatile bool stopping;                  /* the run ends (apollo_mic_dsp_down()) */

static void
ring_put(const int16_t *samples, uint32_t count)
{
	uint64_t at = ring_written;

	for (uint32_t i = 0; i < count; i++) {
		ring[(at + i) % RING_FRAMES] = samples[i];
	}
	__atomic_store_n(&ring_written, at + count, __ATOMIC_RELEASE);
}

static int
mic_dev_read(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	uint64_t written = __atomic_load_n(&ring_written, __ATOMIC_ACQUIRE), avail, want;
	int error = 0;

	if (ring == NULL || uio_resid(uio) < 2) {
		return 0;
	}
	want = (uint64_t)uio_resid(uio) / 2;
	avail = written - ring_read;
	if (avail > RING_MOST) {
		ring_skipped += avail - RING_KEEP;
		ring_read = written - RING_KEEP;
		avail = RING_KEEP;
	}
	if (!ring_primed) {
		if (avail < RING_KEEP) {
			return 0;
		}
		ring_primed = true;
	}
	if (avail < want) {
		ring_dry++;
		ring_primed = false;
	}
	while (want > 0 && avail > 0 && error == 0) {
		uint64_t at = ring_read % RING_FRAMES, n = want < avail ? want : avail;

		n = n < RING_FRAMES - at ? n : RING_FRAMES - at;
		error = uiomove((caddr_t)&ring[at], (int)(n * 2), uio);
		ring_read += n;
		want -= n;
		avail -= n;
	}
	return error;
}

static int
mic_dev_open(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	ios_wants = 1;
	return 0;
}

/* (on the last close) */
static int
mic_dev_close(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	ios_wants = 0;
	return 0;
}

static const struct cdevsw mic_cdevsw = {
	.d_open = mic_dev_open, .d_close = mic_dev_close, .d_read = mic_dev_read, .d_write = eno_rdwrt,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

/* ---- listening ---------------------------------------------------------------------------------------------- */

static uint32_t mic_secs = 10;
static int16_t mono[MIC_PERIOD_FRAMES];         /* a period made one channel (path 1), for iOS */

/* what a second's samples come to: their level in tenths of a dBFS, by a table of halvings (good enough to tell loud from quiet) */
struct loudness {
	int64_t sum;
	uint64_t squares;
	uint32_t peak;
};

static void
level_add(struct loudness *l, int32_t v)
{
	uint32_t a = (uint32_t)(v < 0 ? -v : v);

	l->sum += v;
	l->squares += (uint64_t)((int64_t)v * v);
	l->peak = a > l->peak ? a : l->peak;
}

static int
level_db10(const struct loudness *l, uint32_t frames, uint32_t *rms_out)
{
	uint64_t mean_square = l->squares / frames;
	uint32_t rms = 0, r;
	int tenths = 0;

	while ((uint64_t)(rms + 1) * (rms + 1) <= mean_square) {
		rms++;
	}
	*rms_out = rms;
	if (rms == 0) {
		return -999;
	}
	r = rms;
	while (r < 16384) {
		r *= 2;
		tenths -= 60;   /* 6.02 dB a halving, near enough */
	}
	return tenths - (int)((32768 - r) * 60 / 16384);
}

/*
 * The stream run, read for apollo_mic_secs (for_ios: while iOS has the device open): how loud it is, every
 * second (for iOS: the first three, then every 30th) -- what iOS gets, and on path 1 each microphone's
 * channel as well; for iOS into the ring (path 1: the two channels made one, apollo_mic_mix).
 */
static void
listen(bool for_ios)
{
	uint32_t run[3] = { 0, 0, 0 }, frames = 0, seconds = 0, periods = 0, empty = 0;
	struct loudness levels[3];                 /* what iOS gets; path 1: channel 0 (the top microphone), channel 1 (the main one) */
	unsigned next = 0;
	bool unmuted = false, settled = false;
	uint64_t began = mach_absolute_time(), now, elapsed_ms;
	struct hakios_apr_answer answer;

	bool first_said = false;
	uint64_t last_line_ms = 0;

	bzero(levels, sizeof(levels));
	read_ready = 0;
	reads_done = read_errors = 0;
	first_read_ms = first_read_status = first_read_bytes = first_error_status = 0;
	if (!apollo_adsp_apr(APR_SVC_ASM, MIC_STREAM_PORT, 0, ASM_SESSION_CMD_RUN_V2, run, sizeof(run), APR_WAIT_MS, &answer) ||
	    answer.opcode != APR_BASIC_RSP_RESULT || answer.words[1] != 0) {
		say("apollo mic: the session, run: REFUSED OR NO ANSWER (%#x %#x)\n", answer.opcode, answer.words[1]);
		return;
	}
	say("apollo mic: the session %u, run: done\n", mic_session);
	listen_began = mach_absolute_time();
	for (unsigned p = 0; p < MIC_PERIODS; p++) {
		if (!mic_read(p)) {
			say("apollo mic: a read could not be sent\n");
			return;
		}
	}
	if (for_ios) {
		say("apollo mic: LISTENING for iOS (path %u%s): %u reads of 20 ms given to the DSP\n", mic_path,
		    mic_path != 1 ? "" : mic_mix == 1 ? ", the main microphone" : mic_mix == 2 ? ", the top microphone" :
		    ", the two microphones averaged", MIC_PERIODS);
	} else {
		say("apollo mic: LISTENING (path %u): %u reads of 20 ms given to the DSP; for %u s\n", mic_path, MIC_PERIODS, mic_secs);
	}
	for (;;) {
		absolutetime_to_nanoseconds(mach_absolute_time() - began, &now);
		elapsed_ms = now / NSEC_PER_MSEC;
		if (!unmuted && elapsed_ms >= 100) {            /* TX_MACRO_AMIC_UNMUTE_DELAY_MS */
			unmuted = dec_unmute(0) && dec_unmute(1);
		}
		if (!settled && elapsed_ms >= 300) {            /* TX_MACRO_AMIC_HPF_DELAY_MS */
			settled = dec_settled(0) && dec_settled(1);
		}
		if (!first_said && first_read_ms != 0) {
			say("apollo mic: the first read came back %u ms after the reads were given: %s, status %#x (%s), %u bytes\n",
			    first_read_ms - 1, first_read_status == 0 ? "data" : "AN ERROR", first_read_status,
			    first_read_status < 21 ? dsp_error(first_read_status) : "?", first_read_bytes);
			first_said = true;
		} else if (!first_said && elapsed_ms >= 1000 && first_read_ms == 0 && reads_done == 0) {
			say("apollo mic: NO READ CAME BACK in 1 s (session %u, %u reads given)\n", mic_session, MIC_PERIODS);
			first_said = true;
		}
		if (elapsed_ms >= last_line_ms + 1000) {
			if (frames == 0 && elapsed_ms >= 1000 && (!for_ios || seconds <= 3)) {
				/* (a second without a sample: what came back instead) */
				say("apollo mic: a second without samples: %u periods back, %u empty; reads %u, errors %u (the first: %#x)\n",
				    periods, empty, reads_done, read_errors, first_error_status);
				periods = empty = 0;
			}
			last_line_ms = elapsed_ms;
		}
		if ((for_ios ? (!ios_wants || stopping) : elapsed_ms >= mic_secs * 1000ull) || broken) {
			break;
		}
		if (!(read_ready & (1u << next))) {
			if (elapsed_ms > 1500 && reads_done == 0) {
				say("apollo mic: NO READ HAS COME BACK in 1.5 s\n");
				break;
			}
			sleep_ms(2);
			continue;
		}
		OSBitAndAtomic(~(1u << next), &read_ready);
		{
			int16_t *samples = (int16_t *)(mic_memory + 4096 * next);
			uint32_t got = read_sizes[next] / (2 * channels);

			for (vm_offset_t line = (vm_offset_t)samples; line < (vm_offset_t)samples + MIC_PERIOD_BYTES; line += 64) {
				__asm__ volatile ("dc ivac, %0" : : "r" (line));
			}
			__asm__ volatile ("dsb sy");
			got = got > MIC_PERIOD_FRAMES ? MIC_PERIOD_FRAMES : got;
			empty += got == 0;
			for (uint32_t i = 0; i < got; i++) {
				int32_t v;

				if (channels == 1) {
					v = samples[i];
				} else {
					int32_t top = samples[2 * i], primary = samples[2 * i + 1];

					level_add(&levels[1], top);
					level_add(&levels[2], primary);
					v = mic_mix == 1 ? primary : mic_mix == 2 ? top : (top + primary) / 2;
				}
				mono[i] = (int16_t)v;
				level_add(&levels[0], v);
			}
			frames += got;
			if (for_ios) {
				ring_put(mono, got);
			}
		}
		periods++;
		(void)mic_read(next);
		next = (next + 1) % MIC_PERIODS;
		if (frames >= MIC_RATE) {
			uint32_t rms[3] = { 0, 0, 0 };
			int db10[3];

			for (unsigned c = 0; c < 3; c++) {
				db10[c] = level_db10(&levels[c], frames, &rms[c]);
			}
			seconds++;
			if (!for_ios || seconds <= 3 || seconds % 30 == 0) {
				say("apollo mic: second %u: %u frames (%u periods, %u empty), level %d.%d dBFS (rms %u), peak %u, mean %lld; "
				    "reads %u, errors %u%s%s\n", seconds, frames, periods, empty, db10[0] / 10,
				    db10[0] < 0 ? -db10[0] % 10 : db10[0] % 10, rms[0], levels[0].peak, levels[0].sum / (int64_t)frames,
				    reads_done, read_errors, unmuted ? "" : " (muted)", settled ? "" : " (settling)");
				if (channels == 2) {
					say("apollo mic:   the top microphone (channel 0) %d.%d dBFS (rms %u, peak %u, mean %lld); the main one "
					    "(channel 1) %d.%d dBFS (rms %u, peak %u, mean %lld)\n", db10[1] / 10,
					    db10[1] < 0 ? -db10[1] % 10 : db10[1] % 10, rms[1], levels[1].peak, levels[1].sum / (int64_t)frames,
					    db10[2] / 10, db10[2] < 0 ? -db10[2] % 10 : db10[2] % 10, rms[2], levels[2].peak,
					    levels[2].sum / (int64_t)frames);
				}
			}
			frames = periods = empty = 0;
			bzero(levels, sizeof(levels));
		}
	}
}

static void
undo(void)
{
	dsp_off();
	dec_off(1);
	dec_off(0);
	master_off();
	tx_clocks_off();
	pins_restore();
	reset_restore();
	clocks_off();
}

/* Everything up to the stream's run, in a recording's order; false (and `broken') if any of it fails. */
static bool
path_up_once(void)
{
	if (mic_path == 1 && level >= 3 && !mic_fw_load()) {
		say("apollo mic: without Xiaomi's data path 1 cannot be set up: path 0 (voice-rec-mic) for the rest of the run\n");
		mic_path = 0;
	}
	channels = mic_path == 1 ? 2 : 1;
	if (!clocks_on()) {
		say("apollo mic: the clocks were not all given: the blocks are not touched\n");
		return false;
	}
	if (!reset_release() || !pins_on() || !tx_clocks_on()) {
		return false;
	}
	if (!quiet) {
		master_look();          /* (its clock running now: what the master was before it is started) */
	}
	if (!master_on()) {
		return false;
	}
	sleep_ms(5);    /* ("5 ms for SoundWire's auto-enumeration", wcd938x_bind()) */
	if ((codec_device = codec_wait()) == 0) {
		return false;
	}
	to_all(SWRS_SCP_INT_STATUS_MASK_1, 0x04);       /* (swrm_runtime_resume(): to all, an ordinary command id) */
	if (!quiet) {
		codec_look();
	}
	if (level < 3) {
		return true;
	}
	/* the stream and path, the port's format and start, the codec's widgets */
	/*
	 * Path 1 in Android's order: the port (the DPCM back end, prepared first: afe_port_start()) before the device
	 * path and its calibration (the front end's prepare: adm_open(), adm_matrix_map() and send_adm_cal()); path 0
	 * as it was brought up
	 */
	return bolero_look() && codec_init() && (mic_path == 1 ? dsp_port_on() && dsp_path_on() : dsp_path_on() && dsp_port_on()) &&
	       decimators_on() &&
	       (!adc_on || (adc1_on() && (mic_path != 1 || adc4_on()))) && datapath_on();
}

/*
 * Path 1 that fails -- the DSP refusing Xiaomi's topology, its calibration or the path, or anything else -- is
 * undone and path 0 set up in its place, and kept for the rest of the run: a microphone, rather than none.
 */
static bool
path_up(void)
{
	if (path_up_once()) {
		return true;
	}
	if (mic_path != 1 || level < 3 || stopping) {
		return false;
	}
	say("apollo mic: PATH 1 (Android's handset-mic, Xiaomi's processing) FAILED (last: %s): undone, and path 0 (voice-rec-mic) "
	    "set up in its place for the rest of the run\n", last_write);
	/*
	 * All of it undone as at a recording's end (the stream, the COPP, the port, both decimators, the master reset,
	 * the clocks, the pins, the codec back in reset: its ADCs and mic biases with it), a second for the DSP to be
	 * done with whatever it did not answer, then path 0 from the start, as a fresh recording sets it up -- on
	 * session 3, which nothing of path 1 was routed to, in case the DSP kept path 1's routing
	 */
	undo();
	broken = false;
	mic_path = 0;
	mic_session = MIC_SESSION_FALLBACK;
	restart_port = true;
	sleep_ms(1000);
	return path_up_once();
}

/*
 * apollo_mic=4: the path up whenever iOS opens /dev/hakios-mic, down when it closes it; after three
 * recordings that broke off, no more until the run ends.
 */
static void
for_ios(void)
{
	int major = cdevsw_add(-1, &mic_cdevsw);
	unsigned recordings = 0, failures = 0;

	ring = kalloc_data(RING_FRAMES * sizeof(int16_t), Z_WAITOK | Z_ZERO);
	if (ring == NULL || major < 0 ||
	    devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "hakios-mic") == NULL) {
		say("apollo mic: /dev/hakios-mic cannot be made\n");
		return;
	}
	say("apollo mic: THE MICROPHONE IS IOS'S (/dev/hakios-mic) from here on, until the run ends: it listens only while iOS "
	    "records\n");
	while (!stopping) {
		uint64_t began, ns;
		bool failed;

		if (!ios_wants) {
			sleep_ms(20);
			continue;
		}
		lck_mtx_lock(&mic_lock);
		if (stopping) {
			lck_mtx_unlock(&mic_lock);
			break;
		}
		recordings++;
		quiet = recordings > 1;
		ring_primed = false;
		ring_skipped = ring_dry = 0;
		ring_read = __atomic_load_n(&ring_written, __ATOMIC_ACQUIRE);
		began = mach_absolute_time();
		say("apollo mic: iOS records (%u): the path up%s\n", recordings,
		    quiet ? " (the lines that say all went well are not said again)" : "");
		if (path_up()) {
			listen(true);
		}
		failed = broken;
		absolutetime_to_nanoseconds(mach_absolute_time() - began, &ns);
		say("apollo mic: iOS's recording %u %s after %llu.%llu s; last: %s; %llu frames skipped as too late, %llu reads found "
		    "too little\n", recordings, failed ? "BROKE OFF" : "ended", ns / NSEC_PER_SEC, ns % NSEC_PER_SEC / (NSEC_PER_SEC / 10),
		    last_write, ring_skipped, ring_dry);
		undo();
		lck_mtx_unlock(&mic_lock);
		if (failed) {
			broken = false;
			if (++failures >= 3) {
				say("apollo mic: three recordings broke off: the microphone stays off until the run ends\n");
				break;
			}
			/* (not again until iOS has closed the device and opened it again) */
			while (ios_wants && !stopping) {
				sleep_ms(20);
			}
		}
	}
}

static void
mic_thread(__unused void *parameter, __unused wait_result_t result)
{
	say("apollo mic: level %u, path %u: %s\n", level, mic_path,
	    level == 1 ? "a look, after the LPASS's votes and clocks; nothing written" :
	    level == 2 ? "the codec brought up to answer on SoundWire, nothing of it set; then all undone" :
	    level == 3 ? "the microphones, as Android records from them, listened to; then all undone" :
	    "the microphones for iOS (/dev/hakios-mic), up while iOS records");
	if (level == 4) {
		for_ios();
	} else {
		lck_mtx_lock(&mic_lock);
		if (level == 1) {
			if (clocks_on()) {
				sleep_ms(10);
				look();
			} else {
				say("apollo mic: the clocks were not all given: the blocks are not touched\n");
			}
		} else {
			if (path_up() && level >= 3) {
				listen(false);
			}
			say("apollo mic: %s; last: %s\n", broken ? "STOPPED" : level >= 3 ? "listened" : "done", last_write);
		}
		undo();
		lck_mtx_unlock(&mic_lock);
	}
	running = false;
	say("apollo mic: done\n");
	thread_terminate(current_thread());
}

/* The DSP's audio services are up (hakios.apollo_adsp.c, right after apollo_sound_dsp_up()). */
void
apollo_mic_dsp_up(void)
{
	thread_t thread;

	if (!PE_parse_boot_argn("apollo_mic", &level, sizeof(level)) || level == 0) {
		return;
	}
	level = level > 4 ? 4 : level;
	PE_parse_boot_argn("apollo_mic_secs", &mic_secs, sizeof(mic_secs));
	PE_parse_boot_argn("apollo_mic_path", &mic_path, sizeof(mic_path));
	PE_parse_boot_argn("apollo_mic_mix", &mic_mix, sizeof(mic_mix));
	PE_parse_boot_argn("apollo_mic_pan", &pan_setting, sizeof(pan_setting));
	PE_parse_boot_argn("apollo_mic_ns", &ns_setting, sizeof(ns_setting));
	PE_parse_boot_argn("apollo_mic_drc", &drc_setting, sizeof(drc_setting));
	PE_parse_boot_argn("apollo_mic_input", &smic_input, sizeof(smic_input));
	PE_parse_boot_argn("apollo_mic_adc", &adc_on, sizeof(adc_on));
	PE_parse_boot_argn("apollo_mic_test", &test_bits, sizeof(test_bits));
	mic_secs = mic_secs < 1 ? 1 : mic_secs > 600 ? 600 : mic_secs;
	mic_path = mic_path == 0 ? 0 : 1;
	mic_mix = mic_mix > 2 ? 0 : mic_mix;
	running = true;
	if (kernel_thread_start(mic_thread, NULL, &thread) != KERN_SUCCESS) {
		running = false;
		return;
	}
	thread_set_thread_name(thread, "apollo mic");
	thread_deallocate(thread);
}

/* The DSP is about to be shut down (apollo_adsp_stop()): what was asked for given back while it still listens. */
void
apollo_mic_dsp_down(void)
{
	stopping = true;
	if (running) {
		lck_mtx_lock(&mic_lock);
		undo();
		lck_mtx_unlock(&mic_lock);
	}
}

/*
 * Any way back to the loader (apollo_adsp_quiesce()): one processor, no lock, no sleep. The codec is put back
 * into reset if it was let out (one register: its mic biases go with it), and every line of the run is said again.
 */
void
apollo_mic_quiesce(void)
{
	if (reset_taken && tlmm_32.base != 0) {
		wr(tlmm_32.base + TLMM_IO, 0);
	}
	if (kept_count == 0) {
		return;
	}
	for (const char *c = "apollo mic: what happened in this run, again:\n"; *c != 0; c++) {
		loader_log_putc(*c);
	}
	for (unsigned i = kept_count > KEPT_LINES ? kept_count - KEPT_LINES : 0; i < kept_count; i++) {
		const char *line = kept[i % KEPT_LINES];

		loader_log_putc('>');
		loader_log_putc(' ');
		for (const char *c = line; *c != 0; c++) {
			loader_log_putc(*c);
		}
		if (line[0] == 0 || line[strlen(line) - 1] != '\n') {
			loader_log_putc('\n');
		}
	}
}
