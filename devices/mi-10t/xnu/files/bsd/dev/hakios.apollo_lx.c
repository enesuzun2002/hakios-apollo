/*
 * Darwin-on-apollo: the kernel's side of the Mi 10T's Wi-Fi driver object (devices/mi-10t/tools/mk-ath11k.sh: Linux's ath11k for
 * the QCA6390 on PCIe, the same sources as redwood's object under other names). It is bsd/dev/hakios.lx.c's twin: the
 * calls of sys/hakios.lx.h, with the names apollo_lx_os_* (wifi/lx/include/apollo_prefix.h renames them in the object),
 * for the chip that hakios.apollo_pcie.c brings up (sys/hakios.apollo_wifi.h): memory the chip sees through its SMMU context
 * (apollo_wifi_dma_map()), its interrupts as GIC INTIDs 800 + vector (apollo_wifi_msi()), its firmware files from fw/ of
 * the small image. The network (en0, the station) is the kernel's shared one. Nothing here is redwood's.
 * Boot-args: wifi_irq (default 1; 0: the driver looks at its rings by polling), wifi_ba and wifi_wide (default 1, as
 * wifi=1 stands for on redwood), wifi_trace, wifi_station, wifi_join, wifi_scan, wifi_hold, wifi_mac=AA:BB:CC:DD:EE:FF.
 * A kernel built without the object has the weak apollo_lx_start() and the others at the end of this file.
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <os/atomic_private.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/random.h>
#include <sys/hakios.apollo_wifi.h>
#include <sys/hakios.lx.h>
#include <sys/hakios.net.h>
#include <sys/systm.h>
#include <vm/memory_types.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, unsigned int flags);
extern void ml_io_unmap(vm_offset_t addr, vm_size_t sz);
extern uint64_t kvtophys(vm_offset_t va);       /* pmap_paddr_t */
extern kern_return_t thread_terminate(thread_t thread);
extern void thread_set_thread_name(thread_t thread, const char *name);

/* into the loader's log as well (as the other apollo drivers): a phone that resets keeps only that */
extern void loader_log_putc(char c);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

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


static LCK_GRP_DECLARE(lx_lock_group, "apollo-lx");
static LCK_MTX_DECLARE(lx_lock, &lx_lock_group);

/* ---- memory ------------------------------------------------------------------------------------------ */

void *
apollo_lx_os_alloc(unsigned long size)
{
	return kalloc_data(size, Z_WAITOK | Z_ZERO);
}

void
apollo_lx_os_free(void *memory, unsigned long size)
{
	kfree_data(memory, size);
}

/*
 * In one physical piece, so that it can be mapped a second time without this processor's cache in between (as
 * hakios.lx.c does); the first, cached view is written out and never used again. The chip's address comes from
 * apollo_wifi_dma_map(), given the cached view (kernel memory that kvtophys() knows): the chip's SMMU context gets
 * those pages.
 */
/*
 * The ath11k object's DMA pool (lx_base.c's POOL_SIZE: 64 MB in the apollo build, devices/mi-10t/tools/mk-ath11k.sh) in one physical piece: on the phone (25 October) that was not
 * to be had by the time the chip was up, so hakios.apollo_pcie.c asks for it while the kernel starts (apollo_wifi >= 5).
 */
static vm_offset_t reserved;
static unsigned long reserved_size;

void
apollo_lx_reserve(unsigned long size)
{
	size = round_page(size);
	if (reserved != 0 || kmem_alloc_contig(kernel_map, &reserved, size, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		say("apollo lx: no %lu bytes in one piece at boot either\n", size);
		reserved = 0;
		return;
	}
	reserved_size = size;
	say("apollo lx: %lu MB for the chip's DMA pool, taken at boot\n", size >> 20);
}

void *
apollo_lx_os_radio_memory(unsigned long size, unsigned long long *address)
{
	vm_offset_t cached = 0, uncached;
	uint64_t physical;

	bool from_reserve = false;

	size = round_page(size);
	if (reserved != 0 && size <= reserved_size) {
		cached = reserved;      /* taken at boot (apollo_lx_reserve()): later, 64 MB in one piece was not to be had */
		reserved = 0;           /* (given back below when this fails: the next try may have it) */
		from_reserve = true;
	} else if (kmem_alloc_contig(kernel_map, &cached, size, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) !=
	    KERN_SUCCESS) {
		say("apollo lx: no %lu bytes of memory in one piece\n", size);
		return NULL;
	}
	physical = kvtophys(cached);
	bzero((void *)cached, size);
	for (vm_offset_t line = cached; line < cached + size; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy");
	uncached = ml_io_map_unmappable(physical, size, VM_WIMG_WCOMB);
	if (uncached == 0) {
		say("apollo lx: cannot map %lu bytes of the chip's memory uncached\n", size);
		goto failed;
	}
	if (apollo_wifi_dma_map((void *)cached, size, address) != 0) {
		say("apollo lx: the chip's SMMU takes no %lu bytes\n", size);
		ml_io_unmap(uncached, size);    /* (the chip was never given an address in it: nothing of it is in use) */
		goto failed;
	}
	return (void *)uncached;
failed:
	if (from_reserve) {
		reserved = cached;
	} else {
		kmem_free(kernel_map, cached, size);
	}
	return NULL;
}

void *
apollo_lx_os_map_registers(unsigned long long physical, unsigned long size)
{
	unsigned long long page = trunc_page(physical);
	vm_offset_t mapped = ml_io_map_unmappable(page, round_page(physical + size) - page, VM_WIMG_IO);

	return mapped == 0 ? NULL : (void *)(mapped + (physical - page));
}

/* ---- saying things ----------------------------------------------------------------------------------- */

void
apollo_lx_os_print(const char *text)
{
	say("%s", text);
}

void
apollo_lx_os_tell(const char *text)
{
	say("%s", text);
}

void
apollo_lx_os_panic(const char *text)
{
	panic("%s", text);
}

/* ---- time, threads, sleeping --------------------------------------------------------------------------- */

unsigned long long
apollo_lx_os_nanoseconds(void)
{
	uint64_t nanoseconds;

	absolutetime_to_nanoseconds(mach_absolute_time(), &nanoseconds);
	return nanoseconds;
}

void *
apollo_lx_os_thread_self(void)
{
	return current_thread();
}

struct lx_thread {
	void (*function)(void *);
	void *argument;
};

static void
lx_thread_runs(void *parameter, __unused wait_result_t result)
{
	struct lx_thread start = *(struct lx_thread *)parameter;

	kfree_type(struct lx_thread, parameter);
	start.function(start.argument);
	thread_terminate(current_thread());
}

int
apollo_lx_os_thread(const char *name, void (*function)(void *), void *argument)
{
	struct lx_thread *start = kalloc_type(struct lx_thread, Z_WAITOK | Z_ZERO);
	thread_t thread;

	if (start == NULL) {
		return ENOMEM;
	}
	start->function = function;
	start->argument = argument;
	if (kernel_thread_start(lx_thread_runs, start, &thread) != KERN_SUCCESS) {
		kfree_type(struct lx_thread, start);
		return EAGAIN;
	}
	thread_set_thread_name(thread, name);
	thread_deallocate(thread);
	return 0;
}

void
apollo_lx_os_lock(void)
{
	lck_mtx_lock(&lx_lock);
}

void
apollo_lx_os_unlock(void)
{
	lck_mtx_unlock(&lx_lock);
}

int
apollo_lx_os_sleep(const void *address, unsigned long long nanoseconds)
{
	uint64_t deadline;

	if (nanoseconds == 0) {
		return lck_mtx_sleep(&lx_lock, LCK_SLEEP_DEFAULT, (event_t)address, THREAD_UNINT) == THREAD_TIMED_OUT;
	}
	nanoseconds_to_absolutetime(nanoseconds, &deadline);
	deadline += mach_absolute_time();
	return lck_mtx_sleep_deadline(&lx_lock, LCK_SLEEP_DEFAULT, (event_t)address, THREAD_UNINT, deadline) == THREAD_TIMED_OUT;
}

void
apollo_lx_os_wakeup(const void *address)
{
	thread_wakeup((event_t)address);
}

/* ---- the rest of the machine ------------------------------------------------------------------------- */

void
apollo_lx_os_random(void *buffer, unsigned long size)
{
	read_random(buffer, (u_int)size);
}

/*
 * A number given to the kernel when it was started. Unlike on redwood nothing stands for them but what is given, with
 * these exceptions: the radio's interrupts, and the two things lx_80211.c does with a station that its author of the
 * radio's bring-up found to work (block acknowledgements, 40 MHz) are on unless given as 0, as are the two things Android's
 * cnss2 does that ath11k does not (the regulatory database, wifi_regdb; the host's DDR range, wifi_ddr_range).
 */
unsigned
apollo_lx_os_option(const char *name)
{
	static const char *const on_by_default[] = { "wifi_irq", "wifi_ba", "wifi_wide", "wifi_regdb", "wifi_ddr_range" };
	uint32_t value = 0;

	if (PE_parse_boot_argn(name, &value, sizeof(value))) {
		return value;
	}
	for (unsigned i = 0; i < sizeof(on_by_default) / sizeof(on_by_default[0]); i++) {
		if (strcmp(name, on_by_default[i]) == 0) {
			return 1;
		}
	}
	return 0;
}

/*
 * The names ath11k asks for are Linux's firmware tree's ("ath11k/QCA6390/hw2.0/board-2.bin", m3.bin; also
 * firmware-2.bin, board.bin, caldata.bin, regdb.bin, amss.bin, of which the small image may have none): each is the file
 * of that last name in fw/ of the small image (fw-wifi-apollo/ has amss.bin, m3.bin, board-2.bin and bd_j3s.elf, see
 * userland/build.sh). A file is taken from the image once and kept: the driver keeps what it was given.
 */
#define FILES 8
static struct {
	char name[24];
	const uint8_t *data;
	uint64_t size;
} files[FILES];
static LCK_MTX_DECLARE(files_lock, &lx_lock_group);

int
apollo_lx_os_firmware(const char *name, const void **data, unsigned long *size)
{
	const char *last = strrchr(name, '/');
	unsigned n;

	last = last == NULL ? name : last + 1;
	/*
	 * Boot-arg wifi_bdf_phone=1: the phone's own board data, Android's /vendor/firmware qca6390/bd_j3s.elf (the Mi 10T is
	 * "j3s"), in place of linux-firmware's board-2.bin, whose entry for board id 0xff is a generic one: board-2.bin is
	 * then not there, and ath11k falls back to board.bin (core.c: ath11k_core_fetch_bdf), which takes an ELF as such
	 * (qmi.c: ath11k_qmi_load_bdf_qmi, ATH11K_QMI_BDF_TYPE_ELF).
	 */
	{
		uint32_t phone = 0;

		if (PE_parse_boot_argn("wifi_bdf_phone", &phone, sizeof(phone)) && phone) {
			if (strcmp(last, "board-2.bin") == 0) {
				say("apollo lx: the driver asks for %s: left out (wifi_bdf_phone: the phone's own board data instead)\n", name);
				return ENOENT;
			}
			if (strcmp(last, "board.bin") == 0) {
				last = "bd_j3s.elf";
			}
		}
	}
	if (strlen(last) >= sizeof(files[0].name)) {
		return ENOENT;
	}
	lck_mtx_lock(&files_lock);
	for (n = 0; n < FILES && files[n].name[0] != 0; n++) {
		if (strcmp(files[n].name, last) == 0) {
			break;
		}
	}
	if (n < FILES && files[n].name[0] == 0) {
		uint64_t length = 0;
		const uint8_t *file = hakios_image_file("fw", last, &length, NULL);

		if (file == NULL) {
			lck_mtx_unlock(&files_lock);
			say("apollo lx: the driver asks for %s: no fw/%s in the small image\n", name, last);
			return ENOENT;
		}
		strlcpy(files[n].name, last, sizeof(files[n].name));
		files[n].data = file;
		files[n].size = length;
		say("apollo lx: the driver asks for %s: fw/%s, %llu bytes\n", name, last, length);
	}
	if (n == FILES) {
		lck_mtx_unlock(&files_lock);
		return ENOENT;
	}
	*data = files[n].data;
	*size = (unsigned long)files[n].size;
	lck_mtx_unlock(&files_lock);
	return 0;
}

int
apollo_lx_os_rproc_boot(void)
{
	return 0;       /* (nothing to boot: the chip runs its own firmware from apollo_wifi_power_up() on, which the bus calls) */
}

/*
 * The radio's own address, from the boot-arg wifi_mac (six bytes in hexadecimal with colons between), as
 * hakios_wifi_radio_address() takes it; the log says that one was given and whether it is a station's address, not
 * which. Without one the driver uses what the firmware says.
 */
int
apollo_lx_os_radio_address(unsigned char *address)
{
	char text[32] = "";
	uint8_t bytes[6];
	unsigned count = 0, digits = 0, value = 0;
	bool good = true;

	if (!PE_parse_boot_arg_str("wifi_mac", text, sizeof(text)) || text[0] == 0) {
		return 0;
	}
	for (const char *c = text;; c++) {
		int digit = *c >= '0' && *c <= '9' ? *c - '0' : *c >= 'a' && *c <= 'f' ? *c - 'a' + 10 : *c >= 'A' && *c <= 'F' ?
		    *c - 'A' + 10 : -1;

		if (digit >= 0 && digits < 2) {
			value = value << 4 | (unsigned)digit;
			digits++;
		} else if ((*c == ':' || *c == 0) && digits != 0 && count < 6) {
			bytes[count++] = (uint8_t)value;
			digits = value = 0;
		} else {
			good = false;
		}
		if (*c == 0 || !good) {
			break;
		}
	}
	good = good && count == 6 && !(bytes[0] & 1) && (bytes[0] | bytes[1] | bytes[2] | bytes[3] | bytes[4] | bytes[5]) != 0;
	say("apollo lx: the driver asks for the radio's address: one was given, %lu characters, %s\n", (unsigned long)strlen(text),
	    good ? "six bytes: it is the radio's for this run" : "WHICH IS NOT A STATION'S ADDRESS: not used");
	if (good) {
		memcpy(address, bytes, sizeof(bytes));
	}
	return good;
}

/*
 * The network the radio is to join: the file "network" in fw/ of the small image, the name on its first line, the
 * passphrase on its second (as hakios_wifi_network() reads it; userland/build.sh takes it from a file that is in no
 * repository). The log says how long the two are, and nothing else of them.
 */
int
apollo_lx_os_network(unsigned char *name, unsigned *name_length, unsigned char *passphrase, unsigned *passphrase_length)
{
	uint64_t size = 0;
	const uint8_t *file = hakios_image_file("fw", "network", &size, NULL), *end = file + size, *line = file;
	unsigned lengths[2] = { 0, 0 }, sizes[2] = { *name_length, *passphrase_length };
	uint8_t *where[2] = { name, passphrase };

	if (file == NULL) {
		say("apollo lx: which network to join: none was given (fw/network of the small image)\n");
		return 0;
	}
	for (unsigned n = 0; n < 2 && line < end; n++) {
		const uint8_t *stop = line;

		while (stop < end && *stop != '\n') {
			stop++;
		}
		lengths[n] = (unsigned)(stop - line) - (stop > line && stop[-1] == '\r' ? 1 : 0);
		if (lengths[n] > sizes[n]) {
			lengths[n] = 0;
		} else {
			memcpy(where[n], line, lengths[n]);
		}
		line = stop + 1;
	}
	say("apollo lx: which network to join: a name of %u bytes and a passphrase of %u bytes were given%s\n", lengths[0],
	    lengths[1], lengths[0] == 0 || lengths[1] < 8 || lengths[1] > 63 ? ": NOT A NAME OF 1 TO 32 BYTES AND A PASSPHRASE "
	    "OF 8 TO 63" : "");
	if (lengths[0] == 0 || lengths[1] < 8 || lengths[1] > 63) {
		return 0;
	}
	*name_length = lengths[0];
	*passphrase_length = lengths[1];
	return 1;
}

/* ---- frames: the radio as the link behind en0 (sys/hakios.net.h) ------------------------------------ */

/*
 * A frame from the network stack comes on whatever thread sends it, and the driver's side keeps
 * a record of every thread that enters it: so frames are put in a queue here, and one thread of
 * ours takes them to the driver. A queue that is full drops what comes (TCP sends it again).
 */
#define LX_QUEUE        256

struct lx_frame {
	uint16_t length;
	uint8_t bytes[HAKIOS_NET_FRAME_MAX];
};

static bool lx_link_attached;
static LCK_MTX_DECLARE(lx_queue_lock, &lx_lock_group);
static struct lx_frame *lx_queue;
static uint32_t lx_queue_in, lx_queue_out;      /* frames put in and taken out, counted from the start */
static uint64_t lx_queue_dropped;

static void
lx_queue_runs(__unused void *parameter, __unused wait_result_t result)
{
	for (;;) {
		struct lx_frame *frame;

		lck_mtx_lock(&lx_queue_lock);
		while (lx_queue_in == lx_queue_out) {
			lck_mtx_sleep(&lx_queue_lock, LCK_SLEEP_DEFAULT, (event_t)&lx_queue_in, THREAD_UNINT);
		}
		frame = &lx_queue[lx_queue_out % LX_QUEUE];     /* (stays ours until lx_queue_out moves) */
		lck_mtx_unlock(&lx_queue_lock);
		apollo_lx_transmit(frame->bytes, frame->length);
		lck_mtx_lock(&lx_queue_lock);
		lx_queue_out++;
		lck_mtx_unlock(&lx_queue_lock);
	}
}

static bool
lx_link_transmit(mbuf_t packet, size_t length)
{
	struct lx_frame *frame;

	lck_mtx_lock(&lx_queue_lock);
	if (length > HAKIOS_NET_FRAME_MAX || lx_queue_in - lx_queue_out == LX_QUEUE) {
		lx_queue_dropped++;
		lck_mtx_unlock(&lx_queue_lock);
		return false;
	}
	frame = &lx_queue[lx_queue_in % LX_QUEUE];
	if (mbuf_copydata(packet, 0, length, frame->bytes) != 0) {
		lck_mtx_unlock(&lx_queue_lock);
		return false;
	}
	frame->length = (uint16_t)length;
	lx_queue_in++;
	lck_mtx_unlock(&lx_queue_lock);
	thread_wakeup((event_t)&lx_queue_in);
	return true;
}

static struct hakios_net_link lx_link = {
	.name = "the phone's radio (ath11k, QCA6390)",
	.wireless = true,
	.transmit = lx_link_transmit,
};

void
apollo_lx_os_net_attach(const unsigned char *address)
{
	thread_t thread;

	if (lx_link_attached) {
		return;
	}
	lx_queue = kalloc_data(LX_QUEUE * sizeof(*lx_queue), Z_WAITOK | Z_ZERO);
	if (lx_queue == NULL || kernel_thread_start(lx_queue_runs, NULL, &thread) != KERN_SUCCESS) {
		say("apollo lx: no memory or no thread for the frames to send: the radio's frames go nowhere\n");
		return;
	}
	thread_set_thread_name(thread, "wifi-send");
	thread_deallocate(thread);
	memcpy(lx_link.address, address, sizeof(lx_link.address));
	lx_link_attached = hakios_net_attach(&lx_link);
	say("apollo lx: %s\n", lx_link_attached ? "the radio is the link behind en0" : "EN0 HAS A LINK ALREADY: the radio's frames go "
	    "nowhere");
}

/*
 * en0 made while the kernel starts, before the radio is up (hakios.lx.c's hakios_lx_net_early(), for the same reason:
 * iOS's Wi-Fi side looks for the interface and its registry entry once, as soon as it runs, and a radio that turns up
 * later is one it has already failed to open). Only with an address (wifi_mac=) and wifi_station=1.
 */
void
apollo_lx_net_early(void)
{
	uint8_t address[6];

	if (apollo_lx_os_option("wifi_station") && apollo_lx_os_radio_address(address)) {
		apollo_lx_os_net_attach(address);
	}
}

void
apollo_lx_os_net_link(int up)
{
	if (lx_link_attached) {
		hakios_net_link_state(up != 0);
	}
}

void
apollo_lx_os_net_input(const unsigned char *frame, unsigned long length)
{
	if (lx_link_attached && length <= HAKIOS_NET_FRAME_MAX) {
		hakios_net_input(frame, length);
	}
}

/* ---- the radio's interrupts ------------------------------------------------------------------------------- */

extern kern_return_t gic_v3_spi_attach(uint32_t intid, bool edge, void (*handler)(void *context, uint32_t intid), void *context);
extern void gic_v3_spi_mask(uint32_t intid);
extern void gic_v3_spi_unmask(uint32_t intid);
extern void gic_v3_spi_detach(uint32_t intid);
extern uint64_t gic_v3_spi_count(uint32_t intid);

#define LX_VECTORS      32

static struct {
	void (*function)(void *);
	void *argument;
	bool attached;
} lx_vector[LX_VECTORS];
static uint32_t lx_irq_first;                   /* the GIC's number of vector 0; 0: no interrupts */
static uint32_t lx_irq_pending;                 /* the vectors that have fired and are not dealt with yet */
static uint32_t lx_irq_running;                 /* the vectors whose function the thread is in */
static thread_t lx_irq_thread;
static LCK_MTX_DECLARE(lx_irq_lock, &lx_lock_group);

/* In interrupt context: the vector masked, and the thread told. */
static void
lx_irq_arrived(__unused void *context, uint32_t intid)
{
	gic_v3_spi_mask(intid);
	os_atomic_or(&lx_irq_pending, 1u << (intid - lx_irq_first), relaxed);
	thread_wakeup((event_t)&lx_irq_pending);
}

static void
lx_irq_runs(__unused void *parameter, __unused wait_result_t result)
{
	for (;;) {
		uint32_t pending;

		for (;;) {
			assert_wait((event_t)&lx_irq_pending, THREAD_UNINT);
			if ((pending = os_atomic_xchg(&lx_irq_pending, 0, acquire)) != 0) {
				clear_wait(current_thread(), THREAD_AWAKENED);
				break;
			}
			thread_block(THREAD_CONTINUE_NULL);
		}
		for (unsigned v = 0; v < LX_VECTORS; v++) {
			if (!(pending & 1u << v)) {
				continue;
			}
			lck_mtx_lock(&lx_irq_lock);
			if (!lx_vector[v].attached) {
				lck_mtx_unlock(&lx_irq_lock);
				continue;
			}
			lx_irq_running |= 1u << v;
			lck_mtx_unlock(&lx_irq_lock);
			lx_vector[v].function(lx_vector[v].argument);
			lck_mtx_lock(&lx_irq_lock);
			lx_irq_running &= ~(1u << v);
			lck_mtx_unlock(&lx_irq_lock);
			thread_wakeup((event_t)&lx_irq_running);
		}
	}
}

int
apollo_lx_os_irq_msi(unsigned long long *address, unsigned *data)
{
	if (apollo_wifi_msi(address, data) != 0) {
		return 0;
	}
	lx_irq_first = apollo_wifi_intid(0);    /* (vector v is INTID apollo_wifi_intid(v): 800 + v) */
	return lx_irq_first != 0;
}

int
apollo_lx_os_irq_attach(unsigned vector, void (*function)(void *), void *argument)
{
	kern_return_t kr;

	if (vector >= LX_VECTORS || lx_irq_first == 0 || function == NULL) {
		return EINVAL;
	}
	if (lx_irq_thread == THREAD_NULL) {
		say("apollo lx: wifi_irq: THE GIC'S DISTRIBUTOR IS WRITTEN for the radio's interrupts: each vector's SPI (%u + v) edge-"
		    "triggered, group 1, routed to this processor, then enabled\n", lx_irq_first);
		if (kernel_thread_start(lx_irq_runs, NULL, &lx_irq_thread) != KERN_SUCCESS) {
			lx_irq_thread = THREAD_NULL;
			return ENOMEM;
		}
		thread_set_thread_name(lx_irq_thread, "wifi-irq");
	}
	lck_mtx_lock(&lx_irq_lock);
	lx_vector[vector].function = function;
	lx_vector[vector].argument = argument;
	lx_vector[vector].attached = true;
	lck_mtx_unlock(&lx_irq_lock);
	if ((kr = gic_v3_spi_attach(apollo_wifi_intid(vector), true, lx_irq_arrived, NULL)) != KERN_SUCCESS) {
		lck_mtx_lock(&lx_irq_lock);
		lx_vector[vector].attached = false;
		lck_mtx_unlock(&lx_irq_lock);
		return kr == KERN_NAME_EXISTS ? EBUSY : EINVAL;
	}
	gic_v3_spi_mask(apollo_wifi_intid(vector));
	return 0;
}

void
apollo_lx_os_irq_enable(unsigned vector)
{
	if (vector < LX_VECTORS && lx_vector[vector].attached) {
		gic_v3_spi_unmask(apollo_wifi_intid(vector));
	}
}

void
apollo_lx_os_irq_disable(unsigned vector)
{
	if (vector >= LX_VECTORS || !lx_vector[vector].attached) {
		return;
	}
	gic_v3_spi_mask(apollo_wifi_intid(vector));
	if (current_thread() == lx_irq_thread) {
		return;                         /* (from the vector's own function, or another's: nothing to wait for) */
	}
	lck_mtx_lock(&lx_irq_lock);
	while (lx_irq_running & 1u << vector) {
		lck_mtx_sleep(&lx_irq_lock, LCK_SLEEP_DEFAULT, (event_t)&lx_irq_running, THREAD_UNINT);
	}
	lck_mtx_unlock(&lx_irq_lock);
}

/*
 * Every vector marked detached under lx_irq_lock first, and the functions under way waited for: from then on neither the thread nor
 * apollo_lx_os_irq_enable() (it looks at attached) unmasks an SPI that is about to go. Then the GIC's side.
 */
void
apollo_lx_os_irq_detach_all(void)
{
	bool was[LX_VECTORS];

	lck_mtx_lock(&lx_irq_lock);
	for (unsigned v = 0; v < LX_VECTORS; v++) {
		was[v] = lx_vector[v].attached;
		lx_vector[v].attached = false;
	}
	while (lx_irq_running != 0 && current_thread() != lx_irq_thread) {
		lck_mtx_sleep(&lx_irq_lock, LCK_SLEEP_DEFAULT, (event_t)&lx_irq_running, THREAD_UNINT);
	}
	lck_mtx_unlock(&lx_irq_lock);
	for (unsigned v = 0; v < LX_VECTORS; v++) {
		if (was[v]) {
			gic_v3_spi_mask(apollo_wifi_intid(v));
			gic_v3_spi_detach(apollo_wifi_intid(v));
		}
	}
	os_atomic_store(&lx_irq_pending, 0, relaxed);
}

unsigned long long
apollo_lx_os_irq_count(unsigned vector)
{
	return vector < LX_VECTORS && lx_irq_first != 0 ? gic_v3_spi_count(apollo_wifi_intid(vector)) : 0;
}

/* ---- the station (sys/hakios.sta.h) --------------------------------------------------------------------- */

void
apollo_lx_os_station(const struct hakios_sta_ops *ops)
{
	hakios_sta_attach(ops);
}

void
apollo_lx_os_station_lost(int error)
{
	hakios_sta_lost(error);
}

/* ---- the driver ---------------------------------------------------------------------------------------- */

/*
 * The object has its own (strong: mk-ath11k.sh exports apollo_lx_prepare/start/stop/transmit); these are the ones of a kernel built
 * without it, and the only stand-ins (hakios.apollo_pcie.c has none). Weak, so that the object's win when it is linked in.
 */
__attribute__((weak)) int
apollo_lx_start(void)
{
	say("apollo lx: THE DRIVER IS NOT BUILT IN: its object was not there when this kernel was built (devices/mi-10t/tools/mk-ath11k.sh, "
	    "then build the kernel again)\n");
	return ENODEV;
}

__attribute__((weak)) void
apollo_lx_stop(void)
{
}

__attribute__((weak)) int
apollo_lx_prepare(void)
{
	return 0;
}

__attribute__((weak)) int
apollo_lx_transmit(__unused const unsigned char *frame, __unused unsigned long length)
{
	return ENODEV;
}
