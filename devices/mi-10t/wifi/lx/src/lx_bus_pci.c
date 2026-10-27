/*
 * The radio's bus on the Mi 10T (apollo): the QCA6390 on PCIe, for as long as ath11k's own (pci.c, pcic.c, mhi.c)
 * is not built. It is lx_bus.c's, as it is for redwood's WCN6750, with what is different about this chip: the
 * driver is the one of ath11k's PCI bus (ATH11K_BUS_PCI, QCA6390 hw2.0, the MSI assignment of pcic.c for it,
 * with the MHI's and the wake-up's vectors that the kernel's PCIe side (devices/mi-10t/xnu: hakios.apollo_pcie.c) has), its
 * registers are reached through the kernel's (apollo_wifi_read32/write32, with ath11k_pcic_read32()'s wake-up of
 * the chip around an access above ATH11K_PCI_ACCESS_ALWAYS_OFF once the device is initialised), and its power
 * is the kernel's: apollo_wifi_power_up() brings it to mission mode, with the QRTR channels
 * (devices/mi-10t/xnu: hakios.apollo_qrtr.c) up, before the driver's QMI exchange starts. The names of what is asked of the kernel
 * and of what is shown to it are apollo_*, by devices/mi-10t/wifi/lx/include/apollo_prefix.h.
 *
 * Boot-args: wifi_irq=0 (the rings are looked at every millisecond, as lx_bus.c does without interrupts; the
 * default here is the radio's interrupts), wifi_trace=1 (the driver's debug lines and the chip's accesses, the
 * first 600 of them, are said), wifi_station, wifi_join, wifi_scan, wifi_hold as in lx_bus.c.
 */
#define LX_QMI_AS_IT_IS				/* (this file is where the driver's requests go past) */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/skbuff.h>

#include "core.h"
#include "ce.h"
#include "debug.h"
#include "hal.h"
#include "hif.h"
#include "htc.h"
#include "pcic.h"
#include "pci.h"
#include "qmi.h"

#include <lx_os.h>
#include "lx.h"

/* bsd/sys/apollo_wifi.h (the kernel's, written for both sides: plain C types) */
uint32_t apollo_wifi_read32(uint32_t offset);
void apollo_wifi_write32(uint32_t offset, uint32_t value);
int apollo_wifi_power_up(void);
void apollo_wifi_power_down(void);
void apollo_wifi_wake(int on);

static struct {
	bool powered;				/* apollo_wifi_power_up() has come back: the chip's registers answer */
	bool closed;				/* the driver has stopped the bus: the registers are not touched again */
	unsigned reads, writes, said, refused;
	u32 last, last_value;			/* the register written last, and with what ... */
	unsigned again;				/* ... and how many writes to it in a row have not been said */
	volatile bool looking;			/* the thread that looks at the copy engines is wanted ... */
	volatile bool looks;			/* ... and runs */
	volatile bool data;			/* ... and looks at the data path's rings as well */
	unsigned same;				/* it has written a ring's pointer what it held so many times */

	/* the radio's interrupts (wifi_irq=1): what the copy engines and the data path's groups are taken care of on */
	bool irq;
	struct mutex service;			/* one thing at a time looks at the rings: an interrupt's or the thread */
	int ce_vector[CE_COUNT_MAX];		/* a copy engine's vector, -1: it has none (ce.c: CE_ATTR_DIS_INTR) */
	int group_vector[ATH11K_EXT_IRQ_GRP_NUM_MAX];
	unsigned long long late;		/* work the thread found that no interrupt had brought */
} bus;

/*
 * The radio's interrupt vectors, as pcic.c's ath11k_msi_config has them for the QCA6390 (hw2.0): 32 in all,
 * three for the MHI from vector 0 (the kernel's PCIe side takes care of those), ten for the copy engines from 3,
 * one for the wake-up (13) and eighteen for the data path's groups from 14.
 */
#define USER_CE		1
#define USER_DP		3
static struct ath11k_msi_user msi_users[] = {
	{ .name = "MHI", .num_vectors = 3, .base_vector = 0 },
	{ .name = "CE", .num_vectors = 10, .base_vector = 3 },
	{ .name = "WAKE", .num_vectors = 1, .base_vector = 13 },
	{ .name = "DP", .num_vectors = 18, .base_vector = 14 },
};
static void irq_say(void);

static const struct ath11k_msi_config msi_config = {
	.total_vectors = 32,
	.total_users = 4,
	.users = msi_users,
	.hw_rev = ATH11K_HW_QCA6390_HW20,
};

/* ---- what the driver says ---------------------------------------------------------------------- */

static void
driver_says(const char *kind, const char *format, va_list list)
{
	char line[320];
	size_t length;

	vsnprintf(line, sizeof(line), format, list);
	length = strlen(line);
	/* (Linux ends a line for whoever has not) */
	printf("ath11k%s: %s%s", kind, line, length != 0 && line[length - 1] == '\n' ? "" : "\n");
}

void
ath11k_info(struct ath11k_base *ab, const char *format, ...)
{
	va_list list;

	va_start(list, format);
	driver_says("", format, list);
	va_end(list);
}

/* (the driver's warnings and errors are said whatever happens to the rest) */
static void
driver_complains(const char *kind, const char *format, va_list list)
{
	char line[320];

	vsnprintf(line, sizeof(line), format, list);
	lx_tell("ath11k %s: %s%s", kind, line, line[0] != 0 && line[strlen(line) - 1] == '\n' ? "" : "\n");
}

void
ath11k_warn(struct ath11k_base *ab, const char *format, ...)
{
	va_list list;

	va_start(list, format);
	driver_complains("WARNS", format, list);
	va_end(list);
}

void
ath11k_err(struct ath11k_base *ab, const char *format, ...)
{
	va_list list;

	va_start(list, format);
	driver_complains("ERROR", format, list);
	va_end(list);
}

void
__ath11k_dbg(struct ath11k_base *ab, enum ath11k_debug_mask mask, const char *format, ...)
{
	va_list list;

	va_start(list, format);
	driver_says(" (debug)", format, list);
	va_end(list);
}

void
ath11k_dbg_dump(struct ath11k_base *ab, enum ath11k_debug_mask mask, const char *msg, const char *prefix, const void *buf,
    size_t len)
{
}

/* ---- the chip's registers ---------------------------------------------------------------------- */

/*
 * The kernel reaches them through BAR0 with the window above 0x80000 (ath11k_pcic_read32()'s way). They answer
 * once the chip is in mission mode (apollo_wifi_power_up()), and not after the driver has stopped the bus.
 */
static bool
refused(struct ath11k_base *ab, u32 offset)
{
	if (bus.powered && !bus.closed) {
		return (false);
	}
	if (bus.refused++ < 8) {
		lx_tell("lx: the chip's %#x is reached %s: NOT DONE\n", offset, bus.closed ? "after the bus was stopped" :
		    "before the chip is powered up");
	}
	return (true);
}

/* ath11k_pcic_read32()/write32(): the chip is woken for an access above ATH11K_PCI_ACCESS_ALWAYS_OFF once the device is initialised */
static bool
wake_needed(struct ath11k_base *ab, u32 offset)
{
	return (test_bit(ATH11K_FLAG_DEVICE_INIT_DONE, &ab->dev_flags) && offset >= ATH11K_PCI_ACCESS_ALWAYS_OFF);
}

/*
 * The accesses are said one by one, up to a number of lines; of writes to one register in a row
 * (a ring's pointer, moved once for every buffer) the first three, and then how many followed.
 */
#define SAID_FIRST	600

static void
say_repeats(void)
{
	if (bus.again != 0) {
		printf("lx: ... and %u more to the chip's %#x, the last %#x\n", bus.again, bus.last, bus.last_value);
		bus.again = 0;
	}
}

static bool
said_write(u32 offset, u32 value)
{
	static struct {
		u32 offset, value;
	} pointers[24];				/* while the copy engines are looked at: what each pointer was moved to last */
	static unsigned row;

	if (bus.looks) {
		for (unsigned i = 0; i < nitems(pointers); i++) {
			if (pointers[i].offset == offset || pointers[i].offset == 0) {
				bool moved = pointers[i].offset == 0 || pointers[i].value != value;

				pointers[i].offset = offset;
				pointers[i].value = value;
				bus.same += !moved;
				return (moved && bus.said < SAID_FIRST);
			}
		}
	}
	row = offset == bus.last ? row + 1 : 0;
	if (row >= 3) {
		bus.again++;
		bus.last_value = value;
		return (false);
	}
	say_repeats();
	bus.last = offset;
	return (bus.said < SAID_FIRST);
}

static u32
bus_read32(struct ath11k_base *ab, u32 offset)
{
	u32 value;
	bool wake;

	if (refused(ab, offset)) {
		return (0);
	}
	bus.reads++;
	if ((wake = wake_needed(ab, offset))) {
		apollo_wifi_wake(1);
	}
	value = apollo_wifi_read32(offset);
	if (wake) {
		apollo_wifi_wake(0);
	}
	if (bus.said < SAID_FIRST) {
		say_repeats();
		bus.last = ~0u;
		bus.said++;
		printf("lx: the chip's %#x reads %#x\n", offset, value);
	}
	return (value);
}

static void
bus_write32(struct ath11k_base *ab, u32 offset, u32 value)
{
	bool wake;

	if (refused(ab, offset)) {
		return;
	}
	bus.writes++;
	if (said_write(offset, value)) {
		bus.said++;
		printf("lx: about to write the chip's %#x: %#x\n", offset, value);
	}
	__asm __volatile("dsb sy");		/* (what was written to a ring is in memory before the radio is told of it) */
	if ((wake = wake_needed(ab, offset))) {
		apollo_wifi_wake(1);
	}
	apollo_wifi_write32(offset, value);
	if (wake) {
		apollo_wifi_wake(0);
	}
}

/*
 * In place of the radio's interrupts: each copy engine is looked at as its interrupt would have
 * ath11k do (ce.c: ath11k_ce_per_engine_service(), which takes what has been received to its
 * owner and posts fresh buffers), and once the driver has "enabled the data path's interrupts"
 * so is each of the groups its rings are in (dp.c: ath11k_dp_service_srng(), which is what the
 * driver's NAPI poll calls: frames received, frames that have been sent, errors, the reorder
 * block's answers). Once a millisecond while nothing comes; again at once while a group has
 * used up what it was allowed in one go.
 *
 * With the radio's interrupts (wifi_irq=1) the rings are looked at when one arrives, on the
 * kernel's thread for them (irq_ce(), irq_group() below), and this thread looks only every 100 ms,
 * to count what the interrupts left behind and to take care of it.
 */
#define DATA_BUDGET	64

static void
bus_looks(void *argument)
{
	struct ath11k_base *ab = argument;
	unsigned long long said = lx_os_nanoseconds();

	bus.looks = true;
	while (bus.looking) {
		bool more = false;

		if (lx_os_nanoseconds() - said > 300000000000ull) {	/* every five minutes: how the station and memory stand */
			said = lx_os_nanoseconds();
			lx_station_say();
			lx_memory_say();
			if (bus.irq) {
				irq_say();
			}
		}

		if (bus.irq) {
			mutex_lock(&bus.service);
		}
		for (unsigned i = 0; i < ab->hw_params.ce_count; i++) {
			ath11k_ce_per_engine_service(ab, i);
		}
		for (unsigned i = 0; bus.data && i < ATH11K_EXT_IRQ_GRP_NUM_MAX; i++) {
			int done = ath11k_dp_service_srng(ab, &ab->ext_irq_grp[i], DATA_BUDGET);

			more |= done >= DATA_BUDGET;
			bus.late += bus.irq && done > 0;
		}
		if (bus.irq) {
			mutex_unlock(&bus.service);
		}
		if (!more) {
			lx_pause_ms(bus.irq ? 100 : 1);
		}
	}
	bus.looks = false;
}

/*
 * The copy engines are looked at no more, and no register is touched from here on. This is what
 * the driver's "interrupts off" comes to here, and it comes in time: on its way down the driver
 * says that after its last word to the firmware over a copy engine and before it asks for the
 * mode "off".
 */
static void
bus_quiet(void)
{
	if (bus.irq && bus.looking) {
		irq_say();
	}
	if (bus.irq) {
		lx_os_irq_detach_all();
	}
	bus.data = false;
	bus.looking = false;
	while (bus.looks) {
		lx_pause_ms(1);
	}
	say_repeats();
}

/*
 * A request of the driver's to the firmware, on its way out (linux/soc/qcom/qmi.h, ours). One
 * matters here: the mode. With any mode but mission the radio's registers may stop answering, and
 * the driver does not always let go of the copy engines first -- where its start fails late,
 * core.c asks for the mode "off" and stops the bus after that (ath11k_core_stop()). So the
 * copy engines are let go of here, before the request leaves.
 */
int
lx_qmi_send_request(struct qmi_handle *qmi, struct sockaddr_qrtr *sq, struct qmi_txn *txn, int msg_id, size_t largest,
    const struct qmi_elem_info *ei, const void *structure)
{
	if (msg_id == QMI_WLANFW_WLAN_MODE_REQ_V01) {
		u32 mode = ((const struct qmi_wlanfw_wlan_mode_req_msg_v01 *)structure)->mode;

		if (mode != ATH11K_FIRMWARE_MODE_NORMAL && bus.looking) {
			printf("lx: the driver asks for mode %u while the copy engines are still looked at: they are let go of first\n",
			    mode);
		}
		if (mode != ATH11K_FIRMWARE_MODE_NORMAL) {
			bus_quiet();
		}
		lx_tell("lx: the firmware is asked for mode %u (0: mission, 4: off, 7: calibration)\n", mode);
	}
	return (qmi_send_request(qmi, sq, txn, msg_id, largest, ei, structure));
}

/* ---- the radio's interrupts ---------------------------------------------------------------------- */

/* A copy engine's interrupt (pcic.c: ath11k_pcic_ce_interrupt_handler() and its tasklet). */
static void
irq_ce(void *argument)
{
	struct ath11k_ce_pipe *pipe = argument;

	mutex_lock(&bus.service);
	ath11k_ce_per_engine_service(pipe->ab, pipe->pipe_num);
	mutex_unlock(&bus.service);
	lx_os_irq_enable(bus.ce_vector[pipe->pipe_num]);
}

/* A data path group's (pcic.c: ath11k_pcic_ext_interrupt_handler() and its NAPI poll, until it has less to do than allowed). */
static void
irq_group(void *argument)
{
	struct ath11k_ext_irq_grp *group = argument;
	int done;

	mutex_lock(&bus.service);
	do {
		done = bus.data ? ath11k_dp_service_srng(group->ab, group, DATA_BUDGET) : 0;
	} while (done >= DATA_BUDGET);
	mutex_unlock(&bus.service);
	if (bus.data) {
		lx_os_irq_enable(bus.group_vector[group->grp_id]);
	}
}

/* pcic.c: ath11k_pcic_config_irq(), for this chip: which vector is whose, and the functions for them, all masked */
static int
irq_attach(struct ath11k_base *ab)
{
	unsigned index = 0;
	int ret;

	for (unsigned i = 0; i < CE_COUNT_MAX; i++) {
		bus.ce_vector[i] = -1;
	}
	for (unsigned i = 0; i < ATH11K_EXT_IRQ_GRP_NUM_MAX; i++) {
		bus.group_vector[i] = -1;
	}
	for (unsigned i = 0; i < ab->hw_params.ce_count; i++) {
		if (ath11k_ce_get_attr_flags(ab, i) & CE_ATTR_DIS_INTR) {
			continue;
		}
		bus.ce_vector[i] = (int)(index++ % msi_users[USER_CE].num_vectors + msi_users[USER_CE].base_vector);
		if ((ret = lx_os_irq_attach((unsigned)bus.ce_vector[i], irq_ce, &ab->ce.ce_pipe[i])) != 0) {
			return (ret);
		}
	}
	for (unsigned i = 0; i < ATH11K_EXT_IRQ_GRP_NUM_MAX; i++) {
		const struct ath11k_hw_ring_mask *mask = ab->hw_params.ring_mask;

		ab->ext_irq_grp[i].ab = ab;
		ab->ext_irq_grp[i].grp_id = i;
		if (!(mask->tx[i] || mask->rx[i] || mask->rx_err[i] || mask->rx_wbm_rel[i] || mask->reo_status[i] ||
		    mask->rxdma2host[i] || mask->host2rxdma[i] || mask->rx_mon_status[i])) {
			continue;
		}
		bus.group_vector[i] = (int)(i % msi_users[USER_DP].num_vectors + msi_users[USER_DP].base_vector);
		if ((ret = lx_os_irq_attach((unsigned)bus.group_vector[i], irq_group, &ab->ext_irq_grp[i])) != 0) {
			return (ret);
		}
	}
	return (0);
}

/* What came, by vector: for the log. */
static void
irq_say(void)
{
	char line[400] = "";

	for (unsigned v = 0; v < msi_config.total_vectors; v++) {
		unsigned long long count = lx_os_irq_count(v);

		if (count != 0) {
			snprintf(line + strlen(line), sizeof(line) - strlen(line), " %u: %llu,", v, count);
		}
	}
	lx_tell("lx: the radio's interrupts by vector (copy engines 3-12, the data path's groups 14-31):%s the thread found %llu "
	    "times work no interrupt had brought\n", line[0] ? line : " NONE", bus.late);
}

static int
bus_get_user_msi_vector(struct ath11k_base *ab, char *user, int *count, u32 *user_base_data, u32 *base_vector)
{
	for (unsigned i = 0; i < nitems(msi_users); i++) {
		if (strcmp(user, msi_users[i].name) == 0) {
			*count = (int)msi_users[i].num_vectors;
			*base_vector = msi_users[i].base_vector;
			*user_base_data = msi_users[i].base_vector + ab->pci.msi.ep_base_data;
			return (0);
		}
	}
	return (-EINVAL);
}

static void
bus_get_msi_address(struct ath11k_base *ab, u32 *low, u32 *high)
{
	*low = ab->pci.msi.addr_lo;
	*high = ab->pci.msi.addr_hi;
}

/* pcic.c: ath11k_pcic_get_ce_msi_idx(): the copy engines with interrupts, counted */
static void
bus_get_ce_msi_idx(struct ath11k_base *ab, u32 ce_id, u32 *index)
{
	u32 n = 0;

	for (u32 i = 0; i < ab->hw_params.ce_count && i != ce_id; i++) {
		n += !(ath11k_ce_get_attr_flags(ab, i) & CE_ATTR_DIS_INTR);
	}
	*index = n;
}

/* pcic.c: ath11k_pcic_start(), with the thread above where it enables the interrupts */
static int
bus_start(struct ath11k_base *ab)
{
	int ret;

	set_bit(ATH11K_FLAG_DEVICE_INIT_DONE, &ab->dev_flags);
	printf("lx: the driver starts the bus: buffers for the copy engines that receive (%u reads and %u writes of the chip's "
	    "registers so far; the configuration the firmware was sent named %u shadow registers)\n", bus.reads, bus.writes,
	    ab->qmi.ce_cfg.shadow_reg_v2_len);
	ath11k_ce_rx_post_buf(ab);
	say_repeats();
	lx_tell("lx: the rings are set up and the copy engines have their buffers: the bus is started\n");
	/*
	 * (here and not when the driver is set up: the firmware is asked for calibration in between, and
	 * the copy engines are let go of for that, interrupts and all; see lx_qmi_send_request())
	 */
	if (bus.irq && (ret = irq_attach(ab)) != 0) {
		lx_tell("lx: wifi_irq: the radio's interrupts cannot be had (%d): its rings are looked at every 100 ms only\n", ret);
		lx_os_irq_detach_all();
	}
	if (bus.irq) {
		for (unsigned i = 0; i < CE_COUNT_MAX; i++) {
			if (bus.ce_vector[i] >= 0) {
				lx_os_irq_enable((unsigned)bus.ce_vector[i]);
			}
		}
	}
	bus.looking = true;
	if (lx_os_thread("wifi-rings", bus_looks, ab) != 0) {
		bus.looking = false;
		return (-ENOMEM);
	}
	return (0);
}

/* pcic.c: ath11k_pcic_stop() */
static void
bus_stop(struct ath11k_base *ab)
{
	bus_quiet();
	bus.closed = true;
	timer_delete_sync(&ab->rx_replenish_retry);
	ath11k_ce_cleanup_pipes(ab);
	printf("lx: the driver stops the bus (%u reads and %u writes of the chip's registers in all, %u of the writes a ring's "
	    "pointer written what it held)\n", bus.reads, bus.writes, bus.same);
}

/* pcic.c: ath11k_pcic_map_service_to_pipe(): which copy engines carry a service, by the chip's table */
static int
bus_map_service_to_pipe(struct ath11k_base *ab, u16 service, u8 *up, u8 *down)
{
	bool has_up = false, has_down = false;

	for (u32 i = 0; i < ab->hw_params.svc_to_ce_map_len; i++) {
		const struct service_to_pipe *entry = &ab->hw_params.svc_to_ce_map[i];
		u32 direction = __le32_to_cpu(entry->pipedir);

		if (__le32_to_cpu(entry->service_id) != service) {
			continue;
		}
		if (direction == PIPEDIR_IN || direction == PIPEDIR_INOUT) {
			*down = (u8)__le32_to_cpu(entry->pipenum);
			has_down = true;
		}
		if (direction == PIPEDIR_OUT || direction == PIPEDIR_INOUT) {
			*up = (u8)__le32_to_cpu(entry->pipenum);
			has_up = true;
		}
	}
	return (has_up && has_down ? 0 : -ENOENT);
}

static int
bus_power_up(struct ath11k_base *ab)
{
	int ret = apollo_wifi_power_up();

	bus.powered = ret == 0;
	if (ret != 0) {
		lx_tell("lx: the chip does not come up: %d\n", ret);
	}
	return (-ret);
}

static void
bus_power_down(struct ath11k_base *ab, bool is_suspend)
{
	bus_quiet();
	bus.powered = false;
	apollo_wifi_power_down();
}

/* pcic.c: ath11k_pcic_ext_irq_enable(): the data path's rings are set up, and are looked at from here on */
static void
bus_irq_enable(struct ath11k_base *ab)
{
	for (unsigned i = 0; i < ATH11K_EXT_IRQ_GRP_NUM_MAX; i++) {
		ab->ext_irq_grp[i].ab = ab;
		ab->ext_irq_grp[i].grp_id = i;
	}
	bus.data = true;
	for (unsigned i = 0; bus.irq && i < ATH11K_EXT_IRQ_GRP_NUM_MAX; i++) {
		if (bus.group_vector[i] >= 0) {
			lx_os_irq_enable((unsigned)bus.group_vector[i]);
		}
	}
}

static void
bus_irq_disable(struct ath11k_base *ab)
{
	bus_quiet();
}

static const struct ath11k_hif_ops bus_ops = {
	.read32 = bus_read32,
	.write32 = bus_write32,
	.irq_enable = bus_irq_enable,
	.irq_disable = bus_irq_disable,
	.start = bus_start,
	.stop = bus_stop,
	.map_service_to_pipe = bus_map_service_to_pipe,
	.power_up = bus_power_up,
	.power_down = bus_power_down,
	/* (no interrupt vectors are offered: the rings are then set up without any, see ce.c) */
};

/* ... and with the radio's interrupts: the vectors, and where the radio writes to raise one */
static const struct ath11k_hif_ops bus_ops_irq = {
	.read32 = bus_read32,
	.write32 = bus_write32,
	.irq_enable = bus_irq_enable,
	.irq_disable = bus_irq_disable,
	.start = bus_start,
	.stop = bus_stop,
	.map_service_to_pipe = bus_map_service_to_pipe,
	.power_up = bus_power_up,
	.power_down = bus_power_down,
	.get_user_msi_vector = bus_get_user_msi_vector,
	.get_msi_address = bus_get_msi_address,
	.get_ce_msi_idx = bus_get_ce_msi_idx,
};

/* ---- the device tree, of which there is none ------------------------------------------------------ */

int
of_property_read_string(const struct device_node *np, const char *name, const char **out)
{
	return (-EINVAL);
}

bool
of_machine_is_compatible(const char *compatible)
{
	return (false);
}

int
of_reserved_mem_region_to_resource(const struct device_node *np, unsigned int index, struct resource *res)
{
	return (-ENODEV);
}

/*
 * The radio's own address, which the driver asks its device for before it falls back on what the
 * firmware says (mac.c: ath11k_mac_register()); this radio's firmware says none. The kernel has
 * it, if it was given one. Nothing here says the address.
 */
int
linuxkpi_device_get_mac_address(struct device *dev, char *address)
{
	return (lx_os_radio_address((unsigned char *)address) ? 0 : -ENOENT);
}

/* ---- what the firmware has said of the radio ------------------------------------------------------- */

/* After "unified ready": what wmi.c has kept of the firmware's two announcements. */
static void
say_radio(struct ath11k_base *ab)
{
	const struct ath11k_targ_cap *t = &ab->target_caps;

	printf("lx: THE FIRMWARE SAYS OF THE RADIO: %u radio(s); %u chains (mask %#x); up to %u channels in a scan, %u MACs; "
	    "transmit power %d..%d; phy %#x, HT %#x, VHT %#x (MCS %#x), system %#x\n", ab->num_radios, t->num_rf_chains,
	    t->txrx_chainmask, t->max_num_scan_channels, t->max_supported_macs, (int)t->hw_min_tx_power, (int)t->hw_max_tx_power,
	    t->phy_capability, t->ht_cap_info, t->vht_cap_info, t->vht_supp_mcs, t->sys_cap_info);
	/* (the second number is from the same structure of "ready" as the address: it shows that was read) */
	printf("lx: its address, by the firmware: %02x:%02x:%02x:%02x:%02x:%02x (and its check sum of the packet log's "
	    "definitions: %#x)\n", ab->mac_addr[0], ab->mac_addr[1], ab->mac_addr[2], ab->mac_addr[3], ab->mac_addr[4],
	    ab->mac_addr[5], ab->pktlog_defs_checksum);
	for (unsigned i = 0; i < ab->num_radios && i < MAX_RADIOS; i++) {
		const struct ath11k_hal_reg_capabilities_ext *r = &ab->hal_reg_cap[i];
		const struct ath11k_pdev_cap *c = &ab->pdevs[i].cap;

		printf("lx: radio %u (pdev %u, phy %u): bands%s%s (%#x); 2.4 GHz %u..%u MHz, 5 GHz and up %u..%u MHz; regulatory "
		    "domain %#x, wireless modes %#x; chains tx %#x rx %#x; VHT %#x, A-MPDU density %u\n", i, ab->pdevs[i].pdev_id,
		    r->phy_id, c->supported_bands & WMI_HOST_WLAN_2G_CAP ? " 2.4 GHz" : "", c->supported_bands &
		    WMI_HOST_WLAN_5G_CAP ? " 5 GHz" : "", c->supported_bands, r->low_2ghz_chan, r->high_2ghz_chan, r->low_5ghz_chan,
		    r->high_5ghz_chan, r->eeprom_reg_domain, r->wireless_modes, c->tx_chain_mask, c->rx_chain_mask, c->vht_cap,
		    c->ampdu_density);
		for (unsigned band = 0; band < NUM_NL80211_BANDS; band++) {
			const struct ath11k_band_cap *b = &c->band[band];

			if (b->max_bw_supported == 0 && b->ht_cap_info == 0 && b->he_cap_info[0] == 0) {
				continue;
			}
			printf("lx: radio %u, band %u (0: 2.4 GHz, 1: 5 GHz, 3: 6 GHz): widest channel code %u, HT %#x, HE %#x %#x "
			    "(MCS %#x), HE at 6 GHz %#x\n", i, band, b->max_bw_supported, b->ht_cap_info, b->he_cap_info[0],
			    b->he_cap_info[1], b->he_mcs, b->he_6ghz_capa);
		}
	}
}

/* ---- the entries ---------------------------------------------------------------------------------- */

static struct ath11k_base *volatile driver;	/* the driver, while it is up: between lx_start() and lx_stop() */

int
lx_start(void)
{
	static struct device device;
	static struct device_node node;
	struct ath11k_base *ab;
	unsigned long long began;
	bool up;
	int ret;

	if (lx_prepare() != 0) {
		lx_tell("lx: no thread, or no memory the radio can be given\n");
		return (ENOMEM);
	}
	/* what the driver has to say while it is being looked into; not in a session that just uses the radio */
	if (!lx_os_option("wifi_trace")) {
		ath11k_debug_mask = 0;
		bus.said = SAID_FIRST;
	} else {
		ath11k_debug_mask = ATH11K_DBG_QMI | ATH11K_DBG_BOOT | ATH11K_DBG_PCI | ATH11K_DBG_WMI | ATH11K_DBG_REG | ATH11K_DBG_MAC;
	}
	/*
	 * The driver's parameter "frame_mode": Ethernet. The firmware then puts frames into 802.11
	 * and takes them out of it, and ciphers them: what passes between the driver and the layer
	 * above is what goes over a cable.
	 */
	ath11k_frame_mode = ATH11K_HW_TXRX_ETHERNET;
	device.of_node = &node;
	device.kobj.name = "wifi";

	/* pci.c: ath11k_pci_probe(), for this chip (the PCI device's id is Qualcomm's QCA6390: 17cb:1101) */
	ab = ath11k_core_alloc(&device, 0, ATH11K_BUS_PCI);
	if (ab == NULL) {
		return (ENOMEM);
	}
	ab->id.vendor = 0x17cb;
	ab->id.device = 0x1101;
	ab->hif.ops = &bus_ops;
	bus.irq = false;
	if (lx_os_option("wifi_irq")) {
		unsigned long long address;
		unsigned data;

		/* (pcic.c: ath11k_pcic_init_msi_config(), and ath11k_pci_alloc_msi(): where the chip raises an interrupt and what it writes) */
		if (lx_os_irq_msi(&address, &data)) {
			mutex_init(&bus.service);
			ab->pci.msi.config = &msi_config;
			ab->pci.msi.addr_lo = (u32)address;
			ab->pci.msi.addr_hi = (u32)(address >> 32);
			ab->pci.msi.ep_base_data = data;
			set_bit(ATH11K_FLAG_MULTI_MSI_VECTORS, &ab->dev_flags);
			ab->hif.ops = &bus_ops_irq;
			bus.irq = true;
		} else {
			lx_tell("lx: wifi_irq: the radio cannot be shown where to raise an interrupt: the rings are looked at every millisecond\n");
		}
	}
	ab->hw_rev = ATH11K_HW_QCA6390_HW20;
	ab->fw_mode = ATH11K_FIRMWARE_MODE_NORMAL;
	if ((ret = ath11k_core_pre_init(ab)) != 0 || (ret = ath11k_hal_srng_init(ab)) != 0 || (ret = ath11k_ce_alloc_pipes(ab)) != 0) {
		lx_tell("lx: the driver does not get as far as its exchange with the firmware: %d\n", ret);
		return (-ret);
	}
	/*
	 * The sleep clock: ath11k tells a QCA6390 to use its own (hw_params.internal_sleep_clock: qmi.c sets
	 * SLEEP_CLOCK_SELECT_INTERNAL in the host capability's nm_modem); the Mi 10T's Android (cnss2 qmi.c,
	 * cnss_wlfw_host_cap_send_sync) never sets nm_modem, so its firmware runs on the board's. Boot-arg wifi_extsleep=1:
	 * as Android.
	 */
	if (lx_os_option("wifi_extsleep")) {
		ab->hw_params.internal_sleep_clock = false;
		printf("lx: the firmware is told to use the board's sleep clock, as Android does (wifi_extsleep)\n");
	}
	/*
	 * Cold boot calibration first (boot-arg wifi_coldcal=1): Android's cnss2 has the firmware calibrate (mode 7) before
	 * mission mode on a first boot and tells it cal_done afterwards; ath11k leaves it out for a QCA6390
	 * (hw_params.coldboot_cal_mm false; core.c: ath11k_core_coldboot_cal_support).
	 */
	if (lx_os_option("wifi_coldcal")) {
		ab->hw_params.coldboot_cal_mm = true;
		printf("lx: the firmware calibrates first (wifi_coldcal: mode 7 before mission mode)\n");
	}
	/*
	 * The regulatory database first (boot-arg wifi_regdb, on by default): Android's cnss2 sends qca6390/regdb.bin as BDF
	 * type 4 before the board file (type 1; its dmesg: "Sending BDF download message, type: 4"); ath11k does that only
	 * where hw_params.supports_regdb, which a QCA6390's entry in core.c does not have (qmi.c: ath11k_qmi_event_server_arrive;
	 * the file's name is regdb.bin, core.c: ath11k_core_fetch_regdb).
	 */
	if (lx_os_option("wifi_regdb")) {
		ab->hw_params.supports_regdb = true;
	}
	ab->qmi.ce_cfg.tgt_ce_len = ab->hw_params.target_ce_count;
	ab->qmi.ce_cfg.tgt_ce = ab->hw_params.target_ce_config;
	ab->qmi.ce_cfg.svc_to_ce_map_len = ab->hw_params.svc_to_ce_map_len;
	ab->qmi.ce_cfg.svc_to_ce_map = ab->hw_params.svc_to_ce_map;
	ab->qmi.service_ins_id = ab->hw_params.qmi_service_ins_id;
	printf("lx: the driver (\"%s\") is set up; its exchange with the firmware begins (registers: %s)\n", ab->hw_params.name,
	    "for real");
	if ((ret = ath11k_core_init(ab)) != 0) {
		lx_tell("lx: the driver's start fails: %d\n", ret);
		return (-ret);
	}

	/*
	 * The rest happens on the driver's own threads, until core.c has been through its start (it
	 * says so with the flag below, whether that went well or not) or the exchange has failed.
	 * It went well if the bus is still up: core.c stops it where it gives up.
	 */
	began = lx_os_nanoseconds();
	while (!test_bit(ATH11K_FLAG_REGISTERED, &ab->dev_flags) && !test_bit(ATH11K_FLAG_QMI_FAIL, &ab->dev_flags) &&
	    lx_os_nanoseconds() - began < 60000000000ull) {
		lx_pause_ms(10);
	}
	up = test_bit(ATH11K_FLAG_REGISTERED, &ab->dev_flags) && bus.looks;
	lx_tell("lx: %llu ms after its start the driver %s; its flags: %#lx\n", (lx_os_nanoseconds() - began) / 1000000ull,
	    up ? "IS UP as far as it is built: the firmware has said \"unified ready\"" : test_bit(ATH11K_FLAG_REGISTERED,
	    &ab->dev_flags) ? "HAS GIVEN UP on the way and wound itself down" : "HAS NOT GOT THROUGH its exchange with the firmware",
	    ab->dev_flags);
	if (!up) {
		lx_pause_ms(600);		/* (for core.c to have wound down: the mode "off") */
		ath11k_qmi_deinit_service(ab);
		return (test_bit(ATH11K_FLAG_REGISTERED, &ab->dev_flags) ? ETIMEDOUT : EIO);
	}
	lx_pause_ms(1000);			/* (for what the firmware says unasked once it is ready) */
	say_radio(ab);

	driver = ab;
	if (lx_os_option("wifi_station")) {
		/* whoever wants a network asks the kernel, and the kernel asks the station: the driver stays up until lx_stop() */
		if ((ret = lx_station_offer()) == 0) {
			return (0);
		}
	} else if (lx_os_option("wifi_join")) {
		if ((ret = lx_station_join()) == 0) {
			if (lx_os_option("wifi_hold")) {	/* (seconds to stay before returning: to see frames pass) */
				lx_pause_ms(1000 * lx_os_option("wifi_hold"));
				lx_station_say();
			}
			return (0);			/* the network is joined, and the driver stays up until lx_stop() */
		}
	} else {
		ret = lx_os_option("wifi_scan") ? lx_station_scan() : 0;
	}
	lx_stop();
	return (ret);
}

/* The thread that counts the ticks, and the memory the radio is shown: once. 0, or an errno. */
int
lx_prepare(void)
{
	return (lx_base_init() != 0 || lx_pool_init() != 0 ? ENOMEM : 0);
}

/* The network left, and the driver down as its own removal does it (the copy engines are let go of on the way: bus_quiet()). */
void
lx_stop(void)
{
	struct ath11k_base *ab = driver;

	if (ab == NULL) {
		return;
	}
	driver = NULL;
	lx_station_withdraw();
	lx_station_leave();
	ath11k_core_deinit(ab);
	lx_tell("lx: the driver is down again\n");
}

int
lx_transmit(const unsigned char *frame, unsigned long length)
{
	return (driver != NULL ? lx_station_send(frame, length) : ENETDOWN);
}
