/*
 * The Mi 10T's (apollo) Wi-Fi, a QCA6390 on PCIe0: what the PCIe/MHI side (bsd/dev/hakios.apollo_pcie.c) offers the rest of the
 * apollo Wi-Fi stack -- the QRTR transport (bsd/dev/hakios.apollo_qrtr.c) and the OS side of the apollo ath11k object
 * (bsd/dev/hakios.apollo_lx.c, devices/mi-10t/tools/mk-ath11k.sh). Nothing here is redwood's.
 */
#ifndef _SYS_APOLLO_WIFI_H_
#define _SYS_APOLLO_WIFI_H_

#include <stdint.h>

/*
 * The chip from power to mission mode (W1, W2a, W2b with the chip preparation, W2c: AMSS, MHI M0, MSI on) and the IPCR
 * channels (MHI 20 to the chip, 21 from it) started. Idempotent: 0 at once when it is already so. 0 or an errno. Sleeps.
 */
int apollo_wifi_power_up(void);
void apollo_wifi_power_down(void);      /* PERST# down, the rails off (apollo_wifi_quiesce's way, from a thread) */

/*
 * DMA: page-aligned kernel memory (contiguous or not) entered in the chip's SMMU context; *iova is what the chip uses.
 * The mapping stays (nothing is unmapped while the chip may use it). 0 or an errno.
 */
int apollo_wifi_dma_map(void *buffer, unsigned long size, unsigned long long *iova);

/*
 * MSI: the chip's view of the doorbell (an IOVA) and the data of vector 0 (the GIC INTID 800); vector v is INTID
 * apollo_wifi_intid(v) = 800 + v. The chip's capability is set for 32 vectors.
 */
int apollo_wifi_msi(unsigned long long *address, unsigned *data);
unsigned apollo_wifi_intid(unsigned vector);

/* registers of the chip through BAR0, with the window above 0x80000 (ath11k_pcic_read32/write32's way) */
uint32_t apollo_wifi_read32(uint32_t offset);
void apollo_wifi_write32(uint32_t offset, uint32_t value);

/* MHI device wake (Linux mhi_device_get_sync/put: the wake doorbell), for ath11k's register access while the chip may sleep */
void apollo_wifi_wake(int on);

/*
 * IPCR: one QRTR packet per transfer. Send may sleep; 0 or an errno. The receive function is called on a kernel thread
 * (never in interrupt context, may sleep), one packet at a time, with the bytes valid during the call only.
 */
int apollo_ipcr_send(const void *data, unsigned long length);
void apollo_ipcr_set_receive(void (*receive)(const void *data, unsigned long length, void *context), void *context);

/* bsd/dev/hakios.apollo_qrtr.c: installs itself as the kernel's QMI transport (hakios_qmi_set_transport, shared) */
int apollo_qrtr_init(void);

/* the apollo ath11k object (devices/mi-10t/tools/mk-ath11k.sh): its four entry points, weak in hakios.apollo_lx.c when it is absent */
void apollo_lx_reserve(unsigned long size);     /* hakios.apollo_lx.c: the DMA pool in one piece, taken at boot */
void apollo_lx_net_early(void);                 /* hakios.apollo_lx.c: en0 at the kernel's start (wifi_station=1, wifi_mac=) */
int apollo_lx_prepare(void);
int apollo_lx_start(void);
void apollo_lx_stop(void);
int apollo_lx_transmit(const unsigned char *frame, unsigned long length);

#endif /* _SYS_APOLLO_WIFI_H_ */
