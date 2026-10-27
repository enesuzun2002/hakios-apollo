/*
 * The Mi 10T's (apollo) copy of the Wi-Fi driver's object is built from the same sources as redwood's
 * (devices/mi-10t/tools/mk-ath11k.sh), and both objects are linked into one kernel. This header, included
 * before anything else, gives everything the object shares with the kernel a name of its own, so that
 * nothing of the two collides: what the object asks of the kernel (bsd/sys/hakios.lx.h, which is read
 * as it is) becomes apollo_lx_os_*, implemented in devices/mi-10t/xnu: hakios.apollo_lx.c, and the four names it shows
 * become apollo_lx_prepare(), apollo_lx_start(), apollo_lx_stop() and apollo_lx_transmit()
 * (bsd/sys/apollo_wifi.h). Everything else is made local to the object when it is tied together.
 * (The kernel's QMI helpers, hakios_qmi_*, and the station boundary, hakios_sta_*, are shared as they are.)
 */
#ifndef _APOLLO_PREFIX_H_
#define _APOLLO_PREFIX_H_

#define lx_os_alloc			apollo_lx_os_alloc
#define lx_os_free			apollo_lx_os_free
#define lx_os_radio_memory		apollo_lx_os_radio_memory
#define lx_os_map_registers		apollo_lx_os_map_registers
#define lx_os_print			apollo_lx_os_print
#define lx_os_tell			apollo_lx_os_tell
#define lx_os_panic			apollo_lx_os_panic
#define lx_os_nanoseconds		apollo_lx_os_nanoseconds
#define lx_os_thread_self		apollo_lx_os_thread_self
#define lx_os_thread			apollo_lx_os_thread
#define lx_os_lock			apollo_lx_os_lock
#define lx_os_unlock			apollo_lx_os_unlock
#define lx_os_sleep			apollo_lx_os_sleep
#define lx_os_wakeup			apollo_lx_os_wakeup
#define lx_os_random			apollo_lx_os_random
#define lx_os_option			apollo_lx_os_option
#define lx_os_firmware			apollo_lx_os_firmware
#define lx_os_rproc_boot		apollo_lx_os_rproc_boot
#define lx_os_radio_address		apollo_lx_os_radio_address
#define lx_os_network			apollo_lx_os_network
#define lx_os_net_attach		apollo_lx_os_net_attach
#define lx_os_net_link			apollo_lx_os_net_link
#define lx_os_net_input			apollo_lx_os_net_input
#define lx_os_irq_msi			apollo_lx_os_irq_msi
#define lx_os_irq_attach		apollo_lx_os_irq_attach
#define lx_os_irq_enable		apollo_lx_os_irq_enable
#define lx_os_irq_disable		apollo_lx_os_irq_disable
#define lx_os_irq_detach_all		apollo_lx_os_irq_detach_all
#define lx_os_irq_count			apollo_lx_os_irq_count
#define lx_os_station			apollo_lx_os_station
#define lx_os_station_lost		apollo_lx_os_station_lost

#define lx_prepare			apollo_lx_prepare
#define lx_start			apollo_lx_start
#define lx_stop				apollo_lx_stop
#define lx_transmit			apollo_lx_transmit

#endif
