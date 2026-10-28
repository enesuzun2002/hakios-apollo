/*
 * The Xiaomi Mi 10T (apollo): its own drivers, kept apart from redwood's. bsd_init calls apollo_input_init() right after
 * hakios_input_init() (devices/mi-10t/xnu/mi-10t.patch); it does nothing unless the loader says the phone is apollo (device=apollo),
 * and each part needs its own boot-arg as well, so that redwood's drivers (touchscreen=, buttons=, battery=,
 * cpufreq=) stay off on this phone.
 */
#ifndef _SYS_APOLLO_INPUT_H_
#define _SYS_APOLLO_INPUT_H_

void apollo_input_init(void);
void apollo_touch_init(void);   /* apollo_touch=1: the Novatek NT36675 touch screen (bsd/dev/hakios.apollo_touch.c) */
void apollo_buttons_init(void); /* apollo_buttons=1: power and the volume keys of the PM8150 (bsd/dev/hakios.apollo_buttons.c) */
void apollo_battery_init(void); /* apollo_battery=1: the battery, as the UEFI's charger drivers see it (hakios.apollo_battery.c) */
void apollo_cpufreq_init(void); /* apollo_cpufreq=1: every core group at its top speed (bsd/dev/hakios.apollo_cpufreq.c) */
void apollo_display_init(void);         /* apollo_display=1: the panel's timing, measured; apollo_display_hz=N (bsd/dev/hakios.apollo_display.c) */
bool apollo_display_flip(uint64_t phys, uint32_t stride);       /* page flip of the pipe that scans the boot framebuffer (hakios.apollo_display.c) */
bool apollo_display_flip_latched(void);         /* the last flip has taken effect (the hardware cleared the CTL_FLUSH bit) */
/* apollo_flip=1: three frames to draw into and flip between (hakios.apollo_display.c); one thread (the presenter) only */
int apollo_scanout_get(unsigned *count, vm_offset_t va[3], uint64_t pa[3], uint32_t *stride);   /* va[0] in: frame 0's mapping; count 0: no flips */
bool apollo_scanout_flip(unsigned index, bool gpu);     /* flip to frame index (gpu: drawn by the GPU, with its stride); over a pending one, or queued; false: refused */
int apollo_scanout_take(void);                  /* a free (or, apollo_flip_latest=1, the queued) frame for the presenter's CPU copy, reserved until its flip; -1: none now */
void apollo_scanout_release(unsigned index);    /* a reserved frame given back without a flip */
/* apollo_gpu_present=1: frames the GPU (the window server's Mesa) scales into; ApolloGPU's kApolloScanout */
bool apollo_scanout_gpu_info(unsigned index, uint64_t *phys, uint32_t *stride, uint32_t *width, uint32_t *height, uint64_t *bytes, vm_offset_t *va);
int apollo_scanout_acquire(uint32_t timeout_ms);        /* a free frame for the GPU, reserved for the caller; waits; -1: none */
void apollo_scanout_mark(unsigned index, uint32_t swap);        /* the SwapEnd with this number is of that frame */
int apollo_scanout_take_mark(uint32_t swap, uint32_t surface);  /* present(): the frame swap `swap` (whose layer 0 is `surface`) was drawn into by the GPU, or -1 */
bool apollo_scanout_latched(void);              /* a flip may be asked for now (always: they are queued); the front updated */
void apollo_scanout_wait(uint32_t timeout_us);  /* (returns at once: the flips are queued, apollo_scanout_latched is true) */
unsigned apollo_scanout_front(void);            /* the frame the panel scans */
/* apollo_frames=1 statistics of the flips (bsd/dev/hakios.apollo_frames.c), from hakios.apollo_display.c, under its scan_lock */
#define APOLLO_FRAMES_FLIP_AT_ONCE      0       /* written to the pipe at once: nothing was pending */
#define APOLLO_FRAMES_FLIP_QUEUED       1       /* queued behind a pending flip */
#define APOLLO_FRAMES_FLIP_APPLIED      2       /* a queued one written; ns: after the frame start (the latch's line) */
#define APOLLO_FRAMES_FLIP_LATCHED      3       /* a flip seen latched; ns: since it was asked for */
#define APOLLO_FRAMES_FLIP_REPLACED     4       /* a queued one replaced by a newer (latest wins): never shown */
#define APOLLO_FRAMES_FLIP_TAKEN_BACK   5       /* a queued one taken back to draw a newer picture into: never shown */
#define APOLLO_FRAMES_FLIP_FAILED       6       /* a queued one that could not be written */
#define APOLLO_FRAMES_FLIP_NONE_FREE    7       /* apollo_scanout_take found no frame */
#define APOLLO_FRAMES_FLIP_SUPERSEDED   8       /* a pending one written over by a newer (latest wins): (almost always) never shown */
#define APOLLO_FRAMES_FLIP_KINDS        9
void apollo_frames_flip(unsigned what, uint64_t ns);
void apollo_frames_acquire(uint64_t wait_abs, bool got);       /* the window server's apollo_scanout_acquire: its wait */
/* apollo_hitch=1 (with apollo_frames=1): what the processors run while CoreAnimation hands over no frame (bsd/dev/hakios.apollo_hitch.c) */
void apollo_hitch_init(void);
void apollo_hitch_swap(uint64_t last_abs, uint64_t now_abs);    /* apollo_frames_swap: two swaps of CA's */
void apollo_hitch_window(bool begin, uint64_t window_ns);       /* apollo_frames_shown: a 2 s window begins, or has been told */
uint32_t apollo_gpu_in_flight(void);    /* submissions given to the GPU and not done yet; no lock (hakios.apollo_adreno.c) */
/* a kernel thread's name (osfmk/kern/thread.c), which the hitch tracer shows for it instead of "kernel_task" */
void thread_set_thread_name(struct thread *thread, const char *name);
void apollo_backlight_init(void);       /* apollo_backlight=P: the panel's light, the PM8150L's WLED (bsd/dev/hakios.apollo_backlight.c) */
void apollo_backlight_set(uint32_t fraction);   /* iOS's brightness, 0 to 0x10000 */
void apollo_backlight_power(bool on);   /* the display switched off or on */
void apollo_backlight_stop(void);       /* the run ends: the light as the firmware had it */
void apollo_haptic_init(void);  /* apollo_haptic=N: the vibration motor, an Awinic AW8697 (bsd/dev/hakios.apollo_haptic.c) */
void apollo_haptic_quiesce(void);       /* the run ends or a panic's way back: the chip into reset, no lock, no sleep */
void apollo_adreno_init(void);  /* apollo_gpu=N: the Adreno 650, steps 1 to N (bsd/dev/hakios.apollo_adreno.c) */

/*
 * The sound (apollo_audio=N): the audio DSP (bsd/dev/hakios.apollo_adsp.c), the speakers' path through it (hakios.apollo_sound.c).
 * The APR types are the shared ones of sys/hakios.audio.h.
 */
#include <sys/hakios.audio.h>
void apollo_adsp_init(void);
void apollo_adsp_stop(void);            /* the run ends in order: the sound down, the DSP shut down (loader_return.c) */
void apollo_adsp_quiesce(void);         /* a panic's way back: amplifiers off, the DSP stopped, no lock, no sleep */
bool apollo_adsp_apr(uint8_t service, uint16_t port, uint32_t token, uint32_t opcode, const void *payload, uint32_t length,
    uint32_t wait_ms, struct hakios_apr_answer *answer);
void apollo_adsp_apr_events(hakios_apr_events_t handler);
void apollo_adsp_apr_events_second(hakios_apr_events_t handler);
const uint8_t *apollo_adsp_fw_file(const char *name, uint64_t *size);
void apollo_adsp_fw_free(const uint8_t *data, uint64_t size);
void apollo_sound_dsp_up(void);         /* the DSP's audio services are up */
void apollo_sound_dsp_down(void);       /* the DSP is about to be shut down */
void apollo_sound_quiesce(void);        /* a panic's way back */
/* the main microphone (apollo_mic=N, bsd/dev/hakios.apollo_mic.c): its path through the DSP; /dev/hakios-mic for iOS */
void apollo_mic_dsp_up(void);           /* the DSP's audio services are up */
void apollo_mic_dsp_down(void);         /* the DSP is about to be shut down: what the microphone's path asked for given back */
void apollo_mic_quiesce(void);          /* any way back: the codec into reset, the run's lines said again; no lock, no sleep */
bool apollo_audio_dsp_memory(uintptr_t *memory, uint64_t *dsp_address, uint32_t *bytes);
void apollo_audio_dsp_memory_put(void);
/* the two CS35L41 amplifiers (bsd/dev/hakios.apollo_cs35l41.c), as hakios_speaker_* are for the Awinic ones */
bool apollo_cs35l41_prepare(const uint8_t *config, uint64_t size);
bool apollo_cs35l41_start(void);
bool apollo_cs35l41_volume(uint32_t below, uint32_t step_ms);
bool apollo_cs35l41_healthy(void);
void apollo_cs35l41_stop(void);
void apollo_cs35l41_quiesce(void);
uint32_t apollo_cs35l41_monitor(bool tell);

void apollo_touch_screen(bool on);      /* the display is off or on: the touch chip sleeps or wakes (hakios.apollo_touch.c) */
void apollo_cpufreq_screen(bool on);   /* the display off: every core group at its lowest speed (bsd/dev/hakios.apollo_cpufreq.c) */

/* The Wi-Fi (apollo_wifi=N): the QCA6390 on PCIe0 (bsd/dev/hakios.apollo_pcie.c) */
void apollo_wifi_init(void);
void apollo_wifi_quiesce(void);         /* any way back: PERST# and WLAN_EN down (loader_return.c) */
#include <kern/locks.h>
extern lck_mtx_t apollo_smmu_lock;      /* the apps SMMU's match registers and contexts: whoever takes one (hakios.apollo_sound.c, hakios.apollo_pcie.c) */
/* GCC's APCS branch vote register (0x52008) is shared by the touch, haptic, amplifier and PCIe drivers: set bits under one lock */
void apollo_gcc_vote(vm_offset_t vote_register, uint32_t bits);         /* (hakios.apollo_cpufreq.c) */
extern lck_mtx_t apollo_rpmh_tcs1_lock; /* RPMh's TCS 1: apollo_rpmh_bcm_vote (hakios.apollo_cpufreq.c) and the Wi-Fi's tcs_send (hakios.apollo_pcie.c) */
/* the SPMI arbiter: one transfer at a time, whatever the channel (defined in hakios.apollo_battery.c; taken by the battery, buttons and backlight drivers) */
extern lck_spin_t apollo_spmi_bus_lock;

#endif /* _SYS_APOLLO_INPUT_H_ */
