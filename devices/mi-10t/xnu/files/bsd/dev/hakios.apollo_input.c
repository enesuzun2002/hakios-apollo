/*
 * The Mi 10T's (apollo) drivers, started from bsd_init right after redwood's (devices/mi-10t/xnu/mi-10t.patch). Only on that phone:
 * the loader puts device=apollo among the boot-args when it has recognised it (xnuloader/loader.c, platform_apollo).
 * Each driver then looks for its own boot-arg (sys/hakios.apollo_input.h).
 */
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_input.h>
#include <sys/hakios.apollo_audio.h>
#include <sys/systm.h>

void
apollo_input_init(void)
{
	char device[24] = "";

	if (!PE_parse_boot_argn("device", device, sizeof(device)) || strcmp(device, "apollo") != 0) {
		return;
	}
	printf("apollo: the Mi 10T's drivers\n");
	apollo_cpufreq_init();
	apollo_display_init();           /* apollo_display=1: the panel's refresh, measured (and apollo_display_hz=N) */
	apollo_backlight_init();         /* apollo_backlight=P: the light of the panel */
	apollo_adreno_init();            /* apollo_gpu=N: the GPU, step by step, before anything else draws or talks */
	apollo_touch_init();
	apollo_buttons_init();
	apollo_battery_init();
	apollo_haptic_init();            /* apollo_haptic=N: the AW8697 vibration motor, /dev/apollo-haptic */
	apollo_audio_init();             /* /dev/apollo-audio, for iOS's audio plug-in */
	apollo_adsp_init();              /* apollo_audio=N: the audio DSP, and from it the speakers */
	apollo_wifi_init();              /* apollo_wifi=N: the QCA6390 on PCIe */
}
