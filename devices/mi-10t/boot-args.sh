# The Mi 10T's boot-args, for run.sh (a run from a PC) and mk-recovery.sh (iOS from the phone): sourced, from the
# top of the repository. ARGS set beforehand replaces the drivers' list.
# every driver of the phone (devices/mi-10t/xnu, devices/mi-10t/README.md says what each does)
ARGS=${ARGS-"apollo_touch=1 apollo_buttons=1 apollo_battery=1 apollo_cpufreq=1 apollo_ddr=1 apollo_audio=5 apollo_mic=4 apollo_mic_ns=1 apollo_mic_drc=0 apollo_gpu=21 \
apollo_gpu_use=1 gpu_recover=1 apollo_backlight=50 apollo_gpu_speed=1 apollo_display=1 apollo_display_hz=60 \
apollo_haptic=5 apollo_wifi=5 apollo_wifi_rddm=0 wifi_bdf_phone=1 wifi_station=1 apollo_flip=1 \
apollo_gpu_present=1 apollo_charger=3"}
# the radio's address, kept beside the phone's Wi-Fi firmware (one line, AA:BB:CC:DD:EE:FF)
if [ -f fw-wifi-apollo/mac ]; then ARGS="$ARGS wifi_mac=$(head -1 fw-wifi-apollo/mac)"; fi
