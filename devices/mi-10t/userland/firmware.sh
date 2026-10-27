# The Mi 10T's (apollo) firmware into the small image: read by userland/build.sh (in userland/, with root/ the image
# being made, SHARE set for an image that is going to somebody else). It is the vendor's and never in the repository:
# devices/mi-10t/pull-firmware.sh takes it off the phone into the ../fw-*-apollo directories, kept apart from the
# POCO's ../fw-*. Whatever is not there is left out, and said so.
apollo_fw=
apollo_fw_install() { # FILE...: those that are there, into root/fw
	for f in "$@"; do
		if [ -f "$f" ]; then
			mkdir -p root/fw
			install -m 0644 "$f" root/fw/
			apollo_fw="$apollo_fw $(basename "$f")"
		fi
	done
}
# the touch chip's firmware (hakios.apollo_touch.c; the NT36675 has no flash and is given it at every start, 136 KB):
# built into the phone's Android kernel, so it is taken out of its boot image
apollo_fw_install ../fw-touch-apollo/novatek_nt36675_j3s_fw01.bin
# the audio DSP's image, adsp.mdt and every adsp.bNN (hakios.apollo_adsp.c, apollo_audio=N)
apollo_fw_install ../fw-audio-apollo/adsp.mdt ../fw-audio-apollo/adsp.b[0-9][0-9]
# the speaker amplifiers' protection firmware (Cirrus CS35L41 DSP, hakios.apollo_cs35l41.c, apollo_amps_dsp=N): the
# speaker's (0x40) and the earpiece's (0x42)
apollo_fw_install ../fw-audio-apollo/cs35l41-dsp1-spk-prot-revb2.wmfw ../fw-audio-apollo/cs35l41-dsp1-spk-prot-revb2.bin \
	../fw-audio-apollo/cs35l41-dsp1-spk-prot-RCV-revb2.bin
# the GPU's (hakios.apollo_adreno.c, apollo_gpu=N): a650_gmu.bin, a650_sqe.fw, a650_zap.mdt and its a650_zap.bNN
apollo_fw_install ../fw-gpu-apollo/a650_*
# the vibration motor's waveforms (hakios.apollo_haptic.c, apollo_haptic=N)
apollo_fw_install ../fw-haptic-apollo/aw8697_haptic.bin
# the QCA6390's (hakios.apollo_pcie.c, apollo_wifi=N): amss.bin (handed to the chip over MHI), m3.bin and board-2.bin
# from linux-firmware (ath11k/QCA6390/hw2.0), the phone's board file bd_j3s.elf (wifi_bdf_phone=1) and regdb.bin
apollo_fw_install ../fw-wifi-apollo/amss.bin ../fw-wifi-apollo/m3.bin ../fw-wifi-apollo/board-2.bin \
	../fw-wifi-apollo/regdb.bin ../fw-wifi-apollo/bd_j3s.elf
# the network to join (wifi_station=1): its name on the first line, its passphrase on the second; never in an
# image that is going to somebody else
if [ -z "$SHARE" ] && [ -f ../fw-wifi-apollo/network ]; then
	install -m 0600 ../fw-wifi-apollo/network root/fw/network
	apollo_fw="$apollo_fw network"
fi
[ -n "$apollo_fw" ] && echo "build.sh: fw/: the Mi 10T's:$apollo_fw" | cut -c1-200
unset -f apollo_fw_install
unset apollo_fw
