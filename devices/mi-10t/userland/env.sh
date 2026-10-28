# The Mi 10T's (apollo) settings for building its images: read by userland/build.sh when DEVICE=mi-10t, before the
# launchd jobs are made (userland/mkjobs.py).
#
# DENY_MORE: daemons left out on this phone only. Its modem is not brought up, so CommCenter is never there:
#   com.apple.voicemail.vmd   Visual Voicemail. It asks for com.apple.commcenter.coretelephony.xpc several times a
#                             second and never stops: one core of eight kept busy, and launchd answering it (the hitch
#                             tracer, apollo_hitch=1, 28 October: vmd 11-14% of all cores in every sample, launchd 2-4%).
export DENY_MORE="${DENY_MORE:+$DENY_MORE }com.apple.voicemail.vmd"
