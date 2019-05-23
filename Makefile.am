asound_module_pcm_uwa_LTLIBRARIES = libasound_module_pcm_uwa.la

asound_module_pcm_uwadir = @ALSA_PLUGIN_DIR@

AM_CFLAGS = -Wall -g @ALSA_CFLAGS@
AM_LDFLAGS = -module -avoid-version -export-dynamic -no-undefined $(LDFLAGS_NOUNDEFINED)

libasound_module_pcm_uwa_la_SOURCES = pcm_uwa.c xuwachannel_accelerator.c \
				      xuwachannel_accelerator_sinit.c uwachannel.h \
				      xuwachannel_accelerator.h xuwachannel_accelerator_hw.h
libasound_module_pcm_uwa_la_LIBADD = @ALSA_LIBS@ -lpthread 

