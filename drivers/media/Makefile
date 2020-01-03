#
# Makefile for the kernel multimedia device drivers.
#

media-objs	:= media-device.o media-devnode.o media-entity.o

ifeq ($(CONFIG_MEDIA_CONTROLLER),y)
  obj-$(CONFIG_MEDIA_SUPPORT) += media.o
endif


ifeq ($(CONFIG_SEC_MATISSE_PROJECT),y)
obj-y += common/ platform_tab4/ rc/ video/
else ifeq ($(CONFIG_SEC_MILLET_PROJECT),y)
obj-y += common/ platform_tab4/ rc/ video/
else
obj-y += common/ platform/ rc/ video/
endif
obj-$(CONFIG_VIDEO_DEV) += radio/
obj-$(CONFIG_DVB_CORE)  += dvb/
obj-$(CONFIG_TDMB)  += tdmb/
obj-$(CONFIG_ISDBTMM)   += isdbtmm/
obj-$(CONFIG_ISDBT) += isdbt/
obj-$(CONFIG_MTV) += mtv/
