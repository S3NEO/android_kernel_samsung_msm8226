#Android makefile to build kernel as a part of Android Build
PERL		= perl

ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
ifeq ($(TARGET_KERNEL_APPEND_DTB), true)
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage-dtb
else
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage
endif
KERNEL_HEADERS_INSTALL := $(KERNEL_OUT)/usr
KERNEL_MODULES_INSTALL := system
KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules
KERNEL_IMG=$(KERNEL_OUT)/arch/arm/boot/Image
$(shell rm -rvf $(KERNEL_OUT)/arch/arm/boot/dts)
DTS_NAMES ?= $(shell $(PERL) -e 'while (<>) {$$a = $$1 if /CONFIG_ARCH_((?:MSM|QSD|MPQ)[a-zA-Z0-9]+)=y/; $$r = $$1 if /CONFIG_MSM_SOC_REV_(?!NONE)(\w+)=y/; $$arch = $$arch.lc("$$a$$r ") if /CONFIG_ARCH_((?:MSM|QSD|MPQ)[a-zA-Z0-9]+)=y/} print $$arch;' $(KERNEL_CONFIG))
KERNEL_USE_OF ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_USE_OF=y/) { $$of = "y"; break; } } print $$of;' kernel/arch/arm/configs/$(KERNEL_DEFCONFIG))

MS01_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_MS01=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
CRATERVE_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_CRATERVE=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
CRATERQ_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_CRATERQ=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
CT01_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_CT01=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
S3VE_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_S3VE=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
S3VECTC_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_S3VECTC=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
MILLET3G_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_MILLET3G_EUR=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
MS01_SGLTE_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_MS01_CHN_SGLTE=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
HLITE_SGLTE_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_HLITE_CHN_SGLTE=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
MS01_LTE_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_MS01_LTE=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
MS01_CMCC_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_MS01_CHN_CMCC_3G=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
CS03_SGLTE_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_CS03_SGLTE=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
MS01_LTE_KOR_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_MS01_LTE_KOR=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
MS01_CTC_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_MS01_CHN_CTC=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
S3NEO_LTE_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_S3NEO_LTE=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
Q7_SGLTE_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_Q7_CHN_SGLTE=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
VICTOR_SGLTE_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_VICTOR_CHN_SGLTE=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))
VICTOR_SGLTE_CU_PROJECT ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_MACH_VICTOR_CHN_SGLTE_CU=y/) { $$of = "y"; break; } } print $$of;' $(KERNEL_CONFIG))

ifeq "$(KERNEL_USE_OF)" "y"
ifeq "$(MS01_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8226-sec-ms013geur*.dts)
endif
ifeq "$(MS01_CTC_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8226-sec-ms013gctc*.dts)
endif
ifeq "$(CRATERVE_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8226-sec-craterve3gctc*.dts)
endif
ifeq "$(CRATERQ_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8226-sec-craterq3g*.dts)
endif
ifeq "$(CT01_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8226-sec-ct013g*.dts)
endif
ifeq "$(S3VE_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8226-sec-s3ve013g*.dts)
endif
ifeq "$(S3VECTC_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8226-sec-s3vectc3g*.dts)
endif
ifeq "$(MILLET3G_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8226-sec-millet3geur*.dts)
endif
ifeq "$(MS01_LTE_PROJECT)" "y"
	ifeq "$(MS01_SGLTE_PROJECT)" "y"
	DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8926-sec-ms01sgltechn*.dts)
	else ifeq "$(HLITE_SGLTE_PROJECT)" "y"
		DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8926-sec-hlitesgltechn*.dts)
	else
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8926-sec-ms01lteeur*.dts)
endif
ifeq "$(CS03_SGLTE_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8926-sec-cs03sgltechn*.dts)
endif
ifeq "$(MS01_LTE_KOR_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8926-sec-ms01ltekor*.dts)
endif
ifeq "$(MS01_CMCC_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8626-ms01-cmcc*.dts)
endif
ifeq "$(S3NEO_LTE_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8926-sec-s3neolteeur*.dts)
endif
ifeq "$(Q7_SGLTE_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8926-sec-q7sgltechn*.dts)
endif
ifeq "$(VICTOR_SGLTE_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8926-sec-victorsgltechn*.dts)
endif
ifeq "$(VICTOR_SGLTE_CU_PROJECT)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/msm8926-sec-victorsgltechncu*.dts)
endif
endif

$(info printing $(DTS_FILES))
$(info printing $(DTS_NAMES))
DTS_FILE = $(lastword $(subst /, ,$(1)))
DTB_FILE = $(addprefix $(KERNEL_OUT)/arch/arm/boot/,$(patsubst %.dts,%.dtb,$(call DTS_FILE,$(1))))
ZIMG_FILE = $(addprefix $(KERNEL_OUT)/arch/arm/boot/,$(patsubst %.dts,%-zImage,$(call DTS_FILE,$(1))))
KERNEL_ZIMG = $(KERNEL_OUT)/arch/arm/boot/zImage
DTC = $(KERNEL_OUT)/scripts/dtc/dtc

define append-dtb
mkdir -p $(KERNEL_OUT)/arch/arm/boot;\
$(foreach DTS_NAME, $(DTS_NAMES), \
   $(foreach d, $(DTS_FILES), \
      $(DTC) -p 1024 -O dtb -o $(call DTB_FILE,$(d)) $(d); \
      cat $(KERNEL_ZIMG) $(call DTB_FILE,$(d)) > $(call ZIMG_FILE,$(d));))
endef
else

define append-dtb
endef
endif

ifeq ($(TARGET_USES_UNCOMPRESSED_KERNEL),true)
$(info Using uncompressed kernel)
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/piggy
else
TARGET_PREBUILT_KERNEL := $(TARGET_PREBUILT_INT_KERNEL)
endif

define mv-modules
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`;\
ko=`find $$mpath/kernel -type f -name *.ko`;\
for i in $$ko; do mv $$i $(KERNEL_MODULES_OUT)/; done;\
fi
endef

define clean-module-folder
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`; rm -rf $$mpath;\
fi
endef

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_CONFIG): $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- $(KERNEL_DEFCONFIG) VARIANT_DEFCONFIG=$(VARIANT_DEFCONFIG) DEBUG_DEFCONFIG=$(DEBUG_DEFCONFIG) SELINUX_DEFCONFIG=$(SELINUX_DEFCONFIG)  SELINUX_LOG_DEFCONFIG=$(SELINUX_LOG_DEFCONFIG) TIMA_DEFCONFIG=$(TIMA_DEFCONFIG)

$(KERNEL_OUT)/piggy : $(TARGET_PREBUILT_INT_KERNEL)
	$(hide) gunzip -c $(KERNEL_OUT)/arch/arm/boot/compressed/piggy.gzip > $(KERNEL_OUT)/piggy

$(TARGET_PREBUILT_INT_KERNEL): $(KERNEL_OUT) $(KERNEL_CONFIG) $(KERNEL_HEADERS_INSTALL)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) INSTALL_MOD_STRIP=1 ARCH=arm CROSS_COMPILE=arm-eabi- modules_install
	$(mv-modules)
	$(clean-module-folder)
	$(append-dtb)

$(KERNEL_HEADERS_INSTALL): $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- headers_install

kerneltags: $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- tags

kernelconfig: $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- menuconfig
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- savedefconfig
	cp $(KERNEL_OUT)/defconfig kernel/arch/arm/configs/$(KERNEL_DEFCONFIG)

endif
