# MSM7x01A
   zreladdr-$(CONFIG_ARCH_MSM7X01A)	:= 0x10008000
params_phys-$(CONFIG_ARCH_MSM7X01A)	:= 0x10000100
initrd_phys-$(CONFIG_ARCH_MSM7X01A)	:= 0x10800000

# MSM7x25
   zreladdr-$(CONFIG_ARCH_MSM7X25)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X25)	:= 0x00200100
initrd_phys-$(CONFIG_ARCH_MSM7X25)	:= 0x0A000000

# MSM7x27
   zreladdr-$(CONFIG_ARCH_MSM7X27)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X27)	:= 0x00200100
initrd_phys-$(CONFIG_ARCH_MSM7X27)	:= 0x0A000000

# MSM7x27A
   zreladdr-$(CONFIG_ARCH_MSM7X27A)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X27A)	:= 0x00200100

# MSM8625
   zreladdr-$(CONFIG_ARCH_MSM8625)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM8625)	:= 0x00200100

# MSM7x30
   zreladdr-$(CONFIG_ARCH_MSM7X30)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X30)	:= 0x00200100
initrd_phys-$(CONFIG_ARCH_MSM7X30)	:= 0x01200000

ifeq ($(CONFIG_MSM_SOC_REV_A),y)
# QSD8x50
   zreladdr-$(CONFIG_ARCH_QSD8X50)	:= 0x20008000
params_phys-$(CONFIG_ARCH_QSD8X50)	:= 0x20000100
initrd_phys-$(CONFIG_ARCH_QSD8X50)	:= 0x24000000
endif

# MSM8x60
   zreladdr-$(CONFIG_ARCH_MSM8X60)	:= 0x40208000

# MSM8960
   zreladdr-$(CONFIG_ARCH_MSM8960)	:= 0x80208000

# MSM8930
   zreladdr-$(CONFIG_ARCH_MSM8930)	:= 0x80208000

# APQ8064
   zreladdr-$(CONFIG_ARCH_APQ8064)	:= 0x80208000

# MSM8974
   zreladdr-$(CONFIG_ARCH_MSM8974)	:= 0x00008000
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v1-cdp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v1-fluid.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v1-liquid.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v1-mtp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v1-rumi.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v1-sim.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v2.0-1-cdp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v2.0-1-fluid.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v2.0-1-liquid.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v2.0-1-mtp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= apq8074-v2.0-1-cdp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= apq8074-v2.0-1-liquid.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= apq8074-v2.0-1-dragonboard.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= apq8074-v2.2-cdp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= apq8074-v2.2-liquid.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= apq8074-v2.2-dragonboard.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v2.2-cdp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v2.2-fluid.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v2.2-liquid.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974-v2.2-mtp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974pro-ab-pm8941-cdp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974pro-ab-pm8941-fluid.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974pro-ab-pm8941-liquid.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974pro-ab-pm8941-mtp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974pro-ac-pm8941-cdp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974pro-ac-pm8941-liquid.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974pro-ac-pm8941-mtp.dtb
        dtb-$(CONFIG_ARCH_MSM8974)	+= msm8974pro-ac-pma8084-pm8941-mtp.dtb

# APQ8084
   zreladdr-$(CONFIG_ARCH_APQ8084)	:= 0x00008000
        dtb-$(CONFIG_ARCH_APQ8084)	+= apq8084-sim.dtb

# MSMKRYPTON
   zreladdr-$(CONFIG_ARCH_MSMKRYPTON)	:= 0x00008000
	dtb-$(CONFIG_ARCH_MSMKRYPTON)	+= msmkrypton-sim.dtb

# MSM9615
   zreladdr-$(CONFIG_ARCH_MSM9615)	:= 0x40808000

# MSM9625
   zreladdr-$(CONFIG_ARCH_MSM9625)	:= 0x00208000
        dtb-$(CONFIG_ARCH_MSM9625)	+= msm9625-v1-cdp.dtb
        dtb-$(CONFIG_ARCH_MSM9625)	+= msm9625-v1-mtp.dtb
        dtb-$(CONFIG_ARCH_MSM9625)	+= msm9625-v1-rumi.dtb
	dtb-$(CONFIG_ARCH_MSM9625)      += msm9625-v2-cdp.dtb
	dtb-$(CONFIG_ARCH_MSM9625)      += msm9625-v2-mtp.dtb
	dtb-$(CONFIG_ARCH_MSM9625)      += msm9625-v2.1-mtp.dtb
	dtb-$(CONFIG_ARCH_MSM9625)      += msm9625-v2.1-cdp.dtb

# MSM8226
   zreladdr-$(CONFIG_ARCH_MSM8226)	:= 0x00008000
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sim.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-fluid.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-v1-cdp.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-v1-mtp.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-v1-qrd-evt.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-v1-qrd-dvt.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-v2-cdp.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-v2-mtp.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-v2-qrd-evt.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-v2-qrd-dvt.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-cdp.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-mtp.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-v1-qrd-skuf.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-v2-qrd-skuf.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= apq8026-v1-xpm.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= apq8026-v1-cdp.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= apq8026-v1-mtp.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= apq8026-v2-xpm.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= apq8026-v2-cdp.dtb
#        dtb-$(CONFIG_ARCH_MSM8226)	+= apq8026-v2-mtp.dtb
#	 dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-v1-mtp-r01.dtb
ifeq ($(CONFIG_MACH_MS01),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-ms013geur-r00.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-ms013geur-r01.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-ms013geur-r02.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-ms013geur-r03.dtb
		dtb-$(CONFIG_ARCH_MSM8226)  += msm8226-sec-ms013geur-r04.dtb
		dtb-$(CONFIG_ARCH_MSM8226)  += msm8226-sec-ms013geur-r05.dtb
else ifeq ($(CONFIG_MACH_MS01_CHN_CTC),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-ms013gctc-r00.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-ms013gctc-r01.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-ms013gctc-r02.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-ms013gctc-r03.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-ms013gctc-r04.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-ms013gctc-r05.dtb
else ifeq ($(CONFIG_MACH_MS01_CHN_CMCC_3G),y)
        dtb-$(CONFIG_ARCH_MSM8226)	+= msm8626-ms01-cmcc-r00.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8626-ms01-cmcc-r01.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8626-ms01-cmcc-r02.dtb
else ifeq ($(CONFIG_MACH_CRATERVE),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-craterve3gctc-r00.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-craterve3gctc-r01.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-craterve3gctc-r02.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-craterve3gctc-r03.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-craterve3gctc-r04.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-craterve3gctc-r05.dtb
else ifeq ($(CONFIG_MACH_CRATERQ),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-craterq3g-r00.dtb
else ifeq ($(CONFIG_MACH_CT01),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-ct013g-r00.dtb
else ifeq ($(CONFIG_MACH_S3VE),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-s3ve013g-r00.dtb
else ifeq ($(CONFIG_MACH_S3VECTC),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-s3vectc3g-r00.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-s3vectc3g-r01.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-s3vectc3g-r03.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-s3vectc3g-r04.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-s3vectc3g-r05.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-s3vectc3g-r06.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-s3vectc3g-r07.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-s3vectc3g-r08.dtb
else ifeq ($(CONFIG_MACH_MILLET3G_EUR),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8226-sec-millet3geur-r00.dtb
else ifeq ($(CONFIG_MACH_MS01_CHN_SGLTE),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01sgltechn-r00.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01sgltechn-r03.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01sgltechn-r05.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01sgltechn-r08.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01sgltechn-r09.dtb
else ifeq ($(CONFIG_MACH_HLITE_CHN_SGLTE),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-hlitesgltechn-r00.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-hlitesgltechn-r01.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-hlitesgltechn-r02.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-hlitesgltechn-r03.dtb
else ifeq ($(CONFIG_MACH_MS01_LTE),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01lteeur-r00.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01lteeur-r03.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01lteeur-r05.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01lteeur-r08.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01lteeur-r09.dtb
else ifeq ($(CONFIG_MACH_CS03_SGLTE),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-cs03sgltechn-r00.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-cs03sgltechn-r03.dtb
else ifeq ($(CONFIG_MACH_Q7_CHN_SGLTE),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-q7sgltechn-r00.dtb
else ifeq ($(CONFIG_MACH_VICTOR_CHN_SGLTE),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-victorsgltechn-r00.dtb
else ifeq ($(CONFIG_MACH_VICTOR_CHN_SGLTE_CU),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-victorsgltechncu-r00.dtb

else ifeq ($(CONFIG_MACH_S3NEO_LTE),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-s3neolteeur-r00.dtb
else ifeq($(CONFIG_MACH_MS01_LTE_KOR),y)
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01ltekor-r00.dtb	
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01ltekor-r06.dtb	
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01ltekor-r07.dtb	
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01ltekor-r08.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01ltekor-r09.dtb
		dtb-$(CONFIG_ARCH_MSM8226)	+= msm8926-sec-ms01ltekor-r10.dtb
endif

# FSM9XXX
   zreladdr-$(CONFIG_ARCH_FSM9XXX)	:= 0x10008000
params_phys-$(CONFIG_ARCH_FSM9XXX)	:= 0x10000100
initrd_phys-$(CONFIG_ARCH_FSM9XXX)	:= 0x12000000

# FSM9900
   zreladdr-$(CONFIG_ARCH_FSM9900)	:= 0x00008000
        dtb-$(CONFIG_ARCH_FSM9900)	:= fsm9900-rumi.dtb
        dtb-$(CONFIG_ARCH_FSM9900)	:= fsm9900-sim.dtb

# MPQ8092
   zreladdr-$(CONFIG_ARCH_MPQ8092)	:= 0x00008000

# MSM8610
   zreladdr-$(CONFIG_ARCH_MSM8610)	:= 0x00008000
        dtb-$(CONFIG_ARCH_MSM8610)	+= msm8610-v1-cdp.dtb
        dtb-$(CONFIG_ARCH_MSM8610)	+= msm8610-v2-cdp.dtb
        dtb-$(CONFIG_ARCH_MSM8610)	+= msm8610-v1-mtp.dtb
        dtb-$(CONFIG_ARCH_MSM8610)	+= msm8610-v2-mtp.dtb
        dtb-$(CONFIG_ARCH_MSM8610)	+= msm8610-rumi.dtb
        dtb-$(CONFIG_ARCH_MSM8610)	+= msm8610-sim.dtb
        dtb-$(CONFIG_ARCH_MSM8610)	+= msm8610-v1-qrd-skuaa.dtb
        dtb-$(CONFIG_ARCH_MSM8610)	+= msm8610-v1-qrd-skuab.dtb
        dtb-$(CONFIG_ARCH_MSM8610)	+= msm8610-v2-qrd-skuaa.dtb
        dtb-$(CONFIG_ARCH_MSM8610)	+= msm8610-v2-qrd-skuab.dtb

# MSMSAMARIUM
   zreladdr-$(CONFIG_ARCH_MSMSAMARIUM)	:= 0x00008000
	dtb-$(CONFIG_ARCH_MSMSAMARIUM)	+= msmsamarium-sim.dtb
	dtb-$(CONFIG_ARCH_MSMSAMARIUM)	+= msmsamarium-rumi.dtb
