##########################################
########## Amlogic Drivers ###############
##########################################

##########################################
## The order of directories matter.
## Do not change.
##########################################


obj-$(CONFIG_MESON_TIMER)       += clocksource/

obj-$(CONFIG_AM_UART)           += uart/

obj-$(CONFIG_CPU_FREQ)          += cpufreq/

obj-$(CONFIG_AML_CPU_VERSION)   += cpu_version/

obj-$(CONFIG_PINCTRL_AMLOGIC)   += pinctrl/

obj-$(CONFIG_AMLOGIC_USB)       += usb/

obj-$(CONFIG_AMLOGIC_CLK)       += clk/

obj-$(CONFIG_AML_HDMI_TX)       += hdmi/

obj-$(CONFIG_AML_RTC)           += rtc/

obj-$(CONFIG_AML_VRTC)          += vrtc/

obj-$(CONFIG_AM_IRBLASTER)      += irblaster/

obj-$(CONFIG_I2C_AML)           += i2c/

obj-$(CONFIG_AML_POWER_SUPPORT) += power/

obj-$(CONFIG_AML_DEBUG)         += aml_debug/

obj-$(CONFIG_AML_REG_ACCESS)    += reg_access/

obj-$(CONFIG_REGMAP_MMIO)       += iomap/

obj-$(CONFIG_AMLOGIC_SEC)       += secmon/

obj-$(CONFIG_AMLOGIC_SECURITY_KEY)      += securitykey/

obj-$(CONFIG_AM_PHY)            += ethernet/

obj-$(CONFIG_MMC_AML)           += mmc/

obj-$(CONFIG_AML_VPU)           += vpu/

obj-$(CONFIG_AMLOGIC_ION)       += ion_dev/

obj-$(CONFIG_AML_DISPLAY)       += display/

obj-$(CONFIG_AMLOGIC_LED)       += led/

obj-$(CONFIG_AML_CANVAS)        += canvas/

obj-$(CONFIG_AM_VIDEO)          += amports/

obj-$(CONFIG_DEINTERLACE)   += deinterlace/

obj-$(CONFIG_AML_VFM)           += vfm/

obj-$(CONFIG_EFUSE)             += efuse/

obj-$(CONFIG_KEY_MANAGE)        += key_manage/

obj-$(CONFIG_POST_PROCESS_MANAGER) += ppmgr/

obj-$(CONFIG_AM_WIFI)           += wifi/

ifneq ($(wildcard $(srctree)/../hardware/amlogic/wifi),)
        obj-$(CONFIG_BCMDHD) += ../../../hardware/amlogic/wifi/broadcom/drivers/ap6xxx/bcmdhd_1_201_59_x/
        obj-$(CONFIG_SDIO_DHD_CDC_WIFI_40181_MODULE) += ../../../hardware/amlogic/wifi/broadcom/drivers/ap6xxx/broadcm_40181/
        obj-$(CONFIG_BCMDHD_4354_AMPAK) += ../../../hardware/amlogic/wifi/broadcom/drivers/ap6xxx/broadcm_4354/
        obj-$(CONFIG_RTL8188EU) += ../../../hardware/amlogic/wifi/realtek/drivers/8188eu/rtl8xxx_EU/
        obj-$(CONFIG_RTL8192CU) += ../../../hardware/amlogic/wifi/realtek/drivers/8192cu/rtl8xxx_CU/
        obj-$(CONFIG_RTL8192DU) += ../../../hardware/amlogic/wifi/realtek/drivers/8192du/rtl8xxx_DU/
        obj-$(CONFIG_RTL8192EU) += ../../../hardware/amlogic/wifi/realtek/drivers/8192eu/rtl8192EU/
        obj-$(CONFIG_RTL8189ES) += ../../../hardware/amlogic/wifi/realtek/drivers/8189es/rtl8189ES/
        obj-$(CONFIG_RTL8723BS) += ../../../hardware/amlogic/wifi/realtek/drivers/8723bs/rtl8723BS/
        obj-$(CONFIG_RTL8723AU) += ../../../hardware/amlogic/wifi/realtek/drivers/8723au/rtl8723AU/
        obj-$(CONFIG_RTL8723BU) += ../../../hardware/amlogic/wifi/realtek/drivers/8723bu/rtl8723BU/
        obj-$(CONFIG_RTL8821AU) += ../../../hardware/amlogic/wifi/realtek/drivers/8811au/rtl8811AU/
        obj-$(CONFIG_RTL8812AU) += ../../../hardware/amlogic/wifi/realtek/drivers/8812au/rtl8812AU/
endif

#        obj-$(CONFIG_BCMDHD) += ../../../hardware/amlogic/wifi/broadcom/drivers/usi/bcmdhd/


obj-$(CONFIG_AML_POWER_RESET)   += reset/

obj-$(CONFIG_BT_DEVICE)         += bluetooth/

obj-$(CONFIG_AMLOGIC_IONVIDEO)  += ionvideo/

obj-$(CONFIG_AML_AUDIO_DSP)     += audiodsp/

obj-$(CONFIG_AMAUDIO)           += amaudio/

obj-$(CONFIG_AMAUDIO2)          += amaudio2/

obj-$(CONFIG_V4L_AMLOGIC_VIDEO) += video_dev/

obj-$(CONFIG_V4L_AMLOGIC_VIDEO2) += video_dev/

obj-$(CONFIG_TVIN)              += tvin/

obj-$(CONFIG_AM_VECM)           += amvecm/

obj-$(CONFIG_AML_NAND)          += amlnf/

obj-$(CONFIG_AM_INPUT)          += input/

obj-$(CONFIG_MESON_SUSPEND)     += pm/

obj-$(CONFIG_AM_DVB)            += dvb_tv/

obj-$(CONFIG_MESON_MHU_MBOX)    += mailbox/

obj-$(CONFIG_AML_RDMA)          += rdma/

obj-$(CONFIG_AML_TEMP_SENSOR) += thermal/

obj-$(CONFIG_AUDIO_DATA) += audinfo_kernel/

obj-$(CONFIG_VIDEO_AMLOGIC_CAPTURE) += camera/

obj-$(CONFIG_AML_CODEC_MM)      += codec_mm/

obj-$(CONFIG_AML_WDT)           += watchdog/

obj-$(CONFIG_INSTABOOT)         += instaboot/

obj-$(CONFIG_MESON_PWM) += pwm/


#ifneq ($(wildcard $(srctree)/../hardware/amlogic/nand),)
#        obj-$(CONFIG_AML_NEXT_NAND) += ../../../hardware/amlogic/nand/amlnf_3.14/
#endif


ifneq ($(wildcard $(srctree)/../hardware/amlogic/gpu),)
        obj-$(CONFIG_MALI400) += ../../../hardware/amlogic/gpu/mali/
endif

