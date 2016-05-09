#
#Makefile for the thermal dirver
#
#$(obj)/aml_thermal.o:
#	$(obj)/aml_thermal.o $(obj)/aml_thermal_f.o FORCE

$(obj)/thermal_clean:
	$(call cmd,clean)

KBUILD_CFLAGS += -Wno-error=date-time
CONFIG_AMLOGIC_THERMAL=m
obj-$(CONFIG_AMLOGIC_THERMAL)+= aml_thermal.o

$(obj)/amlogic_thermal.o: $(obj)/thermal_clean FORCE

aml_thermal-objs =
aml_thermal-objs += amlogic_thermal.o 
aml_thermal-objs += amlogic_thermal_module.o

clean: 
	@find $(srctree) \
	-name "*.mod.*" \
	-o -name ".*.rej" \
	-o -name "*%" \
	-o -name ".*.cmd" \
	-o -name "*.bak" \
	-o -name "Module.symvers" \
	-o -name "modules.order" \
	-o -name "*.o" \
	-o -name "*.ko" | xargs rm -f
