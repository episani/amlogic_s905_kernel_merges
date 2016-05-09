#
# Makefile for the Amlogic NAND drivers.
#

CURRENT_DIR = $(shell pwd)

INCLUDE_DIR_BASE = $(CURRENT_DIR)/../../../../../..

#$(warning "$(INCLUDE_DIR_BASE)")

EXTRA_CFLAGS += -I$(INCLUDE_DIR_BASE)/common/drivers/amlogic/amlnf/include
EXTRA_CFLAGS += -I$(INCLUDE_DIR_BASE)/common/drivers/amlogic/amlnf/ntd

#obj-$(CONFIG_AML_NFTL_NEW)	+= nftl/

obj-$(CONFIG_AML_NFTL_NEW) += aml_nftl_dev.o
aml_nftl_dev-objs =
aml_nftl_dev-objs += block/aml_nftl_block.o
aml_nftl_dev-objs += block/aml_nftl_init.o
aml_nftl_dev-objs += block/aml_nftl_hw_interface.o
aml_nftl_dev-objs += nftl/aml_nftl_core_20151111.o_shipped

#aml_nftl_dev-objs += nftl/aml_nftl_list.o
#aml_nftl_dev-objs += nftl/aml_nftl_logic.o
#aml_nftl_dev-objs += nftl/aml_nftl_gc.o
#aml_nftl_dev-objs += nftl/aml_nftl_cache.o
#aml_nftl_dev-objs += nftl/aml_nftl_common.o
#aml_nftl_dev-objs += nftl/aml_nftl_create_list.o
#aml_nftl_dev-objs += nftl/aml_nftl_start.o
#aml_nftl_dev-objs += nftl/aml_nftl_interface.o
#aml_nftl_dev-objs += nftl/aml_nftl_tbl_trans.o

