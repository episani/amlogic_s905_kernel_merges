#
# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

########################

ifeq ($(strip $(WIFI_BAND)),802_11_ABG)
BCM_FW_SRC_FILE_STA := fw_bcm4329_abg.bin
else
BCM_FW_SRC_FILE_STA := fw_bcm4329.bin
endif
BCM_FW_SRC_FILE_AP  := fw_bcm4329_apsta.bin
BCM_FW_SRC_FILE_P2P := fw_bcm4329_p2p.bin

PRODUCT_COPY_FILES += \
    hardware/broadcom/wlan/bcmdhd/firmware/bcm4329/$(BCM_FW_SRC_FILE_STA):system/vendor/firmware/fw_bcmdhd.bin \
    hardware/broadcom/wlan/bcmdhd/firmware/bcm4329/$(BCM_FW_SRC_FILE_AP):system/vendor/firmware/fw_bcmdhd_apsta.bin \
    hardware/broadcom/wlan/bcmdhd/firmware/bcm4329/$(BCM_FW_SRC_FILE_P2P):system/vendor/firmware/fw_bcmdhd_p2p.bin
########################
