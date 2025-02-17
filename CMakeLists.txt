#
# This file is part of AtomVM.
#
# Copyright 2022 Davide Bettio <davide@uninstall.it>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
#

idf_component_register(SRCS
    "display_common.c"
    "display_driver.c"
    "5in65_acep_7c_display_driver.c"
    "ili934x_display_driver.c"
    "memory_display_driver.c"
    "ssd1306_display_driver.c"
    "spi_display.c"
    "backlight_gpio.c"
    PRIV_REQUIRES "libatomvm" "avm_sys" "avm_builtins" "sdmmc" "vfs" "fatfs")

idf_build_set_property(LINK_OPTIONS
"-Wl,--whole-archive ${CMAKE_CURRENT_BINARY_DIR}/lib${COMPONENT_NAME}.a -Wl,--no-whole-archive"
APPEND)
