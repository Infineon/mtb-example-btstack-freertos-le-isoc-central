################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Bluetooth LE Isochronous Central Solution Demo Application Makefile.
#
################################################################################
# \copyright
# Copyright 2018-2024, Cypress Semiconductor Corporation (an Infineon company)
# SPDX-License-Identifier: Apache-2.0
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################


################################################################################
# Basic Configuration
################################################################################

# Type of ModusToolbox Makefile Options include:
#
# COMBINED    -- Top Level Makefile usually for single standalone application
# APPLICATION -- Top Level Makefile usually for multi project application
# PROJECT     -- Project Makefile under Application
#
MTB_TYPE=COMBINED

# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager
# ('make library-manager' from command line), which will also update Eclipse IDE launch
# configurations.
TARGET=CYW920829M2EVK-02
$(info TARGET=$(TARGET))

# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=mtb-example-btstack-freertos-le-isoc-central


# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC provided with ModusToolbox software
# ARM     -- ARM Compiler (must be installed separately)
# IAR     -- IAR Compiler (must be installed separately)
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

# Default build configuration. Options include:
#
# Debug -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
# Custom -- build with custom configuration, set the optimization flag in CFLAGS
#
# If CONFIG is manually edited, ensure to update or regenerate launch configurations
# for your IDE.
CONFIG=Debug

# If set to "true" or "1", display full command-lines when building.
VERBOSE=0

# Enable this to use internal FLASH
USE_INTERNAL_FLASH?=0

# BtSpy Debug enable
ENABLE_SPY_TRACES?=0

# Set LED to 1 to enable LED support
# Set LED to 0 or empty to disable LED
LED?=1

# Set BUTTON to 1 to enable button support
# Set BUTTON to empty to disable button support
BUTTON?=1

# Set number of MAX_CONNECTED_DEVICES (peripherals) to be supported,
# this release supports either 1 or 2
# Also configures the # of CIS in the CIG
MAX_CONNECTED_DEVICES?=1

################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=BTFW-TX10
COMPONENTS=FREERTOS WICED_BLE BTFW-ISOC-TX10  iso_data_handler_module_lib gatt_utils_lib nvram_lib


# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=./configs

# Add additional defines to the build process (without a leading -D).
DEFINES+=CY_RETARGET_IO_CONVERT_LF_TO_CRLF CY_RTOS_AWARE CHIP=20829 MAX_CIS_PER_CIG=$(MAX_CONNECTED_DEVICES)
DEFINES+=CYBSP_BT_PLATFORM_CFG_SLEEP_MODE_LP_ENABLED=0

ifeq ($(TARGET),APP_CYW920829M2EVB-01)
 DEFINES+=CYBSP
endif

ifeq ($(USE_INTERNAL_FLASH),1)
 DEFINES+=USE_INTERNAL_FLASH
endif

ifeq ($(ENABLE_SPY_TRACES),1)
 $(info Include BtSpy/ClientControl Interface)
 DEFINES+=ENABLE_BT_SPY_LOG DEBUG_UART_BAUDRATE=3000000
else
 DEFINES+=ENABLE_AIROC_HCI_TRANSPORT_PRINTF=0
endif

# LED sub-library component
ifeq ($(LED),1)
 COMPONENTS+=led_lib
 DEFINES+=LED_SUPPORT=1
 $(info Include LED library)
else
 $(info LED disabled)
endif

# Button sub-library component
ifeq ($(BUTTON),1)
 COMPONENTS+=button_lib
 DEFINES+=BUTTON_SUPPORT
 $(info Include button library)
else
 $(info Button disabled)
endif

# Disable encryption
DEFINES += DISABLE_ENCRYPTION

# When AUTO_PAIRING option is enabled, upon reset, the pairing information erased and enters scanning immediately.
#DEFINES += AUTO_PAIRING

#default trace options
TRACE_APP_DEV?=1
TRACE_GATT?=0
TRACE_LED?=0
TRACE_NVRAM?=0
TRACE_DEVICE?=0
TRACE_ISOC?=1
DEFINES += APP_DEV_TRACE_EN=$(TRACE_APP_DEV) GATT_TRACE=$(TRACE_GATT) LED_TRACE=$(TRACE_LED) NVRAM_TRACE=$(TRACE_NVRAM) DEVICE_TRACE=$(TRACE_DEVICE) ISOC_TRACE=$(TRACE_ISOC)

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT=

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ifeq ($(TOOLCHAIN),IAR)
CFLAGS=--diag_suppress=Ta023,Ta022
else
CFLAGS=
endif

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
LDFLAGS=

# Additional / custom libraries to link in to the application.
LDLIBS=

# Path to the linker script to use (if empty, use the default linker script).
LINKER_SCRIPT=

# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.
POSTBUILD=


################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler's "bin" directory.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# software provided compiler by default).
CY_COMPILER_PATH=

# Locate ModusToolbox helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)

# If you install ModusToolbox software in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder). Make sure you use forward slashes.
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS). On Windows, use forward slashes.)
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

include $(CY_TOOLS_DIR)/make/start.mk
