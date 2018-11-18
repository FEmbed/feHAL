ifdef CONFIG_ARM
#
# Component Makefile
#
COMPONENT_ADD_INCLUDEDIRS += $(CONFIG_VENDOR)/$(CONFIG_CHIP_SERIES)_HAL_Driver/Inc
COMPONENT_ADD_INCLUDEDIRS += $(CONFIG_VENDOR)/portable/$(CONFIG_CHIP_SERIES)

COMPONENT_SRCDIRS += $(CONFIG_VENDOR)/$(CONFIG_CHIP_SERIES)_HAL_Driver/Src
COMPONENT_SRCDIRS += $(CONFIG_VENDOR)/portable/$(CONFIG_CHIP_SERIES)

# remove all " in paths
COMPONENT_ADD_INCLUDEDIRS := $(shell echo $(COMPONENT_ADD_INCLUDEDIRS))
COMPONENT_SRCDIRS := $(shell echo $(COMPONENT_SRCDIRS))
endif