# this file is included from makefile

local_src := $(wildcard $(SOURCE_DIR)/$(subdirectory)/*.c)

ifneq (,$(filter $(TOOLCHAIN), C30 XC16))
local_src += $(wildcard $(SOURCE_DIR)/$(subdirectory)/*.s)
else
endif

local_inc := include portable portable/MSVC-MingW
incpath += $(addprefix $(SOURCE_DIR)/$(subdirectory)/,$(local_inc))
$(warning incpath = $(incpath))

$(eval $(call make-library, $(subdirectory)/$(subdirectory).a, $(local_src)))
#$(eval $(call make-target,$(subdirectory)/$(subdirectory).a,$(local_src)))
