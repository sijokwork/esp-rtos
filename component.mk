# Component makefile for extras/ds2413

# expected anyone using driver includes it as 'ds2413/ds2413.h'
INC_DIRS += $(ds2413_ROOT)..

# args for passing into compile rule generation
ds2413_SRC_DIR =  $(ds2413_ROOT)

# users can override this setting and get console debug output
DS2413_DEBUG ?= 0
ifeq ($(DS2413_DEBUG),1)
	ds2413_CFLAGS = $(CFLAGS) -DDS2413_DEBUG
endif

$(eval $(call component_compile_rules,ds2413))
