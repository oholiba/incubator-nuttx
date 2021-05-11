include $(TOPDIR)/Make.defs

CSRCS  = ibex_bringup.c ibex_boot.c

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += ibex_appinit.c
endif

include $(TOPDIR)/boards/Board.mk
