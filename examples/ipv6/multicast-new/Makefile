CONTIKI_PROJECT = root 
all: $(CONTIKI_PROJECT)

CONTIKI_CONF_WITH_ORPL_BITMAP = 1

APPS += udp-fragment

APPS += netsynch task-schedule node-function wake

CONTIKI = ../../..

CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include
