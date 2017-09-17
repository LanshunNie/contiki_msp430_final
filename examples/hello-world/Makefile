CONTIKI_PROJECT = hello-world
all: $(CONTIKI_PROJECT)
CONTIKI_CONF_WITH_ORPL_BITMAP = 1
	
APPS += udp-fragment
APPS += netsynch task-schedule node-function rdc-efficiency wake low-latency

MODULES += core/net/mac/energy-efficiency 
CONTIKI = ../..
include $(CONTIKI)/Makefile.include
