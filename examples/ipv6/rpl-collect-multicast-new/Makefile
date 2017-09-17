DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

CONTIKI = ../../..

CONTIKI_CONF_WITH_ORPL_BITMAP = 1

APPS = powertrace collect-view  

APPS += udp-fragment auto-sleep multi-subnet

APPS += netsynch task-schedule correct-time node-function wake


CONTIKI_PROJECT = udp-sender udp-sink
PROJECT_SOURCEFILES += collect-common.c

MODULES += core/net/ipv6/multicast

ifdef PERIOD
CFLAGS=-DPERIOD=$(PERIOD)
endif

all: $(CONTIKI_PROJECT)

CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include
