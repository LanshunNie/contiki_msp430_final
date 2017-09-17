all: udp-client udp-server
APPS=servreg-hack
CONTIKI=../../..

CONTIKI_CONF_WITH_ORPL_BITMAP = 1
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

APPS += udp-fragment auto-sleep multi-subnet

#APPS += netsynch task-schedule node-function wake correct-time low-latency low-latency-msg
APPS += netsynch task-schedule node-function wake correct-time rdc-efficiency rdc-control low-latency low-latency-msg

ifdef WITH_COMPOWER
APPS+=powertrace
CFLAGS+= -DCONTIKIMAC_CONF_COMPOWER=1 -DWITH_COMPOWER=1 -DQUEUEBUF_CONF_NUM=4
endif
MODULES += core/net/mac/energy-efficiency  
ifdef SERVER_REPLY
CFLAGS+=-DSERVER_REPLY=$(SERVER_REPLY)
endif
ifdef PERIOD
CFLAGS+=-DPERIOD=$(PERIOD)
endif
MODULES += core/net/ipv6/multicast

CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include
