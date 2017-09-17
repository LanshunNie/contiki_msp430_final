
WITH_BR = 0

ifeq ($(WITH_BR),1)
CONTIKI_PROJECT = border-router.hex
PROJECT_SOURCEFILES += slip-bridge.c 
# CONTIKI_TARGET_SOURCEFILES += slip_uart0.c 
WITH_SLIP =1

ifeq ($(PREFIX),)
 PREFIX = aaaa::1/64
endif

else
CONTIKI_PROJECT = nodes-sender.hex rnfd-tool.hex

ifdef PERIOD
CFLAGS=-DPERIOD=$(PERIOD)
endif
endif

all: $(CONTIKI_PROJECT) 

CONTIKI=../../..

CONTIKI_CONF_WITH_ORPL_BITMAP = 1

#linker optimizations
SMALL=1

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

MODULES += core/net/ipv6/multicast
CONTIKI_WITH_IPV6 = 1

APPS += udp-fragment auto-sleep multi-subnet

APPS += netsynch task-schedule correct-time node-function wake

include $(CONTIKI)/Makefile.include

$(CONTIKI)/tools/tunslip6:	$(CONTIKI)/tools/tunslip6.c
	(cd $(CONTIKI)/tools && $(MAKE) tunslip6)

connect-router:	$(CONTIKI)/tools/tunslip6
	sudo $(CONTIKI)/tools/tunslip6 $(PREFIX)

connect-router-cooja:	$(CONTIKI)/tools/tunslip6
	sudo $(CONTIKI)/tools/tunslip6 -a 127.0.0.1 $(PREFIX)
