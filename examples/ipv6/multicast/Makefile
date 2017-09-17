DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

CONTIKI_PROJECT = root intermediate sink
all: $(CONTIKI_PROJECT)

CONTIKI_CONF_WITH_ORPL_BITMAP = 1

APPS += udp-fragment auto-sleep multi-subnet

APPS += netsynch task-schedule node-function wake correct-time

CONTIKI = ../../..

MODULES += core/net/ipv6/multicast

CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include
