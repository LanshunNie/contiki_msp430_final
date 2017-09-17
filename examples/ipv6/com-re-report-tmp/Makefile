CONTIKI = ../../..
APPS = powertrace collect-view  drowsie 
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = sender
PROJECT_SOURCEFILES += collect-common.c util.c

ifdef PERIOD
CFLAGS=-DPERIOD=$(PERIOD)
endif

all: $(CONTIKI_PROJECT)
MODULES += core/net/ipv6/multicast
CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include
