VERS26=$(findstring 2.6,$(shell uname -r))
ifeq ($(VERS26),2.6)
	include $(PWD)/Makefile.2.6
else
	include $(shell pwd)/Makefile.2.4
endif
