VERS26=$(findstring 2.4,$(shell uname -r))
ifeq ($(VERS26),2.4)
	include $(shell pwd)/Makefile.2.4
else
	include $(PWD)/Makefile.2.6
endif
