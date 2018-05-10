# $FreeBSD$

.PATH:  ${SRCTOP}/sys/dev/aqc

KMOD=	if_aqc
SRCS=	device_if.h bus_if.h pci_if.h ifdi_if.h
SRCS+=	opt_inet.h opt_inet6.h opt_rss.h
SRCS+=	aqc.c aqc_hw.c aqc_txrx.c
SRCS+=	aqc_fw.c aqc_fw_1x.c aqc_fw_2x.c

CFLAGS+= -DIFLIB -ferror-limit=1

.include <bsd.kmod.mk>
