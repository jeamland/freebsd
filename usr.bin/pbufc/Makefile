PROG=		pbufc
SRCS=		pbufc.c pbufc_parse.y pbufc_scan.l y.tab.h
CLEANFILES+=	y.tab.i

YFLAGS+=	-i

CFLAGS+=	-DYY_NO_UNPUT -DYY_NO_INPUT
CFLAGS+=	-I../../lib/libpbuf

LIBADD=		l

MAN1=

WARNS?=		6

.include <bsd.prog.mk>
