.include <src.opts.mk>

CFLAGS+=		-I.. --coverage -O0
LDFLAGS+=		-L..

TESTSDIR=		${TESTSBASE}/lib/libpbuf

ATF_TESTS_C+=	pbuf_codec_test pbuf_message_test

LIBADD+=		pbuf
LIBADD+=		sbuf
LIBADD+=		util

.include <bsd.test.mk>
