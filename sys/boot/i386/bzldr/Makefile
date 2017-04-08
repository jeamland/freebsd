# $FreeBSD$

# Pick up ../Makefile.inc early.
.include <bsd.init.mk>

PROG=	${LDR}
INTERNALPROG=
FILES=	${BOOT}
MAN=	${BOOT}.8
SRCS=	${LDR}.S
CLEANFILES= ${BOOT}

BOOT=	bzboot
LDR=	bzldr
ORG=	0x0
LOADER=	loader

.if defined(BOOT_BZLDR_PROBE_KEYBOARD)
CFLAGS+=-DPROBE_KEYBOARD
.endif

.if defined(BOOT_BZLDR_ALWAYS_SERIAL)
CFLAGS+=-DALWAYS_SERIAL
.endif

CFLAGS+=-I${.CURDIR}/../common

LOADERBIN= ${.OBJDIR}/../loader/loader.bin

CLEANFILES+= ${LDR}.tmp

${BOOT}: ${LDR} ${LOADER}
	${DD} if=${LDR} of=${LDR}.tmp obs=512 conv=osync
	cat ${LDR}.tmp ${LOADER} > ${.TARGET}
	rm ${LDR}.tmp

LDFLAGS+=${LDFLAGS_BIN}

CLEANFILES+= ${LOADER}

${LOADER}: ${LOADERBIN} ${BTXLDR} ${BTXKERN}
	btxld -v -f aout -e ${LOADER_ADDRESS} -o ${.TARGET} -l ${BTXLDR} \
	    -b ${BTXKERN} ${LOADERBIN}

.include <bsd.prog.mk>

# XXX: clang integrated-as doesn't grok .codeNN directives yet
CFLAGS.bzldr.S=	${CLANG_NO_IAS}
