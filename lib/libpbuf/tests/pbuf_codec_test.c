#include <sys/types.h>
#include <sys/sbuf.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <libutil.h>

#include <pbuf_codec.h>

#include <atf-c.h>

#define	BUFLEN	10

#define	DECODER_TEST(pbtype, ctype)					\
static void								\
test_decode_##pbtype(const char *buffer, size_t buffer_len, ctype expected) \
{									\
	ctype	received;						\
	ssize_t	res;							\
	hexdump(buffer, buffer_len, "input: ", HD_OMIT_COUNT|HD_OMIT_CHARS); \
	res = pbuf_decode_##pbtype(buffer, buffer_len, &received);	\
	printf("expected: %ju received %ju\n", (uintmax_t)expected,	\
	    (uintmax_t)received);					\
	ATF_REQUIRE(res >= 0);						\
	ATF_REQUIRE(res == buffer_len);					\
	ATF_REQUIRE(received == expected);				\
}

static void
check_encode(ssize_t elen, const char *expected, struct sbuf *buf)
{

	printf("expected %jd bytes, got %jd bytes\n", elen, sbuf_len(buf));
	ATF_REQUIRE(sbuf_len(buf) == elen);

	hexdump(expected, elen, "expected: ", HD_OMIT_COUNT|HD_OMIT_CHARS);
	hexdump(sbuf_data(buf), sbuf_len(buf), "received: ",
	    HD_OMIT_COUNT|HD_OMIT_CHARS);
	ATF_REQUIRE(memcmp(sbuf_data(buf), expected, sbuf_len(buf)) == 0);
}

#define ENCODER_TEST(pbtype, ctype)					\
static void								\
test_encode_##pbtype(ctype value, const char *expected, size_t expected_len) \
{									\
	struct sbuf *buf;						\
	int	res;							\
	printf("Input: %ju\n", (uintmax_t)value);			\
	buf = sbuf_new_auto();						\
	res = pbuf_encode_##pbtype(value, buf);				\
	ATF_REQUIRE(res >= 0);						\
	sbuf_finish(buf);						\
	check_encode((ssize_t)expected_len, expected, buf);		\
	sbuf_delete(buf);						\
}

/* Varint types */

DECODER_TEST(varint, uint64_t);
ENCODER_TEST(varint, uint64_t);
ATF_TC_WITHOUT_HEAD(pbuf_codec_varint_test);
ATF_TC_BODY(pbuf_codec_varint_test, tc)
{
	uint64_t	value;
	struct sbuf	*buf;

	test_decode_varint("\x00", 1, 0);
	test_decode_varint("\x01", 1, 1);
	test_decode_varint("\x7f", 1, 127);
	test_decode_varint("\x80\x01", 2, 128);
	test_decode_varint("\x96\x01", 2, 150);
	test_decode_varint("\xac\x02", 2, 300);
	test_decode_varint("\x80\x80\x01", 3, 16384);
	test_decode_varint("\xff\xff\xff\xff\xff\xff\xff\xff\xff\x01", 10,
	    UINT64_MAX);

	test_encode_varint(0, "\x00", 1);
	test_encode_varint(1, "\x01", 1);
	test_encode_varint(127, "\x7f", 1);
	test_encode_varint(128, "\x80\x01", 2);
	test_encode_varint(150, "\x96\x01", 2);
	test_encode_varint(300, "\xac\x02", 2);
	test_encode_varint(16384, "\x80\x80\x01", 3);
	test_encode_varint(UINT64_MAX,
	    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\x01", 10);

	ATF_REQUIRE(pbuf_decode_varint("\x80\x80", 2, &value) < 0);

	buf = sbuf_new(NULL, NULL, 4, SBUF_FIXEDLEN);
	ATF_REQUIRE(pbuf_encode_varint(UINT64_MAX, buf) < 0);
	sbuf_delete(buf);
}

DECODER_TEST(fixed64, uint64_t);
ENCODER_TEST(fixed64, uint64_t);
ATF_TC_WITHOUT_HEAD(pbuf_codec_fixed64_test);
ATF_TC_BODY(pbuf_codec_fixed64_test, tc)
{
	uint64_t	value;
	struct sbuf	*buf;

	test_decode_fixed64("\x00\x00\x00\x00\x00\x00\x00\x00", 8, 0);
	test_decode_fixed64("\x17\x00\x00\x00\x00\x00\x00\x00", 8, 23);
	test_decode_fixed64("\x00\x00\x00\x00\x00\x00\x00\x17", 8,
	    1657324662872342528);

	test_encode_fixed64(0, "\x00\x00\x00\x00\x00\x00\x00\x00", 8);
	test_encode_fixed64(578437695752307201,
	    "\x01\x02\x03\x04\x05\x06\x07\x08", 8);
	test_encode_fixed64(72623859790382856,
	    "\x08\x07\x06\x05\x04\x03\x02\x01", 8);

	ATF_REQUIRE(pbuf_decode_fixed64("\x00\x00\x00\x00", 4, &value) < 0);

	buf = sbuf_new(NULL, NULL, 4, SBUF_FIXEDLEN);
	ATF_REQUIRE(pbuf_encode_fixed64(0, buf) < 0);
	sbuf_delete(buf);
}

DECODER_TEST(fixed32, uint32_t);
ENCODER_TEST(fixed32, uint32_t);
ATF_TC_WITHOUT_HEAD(pbuf_codec_fixed32_test);
ATF_TC_BODY(pbuf_codec_fixed32_test, tc)
{
	uint32_t	value;
	struct sbuf	*buf;

	test_decode_fixed32("\x00\x00\x00\x00", 4, 0);
	test_decode_fixed32("\x17\x00\x00\x00", 4, 23);
	test_decode_fixed32("\x00\x00\x00\x17", 4, 385875968);

	test_encode_fixed32(0, "\x00\x00\x00\x00", 4);
	test_encode_fixed32(67305985, "\x01\x02\x03\x04", 4);
	test_encode_fixed32(16909060, "\x04\x03\x02\x01", 4);

	ATF_REQUIRE(pbuf_decode_fixed32("\x00\x00\x00", 3, &value) < 0);

	buf = sbuf_new(NULL, NULL, 3, SBUF_FIXEDLEN);
	ATF_REQUIRE(pbuf_encode_fixed32(0, buf) < 0);
	sbuf_delete(buf);
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, pbuf_codec_varint_test);
	ATF_TP_ADD_TC(tp, pbuf_codec_fixed64_test);
	ATF_TP_ADD_TC(tp, pbuf_codec_fixed32_test);

	return (atf_no_error());
}
