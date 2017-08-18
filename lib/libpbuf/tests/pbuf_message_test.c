#include <sys/types.h>
#include <sys/sbuf.h>
#include <dlfcn.h>
#include <errno.h>
#include <libutil.h>
#include <math.h>
#include <stdlib.h>

#include <pbuf_message.h>

#include <atf-c.h>

static int malloc_fail_code = 0;

void *
malloc(size_t size)
{
	static void *(*real_malloc)(size_t) = NULL;

	if (real_malloc == NULL) {
		real_malloc = (void *(*)(size_t))dlsym(RTLD_NEXT, "malloc");
	}

	if (malloc_fail_code != 0) {
		errno = malloc_fail_code;
		malloc_fail_code = 0;
		return (NULL);
	}

	return (real_malloc(size));
}

static void
check_encode(void *msg, struct message_def *def, void *expected, size_t length)
{
	struct sbuf *buf;

	buf = sbuf_new_auto();
	ATF_REQUIRE(pbuf_message_encode_sbuf(msg, def, buf) == 0);
	sbuf_finish(buf);

	hexdump(expected, length, "expected: ", HD_OMIT_COUNT|HD_OMIT_CHARS);
	hexdump(sbuf_data(buf), sbuf_len(buf), "encoded:  ",
	    HD_OMIT_COUNT|HD_OMIT_CHARS);

	ATF_REQUIRE_STREQ(sbuf_data(buf), expected);

	sbuf_delete(buf);
}

struct test1_message {
	uint32_t field_id;
	size_t field_name_length;
	char *field_name;
};

struct message_def test1_def[] = {
	MESSAGE_DEF_INT32(test1_message, "id", 1, field_id, 0),
	MESSAGE_DEF_STRING(test1_message, "name", 2, field_name_length, 0),
	MESSAGE_DEF_END
};

ATF_TC_WITHOUT_HEAD(pbuf_simple_message_encode_test);
ATF_TC_BODY(pbuf_simple_message_encode_test, tc)
{
	struct test1_message msg;

	msg.field_id = 23;
	msg.field_name_length = strlen("fnord");
	msg.field_name = "fnord";

	check_encode(&msg, test1_def, "\x08\x17\x12\x05\x66\x6e\x6f\x72\x64",
	    9);
}

ATF_TC_WITHOUT_HEAD(pbuf_simple_message_decode_test);
ATF_TC_BODY(pbuf_simple_message_decode_test, tc)
{
	struct test1_message msg;
	size_t buflen;

	buflen = 9;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_def,
	    "\x08\x17\x12\x05\x66\x6e\x6f\x72\x64", &buflen) == 0);
	ATF_REQUIRE_EQ(buflen, 9);

	ATF_REQUIRE_EQ(msg.field_id, 23);
	ATF_REQUIRE_EQ(msg.field_name_length, 5);
	ATF_REQUIRE_STREQ(msg.field_name, "fnord");

	free(msg.field_name);
}

struct test2_message {
	bool field_b;

	int32_t field_i32;
	uint32_t field_u32;
	int32_t field_s32;

	int64_t field_i64;
	uint64_t field_u64;
	int64_t field_s64;

	uint32_t field_f32;
	int32_t field_sf32;

	uint64_t field_f64;
	int64_t field_sf64;

	float field_f;
	double field_d;

	size_t field_s_length;
	char *field_s;

	size_t field_o_length;
	void *field_o;
};

struct message_def test2_def[] = {
	MESSAGE_DEF_BOOL(test2_message, "b", 1, field_b, 0),
	MESSAGE_DEF_INT32(test2_message, "i32", 2, field_i32, 0),
	MESSAGE_DEF_UINT32(test2_message, "u32", 3, field_u32, 0),
	MESSAGE_DEF_SINT32(test2_message, "s32", 4, field_s32, 0),
	MESSAGE_DEF_INT64(test2_message, "i64", 5, field_i64, 0),
	MESSAGE_DEF_UINT64(test2_message, "u64", 6, field_u64, 0),
	MESSAGE_DEF_SINT64(test2_message, "s64", 7, field_s64, 0),
	MESSAGE_DEF_FIXED32(test2_message, "f32", 8, field_f32, 0),
	MESSAGE_DEF_SFIXED32(test2_message, "sf32", 9, field_sf32, 0),
	MESSAGE_DEF_FIXED64(test2_message, "f64", 10, field_f64, 0),
	MESSAGE_DEF_SFIXED64(test2_message, "sf64", 11, field_sf64, 0),
	MESSAGE_DEF_FLOAT(test2_message, "f", 12, field_f, 0),
	MESSAGE_DEF_DOUBLE(test2_message, "d", 13, field_d, 0),
	MESSAGE_DEF_STRING(test2_message, "s", 14, field_s_length, 0),
	MESSAGE_DEF_BYTES(test2_message, "o", 15, field_o_length, 0),
	MESSAGE_DEF_END
};

ATF_TC_WITHOUT_HEAD(pbuf_everything_message_encode_test);
ATF_TC_BODY(pbuf_everything_message_encode_test, tc)
{
	struct test2_message msg;

	msg.field_b = true;
	msg.field_i32 = 23;
	msg.field_u32 = 55555;
	msg.field_s32 = -55555;
	msg.field_i64 = 55555;
	msg.field_u64 = 8589934592;
	msg.field_s64 = -8589934592;
	msg.field_f32 = 17;
	msg.field_sf32 = -17;
	msg.field_f64 = 8589934575;
	msg.field_sf64 = -8589934575;
	msg.field_f = 23.17;
	msg.field_d = 17.23;
	msg.field_s_length = strlen("fnord");
	msg.field_s = "fnord";
	msg.field_o_length = strlen("\xfe\xed\xc0\xde");
	msg.field_o = "\xfe\xed\xc0\xde";

	check_encode(&msg, test2_def, "\x08\x01\x10\x17\x18\x83\xb2\x03\x20"
	    "\x85\xe4\x06\x28\x83\xb2\x03\x30\x80\x80\x80\x80\x20\x38\xff\xff"
	    "\xff\xff\x3f\x45\x11\x00\x00\x00\x4d\xef\xff\xff\xff\x51\xef\xff"
	    "\xff\xff\x01\x00\x00\x00\x59\x11\x00\x00\x00\xfe\xff\xff\xff\x65"
	    "\x29\x5c\xb9\x41\x69\x7b\x14\xae\x47\xe1\x3a\x31\x40\x72\x05\x66"
	    "\x6e\x6f\x72\x64\x7a\x04\xfe\xed\xc0\xde", 83);
}

ATF_TC_WITHOUT_HEAD(pbuf_everything_message_decode_test);
ATF_TC_BODY(pbuf_everything_message_decode_test, tc)
{
	struct test2_message msg;
	size_t buflen;

	buflen = 83;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test2_def, "\x08\x01\x10"
	    "\x17\x18\x83\xb2\x03\x20\x85\xe4\x06\x28\x83\xb2\x03\x30\x80\x80"
	    "\x80\x80\x20\x38\xff\xff\xff\xff\x3f\x45\x11\x00\x00\x00\x4d\xef"
	    "\xff\xff\xff\x51\xef\xff\xff\xff\x01\x00\x00\x00\x59\x11\x00\x00"
	    "\x00\xfe\xff\xff\xff\x65\x29\x5c\xb9\x41\x69\x7b\x14\xae\x47\xe1"
	    "\x3a\x31\x40\x72\x05\x66\x6e\x6f\x72\x64\x7a\x04\xfe\xed\xc0\xde",
	    &buflen) == 0);
	ATF_REQUIRE_EQ(buflen, 83);

	ATF_REQUIRE_EQ(msg.field_b, true);
	ATF_REQUIRE_EQ(msg.field_i32, 23);
	ATF_REQUIRE_EQ(msg.field_u32, 55555);
	ATF_REQUIRE_EQ(msg.field_s32, -55555);
	ATF_REQUIRE_EQ(msg.field_i64, 55555);
	ATF_REQUIRE_EQ(msg.field_u64, 8589934592);
	ATF_REQUIRE_EQ(msg.field_s64, -8589934592);
	ATF_REQUIRE_EQ(msg.field_f32, 17);
	ATF_REQUIRE_EQ(msg.field_sf32, -17);
	ATF_REQUIRE_EQ(msg.field_f64, 8589934575);
	ATF_REQUIRE_EQ(msg.field_sf64, -8589934575);
	ATF_REQUIRE(fabsf(msg.field_f - (float)23.17) < 0.00001);
	ATF_REQUIRE_EQ(msg.field_d, 17.23);
	ATF_REQUIRE_EQ(msg.field_s_length, 5);
	ATF_REQUIRE_STREQ(msg.field_s, "fnord");
	ATF_REQUIRE_EQ(msg.field_o_length, 4);
	ATF_REQUIRE_STREQ(msg.field_o, "\xfe\xed\xc0\xde");

	free(msg.field_s);
	free(msg.field_o);
}

struct message_def test1_bad_def[] = {
	MESSAGE_DEF_FIELD(test1_message, "id", 1, PBUF_FIELD_TYPE_END + 1,
	    field_id, 0),
	MESSAGE_DEF_STRING(test1_message, "name", 2, field_name_length, 0),
	MESSAGE_DEF_END
};

ATF_TC_WITHOUT_HEAD(pbuf_encode_with_bad_field_type);
ATF_TC_BODY(pbuf_encode_with_bad_field_type, tc)
{
	struct test1_message msg;
	struct sbuf *buf;

	msg.field_id = 23;
	msg.field_name_length = strlen("fnord");
	msg.field_name = "fnord";

	buf = sbuf_new_auto();
	ATF_REQUIRE(pbuf_message_encode_sbuf(&msg, test1_bad_def, buf) != 0);
	sbuf_finish(buf);
	sbuf_delete(buf);
}

ATF_TC_WITHOUT_HEAD(pbuf_decode_bad_tag_varint);
ATF_TC_BODY(pbuf_decode_bad_tag_varint, tc)
{
	struct test1_message msg;
	size_t buflen;

	buflen = 11;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_def,
	    "\x80\x80\x80\x80\x80\x80\x80\x80\x80\x80\x80", &buflen) != 0);
}

ATF_TC_WITHOUT_HEAD(pbuf_decode_bad_wire_type);
ATF_TC_BODY(pbuf_decode_bad_wire_type, tc)
{
	struct test1_message msg;
	size_t buflen;

	buflen = 1;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_def, "\x0f",
	    &buflen) != 0);
}

ATF_TC_WITHOUT_HEAD(pbuf_decode_enomem);
ATF_TC_BODY(pbuf_decode_enomem, tc)
{
	struct test1_message msg;
	size_t buflen;

	buflen = 9;
	malloc_fail_code = ENOMEM;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_def,
	    "\x08\x17\x12\x05\x66\x6e\x6f\x72\x64", &buflen) != 0);
}

ATF_TC_WITHOUT_HEAD(pbuf_decode_with_bad_field_type);
ATF_TC_BODY(pbuf_decode_with_bad_field_type, tc)
{
	struct test1_message msg;
	size_t buflen;

	buflen = 9;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_bad_def,
	    "\x08\x17\x12\x05\x66\x6e\x6f\x72\x64", &buflen) != 0);
}


ATF_TC_WITHOUT_HEAD(pbuf_decode_unknown_length_field);
ATF_TC_BODY(pbuf_decode_unknown_length_field, tc)
{
	struct test1_message msg;
	size_t buflen;

	buflen = 13;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_def,
	    "\x08\x17\x1a\x02\x46\x55\x12\x05\x66\x6e\x6f\x72\x64",
	    &buflen) == 0);
	ATF_REQUIRE_EQ(buflen, 13);

	ATF_REQUIRE_EQ(msg.field_id, 23);
	ATF_REQUIRE_EQ(msg.field_name_length, 5);
	ATF_REQUIRE_STREQ(msg.field_name, "fnord");

	free(msg.field_name);
}

ATF_TC_WITHOUT_HEAD(pbuf_decode_unknown_varint_field);
ATF_TC_BODY(pbuf_decode_unknown_varint_field, tc)
{
	struct test1_message msg;
	size_t buflen;

	buflen = 13;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_def,
	    "\x08\x17\x18\x80\x80\x01\x12\x05\x66\x6e\x6f\x72\x64",
	    &buflen) == 0);
	ATF_REQUIRE_EQ(buflen, 13);

	ATF_REQUIRE_EQ(msg.field_id, 23);
	ATF_REQUIRE_EQ(msg.field_name_length, 5);
	ATF_REQUIRE_STREQ(msg.field_name, "fnord");

	free(msg.field_name);
}

ATF_TC_WITHOUT_HEAD(pbuf_decode_unknown_fixed32_field);
ATF_TC_BODY(pbuf_decode_unknown_fixed32_field, tc)
{
	struct test1_message msg;
	size_t buflen;

	buflen = 14;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_def,
	    "\x08\x17\x1d\x80\x80\x80\x80\x12\x05\x66\x6e\x6f\x72\x64",
	    &buflen) == 0);
	ATF_REQUIRE_EQ(buflen, 14);

	ATF_REQUIRE_EQ(msg.field_id, 23);
	ATF_REQUIRE_EQ(msg.field_name_length, 5);
	ATF_REQUIRE_STREQ(msg.field_name, "fnord");

	free(msg.field_name);
}

ATF_TC_WITHOUT_HEAD(pbuf_decode_unknown_fixed64_field);
ATF_TC_BODY(pbuf_decode_unknown_fixed64_field, tc)
{
	struct test1_message msg;
	size_t buflen;

	buflen = 18;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_def,
	    "\x08\x17\x19\x80\x80\x80\x80\x80\x80\x80\x80\x12\x05\x66\x6e\x6f"
	    "\x72\x64", &buflen) == 0);
	ATF_REQUIRE_EQ(buflen, 18);

	ATF_REQUIRE_EQ(msg.field_id, 23);
	ATF_REQUIRE_EQ(msg.field_name_length, 5);
	ATF_REQUIRE_STREQ(msg.field_name, "fnord");

	free(msg.field_name);
}

ATF_TC_WITHOUT_HEAD(pbuf_encode_bytes_test);
ATF_TC_BODY(pbuf_encode_bytes_test, tc)
{
	char	expected[] = "\x08\x17\x12\x05\x66\x6e\x6f\x72\x64\0";
	size_t	length = 9;

	char	buffer[10];
	size_t	buflen;

	buflen = 10;

	struct test1_message msg;

	msg.field_id = 23;
	msg.field_name_length = strlen("fnord");
	msg.field_name = "fnord";

	ATF_REQUIRE(pbuf_message_encode_bytes(&msg, test1_def, buffer,
	    &buflen) == 0);

	hexdump(expected, length, "expected: ", HD_OMIT_COUNT|HD_OMIT_CHARS);
	hexdump(buffer, buflen, "encoded:  ", HD_OMIT_COUNT|HD_OMIT_CHARS);

	ATF_REQUIRE_STREQ(expected, buffer);
}

ATF_TC_WITHOUT_HEAD(pbuf_decode_sbuf_test);
ATF_TC_BODY(pbuf_decode_sbuf_test, tc)
{
	char data[] = "\x08\x17\x12\x05\x66\x6e\x6f\x72\x64";
	struct test1_message msg;
	struct sbuf *buf;

	buf = sbuf_new_auto();
	sbuf_bcat(buf, data, 9);
	sbuf_finish(buf);

	ATF_REQUIRE(pbuf_message_decode_sbuf(&msg, test1_def, buf) == 0);

	sbuf_delete(buf);

	ATF_REQUIRE_EQ(msg.field_id, 23);
	ATF_REQUIRE_EQ(msg.field_name_length, 5);
	ATF_REQUIRE_STREQ(msg.field_name, "fnord");

	free(msg.field_name);
}

struct message_def test1_req_def[] = {
	MESSAGE_DEF_FIELD(test1_message, "id", 1, PBUF_FIELD_TYPE_INT32,
	    field_id, MDF_FIELD_REQUIRED),
	MESSAGE_DEF_STRING(test1_message, "name", 2, field_name_length, 0),
	MESSAGE_DEF_END
};

ATF_TC_WITHOUT_HEAD(pbuf_decode_required_field_test);
ATF_TC_BODY(pbuf_decode_required_field_test, tc)
{
	struct test1_message msg;
	size_t buflen;

	bzero(&msg, sizeof(struct test1_message));

	buflen = 0;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_def, "",
	    &buflen) == 0);
	ATF_REQUIRE_EQ(buflen, 0);

	ATF_REQUIRE_EQ(msg.field_id, 0);
	ATF_REQUIRE_EQ(msg.field_name_length, 0);
	ATF_REQUIRE_EQ(msg.field_name, NULL);

	bzero(&msg, sizeof(struct test1_message));

	buflen = 0;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_req_def, "",
	    &buflen) != 0);

	bzero(&msg, sizeof(struct test1_message));

	buflen = 7;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_req_def,
	    "\x12\x05\x66\x6e\x6f\x72\x64", &buflen) != 0);

	bzero(&msg, sizeof(struct test1_message));

	buflen = 2;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_req_def,
	    "\x08\x17", &buflen) == 0);
	ATF_REQUIRE_EQ(buflen, 2);

	ATF_REQUIRE_EQ(msg.field_id, 23);
	ATF_REQUIRE_EQ(msg.field_name_length, 0);
	ATF_REQUIRE_EQ(msg.field_name, NULL);

	malloc_fail_code = ENOMEM;
	ATF_REQUIRE(pbuf_message_decode_bytes(&msg, test1_req_def,
	    "\x08\x17\x12\x05\x66\x6e\x6f\x72\x64", &buflen) != 0);
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, pbuf_simple_message_encode_test);
	ATF_TP_ADD_TC(tp, pbuf_simple_message_decode_test);
	ATF_TP_ADD_TC(tp, pbuf_everything_message_encode_test);
	ATF_TP_ADD_TC(tp, pbuf_everything_message_decode_test);

	ATF_TP_ADD_TC(tp, pbuf_encode_with_bad_field_type);
	ATF_TP_ADD_TC(tp, pbuf_decode_bad_tag_varint);
	ATF_TP_ADD_TC(tp, pbuf_decode_bad_wire_type);
	ATF_TP_ADD_TC(tp, pbuf_decode_enomem);
	ATF_TP_ADD_TC(tp, pbuf_decode_with_bad_field_type);

	ATF_TP_ADD_TC(tp, pbuf_decode_unknown_length_field);
	ATF_TP_ADD_TC(tp, pbuf_decode_unknown_varint_field);
	ATF_TP_ADD_TC(tp, pbuf_decode_unknown_fixed32_field);
	ATF_TP_ADD_TC(tp, pbuf_decode_unknown_fixed64_field);

	ATF_TP_ADD_TC(tp, pbuf_encode_bytes_test);
	ATF_TP_ADD_TC(tp, pbuf_decode_sbuf_test);

	ATF_TP_ADD_TC(tp, pbuf_decode_required_field_test);

	return (atf_no_error());
}
