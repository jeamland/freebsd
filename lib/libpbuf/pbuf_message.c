#include <sys/types.h>
#include <sys/queue.h>
#include <sys/sbuf.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "pbuf.h"
#include "pbuf_codec.h"
#include "pbuf_message.h"

typedef enum {
	PBUF_WIRE_TYPE_VARINT = 0,
	PBUF_WIRE_TYPE_64BIT = 1,
	PBUF_WIRE_TYPE_LENGTH_DELIMITED = 2,
	PBUF_WIRE_TYPE_32BIT = 5,
} pbuf_wire_type_t;

#define	min(a, b)	(a > b ? b : a)

struct required_field {
	struct message_def *field;
	SLIST_ENTRY(required_field) fields;
};
SLIST_HEAD(required_fields, required_field);

int
pbuf_message_encode_bytes(void *msg, struct message_def *def, void *buffer,
    size_t *buflenp)
{
	struct sbuf buf;
	int res;

	sbuf_new(&buf, buffer, *buflenp, SBUF_FIXEDLEN);
	res = pbuf_message_encode_sbuf(msg, def, &buf);

	if (res == 0) {
		sbuf_finish(&buf);
		*buflenp = sbuf_len(&buf);
	}

	return (res);
}

int
pbuf_message_encode_sbuf(void *msg, struct message_def *def, struct sbuf *buf)
{
	struct message_def *field;
	int i;
	uint64_t value;
	uint32_t value32;
	pbuf_wire_type_t type;
	bool bvalue;
	void *bytes;
	size_t length;

	for (i = 0; def[i].md_field_type != PBUF_FIELD_TYPE_END; i++) {
		field = &def[i];

		switch(field->md_field_type) {
		case PBUF_FIELD_TYPE_BOOL:
			memcpy(&bvalue, (uint8_t *)msg + field->md_offset,
			    sizeof(bvalue));
			value = bvalue ? 1 : 0;
			type = PBUF_WIRE_TYPE_VARINT;
			break;

		case PBUF_FIELD_TYPE_INT32:
		case PBUF_FIELD_TYPE_UINT32:
		case PBUF_FIELD_TYPE_SINT32:
			memcpy(&value32, (uint8_t *)msg + field->md_offset,
			    sizeof(value32));
			value = value32;
			type = PBUF_WIRE_TYPE_VARINT;
			break;

		case PBUF_FIELD_TYPE_INT64:
		case PBUF_FIELD_TYPE_UINT64:
		case PBUF_FIELD_TYPE_SINT64:
			memcpy(&value, (uint8_t *)msg + field->md_offset,
			    sizeof(value));
			type = PBUF_WIRE_TYPE_VARINT;
			break;

		case PBUF_FIELD_TYPE_FIXED32:
		case PBUF_FIELD_TYPE_SFIXED32:
		case PBUF_FIELD_TYPE_FLOAT:
			memcpy(&value32, (uint8_t *)msg + field->md_offset,
			    sizeof(value32));
			type = PBUF_WIRE_TYPE_32BIT;
			break;

		case PBUF_FIELD_TYPE_FIXED64:
		case PBUF_FIELD_TYPE_SFIXED64:
		case PBUF_FIELD_TYPE_DOUBLE:
			memcpy(&value, (uint8_t *)msg + field->md_offset,
			    sizeof(value));
			type = PBUF_WIRE_TYPE_64BIT;
			break;

		case PBUF_FIELD_TYPE_STRING:
		case PBUF_FIELD_TYPE_BYTES:
			memcpy(&length, (uint8_t *)msg + field->md_offset,
			    sizeof(length));
			memcpy(&bytes,
			    (uint8_t *)msg + field->md_offset + sizeof(size_t),
			    sizeof(void *));
			type = PBUF_WIRE_TYPE_LENGTH_DELIMITED;
			break;

		default:
			return (PBE_BAD_MESSAGE);
		}

		pbuf_encode_varint((field->md_field << 3) | type, buf);

		if (field->md_field_type == PBUF_FIELD_TYPE_SINT32) {
			int32_t tv32 = (int32_t)(value & 0xffffffff);
			value = (uint64_t)(tv32 << 1) ^ (tv32 >> 31);
		} else if (field->md_field_type == PBUF_FIELD_TYPE_SINT64) {
			int64_t tv64 = (int64_t)value;
			value = (uint64_t)(tv64 << 1) ^ (tv64 >> 63);
		}

		switch (type) {
		case PBUF_WIRE_TYPE_VARINT:
			pbuf_encode_varint(value, buf);
			break;

		case PBUF_WIRE_TYPE_32BIT:
			pbuf_encode_fixed32(value32, buf);
			break;

		case PBUF_WIRE_TYPE_64BIT:
			pbuf_encode_fixed64(value, buf);
			break;

		case PBUF_WIRE_TYPE_LENGTH_DELIMITED:
			pbuf_encode_varint((uint64_t)length, buf);
			sbuf_bcat(buf, bytes, length);
			break;
		}

	}

	return (PBE_OK);
}

int
pbuf_message_decode_bytes(void *msg, struct message_def *def,
    const void *bufferp, size_t *buflenp)
{
	size_t		available, consumed, length;
	ssize_t		s;
	const uint8_t *	buffer;
	pbuf_wire_type_t	type;
	uint64_t	tag, field_id, value;
	uint32_t	value32;
	bool		bvalue;
	uint8_t *	bytes;
	int		i;
	struct message_def *field;
	struct required_field *req, *rtmp;
	struct required_fields required;

	consumed = 0;
	value = 0;
	value32 = 0;
	available = *buflenp;
	buffer = (const uint8_t *)bufferp;

	SLIST_INIT(&required);
	for (i = 0; def[i].md_field_type != PBUF_FIELD_TYPE_END; i++) {
		if (MESSAGE_FIELD_REQUIRED(&def[i])) {
			req = malloc(sizeof(struct required_field));
			if (req == NULL) {
				return (PBE_NOMEM);
			}
			req->field = &def[i];
			SLIST_INSERT_HEAD(&required, req, fields);
		}
	}

	while (available > consumed) {
		s = pbuf_decode_varint(buffer + consumed,
		    min(available - consumed, 10), &tag);
		if (s < 0) {
			return (PBE_BAD_MESSAGE);
		}
		consumed += s;

		field_id = tag >> 3;
		type = tag & 0x7;

		switch (type) {
		case PBUF_WIRE_TYPE_VARINT:
			s = pbuf_decode_varint(buffer + consumed,
			    available - consumed, &value);
			break;
		case PBUF_WIRE_TYPE_32BIT:
			s = pbuf_decode_fixed32(buffer + consumed,
			    available - consumed, &value32);
			break;
		case PBUF_WIRE_TYPE_64BIT:
			s = pbuf_decode_fixed64(buffer + consumed,
			    available - consumed, &value);
			break;
		case PBUF_WIRE_TYPE_LENGTH_DELIMITED:
			s = pbuf_decode_varint(buffer + consumed,
			    available - consumed, &length);
			break;
		default:
			s = -1;
			break;
		}
		if (s < 0) {
			return (PBE_BAD_MESSAGE);
		}
		consumed += s;

		field = NULL;
		for (i = 0; def[i].md_field_type != PBUF_FIELD_TYPE_END; i++) {
			if (def[i].md_field == field_id) {
				field = &def[i];
			}
		}
		if (field == NULL) {
			if (type == PBUF_WIRE_TYPE_LENGTH_DELIMITED) {
				consumed += length;
			}
			continue;
		}

		if (field->md_field_type == PBUF_FIELD_TYPE_SINT32 ||
		    field->md_field_type == PBUF_FIELD_TYPE_SINT64) {
			value = (value >> 1) ^ (uint64_t)(-(value & 0x1));
		} else if (field->md_field_type == PBUF_FIELD_TYPE_BOOL) {
			bvalue = value ? true : false;
		}

		if (field->md_field_type == PBUF_FIELD_TYPE_INT32 ||
		    field->md_field_type == PBUF_FIELD_TYPE_UINT32 ||
		    field->md_field_type == PBUF_FIELD_TYPE_SINT32) {
			value32 = (uint32_t)(value & 0xffffffff);
		}

		switch (field->md_field_type) {
		case PBUF_FIELD_TYPE_BOOL:
			memcpy((uint8_t *)msg + field->md_offset, &bvalue,
			    sizeof(bool));
			break;

		case PBUF_FIELD_TYPE_INT32:
		case PBUF_FIELD_TYPE_UINT32:
		case PBUF_FIELD_TYPE_SINT32:
		case PBUF_FIELD_TYPE_FIXED32:
		case PBUF_FIELD_TYPE_SFIXED32:
		case PBUF_FIELD_TYPE_FLOAT:
			memcpy((uint8_t *)msg + field->md_offset, &value32,
			    sizeof(value32));
			break;

		case PBUF_FIELD_TYPE_INT64:
		case PBUF_FIELD_TYPE_UINT64:
		case PBUF_FIELD_TYPE_SINT64:
		case PBUF_FIELD_TYPE_FIXED64:
		case PBUF_FIELD_TYPE_SFIXED64:
		case PBUF_FIELD_TYPE_DOUBLE:
			memcpy((uint8_t *)msg + field->md_offset, &value,
			    sizeof(value));
			break;

		case PBUF_FIELD_TYPE_STRING:
		case PBUF_FIELD_TYPE_BYTES:
			memcpy((uint8_t *)msg + field->md_offset, &length,
			    sizeof(length));
			bytes = malloc((length + 1) * sizeof(uint8_t));
			if (bytes == NULL) {
				return (ENOMEM);
			}
			memcpy(bytes, buffer + consumed, length);
			bytes[length] = 0;
			memcpy((uint8_t *)msg + field->md_offset +
			    sizeof(size_t), &bytes, sizeof(void *));
			consumed += length;
			break;

		default:
			return (PBE_BAD_MESSAGE);
		}

		SLIST_FOREACH_SAFE(req, &required, fields, rtmp) {
			if (field == req->field) {
				SLIST_REMOVE(&required, req, required_field,
				    fields);
				free(req);
				break;
			}
		}
	}

	*buflenp = consumed;

	if (!SLIST_EMPTY(&required)) {
		while (!SLIST_EMPTY(&required)) {
			req = SLIST_FIRST(&required);
			SLIST_REMOVE_HEAD(&required, fields);
			free(req);
		}

		return (PBE_MISSING_REQUIRED);
	}

	return (PBE_OK);
}

int
pbuf_message_decode_sbuf(void *msg, struct message_def *def, struct sbuf *buf)
{
	size_t	buflen;

	buflen = sbuf_len(buf);
	return (pbuf_message_decode_bytes(msg, def, sbuf_data(buf), &buflen));
}