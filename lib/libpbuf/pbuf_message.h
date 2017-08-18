#ifndef _PBUF_MESSAGE_H_
#define	_PBUF_MESSAGE_H_

typedef enum {
	PBUF_FIELD_TYPE_INT32,
	PBUF_FIELD_TYPE_INT64,
	PBUF_FIELD_TYPE_UINT32,
	PBUF_FIELD_TYPE_UINT64,
	PBUF_FIELD_TYPE_SINT32,
	PBUF_FIELD_TYPE_SINT64,
	PBUF_FIELD_TYPE_FIXED32,
	PBUF_FIELD_TYPE_FIXED64,
	PBUF_FIELD_TYPE_SFIXED32,
	PBUF_FIELD_TYPE_SFIXED64,
	PBUF_FIELD_TYPE_BOOL,
	PBUF_FIELD_TYPE_FLOAT,
	PBUF_FIELD_TYPE_DOUBLE,
	PBUF_FIELD_TYPE_STRING,
	PBUF_FIELD_TYPE_BYTES,

	PBUF_FIELD_TYPE_END,
} pbuf_field_type_t;

struct message_def {
	const char *		md_name;
	uint64_t		md_field;
	pbuf_field_type_t	md_field_type;
	size_t			md_offset;
	uint32_t		md_flags;
};

#define	MDF_FIELD_REQUIRED	0x00000001

#define	MESSAGE_FIELD_REQUIRED(f) ((f)->md_flags & MDF_FIELD_REQUIRED)

#define	MESSAGE_DEF_FIELD(sname, fname, id, pbtype, mname, flags) \
	{ fname, id, pbtype, offsetof(struct sname, mname), flags }

#define	MESSAGE_DEF_INT32(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_INT32, mname, \
	   flags)
#define	MESSAGE_DEF_INT64(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_INT64, mname, \
	   flags)
#define	MESSAGE_DEF_UINT32(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_UINT32, mname, \
	   flags)
#define	MESSAGE_DEF_UINT64(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_UINT64, mname, \
	   flags)
#define	MESSAGE_DEF_SINT32(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_SINT32, mname, \
	   flags)
#define	MESSAGE_DEF_SINT64(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_SINT64, mname, \
	   flags)
#define	MESSAGE_DEF_FIXED32(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_FIXED32, mname, \
	   flags)
#define	MESSAGE_DEF_FIXED64(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_FIXED64, mname, \
	   flags)
#define	MESSAGE_DEF_SFIXED32(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_SFIXED32, mname, \
	   flags)
#define	MESSAGE_DEF_SFIXED64(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_SFIXED64, mname, \
	   flags)
#define	MESSAGE_DEF_BOOL(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_BOOL, mname, \
	   flags)
#define	MESSAGE_DEF_FLOAT(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_FLOAT, mname, \
	   flags)
#define	MESSAGE_DEF_DOUBLE(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_DOUBLE, mname, \
	   flags)
#define	MESSAGE_DEF_STRING(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_STRING, mname, \
	   flags)
#define	MESSAGE_DEF_BYTES(sname, fname, id, mname, flags) \
	MESSAGE_DEF_FIELD(sname, fname, id, PBUF_FIELD_TYPE_BYTES, mname, \
	   flags)

#define	MESSAGE_DEF_END { NULL, 0, PBUF_FIELD_TYPE_END, 0,  }

struct sbuf;

int	pbuf_message_encode_bytes(void *msg, struct message_def *def,
			    void *buffer, size_t *buflenp);
int	pbuf_message_encode_sbuf(void *msg, struct message_def *def,
			    struct sbuf *buf);
int	pbuf_message_decode_bytes(void *msg, struct message_def *def,
			    const void *buffer, size_t *buflenp);
int	pbuf_message_decode_sbuf(void *msg, struct message_def *def,
			    struct sbuf *buf);


#endif /* _PBUF_MESSAGE_H_ */