#include <sys/queue.h>

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pbuf_message.h>

#include "pbufc.h"

static char	*basename, *uppername;
static FILE	*header, *code;

static void	pbufc_start_code(void);
static void	pbufc_finish_code(void);

struct field {
	pbuf_field_type_t	type;
	char			*name;
	uint64_t		id;
	uint32_t		flags;

	STAILQ_ENTRY(field) fields;
};

static STAILQ_HEAD(fieldlist, field) fieldlist =
    STAILQ_HEAD_INITIALIZER(fieldlist);

struct type_mapping {
	pbuf_field_type_t	type;
	const char *		ctype;
	const char *		dtype;
};

static struct type_mapping type_mapping[] = {
	{ PBUF_FIELD_TYPE_INT32, "int32_t", "INT32" },
	{ PBUF_FIELD_TYPE_INT64, "int64_t", "INT64" },
	{ PBUF_FIELD_TYPE_UINT32, "uint32_t", "UINT32" },
	{ PBUF_FIELD_TYPE_UINT64, "uint64_t", "UINT64" },
	{ PBUF_FIELD_TYPE_SINT32, "int32_t", "SINT32" },
	{ PBUF_FIELD_TYPE_SINT64, "int64_t", "SINT64" },
	{ PBUF_FIELD_TYPE_FIXED32, "uint32_t", "FIXED32" },
	{ PBUF_FIELD_TYPE_FIXED64, "uint64_t", "FIXED64" },
	{ PBUF_FIELD_TYPE_SFIXED32, "int32_t", "SFIXED32" },
	{ PBUF_FIELD_TYPE_SFIXED64, "int64_t", "SFIXED64" },
	{ PBUF_FIELD_TYPE_BOOL, "bool", "BOOL" },
	{ PBUF_FIELD_TYPE_FLOAT, "float", "FLOAT" },
	{ PBUF_FIELD_TYPE_DOUBLE, "double", "DOUBLE" },
	{ PBUF_FIELD_TYPE_STRING, "char *", "STRING" },
	{ PBUF_FIELD_TYPE_BYTES, "void *", "BYTES" },

	{ PBUF_FIELD_TYPE_END, "void" , "END" },
};

int
main(int argc __unused, char **argv)
{
	char *name, *tmp;
	FILE *src;
	int i;

	tmp = strdup(argv[1]);
	name = strsep(&tmp, ".");
	basename = strdup(name);
	free(name);

	uppername = strdup(basename);
	for (i = 0; uppername[i] != '\0'; i++) {
		uppername[i] = toupper(uppername[i]);
	}

	pbufc_start_code();

	src = fopen(argv[1], "r");
	parse_pbuf(src);

	pbufc_finish_code();
}

#define	HEADER(...)	fprintf(header, __VA_ARGS__)
#define	CODE(...)	fprintf(code, __VA_ARGS__)

static void
pbufc_start_code()
{
	char	filename[1024];

	sprintf(filename, "%s_pbuf.h", basename);
	header = fopen(filename, "w");

	HEADER("#ifndef\t_%s_PBUF_H_\n", uppername);
	HEADER("#define\t_%s_PBUF_H_\n", uppername);
	HEADER("\n");
	HEADER("struct sbuf;\n");
	HEADER("\n");

	sprintf(filename, "%s_pbuf.c", basename);
	code = fopen(filename, "w");

	CODE("#include <sys/types.h>\n");
	CODE("#include <sys/sbuf.h>\n");
	CODE("#include <errno.h>\n");
	CODE("#include <stdbool.h>\n");
	CODE("#include <stddef.h>\n");
	CODE("#include <stdlib.h>\n");
	CODE("#include <string.h>\n");
	CODE("\n");
	CODE("#include <pbuf_message.h>\n");
	CODE("\n");
	CODE("#include \"%s_pbuf.h\"\n", basename);
	CODE("\n");
}

static void
pbufc_finish_code()
{

	HEADER("#endif\t/* _%s_PBUF_H_ */\n", uppername);
	fclose(header);

	fclose(code);
}

static inline struct type_mapping *
typemap_for_field(struct field *field)
{
	struct type_mapping *typemap;

	for (typemap = type_mapping; typemap->type != PBUF_FIELD_TYPE_END;
	    typemap++) {
		if (typemap->type == field->type) {
			break;
		}
	}

	return (typemap);
}

void
pbufc_declare_message(const char *name)
{
	struct field *field;
	struct type_mapping *typemap;

	HEADER("struct %s_message;\n", name);
	HEADER("\n");
	HEADER("struct %s_message *%s_message_alloc(void);\n", name, name);
	HEADER("void\t%s_message_free(struct %s_message **msg);\n", name,
	    name);
	HEADER("\n");
	HEADER("int\t%s_message_encode_bytes(struct %s_message *, void *, "
	    "size_t *);\n", name, name);
	HEADER("int\t%s_message_encode_sbuf(struct %s_message *, "
	    "struct sbuf *);\n", name, name);
	HEADER("int\t%s_message_decode_bytes(struct %s_message *, void *, "
	    "size_t *);\n", name, name);
	HEADER("int\t%s_message_decode_sbuf(struct %s_message *, "
	    "struct sbuf *);\n", name, name);
	HEADER("\n");

	STAILQ_FOREACH(field, &fieldlist, fields) {
		typemap = typemap_for_field(field);

		if (field->type == PBUF_FIELD_TYPE_BYTES) {
			HEADER("int %s_message_set_%s(struct %s_message *, "
			    "%s, size_t);\n", name, field->name, name,
			    typemap->ctype);
		} else {
			HEADER("int %s_message_set_%s(struct %s_message *, "
			    "%s);\n", name, field->name, name, typemap->ctype);
		}
		HEADER("%s %s_message_get_%s(struct %s_message *);\n",
		    typemap->ctype, name, field->name, name);
		if (field->type == PBUF_FIELD_TYPE_BYTES ||
		    field->type == PBUF_FIELD_TYPE_STRING) {
			HEADER("size_t %s_message_get_%s_length("
			    "struct %s_message *);\n", name, field->name,
			    name);
		}
	}

	HEADER("\n");

	CODE("struct %s_message {\n", name);
	STAILQ_FOREACH(field, &fieldlist, fields) {
		typemap = typemap_for_field(field);

		if (field->type == PBUF_FIELD_TYPE_STRING ||
		    field->type == PBUF_FIELD_TYPE_BYTES) {
			CODE("\tsize_t field_%s_length;\n", field->name);
		}
		CODE("\t%s field_%s;\n", typemap->ctype, field->name);
	}
	CODE("};\n");

	CODE("\n");

	CODE("struct message_def %s_def[] = {\n", name);
	STAILQ_FOREACH(field, &fieldlist, fields) {
		typemap = typemap_for_field(field);

		if (field->type == PBUF_FIELD_TYPE_STRING ||
		    field->type == PBUF_FIELD_TYPE_BYTES) {
			CODE("\tMESSAGE_DEF_%s(%s_message, \"%s\", %ju, field_%s_length, %#x),\n",
			    typemap->dtype, name, field->name, field->id, field->name,
			    field->flags);
		} else {
			CODE("\tMESSAGE_DEF_%s(%s_message, \"%s\", %ju, field_%s, %#x),\n",
			    typemap->dtype, name, field->name, field->id, field->name,
			    field->flags);
		}
	}
	CODE("\tMESSAGE_DEF_END\n");
	CODE("};\n");

	CODE("\n");

	CODE("struct %s_message *\n", name);
	CODE("%s_message_alloc(void)\n", name);
	CODE("{\n");
	CODE("\tstruct %s_message *msg;\n", name);
	CODE("\n");
	CODE("\tmsg = malloc(sizeof(struct %s_message));\n", name);
	CODE("\tif (msg == NULL) {\n");
	CODE("\t\treturn (NULL);\n");
	CODE("\t}\n");
	CODE("\n");
	CODE("\tmemset(msg, 0, sizeof(struct %s_message));\n", name);
	CODE("\treturn (msg);\n");
	CODE("}\n");

	CODE("\n");

	CODE("void\n");
	CODE("%s_message_free(struct %s_message **msgp)\n", name, name);
	CODE("{\n");
	CODE("\tstruct %s_message *msg = *msgp;\n", name);
	CODE("\n");
	STAILQ_FOREACH(field, &fieldlist, fields) {
		if (field->type == PBUF_FIELD_TYPE_STRING ||
		    field->type == PBUF_FIELD_TYPE_BYTES) {
			CODE("\tif (msg->field_%s != NULL) {\n", field->name);
			CODE("\t\tfree(msg->field_%s);\n", field->name);
			CODE("\t};\n");
		}
	}
	CODE("\tfree(msg);\n");
	CODE("\t*msgp = NULL;\n");
	CODE("}\n");

	CODE("\n");

	CODE("int\n");
	CODE("%s_message_encode_bytes(struct %s_message *msg, void *buffer,\n",
	    name, name);
	CODE("    size_t *buflen)\n");
	CODE("{\n");
	CODE("\n");
	CODE("\treturn (pbuf_message_encode_bytes(msg, %s_def, buffer, "
	    "buflen));\n", name);
	CODE("}\n");

	CODE("\n");

	CODE("int\n");
	CODE("%s_message_encode_sbuf(struct %s_message *msg, "
	    "struct sbuf *buf)\n", name, name);
	CODE("{\n");
	CODE("\n");
	CODE("\treturn (pbuf_message_encode_sbuf(msg, %s_def, buf));\n", name);
	CODE("}\n");

	CODE("int\n");
	CODE("%s_message_decode_bytes(struct %s_message *msg, void *buffer,\n",
	    name, name);
	CODE("    size_t *buflen)\n");
	CODE("{\n");
	CODE("\n");
	CODE("\treturn (pbuf_message_decode_bytes(msg, %s_def, buffer, "
	    "buflen));\n", name);
	CODE("}\n");

	CODE("\n");

	CODE("int\n");
	CODE("%s_message_decode_sbuf(struct %s_message *msg, "
	    "struct sbuf *buf)\n", name, name);
	CODE("{\n");
	CODE("\n");
	CODE("\treturn (pbuf_message_decode_sbuf(msg, %s_def, buf));\n", name);
	CODE("}\n");

	STAILQ_FOREACH(field, &fieldlist, fields) {
		typemap = typemap_for_field(field);

		CODE("\n");

		if (field->type == PBUF_FIELD_TYPE_STRING) {
			CODE("int\n");
			CODE("%s_message_set_%s(struct %s_message *msg, "
			    "%s value)\n",
			    name, field->name, name, typemap->ctype);
			CODE("{\n");
			CODE("\tchar *tmp;\n");
			CODE("\n");
			CODE("\ttmp = strdup(value);\n");
			CODE("\tif (tmp == NULL) {\n");
			CODE("\t\treturn (errno);\n");
			CODE("\t}\n");
			CODE("\n");
			CODE("\tif (msg->field_%s != NULL) {\n", field->name);
			CODE("\t\tfree(msg->field_%s);\n", field->name);
			CODE("\t}\n");
			CODE("\n");
			CODE("\tmsg->field_%s = tmp;\n", field->name);
			CODE("\tmsg->field_%s_length = strlen(tmp);\n",
			    field->name);
			CODE("\treturn (0);\n");
			CODE("}\n");
		} else if (field->type == PBUF_FIELD_TYPE_BYTES) {
			CODE("int\n");
			CODE("%s_message_set_%s(struct %s_message *msg, "
			    "%s value, size_t length)\n",
			    name, field->name, name, typemap->ctype);
			CODE("{\n");
			CODE("\tvoid *tmp;\n");
			CODE("\n");
			CODE("\ttmp = malloc(length);\n");
			CODE("\tif (tmp == NULL) {\n");
			CODE("\t\treturn (errno);\n");
			CODE("\t}\n");
			CODE("\tmemcpy(tmp, value, length);\n");
			CODE("\n");
			CODE("\tif (msg->field_%s != NULL) {\n", field->name);
			CODE("\t\tfree(msg->field_%s);\n", field->name);
			CODE("\t}\n");
			CODE("\n");
			CODE("\tmsg->field_%s = tmp;\n", field->name);
			CODE("\tmsg->field_%s_length = length;\n",
			    field->name);
			CODE("\treturn (0);\n");
			CODE("}\n");
		} else {
			CODE("int\n");
			CODE("%s_message_set_%s(struct %s_message *msg, "
			    "%s value)\n",
			    name, field->name, name, typemap->ctype);
			CODE("{\n");
			CODE("\n");
			CODE("\tmsg->field_%s = value;\n", field->name);
			CODE("\treturn (0);\n");
			CODE("}\n");
		}

		CODE("\n");
		CODE("%s\n", typemap->ctype);
		CODE("%s_message_get_%s(struct %s_message *msg)\n",
		    name, field->name, name);
		CODE("{\n");
		CODE("\n");
		CODE("\treturn (msg->field_%s);\n", field->name);
		CODE("}\n");

		if (field->type == PBUF_FIELD_TYPE_STRING ||
		    field->type == PBUF_FIELD_TYPE_BYTES) {
			CODE("\n");
			CODE("size_t\n");
			CODE("%s_message_get_%s_length"
			    "(struct %s_message *msg)\n",
			    name, field->name, name);
			CODE("{\n");
			CODE("\n");
			CODE("\treturn (msg->field_%s_length);\n", field->name);
			CODE("}\n");
		}
	}

	CODE("\n");

	while (!STAILQ_EMPTY(&fieldlist)) {
		field = STAILQ_FIRST(&fieldlist);
		STAILQ_REMOVE_HEAD(&fieldlist, fields);
		free(field->name);
		free(field);
	}
}

void
pbufc_add_field(pbuf_field_type_t type, const char *name, uint64_t id,
    uint32_t flags)
{
	struct field *field;

	field = malloc(sizeof(struct field));

	field->type = type;
	field->name = strdup(name);
	field->id = id;
	field->flags = flags;

	STAILQ_INSERT_TAIL(&fieldlist, field, fields);
}