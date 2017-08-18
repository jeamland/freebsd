%{
#include <stdint.h>
#include <string.h>

#include <pbuf_message.h>

#include "y.tab.h"

static char stringbuf[128];
static char *sbp;

static char quote;
%}

%x STRING

%%
syntax			return SYNTAX;
package			return PACKAGE;
message			return MESSAGE;

required		return REQUIRED;

double			{ yylval.type = PBUF_FIELD_TYPE_DOUBLE; return TYPE; }
float			{ yylval.type = PBUF_FIELD_TYPE_FLOAT; return TYPE; }
int32			{ yylval.type = PBUF_FIELD_TYPE_INT32; return TYPE; }
int64			{ yylval.type = PBUF_FIELD_TYPE_INT64; return TYPE; }
uint32			{ yylval.type = PBUF_FIELD_TYPE_UINT32; return TYPE; }
uint64			{ yylval.type = PBUF_FIELD_TYPE_UINT64; return TYPE; }
sint32			{ yylval.type = PBUF_FIELD_TYPE_SINT32; return TYPE; }
sint64			{ yylval.type = PBUF_FIELD_TYPE_SINT64; return TYPE; }
fixed32			{ yylval.type = PBUF_FIELD_TYPE_FIXED32; return TYPE; }
sfixed32		{ yylval.type = PBUF_FIELD_TYPE_SFIXED32; return TYPE; }
fixed64			{ yylval.type = PBUF_FIELD_TYPE_FIXED64; return TYPE; }
sfixed64		{ yylval.type = PBUF_FIELD_TYPE_SFIXED64; return TYPE; }
bool			{ yylval.type = PBUF_FIELD_TYPE_BOOL; return TYPE; }
string			{ yylval.type = PBUF_FIELD_TYPE_STRING; return TYPE; }
bytes			{ yylval.type = PBUF_FIELD_TYPE_BYTES; return TYPE; }

\"			{ BEGIN STRING; quote = '"'; sbp = stringbuf; }
\'			{ BEGIN STRING; quote = '\''; sbp = stringbuf; }
<STRING>\\\"		{ *sbp++ = '"'; }
<STRING>\\\'		{ *sbp++ = '\''; }
<STRING>[\"\']		{
	if (quote == yytext[0]) {
		BEGIN INITIAL;
		yylval.string = stringbuf;
		return QUOTESTRING;
	} else {
		return (int)yytext[0];
	}
}
<STRING>.		{ *sbp++ = *yytext; }

[a-zA-Z][a-zA-Z0-9_]*	{ yylval.string = strdup(yytext); return IDENTIFIER; }
[1-9][0-9]*		{
	yylval.integer = strtoll(yytext, NULL, 10);
	return INTEGER;
}
0[0-9]*			{
	yylval.integer = strtoll(yytext, NULL, 8);
	return INTEGER;
}
0[xX][a-fA-F0-9]+	{
	yylval.integer = strtoll(yytext, NULL, 16);
	return INTEGER;
}

\n			/* ignore EOL */
[ \t]+			/* ignore whitespace */

.			return (int)yytext[0];
%%
