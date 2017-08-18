%{
#include <stdint.h>
#include <stdio.h>

#include <pbuf_message.h>

#include "pbufc.h"

int	yylex(void);

void	yyerror(const char *);
int	yywrap(void);

extern FILE *	yyin;

void
yyerror(const char *str)
{

	fprintf(stderr, "error: %s\n", str);
}

int
yywrap(void)
{
	return (1);
}

%}

%union {
	uint64_t		integer;
	char *			string;
	pbuf_field_type_t	type;
}

%token SYNTAX PACKAGE MESSAGE
%token REQUIRED
%token INTEGER
%token IDENTIFIER QUOTESTRING TYPE
%token <integer> INTEGER
%token <string> IDENTIFIER
%token <string> QUOTESTRING
%token <type> TYPE

%type <string> full_identifier

%%

proto:
	syntax definitions
	;

syntax:
	SYNTAX '=' QUOTESTRING ';'
	{
		printf("Syntax statement %s\n", $3);
	}
	;

definitions:
	/* empty */
	|
	definitions definition

definition:
	package
	|
	message
	|
	';'
	;

full_identifier:
	IDENTIFIER
	{
		$$ = strdup($1);
		free($1);
	}
	|
	full_identifier '.' IDENTIFIER
	{
		$$ = malloc(strlen($1) + strlen($3) + 2);
		sprintf($$, "%s.%s", $1, $3);
		free($1);
		free($3);
	}
	;

package:
	PACKAGE full_identifier ';'
	{
		printf("package: %s\n", $2);
		free($2);
	}
	;

message:
	MESSAGE IDENTIFIER '{' fields '}'
	{
		pbufc_declare_message($2);
		free($2);
	}
	;

fields:
	/* empty */
	|
	fields field
	;

field:
	TYPE IDENTIFIER '=' INTEGER ';'
	{
		pbufc_add_field($1, $2, $4, 0);
		free($2);
	}
	|
	REQUIRED TYPE IDENTIFIER '=' INTEGER ';'
	{
		pbufc_add_field($2, $3, $5, MDF_FIELD_REQUIRED);
		free($3);
	}
	;
%%

int
parse_pbuf(FILE *src)
{

	yyin = src;
	yyparse();
	return (0);
}
