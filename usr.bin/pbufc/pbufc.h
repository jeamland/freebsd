#ifndef	_PBUFC_H_
#define	_PBUFC_H_

int	parse_pbuf(FILE *);

void	pbufc_declare_message(const char *name);
void	pbufc_add_field(pbuf_field_type_t type, const char *name, uint64_t id,
	    uint32_t flags);

#endif	/* _PBUFC_H_ */