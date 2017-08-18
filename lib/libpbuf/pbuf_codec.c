#include <sys/types.h>
#include <sys/endian.h>
#include <sys/sbuf.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "pbuf_codec.h"

ssize_t
pbuf_decode_varint(const char *buf, size_t buflen, uint64_t *value)
{
	size_t		i = 0;
	uint64_t	tmp = 0;

	for (i = 0;; i++) {
		if (i >= buflen) {
			errno = EINVAL;
			return (-1);
		}
		tmp += (uint64_t)(buf[i] & 0x7f) << (7 * i);
		if ((buf[i] & 0x80) == 0) {
			i++;
			break;
		}
	}

	*value = tmp;
	return ((ssize_t)i);
}

int
pbuf_encode_varint(uint64_t value, struct sbuf *buf)
{
	u_char	encoded;

	do {
		encoded = value & 0x7f;
		value >>= 7;
		if (value != 0) {
			encoded |= 0x80;
		}
		if (sbuf_putc(buf, encoded) != 0) {
			return (-1);
		}
	} while (value != 0);

	return (0);
}

ssize_t
pbuf_decode_fixed64(const char *buf, size_t buflen, uint64_t *value)
{
	uint64_t	tmp;
	if (buflen < sizeof(uint64_t)) {
		return (-1);
	}
	memcpy(&tmp, (const void *)buf, sizeof(uint64_t));
	*value = le64toh(tmp);
	return (sizeof(uint64_t));
}

int
pbuf_encode_fixed64(uint64_t value, struct sbuf *buf)
{
	uint64_t	tmp;
	tmp = htole64(value);
	if (sbuf_bcat(buf, (const void *)&tmp, sizeof(uint64_t)) != 0) {
		return (-1);
	}
	return (0);
}

ssize_t
pbuf_decode_fixed32(const char *buf, size_t buflen, uint32_t *value)
{
	uint32_t	tmp;
	if (buflen < sizeof(uint32_t)) {
		return (-1);
	}
	memcpy(&tmp, (const void *)buf, sizeof(uint32_t));
	*value = le32toh(tmp);
	return (sizeof(uint32_t));
}

int
pbuf_encode_fixed32(uint32_t value, struct sbuf *buf)
{
	uint32_t	tmp;
	tmp = htole32(value);
	if (sbuf_bcat(buf, (const void *)&tmp, sizeof(uint32_t)) != 0) {
		return (-1);
	}
	return (0);
}
