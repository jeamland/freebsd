#ifndef _PBUF_CODEC_H_
#define _PBUF_CODEC_H_

ssize_t	pbuf_decode_varint(const char *buf, size_t buflen, uint64_t *value);
int	pbuf_encode_varint(uint64_t value, struct sbuf *buf);

ssize_t	pbuf_decode_fixed64(const char *buf, size_t buflen, uint64_t *value);
int	pbuf_encode_fixed64(uint64_t value, struct sbuf *buf);

ssize_t	pbuf_decode_fixed32(const char *buf, size_t buflen, uint32_t *value);
int	pbuf_encode_fixed32(uint32_t value, struct sbuf *buf);

#endif /* _PBUF_CODEC_H_ */
