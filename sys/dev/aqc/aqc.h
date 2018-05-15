/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2018 Benno Rice <benno@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef	_AQC_H_
#define	_AQC_H_

#define	AQC_XXX_UNIMPLEMENTED_FUNCTION	do {				\
	printf("aqc: unimplemented function: %s@%s:%d\n", __func__, 	\
	    __FILE__, __LINE__);					\
} while (0)

#define	AQC_VENDOR_ID_AQUANTIA	0x1d6a

#define AQC_DEVICE_ID_0001	0x0001
#define AQC_DEVICE_ID_D100	0xD100
#define AQC_DEVICE_ID_D107	0xD107
#define AQC_DEVICE_ID_D108	0xD108
#define AQC_DEVICE_ID_D109	0xD109

#define AQC_DEVICE_ID_AQC100	0x00B1
#define AQC_DEVICE_ID_AQC107	0x07B1
#define AQC_DEVICE_ID_AQC108	0x08B1
#define AQC_DEVICE_ID_AQC109	0x09B1
#define AQC_DEVICE_ID_AQC111	0x11B1
#define AQC_DEVICE_ID_AQC112	0x12B1

#define AQC_DEVICE_ID_AQC100S	0x80B1
#define AQC_DEVICE_ID_AQC107S	0x87B1
#define AQC_DEVICE_ID_AQC108S	0x88B1
#define AQC_DEVICE_ID_AQC109S	0x89B1
#define AQC_DEVICE_ID_AQC111S	0x91B1
#define AQC_DEVICE_ID_AQC112S	0x92B1

#define AQC_DEVICE_ID_AQC111E	0x51B1
#define AQC_DEVICE_ID_AQC112E	0x52B1

#define AQC_TSO_SIZE	UINT16_MAX

#define	AQC_MIN_RXD	16
#define	AQC_MIN_TXD	16

#define	AQC_DEFAULT_RXD	4096
#define	AQC_DEFAULT_TXD	4096

#define	AQC_MAX_RXD	8192
#define	AQC_MAX_TXD	8192

#define	AQC_MIN_FRAME_SIZE	52

enum aqc_media_type {
	AQC_MEDIA_TYPE_UNKNOWN = 0,
	AQC_MEDIA_TYPE_FIBRE,
	AQC_MEDIA_TYPE_TP,
};

#define	AQC_LINK_UNKNOWN	0x00000000
#define	AQC_LINK_100M		0x00000001
#define	AQC_LINK_1G		0x00000002
#define	AQC_LINK_2G5		0x00000004
#define	AQC_LINK_5G		0x00000008
#define	AQC_LINK_10G		0x00000010

#define	AQC_LINK_ALL	(	\
	AQC_LINK_100M	|	\
	AQC_LINK_1G	|	\
	AQC_LINK_2G5	|	\
	AQC_LINK_5G	|	\
	AQC_LINK_10G		\
)

struct aqc_fw_ops;

struct aqc_fw_stats {
	uint32_t uprc;		/* unicast packets received */
	uint32_t mprc;		/* multicast packets received */
	uint32_t bprc;		/* broadcast packets received */
	uint32_t erpt;		/* packet transmit errors */
	uint32_t uptc;		/* unicast packets transmitted */
	uint32_t mptc;		/* multicast packets transmitted */
	uint32_t bptc;		/* broadcast packets transmitted */
	uint32_t erpr;		/* packet receive errors */
	uint32_t mbtc;		/* multicast bytes transmitted */
	uint32_t bbtc;		/* broadcast bytes transmitted */
	uint32_t mbrc;		/* multicast bytes received */
	uint32_t bbrc;		/* broadcast bytes received */
	uint32_t ubrc;		/* unicast bytes received */
	uint32_t ubtc;		/* unicast bytes transmitted */
	uint32_t dpc;		/* rx dropped packet count */
	uint32_t padding;
} __packed;

struct aqc_stats {
	uint64_t uprc;			/* unicast packets received */
	uint64_t mprc;			/* multicast packets received */
	uint64_t bprc;			/* broadcast packets received */
	uint64_t erpt;			/* packet transmit errors */
	uint64_t uptc;			/* unicast packets transmitted */
	uint64_t mptc;			/* multicast packets transmitted */
	uint64_t bptc;			/* broadcast packets transmitted */
	uint64_t erpr;			/* packet receive errors */
	uint64_t mbtc;			/* multicast bytes transmitted */
	uint64_t bbtc;			/* broadcast bytes transmitted */
	uint64_t mbrc;			/* multicast bytes received */
	uint64_t bbrc;			/* broadcast bytes received */
	uint64_t ubrc;			/* unicast bytes received */
	uint64_t ubtc;			/* unicast bytes transmitted */
	uint64_t dpc;			/* rx dropped packet count */
	uint64_t dma_pkts_rx;		/* DMA packets received */
	uint64_t dma_pkts_tx;		/* DMA packets transmitted */
	uint64_t dma_bytes_rx;		/* DMA bytes received */
	uint64_t dma_bytes_tx;		/* DMA bytes transmitted */
};

struct aqc_desc {
	uint64_t	field1;
	uint64_t	field2;
} __packed;

struct aqc_softc {
	device_t		dev;
	if_ctx_t		ctx;
	if_softc_ctx_t		scctx;
	if_shared_ctx_t		sctx;
	struct ifmedia *	media;

	struct aqc_hw_ops *	hw_ops;
	struct aqc_fw_ops *	fw_ops;

	enum aqc_media_type	media_type;
	uint32_t		link_speeds;
	uint32_t		tx_rings;
	uint32_t		rx_rings;
	uint32_t		chip_features;
	uint32_t		mbox_addr;
	uint8_t			mac_addr[ETHER_ADDR_LEN];
	uint64_t		admin_ticks;

	int			mmio_rid;
	struct resource *	mmio_res;
	bus_space_tag_t		mmio_tag;
	bus_space_handle_t	mmio_handle;
	bus_size_t		mmio_size;

	struct aqc_desc *	tx_ring;
	struct aqc_desc *	rx_ring;

	struct aqc_stats	stats;
	struct aqc_fw_stats	fw_stats;
};

extern struct if_txrx aqc_txrx;

int		aqc_intr(void *arg);

#endif /* _AQC_H_ */
