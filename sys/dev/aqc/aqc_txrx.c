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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/endian.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/ethernet.h>
#include <net/iflib.h>

#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_rss.h"

#include "aqc.h"
#include "aqc_hw.h"
#include "aqc_reg.h"

#define	AQC_DESC_TX_PAY_LEN_MASK	0x3ffff
#define	AQC_DESC_TX_PAY_LEN_SHIFT	46
#define	AQC_DESC_TX_CT_EN_MASK		0x1
#define	AQC_DESC_TX_CT_EN_SHIFT		45
#define	AQC_DESC_TX_CT_IDX_MASK		0x1
#define	AQC_DESC_TX_CT_IDX_SHIFT	44
#define	AQC_DESC_TX_CMD_SHIFT		22
#define	AQC_DESC_TX_EOP_MASK		0x1
#define	AQC_DESC_TX_EOP_SHIFT		21
#define	AQC_DESC_TX_DD_MASK		0x1
#define	AQC_DESC_TX_DD_SHIFT		20
#define	AQC_DESC_TX_BUF_LEN_MASK	0xffff
#define	AQC_DESC_TX_BUF_LEN_SHIFT	4
#define	AQC_DESC_TX_DES_TYP_MASK	0x3
#define	AQC_DESC_TX_DES_TYP_SHIFT	0

#define	AQC_DESC_TX_CTX_MSS_LEN_MASK	0x3f
#define	AQC_DESC_TX_CTX_MSS_LEN_SHIFT	48
#define	AQC_DESC_TX_CTX_L4_LEN_MASK	0xff
#define	AQC_DESC_TX_CTX_L4_LEN_SHIFT	40
#define	AQC_DESC_TX_CTX_L3_LEN_MASK	0x1ff
#define	AQC_DESC_TX_CTX_L3_LEN_SHIFT	31
#define	AQC_DESC_TX_CTX_L2_LEN_MASK	0x7f
#define	AQC_DESC_TX_CTX_L2_LEN_SHIFT	24
#define	AQC_DESC_TX_CTX_CT_CMD_MASK	0xf
#define	AQC_DESC_TX_CTX_CT_CMD_SHIFT	20
#define	AQC_DESC_TX_CTX_VLAN_TAG_MASK	0xffff
#define	AQC_DESC_TX_CTX_VLAN_TAG_SHIFT	4
#define	AQC_DESC_TX_CTX_CT_IDX_MASK	0x1
#define	AQC_DESC_TX_CTX_CT_IDX_SHIFT	3
#define	AQC_DESC_TX_CTX_DES_TYP_MASK	0x3
#define	AQC_DESC_TX_CTX_DES_TYP_SHIFT	0

#define	AQC_DESC_TX_DES_TYP_DESCRIPTOR	0x01
#define	AQC_DESC_TX_DES_TYP_CONTEXT	0x02

#define	AQC_TX_CMD_VLAN_INSERT		0x01
#define	AQC_TX_CMD_MAC_FCS_INSERT	0x02
#define	AQC_TX_CMD_IPV4_CSUM		0x04
#define	AQC_TX_CMD_TCP_UDP_CSUM		0x08
#define	AQC_TX_CMD_LSO			0x10
#define	AQC_TX_CMD_DESC_WRITEBACK	0x20

#define	AQC_TX_DES_TYP_PACKET		0x1
#define	AQC_TX_DES_TYP_CONTEXT		0x2

#define	AQC_TX_CT_CMD_L2_TYPE_802_3	0x0
#define	AQC_TX_CT_CMD_L2_TYPE_SNAP	0x1
#define	AQC_TX_CT_CMD_L3_TYPE_IPV4	0x0
#define	AQC_TX_CT_CMD_L3_TYPE_IPV6	0x2
#define	AQC_TX_CT_CMD_L4_TYPE_UDP	0x0
#define	AQC_TX_CT_CMD_L4_TYPE_TCP	0x4

static inline void
aqc_tx_desc_packet(struct aqc_desc *desc, bus_addr_t data_buf_addr,
    size_t pay_len, int ct_en, int ct_idx, uint8_t tx_cmd, int eop,
    bus_size_t buf_len)
{

	desc->field1 = (uint64_t)data_buf_addr;
	desc->field2 =
	    ((pay_len & AQC_DESC_TX_PAY_LEN_MASK)
	     << AQC_DESC_TX_PAY_LEN_SHIFT) |
	    ((uint64_t)(ct_en & AQC_DESC_TX_CT_EN_MASK) << AQC_DESC_TX_CT_EN_SHIFT) |
	    ((uint64_t)(ct_idx & AQC_DESC_TX_CT_IDX_MASK) << AQC_DESC_TX_CT_IDX_SHIFT) |
	    (tx_cmd << AQC_DESC_TX_CMD_SHIFT) |
	    ((eop & AQC_DESC_TX_EOP_MASK) << AQC_DESC_TX_EOP_SHIFT) |
	    ((buf_len & AQC_DESC_TX_BUF_LEN_MASK)
	     << AQC_DESC_TX_BUF_LEN_SHIFT) |
	    (AQC_DESC_TX_DES_TYP_DESCRIPTOR << AQC_DESC_TX_DES_TYP_SHIFT);
}

static inline bool
aqc_tx_desc_done(struct aqc_desc *desc)
{
	int des_typ;

	des_typ = desc->field2 & (AQC_DESC_TX_DES_TYP_DESCRIPTOR
	    << AQC_DESC_TX_DES_TYP_SHIFT);
	if (des_typ != AQC_DESC_TX_DES_TYP_DESCRIPTOR)
		return (true);
	return (desc->field2 & (AQC_DESC_TX_DD_MASK << AQC_DESC_TX_DD_SHIFT));
}

static inline void __unused
aqc_tx_desc_context(struct aqc_desc *desc, int mss_len, int l4_len, int l3_len,
    int l2_len, int ct_cmd, uint16_t vlan_tag, int ct_idx)
{

	desc->field1 = 0;
	desc->field2 =
	    ((uint64_t)(mss_len & AQC_DESC_TX_CTX_MSS_LEN_MASK)
	     << AQC_DESC_TX_CTX_MSS_LEN_SHIFT) |
	    ((uint64_t)(l4_len * AQC_DESC_TX_CTX_L4_LEN_MASK)
	     << AQC_DESC_TX_CTX_L4_LEN_SHIFT) |
	    ((uint64_t)(l3_len * AQC_DESC_TX_CTX_L3_LEN_MASK)
	     << AQC_DESC_TX_CTX_L3_LEN_SHIFT) |
	    ((uint64_t)(l2_len * AQC_DESC_TX_CTX_L2_LEN_MASK)
	     << AQC_DESC_TX_CTX_L2_LEN_SHIFT) |
	    ((ct_cmd * AQC_DESC_TX_CTX_CT_CMD_MASK)
	     << AQC_DESC_TX_CTX_CT_CMD_SHIFT) |
	    (vlan_tag << AQC_DESC_TX_CTX_VLAN_TAG_SHIFT) |
	    ((ct_idx & AQC_DESC_TX_CTX_CT_IDX_MASK)
	     << AQC_DESC_TX_CTX_CT_IDX_SHIFT) |
	    (AQC_DESC_TX_DES_TYP_CONTEXT << AQC_DESC_TX_CTX_DES_TYP_SHIFT);
}

#define	AQC_RX_RSS_HASH_SHIFT		0x20
#define	AQC_RX_RSS_HASH_MASK		0xffffffff
#define	AQC_RX_HDR_LEN_SHIFT		0x16
#define	AQC_RX_HDR_LEN_MASK		0x3ff
#define	AQC_RX_SPH_MASK			0x00200000
#define	AQC_RX_RX_CNTL_SHIFT		0x13
#define	AQC_RX_RX_CNTL_MASK		0x3
#define	AQC_RX_RDM_ERR_MASK		0x00001000
#define	AQC_RX_PKT_TYPE_SHIFT		0x04
#define	AQC_RX_PKT_TYPE_MASK		0xff
#define	AQC_RX_RSS_TYPE_SHIFT		0x00
#define	AQC_RX_RSS_TYPE_MASK		0xf
#define	AQC_RX_VLAN_TAG_SHIFT		0x30
#define	AQC_RX_VLAN_TAG_MASK		0xffff
#define	AQC_RX_NEXT_DESP_SHIFT		0x20
#define	AQC_RX_NEXT_DESP_MASK		0xffff
#define	AQC_RX_PKT_LEN_SHIFT		0x10
#define	AQC_RX_PKT_LEN_MASK		0xffff
#define	AQC_RX_RSC_CNT_SHIFT		0x0c
#define	AQC_RX_RSC_CNT_MASK		0xf
#define	AQC_RX_RX_E_STAT_SHIFT		0x06
#define	AQC_RX_RX_E_STAT_MASK		0x3f
#define	AQC_RX_RX_STAT_SHIFT		0x02
#define	AQC_RX_RX_STAT_MASK		0xf
#define	AQC_RX_EOP_MASK			0x00000002
#define	AQC_RX_DD_MASK			0x00000001

struct aqc_rx_info {
	uint32_t	rss_hash;
	size_t		hdr_len;
	bool		sph;
	uint8_t		rx_cntl;
	bool		rdm_err;
	uint8_t		pkt_type;
	uint8_t		rss_type;
	uint16_t	vlan_tag;
	uint16_t	next_desp;
	size_t		pkt_len;
	uint8_t		rsc_cnt;
	uint8_t		rx_e_stat;
	uint8_t		rx_stat;
	bool		eop;
};

static inline void
aqc_rx_desc_init(struct aqc_desc *desc, bus_addr_t data_buf_addr)
{

	desc->field1 = (uint64_t)data_buf_addr;
	desc->field2 = 0;
}

static inline bool
aqc_rx_desc_done(struct aqc_desc *desc)
{

	return (desc->field2 & AQC_RX_DD_MASK);
}


static inline void
aqc_rx_desc_read(struct aqc_desc *desc, struct aqc_rx_info *ri)
{

#define	_BITFIELD(src, shift, mask)	(((src) >> (shift)) & (mask))
#define	_BOOL(src, mask)		((src) & (mask))
	ri->rss_hash = _BITFIELD(desc->field1, AQC_RX_RSS_HASH_SHIFT,
	    AQC_RX_RSS_HASH_MASK);
	ri->hdr_len = _BITFIELD(desc->field1, AQC_RX_HDR_LEN_SHIFT,
	    AQC_RX_HDR_LEN_MASK);
	ri->sph = _BOOL(desc->field1, AQC_RX_SPH_MASK);
	ri->rx_cntl = _BITFIELD(desc->field1, AQC_RX_RX_CNTL_SHIFT,
	    AQC_RX_RX_CNTL_MASK);
	ri->rdm_err = _BOOL(desc->field1, AQC_RX_RDM_ERR_MASK);
	ri->pkt_type = _BITFIELD(desc->field1, AQC_RX_PKT_TYPE_SHIFT,
	    AQC_RX_PKT_TYPE_MASK);
	ri->rss_type = _BITFIELD(desc->field1, AQC_RX_RSS_TYPE_SHIFT,
	    AQC_RX_RSS_TYPE_MASK);

	ri->vlan_tag = _BITFIELD(desc->field2, AQC_RX_VLAN_TAG_SHIFT,
	    AQC_RX_VLAN_TAG_MASK);
	ri->next_desp = _BITFIELD(desc->field2, AQC_RX_NEXT_DESP_SHIFT,
	    AQC_RX_NEXT_DESP_MASK);
	ri->pkt_len = _BITFIELD(desc->field2, AQC_RX_PKT_LEN_SHIFT,
	    AQC_RX_PKT_LEN_MASK);
	ri->rsc_cnt = _BITFIELD(desc->field2, AQC_RX_RSC_CNT_SHIFT,
	    AQC_RX_RSC_CNT_MASK);
	ri->rx_e_stat = _BITFIELD(desc->field2, AQC_RX_RX_E_STAT_SHIFT,
	    AQC_RX_RX_E_STAT_MASK);
	ri->rx_e_stat = _BITFIELD(desc->field2, AQC_RX_RX_STAT_SHIFT,
	    AQC_RX_RX_STAT_MASK);
	ri->eop = _BOOL(desc->field2, AQC_RX_EOP_MASK);
#undef	_BITFIELD
#undef	_BOOL
}

/* iflib txrx interface prototypes */
static int	aqc_isc_txd_encap(void *arg, if_pkt_info_t pi);
static void	aqc_isc_txd_flush(void *arg, uint16_t txqid, qidx_t pidx);
static int	aqc_isc_txd_credits_update(void *arg, uint16_t txqid,
		    bool clear);
static void	aqc_isc_rxd_refill(void *arg, if_rxd_update_t iru);
static void	aqc_isc_rxd_flush(void *arg, uint16_t rxqid,
		    uint8_t flid __unused, qidx_t pidx);
static int	aqc_isc_rxd_available(void *arg, uint16_t rxqid, qidx_t idx,
		    qidx_t budget);
static int	aqc_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri);

struct if_txrx aqc_txrx = {
	.ift_txd_encap = aqc_isc_txd_encap,
	.ift_txd_flush = aqc_isc_txd_flush,
	.ift_txd_credits_update = aqc_isc_txd_credits_update,
	.ift_rxd_available = aqc_isc_rxd_available,
	.ift_rxd_pkt_get = aqc_isc_rxd_pkt_get,
	.ift_rxd_refill = aqc_isc_rxd_refill,
	.ift_rxd_flush = aqc_isc_rxd_flush,
	.ift_legacy_intr = aqc_intr
};

static int
aqc_isc_txd_encap(void *arg, if_pkt_info_t pi)
{
	struct aqc_softc *softc;
	struct aqc_ring *ring;
	bus_dma_segment_t *segs;
	qidx_t i, pidx;
	uint32_t pay_len, eop;

	softc = arg;
	ring = &softc->tx_ring[pi->ipi_qsidx];
	segs = pi->ipi_segs;
	pidx = pi->ipi_pidx;

	pay_len = pi->ipi_len;
	eop = 0;

	for (i = 0; i < pi->ipi_nsegs; i++) {
		if (i == pi->ipi_nsegs - 1)
			eop = 1;

		aqc_tx_desc_packet(&ring->descriptors[pidx], segs[i].ds_addr,
		    pay_len, 0, 0, AQC_TX_CMD_DESC_WRITEBACK, eop,
		    segs[i].ds_len);

		if (i == 0)
			pay_len = 0;

		pidx++;
		if (pidx >= ring->ndesc)
			pidx = 0;
	}

	pi->ipi_new_pidx = pidx;

	return (0);
}

static void
aqc_isc_txd_flush(void *arg, uint16_t txqid, qidx_t pidx)
{
	struct aqc_softc *softc;

	softc = arg;
	aqc_hw_write(softc, AQC_REG_TX_DMA_DESCRIPTOR_TAIL_IDX(txqid), pidx);
}

static int
aqc_isc_txd_credits_update(void *arg, uint16_t txqid, bool clear)
{
	struct aqc_softc *softc;
	struct aqc_ring *ring;
	qidx_t pidx;
	uint32_t head, tail;
	int avail;
	uint64_t counter;

	softc = arg;
	ring = &softc->tx_ring[txqid];
	avail = 0;

	head = aqc_hw_read(softc, AQC_REG_TX_DMA_DESCRIPTOR_HEAD_IDX(txqid));
	head &= AQC_REG_TX_DMA_DESCRIPTOR_HEAD_HEAD_MASK;
	tail = aqc_hw_read(softc, AQC_REG_TX_DMA_DESCRIPTOR_TAIL_IDX(txqid));
	/* XXX: debugging for an odd case where we get a weird head value */
	if (head >= ring->ndesc)
		device_printf(softc->dev, "head=%d tail=%d\n", head, tail);

	if (head == tail) {
		avail = ring->ndesc;
		goto done;
	}

	pidx = tail;
	counter = 0;	/* XXX: workaround for issue mentioned above. */
	do {
		if (aqc_tx_desc_done(&ring->descriptors[pidx])) {
			avail++;
			if (!clear)
				goto done;
		}

		pidx++;
		if (pidx >= ring->ndesc)
			pidx = 0;

		/* XXX: workaround, see above. */
		if (counter++ > 256)
			break;
	} while (pidx != head);

done:
	return (avail);
}

static void
aqc_isc_rxd_refill(void *arg, if_rxd_update_t iru)
{
	struct aqc_softc *softc;
	struct aqc_ring *ring;
	qidx_t i, pidx;
	uint32_t buf_size;

	softc = arg;
	ring = &softc->rx_ring[iru->iru_qsidx];
	pidx = iru->iru_pidx;

	/* Check if we need to change our ring buffer size. */
	buf_size = aqc_hw_read(softc,
	    AQC_REG_RX_DMA_DESCRIPTOR_BUFFER_SIZE(iru->iru_qsidx));
	buf_size &= AQC_RX_DMA_DESCRIPTOR_DATA_SIZE_MASK;
	if ((iru->iru_buf_size >> 10) != buf_size) {
		buf_size = iru->iru_buf_size >> 10;
		aqc_hw_write(softc,
		    AQC_REG_RX_DMA_DESCRIPTOR_BUFFER_SIZE(iru->iru_qsidx),
		    buf_size);
	}

	for (i = 0; i < iru->iru_count; i++) {
		aqc_rx_desc_init(&ring->descriptors[pidx], iru->iru_paddrs[i]);

		pidx++;
		if (pidx >= ring->ndesc)
			pidx = 0;
	}

	return;
}

static void
aqc_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, qidx_t pidx)
{
	struct aqc_softc *softc;

	softc = arg;
	aqc_hw_write(softc, AQC_REG_RX_DMA_DESCRIPTOR_TAIL_IDX(rxqid), pidx);
}

static int
aqc_isc_rxd_available(void *arg, uint16_t rxqid, qidx_t idx, qidx_t budget)
{
	struct aqc_softc *softc;
	struct aqc_ring *ring;
	struct aqc_rx_info ari;
	int cnt, i, iter;

	softc = arg;
	ring = &softc->rx_ring[rxqid];

	if (budget == 1 && aqc_rx_desc_done(&ring->descriptors[idx])) {
		return (1);
	}

	for (iter = cnt = 0, i = idx; iter < ring->ndesc && iter <= budget;) {
		if (!aqc_rx_desc_done(&ring->descriptors[i]))
			break;

		if (++i == ring->ndesc) {
			i = 0;
		}

		aqc_rx_desc_read(&ring->descriptors[i], &ari);

		if (ari.eop)
			cnt++;
		iter++;
	}
	return (cnt);
}

static int
aqc_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri)
{
	struct aqc_softc *softc;
	struct aqc_ring *ring;
	struct aqc_rx_info ari;

	softc = arg;
	ring = &softc->rx_ring[ri->iri_qsidx];

	aqc_rx_desc_read(&ring->descriptors[ri->iri_cidx], &ari);

	ri->iri_len += ari.pkt_len;
	ri->iri_frags[0].irf_flid = 0;
	ri->iri_frags[0].irf_idx = ri->iri_cidx;
	ri->iri_frags[0].irf_len = ari.pkt_len;

	ri->iri_nfrags = 1;

	return (0);
}
