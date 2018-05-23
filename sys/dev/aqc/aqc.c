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

#include <sys/param.h>
#include <sys/malloc.h>
#include <sys/socket.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/endian.h>
#include <sys/sockio.h>
#include <sys/priv.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/if_var.h>
#include <net/if_dl.h>
#include <net/ethernet.h>
#include <net/iflib.h>

#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_rss.h"

#include "ifdi_if.h"

#include "aqc.h"
#include "aqc_hw.h"
#include "aqc_fw.h"
#include "aqc_reg.h"

MALLOC_DEFINE(M_AQC, "aqc", "Aquantia");

char aqc_driver_version[] = "1.0.0";

static pci_vendor_info_t aqc_vendor_info_array[] = {
	PVID(AQC_VENDOR_ID_AQUANTIA, AQC_DEVICE_ID_D107,
	    "Aquantia AQtion 10Gbit Network Adapter"),
	PVID(AQC_VENDOR_ID_AQUANTIA, AQC_DEVICE_ID_AQC107,
	    "Aquantia AQtion 10Gbit Network Adapter"),
	PVID(AQC_VENDOR_ID_AQUANTIA, AQC_DEVICE_ID_AQC107S,
	    "Aquantia AQtion 10Gbit Network Adapter"),

	PVID(AQC_VENDOR_ID_AQUANTIA, AQC_DEVICE_ID_D108,
	    "Aquantia AQtion 5Gbit Network Adapter"),
	PVID(AQC_VENDOR_ID_AQUANTIA, AQC_DEVICE_ID_AQC108,
	    "Aquantia AQtion 5Gbit Network Adapter"),
	PVID(AQC_VENDOR_ID_AQUANTIA, AQC_DEVICE_ID_AQC108S,
	    "Aquantia AQtion 5Gbit Network Adapter"),

	PVID_END
};

/* Device setup, teardown, etc */
static void *	aqc_register(device_t dev);
static int	aqc_if_attach_pre(if_ctx_t ctx);
static int	aqc_if_attach_post(if_ctx_t ctx);
static int	aqc_if_detach(if_ctx_t ctx);
static int	aqc_if_shutdown(if_ctx_t ctx);
static int	aqc_if_suspend(if_ctx_t ctx);
static int	aqc_if_resume(if_ctx_t ctx);

/* Soft queue setup and teardown */
static int	aqc_if_tx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs,
		    uint64_t *paddrs, int ntxqs, int ntxqsets);
static int	aqc_if_rx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs,
		    uint64_t *paddrs, int nrxqs, int nrxqsets);
static void	aqc_if_queues_free(if_ctx_t ctx);

/* Device configuration */
static void	aqc_if_init(if_ctx_t ctx);
static void	aqc_if_stop(if_ctx_t ctx);
static void	aqc_if_multi_set(if_ctx_t ctx);
static int	aqc_if_mtu_set(if_ctx_t ctx, uint32_t mtu);
static void	aqc_if_media_status(if_ctx_t ctx, struct ifmediareq *ifmr);
static int	aqc_if_media_change(if_ctx_t ctx);
static int	aqc_if_promisc_set(if_ctx_t ctx, int flags);
static uint64_t	aqc_if_get_counter(if_ctx_t ctx, ift_counter cnt);
static void	aqc_if_update_admin_status(if_ctx_t ctx);
static void	aqc_if_timer(if_ctx_t ctx, uint16_t qid);
static int	aqc_if_priv_ioctl(if_ctx_t ctx, u_long command, caddr_t data);

/* Interrupt enable / disable */
static void	aqc_if_enable_intr(if_ctx_t ctx);
static void	aqc_if_disable_intr(if_ctx_t ctx);
static int	aqc_if_rx_queue_intr_enable(if_ctx_t ctx, uint16_t rxqid);
static int	aqc_if_tx_queue_intr_enable(if_ctx_t ctx, uint16_t txqid);
static int	aqc_if_msix_intr_assign(if_ctx_t ctx, int msix);

/* VLAN support */
static void	aqc_if_vlan_register(if_ctx_t ctx, uint16_t vtag);
static void	aqc_if_vlan_unregister(if_ctx_t ctx, uint16_t vtag);

/* Informational/diagnostic */
static void	aqc_if_debug(if_ctx_t ctx);
static void	aqc_if_led_func(if_ctx_t ctx, int onoff);

/* Internal interrupt handler */
static int	aqc_handle_rx(void *);

static device_method_t aqc_methods[] = {
	DEVMETHOD(device_register, aqc_register),
	DEVMETHOD(device_probe, iflib_device_probe),
	DEVMETHOD(device_attach, iflib_device_attach),
	DEVMETHOD(device_detach, iflib_device_detach),
	DEVMETHOD(device_shutdown, iflib_device_shutdown),
	DEVMETHOD(device_suspend, iflib_device_suspend),
	DEVMETHOD(device_resume, iflib_device_resume),

	DEVMETHOD_END
};

static driver_t aqc_driver = {
	"aqc", aqc_methods, sizeof(struct aqc_softc),
};

static devclass_t aqc_devclass;
DRIVER_MODULE(aqc, pci, aqc_driver, aqc_devclass, 0, 0);

MODULE_DEPEND(aqc, pci, 1, 1, 1);
MODULE_DEPEND(aqc, ether, 1, 1, 1);
MODULE_DEPEND(aqc, iflib, 1, 1, 1);

IFLIB_PNP_INFO(pci, aqc, aqc_vendor_info_array);

static device_method_t aqc_if_methods[] = {
	/* Device setup, teardown, etc */
	DEVMETHOD(ifdi_attach_pre, aqc_if_attach_pre),
	DEVMETHOD(ifdi_attach_post, aqc_if_attach_post),
	DEVMETHOD(ifdi_detach, aqc_if_detach),
	DEVMETHOD(ifdi_shutdown, aqc_if_shutdown),
	DEVMETHOD(ifdi_suspend, aqc_if_suspend),
	DEVMETHOD(ifdi_resume, aqc_if_resume),

	/* Soft queue setup and teardown */
	DEVMETHOD(ifdi_tx_queues_alloc, aqc_if_tx_queues_alloc),
	DEVMETHOD(ifdi_rx_queues_alloc, aqc_if_rx_queues_alloc),
	DEVMETHOD(ifdi_queues_free, aqc_if_queues_free),

	/* Device configuration */
	DEVMETHOD(ifdi_init, aqc_if_init),
	DEVMETHOD(ifdi_stop, aqc_if_stop),
	DEVMETHOD(ifdi_multi_set, aqc_if_multi_set),
	DEVMETHOD(ifdi_mtu_set, aqc_if_mtu_set),
	DEVMETHOD(ifdi_media_status, aqc_if_media_status),
	DEVMETHOD(ifdi_media_change, aqc_if_media_change),
	DEVMETHOD(ifdi_promisc_set, aqc_if_promisc_set),
	DEVMETHOD(ifdi_get_counter, aqc_if_get_counter),
	DEVMETHOD(ifdi_update_admin_status, aqc_if_update_admin_status),
	DEVMETHOD(ifdi_timer, aqc_if_timer),
	DEVMETHOD(ifdi_priv_ioctl, aqc_if_priv_ioctl),

	/* Interrupt enable / disable */
	DEVMETHOD(ifdi_intr_enable, aqc_if_enable_intr),
	DEVMETHOD(ifdi_intr_disable, aqc_if_disable_intr),
	DEVMETHOD(ifdi_rx_queue_intr_enable, aqc_if_rx_queue_intr_enable),
	DEVMETHOD(ifdi_tx_queue_intr_enable, aqc_if_tx_queue_intr_enable),
	DEVMETHOD(ifdi_msix_intr_assign, aqc_if_msix_intr_assign),

	/* VLAN support */
	DEVMETHOD(ifdi_vlan_register, aqc_if_vlan_register),
	DEVMETHOD(ifdi_vlan_unregister, aqc_if_vlan_unregister),

	/* Informational/diagnostic */
	DEVMETHOD(ifdi_led_func, aqc_if_led_func),
	DEVMETHOD(ifdi_debug, aqc_if_debug),

	DEVMETHOD_END
};

static driver_t aqc_if_driver = {
	"aqc_if", aqc_if_methods, sizeof(struct aqc_softc)
};

static struct if_shared_ctx aqc_sctx_init = {
	.isc_magic = IFLIB_MAGIC,
	.isc_q_align = 128,
	.isc_tx_maxsize = AQC_TSO_SIZE,
	.isc_tx_maxsegsize = PAGE_SIZE,
	.isc_rx_maxsize = MJUM9BYTES,
	.isc_rx_nsegments = 1,
	.isc_rx_maxsegsize = MJUM9BYTES,
	.isc_nfl = 1,
	.isc_nrxqs = 1,
	.isc_ntxqs = 1,
	.isc_admin_intrcnt = 1,
	.isc_vendor_info = aqc_vendor_info_array,
	.isc_driver_version = aqc_driver_version,
	.isc_driver = &aqc_if_driver,
	.isc_flags = IFLIB_NEED_SCRATCH | IFLIB_TSO_INIT_IP |
	    IFLIB_NEED_ZERO_CSUM,

	.isc_nrxd_min = {AQC_MIN_RXD},
	.isc_ntxd_min = {AQC_MIN_TXD},
	.isc_nrxd_max = {AQC_MAX_RXD},
	.isc_ntxd_max = {AQC_MAX_TXD},
	.isc_nrxd_default = {PAGE_SIZE / sizeof(struct aqc_desc)},
	.isc_ntxd_default = {PAGE_SIZE / sizeof(struct aqc_desc)},
};

if_shared_ctx_t aqc_sctx = &aqc_sctx_init;

/*
 * Device Methods
 */
static void *
aqc_register(device_t dev)
{

	return (aqc_sctx);
}

static int
aqc_if_attach_pre(if_ctx_t ctx)
{
	struct aqc_softc *softc;
	if_softc_ctx_t scctx;
	int rc;

	softc = iflib_get_softc(ctx);
	rc = 0;

	softc->ctx = ctx;
	softc->dev = iflib_get_dev(ctx);
	softc->media = iflib_get_media(ctx);
	softc->scctx = iflib_get_softc_ctx(ctx);
	softc->sctx = iflib_get_sctx(ctx);
	scctx = softc->scctx;

	softc->mmio_rid = PCIR_BAR(0);
	softc->mmio_res = bus_alloc_resource_any(softc->dev, SYS_RES_MEMORY,
	    &softc->mmio_rid, RF_ACTIVE|RF_SHAREABLE);
	if (softc->mmio_res == NULL) {
		device_printf(softc->dev,
		    "failed to allocate MMIO resources\n");
		rc = ENXIO;
		goto fail;
	}

	softc->mmio_tag = rman_get_bustag(softc->mmio_res);
	softc->mmio_handle = rman_get_bushandle(softc->mmio_res);
	softc->mmio_size = rman_get_size(softc->mmio_res);

	/* Look up ops and caps. */
	if ((rc = aqc_hw_probe(softc)) != 0) {
		device_printf(softc->dev, "hardware probe failed\n");
		goto fail;
	}
	if ((rc = aqc_fw_probe(softc)) != 0) {
		device_printf(softc->dev, "firmware probe failed\n");
		goto fail;
	}

	softc->admin_ticks = 0;
	
	aqc_fw_get_permanent_mac(softc);
	iflib_set_mac(ctx, softc->mac_addr);

	scctx->isc_tx_csum_flags = 0;
	scctx->isc_capenable = 0;

	scctx->isc_tx_nsegments = 31,
	scctx->isc_tx_tso_segments_max = 31;
	scctx->isc_tx_tso_size_max = AQC_TSO_SIZE;
	scctx->isc_tx_tso_segsize_max = 256 * 1024;
	scctx->isc_vectors = 3;
	scctx->isc_min_frame_size = AQC_MIN_FRAME_SIZE;
	scctx->isc_txrx = &aqc_txrx;

	scctx->isc_txqsizes[0] = sizeof(struct aqc_desc) * scctx->isc_ntxd[0];
	scctx->isc_rxqsizes[0] = sizeof(struct aqc_desc) * scctx->isc_nrxd[0];

	scctx->isc_ntxqsets_max = softc->tx_rings;
	scctx->isc_nrxqsets_max = softc->rx_rings;
	/* Fill in scctx stuff. */
	/* XXX THIS STUFF IS NOT RIGHT YET */
	#if 0

	scctx->isc_tx_csum_flags = (CSUM_IP | CSUM_TCP | CSUM_UDP |
	    CSUM_TCP_IPV6 | CSUM_UDP_IPV6 | CSUM_TSO);
	scctx->isc_capenable =
	    /* These are translated to hwassit bits */
	    IFCAP_TXCSUM | IFCAP_TXCSUM_IPV6 | IFCAP_TSO4 | IFCAP_TSO6 |
	    /* These are checked by iflib */
	    IFCAP_LRO | IFCAP_VLAN_HWFILTER |
	    /* These are part of the iflib mask */
	    IFCAP_RXCSUM | IFCAP_RXCSUM_IPV6 | IFCAP_VLAN_MTU |
	    IFCAP_VLAN_HWTAGGING | IFCAP_VLAN_HWTSO |
	    /* These likely get lost... */
	    IFCAP_VLAN_HWCSUM | IFCAP_JUMBO_MTU;
	
	if (bnxt_wol_supported(softc))
		scctx->isc_capenable |= IFCAP_WOL_MAGIC;
	
	/* Now set up iflib sc */
	scctx->isc_tx_nsegments = 31,
	scctx->isc_tx_tso_segments_max = 31;
	scctx->isc_tx_tso_size_max = BNXT_TSO_SIZE;
	scctx->isc_tx_tso_segsize_max = BNXT_TSO_SIZE;
	scctx->isc_vectors = softc->func.max_cp_rings;
	scctx->isc_min_frame_size = BNXT_MIN_FRAME_SIZE;
	scctx->isc_txrx = &bnxt_txrx;

	if (scctx->isc_nrxd[0] <
	    ((scctx->isc_nrxd[1] * 4) + scctx->isc_nrxd[2]))
		device_printf(softc->dev,
		    "WARNING: nrxd0 (%d) should be at least 4 * nrxd1 (%d) + nrxd2 (%d).  Driver may be unstable\n",
		    scctx->isc_nrxd[0], scctx->isc_nrxd[1], scctx->isc_nrxd[2]);
	if (scctx->isc_ntxd[0] < scctx->isc_ntxd[1] * 2)
		device_printf(softc->dev,
		    "WARNING: ntxd0 (%d) should be at least 2 * ntxd1 (%d).  Driver may be unstable\n",
		    scctx->isc_ntxd[0], scctx->isc_ntxd[1]);
	scctx->isc_txqsizes[0] = sizeof(struct cmpl_base) * scctx->isc_ntxd[0];
	scctx->isc_txqsizes[1] = sizeof(struct tx_bd_short) *
	    scctx->isc_ntxd[1];
	scctx->isc_rxqsizes[0] = sizeof(struct cmpl_base) * scctx->isc_nrxd[0];
	scctx->isc_rxqsizes[1] = sizeof(struct rx_prod_pkt_bd) *
	    scctx->isc_nrxd[1];
	scctx->isc_rxqsizes[2] = sizeof(struct rx_prod_pkt_bd) *
	    scctx->isc_nrxd[2];

	scctx->isc_nrxqsets_max = min(pci_msix_count(softc->dev)-1,
	    softc->fn_qcfg.alloc_completion_rings - 1);
	scctx->isc_nrxqsets_max = min(scctx->isc_nrxqsets_max,
	    softc->fn_qcfg.alloc_rx_rings);
	scctx->isc_nrxqsets_max = min(scctx->isc_nrxqsets_max,
	    softc->fn_qcfg.alloc_vnics);
	scctx->isc_ntxqsets_max = min(softc->fn_qcfg.alloc_tx_rings,
	    softc->fn_qcfg.alloc_completion_rings - scctx->isc_nrxqsets_max - 1);

	scctx->isc_rss_table_size = HW_HASH_INDEX_SIZE;
	scctx->isc_rss_table_mask = scctx->isc_rss_table_size - 1;
	#endif

	/* iflib will map and release this bar */
	scctx->isc_msix_bar = pci_msix_table_bar(softc->dev);
	/* XXX END OF THIS STUFF IS NOT RIGHT YET */

	return (rc);

fail:
	if (softc->mmio_res != NULL)
		bus_release_resource(softc->dev, SYS_RES_MEMORY,
		    softc->mmio_rid, softc->mmio_res);

	return (ENXIO);
}

static int
aqc_if_attach_post(if_ctx_t ctx)
{
	struct aqc_softc *softc;
	if_t ifp;
	uint32_t value;
	int rc, i;

	softc = iflib_get_softc(ctx);
	ifp = iflib_get_ifp(ctx);
	rc = 0;

	aqc_hw_update_stats(softc);

	if (AQC_HW_SUPPORT_SPEED(AQC_LINK_100M))
		ifmedia_add(softc->media, IFM_ETHER | IFM_100_TX | IFM_FDX, 0,
		    NULL);

	if (AQC_HW_SUPPORT_SPEED(AQC_LINK_1G))
		ifmedia_add(softc->media, IFM_ETHER | IFM_1000_T | IFM_FDX, 0,
		    NULL);
	
	if (AQC_HW_SUPPORT_SPEED(AQC_LINK_2G5))
		ifmedia_add(softc->media, IFM_ETHER | IFM_2500_T | IFM_FDX, 0,
		    NULL);

	if (AQC_HW_SUPPORT_SPEED(AQC_LINK_5G))
		ifmedia_add(softc->media, IFM_ETHER | IFM_5000_T | IFM_FDX, 0,
		    NULL);
	if (AQC_HW_SUPPORT_SPEED(AQC_LINK_10G))
		ifmedia_add(softc->media, IFM_ETHER | IFM_10G_T | IFM_FDX, 0,
		    NULL);

	ifmedia_add(softc->media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(softc->media, IFM_ETHER | IFM_AUTO);

	/* XXX TSO flags */

	value = aqc_hw_read(softc, AQC_REG_TX_INTERRUPT_CONTROL);
	value |= AQC_TX_INTERRUPT_CONTROL_DESC_WRB_EN;
	aqc_hw_write(softc, AQC_REG_TX_INTERRUPT_CONTROL, value);

	if (AQC_HW_FEATURE(softc, AQC_HW_FEATURE_TPO2)) {
		aqc_hw_write(softc, AQC_REG_TX_SPARE_CONTROL_DEBUG,
		    0x00010000);
	} else {
		aqc_hw_write(softc, AQC_REG_TX_SPARE_CONTROL_DEBUG,
		    0x00000000);
	}

	value = aqc_hw_read(softc, AQC_REG_TX_DCA_CONTROL_33);
	value &= ~(AQC_TX_DCA_EN + AQC_TX_DCA_MODE_MASK);
	value |= AQC_TX_DCA_MODE_LEGACY;
	aqc_hw_write(softc, AQC_REG_TX_DCA_CONTROL_33, value);

	value = aqc_hw_read(softc, AQC_REG_TX_PACKET_BUFFER_CONTROL_1);
	value |= AQC_TX_PACKET_BUFFER_PAD_INS_EN;
	aqc_hw_write(softc, AQC_REG_TX_PACKET_BUFFER_CONTROL_1, value);

	/* XXX init RX path */
	value = aqc_hw_read(softc, AQC_REG_RX_PACKET_BUFFER_CONTROL_1);
	value &= ~(AQC_RX_PACKET_BUFFER_FC_MODE_MASK <<
	    AQC_RX_PACKET_BUFFER_FC_MODE_SHIFT);
	value |= AQC_RX_PACKET_BUFFER_TC_MODE |
	     (AQC_RX_PACKET_BUFFER_FC_MODE_EN <<
	      AQC_RX_PACKET_BUFFER_FC_MODE_SHIFT);
	aqc_hw_write(softc, AQC_REG_RX_PACKET_BUFFER_CONTROL_1, value);

	value = AQC_RX_RSS_RXQ_EN;
	for (i = 0; i < 8; i++) {
		value |= 3 << AQC_RX_RSS_SEL_SHIFT(i);
	}
	aqc_hw_write(softc, AQC_REG_RX_RSS_CONTROL_1, value);

	for (i = 0; i < AQC_HW_MAX_UNICAST; i++) {
		value = aqc_hw_read(softc, AQC_REG_RX_UNICAST_FILTER_2(i));
		if (i == 0) {
			value |= AQC_RX_UNICAST_FILTER_L2_EN;
		} else {
			value &= ~AQC_RX_UNICAST_FILTER_L2_EN;
		}
		value &= ~AQC_RX_UNICAST_FILTER_ACT_MASK;
		value |= AQC_RX_UNICAST_FILTER_ACT_HOST;
		aqc_hw_write(softc, AQC_REG_RX_UNICAST_FILTER_2(i), value);
	}

	aqc_hw_write(softc, AQC_REG_RX_MULTICAST_FILTER(0), 0);
	value = AQC_RX_MULTICAST_FILTER_ACT_HOST;
	value |= 0xfff;
	aqc_hw_write(softc, AQC_REG_RX_MULTICAST_FILTER(0), value);

	value = (0x88a8 << AQC_RX_VLAN_OUTER_SHIFT) |
	    (0x8100 << AQC_RX_VLAN_INNER_SHIFT);
	aqc_hw_write(softc, AQC_REG_RX_VLAN_CONTROL_2, value);

	value = aqc_hw_read(softc, AQC_REG_RX_VLAN_CONTROL_1);
	value |= AQC_RX_VLAN_PROMISC_MODE;
	aqc_hw_write(softc, AQC_REG_RX_VLAN_CONTROL_1, value);

	value = aqc_hw_read(softc, AQC_REG_RX_INTERRUPT_CONTROL);
	value |= AQC_RX_INTERRUPT_CONTROL_DESC_WRB_EN;
	aqc_hw_write(softc, AQC_REG_RX_INTERRUPT_CONTROL, value);

	if (AQC_HW_FEATURE(softc, AQC_HW_FEATURE_RPF2)) {
		aqc_hw_write(softc, AQC_REG_RX_SPARE_CONTROL_DEBUG,
		    0x000f0000);
	} else {
		aqc_hw_write(softc, AQC_REG_RX_SPARE_CONTROL_DEBUG,
		    0x00000000);
	}

	value = aqc_hw_read(softc, AQC_REG_RX_FILTER_CONTROL_1);
	value &= ~(AQC_RX_FILTER_BC_ACT_MASK);
	value |= AQC_RX_FILTER_BC_ACT_HOST;
	value |= 0xffff << AQC_RX_FILTER_BC_THRESH_SHIFT;
	aqc_hw_write(softc, AQC_REG_RX_FILTER_CONTROL_1, value);

	value = aqc_hw_read(softc, AQC_REG_RX_DCA_CONTROL_33);
	value &= ~(AQC_RX_DCA_EN + AQC_RX_DCA_MODE_MASK);
	value |= AQC_RX_DCA_MODE_LEGACY;
	aqc_hw_write(softc, AQC_REG_RX_DCA_CONTROL_33, value);

	aqc_hw_set_mac(softc);

	aqc_fw_set_link_speed(softc, softc->link_speeds);
	aqc_fw_set_state(softc, AQC_MPI_INIT);

	value = aqc_hw_read(softc, AQC_REG_TX_PKT_SCHED_DESC_RATE_CONTROL);
	value &= ~AQC_TX_PKT_SCHED_DESC_RATE_TA_RST;
	aqc_hw_write(softc, AQC_REG_TX_PKT_SCHED_DESC_RATE_CONTROL, value);
	value &= ~(AQC_TX_PKT_SCHED_DESC_RATE_LIMIT_MASK <<
	    AQC_TX_PKT_SCHED_DESC_RATE_LIMIT_SHIFT);
	value |= 0xa << AQC_TX_PKT_SCHED_DESC_RATE_LIMIT_SHIFT;
	aqc_hw_write(softc, AQC_REG_TX_PKT_SCHED_DESC_RATE_CONTROL, value);

	value = aqc_hw_read(softc, AQC_REG_TX_PKT_SCHED_DESC_VM_CONTROL);
	value &= ~AQC_TX_PKT_SCHED_DESC_VM_ARB_MODE;
	aqc_hw_write(softc, AQC_REG_TX_PKT_SCHED_DESC_VM_CONTROL, value);

	value = aqc_hw_read(softc, AQC_REG_TX_PKT_SCHED_DESC_TC_CONTROL_1);
	value &= ~(AQC_TX_PKT_SCHED_DESC_TC_ARB_MODE_MASK <<
	    AQC_TX_PKT_SCHED_DESC_TC_ARB_MODE_SHIFT);
	value |= AQC_TX_PKT_SCHED_DESC_TC_ARB_MODE_RR <<
	    AQC_TX_PKT_SCHED_DESC_TC_ARB_MODE_SHIFT;
	aqc_hw_write(softc, AQC_REG_TX_PKT_SCHED_DESC_TC_CONTROL_1, value);

	value = aqc_hw_read(softc, AQC_REG_TX_PKT_SCHED_DATA_TC_CONTROL_1);
	value &= ~AQC_TX_PKT_SCHED_DATA_TC_ARB_MODE;
	aqc_hw_write(softc, AQC_REG_TX_PKT_SCHED_DATA_TC_CONTROL_1, value);

	value = (0xfff << AQC_TX_PKT_SCHED_DATA_TC_CREDIT_MAX_SHIFT) |
	    (0x64 << AQC_TX_PKT_SCHED_DATA_TC_WEIGHT_SHIFT);
	aqc_hw_write(softc, AQC_REG_TX_PKT_SCHED_DATA_TC_CONTROL(0), value);

	value = (0x50 << AQC_TX_PKT_SCHED_DESC_TC_CREDIT_MAX_SHIFT) |
	    (0x1e << AQC_TX_PKT_SCHED_DESC_TC_WEIGHT_SHIFT);
	aqc_hw_write(softc, AQC_REG_TX_PKT_SCHED_DESC_TC_CONTROL(0), value);

	value = AQC_TXBUF_MAX << AQC_TX_PACKET_BUFFER_SIZE_SHIFT;
	aqc_hw_write(softc, AQC_REG_TX_PACKET_BUFFER_1(0), value);

	value = ((AQC_TXBUF_MAX * (1024 / 32) * 66) / 100) <<
	    AQC_TX_PACKET_BUFFER_HI_THRESH_SHIFT;
	value |= ((AQC_TXBUF_MAX * (1024 / 32) * 50) / 100) <<
	    AQC_TX_PACKET_BUFFER_LO_THRESH_SHIFT;
	aqc_hw_write(softc, AQC_REG_TX_PACKET_BUFFER_2(0), value);

	/* XXX RX qos */
	value = AQC_RXBUF_MAX << AQC_RX_PACKET_BUFFER_SIZE_SHIFT;
	aqc_hw_write(softc, AQC_REG_TX_PACKET_BUFFER_1(0), value);

	value = ((AQC_RXBUF_MAX * (1024 / 32) * 66) / 100) <<
	    AQC_RX_PACKET_BUFFER_HI_THRESH_SHIFT;
	value |= ((AQC_RXBUF_MAX * (1024 / 32) * 50) / 100) <<
	    AQC_RX_PACKET_BUFFER_LO_THRESH_SHIFT;
	/* XXX flow control configuration */
	value |= AQC_RX_PACKET_BUFFER_XOFF_EN;
	aqc_hw_write(softc, AQC_REG_RX_PACKET_BUFFER_2(0), value);

	aqc_hw_write(softc, AQC_REG_RX_FILTER_TC_USER_PRIORITY, 0);

	/* XXX rss */
	/* XXX rss hash */

	value = aqc_hw_read(softc, AQC_REG_PCIE_CONTROL_6);
	value &= ~(
	    (AQC_PCIE_RDM_MRRS_OVERRIDE_MASK <<
	     AQC_PCIE_RDM_MRRS_OVERRIDE_SHIFT) |
	    (AQC_PCIE_TDM_MRRS_OVERRIDE_MASK <<
	     AQC_PCIE_TDM_MRRS_OVERRIDE_SHIFT)
	);
	value |= (0x4 << AQC_PCIE_RDM_MRRS_OVERRIDE_SHIFT) |
	    (0x4 << AQC_PCIE_TDM_MRRS_OVERRIDE_SHIFT);
	aqc_hw_write(softc, AQC_REG_PCIE_CONTROL_6, value);

	aqc_hw_write(softc, AQC_TX_DMA_TOTAL_REQUEST_LIMIT, 24);

	aqc_hw_update_stats(softc);

	value = AQC_REG_INTR_RESET_DSBL;
	switch (softc->scctx->isc_intr) {
	case IFLIB_INTR_LEGACY:
		value |= AQC_REG_INTR_MODE_LEGACY | AQC_REG_INTR_ISR_COR_EN;
		break;

	case IFLIB_INTR_MSI:
		value |= AQC_REG_INTR_MODE_MSI;
		if (softc->scctx->isc_vectors > 1)
			value |= AQC_REG_INTR_MULT_VEC_EN;
		break;

	case IFLIB_INTR_MSIX:
		value |= AQC_REG_INTR_MODE_MSIX;
		if (softc->scctx->isc_vectors > 1)
			value |= AQC_REG_INTR_MULT_VEC_EN;
		break;

	default:
		device_printf(softc->dev, "unknown interrupt mode\n");
		return (EOPNOTSUPP);
	}
	aqc_hw_write(softc, AQC_REG_INTR_GLOBAL_CONTROL, value);

	aqc_hw_write(softc, AQC_REG_INTR_AUTO_MASK, 0xffffffff);

	value = AQC_INTR_MAP_PCI_EN | (8 << AQC_INTR_MAP_PCI_SHIFT);
	value |= AQC_INTR_MAP_FATAL_EN | (8 << AQC_INTR_MAP_FATAL_SHIFT);
	aqc_hw_write(softc, AQC_REG_INTR_GENERAL_MAP_1, value);

	/* XXX hardware offload */

	return (rc);
}

static int
aqc_if_detach(if_ctx_t ctx)
{
	struct aqc_softc *softc;
	int i;

	softc = iflib_get_softc(ctx);

	aqc_hw_write(softc, AQC_REG_MPI_CONTROL, AQC_MPI_DEINIT);

	for (i = 0; i < softc->scctx->isc_nrxqsets; i++)
		iflib_irq_free(ctx, &softc->rx_ring[i].irq);

	if (softc->mmio_res != NULL)
		bus_release_resource(softc->dev, SYS_RES_MEMORY,
		    softc->mmio_rid, softc->mmio_res);

	return (0);
}

static int
aqc_if_shutdown(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

static int
aqc_if_suspend(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

static int
aqc_if_resume(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

/* Soft queue setup and teardown */
static int
aqc_if_tx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs,
    uint64_t *paddrs, int ntxqs, int ntxqsets)
{
	struct aqc_softc *softc;
	struct aqc_ring *ring;
	uint32_t value;
	int i;
	
	softc = iflib_get_softc(ctx);

	for (i = 0; i < ntxqsets; i++) {
		ring = &softc->tx_ring[i];

		ring->descriptors = (struct aqc_desc *)vaddrs[i];
		ring->ndesc = softc->scctx->isc_ntxd[i];

		aqc_hw_write(softc, AQC_REG_TX_DMA_DESCRIPTOR_BASE_LSW(i),
		    paddrs[i] & 0xffffffff);
		aqc_hw_write(softc, AQC_REG_TX_DMA_DESCRIPTOR_BASE_MSW(i),
		    (paddrs[i] & 0xffffffff00000000) >> 32);

		value = aqc_hw_read(softc,
		    AQC_REG_TX_DMA_DESCRIPTOR_CONTROL(i));
		value &= ~(AQC_TX_DMA_DESCRIPTOR_LEN_MASK
		    << AQC_TX_DMA_DESCRIPTOR_LEN_SHIFT);
		value |= (ring->ndesc & AQC_TX_DMA_DESCRIPTOR_LEN_MASK)
		    << AQC_TX_DMA_DESCRIPTOR_LEN_SHIFT;
		value &= ~AQC_TX_DMA_DESCRIPTOR_WRB_HDR_EN;
		aqc_hw_write(softc, AQC_REG_TX_DMA_DESCRIPTOR_CONTROL(i),
		    value);

		value = aqc_hw_read(softc, AQC_REG_TX_DMA_THRESHOLD(i));
		value &= ~(AQC_TX_DMA_THRESHOLD_WRB_MASK <<
		     AQC_TX_DMA_THRESHOLD_WRB_SHIFT);
		aqc_hw_write(softc, AQC_REG_TX_DMA_THRESHOLD(i), value);
	}

	return (0);
}

static int
aqc_if_rx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs,
    uint64_t *paddrs, int nrxqs, int nrxqsets)
{
	struct aqc_softc *softc;
	struct aqc_ring *ring;
	uint32_t value;
	int i;

	softc = iflib_get_softc(ctx);

	for (i = 0; i < nrxqsets; i++) {
		ring = &softc->rx_ring[i];

		ring->descriptors = (struct aqc_desc *)vaddrs[i];
		ring->ndesc = softc->scctx->isc_nrxd[i];

		aqc_hw_write(softc, AQC_REG_RX_DMA_DESCRIPTOR_BASE_LSW(i),
		    paddrs[i] & 0xffffffff);
		aqc_hw_write(softc, AQC_REG_RX_DMA_DESCRIPTOR_BASE_MSW(i),
		    (paddrs[i] & 0xffffffff00000000) >> 32);

		value = aqc_hw_read(softc,
		    AQC_REG_RX_DMA_DESCRIPTOR_CONTROL(i));
		value &= ~(AQC_RX_DMA_DESCRIPTOR_LEN_MASK
		    << AQC_RX_DMA_DESCRIPTOR_LEN_SHIFT);
		value |= (ring->ndesc & AQC_RX_DMA_DESCRIPTOR_LEN_MASK)
		    << AQC_RX_DMA_DESCRIPTOR_LEN_SHIFT;
		aqc_hw_write(softc, AQC_REG_RX_DMA_DESCRIPTOR_CONTROL(i),
		    value);
	}

	return (0);
}

static void
aqc_if_queues_free(if_ctx_t ctx)
{

	return;
}

/* Device configuration */
static void
aqc_if_init(if_ctx_t ctx)
{
	struct aqc_softc *softc;
	struct ifmediareq ifmr;
	uint32_t value;
	int i;

	softc = iflib_get_softc(ctx);

	aqc_if_media_status(ctx, &ifmr);

	value = aqc_hw_read(softc, AQC_REG_TX_PACKET_BUFFER_CONTROL_1);
	value |= AQC_TX_PACKET_BUFFER_TX_BUF_EN;
	aqc_hw_write(softc, AQC_REG_TX_PACKET_BUFFER_CONTROL_1, value);

	for (i = 0; i < softc->scctx->isc_ntxqsets; i++) {
		value = aqc_hw_read(softc,
		    AQC_REG_TX_DMA_DESCRIPTOR_CONTROL(i));
		value |= AQC_TX_DMA_DESCRIPTOR_EN;
		aqc_hw_write(softc, AQC_REG_TX_DMA_DESCRIPTOR_CONTROL(i),
		    value);
	}

	value = aqc_hw_read(softc, AQC_REG_RX_PACKET_BUFFER_CONTROL_1);
	value |= AQC_RX_PACKET_BUFFER_RX_BUF_EN;
	aqc_hw_write(softc, AQC_REG_RX_PACKET_BUFFER_CONTROL_1, value);

	for (i = 0; i < softc->scctx->isc_nrxqsets; i++) {
		value = aqc_hw_read(softc,
		    AQC_REG_RX_DMA_DESCRIPTOR_CONTROL(i));
		value |= AQC_RX_DMA_DESCRIPTOR_EN;
		aqc_hw_write(softc, AQC_REG_RX_DMA_DESCRIPTOR_CONTROL(i),
		    value);
	}

	value = aqc_hw_read(softc, AQC_REG_RX_FILTER_CONTROL_1);
	value |= AQC_RX_FILTER_BC_EN;
	aqc_hw_write(softc, AQC_REG_RX_FILTER_CONTROL_1, value);
}

static void
aqc_if_stop(if_ctx_t ctx)
{
	struct aqc_softc *softc;
	int i;
	uint32_t value;

	softc = iflib_get_softc(ctx);

	for (i = 0; i < softc->scctx->isc_ntxqsets; i++) {
		value = aqc_hw_read(softc,
		    AQC_REG_TX_DMA_DESCRIPTOR_CONTROL(i));
		value &= ~AQC_TX_DMA_DESCRIPTOR_EN;
		aqc_hw_write(softc, AQC_REG_TX_DMA_DESCRIPTOR_CONTROL(i),
		    value);
	}

	value = aqc_hw_read(softc, AQC_REG_TX_PACKET_BUFFER_CONTROL_1);
	value &= ~AQC_TX_PACKET_BUFFER_TX_BUF_EN;
	aqc_hw_write(softc, AQC_REG_TX_PACKET_BUFFER_CONTROL_1, value);

	for (i = 0; i < softc->scctx->isc_nrxqsets; i++) {
		value = aqc_hw_read(softc,
		    AQC_REG_RX_DMA_DESCRIPTOR_CONTROL(i));
		value &= ~AQC_RX_DMA_DESCRIPTOR_EN;
		aqc_hw_write(softc, AQC_REG_RX_DMA_DESCRIPTOR_CONTROL(i),
		    value);
	}

	value = aqc_hw_read(softc, AQC_REG_RX_PACKET_BUFFER_CONTROL_1);
	value &= ~AQC_RX_PACKET_BUFFER_RX_BUF_EN;
	aqc_hw_write(softc, AQC_REG_RX_PACKET_BUFFER_CONTROL_1, value);
}

static int
aqc_add_multicast_addr(void *arg, struct ifmultiaddr *ifma, int count)
{
	struct aqc_softc *softc;
	uint8_t *mca;
	uint32_t value, f;

	softc = arg;

	if (ifma->ifma_addr->sa_family != AF_LINK)
		return (0);
	if (count >= AQC_HW_MAX_MULTICAST)
		return (0);

	mca = LLADDR((struct sockaddr_dl *)ifma->ifma_addr);
	f = AQC_HW_MULTICAST_FILTER(count);
	value = (mca[2] << 24) | (mca[3] << 16) | (mca[4] << 8) | (mca[5]);
	aqc_hw_write(softc, AQC_REG_RX_UNICAST_FILTER_2(f), value);
	value = (mca[0] << 8) | (mca[1]) | AQC_RX_UNICAST_FILTER_L2_EN |
	    AQC_RX_UNICAST_FILTER_ACT_HOST;
	aqc_hw_write(softc, AQC_REG_RX_UNICAST_FILTER_1(f), value);

	return (1);
}

static void
aqc_if_multi_set(if_ctx_t ctx)
{
	struct aqc_softc *softc;
	if_t ifp;
	int mcnt, i;
	uint32_t value, f;

	softc = iflib_get_softc(ctx);
	ifp = iflib_get_ifp(ctx);

	mcnt = if_multi_apply(ifp, aqc_add_multicast_addr, softc);

	value = aqc_hw_read(softc, AQC_REG_RX_MULTICAST_FILTER_MASK);
	if (mcnt >= AQC_HW_MAX_MULTICAST) {
		value |= AQC_RX_MULTICAST_FILTER_ACCEPT_ALL;
	} else {
		value &= ~AQC_RX_MULTICAST_FILTER_ACCEPT_ALL;
	}
	aqc_hw_write(softc, AQC_REG_RX_MULTICAST_FILTER_MASK, value);

	for (i = mcnt; i < AQC_HW_MAX_MULTICAST; i++) {
		f = AQC_HW_MULTICAST_FILTER(i);
		value = aqc_hw_read(softc, AQC_REG_RX_UNICAST_FILTER_2(f));
		value &= ~AQC_RX_UNICAST_FILTER_L2_EN;
		aqc_hw_write(softc, AQC_REG_RX_UNICAST_FILTER_2(f), value);
	}
}

static int
aqc_if_mtu_set(if_ctx_t ctx, uint32_t mtu)
{
	struct aqc_softc *softc;
	uint16_t max_frame_size;

	softc = iflib_get_softc(ctx);

	if (mtu > AQC_MAX_FRAME_SIZE - ETHER_HDR_LEN - ETHER_CRC_LEN) {
		return (EINVAL);
	}

	max_frame_size = mtu + ETHER_HDR_LEN + ETHER_CRC_LEN;

	/* We can only size buffers in units of 1KB so round up if needed. */
	if (max_frame_size % 1024 != 0) {
		max_frame_size >>= 10;
		max_frame_size <<= 10;
		max_frame_size++;
	}

	if (max_frame_size >
	    AQC_MAX_FRAME_SIZE - ETHER_HDR_LEN - ETHER_CRC_LEN) {
		return (EINVAL);
	}

	softc->scctx->isc_max_frame_size = max_frame_size;
	return (0);
}

static void
aqc_if_media_status(if_ctx_t ctx, struct ifmediareq *ifmr)
{
	struct aqc_softc *softc;
	struct ifnet *ifp;
	
	softc = iflib_get_softc(ctx);
	ifp = iflib_get_ifp(ctx);

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	/* AQC100 apparently does fibre but we'll ignore it for now. */
	
	switch (aqc_fw_get_link_speed(softc)) {
	case AQC_LINK_100M:
		ifmr->ifm_active |= IFM_100_TX;
		if_setbaudrate(ifp, IF_Mbps(100));
		break;
	
	case AQC_LINK_1G:
		ifmr->ifm_active |= IFM_1000_T;
		if_setbaudrate(ifp, IF_Gbps(1));
		break;
	
	case AQC_LINK_2G5:
		ifmr->ifm_active |= IFM_2500_T;
		if_setbaudrate(ifp, IF_Mbps(2500));
		break;
	
	case AQC_LINK_5G:
		ifmr->ifm_active |= IFM_5000_T;
		if_setbaudrate(ifp, IF_Gbps(5));
		break;
	
	case AQC_LINK_10G:
		ifmr->ifm_active |= IFM_10G_T;
		if_setbaudrate(ifp, IF_Gbps(10));
		break;
	
	default:
		ifmr->ifm_status &= ~IFM_ACTIVE;
		iflib_link_state_change(ctx, LINK_STATE_DOWN,
		    ifp->if_baudrate);
		return;
	}

	iflib_link_state_change(ctx, LINK_STATE_UP, ifp->if_baudrate);

	ifmr->ifm_status |= IFM_ACTIVE;
	ifmr->ifm_active |= IFM_FDX;
}

static int
aqc_if_media_change(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

static int
aqc_if_promisc_set(if_ctx_t ctx, int flags)
{
	struct aqc_softc *softc;
	if_t ifp;
	uint32_t value;

	softc = iflib_get_softc(ctx);
	ifp = iflib_get_ifp(ctx);

	value = aqc_hw_read(softc, AQC_REG_RX_FILTER_CONTROL_1);
	if (ifp->if_flags & IFF_PROMISC) {
		value |= AQC_RX_FILTER_L2_PROMISC_MODE;
	} else {
		value &= ~AQC_RX_FILTER_L2_PROMISC_MODE;
	}
	aqc_hw_write(softc, AQC_REG_RX_FILTER_CONTROL_1, value);

	value = aqc_hw_read(softc, AQC_REG_RX_MULTICAST_FILTER_MASK);
	if (ifp->if_flags & IFF_ALLMULTI ||
	    if_multiaddr_count(ifp, -1) > AQC_HW_MAX_MULTICAST) {
		value |= AQC_RX_MULTICAST_FILTER_ACCEPT_ALL;
	} else {
		value &= ~AQC_RX_MULTICAST_FILTER_ACCEPT_ALL;
	}
	aqc_hw_write(softc, AQC_REG_RX_MULTICAST_FILTER_MASK, value);

	return (0);
}

static uint64_t
aqc_if_get_counter(if_ctx_t ctx, ift_counter cnt)
{
	struct aqc_softc *softc = iflib_get_softc(ctx);
	struct ifnet *ifp = iflib_get_ifp(ctx);

	switch (cnt) {
	case IFCOUNTER_IERRORS:
		return (softc->stats.erpr + softc->stats.dpc);
	case IFCOUNTER_OERRORS:
		return (softc->stats.erpt);
	default:
		return (if_get_counter_default(ifp, cnt));
	}
}

static void
aqc_if_update_admin_status(if_ctx_t ctx)
{

	aqc_hw_update_stats(iflib_get_softc(ctx));
}

static void
aqc_if_timer(if_ctx_t ctx, uint16_t qid)
{
	struct aqc_softc *softc;
	struct ifmediareq ifmr;
	uint64_t ticks_now;

	softc = iflib_get_softc(ctx);
	ticks_now = ticks;

	/* Schedule aqc_if_update_admin_status() once per sec */
	if (ticks_now - softc->admin_ticks >= hz) {
		softc->admin_ticks = ticks_now;
		iflib_admin_intr_deferred(ctx);

		aqc_if_media_status(ctx, &ifmr);
	}

	return;

}

/* Interrupt enable / disable */
static void
aqc_if_enable_intr(if_ctx_t ctx)
{

	aqc_hw_write(iflib_get_softc(ctx), AQC_REG_INTR_MASK_SET, 0x1ff);
}

static void
aqc_if_disable_intr(if_ctx_t ctx)
{

	aqc_hw_write(iflib_get_softc(ctx), AQC_REG_INTR_MASK_CLEAR, 0x1ff);
	aqc_hw_write(iflib_get_softc(ctx), AQC_REG_INTR_STATUS_CLEAR, 0x1ff);
}

static int
aqc_if_rx_queue_intr_enable(if_ctx_t ctx, uint16_t rxqid)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

static int
aqc_if_tx_queue_intr_enable(if_ctx_t ctx, uint16_t txqid)
{

	return (0);
}

static int
aqc_if_msix_intr_assign(if_ctx_t ctx, int msix)
{
	struct aqc_softc *softc;
	int i, rc;
	char irq_name[16];

	softc = iflib_get_softc(ctx);

	for (i = 0; i < softc->scctx->isc_nrxqsets; i++) {
		snprintf(irq_name, sizeof(irq_name), "rxq%d", i);
		rc = iflib_irq_alloc_generic(ctx, &softc->rx_ring[i].irq,
		    i + 1, IFLIB_INTR_RX, aqc_handle_rx, &softc->rx_ring[i],
		    i, irq_name);

		if (rc) {
			device_printf(softc->dev,
			    "failed to set up RX handler\n");
			i--;
			goto fail;
		}

		softc->rx_ring[i].msix = i;
	}

	for (i = 0; i < softc->scctx->isc_ntxqsets; i++) {
		snprintf(irq_name, sizeof(irq_name), "txq%d", i);
		iflib_softirq_alloc_generic(ctx, NULL, IFLIB_INTR_TX, NULL, i,
		    irq_name);
	}

	return (0);

fail:
	for (; i >= 0; i--)
		iflib_irq_free(ctx, &softc->rx_ring[i].irq);
	return (rc);
}

/* VLAN support */
static void
aqc_if_vlan_register(if_ctx_t ctx, uint16_t vtag)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

static void
aqc_if_vlan_unregister(if_ctx_t ctx, uint16_t vtag)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

/* ioctl */
static int
aqc_if_priv_ioctl(if_ctx_t ctx, u_long command, caddr_t data)
{

	return (ENOTSUP);
}

static void
aqc_if_debug(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

static void
aqc_if_led_func(if_ctx_t ctx, int onoff)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

int
aqc_intr(void *arg)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

static int
aqc_handle_rx(void *arg __unused)
{

	printf("beep\n");
	return (FILTER_SCHEDULE_THREAD);
}
