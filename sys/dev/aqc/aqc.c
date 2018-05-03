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
#include <net/ethernet.h>
#include <net/iflib.h>

#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_rss.h"

#include "ifdi_if.h"

#include "aqc.h"
#include "aqc_hw.h"
#include "aqc_fw.h"

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
	.isc_q_align = PAGE_SIZE,
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
	.isc_nrxd_default = {AQC_DEFAULT_RXD},
	.isc_ntxd_default = {AQC_DEFAULT_TXD},
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

	softc->caps = malloc(sizeof(struct aqc_caps), M_AQC, M_WAITOK);
	if (softc->caps == NULL) {
		rc = ENOMEM;
		goto fail;
	}

	/* Look up ops and caps. */
	if ((rc = aqc_hw_probe(softc)) != 0) {
		device_printf(softc->dev, "hardware probe failed\n");
		goto fail;
	}
	if ((rc = aqc_fw_probe(softc)) != 0) {
		device_printf(softc->dev, "firmware probe failed\n");
		goto fail;
	}
	
	/* Fill in scctx stuff. */
	/* XXX THIS STUFF IS NOT RIGHT YET */
	#if 0
	iflib_set_mac(ctx, softc->func.mac_addr);

	scctx->isc_txrx = &bnxt_txrx;
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

	// return (rc);

fail:
	if (softc->caps != NULL)
		free(softc->caps, M_AQC);

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
	int rc;

	softc = iflib_get_softc(ctx);
	ifp = iflib_get_ifp(ctx);
	rc = 0;

	device_printf(softc->dev, "hi there\n");
	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (rc);
}

static int
aqc_if_detach(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
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

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

static int
aqc_if_rx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs,
    uint64_t *paddrs, int nrxqs, int nrxqsets)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

static void
aqc_if_queues_free(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

/* Device configuration */
static void
aqc_if_init(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

static void
aqc_if_stop(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

static void
aqc_if_multi_set(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

static int
aqc_if_mtu_set(if_ctx_t ctx, uint32_t mtu)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

static void
aqc_if_media_status(if_ctx_t ctx, struct ifmediareq *ifmr)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
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

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

static uint64_t
aqc_if_get_counter(if_ctx_t ctx, ift_counter cnt)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

static void
aqc_if_update_admin_status(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

static void
aqc_if_timer(if_ctx_t ctx, uint16_t qid)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

/* Interrupt enable / disable */
static void
aqc_if_enable_intr(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

static void
aqc_if_disable_intr(if_ctx_t ctx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
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

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
}

static int
aqc_if_msix_intr_assign(if_ctx_t ctx, int msix)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
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

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (0);
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

