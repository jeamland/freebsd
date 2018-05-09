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

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (EIO);
}

static void
aqc_isc_txd_flush(void *arg, uint16_t txqid, qidx_t pidx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

static int
aqc_isc_txd_credits_update(void *arg, uint16_t txqid, bool clear)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (EIO);
}

static void
aqc_isc_rxd_refill(void *arg, if_rxd_update_t iru)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

static void
aqc_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, qidx_t pidx)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
}

static int
aqc_isc_rxd_available(void *arg, uint16_t rxqid, qidx_t idx, qidx_t budget)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (EIO);
}

static int
aqc_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (EIO);
}

