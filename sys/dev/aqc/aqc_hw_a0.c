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
#include <sys/kernel.h>
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/if_var.h>
#include <net/ethernet.h>
#include <net/iflib.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "aqc.h"
#include "aqc_reg.h"
#include "aqc_hw.h"
#include "aqc_fw.h"

#define	AQC_A0_TX_RINGS	4
#define	AQC_A0_RX_RINGS	4

struct aqc_caps a0_base_caps = {
	.tx_rings = AQC_A0_TX_RINGS,
	.rx_rings = AQC_A0_RX_RINGS,
	.link_speeds = 0,
};

static int
aqc_a0_probe_caps(struct aqc_softc *softc)
{

	softc->caps = a0_base_caps;

	switch (pci_get_device(softc->dev)) {
	case AQC_DEVICE_ID_D100:
		softc->caps.media_type = AQC_MEDIA_TYPE_FIBRE;
		softc->caps.link_speeds =
		    AQC_LINK_SPEED_ALL & ~AQC_LINK_SPEED_10G;
		return (0);
		break;
	
	case AQC_DEVICE_ID_0001:
	case AQC_DEVICE_ID_D107:
		softc->caps.media_type = AQC_MEDIA_TYPE_TP;
		softc->caps.link_speeds = AQC_LINK_SPEED_ALL;
		return (0);
		break;
	
	case AQC_DEVICE_ID_D108:
		softc->caps.media_type = AQC_MEDIA_TYPE_TP;
		softc->caps.link_speeds =
		    AQC_LINK_SPEED_ALL & ~AQC_LINK_SPEED_10G;
		return (0);
		break;
	
	case AQC_DEVICE_ID_D109:
		softc->caps.media_type = AQC_MEDIA_TYPE_TP;
		softc->caps.link_speeds = AQC_LINK_SPEED_ALL & 
		    ~(AQC_LINK_SPEED_10G | AQC_LINK_SPEED_5G);
		return (0);
		break;

	default:
		return (ENXIO);
		break;
	}
}

struct aqc_hw_ops aqc_hw_ops_a0 = {
	.probe_caps = aqc_a0_probe_caps,
};