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

#include "aqc.h"
#include "aqc_reg.h"
#include "aqc_hw.h"
#include "aqc_fw.h"

#define	AQC_FW_2X_RATE_10G	0x800
#define	AQC_FW_2X_RATE_5G	0x400
#define	AQC_FW_2X_RATE_2G5	0x200
#define	AQC_FW_2X_RATE_1G	0x100
#define	AQC_FW_2X_RATE_100M	0x020

#define	AQC_FW_2X_RATE_MASK	(	\
	AQC_FW_2X_RATE_100M	|	\
	AQC_FW_2X_RATE_1G	|	\
	AQC_FW_2X_RATE_2G5	|	\
	AQC_FW_2X_RATE_5G	|	\
	AQC_FW_2X_RATE_10G		\
)

static int
aqc_fw_2x_init(struct aqc_softc *softc)
{
	int error;

	error = 0;

	AQC_HW_POLL(
	    (softc->mbox_addr = aqc_hw_read(softc, AQC_REG_MPI_MBOX_ADDR)) != 0,
	    1000, 10, error);
	return (error);
}

static int
aqc_fw_2x_get_permanent_mac(struct aqc_softc *softc)
{
	uint32_t efuse_addr;

	efuse_addr = aqc_hw_read(softc, AQC_REG_MPI_FW_2X_EFUSE_ADDR);
	if (efuse_addr == 0) {
		device_printf(softc->dev, "unable to read MAC address\n");
		return (EIO);
	}

	return (aqc_fw_copy_mac_from_efuse(softc, efuse_addr,
	    softc->mac_addr));
}

static int
aqc_fw_2x_set_state(struct aqc_softc *softc, enum aqc_fw_state state)
{

	return (0);
}

static int
aqc_fw_2x_set_link_speed(struct aqc_softc *softc, uint32_t speed)
{
	uint32_t	rate_mask;

	rate_mask = 0;

	if ((speed & AQC_LINK_10G) != 0)
		rate_mask |= AQC_FW_2X_RATE_10G;

	if ((speed & AQC_LINK_5G) != 0)
		rate_mask |= AQC_FW_2X_RATE_5G;

	if ((speed & AQC_LINK_2G5) != 0)
		rate_mask |= AQC_FW_2X_RATE_2G5;

	if ((speed & AQC_LINK_1G) != 0)
		rate_mask |= AQC_FW_2X_RATE_1G;

	if ((speed & AQC_LINK_100M) != 0)
		rate_mask |= AQC_FW_2X_RATE_100M;

	aqc_hw_write(softc, AQC_REG_MPI_CONTROL, rate_mask);

	return (0);
}

static uint32_t
aqc_fw_2x_get_link_speed(struct aqc_softc *softc)
{
	uint32_t speed;

	speed = aqc_hw_read(softc, AQC_REG_MPI_FW_2X_STATE);
	speed = speed & AQC_FW_2X_RATE_MASK;

	if (speed & AQC_FW_2X_RATE_10G)
		return (AQC_LINK_10G);
	else if (speed & AQC_FW_2X_RATE_5G)
		return (AQC_LINK_5G);
	else if (speed & AQC_FW_2X_RATE_2G5)
		return (AQC_LINK_2G5);
	else if (speed & AQC_FW_2X_RATE_1G)
		return (AQC_LINK_1G);
	else if (speed & AQC_FW_2X_RATE_100M)
		return (AQC_LINK_100M);
	
	return (AQC_LINK_UNKNOWN);
}

struct aqc_fw_ops aqc_fw_ops_2x = {
	.init = aqc_fw_2x_init,
	.get_permanent_mac = aqc_fw_2x_get_permanent_mac,
	.set_state = aqc_fw_2x_set_state,
	.set_link_speed = aqc_fw_2x_set_link_speed,
	.get_link_speed = aqc_fw_2x_get_link_speed,
};
