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

static int
aqc_fw_1x_init(struct aqc_softc *softc)
{
	int i;

	if (aqc_hw_read(softc, AQC_REG_SCRATCH_PAD_29) == 0) {
		aqc_hw_write(softc, AQC_REG_SCRATCH_PAD_29, 0xdeaec2de);
	}

	aqc_hw_write(softc, AQC_REG_SCRATCH_PAD_26, 0);

	for (i = 0; i < 1000; i++) {
		softc->mbox_addr = aqc_hw_read(softc, AQC_REG_SCRATCH_PAD_25);
		if (softc->mbox_addr != 0)
			break;
		DELAY(100);
	}
	if (i >= 1000)
		return (ETIMEDOUT);

	return (0);
}

static int
aqc_fw_1x_permanent_mac(struct aqc_softc *softc, uint8_t *mac)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (EOPNOTSUPP);
}

static int
aqc_fw_1x_set_link_speed(struct aqc_softc *softc, uint32_t speed)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (EOPNOTSUPP);
}

static int
aqc_fw_1x_set_state(struct aqc_softc *softc, enum aqc_fw_state state)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (EOPNOTSUPP);
}

static int
aqc_fw_1x_update_link_status(struct aqc_softc *softc)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (EOPNOTSUPP);
}

static int
aqc_fw_1x_update_stats(struct aqc_softc *softc)
{

	AQC_XXX_UNIMPLEMENTED_FUNCTION;
	return (EOPNOTSUPP);
}

struct aqc_fw_ops aqc_fw_ops_1x = {
	.init = aqc_fw_1x_init,
	.permanent_mac = aqc_fw_1x_permanent_mac,
	.set_link_speed = aqc_fw_1x_set_link_speed,
	.set_state = aqc_fw_1x_set_state,
	.update_link_status = aqc_fw_1x_update_link_status,
	.update_stats = aqc_fw_1x_update_stats,
};
