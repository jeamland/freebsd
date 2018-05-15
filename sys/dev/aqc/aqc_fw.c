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
#include <sys/endian.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/if_var.h>
#include <net/ethernet.h>
#include <net/iflib.h>

#include "aqc.h"
#include "aqc_reg.h"
#include "aqc_hw.h"
#include "aqc_fw.h"

bool
aqc_fw_version_check(uint32_t expected, uint32_t actual)
{

	if (AQC_FW_VERSION_MAJOR(expected) != AQC_FW_VERSION_MAJOR(actual))
		return (false);
	if (AQC_FW_VERSION_MINOR(expected) > AQC_FW_VERSION_MINOR(actual))
		return (false);
	return (true);
}

int
aqc_fw_probe(struct aqc_softc *softc)
{
	int error;
	uint32_t fw_version;

	if ((error = aqc_hw_soft_reset(softc)) != 0) {
		return (error);
	}

	fw_version = aqc_hw_read(softc, AQC_REG_FW_IMAGE_ID);

	if (aqc_fw_version_check(AQC_FW_VERSION_1X, fw_version)) {
		softc->fw_ops = &aqc_fw_ops_1x;
	} else if (aqc_fw_version_check(AQC_FW_VERSION_2X, fw_version) ||
	    aqc_fw_version_check(AQC_FW_VERSION_3X, fw_version)) {
		softc->fw_ops = &aqc_fw_ops_2x;
	} else {
		device_printf(softc->dev, "unknown firmware version\n");
		return (EOPNOTSUPP);
	}

	aqc_fw_set_state(softc, AQC_MPI_RESET);

	return (softc->fw_ops->init(softc));
}

int
aqc_fw_copyin(struct aqc_softc *softc, uint32_t addr, uint32_t len,
    uint32_t *buffer)
{
	int error;

	error = 0;

	AQC_HW_POLL(aqc_hw_read(softc, AQC_REG_FW_RAM_SEMAPHORE) == 1, 1, 10000,
	    error);
	if (error != 0) {
		aqc_hw_write(softc, AQC_REG_FW_RAM_SEMAPHORE, 1);
		if (aqc_hw_read(softc, AQC_REG_FW_RAM_SEMAPHORE) == 0) {
			return (ETIMEDOUT);
		}
	}

	aqc_hw_write(softc, AQC_REG_MAILBOX_ADDRESS, addr);

	for (; len > 0 && error == 0; len--) {
		aqc_hw_write(softc, AQC_REG_MAILBOX_1, AQC_MAILBOX_EXECUTE);

		if (AQC_HW_FEATURE(softc, AQC_HW_FEATURE_REV_B0)) {
			AQC_HW_POLL(
			    aqc_hw_read(softc, AQC_REG_MAILBOX_ADDRESS) != addr,
			    1, 1000, error);
		} else {
			AQC_HW_POLL(
			    (aqc_hw_read(softc, AQC_REG_MAILBOX_1) &
			     AQC_MAILBOX_BUSY) == 0, 1, 1000, error);
		}

		*buffer = aqc_hw_read(softc, AQC_REG_MAILBOX_DATA);
		
		buffer += 1;
		addr += 4;
	}

	aqc_hw_write(softc, AQC_REG_FW_RAM_SEMAPHORE, 1);
	return (0);
}

int
aqc_fw_copy_mac_from_efuse(struct aqc_softc *softc, uint32_t efuse_addr,
    uint8_t *mac)
{
	int error;
	uint32_t mac_buffer[2];

	/* XXX magic number */
	error = aqc_fw_copyin(softc, efuse_addr + (40 * 4), 2, mac_buffer);
	if (error != 0)
		return (error);

	mac_buffer[0] = bswap32(mac_buffer[0]);
	mac_buffer[1] = bswap32(mac_buffer[1]);

	bcopy(mac_buffer, mac, ETHER_ADDR_LEN);

	return (0);
}

int
aqc_fw_read_mbox(struct aqc_softc *softc, struct aqc_fw_mbox *mbox)
{

	return (aqc_fw_copyin(softc, softc->mbox_addr,
	    sizeof(struct aqc_fw_mbox) / sizeof(uint32_t), (uint32_t *)mbox));
}
