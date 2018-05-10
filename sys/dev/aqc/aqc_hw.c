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
 *    notice, this list of conditions and the following disoftclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disoftclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DIsoftcLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
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

#define AQC_UNICAST_FILTER_MAC	0	/* RX unicast filter for primary MAC */

static int	aqc_hw_soft_reset_rbl(struct aqc_softc *);
static int	aqc_hw_soft_reset_flb(struct aqc_softc *);

int
aqc_hw_probe(struct aqc_softc *softc)
{

	if (pci_get_vendor(softc->dev) != AQC_VENDOR_ID_AQUANTIA)
		return (ENXIO);
	
	softc->tx_rings = AQC_TX_RING_COUNT;
	softc->rx_rings = AQC_RX_RING_COUNT;

	switch (pci_get_device(softc->dev)) {
	case AQC_DEVICE_ID_D100:
	case AQC_DEVICE_ID_AQC100:
	case AQC_DEVICE_ID_AQC100S:
		softc->media_type = AQC_MEDIA_TYPE_FIBRE;
		softc->link_speeds = AQC_LINK_ALL & ~AQC_LINK_10G;
		break;

	case AQC_DEVICE_ID_0001:
	case AQC_DEVICE_ID_D107:
	case AQC_DEVICE_ID_AQC107:
	case AQC_DEVICE_ID_AQC107S:
		softc->media_type = AQC_MEDIA_TYPE_TP;
		softc->link_speeds = AQC_LINK_ALL;
		break;

	case AQC_DEVICE_ID_D108:
	case AQC_DEVICE_ID_AQC108:
	case AQC_DEVICE_ID_AQC108S:
	case AQC_DEVICE_ID_AQC111:
	case AQC_DEVICE_ID_AQC111S:
	case AQC_DEVICE_ID_AQC111E:
		softc->media_type = AQC_MEDIA_TYPE_TP;
		softc->link_speeds = AQC_LINK_ALL & ~AQC_LINK_10G;
		break;

	case AQC_DEVICE_ID_D109:
	case AQC_DEVICE_ID_AQC109:
	case AQC_DEVICE_ID_AQC109S:
	case AQC_DEVICE_ID_AQC112:
	case AQC_DEVICE_ID_AQC112S:
	case AQC_DEVICE_ID_AQC112E:
		softc->media_type = AQC_MEDIA_TYPE_TP;
		softc->link_speeds = AQC_LINK_ALL &
		    ~(AQC_LINK_10G | AQC_LINK_5G);
		break;
	
	default:
		return (ENXIO);
	}

	switch(aqc_hw_read(softc, AQC_REG_MIF_ID) & 0xf) {
	case 0x2:
		softc->chip_features =
		    AQC_HW_FEATURE_REV_B0 |
		    AQC_HW_FEATURE_MPI_AQ |
		    AQC_HW_FEATURE_MIPS |
		    AQC_HW_FEATURE_TPO2 |
		    AQC_HW_FEATURE_RPF2;
		break;
	
	case 0xa:
		softc->chip_features =
		    AQC_HW_FEATURE_REV_B1 |
		    AQC_HW_FEATURE_MPI_AQ |
		    AQC_HW_FEATURE_MIPS |
		    AQC_HW_FEATURE_TPO2 |
		    AQC_HW_FEATURE_RPF2;
		break;

	default:
		device_printf(softc->dev, "unknown hardware revision\n");
		return (EOPNOTSUPP);
	}

	return (0);
}

int
aqc_hw_set_mac(struct aqc_softc *softc)
{
	uint32_t lsw, msw, value;

	msw = (softc->mac_addr[0] << 8) | softc->mac_addr[1];
	lsw = (softc->mac_addr[2] << 24) | (softc->mac_addr[3] << 16) |
	    (softc->mac_addr[4] << 8) | softc->mac_addr[5];

	value = aqc_hw_read(softc,
	    AQC_REG_RX_UNICAST_FILTER_2(AQC_UNICAST_FILTER_MAC));
	value &= ~(AQC_RX_UNICAST_FILTER_L2_EN);
	aqc_hw_write(softc,
	    AQC_REG_RX_UNICAST_FILTER_2(AQC_UNICAST_FILTER_MAC), value);

	aqc_hw_write(softc,
	    AQC_REG_RX_UNICAST_FILTER_1(AQC_UNICAST_FILTER_MAC), lsw);

	value &= 0xffff0000;
	value |= msw;
	aqc_hw_write(softc,
	    AQC_REG_RX_UNICAST_FILTER_2(AQC_UNICAST_FILTER_MAC), value);

	value |= AQC_RX_UNICAST_FILTER_L2_EN;
	aqc_hw_write(softc,
	    AQC_REG_RX_UNICAST_FILTER_2(AQC_UNICAST_FILTER_MAC), value);

	return (0);
}

int
aqc_hw_soft_reset(struct aqc_softc *softc)
{
	int i, err;
	uint32_t	flash_boot_status, boot_exit_code, value;
	uint32_t	fw_version;

	for (i = 0; i < 1000; i++) {
		flash_boot_status = aqc_hw_read(softc,
		    AQC_REG_DAISY_CHAIN_STATUS_1);
		boot_exit_code = aqc_hw_read(softc, AQC_REG_MPI_BOOT_EXIT_CODE);
		if (flash_boot_status !=
		    (AQC_DAISY_CHAIN_PHY_ERROR | AQC_DAISY_CHAIN_MAC_ERROR))
			break;
		
		if (boot_exit_code != 0)
			break;
	}

	if (i >= 1000) {
		device_printf(softc->dev, "no firmware started\n");
		return (EOPNOTSUPP);
	}

	/*
	 * Firmware version 1.x can boot up in an invalid POWER state.
	 * Work around this by forcing it in to DEINIT state.
	 */
	fw_version = aqc_hw_read(softc, AQC_REG_FW_IMAGE_ID);
	if (aqc_fw_version_check(AQC_FW_VERSION_1X, fw_version)) {
		value = aqc_hw_read(softc, AQC_REG_MPI_CONTROL);
		value &= 0xffff;
		aqc_hw_write(softc, AQC_REG_MPI_CONTROL, value);
		
		AQC_HW_POLL((aqc_hw_read(softc, AQC_REG_MPI_FW_1X_STATE) &
		    AQC_MPI_FW_1X_STATE_MASK) == AQC_MPI_DEINIT, 10, 1000, err);
	}

	if (boot_exit_code != 0) {
		/* We're using the ROM boot loader. */
		return (aqc_hw_soft_reset_rbl(softc));
	}

	/* We're using the flash boot loader. */
	return (aqc_hw_soft_reset_flb(softc));
}

int
aqc_hw_update_stats(struct aqc_softc *softc)
{
	struct aqc_fw_mbox mbox;
	int error;

	if ((error = aqc_fw_read_mbox(softc, &mbox)) != 0)
		return (error);
	mbox.stats.dpc = aqc_hw_read(softc, AQC_REG_RX_DMA_STATS_COUNTER_7);

#define _ACCUMULATE(V)	\
    softc->stats.V += mbox.stats.V - softc->fw_stats.V

	_ACCUMULATE(uprc);
	_ACCUMULATE(mprc);
	_ACCUMULATE(bprc);
	_ACCUMULATE(erpt);
	_ACCUMULATE(uptc);
	_ACCUMULATE(mptc);
	_ACCUMULATE(bptc);
	_ACCUMULATE(erpr);
	_ACCUMULATE(mbtc);
	_ACCUMULATE(bbtc);
	_ACCUMULATE(mbrc);
	_ACCUMULATE(bbrc);
	_ACCUMULATE(ubrc);
	_ACCUMULATE(ubtc);
	_ACCUMULATE(dpc);

#undef _ACCUMULATE

	softc->fw_stats = mbox.stats;

	softc->stats.dma_pkts_rx =
	    (uint64_t)aqc_hw_read(softc, AQC_REG_RX_DMA_STATS_COUNTER_2) << 32;
	softc->stats.dma_pkts_rx +=
	    aqc_hw_read(softc, AQC_REG_RX_DMA_STATS_COUNTER_1);

	softc->stats.dma_pkts_tx =
	    (uint64_t)aqc_hw_read(softc, AQC_REG_TX_DMA_STATS_COUNTER_2) << 32;
	softc->stats.dma_pkts_tx +=
	    aqc_hw_read(softc, AQC_REG_TX_DMA_STATS_COUNTER_1);

	softc->stats.dma_bytes_rx =
	    (uint64_t)aqc_hw_read(softc, AQC_REG_RX_DMA_STATS_COUNTER_4) << 32;
	softc->stats.dma_bytes_rx +=
	    aqc_hw_read(softc, AQC_REG_RX_DMA_STATS_COUNTER_3);

	softc->stats.dma_bytes_tx =
	    (uint64_t)aqc_hw_read(softc, AQC_REG_TX_DMA_STATS_COUNTER_4) << 32;
	softc->stats.dma_bytes_tx +=
	    aqc_hw_read(softc, AQC_REG_TX_DMA_STATS_COUNTER_3);

	return (0);
}

uint32_t
aqc_hw_read(struct aqc_softc *softc, uint32_t reg)
{

	return (bus_space_read_4(softc->mmio_tag, softc->mmio_handle, reg));
}

void
aqc_hw_write(struct aqc_softc *softc, uint32_t reg, uint32_t value)
{

	bus_space_write_4(softc->mmio_tag, softc->mmio_handle, reg, value);
}

static int
aqc_hw_soft_reset_rbl(struct aqc_softc *softc)
{
	int i;
	uint32_t value;

	aqc_hw_write(softc, AQC_REG_GLOBAL_CONTROL_2,
	    AQC_GCTRL_MCP_RESET_PULSE |
	    AQC_GCTRL_MIF_ITR_INTR_FLB |
	    AQC_GCTRL_MIF_ITR_INTR_MAILBOX |
	    AQC_GCTRL_WATCHDOG_TIMER_ENABLE |
	    AQC_GCTRL_UP_RUN_STALL);
	aqc_hw_write(softc, AQC_REG_SEMAPHORE_01, 0x1);
	aqc_hw_write(softc, AQC_REG_MIF_POWER_GATE_ENABLE_CONTROL, 0x0);

	/* Alter RBL status - XXX find out what this actually means */
	aqc_hw_write(softc, AQC_REG_MPI_BOOT_EXIT_CODE, 0xdead); /* XXX magic number */

	/* Reset SPI */
	value = aqc_hw_read(softc, AQC_REG_NVR_PROVISIONING_4);
	aqc_hw_write(softc, AQC_REG_NVR_PROVISIONING_4, value | AQC_NVR_RESET);

	/* Enable RX control register reset */
	value = aqc_hw_read(softc, AQC_REG_RX_CONTROL_1);
	value &= ~AQC_RX_CONTROL_REG_RESET_DSBL;
	aqc_hw_write(softc, AQC_REG_RX_CONTROL_1, value);
	
	/* Enable TX control register reset */
	value = aqc_hw_read(softc, AQC_REG_TX_CONTROL_1);
	value &= ~AQC_TX_CONTROL_REG_RESET_DSBL;
	aqc_hw_write(softc, AQC_REG_TX_CONTROL_1, value);

	/* Enable MAC-PHY control register reset */
	value = aqc_hw_read(softc, AQC_REG_MAC_PHY_CONTROL_1);
	value &= ~AQC_MAC_PHY_CONTROL_REG_RESET_DSBL;
	aqc_hw_write(softc, AQC_REG_MAC_PHY_CONTROL_1, value);

	/* Enable global register reset and issue the reset command */
	value = aqc_hw_read(softc, AQC_REG_CONTROL_1);
	value &= ~AQC_CONTROL_REG_RESET_DSBL;
	value |= AQC_CONTROL_SOFT_RESET;
	aqc_hw_write(softc, AQC_REG_CONTROL_1, value);

	aqc_hw_write(softc, AQC_REG_GLOBAL_CONTROL_2,
	    AQC_GCTRL_MCP_RESET_PULSE |
	    AQC_GCTRL_MIF_ITR_INTR_FLB |
	    AQC_GCTRL_MIF_ITR_INTR_MAILBOX |
	    AQC_GCTRL_WATCHDOG_TIMER_ENABLE);

	for (i = 0; i < 1000; i++) {
		value = aqc_hw_read(softc, AQC_REG_MPI_BOOT_EXIT_CODE);
		if (value != 0 && value != 0xdead)
			break;
		DELAY(10 * 1000);
	}

	if (value == 0xf1a7) {	/* XXX magic number */
		device_printf(softc->dev, "no firmware detected\n");
		return (EOPNOTSUPP);
	}

	for (i = 0; i < 1000; i++) {
		if (aqc_hw_read(softc, AQC_REG_FW_IMAGE_ID) != 0)
			break;
		DELAY(10 * 1000);
	}
	if (i >= 1000) {
		device_printf(softc->dev, "firmware kickstart failed\n");
		return (EIO);
	}

	/* Older firmware requires a fixed delay after init */
	DELAY(15 * 1000);

	return (0);
}

static int
aqc_hw_soft_reset_flb(struct aqc_softc *softc)
{
	int i;
	uint32_t value;

	aqc_hw_write(softc, AQC_REG_GLOBAL_CONTROL_2,
	    AQC_GCTRL_MCP_RESET_PULSE |
	    AQC_GCTRL_MIF_ITR_INTR_FLB |
	    AQC_GCTRL_MIF_ITR_INTR_MAILBOX |
	    AQC_GCTRL_WATCHDOG_TIMER_ENABLE |
	    AQC_GCTRL_UP_RUN_STALL);
	DELAY(50 * 1000);

	/* Reset SPI */
	value = aqc_hw_read(softc, AQC_REG_NVR_PROVISIONING_4);
	aqc_hw_write(softc, AQC_REG_NVR_PROVISIONING_4, value | AQC_NVR_RESET);

	value = aqc_hw_read(softc, AQC_REG_CONTROL_1);
	value &= ~AQC_CONTROL_REG_RESET_DSBL;
	value |= AQC_CONTROL_SOFT_RESET;
	aqc_hw_write(softc, AQC_REG_CONTROL_1, value);

	/* Kickstart MAC */
	aqc_hw_write(softc, AQC_REG_GLOBAL_CONTROL_2,
	    AQC_GCTRL_MCP_RESET |
	    AQC_GCTRL_MIF_ITR_INTR_FLB |
	    AQC_GCTRL_MIF_ITR_INTR_MAILBOX |
	    AQC_GCTRL_WATCHDOG_TIMER_ENABLE);
	aqc_hw_write(softc, AQC_REG_MIF_POWER_GATE_ENABLE_CONTROL, 0x0);
	aqc_hw_write(softc, AQC_REG_GENERAL_PROVISIONING_9,
	    AQC_GENERAL_HOST_BOOT_LOAD_ENABLE);

	/* Reset SPI again */
	value = aqc_hw_read(softc, AQC_REG_NVR_PROVISIONING_4);
	aqc_hw_write(softc, AQC_REG_NVR_PROVISIONING_4, value | AQC_NVR_RESET);
	DELAY(10 * 1000);

	/* Clear SPI reset */
	aqc_hw_write(softc, AQC_REG_NVR_PROVISIONING_4, value & ~AQC_NVR_RESET);

	aqc_hw_write(softc, AQC_REG_GLOBAL_CONTROL_2,
	    AQC_GCTRL_FLB_KICKSTART |
	    AQC_GCTRL_MCP_RESET |
	    AQC_GCTRL_MIF_ITR_INTR_FLB |
	    AQC_GCTRL_MIF_ITR_INTR_MAILBOX |
	    AQC_GCTRL_WATCHDOG_TIMER_ENABLE);

	for (i = 0; i < 1000; i++) {
		value = aqc_hw_read(softc, AQC_REG_DAISY_CHAIN_STATUS_1);
		if ((value & AQC_DAISY_CHAIN_MAC_BOOT_COMPLETE) != 0)
			break;
		DELAY(10 * 1000);
	}
	if (i >= 1000) {
		device_printf(softc->dev, "MAC kickstart failed\n");
		return (EIO);
	}

	/* Firmware reset */
	aqc_hw_write(softc, AQC_REG_GLOBAL_CONTROL_2,
	    AQC_GCTRL_MCP_RESET |
	    AQC_GCTRL_MIF_ITR_INTR_FLB |
	    AQC_GCTRL_MIF_ITR_INTR_MAILBOX |
	    AQC_GCTRL_WATCHDOG_TIMER_ENABLE);
	DELAY(50 * 1000);
	aqc_hw_write(softc, AQC_REG_SEMAPHORE_01, 0x1);

	/* Enable RX control register reset */
	value = aqc_hw_read(softc, AQC_REG_RX_CONTROL_1);
	value &= ~AQC_RX_CONTROL_REG_RESET_DSBL;
	aqc_hw_write(softc, AQC_REG_RX_CONTROL_1, value);
	
	/* Enable TX control register reset */
	value = aqc_hw_read(softc, AQC_REG_TX_CONTROL_1);
	value &= ~AQC_TX_CONTROL_REG_RESET_DSBL;
	aqc_hw_write(softc, AQC_REG_TX_CONTROL_1, value);

	/* Enable MAC-PHY control register reset */
	value = aqc_hw_read(softc, AQC_REG_MAC_PHY_CONTROL_1);
	value &= ~AQC_MAC_PHY_CONTROL_REG_RESET_DSBL;
	aqc_hw_write(softc, AQC_REG_MAC_PHY_CONTROL_1, value);

	/* Enable global register reset and issue the reset command */
	value = aqc_hw_read(softc, AQC_REG_CONTROL_1);
	value &= ~AQC_CONTROL_REG_RESET_DSBL;
	value |= AQC_CONTROL_SOFT_RESET;
	aqc_hw_write(softc, AQC_REG_CONTROL_1, value);

	for (i = 0; i < 1000; i++) {
		if (aqc_hw_read(softc, AQC_REG_FW_IMAGE_ID) != 0)
			break;
		DELAY(10 * 1000);
	}
	if (i >= 1000) {
		device_printf(softc->dev, "firmware kickstart failed\n");
		return (EIO);
	}

	/* Older firmware requires a fixed delay after init */
	DELAY(15 * 1000);

	return (0);
}
