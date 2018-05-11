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

#ifndef	_AQC_HW_H_
#define	_AQC_HW_H_

#define	AQC_HW_SUPPORT_SPEED(s)	((softc)->link_speeds & s)

#define	AQC_HW_POLL(condition, usecs, iterations, err)			\
	do {								\
		unsigned int _i;					\
		for (_i = (iterations); !(condition) && (_i > 0);	\
		    _i--) {						\
			DELAY(usecs);					\
		}							\
		if (_i <= 0)						\
			err = ETIMEDOUT;				\
	} while (0);

#define	AQC_HW_FEATURE(softc, feature)	\
	(((softc)->chip_features & (feature)) != 0)

#define	AQC_HW_FEATURE_MIPS	0x00000001
#define	AQC_HW_FEATURE_TPO2	0x00000002
#define	AQC_HW_FEATURE_RPF2	0x00000004
#define	AQC_HW_FEATURE_MPI_AQ	0x00000008
#define	AQC_HW_FEATURE_REV_B0	0x02000000
#define	AQC_HW_FEATURE_REV_B1	0x04000000

#define	AQC_TX_RING_COUNT	4
#define	AQC_RX_RING_COUNT	4

#define	AQC_TXBUF_MAX		160

struct aqc_ring;
struct aqc_softc;

int		aqc_hw_probe(struct aqc_softc *);
int		aqc_hw_set_mac(struct aqc_softc *);
int		aqc_hw_soft_reset(struct aqc_softc *);
int		aqc_hw_update_stats(struct aqc_softc *);

uint32_t	aqc_hw_read(struct aqc_softc *, uint32_t);
void		aqc_hw_write(struct aqc_softc *, uint32_t, uint32_t);


#endif	/* _AQC_HW_H_ */