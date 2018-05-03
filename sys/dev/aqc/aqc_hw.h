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

enum aqc_media_type {
	AQC_MEDIA_TYPE_UNKNOWN = 0,
	AQC_MEDIA_TYPE_FIBRE,
	AQC_MEDIA_TYPE_TP,
};

#define	AQC_LINK_SPEED_100M	0x00000001
#define	AQC_LINK_SPEED_1G	0x00000002
#define	AQC_LINK_SPEED_2G5	0x00000004
#define	AQC_LINK_SPEED_5G	0x00000008
#define	AQC_LINK_SPEED_10G	0x00000010

#define	AQC_LINK_SPEED_ALL	(	\
    AQC_LINK_SPEED_100M	|		\
    AQC_LINK_SPEED_1G	|		\
    AQC_LINK_SPEED_2G5	|		\
    AQC_LINK_SPEED_5G	|		\
    AQC_LINK_SPEED_10G			\
)

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
#define	AQC_HW_FEATURE_REV_A0	0x01000000
#define	AQC_HW_FEATURE_REV_B0	0x02000000
#define	AQC_HW_FEATURE_REV_B1	0x04000000

struct aqc_ring;
struct aqc_softc;

struct aqc_hw_ops {
	int	(*probe_caps)(struct aqc_softc *);
};

struct aqc_caps {
	enum aqc_media_type	media_type;
	uint32_t		link_speeds;
	uint32_t		tx_rings;
	uint32_t		rx_rings;
};

extern struct aqc_hw_ops	aqc_hw_ops_a0;
extern struct aqc_hw_ops	aqc_hw_ops_b0b1;

extern struct aqc_caps		aqc_caps_a0_aqc100;
extern struct aqc_caps		aqc_caps_a0_aqc107;
extern struct aqc_caps		aqc_caps_a0_aqc108;
extern struct aqc_caps		aqc_caps_a0_aqc109;

extern struct aqc_caps		aqc_caps_b0b1_aqc100;
extern struct aqc_caps		aqc_caps_b0b1_aqc107;
extern struct aqc_caps		aqc_caps_b0b1_aqc108;
extern struct aqc_caps		aqc_caps_b0b1_aqc109;
extern struct aqc_caps		aqc_caps_b0b1_aqc111;
extern struct aqc_caps		aqc_caps_b0b1_aqc112;

extern struct aqc_caps		aqc_caps_b0b1_aqc100s;
extern struct aqc_caps		aqc_caps_b0b1_aqc107s;
extern struct aqc_caps		aqc_caps_b0b1_aqc108s;
extern struct aqc_caps		aqc_caps_b0b1_aqc109s;
extern struct aqc_caps		aqc_caps_b0b1_aqc111s;
extern struct aqc_caps		aqc_caps_b0b1_aqc112s;

extern struct aqc_caps		aqc_caps_b0b1_aqc111e;
extern struct aqc_caps		aqc_caps_b0b1_aqc112e;

int		aqc_hw_probe(struct aqc_softc *);
int		aqc_hw_soft_reset(struct aqc_softc *);

uint32_t	aqc_hw_read(struct aqc_softc *, uint32_t);
void		aqc_hw_write(struct aqc_softc *, uint32_t, uint32_t);

#endif	/* _AQC_HW_H_ */