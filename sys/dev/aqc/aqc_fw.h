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

#ifndef	_AQC_FW_H_
#define	_AQC_FW_H_

#define	AQC_FW_VERSION_MAJOR(x)	(x & 0xff000000)
#define	AQC_FW_VERSION_MINOR(x)	(x & 0x00ffffff)

#define	AQC_FW_VERSION_1X	0x01050006
#define	AQC_FW_VERSION_2X	0x02000000
#define	AQC_FW_VERSION_3X	0x03000000

enum aqc_fw_state {
	AQC_MPI_DEINIT = 0,
	AQC_MPI_RESET = 1,
	AQC_MPI_INIT = 2,
	AQC_MPI_POWER = 4,
};

struct aqc_fw_rpc_tid {
	union {
		uint32_t val;
		struct {
			uint16_t tid;
			uint16_t len;
		};
	};
};

struct aqc_fw_mbox_header {
	uint32_t version;
	uint32_t transaction_id;
	uint32_t error;
};

struct aqc_fw_mbox {
	struct aqc_fw_mbox_header header;
	struct aqc_fw_stats stats;
};

struct aqc_fw_ops {
	int		(*init)(struct aqc_softc *);
	int		(*get_permanent_mac)(struct aqc_softc *);
	int		(*set_state)(struct aqc_softc *, enum aqc_fw_state);
	int		(*set_link_speed)(struct aqc_softc *, uint32_t);
	uint32_t	(*get_link_speed)(struct aqc_softc *);
};

extern struct aqc_fw_ops aqc_fw_ops_1x;
extern struct aqc_fw_ops aqc_fw_ops_2x;

bool	aqc_fw_version_check(uint32_t, uint32_t);

int		aqc_fw_probe(struct aqc_softc *);
int		aqc_fw_copyin(struct aqc_softc *, uint32_t, uint32_t,
		    uint32_t *);
int		aqc_fw_copy_mac_from_efuse(struct aqc_softc *, uint32_t,
		    uint8_t *);
int		aqc_fw_read_mbox(struct aqc_softc *, struct aqc_fw_mbox *);

inline int
aqc_fw_get_permanent_mac(struct aqc_softc *softc)
{

	return (softc->fw_ops->get_permanent_mac(softc));
}

inline int
aqc_fw_set_state(struct aqc_softc *softc, enum aqc_fw_state state)
{

	return (softc->fw_ops->set_state(softc, state));
}

inline int
aqc_fw_set_link_speed(struct aqc_softc *softc, uint32_t speed)
{

	return (softc->fw_ops->set_link_speed(softc, speed));
}

inline uint32_t
aqc_fw_get_link_speed(struct aqc_softc *softc)
{

	return (softc->fw_ops->get_link_speed(softc));
}

#endif	/* _AQC_FW_H_ */