/*
 * rxp.h - Public API definitions for the Titan IC Systems RXP
 *
 * Copyright 2017-2019 Titan IC Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _RXP_H_
#define _RXP_H_

#ifndef __KERNEL__
#include <stdint.h>
#endif

/*
 * Note: RXP Hardware for Mellanox BF2 is v5.7.  However aiming to keep with
 *       latest Software API code.
 */
#define RXP_API_VERSION "5.8.3"

#define RXP_MAX_JOB_LENGTH			16384

#define RXP_CTRL_TYPE_MASK			7
#define RXP_CTRL_TYPE_JOB_DESCRIPTOR		0
#define RXP_CTRL_TYPE_RESPONSE_DESCRIPTOR	1
#define RXP_CTRL_TYPE_MEMORY_WRITE		4

#define RXP_CTRL_JOB_DESC_SOF			0x0010
#define RXP_CTRL_JOB_DESC_EOF			0x0020
#define RXP_CTRL_JOB_DESC_HPM_ENABLE		0x0100
#define RXP_CTRL_JOB_DESC_ANYMATCH_ENABLE	0x0200
#define RXP_CTRL_JOB_DESC_FLAGS			(RXP_CTRL_JOB_DESC_SOF | \
						 RXP_CTRL_JOB_DESC_EOF | \
						 RXP_CTRL_JOB_DESC_HPM_ENABLE | \
						 RXP_CTRL_JOB_DESC_ANYMATCH_ENABLE)

#define RXP_CTRL_VALID				0x8000

#define RXP_RESP_STATUS_MAX_PRI_THREADS		(1 << 3)
#define RXP_RESP_STATUS_MAX_SEC_THREADS		(1 << 4)
#define RXP_RESP_STATUS_MAX_LATENCY		(1 << 5)
#define RXP_RESP_STATUS_MAX_MATCH		(1 << 6)
#define RXP_RESP_STATUS_MAX_PREFIX		(1 << 7)
#define RXP_RESP_STATUS_HPM			(1 << 8)
#define RXP_RESP_STATUS_ANYMATCH		(1 << 9)
#define RXP_RESP_STATUS_PMI_SOJ			(1 << 13)
#define RXP_RESP_STATUS_PMI_EOJ			(1 << 14)

#define RXP_SUBSET_ID_MAX			65535

/*
 * This describes the header the RXP expects for any search data.
 */
struct rxp_job_desc {
	uint32_t	job_id;
	uint16_t	ctrl;
	uint16_t	len;
	uint16_t	subset[4];
}  __attribute__ ((__packed__));

struct rxp_response_desc {
	uint32_t	job_id;
	uint16_t	status;
	uint8_t		detected_match_count;
	uint8_t		match_count;
	uint16_t	primary_thread_count;
	uint16_t	instruction_count;
	uint16_t	latency_count;
	uint16_t	pmi_min_byte_ptr;
}  __attribute__ ((__packed__));

//struct rxp_match_tuple {
//	uint32_t	job_id;
//	uint32_t	rule_id;
//	uint32_t	start_ptr;
//	uint32_t	length;
//}  __attribute__ ((__packed__));

//New 64bit match tuple for Mellanox
struct rxp_match_tuple {
	uint32_t	rule_id;
	uint16_t	start_ptr;
	uint16_t	length;
}  __attribute__ ((__packed__));

struct rxp_response {
	struct rxp_response_desc header;
	struct rxp_match_tuple matches[0];
};

#define RXP_MAX_MATCHES 254

#define RXP_CTL_RULES_PGM	1
#define RXP_CTL_RULES_PGM_INCR	2

#define RXP_ROF_ENTRY_INST            0
#define RXP_ROF_ENTRY_EQ              1
#define RXP_ROF_ENTRY_GTE             2
#define RXP_ROF_ENTRY_LTE             3
#define RXP_ROF_ENTRY_CHECKSUM        4
#define RXP_ROF_ENTRY_CHECKSUM_EX_EM  5
#define RXP_ROF_ENTRY_IM              6
#define RXP_ROF_ENTRY_EM              7
#define RXP_ROF_ENTRY_TYPE_MAX        7

struct rxp_ctl_hdr {
	uint16_t	cmd;
	uint32_t	len;
};

struct rxp_rof_entry {
	uint8_t		type;
	uint32_t	addr;
	uint64_t	value;
};

struct rxp_rof {
	uint32_t rof_version;
	char *timestamp;
	char *rxp_compiler_version;
	uint32_t rof_revision;
	uint32_t number_of_entries;
	struct rxp_rof_entry *rof_entries;
};

struct rxp_ctl_rules_pgm {
	struct rxp_ctl_hdr	hdr;
	uint32_t		count;
	struct rxp_rof_entry	rules[0];
} __attribute__ ((__packed__));

#endif /* _RXP_H_ */
