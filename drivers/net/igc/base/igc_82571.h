/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2001-2019
 */

#ifndef _IGC_82571_H_
#define _IGC_82571_H_

#define ID_LED_RESERVED_F746	0xF746
#define ID_LED_DEFAULT_82573	((ID_LED_DEF1_DEF2 << 12) | \
				 (ID_LED_OFF1_ON2  <<  8) | \
				 (ID_LED_DEF1_DEF2 <<  4) | \
				 (ID_LED_DEF1_DEF2))

#define IGC_GCR_L1_ACT_WITHOUT_L0S_RX	0x08000000
#define AN_RETRY_COUNT		5 /* Autoneg Retry Count value */

/* Intr Throttling - RW */
#define IGC_EITR_82574(_n)	(0x000E8 + (0x4 * (_n)))

#define IGC_EIAC_82574	0x000DC /* Ext. Interrupt Auto Clear - RW */
#define IGC_EIAC_MASK_82574	0x01F00000

#define IGC_IVAR_INT_ALLOC_VALID	0x8

/* Manageability Operation Mode mask */
#define IGC_NVM_INIT_CTRL2_MNGM	0x6000

#define IGC_BASE1000T_STATUS		10
#define IGC_IDLE_ERROR_COUNT_MASK	0xFF
#define IGC_RECEIVE_ERROR_COUNTER	21
#define IGC_RECEIVE_ERROR_MAX		0xFFFF
bool igc_check_phy_82574(struct igc_hw *hw);
bool igc_get_laa_state_82571(struct igc_hw *hw);
void igc_set_laa_state_82571(struct igc_hw *hw, bool state);

#endif