// SPDX-License-Identifier: GPL-2.0
/*
<<<<<<< HEAD
 * Copyright (c) 2021 MediaTek Inc.
=======
 * Copyright (C) 2021 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
 */

#include "inc/tcpci_typec.h"
#ifdef CONFIG_USB_POWER_DELIVERY
#include "inc/tcpci_event.h"
#endif /* CONFIG_USB_POWER_DELIVERY */

/*
 * [BLOCK] TCPCI IRQ Handler
 */

static int tcpci_alert_cc_changed(struct tcpc_device *tcpc)
{
	return tcpc_typec_handle_cc_change(tcpc);
}

#ifdef CONFIG_TCPC_VSAFE0V_DETECT_IC
<<<<<<< HEAD

static inline int tcpci_alert_vsafe0v(struct tcpc_device *tcpc)
{
	tcpc_typec_handle_vsafe0v(tcpc);

=======
static inline int tcpci_alert_vsafe0v(struct tcpc_device *tcpc)
{
	tcpc_typec_handle_vsafe0v(tcpc);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
#ifdef CONFIG_USB_POWER_DELIVERY
#ifdef CONFIG_USB_PD_SAFE0V_DELAY
	tcpc_enable_timer(tcpc, PD_TIMER_VSAFE0V_DELAY);
#else
	pd_put_vbus_safe0v_event(tcpc);
#endif	/* CONFIG_USB_PD_SAFE0V_DELAY */
#endif	/* CONFIG_USB_POWER_DELIVERY */
	return 0;
}
#endif	/* CONFIG_TCPC_VSAFE0V_DETECT_IC */

static inline void tcpci_vbus_level_init_v10(
	struct tcpc_device *tcpc, uint16_t power_status)
{
	mutex_lock(&tcpc->access_lock);
<<<<<<< HEAD

	tcpc->vbus_level =
			power_status & TCPC_REG_POWER_STATUS_VBUS_PRES ?
			TCPC_VBUS_VALID : TCPC_VBUS_INVALID;

=======
	tcpc->vbus_level = power_status & TCPC_REG_POWER_STATUS_VBUS_PRES ?
			   TCPC_VBUS_VALID : TCPC_VBUS_INVALID;
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
#ifdef CONFIG_TCPC_VSAFE0V_DETECT_IC
	if (power_status & TCPC_REG_POWER_STATUS_EXT_VSAFE0V) {
		if (tcpc->vbus_level == TCPC_VBUS_INVALID)
			tcpc->vbus_level = TCPC_VBUS_SAFE0V;
		else
<<<<<<< HEAD
			TCPC_INFO("ps_confused: 0x%02x\n", power_status);
	}
#endif	/* CONFIG_TCPC_VSAFE0V_DETECT_IC */

	mutex_unlock(&tcpc->access_lock);
}

static inline void __tcpci_vbus_level_refresh(struct tcpc_device *tcpc)
{
	tcpc->vbus_level = tcpc->vbus_present ? TCPC_VBUS_VALID :
			       TCPC_VBUS_INVALID;

=======
			TCPC_INFO("ps_confused: 0x%04x\n", power_status);
	}
#endif	/* CONFIG_TCPC_VSAFE0V_DETECT_IC */
	mutex_unlock(&tcpc->access_lock);
}

static void __tcpci_vbus_level_refresh(struct tcpc_device *tcpc)
{
	tcpc->vbus_level = tcpc->vbus_present ? TCPC_VBUS_VALID :
						TCPC_VBUS_INVALID;
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
#ifdef CONFIG_TCPC_VSAFE0V_DETECT_IC
	if (tcpc->vbus_safe0v) {
		if (tcpc->vbus_level == TCPC_VBUS_INVALID)
			tcpc->vbus_level = TCPC_VBUS_SAFE0V;
		else
			TCPC_INFO("ps_confused: %d\n", tcpc->vbus_level);
	}
#endif	/* CONFIG_TCPC_VSAFE0V_DETECT_IC */
}

static inline void tcpci_vbus_level_refresh(struct tcpc_device *tcpc)
{
	mutex_lock(&tcpc->access_lock);
	__tcpci_vbus_level_refresh(tcpc);
	mutex_unlock(&tcpc->access_lock);
}

<<<<<<< HEAD
void tcpci_vbus_level_init(struct tcpc_device *tcpc, uint16_t status)
{
	if (tcpc->tcpc_flags & TCPC_FLAGS_ALERT_V10) {
		tcpci_vbus_level_init_v10(tcpc, status);
=======
void tcpci_vbus_level_init(struct tcpc_device *tcpc, uint16_t power_status)
{
	if (tcpc->tcpc_flags & TCPC_FLAGS_ALERT_V10) {
		tcpci_vbus_level_init_v10(tcpc, power_status);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
		return;
	}

	mutex_lock(&tcpc->access_lock);
<<<<<<< HEAD

	tcpc->vbus_present = status & TCPC_REG_POWER_STATUS_VBUS_PRES ?
				 true : false;
=======
	tcpc->vbus_present = !!(power_status & TCPC_REG_POWER_STATUS_VBUS_PRES);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
	__tcpci_vbus_level_refresh(tcpc);
	mutex_unlock(&tcpc->access_lock);
}

<<<<<<< HEAD
static inline int tcpci_alert_power_status_changed_v10(struct tcpc_device *tcpc)
=======
static int tcpci_vbus_level_changed(struct tcpc_device *tcpc)
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
{
	int rv = 0;

<<<<<<< HEAD
	rv = tcpci_get_power_status(tcpc, &power_status);
	if (rv < 0)
		return rv;

#ifdef CONFIG_USB_PD_DIRECT_CHARGE
	if (tcpc->pd_during_direct_charge && tcpc->vbus_level != 0)
		show_msg = false;
#endif	/* CONFIG_USB_PD_DIRECT_CHARGE */

	if (show_msg)
		TCPC_INFO("ps_change=%d\n", tcpc->vbus_level);

=======
	TCPC_INFO("ps_change=%d\n", tcpc->vbus_level);

>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
	rv = tcpc_typec_handle_ps_change(tcpc, tcpc->vbus_level);
	if (rv < 0)
		return rv;

#ifdef CONFIG_USB_POWER_DELIVERY
	pd_put_vbus_changed_event(tcpc, true);
#endif /* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_TCPC_VSAFE0V_DETECT_IC
	if (tcpc->vbus_level == TCPC_VBUS_SAFE0V)
		rv = tcpci_alert_vsafe0v(tcpc);
#endif	/* CONFIG_TCPC_VSAFE0V_DETECT_IC */

	return rv;
}

<<<<<<< HEAD
static inline int tcpci_vbus_level_changed(struct tcpc_device *tcpc)
{
	int rv = 0;
	bool show_msg = true;

#ifdef CONFIG_USB_PD_DIRECT_CHARGE
	if (tcpc->pd_during_direct_charge && tcpc->vbus_level != 0)
		show_msg = false;
#endif	/* CONFIG_USB_PD_DIRECT_CHARGE */

	if (show_msg)
		TCPC_INFO("ps_change=%d\n", tcpc->vbus_level);

	rv = tcpc_typec_handle_ps_change(tcpc, tcpc->vbus_level);
	if (rv < 0)
		return rv;

#ifdef CONFIG_USB_POWER_DELIVERY
	pd_put_vbus_changed_event(tcpc, true);
#endif /* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_TCPC_VSAFE0V_DETECT_IC
	if (tcpc->vbus_level == TCPC_VBUS_SAFE0V)
		rv = tcpci_alert_vsafe0v(tcpc);
#endif	/* CONFIG_TCPC_VSAFE0V_DETECT_IC */
	return rv;
}

static int tcpci_alert_power_status_changed(struct tcpc_device *tcpc)
{
	int rv = 0;
	uint16_t status = 0;

	if (tcpc->tcpc_flags & TCPC_FLAGS_ALERT_V10)
		return tcpci_alert_power_status_changed_v10(tcpc);

	rv = tcpci_get_power_status(tcpc, &status);
	if (rv < 0)
		return rv;

	tcpc->vbus_present = (status & TCPC_REG_POWER_STATUS_VBUS_PRES) ?
				 true : false;
	return rv;
}

#ifdef CONFIG_USB_POWER_DELIVERY
static int tcpci_alert_tx_success(struct tcpc_device *tcpc)
=======
static int tcpci_alert_power_status_changed(struct tcpc_device *tcpc)
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
{
	int rv = 0;
	uint16_t power_status = 0;

	rv = tcpci_get_power_status(tcpc, &power_status);
	if (rv < 0)
		return rv;

	if (tcpc->tcpc_flags & TCPC_FLAGS_ALERT_V10)
		return tcpci_vbus_level_changed(tcpc);

	return rv;
}

#ifdef CONFIG_USB_POWER_DELIVERY
static int tcpci_alert_tx_success(struct tcpc_device *tcpc)
{
	uint8_t tx_state = PD_TX_STATE_GOOD_CRC;
	struct pd_event evt = {
		.event_type = PD_EVT_CTRL_MSG,
		.msg = PD_CTRL_GOOD_CRC,
		.pd_msg = NULL,
	};

	mutex_lock(&tcpc->access_lock);
	tx_state = tcpc->pd_transmit_state;
	tcpc->pd_transmit_state = PD_TX_STATE_GOOD_CRC;
	mutex_unlock(&tcpc->access_lock);

	if (tx_state == PD_TX_STATE_WAIT_CRC_VDM)
		pd_put_vdm_event(tcpc, &evt, false);
	else
		pd_put_event(tcpc, &evt, false);

	return 0;
}

static int tcpci_alert_tx_failed(struct tcpc_device *tcpc)
{
	uint8_t tx_state = PD_TX_STATE_GOOD_CRC;

	mutex_lock(&tcpc->access_lock);
	tx_state = tcpc->pd_transmit_state;
	tcpc->pd_transmit_state = PD_TX_STATE_NO_GOOD_CRC;
	mutex_unlock(&tcpc->access_lock);

	if (tx_state == PD_TX_STATE_WAIT_CRC_VDM)
		vdm_put_hw_event(tcpc, PD_HW_TX_FAILED);
	else
		pd_put_hw_event(tcpc, PD_HW_TX_FAILED);

	return 0;
}

static int tcpci_alert_tx_discard(struct tcpc_device *tcpc)
{
	uint8_t tx_state = PD_TX_STATE_GOOD_CRC;
	bool retry_crc_discard =
		!!(tcpc->tcpc_flags & TCPC_FLAGS_RETRY_CRC_DISCARD);

<<<<<<< HEAD
	mutex_lock(&tcpc->access_lock);
	tx_state = tcpc->pd_transmit_state;
	tcpc->pd_transmit_state = PD_TX_STATE_DISCARD;
	mutex_unlock(&tcpc->access_lock);

	TCPC_INFO("Discard\n");
=======
	TCPC_INFO("Discard\n");

	mutex_lock(&tcpc->access_lock);
	tx_state = tcpc->pd_transmit_state;
	tcpc->pd_transmit_state = PD_TX_STATE_DISCARD;
	mutex_unlock(&tcpc->access_lock);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)

	if (tx_state == PD_TX_STATE_WAIT_CRC_VDM)
		pd_put_last_vdm_event(tcpc);
	else {
<<<<<<< HEAD
		retry_crc_discard =
			(tcpc->tcpc_flags &
					TCPC_FLAGS_RETRY_CRC_DISCARD) != 0;

=======
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
		if (retry_crc_discard) {
#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
			tcpc->pd_discard_pending = true;
			tcpc_enable_timer(tcpc, PD_TIMER_DISCARD);
#else
			TCPC_ERR("RETRY_CRC_DISCARD\n");
#endif	/* CONFIG_USB_PD_RETRY_CRC_DISCARD */
		} else {
			pd_put_hw_event(tcpc, PD_HW_TX_FAILED);
		}
	}

	return 0;
}

static int tcpci_alert_recv_msg(struct tcpc_device *tcpc)
{
	int rv = 0;
	struct pd_msg *pd_msg = NULL;
	enum tcpm_transmit_type type = TCPC_TX_SOP;
	uint32_t chip_id = 0;
	rv = tcpci_get_chip_id(tcpc, &chip_id);
	if (!rv && (SC2150A_DID == chip_id)) {
		tcpci_set_rx_enable(tcpc, PD_RX_CAP_PE_STARTUP);
	}

	pd_msg = pd_alloc_msg(tcpc);
	if (pd_msg == NULL) {
<<<<<<< HEAD
		tcpci_alert_status_clear(tcpc, TCPC_REG_ALERT_RX_MASK);
		return -EINVAL;
	}

	retval = tcpci_get_message(tcpc,
		pd_msg->payload, &pd_msg->msg_hdr, &type);
	if (retval < 0) {
		TCPC_INFO("recv_msg failed: %d\n", retval);
		pd_free_msg(tcpc, pd_msg);
		return retval;
=======
		rv = -EINVAL;
		goto out;
	}

	rv = tcpci_get_message(tcpc, pd_msg->payload, &pd_msg->msg_hdr, &type);
	if (rv < 0) {
		TCPC_INFO("recv_msg failed: %d\n", rv);
		pd_free_msg(tcpc, pd_msg);
		goto out;
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
	}
	
	if (!rv && (SC2150A_DID == chip_id)) {
		tcpci_set_rx_enable(tcpc, tcpc->pd_port.rx_cap);
	}
	pd_msg->frame_type = type;
	pd_put_pd_msg_event(tcpc, pd_msg);
out:
	tcpci_alert_status_clear(tcpc, TCPC_REG_ALERT_RX_MASK);

<<<<<<< HEAD
	pd_msg->frame_type = (uint8_t) type;
	pd_put_pd_msg_event(tcpc, pd_msg);
	return 0;
=======
	return rv;
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
}

static int tcpci_alert_rx_overflow(struct tcpc_device *tcpc)
{
<<<<<<< HEAD
	int rv;
	uint32_t alert_status;

	TCPC_INFO("RX_OVERFLOW\n");

	rv = tcpci_get_alert_status(tcpc, &alert_status);
	if (rv)
		return rv;

	if (alert_status & TCPC_REG_ALERT_RX_STATUS)
		return tcpci_alert_recv_msg(tcpc);

	return 0;
=======
	TCPC_INFO("RX_OVERFLOW\n");
	return tcpci_alert_recv_msg(tcpc);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
}

static int tcpci_alert_recv_hard_reset(struct tcpc_device *tcpc)
{
	TCPC_INFO("HardResetAlert\n");
	pd_put_recv_hard_reset_event(tcpc);
<<<<<<< HEAD
	tcpci_init_alert_mask(tcpc);
	return 0;
=======
	return tcpci_init_alert_mask(tcpc);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
}
#endif /* CONFIG_USB_POWER_DELIVERY */

static int tcpci_alert_vendor_defined(struct tcpc_device *tcpc)
<<<<<<< HEAD
{
	tcpci_alert_vendor_defined_handler(tcpc);
	return 0;
}

static int tcpci_alert_fault(struct tcpc_device *tcpc)
=======
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
{
	return tcpci_alert_vendor_defined_handler(tcpc);
}

<<<<<<< HEAD
	tcpci_get_fault_status(tcpc, &status);
	TCPC_INFO("FaultAlert=0x%x\n", status);
	tcpci_fault_status_clear(tcpc, status);
=======
static int tcpci_alert_fault(struct tcpc_device *tcpc)
{
	uint8_t fault_status = 0;

	tcpci_get_fault_status(tcpc, &fault_status);
	TCPC_INFO("FaultAlert=0x%02x\n", fault_status);
	tcpci_fault_status_clear(tcpc, fault_status);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
	return 0;
}

#ifdef CONFIG_TYPEC_CAP_LPM_WAKEUP_WATCHDOG
static int tcpci_alert_wakeup(struct tcpc_device *tcpc)
{
	if (tcpc->tcpc_flags & TCPC_FLAGS_LPM_WAKEUP_WATCHDOG) {
		TCPC_INFO("Wakeup\n");
<<<<<<< HEAD

=======
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
		if (tcpc->typec_remote_cc[0] == TYPEC_CC_DRP_TOGGLING)
			tcpc_enable_wakeup_timer(tcpc, true);
	}

	return 0;
}
#endif /* CONFIG_TYPEC_CAP_LPM_WAKEUP_WATCHDOG */

#ifdef CONFIG_TYPEC_CAP_RA_DETACH
static int tcpci_alert_ra_detach(struct tcpc_device *tcpc)
{
	if (tcpc->tcpc_flags & TCPC_FLAGS_CHECK_RA_DETACHE) {
<<<<<<< HEAD
		TCPC_DBG("RA_DETACH\n");

=======
		TCPC_INFO("RA_DETACH\n");
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
		if (tcpc->typec_remote_cc[0] == TYPEC_CC_DRP_TOGGLING)
			tcpc_typec_enter_lpm_again(tcpc);
	}

	return 0;
}
#endif /* CONFIG_TYPEC_CAP_RA_DETACH */

struct tcpci_alert_handler {
	uint32_t bit_mask;
	int (*handler)(struct tcpc_device *tcpc);
};

#define DECL_TCPCI_ALERT_HANDLER(xbit, xhandler) {\
	.bit_mask = 1 << xbit,\
	.handler = xhandler,\
}

static const struct tcpci_alert_handler tcpci_alert_handlers[] = {
	DECL_TCPCI_ALERT_HANDLER(15, tcpci_alert_vendor_defined),
#ifdef CONFIG_USB_POWER_DELIVERY
	DECL_TCPCI_ALERT_HANDLER(4, tcpci_alert_tx_failed),
	DECL_TCPCI_ALERT_HANDLER(5, tcpci_alert_tx_discard),
	DECL_TCPCI_ALERT_HANDLER(6, tcpci_alert_tx_success),
	DECL_TCPCI_ALERT_HANDLER(2, tcpci_alert_recv_msg),
	DECL_TCPCI_ALERT_HANDLER(7, NULL),
	DECL_TCPCI_ALERT_HANDLER(8, NULL),
	DECL_TCPCI_ALERT_HANDLER(3, tcpci_alert_recv_hard_reset),
	DECL_TCPCI_ALERT_HANDLER(10, tcpci_alert_rx_overflow),
#endif /* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_TYPEC_CAP_LPM_WAKEUP_WATCHDOG
	DECL_TCPCI_ALERT_HANDLER(16, tcpci_alert_wakeup),
#endif /* CONFIG_TYPEC_CAP_LPM_WAKEUP_WATCHDOG */

#ifdef CONFIG_TYPEC_CAP_RA_DETACH
	DECL_TCPCI_ALERT_HANDLER(21, tcpci_alert_ra_detach),
#endif /* CONFIG_TYPEC_CAP_RA_DETACH */

	DECL_TCPCI_ALERT_HANDLER(9, tcpci_alert_fault),
	DECL_TCPCI_ALERT_HANDLER(0, tcpci_alert_cc_changed),
	DECL_TCPCI_ALERT_HANDLER(1, tcpci_alert_power_status_changed),
};

#ifdef CONFIG_USB_POWER_DELIVERY
static inline bool tcpci_check_hard_reset_complete(
	struct tcpc_device *tcpc, uint32_t alert_status)
{
	if ((alert_status & TCPC_REG_ALERT_HRESET_SUCCESS)
			== TCPC_REG_ALERT_HRESET_SUCCESS) {
		pd_put_sent_hard_reset_event(tcpc);
		return true;
	}

	if (alert_status & TCPC_REG_ALERT_TX_DISCARDED) {
		TCPC_INFO("HResetFailed\n");
		tcpci_transmit(tcpc, TCPC_TX_HARD_RESET, 0, NULL);
<<<<<<< HEAD
		return false;
=======
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
	}

	return false;
}
#endif	/* CONFIG_USB_POWER_DELIVERY */
<<<<<<< HEAD
int tcpci_alert(struct tcpc_device *tcpc)
{
	int rv, i;
	uint32_t alert_status;
	uint32_t alert_mask;

	rv = tcpci_get_alert_status(tcpc, &alert_status);
	if (rv)
		return rv;

	rv = tcpci_get_alert_mask(tcpc, &alert_mask);
	if (rv)
		return rv;

	/* mask all alert */
	rv = tcpci_set_alert_mask(tcpc, 0);
	if (rv)
		return rv;

#ifdef CONFIG_USB_PD_DBG_ALERT_STATUS
	if (alert_status != 0)
		TCPC_INFO("Alert:0x%04x, Mask:0x%04x\n",
			  alert_status, alert_mask);
=======

int tcpci_alert(struct tcpc_device *tcpc)
{
	int rv = 0, i = 0;
	uint32_t alert_status = 0, alert_mask = 0;
	const uint8_t typec_role = tcpc->typec_role;
	uint32_t chip_id;

	rv = tcpci_get_alert_status(tcpc, &alert_status);
	if (rv < 0)
		return rv;

	rv = tcpci_get_alert_mask(tcpc, &alert_mask);
	if (rv < 0)
		return rv;

#ifdef CONFIG_USB_PD_DBG_ALERT_STATUS
	TCPC_INFO("Alert:0x%04x, Mask:0x%04x\n", alert_status, alert_mask);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
#endif /* CONFIG_USB_PD_DBG_ALERT_STATUS */

	rv = tcpci_get_chip_id(tcpc,&chip_id);
  	if (rv || SC2150A_DID != chip_id)
          alert_status &= alert_mask;

<<<<<<< HEAD
	tcpci_alert_status_clear(tcpc,
		alert_status & (~TCPC_REG_ALERT_RX_MASK));

	if (tcpc->typec_role == TYPEC_ROLE_UNKNOWN)
=======
	if (typec_role == TYPEC_ROLE_UNKNOWN ||
		typec_role >= TYPEC_ROLE_NR) {
		TYPEC_INFO("Wrong TypeC-Role: %d\n", typec_role);
		tcpci_alert_status_clear(tcpc, alert_status);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
		return 0;
	}

<<<<<<< HEAD
=======
	/* mask all alert */
	rv = tcpci_set_alert_mask(tcpc, 0);
	if (rv < 0) {
		tcpci_alert_status_clear(tcpc, alert_status);
		return rv;
	}

	tcpci_alert_status_clear(tcpc, alert_status & ~TCPC_REG_ALERT_RX_MASK);

>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
	if ((tcpc->tcpc_flags & TCPC_FLAGS_ALERT_V10) &&
	    (alert_status & TCPC_REG_ALERT_EXT_VBUS_80))
		alert_status |= TCPC_REG_ALERT_POWER_STATUS;

#ifdef CONFIG_USB_POWER_DELIVERY
	if (tcpc->pd_transmit_state == PD_TX_STATE_WAIT_HARD_RESET) {
		tcpci_check_hard_reset_complete(tcpc, alert_status);
		alert_status &= ~TCPC_REG_ALERT_TX_MASK;
	}
#endif	/* CONFIG_USB_POWER_DELIVERY */

#ifndef CONFIG_USB_PD_DBG_SKIP_ALERT_HANDLER
	for (i = 0; i < ARRAY_SIZE(tcpci_alert_handlers); i++) {
		if (tcpci_alert_handlers[i].bit_mask & alert_status) {
<<<<<<< HEAD
			if (tcpci_alert_handlers[i].handler != 0)
=======
			if (tcpci_alert_handlers[i].handler)
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
				tcpci_alert_handlers[i].handler(tcpc);
		}
	}
#endif /* CONFIG_USB_PD_DBG_SKIP_ALERT_HANDLER */

	/* unmask alert */
	rv = tcpci_set_alert_mask(tcpc, alert_mask);
<<<<<<< HEAD
	if (rv)
		return rv;

	if (tcpc->tcpc_flags & TCPC_FLAGS_ALERT_V10)
		return 0;

	tcpci_vbus_level_refresh(tcpc);
	tcpci_vbus_level_changed(tcpc);
	return 0;
}

=======

	if (tcpc->tcpc_flags & TCPC_FLAGS_ALERT_V10)
		return rv;

	tcpci_vbus_level_refresh(tcpc);
	tcpci_vbus_level_changed(tcpc);

	return rv;
}

>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
/*
 * [BLOCK] TYPEC device changed
 */

static inline int tcpci_set_wake_lock(struct tcpc_device *tcpc, bool pd_lock)
{
<<<<<<< HEAD
#ifdef CONFIG_TCPC_ATTACH_WAKE_LOCK_TOUT
	__pm_wakeup_event(tcpc->attach_wake_lock,
		CONFIG_TCPC_ATTACH_WAKE_LOCK_TOUT);
#else
	__pm_stay_awake(tcpc->attach_wake_lock);
#endif	/* CONFIG_TCPC_ATTACH_WAKE_LOCK_TOUT */
}

int tcpci_set_wake_lock(
	struct tcpc_device *tcpc, bool pd_lock, bool user_lock)
{
	bool ori_lock, new_lock;

	if (tcpc->wake_lock_pd && tcpc->wake_lock_user)
		ori_lock = true;
	else
		ori_lock = false;

	if (pd_lock && user_lock)
		new_lock = true;
	else
		new_lock = false;

	if (new_lock != ori_lock) {
		if (new_lock) {
			TCPC_DBG("wake_lock=1\n");
			tcpci_attach_wake_lock(tcpc);
=======
	if (!!pd_lock != !!tcpc->wake_lock_pd) {
		if (pd_lock) {
			TCPC_DBG("wake_lock=1\n");
			__pm_wakeup_event(tcpc->attach_wake_lock,
					  CONFIG_TCPC_ATTACH_WAKE_LOCK_TOUT);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
			if (tcpc->typec_watchdog)
				tcpci_set_intrst(tcpc, true);
		} else {
			TCPC_DBG("wake_lock=0\n");
			if (tcpc->typec_watchdog)
				tcpci_set_intrst(tcpc, false);
			__pm_relax(tcpc->attach_wake_lock);
		}
		return 1;
	}

	return 0;
}

static int tcpci_set_wake_lock_pd(struct tcpc_device *tcpc, bool pd_lock)
{
	uint8_t wake_lock_pd = 0;

	mutex_lock(&tcpc->access_lock);

	wake_lock_pd = tcpc->wake_lock_pd;
	if (pd_lock)
		wake_lock_pd++;
	else if (wake_lock_pd > 0)
		wake_lock_pd--;

	if (wake_lock_pd == 0)
		__pm_wakeup_event(tcpc->detach_wake_lock, 5000);

	tcpci_set_wake_lock(tcpc, wake_lock_pd);
	tcpc->wake_lock_pd = wake_lock_pd;

	if (wake_lock_pd == 1)
		__pm_relax(tcpc->detach_wake_lock);

	mutex_unlock(&tcpc->access_lock);

	return 0;
}

static inline int tcpci_report_usb_port_attached(struct tcpc_device *tcpc)
{
	TCPC_INFO("usb_port_attached\n");

	tcpci_set_wake_lock_pd(tcpc, true);

#ifdef CONFIG_USB_POWER_DELIVERY
#ifdef CONFIG_USB_PD_DISABLE_PE
	if (tcpc->disable_pe)
		return 0;
#endif	/* CONFIG_USB_PD_DISABLE_PE */

	/* MTK Only */
	if (tcpc->pd_inited_flag) {
#ifdef CONFIG_USB_PD_WAIT_BC12
		if (tcpc->typec_attach_new == TYPEC_ATTACHED_SNK)
			tcpc_enable_timer(tcpc, TYPEC_RT_TIMER_PD_WAIT_BC12);
		else
			pd_put_cc_attached_event(tcpc, tcpc->typec_attach_new);
#else
		pd_put_cc_attached_event(tcpc, tcpc->typec_attach_new);
#endif
	}
#endif /* CONFIG_USB_POWER_DLEIVERY */

	return 0;
}

static inline int tcpci_report_usb_port_detached(struct tcpc_device *tcpc)
{
	TCPC_INFO("usb_port_detached\n");

#ifdef CONFIG_USB_POWER_DELIVERY
	/* MTK Only */
	if (tcpc->pd_inited_flag)
		pd_put_cc_detached_event(tcpc);
	else {
		pd_event_buf_reset(tcpc);
		tcpc_enable_timer(tcpc, TYPEC_RT_TIMER_PE_IDLE);
	}
#endif /* CONFIG_USB_POWER_DELIVERY */

	tcpci_set_wake_lock_pd(tcpc, false);

	return 0;
}

int tcpci_report_usb_port_changed(struct tcpc_device *tcpc)
{
	tcpci_notify_typec_state(tcpc);

	if (tcpc->typec_attach_old == TYPEC_UNATTACHED)
		tcpci_report_usb_port_attached(tcpc);
	else if (tcpc->typec_attach_new == TYPEC_UNATTACHED)
		tcpci_report_usb_port_detached(tcpc);
	else
		TCPC_DBG2("TCPC Attach Again\n");

	return 0;
}

/*
 * [BLOCK] TYPEC power control changed
 */

static inline int tcpci_report_power_control_on(struct tcpc_device *tcpc)
{
	tcpci_set_wake_lock_pd(tcpc, true);

	mutex_lock(&tcpc->access_lock);
	tcpc_disable_timer(tcpc, TYPEC_RT_TIMER_DISCHARGE);
	tcpci_enable_auto_discharge(tcpc, true);
	tcpci_enable_force_discharge(tcpc, false, 0);
	mutex_unlock(&tcpc->access_lock);

	return 0;
}

static inline int tcpci_report_power_control_off(struct tcpc_device *tcpc)
{
	mutex_lock(&tcpc->access_lock);
	tcpci_enable_force_discharge(tcpc, true, 0);
	tcpc_enable_timer(tcpc, TYPEC_RT_TIMER_DISCHARGE);
	mutex_unlock(&tcpc->access_lock);

	tcpci_set_wake_lock_pd(tcpc, false);

	return 0;
}

int tcpci_report_power_control(struct tcpc_device *tcpc, bool en)
{
	if (tcpc->typec_power_ctrl == en)
		return 0;

	tcpc->typec_power_ctrl = en;

	if (en)
		tcpci_report_power_control_on(tcpc);
	else
		tcpci_report_power_control_off(tcpc);

	return 0;
}
