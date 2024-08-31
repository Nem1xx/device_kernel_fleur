// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017-2018 Samsung Electronics Co., Ltd.
 */

#ifndef _UFSTW_H_
#define _UFSTW_H_

#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/blktrace_api.h>
#include <linux/blkdev.h>
#include <scsi/scsi_cmnd.h>

#include "../../../block/blk.h"

#define UFSTW_VER					0x0110
#define UFSTW_DD_VER					0x010300
#define UFSTW_DD_VER_POST				UFSFEATURE_DD_VER_POST

#define UFSTW_LIFETIME_SECT				2097152 /* 1GB */
#define UFSTW_MAX_LIFETIME_VALUE			0x0B
#define MASK_UFSTW_LIFETIME_NOT_GUARANTEE		0x80
#define UFS_FEATURE_SUPPORT_TW_BIT			0x100

#define TW_LU_SHARED					-1

enum UFSTW_STATE {
	TW_NEED_INIT = 0,
	TW_PRESENT = 1,
	TW_RESET = -3,
	TW_FAILED = -4
};

enum {
	TW_BUF_TYPE_LU = 0,
	TW_BUF_TYPE_SHARED,
};

struct ufstw_dev_info {
	/* from Device Descriptor */
	u16 tw_ver;
	u32 tw_shared_buf_alloc_units;
	u8 tw_buf_no_reduct;
	u8 tw_buf_type;

	/* from Geometry Descriptor */
	u8 tw_number_lu;
#if defined(CONFIG_UFS_CHECK) && defined(CONFIG_FACTORY_BUILD)
	u32 seg_size;
	u8 unit_size;
#endif
};

struct ufstw_lu {
	struct ufsf_feature *ufsf;

	int lun;

	bool tw_enable;
	bool flush_enable;
	bool flush_during_hibern_enter;

	unsigned int flush_status;
	unsigned int available_buffer_size;
	unsigned int curr_buffer_size;

	unsigned int lifetime_est;
	spinlock_t lifetime_lock;
	u32 stat_write_sec;
	struct work_struct tw_lifetime_work;

	/* for sysfs */
	struct kobject kobj;
	struct mutex sysfs_lock;
	struct ufstw_sysfs_entry *sysfs_entries;
};

struct ufstw_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct ufstw_lu *tw, char *buf);
	ssize_t (*store)(struct ufstw_lu *tw, const char *buf, size_t count);
};

struct ufshcd_lrb;

int ufstw_get_state(struct ufsf_feature *ufsf);
void ufstw_set_state(struct ufsf_feature *ufsf, int state);
void ufstw_get_dev_info(struct ufsf_feature *ufsf, u8 *desc_buf);
void ufstw_get_geo_info(struct ufsf_feature *ufsf, u8 *geo_buf);
void ufstw_alloc_lu(struct ufsf_feature *ufsf, int lun, u8 *lu_buf);

void ufstw_prep_fn(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp);
void ufstw_init(struct ufsf_feature *ufsf);
void ufstw_remove(struct ufsf_feature *ufsf);
void ufstw_reset_host(struct ufsf_feature *ufsf);
void ufstw_reset(struct ufsf_feature *ufsf, bool resume);
#endif /* End of Header */
