/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2016 MediaTek Inc.
 */
#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>

#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/sched.h>
#include <linux/mm.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <mt-plat/sync_write.h>

#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include "i2c-mtk.h"

#include "mtk_ion.h"
#include "ion_drv.h"
#include <linux/iommu.h>


#ifdef CONFIG_MTK_IOMMU
#include "mtk_iommu.h"
#include <dt-bindings/memory/mt6779-larb-port.h>
#else
#include "m4u.h"
#endif


#include <linux/clk.h>

#include "ccu_drv.h"
#include "ccu_cmn.h"
#include "ccu_reg.h"
#include "ccu_platform_def.h"
#include "ccu_i2c.h"
#include "ccu_i2c_hw.h"
#include "ccu_imgsensor.h"
#include "kd_camera_feature.h"/*for IMGSENSOR_SENSOR_IDX*/
#include "ccu_mva.h"
#include "ccu_qos.h"
#include "ccu_ipc.h"
//for mmdvfs
#include <linux/soc/mediatek/mtk-pm-qos.h>
#include <mmdvfs_pmqos.h>
#define CONFIG_MTK_QOS_SUPPORT_ENABLE
/***************************************************************************
 *
 **************************************************************************/

#define CCU_DEV_NAME            "ccu"

#define CCU_CLK_NUM 4 /* [0]: Camsys, [1]: Mmsys, [2]: TopMux */

static int32_t _user_count;

struct clk *ccu_clk_ctrl[CCU_CLK_NUM];

struct ccu_device_s *g_ccu_device;
static struct ccu_power_s power;
static uint32_t ccu_hw_base;

static wait_queue_head_t wait_queue_deque;
static wait_queue_head_t wait_queue_enque;

static struct ion_handle
	*import_buffer_handle[CCU_IMPORT_BUF_NUM];

#ifdef CONFIG_PM_SLEEP
struct wakeup_source ccu_wake_lock;
#endif
/*static int g_bWaitLock;*/

static irqreturn_t ccu_isr_callback_xxx(int rrq,
					void *device_id);

typedef irqreturn_t(*ccu_isr_fp_t) (int, void *);

struct ccu_isr_callback_t {
	ccu_isr_fp_t irq_fp;
	unsigned int int_number;
	char device_name[16];
};

/* int number is got from kernel api */
const struct ccu_isr_callback_t
	ccu_isr_callbacks[CCU_IRQ_NUM_TYPES] = {
	/* The last used be mapping to device node.
	 * Must be the same name with that in device node.
	 */
	{ccu_isr_callback_xxx, 0, "ccu2"}
};

static irqreturn_t ccu_isr_callback_xxx(int irq, void *device_id)
{
	LOG_DBG("%s:0x%x\n", __func__, irq);
	return IRQ_HANDLED;
}

static struct mtk_pm_qos_request _ccu_qos_request;
static u64 _g_freq_steps[MAX_FREQ_STEP];
#ifdef CONFIG_MTK_QOS_SUPPORT_ENABLE
static u32 _step_size;
#endif
static int ccu_probe(struct platform_device *dev);

static int ccu_remove(struct platform_device *dev);

static int ccu_suspend(struct platform_device *dev,
		       pm_message_t mesg);

static int ccu_resume(struct platform_device *dev);

/*-------------------------------------------------------------------------*/
/* CCU Driver: pm operations                                               */
/*-------------------------------------------------------------------------*/
#ifdef CONFIG_PM
int ccu_pm_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	WARN_ON(pdev == NULL);

	return ccu_suspend(pdev, PMSG_SUSPEND);
}

int ccu_pm_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	WARN_ON(pdev == NULL);

	return ccu_resume(pdev);
}

/* extern void mt_irq_set_sens(unsigned int irq, unsigned int sens); */
/* extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity); */
int ccu_pm_restore_noirq(struct device *device)
{
#ifndef CONFIG_OF
	mt_irq_set_sens(CAM0_IRQ_BIT_ID, MT_LEVEL_SENSITIVE);
	mt_irq_set_polarity(CAM0_IRQ_BIT_ID, MT_POLARITY_LOW);
#endif
	return 0;
}
#else
#define ccu_pm_suspend NULL
#define ccu_pm_resume  NULL
#define ccu_pm_restore_noirq NULL
#endif

const struct dev_pm_ops ccu_pm_ops = {
	.suspend = ccu_pm_suspend,
	.resume = ccu_pm_resume,
	.freeze = ccu_pm_suspend,
	.thaw = ccu_pm_resume,
	.poweroff = ccu_pm_suspend,
	.restore = ccu_pm_resume,
	.restore_noirq = ccu_pm_restore_noirq,
};


/*---------------------------------------------------------------------------*/
/* CCU Driver: Prototype                                                     */
/*---------------------------------------------------------------------------*/

static const struct of_device_id ccu_of_ids[] = {
	{.compatible = "mediatek,ccu",},
	{.compatible = "mediatek,ccu_camsys",},
	{.compatible = "mediatek,n3d_ctl_a",},
	{}
};

static struct platform_driver ccu_driver = {
	.probe = ccu_probe,
	.remove = ccu_remove,
	.suspend = ccu_suspend,
	.resume = ccu_resume,
	.driver = {
		.name = CCU_DEV_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = ccu_of_ids,
#endif
#ifdef CONFIG_PM
		.pm = &ccu_pm_ops,
#endif
	}
};


/*---------------------------------------------------------------------------*/
/* CCU Driver: file operations                                               */
/*---------------------------------------------------------------------------*/
static int ccu_open(struct inode *inode, struct file *flip);

static int ccu_release(struct inode *inode, struct file *flip);


static long ccu_ioctl(struct file *flip, unsigned int cmd,
		      unsigned long arg);

#ifdef CONFIG_COMPAT
static long ccu_compat_ioctl(struct file *flip, unsigned int cmd,
			     unsigned long arg);
#endif

static const struct file_operations ccu_fops = {
	.owner = THIS_MODULE,
	.open = ccu_open,
	.release = ccu_release,
	.unlocked_ioctl = ccu_ioctl,
#ifdef CONFIG_COMPAT
	/*for 32bit usersapce program doing ioctl, compat_ioctl will be called*/
	.compat_ioctl = ccu_compat_ioctl
#endif
};

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static int ccu_num_users;

int ccu_create_user(struct ccu_user_s **user)
{
	struct ccu_user_s *u;

	LOG_INF_MUST("%s+\n",
		__func__);

	u = kmalloc(sizeof(vlist_type(struct ccu_user_s)), GFP_ATOMIC);
	if (!u)
		return -1;

	mutex_init(&u->data_mutex);
	/*mutex_lock(&u->data_mutex);*/
	u->id = ++ccu_num_users;
	u->open_pid = current->pid;
	u->open_tgid = current->tgid;
	u->running = false;
	u->flush = false;
	INIT_LIST_HEAD(&u->enque_ccu_cmd_list);
	INIT_LIST_HEAD(&u->deque_ccu_cmd_list);
	init_waitqueue_head(&u->deque_wait);
	/*mutex_unlock(&u->data_mutex);*/

	mutex_lock(&g_ccu_device->user_mutex);
	list_add_tail(vlist_link(u, struct ccu_user_s),
		&g_ccu_device->user_list);
	mutex_unlock(&g_ccu_device->user_mutex);

	LOG_INF_MUST("%s-\n",
		__func__);

	*user = u;
	return 0;
}

int ccu_delete_user(struct ccu_user_s *user)
{

	if (!user) {
		LOG_ERR("delete empty user!\n");
		return -1;
	}

	mutex_lock(&g_ccu_device->user_mutex);
	list_del(vlist_link(user, struct ccu_user_s));
	mutex_unlock(&g_ccu_device->user_mutex);

	kfree(user);

	return 0;
}

int ccu_lock_ion_client_mutex(void)
{
	mutex_lock(&g_ccu_device->ion_client_mutex);
	return 0;
}

int ccu_unlock_ion_client_mutex(void)
{
	mutex_unlock(&g_ccu_device->ion_client_mutex);
	return 0;
}

/*---------------------------------------------------------------------------*/
/* IOCTL: implementation                                                     */
/*---------------------------------------------------------------------------*/
int ccu_set_power(struct ccu_power_s *power)
{
	return ccu_power(power);
}

static int ccu_open(struct inode *inode, struct file *flip)
{
	int ret = 0, i;

<<<<<<< HEAD
	struct ccu_user_s *user = NULL;
	struct CcuMemHandle handle = {0};

	mutex_lock(&g_ccu_device->dev_mutex);

	LOG_INF_MUST("%s pid:%d tid:%d cnt:%d+\n",
		__func__, current->pid, current->tgid, _user_count);

	ret = ccu_create_user(&user);
=======
	struct ccu_user_s *user;

	mutex_lock(&g_ccu_device->dev_mutex);

	_clk_count = 0;
	ccu_create_user(&user);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
	if (IS_ERR_OR_NULL(user)) {
		LOG_ERR("fail to create user\n");
		mutex_unlock(&g_ccu_device->dev_mutex);
		return -ENOMEM;
	}

	flip->private_data = user;
	_user_count++;

	if (_user_count > 1) {
		LOG_INF_MUST("%s clean legacy data flow-\n", __func__);
		ccu_force_powerdown();

		for (i = 0; i < CCU_IMPORT_BUF_NUM; i++) {
			if (import_buffer_handle[i] == (struct ion_handle *)
			    CCU_IMPORT_BUF_UNDEF) {
				LOG_INF_MUST("freed buffer count: %d\n", i);
				break;
			}

			ccu_ion_free_import_handle(
				import_buffer_handle[i]);/*can't in spin_lock*/
		}

		handle.meminfo.cached = 0;
		ccu_deallocate_mem(&handle);
		handle.meminfo.cached = 1;
		ccu_deallocate_mem(&handle);

		ccu_ion_uninit();
	}

	ccu_ion_init();

	for (i = 0; i < CCU_IMPORT_BUF_NUM; i++)
		import_buffer_handle[i] = (struct ion_handle *)
					  CCU_IMPORT_BUF_UNDEF;

	LOG_INF_MUST("%s-\n",
		__func__);

	mutex_unlock(&g_ccu_device->dev_mutex);

	mutex_unlock(&g_ccu_device->dev_mutex);

	return ret;
}

#ifdef CONFIG_COMPAT
static long ccu_compat_ioctl(struct file *flip, unsigned int cmd,
			     unsigned long arg)
{
	/*<<<<<<<<<< debug 32/64 compat check*/
	struct compat_ccu_power_s __user *ptr_power32;
	struct ccu_power_s __user *ptr_power64;

	compat_uptr_t uptr_Addr32;
	compat_uint_t uint_Data32;

	int err;
	int i;
	/*>>>>>>>>>> debug 32/64 compat check*/

	int ret = 0;
	struct ccu_user_s *user = flip->private_data;

#define CCU_ASSERT(condition, fmt, args...) { \
			if (!(condition)) { \
				LOG_ERR(fmt, ##args);\
				return -EFAULT; \
			} \
		}

	LOG_DBG("+, cmd: %d\n", cmd);

	switch (cmd) {
	case CCU_IOCTL_SET_POWER: {
		LOG_DBG("CCU_IOCTL_SET_POWER+\n");

		/*<<<<<<<<<< debug 32/64 compat check*/
		LOG_DBG("[IOCTL_DBG] struct ccu_power_s size: %zu\n",
			sizeof(struct ccu_power_s));
		LOG_DBG("[IOCTL_DBG] struct ccu_working_buffer_s size: %zu\n",
			sizeof(struct ccu_working_buffer_s));
		LOG_DBG("[IOCTL_DBG] arg: %p\n", (void *)arg);
		LOG_DBG("[IOCTL_DBG] long size: %zu\n", sizeof(long));
		LOG_DBG("[IOCTL_DBG] long long: %zu\n", sizeof(long long));
		LOG_DBG("[IOCTL_DBG] char *size: %zu\n", sizeof(char *));
		LOG_DBG("[IOCTL_DBG] power.workBuf.va_log[0]: %p\n",
			power.workBuf.va_log[0]);

		ptr_power32 = compat_ptr(arg);
		ptr_power64 = compat_alloc_user_space(sizeof(*ptr_power64));
		if (ptr_power64 == NULL)
			return -EFAULT;

		LOG_DBG("[IOCTL_DBG] (void *)arg: %p\n", (void *)arg);
		LOG_DBG("[IOCTL_DBG] ptr_power32: %p\n", ptr_power32);
		LOG_DBG("[IOCTL_DBG] ptr_power64: %p\n", ptr_power64);
		LOG_DBG("[IOCTL_DBG] *ptr_power32 size: %zu\n",
			sizeof(*ptr_power32));
		LOG_DBG("[IOCTL_DBG] *ptr_power64 size: %zu\n",
			sizeof(*ptr_power64));

		err = 0;
		err |= get_user(uint_Data32, &(ptr_power32->bON));
		err |= put_user(uint_Data32, &(ptr_power64->bON));

		for (i = 0; i < MAX_LOG_BUF_NUM; i++) {
			err |= get_user(uptr_Addr32,
					(&ptr_power32->workBuf.va_log[i]));
			err |=
				put_user(compat_ptr(uptr_Addr32),
					(&ptr_power64->workBuf.va_log[i]));
			err |= get_user(uint_Data32,
					(&ptr_power32->workBuf.mva_log[i]));

			err |=
				copy_to_user(&(ptr_power64->workBuf.mva_log[i]),
					&uint_Data32,
					sizeof(uint_Data32));
		}

		LOG_DBG("[IOCTL_DBG] err: %d\n", err);
		LOG_DBG("[IOCTL_DBG] ptr_power32->workBuf.va_pool: %x\n",
			ptr_power32->workBuf.va_pool);
		LOG_DBG("[IOCTL_DBG] ptr_power64->workBuf.va_pool: %p\n",
			ptr_power64->workBuf.va_pool);
		LOG_DBG("[IOCTL_DBG] ptr_power32->workBuf.va_log: %x\n",
			ptr_power32->workBuf.va_log[0]);
		LOG_DBG("[IOCTL_DBG] ptr_power64->workBuf.va_log: %p\n",
			ptr_power64->workBuf.va_log[0]);
		LOG_DBG("[IOCTL_DBG] ptr_power32->workBuf.mva_log: %x\n",
			ptr_power32->workBuf.mva_log[0]);
		LOG_DBG("[IOCTL_DBG] ptr_power64->workBuf.mva_log: %x\n",
			ptr_power64->workBuf.mva_log[0]);

		ret = flip->f_op->unlocked_ioctl(flip, cmd,
						 (unsigned long)ptr_power64);
		/*>>>>>>>>>> debug 32/64 compat check*/

		LOG_DBG("CCU_IOCTL_SET_POWER-");
		break;
	}
	default:
		ret = flip->f_op->unlocked_ioctl(flip, cmd, arg);
		break;
	}

	if (ret != 0) {
		LOG_ERR("fail, cmd(%d), pid(%d), (process, pid, tgid)=(%s, %d, %d)\n",
			cmd, user->open_pid, current->comm, current->pid, current->tgid);
	}
#undef CCU_ASSERT
	return ret;
}
#endif

int ccu_clock_enable(void)
{
	int ret;

	mutex_lock(&g_ccu_device->clk_mutex);

	LOG_DBG_MUST("%s(%d).\n",
		__func__, CCU_CLK_NUM);
	ccu_qos_init();
	ret = clk_prepare_enable(ccu_clk_ctrl[0]);
	if (ret)
		LOG_ERR("CCU_CLK_TOP_MUX enable fail.\n");
	ret = clk_prepare_enable(ccu_clk_ctrl[1]);
	if (ret)
		LOG_ERR("CAM_PWR enable fail.\n");
	ret = clk_prepare_enable(ccu_clk_ctrl[2]);
	if (ret)
		LOG_ERR("CCU_CLK_MMSYS_CCU enable fail.\n");
	ret = clk_prepare_enable(ccu_clk_ctrl[3]);
	if (ret)
		LOG_ERR("CCU_CLK_CAM_CCU enable fail.\n");

	mutex_unlock(&g_ccu_device->clk_mutex);

	if (ret)
		LOG_ERR("clock enable fail.\n");
	return ret;
}

void ccu_clock_disable(void)
{
	mutex_lock(&g_ccu_device->clk_mutex);

	LOG_DBG_MUST("%s.\n", __func__);
	clk_disable_unprepare(ccu_clk_ctrl[3]);
	clk_disable_unprepare(ccu_clk_ctrl[2]);
	clk_disable_unprepare(ccu_clk_ctrl[1]);
	clk_disable_unprepare(ccu_clk_ctrl[0]);

	ccu_qos_uninit();

	mutex_unlock(&g_ccu_device->clk_mutex);
}

static long ccu_ioctl(struct file *flip, unsigned int cmd,
		      unsigned long arg)
{
	int ret = 0;
	int i = 0;
	int powert_stat;
	struct CCU_WAIT_IRQ_STRUCT IrqInfo;
	struct ccu_user_s *user = flip->private_data;
	struct ccu_run_s ccu_run_info;

	if ((cmd != CCU_IOCTL_WAIT_IRQ) && (cmd != CCU_IOCTL_WAIT_AF_IRQ))
		mutex_lock(&g_ccu_device->dev_mutex);

	if ((cmd != CCU_IOCTL_WAIT_IRQ) && (cmd != CCU_IOCTL_WAIT_AF_IRQ))
		mutex_lock(&g_ccu_device->dev_mutex);

	LOG_DBG("%s+, cmd:%d\n", __func__, cmd);

	if ((cmd != CCU_IOCTL_SET_POWER) && (cmd != CCU_IOCTL_FLUSH_LOG) &&
		(cmd != CCU_IOCTL_WAIT_IRQ) && (cmd != CCU_IOCTL_IMPORT_MEM) &&
		(cmd != CCU_IOCTL_ALLOC_MEM) && (cmd != CCU_IOCTL_DEALLOC_MEM) &&
		(cmd != CCU_IOCTL_LOAD_CCU_BIN)) {
		powert_stat = ccu_query_power_status();
		if (powert_stat == 0) {
<<<<<<< HEAD
			LOG_WARN("ccuk: ioctl(%d) without powered on\n", cmd);
=======
			LOG_WARN("ccuk: ioctl without powered on\n");
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
			if (cmd != CCU_IOCTL_WAIT_AF_IRQ)
				mutex_unlock(&g_ccu_device->dev_mutex);
			return -EFAULT;
		}
	}

	switch (cmd) {
	case CCU_IOCTL_SET_POWER: {
		LOG_DBG("ccuk: ioctl set powerk+\n");
		ret = copy_from_user(&power, (void *)arg,
				     sizeof(struct ccu_power_s));
		if (ret != 0) {
			LOG_ERR(
			"[SET_POWER] copy_from_user failed, ret=%d\n", ret);
			mutex_unlock(&g_ccu_device->dev_mutex);
			return -EFAULT;
		}
		ret = ccu_set_power(&power);
		LOG_DBG("ccuk: ioctl set powerk-\n");
		break;
	}
	case CCU_IOCTL_SET_RUN_INPUT:
	{
		ret = copy_from_user(&ccu_run_info,
			(void *)arg, sizeof(struct ccu_run_s));
		if (ret != 0) {
			LOG_ERR(
			"CCU_IOCTL_SET_RUN_INPUT copy_from_user failed: %d\n",
			ret);
			break;
		}

		ret = ccu_run(&ccu_run_info);
		break;
	}
<<<<<<< HEAD
	case CCU_IOCTL_WAIT_IRQ: {
		if (copy_from_user(&IrqInfo, (void *)arg,
				   sizeof(struct CCU_WAIT_IRQ_STRUCT)) == 0) {
=======

	case CCU_IOCTL_ENQUE_COMMAND:
	{
		struct ccu_cmd_s *cmd = 0;

		/*allocate ccu_cmd_st_list instead of ccu_cmd_st*/
		ccu_alloc_command(&cmd);
		ret = copy_from_user(
			cmd, (void *)arg, sizeof(struct ccu_cmd_s));
		if (ret != 0) {
			LOG_ERR(
			"[ENQUE_COMMAND] copy_from_user failed, ret=%d\n", ret);
			mutex_unlock(&g_ccu_device->dev_mutex);
			return -EFAULT;
		}

		ret = ccu_push_command_to_queue(user, cmd);
		break;
	}

	case CCU_IOCTL_DEQUE_COMMAND:
	{
		struct ccu_cmd_s *cmd = 0;

		ret = ccu_pop_command_from_queue(user, &cmd);
		if (ret != 0) {
			LOG_ERR(
			"[DEQUE_COMMAND] pop command failed, ret=%d\n", ret);
			mutex_unlock(&g_ccu_device->dev_mutex);
			return -EFAULT;
		}
		ret = copy_to_user((void *)arg, cmd, sizeof(struct ccu_cmd_s));
		if (ret != 0) {
			LOG_ERR(
			"[DEQUE_COMMAND] copy_to_user failed, ret=%d\n", ret);
			mutex_unlock(&g_ccu_device->dev_mutex);
			return -EFAULT;
		}
		ret = ccu_free_command(cmd);
		if (ret != 0) {
			LOG_ERR(
			"[DEQUE_COMMAND] free command, ret=%d\n", ret);
			mutex_unlock(&g_ccu_device->dev_mutex);
			return -EFAULT;
		}

		break;
	}

	case CCU_IOCTL_FLUSH_COMMAND:
	{
		ret = ccu_flush_commands_from_queue(user);
		if (ret != 0) {
			LOG_ERR(
			"[FLUSH_COMMAND] flush command failed, ret=%d\n", ret);
			mutex_unlock(&g_ccu_device->dev_mutex);
			return -EFAULT;
		}

		break;
	}

	case CCU_IOCTL_WAIT_IRQ:
	{
		if (copy_from_user(&IrqInfo,
			(void *)arg, sizeof(struct CCU_WAIT_IRQ_STRUCT)) == 0) {
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
			if ((IrqInfo.Type >= CCU_IRQ_TYPE_AMOUNT)
			    || (IrqInfo.Type < 0)) {
				ret = -EFAULT;
				LOG_ERR("invalid type(%d)\n", IrqInfo.Type);
				goto EXIT;
			}

			LOG_DBG("IRQ type(%d), userKey(%d), timeout(%d), sttype(%d), st(%d)\n",
				IrqInfo.Type,
				IrqInfo.EventInfo.UserKey,
				IrqInfo.EventInfo.Timeout,
				IrqInfo.EventInfo.St_type,
				IrqInfo.EventInfo.Status);

			ret = ccu_waitirq(&IrqInfo);

			if (copy_to_user((void *)arg, &IrqInfo,
					 sizeof(struct CCU_WAIT_IRQ_STRUCT))
			    != 0) {
				LOG_ERR("copy_to_user failed\n");
				ret = -EFAULT;
			}
		} else {
			LOG_ERR("copy_from_user failed\n");
			ret = -EFAULT;
		}

		break;
	}
<<<<<<< HEAD
	case CCU_IOCTL_WAIT_AF_IRQ: {
		if (copy_from_user(&IrqInfo, (void *)arg,
				   sizeof(struct CCU_WAIT_IRQ_STRUCT)) == 0) {
			if ((IrqInfo.Type >= CCU_IRQ_TYPE_AMOUNT)
			    || (IrqInfo.Type < 0)) {
				ret = -EFAULT;
				LOG_ERR("invalid type(%d)\n", IrqInfo.Type);
				goto EXIT;
			}

			LOG_DBG("AFIRQ type(%d), userKey(%d), timeout(%d), sttype(%d), st(%d)\n",
				IrqInfo.Type,
				IrqInfo.EventInfo.UserKey,
				IrqInfo.EventInfo.Timeout,
				IrqInfo.EventInfo.St_type,
				IrqInfo.EventInfo.Status);

			if ((IrqInfo.bDumpReg >=
			     IMGSENSOR_SENSOR_IDX_MIN_NUM) &&
			    (IrqInfo.bDumpReg <
			     IMGSENSOR_SENSOR_IDX_MAX_NUM)) {
				ret = ccu_AFwaitirq(
					      &IrqInfo, IrqInfo.bDumpReg);
			} else {
				LOG_DBG_MUST(
					"unknown sensorIdx(%d)(CCU_IOCTL_WAIT_AF_IRQ)\n",
					IrqInfo.bDumpReg);
				ret = -EFAULT;
				goto EXIT;
			}

			if (copy_to_user((void *)arg, &IrqInfo,
					 sizeof(struct CCU_WAIT_IRQ_STRUCT))
			    != 0) {
				LOG_ERR("copy_to_user failed\n");
				ret = -EFAULT;
			}
		} else {
			LOG_ERR("copy_from_user failed\n");
			ret = -EFAULT;
		}

		break;
	}
	case CCU_IOCTL_FLUSH_LOG: {
=======

	case CCU_IOCTL_FLUSH_LOG:
	{
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
		ccu_flushLog(0, NULL);
		break;
	}

	case CCU_IOCTL_IPC_SEND_CMD:
	{
		struct ccu_control_info msg;
		uint32_t *indata = NULL;
		uint32_t *outdata = NULL;

		indata = kzalloc(CCU_IPC_IBUF_CAPACITY, GFP_KERNEL);
		outdata = kzalloc(CCU_IPC_OBUF_CAPACITY, GFP_KERNEL);
		ret = copy_from_user(&msg,
			(void *)arg, sizeof(struct ccu_control_info));
		if (ret != 0) {
			LOG_ERR(
			"CCU_IOCTL_IPC_SEND_CMD copy_from_user failed: %d\n",
			ret);
			kfree(indata);
			kfree(outdata);
			break;
		}
		if (msg.inDataSize > CCU_IPC_IBUF_CAPACITY) {
			LOG_ERR(
			"CCU_IOCTL_IPC_SEND_CMD copy_from_user 2 oversize\n");
			ret = -EINVAL;
			kfree(indata);
			kfree(outdata);
			break;
		}
		ret = copy_from_user(indata,
			(void *)msg.inDataPtr, msg.inDataSize);
		if (ret != 0) {
			LOG_ERR(
			"CCU_IOCTL_IPC_SEND_CMD copy_from_user 2 failed: %d\n",
			ret);
			kfree(indata);
			kfree(outdata);
			break;
		}

		ret = ccuControl(
		msg.feature_type,
		(enum IMGSENSOR_SENSOR_IDX)msg.sensor_idx,
		msg.msg_id, indata, msg.inDataSize, outdata, msg.outDataSize);
		if (msg.outDataSize > CCU_IPC_OBUF_CAPACITY) {
			LOG_ERR(
			"CCU_IOCTL_IPC_SEND_CMD copy_to_user oversize\n");
			ret = -EINVAL;
			kfree(indata);
			kfree(outdata);
			break;
		}
		ret = copy_to_user((void *)msg.outDataPtr, outdata, msg.outDataSize);
		kfree(indata);
		kfree(outdata);
		break;
	}
	case CCU_IOCTL_UPDATE_QOS_REQUEST: {
		uint32_t ccu_bw[3];

		ccu_qos_update_req(&ccu_bw[0]);
		ret = copy_to_user((void *)arg, &ccu_bw,
				   sizeof(uint32_t) * 3);
		break;
	}
	case CCU_IOCTL_UPDATE_CAM_FREQ_REQUEST: {
		uint32_t freq_level;

		ret = copy_from_user(&freq_level,
				     (void *)arg, sizeof(uint32_t));

		LOG_DBG_MUST("request freq level: %d\n", freq_level);

		if (freq_level == CCU_REQ_CAM_FREQ_NONE)
			mtk_pm_qos_update_request(&_ccu_qos_request, 0);
		else
			mtk_pm_qos_update_request(&_ccu_qos_request,
					      _g_freq_steps[freq_level]);

		//use pm_qos_request to get
		//current freq setting
		LOG_DBG_MUST("current freq: %d\n",
			     mtk_pm_qos_request(PM_QOS_CAM_FREQ));

		break;
	}
	case CCU_IOCTL_GET_I2C_DMA_BUF_ADDR: {
		struct ccu_i2c_buf_mva_ioarg ioarg;

		ret = copy_from_user(&ioarg, (void *)arg,
				     sizeof(struct ccu_i2c_buf_mva_ioarg));
		if (ret != 0) {
			LOG_ERR("CCU_IOCTL_GET_I2C_DMA_BUF_ADDR copy_from_user fail: %d\n",
				ret);
			ret = -EFAULT;
			break;
		}

		ret = ccu_get_i2c_dma_buf_addr(g_ccu_device, &ioarg);
		if (ret != 0) {
			LOG_ERR("ccu_get_i2c_dma_buf_addr fail: %d\n", ret);
			break;
		}
		ret = copy_to_user((void *)arg, &ioarg,
				   sizeof(struct ccu_i2c_buf_mva_ioarg));

		break;
	}
	case CCU_IOCTL_SET_I2C_MODE: {
		ret = ccu_i2c_controller_init((uint32_t)arg);

		if (ret == -1) {
			LOG_DBG("ccu_i2c_controller_init fail\n");
			ret = -EINVAL;
		}

		break;
	}
	case CCU_IOCTL_GET_CURRENT_FPS: {
		int32_t current_fps_list[IMGSENSOR_SENSOR_IDX_MAX_NUM];

		ccu_get_current_fps(current_fps_list);

		ret = copy_to_user((void *)arg, &current_fps_list,
				   sizeof(int32_t) * IMGSENSOR_SENSOR_IDX_MAX_NUM);
			break;
	}
	case CCU_IOCTL_GET_SENSOR_I2C_SLAVE_ADDR:
	{
		int32_t sensorI2cSlaveAddr[IMGSENSOR_SENSOR_IDX_MAX_NUM];

		ccu_get_sensor_i2c_slave_addr(&sensorI2cSlaveAddr[0]);
		ret = copy_to_user((void *)arg, &sensorI2cSlaveAddr,
			sizeof(int32_t) * IMGSENSOR_SENSOR_IDX_MAX_NUM);

		break;
	}

	case CCU_IOCTL_GET_SENSOR_NAME:
	{
	#define SENSOR_NAME_MAX_LEN 32
	char *sensor_names[IMGSENSOR_SENSOR_IDX_MAX_NUM];

	ccu_get_sensor_name(sensor_names);
	for (i = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		i < IMGSENSOR_SENSOR_IDX_MAX_NUM ; i++){
		if (sensor_names[i] != NULL) {
			ret = copy_to_user(((char *)arg+
				SENSOR_NAME_MAX_LEN*i),
			sensor_names[i], strlen(sensor_names[i])+1);
			if (ret != 0) {
				LOG_ERR("copy_to_user failed: %d\n", ret);
				break;
			}
		}
	}
	#undef SENSOR_NAME_MAX_LEN
	break;
	}

	case CCU_READ_STRUCT_SIZE:
	{
		uint32_t structCnt;
		uint32_t *structSizes;

		ret = copy_from_user(&structCnt,
			(void *)arg, sizeof(uint32_t));
		if (ret != 0) {
			LOG_ERR(
			"CCU_READ_STRUCT_SIZE copy_from_user failed: %d\n",
			ret);
			break;
		}
		structSizes = kzalloc(sizeof(uint32_t)*structCnt, GFP_KERNEL);
		if (!structSizes) {
			LOG_ERR(
			"CCU_READ_STRUCT_SIZE alloc failed\n");
			break;
		}
		ret = ccu_read_struct_size(structSizes, structCnt);
		if (ret != 0) {
			LOG_ERR(
			"ccu_read_struct_size failed: %d\n", ret);
			kfree(structSizes);
			break;
		}
		ret = copy_to_user((char *)arg,
			structSizes, sizeof(uint32_t)*structCnt);
		if (ret != 0) {
			LOG_ERR(
			"CCU_READ_STRUCT_SIZE copy_to_user failed: %d\n", ret);
		}
		kfree(structSizes);
		break;
	}

	case CCU_IOCTL_PRINT_REG:
	{
<<<<<<< HEAD
		uint32_t *Reg;

		Reg = kzalloc(sizeof(uint8_t)*
			(CCU_HW_DUMP_SIZE+CCU_DMEM_SIZE+CCU_PMEM_SIZE),
			GFP_KERNEL);
		if (!Reg) {
			LOG_ERR(
			"CCU_IOCTL_PRINT_REG alloc failed\n");
			break;
		}
		ccu_print_reg(Reg);
		ret = copy_to_user((char *)arg,
			Reg, sizeof(uint8_t)*
			(CCU_HW_DUMP_SIZE+CCU_DMEM_SIZE+CCU_PMEM_SIZE));
		if (ret != 0) {
			LOG_ERR(
			"CCU_IOCTL_PRINT_REG copy_to_user failed: %d\n", ret);
		}
		kfree(Reg);
		break;
=======
		int regToRead = (int)arg;
		int rc = ccu_read_info_reg(regToRead);

		mutex_unlock(&g_ccu_device->dev_mutex);
		return rc;
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
	}

	case CCU_IOCTL_PRINT_SRAM_LOG:
	{
		char *sram_log;

		sram_log = kzalloc(sizeof(char)*
			(CCU_LOG_SIZE*2+CCU_ISR_LOG_SIZE),
			GFP_KERNEL);
		if (!sram_log) {
			LOG_ERR(
			"CCU_IOCTL_PRINT_SRAM_LOG alloc failed\n");
			break;
		}
		ccu_print_sram_log(sram_log);
		ret = copy_to_user((char *)arg,
			sram_log, sizeof(char)*
			(CCU_LOG_SIZE*2+CCU_ISR_LOG_SIZE));
		if (ret != 0) {
			LOG_ERR(
			"CCU_IOCTL_PRINT_SRAM_LOG copy_to_user failed: %d\n", ret);
		}
		kfree(sram_log);
		break;
	}

	case CCU_READ_DATA:
	{
		uint32_t ccu_info[2];
		uint32_t *buf;

		ret = copy_from_user(ccu_info,
			(void *)arg, sizeof(uint32_t) * 2);
		if (ret != 0) {
			LOG_ERR(
			"CCU_READ_DATA copy_from_user failed: %d\n",
			ret);
			break;
		}
		buf = kzalloc(sizeof(uint32_t)*ccu_info[1], GFP_KERNEL);
		if (!buf) {
			LOG_ERR(
			"CCU_READ_DATA alloc failed da:%d sz:%d\n",
			ccu_info[0], ccu_info[1]);
			kfree(buf);
			ret = -EINVAL;
			break;
		}
		ret = ccu_read_data(buf, ccu_info[0], ccu_info[1]);
		if (ret != 0) {
			LOG_ERR(
			"CCU_READ_DATA failed: %d\n", ret);
			kfree(buf);
			break;
		}
		ret = copy_to_user((char *)arg,
			 buf, sizeof(uint32_t)*ccu_info[1]);
		if (ret != 0) {
			LOG_ERR(
			"CCU_READ_DATA copy_to_user failed: %d\n", ret);
		}
		kfree(buf);
		break;
	}

	case CCU_IOCTL_IMPORT_MEM: {
		struct ion_handle *handle;
		struct import_mem_s import_mem;

		ret = copy_from_user(&import_mem, (void *)arg,
				     sizeof(struct import_mem_s));
		if (ret != 0) {
			LOG_ERR("CCU_IOCTL_IMPORT_MEM copy_to_user failed: %d\n", ret);
			break;
		}

		for (i = 0; i < CCU_IMPORT_BUF_NUM; i++) {
			if (import_mem.memID[i] == CCU_IMPORT_BUF_UNDEF) {
				LOG_INF_MUST("imported buffer count: %d\n", i);
				break;
			}

			handle = ccu_ion_import_handle(
					 import_mem.memID[i]);

			if (IS_ERR(handle)) {
				ret = -EFAULT;
				LOG_ERR("CCU ccu_ion_import_handle failed: %d\n", ret);
				break;
			} else {
				import_buffer_handle[i] = handle;
			}
		}

		break;
	}

	case CCU_IOCTL_LOAD_CCU_BIN:
	{
		struct ccu_bin_info_s bin_info;

		ret = copy_from_user(&bin_info,
			(void *)arg, sizeof(struct ccu_bin_info_s));
		LOG_INF_MUST("load ccu bin %d name %s\n",
			bin_info.type,
			bin_info.name);
		powert_stat = ccu_query_power_status();
		if (bin_info.type == 0 && powert_stat == 0) {
			LOG_WARN("ccuk: ioctl without powered on\n");
			mutex_unlock(&g_ccu_device->dev_mutex);
			return -EFAULT;
		}
		ret = ccu_load_bin(g_ccu_device, &bin_info);

		break;
	}

	case CCU_IOCTL_ALLOC_MEM:
	{
		struct CcuMemHandle handle;

		handle.ionHandleKd = 0;
		ret = copy_from_user(&(handle.meminfo),
			(void *)arg, sizeof(struct CcuMemInfo));
		if (ret != 0) {
			LOG_ERR(
			"CCU_IOCTL_ALLOC_MEM copy_to_user failed: %d\n",
			ret);
			break;
		}
		ret = ccu_allocate_mem(&handle, handle.meminfo.size,
			handle.meminfo.cached);
		if (ret != 0) {
			LOG_ERR(
			"CCU_IOCTL_ALLOC_MEM ccu_allocate_mem failed: %d\n",
			ret);
			break;
		}
		break;
	}

	case CCU_IOCTL_DEALLOC_MEM:
	{
		struct CcuMemHandle handle;

		ret = copy_from_user(&(handle.meminfo),
			(void *)arg, sizeof(struct CcuMemInfo));
		if (ret != 0) {
			LOG_ERR(
			"CCU_IOCTL_ALLOC_MEM copy_to_user failed: %d\n",
			ret);
			break;
		}
		ret = ccu_deallocate_mem(&handle);
		if (ret != 0) {
			LOG_ERR(
			"CCU_IOCTL_ALLOC_MEM ccu_allocate_mem failed: %d\n",
			ret);
			break;
		}
		ret = copy_to_user((void *)arg, &handle.meminfo,
			sizeof(struct CcuMemInfo));
		break;
	}
	default:
		LOG_WARN("ioctl:No such command!\n");
		ret = -EINVAL;
		break;
	}

EXIT:
	if (ret != 0) {
		LOG_ERR(
		"fail, cmd(%d), cmd_nr(%d), pid(%d), (process, pid, tgid)=(%s, %d, %d)\n",
			cmd, _IOC_NR(cmd), user->open_pid, current->comm, current->pid,
			current->tgid);
	}

	if ((cmd != CCU_IOCTL_WAIT_IRQ) && (cmd != CCU_IOCTL_WAIT_AF_IRQ))
		mutex_unlock(&g_ccu_device->dev_mutex);

	return ret;
}

static int ccu_release(struct inode *inode, struct file *flip)
{
	struct ccu_user_s *user = flip->private_data;
	int i = 0;
	struct CcuMemHandle handle = {0};

	mutex_lock(&g_ccu_device->dev_mutex);

	LOG_INF_MUST("%s pid:%d tid:%d cnt:%d+\n",
		__func__, user->open_pid, user->open_tgid, _user_count);
	ccu_delete_user(user);
	_user_count--;

	if (_user_count > 0) {
		LOG_INF_MUST("%s bypass release flow-", __func__);
		mutex_unlock(&g_ccu_device->dev_mutex);
		return 0;
	}

<<<<<<< HEAD
=======
	mutex_lock(&g_ccu_device->dev_mutex);

	LOG_INF_MUST("%s +\n", __func__);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
	ccu_force_powerdown();

	for (i = 0; i < CCU_IMPORT_BUF_NUM; i++) {
		if (import_buffer_handle[i] == (struct ion_handle *)
		    CCU_IMPORT_BUF_UNDEF) {
			LOG_INF_MUST("freed buffer count: %d\n", i);
			break;
		}

		ccu_ion_free_import_handle(
			import_buffer_handle[i]);/*can't in spin_lock*/
	}

	handle.meminfo.cached = 0;
	ccu_deallocate_mem(&handle);
	handle.meminfo.cached = 1;
	ccu_deallocate_mem(&handle);

	ccu_ion_uninit();

	LOG_INF_MUST("%s -", __func__);

	mutex_unlock(&g_ccu_device->dev_mutex);

	mutex_unlock(&g_ccu_device->dev_mutex);

	return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
static dev_t ccu_devt;
static struct cdev *ccu_chardev;
static struct class *ccu_class;
static int ccu_num_devs;

static inline void ccu_unreg_chardev(void)
{
	/* Release char driver */
	if (ccu_chardev != NULL) {
		cdev_del(ccu_chardev);
		ccu_chardev = NULL;
	}
	unregister_chrdev_region(ccu_devt, 1);
}

static inline int ccu_reg_chardev(void)
{
	int ret = 0;

	ret = alloc_chrdev_region(&ccu_devt, 0, 1, CCU_DEV_NAME);
	if ((ret) < 0) {
		LOG_ERR("alloc_chrdev_region failed, %d\n", ret);
		return ret;
	}
	/* Allocate driver */
	ccu_chardev = cdev_alloc();
	if (ccu_chardev == NULL) {
		LOG_ERR("cdev_alloc failed\n");
		ret = -ENOMEM;
		goto EXIT;
	}

	/* Attatch file operation. */
	cdev_init(ccu_chardev, &ccu_fops);

	ccu_chardev->owner = THIS_MODULE;

	/* Add to system */
	ret = cdev_add(ccu_chardev, ccu_devt, 1);
	if ((ret) < 0) {
		LOG_ERR("Attatch file operation failed, %d\n", ret);
		goto EXIT;
	}

EXIT:
	if (ret < 0)
		ccu_unreg_chardev();

	return ret;
}

/*****************************************************************************
 * platform_driver
 ****************************************************************************/

static int ccu_read_platform_info_from_dt(struct device_node
		*node)
{
	uint32_t reg[4] = {0, 0, 0, 0};
	int ret = 0;

	ret = of_property_read_u32_array(node, "reg", reg, 4);
	if (ret < 0)
		LOG_ERR("of_property_read_u32_array ERR : %d\n", ret);

	ccu_hw_base = reg[1];

	LOG_DBG("ccu read dt property ccu_hw_base = %x\n", ccu_hw_base);

	return ret;
}

static int ccu_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF
	struct device *dev;
	struct device_node *node;
	int ret = 0;
	uint32_t phy_addr;
	uint32_t phy_size;


	node = pdev->dev.of_node;
	g_ccu_device->dev = &pdev->dev;
	LOG_DBG("probe 0, pdev id = %d name = %s\n", pdev->id,
		pdev->name);

	ccu_read_platform_info_from_dt(node);

#ifdef MTK_CCU_EMULATOR
	/* emulator will fill ccu_base and bin_base */
	/*ccu_init_emulator(g_ccu_device);*/
#else
	/* get register address */
	if ((strcmp("ccu", g_ccu_device->dev->of_node->name) == 0)) {

/* get physical address of pmem  */
/* ioremap_wc() has no access 4 bytes alignmen
 * limitation as of_iomap() does?
 * https://forums.xilinx.com/xlnx/attachments/
 * xlnx/ELINUX/11158/1/Linux%20CPU%20to%20PL%20Access.pdf
 */

		/*remap ccu_base*/
		phy_addr = ccu_hw_base;
		phy_size = 0x1000;
		g_ccu_device->ccu_base =
			ioremap(phy_addr, phy_size);
		LOG_INF("ccu_base pa: 0x%x, size: 0x%x\n", phy_addr, phy_size);
		LOG_INF("ccu_base va: 0x%lx\n", g_ccu_device->ccu_base);

		/*remap dmem_base*/
		phy_addr = CCU_DMEM_BASE;
		phy_size = CCU_DMEM_SIZE;
		g_ccu_device->dmem_base =
			ioremap(phy_addr, phy_size);
		LOG_INF("dmem_base pa: 0x%x, size: 0x%x\n", phy_addr, phy_size);
		LOG_INF("dmem_base va: 0x%lx\n", g_ccu_device->dmem_base);

		/*remap camsys_base*/
		phy_addr = CCU_CAMSYS_BASE;
		phy_size = CCU_CAMSYS_SIZE;
		g_ccu_device->camsys_base =
			ioremap(phy_addr, phy_size);
		LOG_INF("camsys_base pa: 0x%x, size: 0x%x\n", phy_addr,
			phy_size);
		LOG_INF("camsys_base va: 0x%lx\n", g_ccu_device->camsys_base);

		phy_addr = CCU_PMEM_BASE;
		phy_size = CCU_PMEM_SIZE;
		g_ccu_device->pmem_base =
			ioremap(phy_addr, phy_size);
		LOG_DBG_MUST("pmem_base pa: 0x%x, size: 0x%x\n", phy_addr, phy_size);
		LOG_DBG_MUST("pmem_base va: 0x%lx\n", g_ccu_device->pmem_base);

		/* get Clock control from device tree.  */
		ccu_clk_ctrl[0] = devm_clk_get(g_ccu_device->dev,
					       "CCU_CLK_TOP_MUX");
		if (ccu_clk_ctrl[0] == NULL)
			LOG_ERR("Get CCU_CLK_TOP_MUX fail.\n");
		ccu_clk_ctrl[1] = devm_clk_get(g_ccu_device->dev,
					       "CAM_PWR");
		if (ccu_clk_ctrl[1] == NULL)
			LOG_ERR("Get CAM_PWR fail.\n");
		ccu_clk_ctrl[2] = devm_clk_get(g_ccu_device->dev,
					       "CCU_CLK_MMSYS_CCU");
		if (ccu_clk_ctrl[2] == NULL)
			LOG_ERR("Get CCU_CLK_MMSYS_CCU fail.\n");
		ccu_clk_ctrl[3] = devm_clk_get(g_ccu_device->dev,
					       "CCU_CLK_CAM_CCU");
		if (ccu_clk_ctrl[3] == NULL)
			LOG_ERR("Get CCU_CLK_CAM_CCU fail.\n");

		g_ccu_device->irq_num = irq_of_parse_and_map(node, 0);
		LOG_DBG("probe 1, ccu_base: 0x%lx, bin_base: 0x%lx, irq_num: %d, pdev: %p\n",
			g_ccu_device->ccu_base, g_ccu_device->bin_base,
			g_ccu_device->irq_num, g_ccu_device->dev);

		if (g_ccu_device->irq_num > 0) {
			/* get IRQ flag from device node */
			unsigned int irq_info[3];

			if (of_property_read_u32_array
			    (node, "interrupts", irq_info, ARRAY_SIZE(irq_info))) {
				LOG_DERR(g_ccu_device->dev, "get irq flags from DTS fail!\n");
				return -ENODEV;
			}
		} else {
			LOG_DBG("No IRQ!!: ccu_num_devs=%d, devnode(%s), irq=%d\n",
				ccu_num_devs,
				g_ccu_device->dev->of_node->name, g_ccu_device->irq_num);
		}

		/* Only register char driver in the 1st time */
		if (++ccu_num_devs == 1) {

			/* Register char driver */
			ret = ccu_reg_chardev();
			if (ret) {
				LOG_DERR(g_ccu_device->dev, "register char failed");
				return ret;
			}

			/* Create class register */
			ccu_class = class_create(THIS_MODULE, "ccudrv");
			if (IS_ERR(ccu_class)) {
				ret = PTR_ERR(ccu_class);
				LOG_ERR("Unable to create class, err = %d\n", ret);
				goto EXIT;
			}

			dev = device_create(ccu_class, NULL, ccu_devt, NULL,
					    CCU_DEV_NAME);
			if (IS_ERR(dev)) {
				ret = PTR_ERR(dev);
				LOG_DERR(g_ccu_device->dev,
					 "Failed to create device: /dev/%s, err = %d", CCU_DEV_NAME,
					 ret);
				goto EXIT;
			}
#ifdef CONFIG_PM_SLEEP
/*wakeup_source_init(&ccu_wake_lock, "ccu_lock_wakelock");*/
#endif

			/* enqueue/dequeue control in ihalpipe wrapper */
			init_waitqueue_head(&wait_queue_deque);
			init_waitqueue_head(&wait_queue_enque);

			/*for (i = 0; i < CCU_IRQ_NUM_TYPES; i++) {*/
			/*      tasklet_init(ccu_tasklet[i].pCCU_tkt, */
			/* ccu_tasklet[i].tkt_cb, 0);*/
			/*}*/

			/*register i2c driver callback*/
			ret = ccu_i2c_register_driver();
			if (ret < 0)
				goto EXIT;

			/*allocate dma buffer for i2c*/
			if (dma_set_mask(g_ccu_device->dev, DMA_BIT_MASK(36))) {
				LOG_DERR(g_ccu_device->dev, "dma_set_mask return error.");
				goto EXIT;
			}
			g_ccu_device->i2c_dma_vaddr = dma_alloc_coherent(
							      g_ccu_device->dev,
							      CCU_I2C_DMA_BUF_SIZE,
							      &g_ccu_device->i2c_dma_paddr, GFP_KERNEL);
			if (g_ccu_device->i2c_dma_vaddr == NULL) {
				LOG_DERR(g_ccu_device->dev, "dma_alloc_coherent fail\n");
				ret = -ENOMEM;
				goto EXIT;
			}

			g_ccu_device->i2c_dma_mva = 0;

EXIT:
			if (ret < 0)
				ccu_unreg_chardev();
		}

		ccu_init_hw(g_ccu_device);

		LOG_INF_MUST("ccu probe cuccess...\n");

	}
#endif
#endif

	LOG_DBG("- X. CCU driver probe.\n");

	return ret;
}


static int ccu_remove(struct platform_device *pDev)
{
	/*    struct resource *pRes; */
	int irq_num;

	/*  */
	LOG_DBG("- E.");

	/*free i2c dma buffer*/
	dma_free_coherent(g_ccu_device->dev, CCU_I2C_DMA_BUF_SIZE,
			  g_ccu_device->i2c_dma_vaddr, g_ccu_device->i2c_dma_paddr);

	/*uninit hw*/
	ccu_uninit_hw(g_ccu_device);

	/* unregister char driver. */
	ccu_unreg_chardev();

	/*ccu_i2c_del_drivers();*/
	ccu_i2c_delete_driver();

	/* Release IRQ */
	disable_irq(g_ccu_device->irq_num);
	irq_num = platform_get_irq(pDev, 0);
	free_irq(irq_num, (void *)ccu_chardev);

	/* kill tasklet */
	/*for (i = 0; i < CCU_IRQ_NUM_TYPES; i++) {*/
	/*      tasklet_kill(ccu_tasklet[i].p_ccu_tkt);*/
	/*}*/

	/*  */
	device_destroy(ccu_class, ccu_devt);
	/*  */
	class_destroy(ccu_class);
	ccu_class = NULL;
	/*  */
	return 0;
}

static int ccu_suspend(struct platform_device *pdev,
		       pm_message_t mesg)
{
	return 0;
}

static int ccu_resume(struct platform_device *pdev)
{
	return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int __init CCU_INIT(void)
{
	int ret = 0;

	/*struct device_node *node = NULL;*/

	g_ccu_device = kzalloc(sizeof(struct ccu_device_s), GFP_KERNEL);
	/*g_ccu_device = dma_cache_coherent();*/

	INIT_LIST_HEAD(&g_ccu_device->user_list);
	mutex_init(&g_ccu_device->user_mutex);
<<<<<<< HEAD
=======
	mutex_init(&g_ccu_device->clk_mutex);
	mutex_init(&g_ccu_device->dev_mutex);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
	mutex_init(&g_ccu_device->ion_client_mutex);
	mutex_init(&g_ccu_device->dev_mutex);
	mutex_init(&g_ccu_device->clk_mutex);

	LOG_DBG("platform_driver_register start\n");
	if (platform_driver_register(&ccu_driver)) {
		LOG_ERR("failed to register CCU driver");
		return -ENODEV;
	}

	LOG_DBG("platform_driver_register finsish\n");
#ifdef CONFIG_MTK_QOS_SUPPORT_ENABLE
	//Call pm_qos_add_request when
	//initialize module or driver prob
	LOG_INF_MUST("mtk_pm_qos_add_request start %x\n", &_ccu_qos_request);

	mtk_pm_qos_add_request(&_ccu_qos_request,
			   PM_QOS_CAM_FREQ, PM_QOS_MM_FREQ_DEFAULT_VALUE);

	//Call mmdvfs_qos_get_freq_steps
	//to get supported frequency
	ret = mmdvfs_qos_get_freq_steps(PM_QOS_CAM_FREQ,
					   _g_freq_steps, &_step_size);

	if (ret < 0)
		LOG_ERR("get MMDVFS freq steps failed, result: %d\n", ret);
#endif
	return ret;
}


static void __exit CCU_EXIT(void)
{
	//Call pm_qos_remove_request when
	//de-initialize module or driver remove
#ifdef CONFIG_MTK_QOS_SUPPORT_ENABLE
	mtk_pm_qos_remove_request(&_ccu_qos_request);
#endif
	platform_driver_unregister(&ccu_driver);
	kfree(g_ccu_device);
}


/*******************************************************************************
 *
 ******************************************************************************/
module_init(CCU_INIT);
module_exit(CCU_EXIT);
MODULE_DESCRIPTION("MTK CCU Driver");
MODULE_AUTHOR("SW1");
MODULE_LICENSE("GPL");
