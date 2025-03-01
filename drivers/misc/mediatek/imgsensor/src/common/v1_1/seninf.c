/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>

#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include <mt-plat/sync_write.h>

#include "kd_seninf.h"

#include "seninf_common.h"
#include "seninf_clk.h"
#include "seninf.h"
#include "kd_imgsensor_errcode.h"
#include "imgsensor_ca.h"
#include <linux/delay.h>

#define SENINF_WR32(addr, data)    mt_reg_sync_writel(data, addr)
#define SENINF_RD32(addr)          ioread32((void *)addr)

static struct SENINF gseninf;

#ifdef DFS_CTRL_BY_OPP
static int seninf_dfs_init(struct seninf_dfs_ctx *ctx, struct device *dev)
{
	int ret, i;
	struct dev_pm_opp *opp;
	unsigned long freq;

	ctx->dev = dev;

	ret = dev_pm_opp_of_add_table(dev);
	if (ret < 0) {
		dev_info(dev, "fail to init opp table: %d\n", ret);
		return ret;
	}

	ctx->reg = devm_regulator_get_optional(dev, "dvfsrc-vcore");
	if (IS_ERR(ctx->reg)) {
		dev_info(dev, "can't get dvfsrc-vcore\n");
		return PTR_ERR(ctx->reg);
	}

	ctx->cnt = dev_pm_opp_get_opp_count(dev);

	ctx->freqs = devm_kzalloc(dev,
			sizeof(unsigned long) * ctx->cnt, GFP_KERNEL);
	ctx->volts = devm_kzalloc(dev,
			sizeof(unsigned long) * ctx->cnt, GFP_KERNEL);
	if (!ctx->freqs || !ctx->volts)
		return -ENOMEM;

	i = 0;
	freq = 0;
	while (!IS_ERR(opp = dev_pm_opp_find_freq_ceil(dev, &freq))) {
		ctx->freqs[i] = freq;
		ctx->volts[i] = dev_pm_opp_get_voltage(opp);
		freq++;
		i++;
		dev_pm_opp_put(opp);
	}

	return 0;
}

static void seninf_dfs_exit(struct seninf_dfs_ctx *ctx)
{
	dev_pm_opp_of_remove_table(ctx->dev);
}

static int seninf_dfs_ctrl(struct seninf_dfs_ctx *ctx,
		enum DFS_OPTION option, void *pbuff)
{
	int i4RetValue = 0;

	/*pr_info("%s\n", __func__);*/

	switch (option) {
	case DFS_CTRL_ENABLE:
		break;
	case DFS_CTRL_DISABLE:
		break;
	case DFS_UPDATE:
	{
		unsigned long freq, volt;
		struct dev_pm_opp *opp;

		freq = *(unsigned int *)pbuff;
		opp = dev_pm_opp_find_freq_ceil(ctx->dev, &freq);
		volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);
		pr_debug("%s: freq=%ld volt=%ld\n", __func__, freq, volt);
		regulator_set_voltage(ctx->reg, volt, ctx->volts[ctx->cnt-1]);
	}
		break;
	case DFS_RELEASE:
		break;
	case DFS_SUPPORTED_ISP_CLOCKS:
	{
		struct IMAGESENSOR_GET_SUPPORTED_ISP_CLK *pIspclks;
		int i;

		pIspclks = (struct IMAGESENSOR_GET_SUPPORTED_ISP_CLK *) pbuff;

		pIspclks->clklevelcnt = ctx->cnt;

		if (pIspclks->clklevelcnt > ISP_CLK_LEVEL_CNT) {
			pr_info("ERR: clklevelcnt is exceeded\n");
			i4RetValue = -EFAULT;
			break;
		}

		for (i = 0; i < pIspclks->clklevelcnt; i++)
			pIspclks->clklevel[i] = ctx->freqs[i];
	}
		break;
	case DFS_CUR_ISP_CLOCK:
	{
		unsigned int *pGetIspclk;
		int i, cur_volt;

		pGetIspclk = (unsigned int *) pbuff;
		cur_volt = regulator_get_voltage(ctx->reg);

		for (i = 0; i < ctx->cnt; i++) {
			if (ctx->volts[i] == cur_volt) {
				*pGetIspclk = (u32)ctx->freqs[i];
				break;
			}
		}
	}
		break;
	default:
		pr_info("None\n");
		break;
	}
	return i4RetValue;
}
#endif

extern MUINT32 Switch_Tg_For_Stagger(MUINT16 camtg)
{
#ifdef _CAM_MUX_SWITCH
	return _switch_tg_for_stagger(camtg, &gseninf);
#else
	return 0;
#endif
}
EXPORT_SYMBOL(Switch_Tg_For_Stagger);

MINT32 seninf_dump_reg(void)
{
	int i = 0;
	int k = 0;
	PK_PR_ERR("- E.");
	/*Sensor interface Top mux and Package counter */
	PK_PR_ERR(
	"seninf_top: SENINF_TOP_MUX_CTROL_0(0x%x) SENINF_TOP_MUX_CTROL_1(0x%x)\n",
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0010),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0014));

	PK_PR_ERR(
	"seninf_cam_mux: SENINF_CAM_MUX_CTRL1(0x%x) SENINF_CAM_MUX_CTRL2(0x%x) SENINF_CAM_MUX_IRQ_EN(0x%x) SENINF_CAM_MUX_IRQ_STATUS(0x%x)\n",
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0404),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0408),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x04A0),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x04A8));
	PK_PR_ERR(
	"seninf_cam_mux: SENINF_CAM_MUX0_CHK_RES(0x%x) SENINF_CAM_MUX1_CHK_RES(0x%x) SENINF_CAM_MUX2_CHK_RES(0x%x) SENINF_CAM_MUX3_CHK_RES(0x%x)\n",
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0508),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0518),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0528),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0538));
	PK_PR_ERR(
	"seninf_cam_mux: SENINF_CAM_MUX4_CHK_RES(0x%x) SENINF_CAM_MUX5_CHK_RES(0x%x) SENINF_CAM_MUX6_CHK_RES(0x%x) SENINF_CAM_MUX7_CHK_RES(0x%x)\n",
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0548),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0558),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0568),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0578));
	PK_PR_ERR(
	"seninf_cam_mux: SENINF_CAM_MUX8_CHK_RES(0x%x) SENINF_CAM_MUX9_CHK_RES(0x%x) SENINF_CAM_MUX10_CHK_RES(0x%x)\n",
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0588),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x0598),
	     SENINF_RD32(gseninf.pseninf_base[0] + 0x05A8));

	for (k = 0; k < 2; k++) {
		for (i = 0; i < SENINF_MAX_NUM ; i++) {
			PK_DBG(
		"seninf%d: SENINF%d_CTRL(0x%x) SENINF%d_CSI2_CTRL(0x%x)\n",
			i + 1,
			i + 1,
			SENINF_RD32(gseninf.pseninf_base[i] + 0x0200),
			i + 1,
			SENINF_RD32(gseninf.pseninf_base[i] + 0x0210));

		PK_DBG(
		"seninf%d_csi2: SENINF%d_CSI2_EN(0x%x) SENINF%d_CSI2_IRQ_STATUS(0x%x) SENINF%d_CSI2_PACKET_CNT_STATUS(0x%x)\n",
			i + 1,
			i + 1,
			SENINF_RD32(gseninf.pseninf_base[i] + 0x0A00),
			i + 1,
			SENINF_RD32(gseninf.pseninf_base[i] + 0x0AC8),
			i + 1,
			SENINF_RD32(gseninf.pseninf_base[i] + 0x0ADC));
		SENINF_WR32(gseninf.pseninf_base[i] + 0x0AC8, 0xFFFFFFFF);

		PK_DBG(
		"seninf%d_mux: SENINF%d_MUX_CTRL_0(0x%x) SENINF%d_MUX_IRQ_STATUS(0x%x) SENINF%d_MUX_SIZE(0x%x)\n",
			i + 1,
			i + 1,
			SENINF_RD32(gseninf.pseninf_base[i] + 0x0D00),
			i + 1,
			SENINF_RD32(gseninf.pseninf_base[i] + 0x0D18),
			i + 1,
			SENINF_RD32(gseninf.pseninf_base[i] + 0x0D30));
		SENINF_WR32(gseninf.pseninf_base[i] + 0x0D18, 0xFFFFFFFF);

		}
		mdelay(5);
	}

	return 0;
}

static irqreturn_t seninf_irq(MINT32 Irq, void *DeviceId)
{
#ifdef SENINF_IRQ
	_seninf_irq(Irq, DeviceId, &gseninf);
#else
	seninf_dump_reg();
#endif

	return IRQ_HANDLED;
}

static MINT32 seninf_open(struct inode *pInode, struct file *pFile)
{
#if SENINF_CLK_CONTROL
	struct SENINF *pseninf = &gseninf;

#ifdef SENINF_USE_RPM
	pm_runtime_get_sync(pseninf->dev);
#endif

	mutex_lock(&pseninf->seninf_mutex);
	if (atomic_inc_return(&pseninf->seninf_open_cnt) == 1)
		seninf_clk_open(&pseninf->clk);

	PK_DBG("%s %d\n", __func__,
	       atomic_read(&pseninf->seninf_open_cnt));

	mutex_unlock(&pseninf->seninf_mutex);
#endif
	return 0;
}

static MINT32 seninf_release(struct inode *pInode, struct file *pFile)
{
#if SENINF_CLK_CONTROL
	struct SENINF *pseninf = &gseninf;
#endif

	mutex_lock(&pseninf->seninf_mutex);
#if SENINF_CLK_CONTROL
	if (atomic_dec_and_test(&pseninf->seninf_open_cnt))
		seninf_clk_release(&pseninf->clk);
#endif

#ifdef IMGSENSOR_DFS_CTRL_ENABLE
		imgsensor_dfs_ctrl(DFS_RELEASE, NULL);
#endif

#if SENINF_CLK_CONTROL
	PK_DBG("%s %d\n", __func__,
	       atomic_read(&pseninf->seninf_open_cnt));
	mutex_unlock(&pseninf->seninf_mutex);

#ifdef SENINF_USE_RPM
	pm_runtime_put_sync(pseninf->dev);
#endif
#endif

	return 0;
}

static MINT32 seninf_mmap(struct file *pFile, struct vm_area_struct *pVma)
{
	unsigned long length = 0;
	unsigned long pfn = 0x0;

	/*PK_DBG("- E."); */
	length = (pVma->vm_end - pVma->vm_start);
	/*  */
	pVma->vm_page_prot = pgprot_noncached(pVma->vm_page_prot);
	pfn = pVma->vm_pgoff << PAGE_SHIFT;

	/*PK_DBG("mmap: vm_pgoff(0x%lx),pfn(0x%x),phy(0x%lx),
	 * vm_start(0x%lx),vm_end(0x%lx),length(0x%lx)\n",
	 *pVma->vm_pgoff, pfn, pVma->vm_pgoff << PAGE_SHIFT,
	 *pVma->vm_start, pVma->vm_end, length);
	 */
	switch (pfn) {
	case SENINF_MAP_BASE_REG:
		if (length > SENINF_MAP_LENGTH_REG) {
			PK_PR_ERR(
			"mmap range error :module(0x%lx),length(0x%lx),SENINF_BASE_RANGE(0x%x)!\n",
			pfn, length, SENINF_MAP_LENGTH_REG);
			return -EINVAL;
		}
		break;
	case SENINF_MAP_BASE_ANA:
		if (length > SENINF_MAP_LENGTH_ANA) {
			PK_PR_ERR(
			"mmap range error :module(0x%lx),length(0x%lx),MIPI_RX_RANGE(0x%x)!\n",
			pfn, length, SENINF_MAP_LENGTH_ANA);
			return -EINVAL;
		}
		break;
	case SENINF_MAP_BASE_GPIO:
		if (length > SENINF_MAP_LENGTH_GPIO) {
			PK_PR_ERR(
			"mmap range error :module(0x%lx),length(0x%lx),GPIO_RX_RANGE(0x%x)!\n",
			pfn, length, SENINF_MAP_LENGTH_GPIO);
			return -EINVAL;
		}
		break;
	default:
		PK_PR_ERR("Illegal starting HW addr for mmap!\n");
		return -EINVAL;

	}

	if (remap_pfn_range
		(pVma,
		pVma->vm_start,
		pVma->vm_pgoff,
		pVma->vm_end - pVma->vm_start,
		pVma->vm_page_prot))
		return -EINVAL;

	return 0;
}

static long seninf_ioctl(struct file *pfile,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	void *pbuff = NULL;
#if SENINF_CLK_CONTROL
	struct SENINF *pseninf = &gseninf;
#endif

#ifdef CONFIG_MTK_CAM_SECURITY_SUPPORT
	struct command_params c_params = {0};
#endif

	if (_IOC_DIR(cmd) != _IOC_NONE) {
		pbuff = kmalloc(_IOC_SIZE(cmd), GFP_KERNEL);
		if (pbuff == NULL) {
			PK_DBG("ioctl allocate mem failed\n");
			ret = -ENOMEM;
			goto SENINF_IOCTL_EXIT;
		}

		if (_IOC_WRITE & _IOC_DIR(cmd)) {
			if (copy_from_user(pbuff,
						(void *)arg, _IOC_SIZE(cmd))) {
				PK_DBG("ioctl copy from user failed\n");
				ret = -EFAULT;
				goto SENINF_IOCTL_EXIT;
			}
		} else
			memset(pbuff, 0, _IOC_SIZE(cmd));
	} else {
		ret = -EFAULT;
		goto SENINF_IOCTL_EXIT;
	}

	switch (cmd) {
	case KDSENINFIOC_X_GET_REG_ADDR:
		{
			struct KD_SENINF_REG *preg = pbuff;

			preg->seninf.map_addr = SENINF_MAP_BASE_REG;
			preg->seninf.map_length = SENINF_MAP_LENGTH_REG;
			preg->ana.map_addr = SENINF_MAP_BASE_ANA;
			preg->ana.map_length = SENINF_MAP_LENGTH_ANA;
			preg->gpio.map_addr = SENINF_MAP_BASE_GPIO;
			preg->gpio.map_length = SENINF_MAP_LENGTH_GPIO;
		}
		break;

	case KDSENINFIOC_X_SET_MCLK_PLL:
#if SENINF_CLK_CONTROL
		ret = seninf_clk_set(&pseninf->clk,
				(struct ACDK_SENSOR_MCLK_STRUCT *) pbuff);
#endif
		break;

	case KDSENINFIOC_X_GET_ISP_CLK:
	case KDSENINFIOC_X_GET_CSI_CLK:
#if SENINF_CLK_CONTROL
		*(unsigned int *)pbuff =
			seninf_clk_get_meter(&pseninf->clk,
			*(unsigned int *)pbuff);
#endif
		break;
#ifdef DFS_CTRL_BY_OPP
	case KDSENINFIOC_DFS_UPDATE:
		ret = seninf_dfs_ctrl(&gseninf.dfs_ctx, DFS_UPDATE, pbuff);
		break;
	case KDSENINFIOC_GET_SUPPORTED_ISP_CLOCKS:
		ret = seninf_dfs_ctrl(&gseninf.dfs_ctx,
				DFS_SUPPORTED_ISP_CLOCKS, pbuff);
		break;
	case KDSENINFIOC_GET_CUR_ISP_CLOCK:
		ret = seninf_dfs_ctrl(&gseninf.dfs_ctx,
				DFS_CUR_ISP_CLOCK, pbuff);
		break;
#endif
#ifdef IMGSENSOR_DFS_CTRL_ENABLE
	/*mmdvfs start*/
	case KDSENINFIOC_DFS_UPDATE:
		ret = imgsensor_dfs_ctrl(DFS_UPDATE, pbuff);
		break;
	case KDSENINFIOC_GET_SUPPORTED_ISP_CLOCKS:
		ret = imgsensor_dfs_ctrl(DFS_SUPPORTED_ISP_CLOCKS, pbuff);
		break;
	case KDSENINFIOC_GET_CUR_ISP_CLOCK:
		ret = imgsensor_dfs_ctrl(DFS_CUR_ISP_CLOCK, pbuff);
		break;
	/*mmdvfs end*/
#endif
#ifdef CONFIG_MTK_CAM_SECURITY_SUPPORT
	case KDSENINFIOC_X_SECURE_DUMP:
		if (imgsensor_ca_invoke_command(
			IMGSENSOR_TEE_CMD_DUMP_REG, c_params, &ret) != 0)
			ret = ERROR_TEE_CA_TA_FAIL;
		break;
#endif
	case KDSENINFIOC_SET_CAM_MUX_FOR_SWITCH:
#ifdef _CAM_MUX_SWITCH
		ret = _seninf_set_tg_for_switch(
			(*(unsigned int *)pbuff) >> 16, (*(unsigned int *)pbuff) & 0xFFFF);
#endif
		break;
	default:
		PK_DBG("No such command %d\n", cmd);
		ret = -EPERM;
		goto SENINF_IOCTL_EXIT;
		break;
	}

	if ((_IOC_READ & _IOC_DIR(cmd)) && copy_to_user((void __user *)arg,
			pbuff, _IOC_SIZE(cmd))) {
		PK_DBG("[CAMERA SENSOR] ioctl copy to user failed\n");
		ret = -EFAULT;
		goto SENINF_IOCTL_EXIT;
	}

SENINF_IOCTL_EXIT:
<<<<<<< HEAD
	if (pbuff != NULL) {
		kfree(pbuff);
		pbuff = NULL;
	}

=======
	if (pbuff != NULL)
		kfree(pbuff);
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
	return ret;
}

#ifdef CONFIG_COMPAT
static long seninf_ioctl_compat(struct file *pfile,
	unsigned int cmd, unsigned long arg)
{
	if (!pfile->f_op || !pfile->f_op->unlocked_ioctl)
		return -ENOTTY;

	return pfile->f_op->unlocked_ioctl(pfile, cmd, arg);
}
#endif

static const struct file_operations gseninf_file_operations = {
	.owner          = THIS_MODULE,
	.open           = seninf_open,
	.release        = seninf_release,
	.mmap           = seninf_mmap,
	.unlocked_ioctl = seninf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = seninf_ioctl_compat,
#endif
};

static inline void seninf_unreg_char_dev(struct SENINF *pseninf)
{
	PK_DBG("- E.");

	/* Release char driver */
	if (pseninf->pchar_dev != NULL) {
		cdev_del(pseninf->pchar_dev);
		pseninf->pchar_dev = NULL;
	}

	unregister_chrdev_region(pseninf->dev_no, 1);
}

static inline MINT32 seninf_reg_char_dev(struct SENINF *pseninf)
{
	MINT32 ret = 0;

#ifdef CONFIG_OF
	struct device *dev = NULL;
#endif

	PK_DBG("- E.\n");

	ret = alloc_chrdev_region(&pseninf->dev_no, 0, 1, SENINF_DEV_NAME);
	if (ret < 0) {
		PK_PR_ERR("alloc_chrdev_region failed, %d\n", ret);
		return ret;
	}
	/* Allocate driver */
	pseninf->pchar_dev = cdev_alloc();
	if (pseninf->pchar_dev == NULL) {
		PK_PR_ERR("cdev_alloc failed\n");
		ret = -ENOMEM;
		goto EXIT;
	}
	/* Attatch file operation. */
	cdev_init(pseninf->pchar_dev, &gseninf_file_operations);

	pseninf->pchar_dev->owner = THIS_MODULE;
	/* Add to system */
	if (cdev_add(pseninf->pchar_dev, pseninf->dev_no, 1) < 0) {
		PK_PR_ERR("Attatch file operation failed, %d\n", ret);
		goto EXIT;
	}

	/* Create class register */
	pseninf->pclass = class_create(THIS_MODULE, SENINF_DEV_NAME);
	if (IS_ERR(pseninf->pclass)) {
		ret = PTR_ERR(pseninf->pclass);
		PK_PR_ERR("Unable to create class, err = %d\n", ret);
		goto EXIT;
	}

	dev = device_create(pseninf->pclass,
						NULL,
						pseninf->dev_no,
						NULL,
						SENINF_DEV_NAME);
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		dev_err(dev, "Failed to create device: /dev/%s, err = %d",
			SENINF_DEV_NAME, ret);
		goto EXIT;
	}

EXIT:
	if (ret < 0)
		seninf_unreg_char_dev(pseninf);

	PK_DBG("- X.\n");
	return ret;
}

static MINT32 seninf_probe(struct platform_device *pDev)
{
	struct SENINF *pseninf = &gseninf;
	MINT32 ret = 0;
	MUINT32 irq_info[3];	/* Record interrupts info from device tree */
	int irq;

	seninf_reg_char_dev(pseninf);

	mutex_init(&pseninf->seninf_mutex);
	atomic_set(&pseninf->seninf_open_cnt, 0);
	pseninf->dev = &pDev->dev;

#ifdef SENINF_USE_RPM
	pm_runtime_enable(pseninf->dev);
#endif

#if SENINF_CLK_CONTROL
	pseninf->clk.pplatform_device = pDev;
	seninf_clk_init(&pseninf->clk);
#endif

#ifdef DFS_CTRL_BY_OPP
	if (seninf_dfs_init(&pseninf->dfs_ctx, &pDev->dev))
		return -ENODEV;
#endif

	/* get IRQ ID and request IRQ */
	irq = irq_of_parse_and_map(pDev->dev.of_node, 0);

	if (irq > 0) {
		/* Get IRQ Flag from device node */
		if (of_property_read_u32_array
				(pDev->dev.of_node,
				"interrupts",
				irq_info,
				ARRAY_SIZE(irq_info))) {
			dev_err(&pDev->dev, "get irq flags from DTS fail!!\n");
			return -ENODEV;
		}

		ret = request_irq(irq,
						(irq_handler_t) seninf_irq,
						irq_info[2],
						"SENINF",
						NULL);
		if (ret) {
			dev_err(&pDev->dev, "request_irq fail\n");
			return ret;
		}

		PK_DBG("devnode(%s), irq=%d\n", pDev->dev.of_node->name, irq);
	} else {
		PK_DBG("No IRQ!!\n");
	}

	return ret;
}

static MINT32 seninf_remove(struct platform_device *pDev)
{
	struct SENINF *pseninf = &gseninf;

	PK_DBG("- E.");

#ifdef DFS_CTRL_BY_OPP
	seninf_dfs_exit(&pseninf->dfs_ctx);
#endif

#if SENINF_CLK_CONTROL && defined(HAVE_SENINF_CLK_EXIT)
	seninf_clk_exit(&pseninf->clk);
#endif

	/* unregister char driver. */
	seninf_unreg_char_dev(pseninf);

	/* Release IRQ */
	free_irq(platform_get_irq(pDev, 0), NULL);

	device_destroy(pseninf->pclass, pseninf->dev_no);

	class_destroy(pseninf->pclass);
	pseninf->pclass = NULL;

	return 0;
}

static MINT32 seninf_suspend(struct platform_device *pDev, pm_message_t mesg)
{
	return 0;
}

static MINT32 seninf_resume(struct platform_device *pDev)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id gseninf_of_device_id[] = {
	{.compatible = "mediatek,seninf_top",},
	{}
};
#endif

static struct platform_driver gseninf_platform_driver = {
	.probe = seninf_probe,
	.remove = seninf_remove,
	.suspend = seninf_suspend,
	.resume = seninf_resume,
	.driver = {
			.name = SENINF_DEV_NAME,
			.owner = THIS_MODULE,
#ifdef CONFIG_OF
			.of_match_table = gseninf_of_device_id,
#endif
			}
};

static inline MINT32 seninf_reg_of_dev(struct SENINF *pseninf)
{
	int i;
	char pdev_name[64];
	struct device_node *node = NULL;

	/* Map seninf */
	for (i = 0; i < SENINF_MAX_NUM; i++) {
		snprintf(pdev_name, 64, "mediatek,seninf%d", i + 1);
		node = of_find_compatible_node(NULL, NULL, pdev_name);
		if (!node) {
			PK_PR_ERR("find mediatek,seninf%d node failed!!!\n",
			i + 1);
			return -ENODEV;
		}
		pseninf->pseninf_base[i] = of_iomap(node, 0);
		if (!pseninf->pseninf_base[i]) {
			PK_PR_ERR("unable to map SENINF%d_BASE registers!!!\n",
			i + 1);
			return -ENODEV;
		}
		PK_DBG("SENINF%d_BASE: %p\n", i + 1, pseninf->pseninf_base[i]);
	}

	return 0;
}

static int __init seninf_init(void)
{
	if (platform_driver_register(&gseninf_platform_driver) < 0) {
		PK_PR_ERR("platform_driver_register fail");
		return -ENODEV;
	}

	seninf_reg_of_dev(&gseninf);

#ifdef IMGSENSOR_DFS_CTRL_ENABLE
	imgsensor_dfs_ctrl(DFS_CTRL_ENABLE, NULL);
#endif

	return 0;
}

static void __exit seninf_exit(void)
{
#ifdef IMGSENSOR_DFS_CTRL_ENABLE
	imgsensor_dfs_ctrl(DFS_CTRL_DISABLE, NULL);
#endif

	platform_driver_unregister(&gseninf_platform_driver);
}

module_init(seninf_init);
module_exit(seninf_exit);

MODULE_DESCRIPTION("sensor interface driver");
MODULE_AUTHOR("Mediatek");
MODULE_LICENSE("GPL v2");

