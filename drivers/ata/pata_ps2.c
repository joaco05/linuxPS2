// SPDX-License-Identifier: GPL
/*
 * PlayStation 2 parallel ATA driver
 *
 * Copyright (C) 2006-2007  Paul Mundt
 * Copyright (C) 2016       Rick Gaiser
 * Copyright (C) 2022       Fredrik Noring
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/ata.h>
#include <linux/libata.h>
#include <linux/platform_device.h>
#include <linux/ata_platform.h>

#include <scsi/scsi_host.h>

#include <asm/mach-ps2/iop-heap.h>
#include <asm/mach-ps2/iop-memory.h>
#include <asm/mach-ps2/iop-module.h>
#include <asm/mach-ps2/iop-registers.h>
#include <asm/mach-ps2/sif.h>

#define DRV_NAME "pata-ps2"

#define MAX_ATA_SIF_SG (SIF_CMD_PACKET_DATA_MAX / sizeof(struct ata_sif_payload_sg))

/**
 * enum iop_ata_ops - IOP ATA remote operations
 * @rop_bb: Announce bounce buffer for unaligned addresses and sizes
 * @rop_sg: Request scatter-gather transfers
 * @rop_rd: Read request
 * @rop_wr: Write request
 */
enum iop_ata_rops {
	rop_bb  = 0,
	rop_sg  = 1,
	rop_rd  = 2,
	rop_wr  = 3,
};

union ata_sif_opt {
	u32 raw;
	struct {
		u32 op : 3;
		u32 count : 8;
		u32 write : 1;
		u32 : 20;
	};
};

struct ata_sif_bb {
	u32 addr;
	u32 size;
};

struct ata_sif_payload {
	struct ata_sif_payload_sg {
		u32 addr;
		u32 size;
	} sg[MAX_ATA_SIF_SG];
};

struct ata_sif_rd {
	u32 src;
	u32 dst;
	u32 size;
};

struct ata_sif_wr {
	u32 src;
	u32 dst;
	u32 size;
};

#define SPD_REGBASE		0x14000000
#define SPD_R_XFR_CTRL		0x32
#define SPD_R_0x38		0x38
#define SPD_R_IF_CTRL		0x64
#define   SPD_IF_ATA_RESET	  0x80
#define   SPD_IF_DMA_ENABLE	  0x04
#define SPD_R_PIO_MODE		0x70
#define SPD_R_MWDMA_MODE	0x72
#define SPD_R_UDMA_MODE		0x74

struct ps2_port {
	struct device *dev;
	struct ata_port *ap;

	struct {
		size_t size;
		void *data;
	} bb;
};

static void pata_ps2_dma_finished(struct ata_port *ap,
	struct ata_queued_cmd *qc)
{
	ata_sff_interrupt(IRQ_IOP_SPD_ATA0 /* FIXME */, ap->host);
}

static unsigned int pata_ps2_dma_request(struct ata_queued_cmd *qc)
{
	union ata_sif_opt opt = {
		.op = rop_sg,
		.write = (qc->tf.flags & ATA_TFLAG_WRITE) != 0,
	};
	struct ata_sif_payload payload;
	int err;

	if (opt.write) {
		WARN_ONCE("%s: Writing is provisionally disabled\n", __func__);

		return AC_ERR_INVALID;	// FIXME: Provisionally disable writing
	}

	while (qc->cursg && opt.count < MAX_ATA_SIF_SG) {
		const u32 addr = (u32)sg_dma_address(qc->cursg); /* FIXME: dma_map_sg */
		const u32 size = sg_dma_len(qc->cursg);

		if (size & 0x7) {
			/* FIXME: DEV9_DMAC_BCR can only do 8 byte sizes */
			WARN_ONCE(1, "%s: unaligned size %u\n", __func__, size);
			return AC_ERR_SYSTEM;
		}

		payload.sg[opt.count++] = (struct ata_sif_payload_sg) {
			.addr = addr,
			.size = size,
		};

		qc->cursg = sg_next(qc->cursg);
	}

	qc->ap->hsm_task_state = HSM_ST_LAST;

	err = sif_cmd_opt(SIF_CMD_ATA, opt.raw,
		&payload, opt.count * sizeof(payload.sg[0]));
	if (err < 0)
		return AC_ERR_SYSTEM;

	return AC_ERR_OK;
}

static void pata_ps2_cmd_sg(const void *payload, void *arg)
{
	struct ps2_port *pp = (struct ps2_port *)arg;
	struct ata_port *ap = pp->ap;
	struct ata_queued_cmd *qc;
	unsigned long flags;

	spin_lock_irqsave(&ap->host->lock, flags);

	qc = ata_qc_from_tag(ap, ap->link.active_tag);

	if (qc && !(qc->tf.flags & ATA_TFLAG_POLLING)) {
		if (qc->cursg) {
			pata_ps2_dma_request(qc);    // FIXME: Error handling
		} else {
			pata_ps2_dma_finished(ap, qc);

#if 0
			const u8 status = ap->ops->sff_check_altstatus(ap);

			ata_sff_hsm_move(ap, qc, status, 0);
#endif
		}
	}

	spin_unlock_irqrestore(&ap->host->lock, flags);
}

static void pata_ps2_cmd_rd(const void *payload, void *arg)
{
	const struct ata_sif_rd *rd = payload;

	if (rd->size)
		memcpy(phys_to_virt(rd->dst), phys_to_virt(rd->src), rd->size);
}

static void pata_ps2_cmd_wr(const void *payload, void *arg)
{
	const struct ata_sif_wr *wr = payload;

	sif_cmd_opt_data(SIF_CMD_ATA,	/* FIXME: Error handling */
		(union ata_sif_opt) { .op = rop_wr }.raw,
		NULL, 0, wr->dst, (const void *)wr->src, wr->size);
}

static int pata_ps2_cmd_bb(struct ps2_port *pp)
{
	/*
	 * Announce the bounce buffer for use with addresses and sizes that
	 * are nonmultiples of 16 bytes, as required for DMA transfers.
	 */
	const struct ata_sif_bb bb = {
		.addr = virt_to_phys(pp->bb.data),
		.size = pp->bb.size,
	};

	return sif_cmd_opt(SIF_CMD_ATA,
		(union ata_sif_opt) { .op = rop_bb }.raw,
		&bb, sizeof(bb));
}

static void pata_ps2_set_dmamode(struct ata_port *ap, struct ata_device *adev)
{
	u16 val;

	switch(adev->dma_mode)
	{
	case XFER_MW_DMA_0: val = 0xff; break;
	case XFER_MW_DMA_1: val = 0x45; break;
	case XFER_MW_DMA_2: val = 0x24; break;
	case XFER_UDMA_0:   val = 0xa7; break; /* UDMA16 */
	case XFER_UDMA_1:   val = 0x85; break; /* UDMA25 */
	case XFER_UDMA_2:   val = 0x63; break; /* UDMA33 */
	case XFER_UDMA_3:   val = 0x62; break; /* UDMA44 */
	case XFER_UDMA_4:   val = 0x61; break; /* UDMA66 */
	case XFER_UDMA_5:   val = 0x60; break; /* UDMA100 ??? */
	default:
		dev_err(ap->dev, "Invalid DMA mode %d\n", adev->dma_mode);
		return;
	}

	if (adev->dma_mode < XFER_UDMA_0) {
		// MWDMA
		outw(val, SPD_REGBASE + SPD_R_MWDMA_MODE);
		outw((inw(SPD_REGBASE + SPD_R_IF_CTRL) & 0xfffe) | 0x48, SPD_REGBASE + SPD_R_IF_CTRL);
	} else {
		// UDMA
		outw(val, SPD_REGBASE + SPD_R_UDMA_MODE);
		outw(inw(SPD_REGBASE + SPD_R_IF_CTRL) | 0x49, SPD_REGBASE + SPD_R_IF_CTRL);
	}
}

static void pata_ps2_set_piomode(struct ata_port *ap, struct ata_device *adev)
{
	u16 val;

	switch(adev->pio_mode)
	{
	case XFER_PIO_0: val = 0x92; break;
	case XFER_PIO_1: val = 0x72; break;
	case XFER_PIO_2: val = 0x32; break;
	case XFER_PIO_3: val = 0x24; break;
	case XFER_PIO_4: val = 0x23; break;
	default:
		dev_err(ap->dev, "Invalid PIO mode %d\n", adev->pio_mode);
		return;
	}

	outw(val, SPD_REGBASE + SPD_R_PIO_MODE);
}

static unsigned int pata_ps2_qc_issue_dma(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;

	WARN_ON(qc->tf.flags & ATA_TFLAG_POLLING);

	ap->ops->sff_tf_load(ap, &qc->tf);  /* Load TF registers. */

	qc->cursg = qc->sg;
	ap->ops->sff_exec_command(ap, &qc->tf);

	return pata_ps2_dma_request(qc);
}

static unsigned int pata_ps2_qc_issue(struct ata_queued_cmd *qc)
{
	switch (qc->tf.protocol) {
	case ATA_PROT_DMA:
		return pata_ps2_qc_issue_dma(qc);

	case ATAPI_PROT_DMA:
		dev_err(qc->ap->dev, "ATAPI DMA is not supported\n");
		return AC_ERR_INVALID;

	default:
		return ata_sff_qc_issue(qc);
	}
}

static struct scsi_host_template pata_ps2_sht = {
	ATA_PIO_SHT(DRV_NAME),
};

static struct ata_port_operations pata_ps2_port_ops = {
	.inherits		= &ata_sff_port_ops,
	.qc_prep		= ata_noop_qc_prep,
	.qc_issue		= pata_ps2_qc_issue,
	.cable_detect		= ata_cable_unknown,
	.set_piomode		= pata_ps2_set_piomode,
	.set_dmamode		= pata_ps2_set_dmamode,
};

static void pata_ps2_setup_port(struct ata_ioports *ioaddr,
	void __iomem *base, unsigned int shift)
{
	ioaddr->cmd_addr = base;
	ioaddr->ctl_addr = base + 0x1c;
	ioaddr->altstatus_addr = ioaddr->ctl_addr;

	ioaddr->data_addr    = ioaddr->cmd_addr + (ATA_REG_DATA    << shift);
	ioaddr->error_addr   = ioaddr->cmd_addr + (ATA_REG_ERR     << shift);
	ioaddr->feature_addr = ioaddr->cmd_addr + (ATA_REG_FEATURE << shift);
	ioaddr->nsect_addr   = ioaddr->cmd_addr + (ATA_REG_NSECT   << shift);
	ioaddr->lbal_addr    = ioaddr->cmd_addr + (ATA_REG_LBAL    << shift);
	ioaddr->lbam_addr    = ioaddr->cmd_addr + (ATA_REG_LBAM    << shift);
	ioaddr->lbah_addr    = ioaddr->cmd_addr + (ATA_REG_LBAH    << shift);
	ioaddr->device_addr  = ioaddr->cmd_addr + (ATA_REG_DEVICE  << shift);
	ioaddr->status_addr  = ioaddr->cmd_addr + (ATA_REG_STATUS  << shift);
	ioaddr->command_addr = ioaddr->cmd_addr + (ATA_REG_CMD     << shift);
}

static irqreturn_t pata_ps2_interrupt(int irq, void *dev)
{
	return ata_sff_interrupt(irq, dev);
}

static void pata_ps2_sif_cmd(const struct sif_cmd_header *header, void *arg)
{
	const union ata_sif_opt opt = { .raw = header->opt };

	switch (opt.op)
	{
	case rop_sg:
		pata_ps2_cmd_sg(sif_cmd_payload(header), arg);
		break;
	case rop_rd:
		pata_ps2_cmd_rd(sif_cmd_payload(header), arg);
		break;
	case rop_wr:
		pata_ps2_cmd_wr(sif_cmd_payload(header), arg);
		break;
	default:
		WARN_ONCE(1, "%s: Unknown op %d\n", __func__, opt.op);
	}
}

static int pata_ps2_probe(struct platform_device *pdev)
{
	struct resource *regs;
	struct ata_host *host;
	struct ata_port *ap;
	struct ps2_port *pp;
	void __iomem *base;
	int irq;
	int err;

	BUILD_BUG_ON(sizeof(union ata_sif_opt) != sizeof(u32));
	BUILD_BUG_ON(sizeof(struct ata_sif_payload) > SIF_CMD_PACKET_DATA_MAX); // FIXME: CMD_PACKET_PAYLOAD_MAX

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		err = irq;
		dev_err(&pdev->dev, "platform_get_irq failed width %d\n", err);
		goto err_irq;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(base)) {
		err = PTR_ERR(base);
		dev_err(&pdev->dev, "devm_ioremap_resource 0 failed with %d\n", err);
		goto err_ioremap;
	}

	pp = devm_kzalloc(&pdev->dev, sizeof(*pp), GFP_KERNEL);
	if (!pp) {
		err = -ENOMEM;
		goto err_kzalloc;
	}

	pp->bb.data = (void *)__get_free_page(GFP_DMA);
	if (!pp->bb.data) {
		err = -ENOMEM;
		goto err_bb_alloc;
	}
	pp->bb.size = 1024;	// FIXME: PAGE_SIZE;

	host = ata_host_alloc(&pdev->dev, 1);
	if (!host) {
		err = -ENOMEM;
		goto err_host_alloc;
	}

	ap = host->ports[0];
	ap->private_data = pp;

	ap->ops = &pata_ps2_port_ops;
	ap->pio_mask   = ATA_PIO4;
	ap->mwdma_mask = ATA_MWDMA2;
	ap->udma_mask  = ATA_UDMA4;	// FIXME: ATA_UDMA5?
	ap->flags     |= ATA_FLAG_NO_ATAPI;

	pp->dev = &pdev->dev;
	pp->ap = ap;

	pata_ps2_setup_port(&ap->ioaddr, base, 1);

	err = sif_request_cmd(SIF_CMD_ATA, pata_ps2_sif_cmd, pp);
	if (err)
		goto err_sif_request;

	err = iop_module_request("ata", 0x0100, NULL);
	if (err < 0)
		goto err_iop_module;

	printk("%s cmd %x ctl %x status %x irq %u\n", __func__,
		(u32)ap->ioaddr.cmd_addr,
		(u32)ap->ioaddr.ctl_addr,
		(u32)ap->ioaddr.status_addr,
		irq);

	iop_set_dma_dpcr2(IOP_DMA_DPCR2_DEV9);

	err = pata_ps2_cmd_bb(pp);
	if (err < 0)
		goto err_bb_cmd;

	return ata_host_activate(host,
		irq, pata_ps2_interrupt, IRQF_SHARED,
		&pata_ps2_sht);

err_bb_cmd:
err_iop_module:
	sif_request_cmd(SIF_CMD_ATA, NULL, NULL);	/* FIXME: sif_release_cmd(SIF_CMD_ATA) */
err_sif_request:
err_host_alloc:
err_bb_alloc:
err_kzalloc:
err_ioremap:
err_irq:
	return err;
}

static int pata_ps2_remove(struct platform_device *pdev)
{
	struct ata_host *host = dev_get_drvdata(&pdev->dev);

	ata_host_detach(host);

	return 0;
}

static struct platform_driver pata_ps2_driver = {
	.probe		= pata_ps2_probe,
	.remove		= pata_ps2_remove,
	.driver = {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(pata_ps2_driver);

MODULE_AUTHOR("Rick Gaiser");
MODULE_AUTHOR("Fredrik Noring");
MODULE_DESCRIPTION("PlayStation 2 parallel ATA driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
