/*
 * SkyNET_TX
 *
 * Copyright 2023 Benjamin Menkuec
 *
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/hw-consumer.h>

#include <linux/dma-direction.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#define AXI_REG_VERSION 0
#define MAX_CHANNEL 128

#define AXI_PCORE_VER(major, minor, patch)	\
	(((major) << 16) | ((minor) << 8) | (patch))
#define AXI_PCORE_VER_MAJOR(version)	(((version) >> 16) & 0xff)
#define AXI_PCORE_VER_MINOR(version)	(((version) >> 8) & 0xff)
#define AXI_PCORE_VER_PATCH(version)	((version) & 0xff)

struct sdr_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
    unsigned int version;
};

struct axiadc_state {
	void __iomem			*regs;
	/* protect against device accesses */
	struct mutex			lock;
	unsigned int			max_usr_channel;
	struct iio_chan_spec		channels[MAX_CHANNEL];
    unsigned int            pcore_version;
};

#define SKYNET_TX_CHAN(_ch) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = _ch, \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.output = 1, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 32, \
		.storagebits = 32, \
		.shift = 0, \
		.endianness = IIO_LE, \
	}, \
}

static const struct iio_chan_spec skynet_tx_channels[] = {
	SKYNET_TX_CHAN(0)
};

static const struct sdr_chip_info skynet_tx_chip_info = {
	.channels = skynet_tx_channels,
	.num_channels = ARRAY_SIZE(skynet_tx_channels),
    .version = AXI_PCORE_VER(1, 0, 'a'),
};


static inline void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int axiadc_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int skynet_tx_hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	// struct iio_dev *indio_dev = queue->driver_data;
	// struct axiadc_state *st = iio_priv(indio_dev);

	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_TO_DEVICE);
}

static const struct iio_dma_buffer_ops skynet_tx_buffer_ops = {
	.submit = skynet_tx_hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static int configure_buffer(struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "tx",
						 &skynet_tx_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	indio_dev->direction = IIO_DEVICE_DIRECTION_OUT;
	iio_device_attach_buffer(indio_dev, buffer);

	return 0;
}

static int skynet_tx_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	// struct axiadc_state *st = iio_priv(indio_dev);
	// uint32_t reg;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
        *val = 6969;
        *val2 = 6969;
		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static int skynet_tx_write_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int val, int val2, long info)
{
	// struct axiadc_state *st = iio_priv(indio_dev);
	// uint32_t reg;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return 0;
	default:
		break;
	}

	return -EINVAL;
}

static int sdr_reg_access(struct iio_dev *indio_dev,
			  unsigned int reg, unsigned int writeval,
			  unsigned int *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
    if (readval == NULL) {
        axiadc_write(st, reg & 0xFFFF, writeval);
    }
    else {
        *readval = axiadc_read(st, reg & 0xFFFF);
    }
	mutex_unlock(&st->lock);

	return 0;
}

static ssize_t show_reg(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
    unsigned int readval = axiadc_read(st, (u32)this_attr->address);
	
    return sysfs_emit(buf, "%u\n", readval);
}

static ssize_t show_reg_hex(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
    unsigned int readval = axiadc_read(st, (u32)this_attr->address);
	
    return sysfs_emit(buf, "%x\n", readval);
}

static ssize_t set_reg_int(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
    unsigned int writeval;
    int ret = kstrtouint(buf, 10, &writeval);
    if (ret)
		return ret;
    axiadc_write(st, (u32)this_attr->address, writeval);
	return len;
}

static IIO_DEVICE_ATTR(source_select, S_IWUSR | S_IRUGO,
	show_reg, set_reg_int, 0x14);

static IIO_DEVICE_ATTR(pdsch_encoder_in_state, S_IRUGO,
	show_reg, NULL, 0x18);

static IIO_DEVICE_ATTR(pdsch_encoder_out_state, S_IRUGO,
	show_reg, NULL, 0x1c);

static IIO_DEVICE_ATTR(pdsch_encoder_out_2_state, S_IRUGO,
	show_reg, NULL, 0x20);

static IIO_DEVICE_ATTR(pdsch_encoder_has_data, S_IRUGO,
	show_reg, NULL, 0x24);

static IIO_DEVICE_ATTR(underflow, S_IRUGO,
	show_reg, NULL, 0x28);

static IIO_DEVICE_ATTR(pdsch_encoder_blk_cnt_in, S_IRUGO,
	show_reg, NULL, 0x30);

static IIO_DEVICE_ATTR(pdsch_encoder_blk_cnt_out, S_IRUGO,
	show_reg, NULL, 0x2c);

static IIO_DEVICE_ATTR(ldpc_cnt_in, S_IRUGO,
	show_reg, NULL, 0x34);

static IIO_DEVICE_ATTR(ldpc_cnt_out, S_IRUGO,
	show_reg, NULL, 0x38);

static IIO_DEVICE_ATTR(ldpc_1st_in_byte, S_IRUGO,
	show_reg, NULL, 0x3C);

static IIO_DEVICE_ATTR(ldpc_1st_out_byte, S_IRUGO,
	show_reg, NULL, 0x40);

static IIO_DEVICE_ATTR(ldpc_1st_in_byte2, S_IRUGO,
	show_reg, NULL, 0x44);

static IIO_DEVICE_ATTR(ldpc_1st_out_byte2, S_IRUGO,
	show_reg, NULL, 0x48);

static IIO_DEVICE_ATTR(ssb_sym, S_IWUSR | S_IRUGO,
	show_reg, NULL, 0x4C);

static IIO_DEVICE_ATTR(pkt_cnt_eth, S_IRUGO,
	show_reg, NULL, 0x50);

static IIO_DEVICE_ATTR(pkt_cnt_ip, S_IRUGO,
	show_reg, NULL, 0x54);

static IIO_DEVICE_ATTR(pkt_cnt_udp, S_IRUGO,
	show_reg, NULL, 0x58);

static IIO_DEVICE_ATTR(eth_status, S_IRUGO,
	show_reg, NULL, 0x5C);

static IIO_DEVICE_ATTR(ip_status, S_IRUGO,
	show_reg, NULL, 0x60);

static IIO_DEVICE_ATTR(eth_type, S_IRUGO,
	show_reg, NULL, 0x64);

static IIO_DEVICE_ATTR(pkt_cnt_ip_fail, S_IRUGO,
	show_reg, NULL, 0x68);

static IIO_DEVICE_ATTR(pkt_cnt_ip_match, S_IRUGO,
	show_reg, NULL, 0x6C);

static IIO_DEVICE_ATTR(last_dest_ip, S_IRUGO,
	show_reg, NULL, 0x70);

static IIO_DEVICE_ATTR(enable_control_tx, S_IWUSR | S_IRUGO,
	show_reg, set_reg_int, 0x74);

static IIO_DEVICE_ATTR(enable_video_tx, S_IWUSR | S_IRUGO,
	show_reg, set_reg_int, 0x78);

static IIO_DEVICE_ATTR(debug_in_byte_cnt, S_IRUGO,
	show_reg, NULL, 0x7C);

static IIO_DEVICE_ATTR(mcs, S_IRUGO,
	show_reg, NULL, 0x80);

static IIO_DEVICE_ATTR(tbs, S_IRUGO,
	show_reg, NULL, 0x84);

static IIO_DEVICE_ATTR(num_cb, S_IRUGO,
	show_reg, NULL, 0x88);

static IIO_DEVICE_ATTR(zc, S_IRUGO,
	show_reg, NULL, 0x8C);

static IIO_DEVICE_ATTR(pkt_cnt_crsf, S_IRUGO,
	show_reg, NULL, 0x90);

static IIO_DEVICE_ATTR(aes_key_0, S_IWUSR | S_IRUGO,
	show_reg_hex, set_reg_int, 0x100);
static IIO_DEVICE_ATTR(aes_key_1, S_IWUSR | S_IRUGO,
	show_reg_hex, set_reg_int, 0x104);
static IIO_DEVICE_ATTR(aes_key_2, S_IWUSR | S_IRUGO,
	show_reg_hex, set_reg_int, 0x108);
static IIO_DEVICE_ATTR(aes_key_3, S_IWUSR | S_IRUGO,
	show_reg_hex, set_reg_int, 0x10C);
static IIO_DEVICE_ATTR(aes_key_4, S_IWUSR | S_IRUGO,
	show_reg_hex, set_reg_int, 0x110);
static IIO_DEVICE_ATTR(aes_key_5, S_IWUSR | S_IRUGO,
	show_reg_hex, set_reg_int, 0x114);
static IIO_DEVICE_ATTR(aes_key_6, S_IWUSR | S_IRUGO,
	show_reg_hex, set_reg_int, 0x118);
static IIO_DEVICE_ATTR(aes_key_7, S_IWUSR | S_IRUGO,
	show_reg_hex, set_reg_int, 0x11C);

static IIO_DEVICE_ATTR(aes_id, S_IWUSR | S_IRUGO,
	show_reg, set_reg_int, 0x120);

static IIO_DEVICE_ATTR(aes_enable, S_IWUSR | S_IRUGO,
	show_reg, set_reg_int, 0x124);

static IIO_DEVICE_ATTR(update_aes_key_request, S_IWUSR | S_IRUGO,
	show_reg, set_reg_int, 0x140);

static IIO_DEVICE_ATTR(tx_port, S_IWUSR | S_IRUGO,
	show_reg, set_reg_int, 0x144);

static IIO_DEVICE_ATTR(precoding, S_IWUSR | S_IRUGO,
	show_reg, set_reg_int, 0x148);

static struct attribute *skynet_tx_attributes[] = {
	&iio_dev_attr_source_select.dev_attr.attr,
	&iio_dev_attr_pdsch_encoder_in_state.dev_attr.attr,
	&iio_dev_attr_pdsch_encoder_out_state.dev_attr.attr,
	&iio_dev_attr_pdsch_encoder_out_2_state.dev_attr.attr,
	&iio_dev_attr_pdsch_encoder_has_data.dev_attr.attr,
	&iio_dev_attr_underflow.dev_attr.attr,
	&iio_dev_attr_pdsch_encoder_blk_cnt_in.dev_attr.attr,
	&iio_dev_attr_pdsch_encoder_blk_cnt_out.dev_attr.attr,
	&iio_dev_attr_ldpc_cnt_in.dev_attr.attr,
	&iio_dev_attr_ldpc_cnt_out.dev_attr.attr,
	&iio_dev_attr_ldpc_1st_in_byte.dev_attr.attr,
	&iio_dev_attr_ldpc_1st_out_byte.dev_attr.attr,
	&iio_dev_attr_ldpc_1st_in_byte2.dev_attr.attr,
	&iio_dev_attr_ldpc_1st_out_byte2.dev_attr.attr,
	&iio_dev_attr_ssb_sym.dev_attr.attr,
	&iio_dev_attr_pkt_cnt_eth.dev_attr.attr,
	&iio_dev_attr_pkt_cnt_ip.dev_attr.attr,
	&iio_dev_attr_pkt_cnt_udp.dev_attr.attr,
	&iio_dev_attr_eth_status.dev_attr.attr,
	&iio_dev_attr_ip_status.dev_attr.attr,
	&iio_dev_attr_eth_type.dev_attr.attr,
	&iio_dev_attr_pkt_cnt_ip_fail.dev_attr.attr,
	&iio_dev_attr_pkt_cnt_ip_match.dev_attr.attr,
	&iio_dev_attr_last_dest_ip.dev_attr.attr,
	&iio_dev_attr_enable_control_tx.dev_attr.attr,
	&iio_dev_attr_enable_video_tx.dev_attr.attr,
	&iio_dev_attr_debug_in_byte_cnt.dev_attr.attr,
	&iio_dev_attr_mcs.dev_attr.attr,
	&iio_dev_attr_tbs.dev_attr.attr,
	&iio_dev_attr_num_cb.dev_attr.attr,
	&iio_dev_attr_zc.dev_attr.attr,
	&iio_dev_attr_pkt_cnt_crsf.dev_attr.attr,
	&iio_dev_attr_aes_key_0.dev_attr.attr,
	&iio_dev_attr_aes_key_1.dev_attr.attr,
	&iio_dev_attr_aes_key_2.dev_attr.attr,
	&iio_dev_attr_aes_key_3.dev_attr.attr,
	&iio_dev_attr_aes_key_4.dev_attr.attr,
	&iio_dev_attr_aes_key_5.dev_attr.attr,
	&iio_dev_attr_aes_key_6.dev_attr.attr,
	&iio_dev_attr_aes_key_7.dev_attr.attr,
	&iio_dev_attr_aes_id.dev_attr.attr,
	&iio_dev_attr_aes_enable.dev_attr.attr,
	&iio_dev_attr_update_aes_key_request.dev_attr.attr,
	&iio_dev_attr_tx_port.dev_attr.attr,
	&iio_dev_attr_precoding.dev_attr.attr,
	NULL,
};

static const struct attribute_group skynet_tx_group = {
	.attrs = skynet_tx_attributes,
};

static const struct iio_info sdr_info = {
	.read_raw = skynet_tx_read_raw,
	.write_raw = skynet_tx_write_raw,
	.debugfs_reg_access = &sdr_reg_access,
    .attrs = &skynet_tx_group,
	// .update_scan_mode = axiadc_update_scan_mode,
};

static const struct of_device_id sdr_of_match[] = {
	{ .compatible = "catkira,skynet_tx-1.00.a", .data = &skynet_tx_chip_info },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, sdr_of_match);

static const struct iio_chan_spec dummy_channels[] = {
	{}
};

static int sdr_probe(struct platform_device *pdev)
{
	const struct sdr_chip_info *info;
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	struct axiadc_state *st;
	struct resource *mem;
	int ret;

	id = of_match_node(sdr_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	info = id->data;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	mutex_init(&st->lock);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &sdr_info;

	/* Reset all HDL Cores */
	// axiadc_write(st, ADI_REG_RSTN, 0);
	// axiadc_write(st, ADI_REG_RSTN, ADI_RSTN);

    st->pcore_version = axiadc_read(st, AXI_REG_VERSION);
	if (AXI_PCORE_VER_MAJOR(st->pcore_version) >
		AXI_PCORE_VER_MAJOR(info->version)) {
		dev_err(&pdev->dev, "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			AXI_PCORE_VER_MAJOR(info->version),
			AXI_PCORE_VER_MINOR(info->version),
			AXI_PCORE_VER_PATCH(info->version),
			AXI_PCORE_VER_MAJOR(st->pcore_version),
			AXI_PCORE_VER_MINOR(st->pcore_version),
			AXI_PCORE_VER_PATCH(st->pcore_version));
		return -ENODEV;
	}

    indio_dev->channels = info->channels;
    indio_dev->num_channels = info->num_channels;

	ret = configure_buffer(indio_dev);
	if (ret)
		return ret;

	dev_info(&pdev->dev,
		 "Skynet TX (%d.%.2d.%c) at 0x%08llX mapped to 0x%p\n",
		 AXI_PCORE_VER_MAJOR(st->pcore_version),
		 AXI_PCORE_VER_MINOR(st->pcore_version),
		 AXI_PCORE_VER_PATCH(st->pcore_version),
		 (unsigned long long)mem->start, st->regs);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static struct platform_driver sdr_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = sdr_of_match,
	},
	.probe	  = sdr_probe,
};

module_platform_driver(sdr_driver);

MODULE_AUTHOR("Benjamin Menkuec <benjamin@menkuec.de>");
MODULE_DESCRIPTION("SkyNET TX");
MODULE_LICENSE("closed source");
