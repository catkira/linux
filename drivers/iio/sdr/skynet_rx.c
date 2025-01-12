/*
 * SkyNET_RX
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

#define SKYNET_RX_CHAN(_ch) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = _ch, \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.output = 0, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 32, \
		.storagebits = 32, \
		.shift = 0, \
		.endianness = IIO_LE, \
	}, \
}

static const struct iio_chan_spec skynet_rx_channels[] = {
	SKYNET_RX_CHAN(0)
};

static const struct sdr_chip_info skynet_rx_chip_info = {
	.channels = skynet_rx_channels,
	.num_channels = ARRAY_SIZE(skynet_rx_channels),
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

static int skynet_rx_hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	// struct iio_dev *indio_dev = queue->driver_data;
	// struct axiadc_state *st = iio_priv(indio_dev);

	block->block.bytes_used = block->block.size;

	iio_dmaengine_buffer_submit_block(queue, block, DMA_FROM_DEVICE);

	return 0;
}

static const struct iio_dma_buffer_ops axiadc_dma_buffer_ops = {
	.submit = skynet_rx_hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static int axiadc_configure_ring_stream(struct iio_dev *indio_dev,
	const char *dma_name)
{
	struct iio_buffer *buffer;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, dma_name,
						 &axiadc_dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
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
        *val = 69;
        *val2 = 69;
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
	
    return sysfs_emit(buf, "%d\n", readval);
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

// frame_sync regmap
static IIO_DEVICE_ATTR(fs_status, S_IRUGO,
	show_reg, NULL, 0xC014 - 0x4000);
static IIO_DEVICE_ATTR(rgf_overflow, S_IRUGO,
	show_reg, NULL, 0xC01C - 0x4000);
static IIO_DEVICE_ATTR(num_disconnects, S_IRUGO,
	show_reg, NULL, 0xC034 - 0x4000);
static IIO_DEVICE_ATTR(allowed_ssb_misses, S_IWUSR | S_IRUGO,
	show_reg, set_reg_int, 0xC056 - 0x4000);

// pss_detector regmap
static IIO_DEVICE_ATTR(detection_shift, S_IRUGO,
	show_reg, NULL, 0x4030 - 0x4000);
static IIO_DEVICE_ATTR(require_single_peak, S_IWUSR | S_IRUGO,
	show_reg, set_reg_int, 0x403C - 0x4000);

// receiver regmap
static IIO_DEVICE_ATTR(git_hash, S_IRUGO,
	show_reg_hex, NULL, 0x8014 - 0x4000);
static IIO_DEVICE_ATTR(n_id, S_IRUGO,
	show_reg, NULL, 0x8020 - 0x4000);
static IIO_DEVICE_ATTR(nfft, S_IRUGO,
	show_reg, NULL, 0x8048 - 0x4000);
static IIO_DEVICE_ATTR(dna_low, S_IRUGO,
	show_reg, NULL, 0x8040 - 0x4000);
static IIO_DEVICE_ATTR(dna_high, S_IRUGO,
	show_reg, NULL, 0x8044 - 0x4000);

static struct attribute *skynet_rx_attributes[] = {
	&iio_dev_attr_git_hash.dev_attr.attr,
	&iio_dev_attr_fs_status.dev_attr.attr,
	&iio_dev_attr_rgf_overflow.dev_attr.attr,
	&iio_dev_attr_n_id.dev_attr.attr,
	&iio_dev_attr_num_disconnects.dev_attr.attr,
	&iio_dev_attr_detection_shift.dev_attr.attr,
	&iio_dev_attr_allowed_ssb_misses.dev_attr.attr,
	&iio_dev_attr_require_single_peak.dev_attr.attr,
	&iio_dev_attr_nfft.dev_attr.attr,
	&iio_dev_attr_dna_low.dev_attr.attr,
	&iio_dev_attr_dna_high.dev_attr.attr,
	NULL,
};

static const struct attribute_group skynet_rx_group = {
	.attrs = skynet_rx_attributes,
};

static const struct iio_info sdr_info = {
	.read_raw = skynet_tx_read_raw,
	.write_raw = skynet_tx_write_raw,
	.debugfs_reg_access = &sdr_reg_access,
    .attrs = &skynet_rx_group,
	// .update_scan_mode = axiadc_update_scan_mode,
};

static const struct of_device_id sdr_of_match[] = {
	{ .compatible = "catkira,skynet_rx-1.00.a", .data = &skynet_rx_chip_info },
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

	ret = axiadc_configure_ring_stream(indio_dev, "rx");
	if (ret)
		return ret;

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
MODULE_DESCRIPTION("SkyNET RX");
MODULE_LICENSE("closed source");
