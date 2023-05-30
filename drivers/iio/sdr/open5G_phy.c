/*
 * open5G_phy
 *
 * Copyright 2023 Benjamin Menkuec
 *
 * Licensed under the GPL-2.
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

#define SDR_CHANNEL(_address, _type, _ch, _mod, _rb) { \
	.type = _type, \
	.indexed = 1, \
	.channel = _ch, \
	.modified = (_mod == 0) ? 0 : 1, \
	.channel2 = _mod, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.address = _address, \
	.scan_index = _address, \
	.scan_type = { \
		.sign = (_type == IIO_ANGL) ? 'u' : 's', \
		.realbits = _rb, \
		.storagebits = 32, \
		.shift = 0, \
		.endianness = IIO_LE, \
	}, \
}

static const struct iio_chan_spec open5G_channels[] = {
	SDR_CHANNEL(0, IIO_ANGL, 0, 0, 32),
	SDR_CHANNEL(1, IIO_VOLTAGE, 0, 0, 24)
};

static const struct sdr_chip_info open5G_chip_info = {
	.channels = open5G_channels,
	.num_channels = ARRAY_SIZE(open5G_channels),
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

static int axiadc_hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = queue->driver_data;
	struct axiadc_state *st = iio_priv(indio_dev);

	block->block.bytes_used = block->block.size;

	iio_dmaengine_buffer_submit_block(queue, block, DMA_FROM_DEVICE);

	return 0;
}

static const struct iio_dma_buffer_ops axiadc_dma_buffer_ops = {
	.submit = axiadc_hw_submit_block,
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

static int axiadc_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	uint32_t reg;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
        *val = 69;
        *val2 = 69;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
        *val = 69;
        *val2 = 69;
		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static int axiadc_write_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int val, int val2, long info)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	uint32_t reg;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return 0;
	case IIO_CHAN_INFO_CALIBSCALE:
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
    if (readval == NULL)
        axiadc_write(st, reg & 0xFFFF, writeval);
    else
        *readval = axiadc_read(st, reg & 0xFFFF);
	mutex_unlock(&st->lock);

	return 0;
}

static const struct iio_info sdr_info = {
	.read_raw = axiadc_read_raw,
	.write_raw = axiadc_write_raw,
	.debugfs_reg_access = &sdr_reg_access,
	// .update_scan_mode = axiadc_update_scan_mode,
};

static const struct of_device_id sdr_of_match[] = {
	{ .compatible = "catkira,open5G_phy-1.00.a", .data = &open5G_chip_info },
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
MODULE_DESCRIPTION("open5G_phy");
MODULE_LICENSE("GPL v2");
