/*
 * Analog Devices ADC Module
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/clk.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/hw-consumer.h>

#include <linux/dma-direction.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

/* ADC Common */
#define ADI_REG_RSTN			0x0040
#define ADI_RSTN			(1 << 0)

#define ADI_REG_STATUS			0x005C
#define ADI_REG_DMA_STATUS		0x0088
#define ADI_REG_USR_CNTRL_1		0x00A0

/* ADC Channel */
#define ADI_REG_CHAN_CNTRL(c)		(0x0400 + (c) * 0x40)
#define ADI_IQCOR_ENB			(1 << 9)
#define ADI_FORMAT_SIGNEXT		(1 << 6)
#define ADI_FORMAT_ENABLE		(1 << 4)
#define ADI_ENABLE			(1 << 0)

#define ADI_REG_CHAN_CNTRL_2(c)		(0x0414 + (c) * 0x40)

#define ADI_REG_CORRECTION_ENABLE 0x48
#define ADI_REG_CORRECTION_COEFFICIENT(x) (0x4c + (x) * 4)

#define ADI_USR_CHANMAX(x)		(((x) & 0xFF) << 0)
#define ADI_TO_USR_CHANMAX(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_CHAN_USR_CNTRL_1(c)		(0x0420 + (c) * 0x40)
#define ADI_USR_DATATYPE_BE			(1 << 25)
#define ADI_USR_DATATYPE_SIGNED			(1 << 24)
#define ADI_USR_DATATYPE_SHIFT(x)		(((x) & 0xFF) << 16)
#define ADI_TO_USR_DATATYPE_SHIFT(x)		(((x) >> 16) & 0xFF)
#define ADI_USR_DATATYPE_TOTAL_BITS(x)		(((x) & 0xFF) << 8)
#define ADI_TO_USR_DATATYPE_TOTAL_BITS(x)	(((x) >> 8) & 0xFF)
#define ADI_USR_DATATYPE_BITS(x)			(((x) & 0xFF) << 0)
#define ADI_TO_USR_DATATYPE_BITS(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_CHAN_USR_CNTRL_2(c)		(0x0424 + (c) * 0x40)
#define ADI_USR_DECIMATION_M(x)			(((x) & 0xFFFF) << 16)
#define ADI_TO_USR_DECIMATION_M(x)		(((x) >> 16) & 0xFFFF)
#define ADI_USR_DECIMATION_N(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_USR_DECIMATION_N(x)		(((x) >> 0) & 0xFFFF)

#define ADI_MAX_CHANNEL			128

struct sdr_chip_info {
	bool has_no_sample_clk;
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	unsigned int ctrl_flags;
};

struct axiadc_state {
	void __iomem			*regs;
	void __iomem			*slave_regs;
	struct clk 			*clk;
	/* protect against device accesses */
	struct mutex			lock;
	unsigned int			adc_def_output_mode;
	unsigned int			max_usr_channel;
	struct iio_chan_spec		channels[ADI_MAX_CHANNEL];
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
	.has_no_sample_clk = false,
	.channels = open5G_channels,
	.num_channels = ARRAY_SIZE(open5G_channels),
};

static inline void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int axiadc_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static inline void axiadc_slave_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->slave_regs + reg);
}

static inline unsigned int axiadc_slave_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->slave_regs + reg);
}

static int axiadc_hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = queue->driver_data;
	struct axiadc_state *st = iio_priv(indio_dev);

	block->block.bytes_used = block->block.size;

	iio_dmaengine_buffer_submit_block(queue, block, DMA_FROM_DEVICE);

	axiadc_write(st, ADI_REG_STATUS, ~0);
	axiadc_write(st, ADI_REG_DMA_STATUS, ~0);

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

static int axiadc_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned int i, ctrl;

	for (i = 0; i < indio_dev->masklength; i++) {
		ctrl = axiadc_read(st, ADI_REG_CHAN_CNTRL(i));

		if (test_bit(i, scan_mask))
			ctrl |= ADI_ENABLE;
		else
			ctrl &= ~ADI_ENABLE;

		ctrl |= st->adc_def_output_mode;

		axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ctrl);
	}

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
	if (st->slave_regs && (reg & 0x80000000)) {
		if (readval == NULL)
			axiadc_slave_write(st, (reg & 0xffff), writeval);
		else
			*readval = axiadc_slave_read(st, (reg & 0xffff));
	} else {
		if (readval == NULL)
			axiadc_write(st, reg & 0xFFFF, writeval);
		else
			*readval = axiadc_read(st, reg & 0xFFFF);
	}
	mutex_unlock(&st->lock);

	return 0;
}

static const struct iio_info sdr_info = {
	.read_raw = axiadc_read_raw,
	.write_raw = axiadc_write_raw,
	.debugfs_reg_access = &sdr_reg_access,
	.update_scan_mode = axiadc_update_scan_mode,
};

static const struct of_device_id sdr_of_match[] = {
	{ .compatible = "catkira,open5G_phy-1.00.a", .data = &open5G_chip_info },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, sdr_of_match);

static void adc_clk_disable(void *clk)
{
	clk_disable_unprepare(clk);
}

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

	st->clk = devm_clk_get(&pdev->dev, "sampl_clk");
	if (IS_ERR(st->clk)) {
		if (!info->has_no_sample_clk)
			return PTR_ERR(st->clk);
	} else {
		ret = clk_prepare_enable(st->clk);
		if (ret)
			return ret;

		ret = devm_add_action_or_reset(&pdev->dev, adc_clk_disable, st->clk);
		if (ret)
			return ret;
	}

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
	axiadc_write(st, ADI_REG_RSTN, 0);
	axiadc_write(st, ADI_REG_RSTN, ADI_RSTN);

	st->adc_def_output_mode = info->ctrl_flags;

    indio_dev->channels = info->channels;
    indio_dev->num_channels = info->num_channels;

	ret = axiadc_configure_ring_stream(indio_dev, "rx");
	if (ret)
		return ret;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static struct platform_driver adc_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = sdr_of_match,
	},
	.probe	  = sdr_probe,
};

module_platform_driver(adc_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADC");
MODULE_LICENSE("GPL v2");
