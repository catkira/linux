#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>


#define FAKE_VOLTAGE_CHANNEL(num)                  \
   {                                               \
         .type = IIO_VOLTAGE,                      \
         .indexed = 1,                             \
         .channel = (num),                         \
         .address = (num),                         \
         .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),   \
         .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
   }


struct my_private_data {
    int gpio_ext_ref_lock;
    int gpio_ext_is_pps;
    int ref_sel;
};


static int fake_read_raw(struct iio_dev *indio_dev,
                   struct iio_chan_spec const *channel, int *val,
                   int *val2, long mask)
{
    struct my_private_data *gpio = iio_priv(indio_dev);
    switch (mask)
    {
    case IIO_CHAN_INFO_RAW:
        /* code */
        if(channel->channel == 0){
            *val = gpio_get_value(gpio->gpio_ext_is_pps);
        }
        if(channel->channel == 1){
            *val = gpio_get_value(gpio->ref_sel);
        }
        break;
    case IIO_CHAN_INFO_SCALE:
        *val = gpio_get_value(gpio->gpio_ext_ref_lock);
        break;
    default:
        break;
    }
    return IIO_VAL_INT;
}

static int fake_write_raw(struct iio_dev *indio_dev,
                   struct iio_chan_spec const *chan,
                   int val, int val2, long mask)
{
    struct my_private_data *gpio = iio_priv(indio_dev);
    switch (mask)
    {
    case IIO_CHAN_INFO_RAW:
        if(chan->channel == 0){
            gpio_set_value(gpio->gpio_ext_is_pps,val);
        }
        if(chan->channel == 1){
           gpio_set_value(gpio->ref_sel,val);
        }
        break;
    case IIO_CHAN_INFO_SCALE:
         break;
    default:
        break;
    }
    return 0;

}

static const struct iio_chan_spec fake_channels[] = {
   FAKE_VOLTAGE_CHANNEL(0),
   FAKE_VOLTAGE_CHANNEL(1), 
};

static const struct of_device_id iio_dummy_ids[] = {
    { .compatible = "microphase,ltc2630", },
    { /* sentinel */ }
};

static const struct iio_info fake_iio_info = {
   .read_raw = fake_read_raw,
   .write_raw        = fake_write_raw,
};

static int init_gpio(struct iio_dev *indio_dev){
    struct my_private_data *data;
    struct device_node *nd;
    int ret;

    data = iio_priv(indio_dev);

    nd = of_find_node_by_path("/ltc2630");
    if(NULL == nd){
        printk(KERN_EMERG "ltc:get failed\n");
        return -EINVAL;
    }

    data->gpio_ext_is_pps = of_get_named_gpio(nd,"ext_ref_ispps",0);
    if(!gpio_is_valid(data->gpio_ext_is_pps)){
        printk(KERN_EMERG "gpio:gpio_ext_is_pps get failed\n");
        return -EINVAL;
    }

    data->gpio_ext_ref_lock = of_get_named_gpio(nd,"ext_ref_locked",0);
    if(!gpio_is_valid(data->gpio_ext_ref_lock)){
        printk(KERN_EMERG "gpio:ext_ref_locked get failed\n");
        return -EINVAL;
    }

    data->ref_sel = of_get_named_gpio(nd,"ref_sel",0);
    if(!gpio_is_valid(data->ref_sel)){
        printk(KERN_EMERG "gpio:ref_sel get failed\n");
        return -EINVAL;
    }

    ret = gpio_request(data->gpio_ext_is_pps,"ext_is_pps");
    if(ret){
        printk(KERN_EMERG "gpio:ext_is_pps request failed %d\n",data->gpio_ext_is_pps);
        return ret;
    }
     ret = gpio_request(data->gpio_ext_ref_lock,"ext_ref_lock");
    if(ret){
        printk(KERN_EMERG "gpio:ext_ref_lock request failed\n");
        return ret;
    }
     ret = gpio_request(data->ref_sel,"ref_sel");
    if(ret){
        printk(KERN_EMERG "gpio:ref_sel request failed\n");
        return ret;
    }

    gpio_direction_output(data->ref_sel,1);
    gpio_direction_output(data->gpio_ext_is_pps,0);
    gpio_direction_input(data->gpio_ext_ref_lock);    

    return 0;

}

static void free_gpio(struct iio_dev *indio_dev){
    struct my_private_data *data = iio_priv(indio_dev);

    gpio_free(data->gpio_ext_is_pps);
    gpio_free(data->gpio_ext_ref_lock);
    gpio_free(data->ref_sel);
}

static int my_pdrv_probe (struct platform_device *pdev)
{
    struct iio_dev *indio_dev;
    struct my_private_data *data;
   indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));

   if (!indio_dev) {
         dev_err(&pdev->dev, "iio allocation failed!\n");
         return -ENOMEM;
   }
    init_gpio(indio_dev);
   data = iio_priv(indio_dev);
   indio_dev->dev.parent = &pdev->dev;
   indio_dev->info = &fake_iio_info;
   indio_dev->name = "ref-pll";
   indio_dev->modes = INDIO_DIRECT_MODE;
   indio_dev->channels = fake_channels;
   indio_dev->num_channels = ARRAY_SIZE(fake_channels);
    iio_device_register(indio_dev);
    platform_set_drvdata(pdev, indio_dev);
    return 0;
}
static int my_pdrv_remove(struct platform_device *pdev)
{
    struct iio_dev *indio_dev = platform_get_drvdata(pdev);
    iio_device_unregister(indio_dev);
    free_gpio(indio_dev);
    return 0;
}
static struct platform_driver mypdrv = {
    .probe      = my_pdrv_probe,
    .remove     = my_pdrv_remove,
    .driver     = {
        .name     = KBUILD_MODNAME,
        .of_match_table = of_match_ptr(iio_dummy_ids),  
        .owner    = THIS_MODULE,
    },
};
module_platform_driver(mypdrv);
MODULE_AUTHOR("lone_boy 995586238@qq.com");
MODULE_LICENSE("GPL");

