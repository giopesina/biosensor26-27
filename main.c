#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>


#define ADC_NODE DT_NODELABEL(xiao_adc)

#if !DT_NODE_EXISTS(ADC_NODE)
#error "xiao_adc node not found"
#endif

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

int main(void)
{
    int err;
    int16_t samples[3] = {0, 0, 0};  /* 3 channels */

    struct adc_channel_cfg ch0_cfg = {
        .gain             = ADC_GAIN_1_4,
        .reference        = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id       = 0,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        .input_positive   = 0,
#endif
    };

    struct adc_channel_cfg ch1_cfg = {
        .gain             = ADC_GAIN_1_4,
        .reference        = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id       = 1,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        .input_positive   = 1,
#endif
    };

    struct adc_channel_cfg ch2_cfg = {
        .gain             = ADC_GAIN_1_4,
        .reference        = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id       = 2,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        .input_positive   = 2,
#endif
    };

    struct adc_sequence sequence = {
        .channels    = BIT(0) | BIT(1) | BIT(2),
        .buffer      = samples,
        .buffer_size = sizeof(samples),
        .resolution  = 12,
    };

    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return 0;
    }

    err = adc_channel_setup(adc_dev, &ch0_cfg);
    if (err < 0) { printk("CH0 setup failed: %d\n", err); return 0; }

    err = adc_channel_setup(adc_dev, &ch1_cfg);
    if (err < 0) { printk("CH1 setup failed: %d\n", err); return 0; }

    err = adc_channel_setup(adc_dev, &ch2_cfg);
    if (err < 0) { printk("CH2 setup failed: %d\n", err); return 0; }

    printk("ADC start on A0, A1, A2\n");

    while (1) {
        samples[0] = 0;
        samples[1] = 0;
        samples[2] = 0;

        err = adc_read(adc_dev, &sequence);
        if (err < 0) {
            printk("adc_read failed: %d\n", err);
        } else {
            /* A0: voltage + current */
            int32_t mv0 = (int32_t)samples[0] * 2;
            adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN_1_4, 12, &mv0);
            int v0_whole = (mv0 / 1000);
            int v0_frac  = (mv0 % 1000);
            if (v0_frac < 0) v0_frac = -v0_frac;
            int32_t i_pA0   = (mv0 * 1000) / 22;
            int     i_whole0 = i_pA0 / 1000;
            int     i_frac0  = i_pA0 % 1000;
            if (i_frac0 < 0) i_frac0 = -i_frac0;

            /* A1: voltage only */
            int32_t mv1 = (int32_t)samples[1];
            adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN_1_4, 12, &mv1);
            int v1_whole = mv1 / 1000;
            int v1_frac  = mv1 % 1000;
            if (v1_frac < 0) v1_frac = -v1_frac;
            printk("potmeter: %d.%03d V  \n", v1_whole, v1_frac);

            /* A2: voltage + current */
            int32_t mv2 = (int32_t)samples[2];
            adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN_1_4, 12, &mv2);
            int v2_whole = mv2 / 1000;
            int v2_frac  = mv2 % 1000;
            if (v2_frac < 0) v2_frac = -v2_frac;
            int32_t i_pA2   = (mv2 * 1000) / 22;
            int     i_whole2 = i_pA2 / 1000;
            int     i_frac2  = i_pA2 % 1000;
            if (i_frac2 < 0) i_frac2 = -i_frac2;

			
            if (mv1 > 1700) {
                printk("potstat: %d.%03d V  |  %d.%03d nA\n", v0_whole, v0_frac, i_whole0, i_frac0);
            } else {
                printk("potstat: -%d.%03d V  |  %d.%03d nA\n", v2_whole, v2_frac, i_whole2, i_frac2);
            }
            printk("---\n");
        }

        k_sleep(K_MSEC(100));
    }

    return 0;
}