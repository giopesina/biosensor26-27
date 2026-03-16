#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>

/* ── ADC ─────────────────────────────────────────────────────── */
#define ADC_NODE DT_NODELABEL(xiao_adc)
#if !DT_NODE_EXISTS(ADC_NODE)
#error "xiao_adc node not found"
#endif
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

/* ── PWM analog out on D1 (P1.05) via pwm20 channel 0 ───────── */
static const struct device *pwm_dev;
#define PWM_CHANNEL    0
#define PWM_FLAGS      0
#define PWM_PERIOD_NS  100000UL              /* 10 kHz */
#define SWEEP_STEP_NS  (PWM_PERIOD_NS / 100) /* 1% per step */

/* ── PWM sweep state ─────────────────────────────────────────── */
static uint32_t sweep_duty_ns = 0;
static int      sweep_dir     = 1;

static void sweep_step(void)
{
    if (sweep_dir == 1) {
        sweep_duty_ns += SWEEP_STEP_NS;
        if (sweep_duty_ns >= PWM_PERIOD_NS) {
            sweep_duty_ns = PWM_PERIOD_NS;
            sweep_dir = -1;
        }
    } else {
        if (sweep_duty_ns >= SWEEP_STEP_NS) {
            sweep_duty_ns -= SWEEP_STEP_NS;
        } else {
            sweep_duty_ns = 0;
            sweep_dir = 1;
        }
    }
    pwm_set(pwm_dev, PWM_CHANNEL, PWM_PERIOD_NS, sweep_duty_ns, PWM_FLAGS);
}

/* ── main ────────────────────────────────────────────────────── */
int main(void)
{
    int err;
    int16_t samples[2] = {0, 0};

    struct adc_channel_cfg ch0_cfg = {
        .gain = ADC_GAIN_1_4, .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT, .channel_id = 0,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        .input_positive = 0,
#endif
    };
    struct adc_channel_cfg ch2_cfg = {
        .gain = ADC_GAIN_1_4, .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT, .channel_id = 2,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        .input_positive = 2,
#endif
    };
    struct adc_sequence sequence = {
        .channels    = BIT(0) | BIT(2),
        .buffer      = samples,
        .buffer_size = sizeof(samples),
        .resolution  = 12,
    };

    /* ADC init */
    if (!device_is_ready(adc_dev)) { printk("ADC not ready\n"); return 0; }
    err = adc_channel_setup(adc_dev, &ch0_cfg);
    if (err < 0) { printk("CH0 failed: %d\n", err); return 0; }
    err = adc_channel_setup(adc_dev, &ch2_cfg);
    if (err < 0) { printk("CH2 failed: %d\n", err); return 0; }

    /* PWM init */
    pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm20));
    if (!device_is_ready(pwm_dev)) { printk("PWM not ready\n"); return 0; }
    pwm_set(pwm_dev, PWM_CHANNEL, PWM_PERIOD_NS, 0, PWM_FLAGS);
    printk("PWM analog out on D1, ADC on A0 and A2\n");

    while (1) {
        sweep_step();

        /* sweep voltage with -1.7V offset so 0% duty = -1.700V */
        int32_t sweep_mv = (int32_t)((3300UL * sweep_duty_ns) / PWM_PERIOD_NS);
        sweep_mv -= 1700;
        int sw_whole = sweep_mv / 1000;
        int sw_frac  = sweep_mv % 1000;
        if (sw_frac < 0) sw_frac = -sw_frac;

        samples[0] = 0;
        samples[1] = 0;

        err = adc_read(adc_dev, &sequence);
        if (err < 0) {
            printk("adc_read failed: %d\n", err);
        } else {
            /* A0: voltage + current */
            int32_t mv0 = (int32_t)samples[0] * 2;
            adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN_1_4, 12, &mv0);
            int v0_whole = mv0 / 1000, v0_frac = mv0 % 1000;
            if (v0_frac < 0) v0_frac = -v0_frac;
            int32_t i_pA0    = (mv0 * 1000) / 22;
            int     i_whole0 = i_pA0 / 1000, i_frac0 = i_pA0 % 1000;
            if (i_frac0 < 0) i_frac0 = -i_frac0;

            /* A2: voltage + current (samples[1] = second channel in buffer) */
            int32_t mv2 = (int32_t)samples[1];
            adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN_1_4, 12, &mv2);
            int v2_whole = mv2 / 1000, v2_frac = mv2 % 1000;
            if (v2_frac < 0) v2_frac = -v2_frac;
            int32_t i_pA2    = (mv2 * 1000) / 22;
            int     i_whole2 = i_pA2 / 1000, i_frac2 = i_pA2 % 1000;
            if (i_frac2 < 0) i_frac2 = -i_frac2;

            printk("sweep: %s%d.%03d V  (duty %lu%%)\n",
                   sweep_mv < 0 ? "-" : "",
                   sw_whole < 0 ? -sw_whole : sw_whole,
                   sw_frac,
                   (unsigned long)(sweep_duty_ns * 100 / PWM_PERIOD_NS));

            printk("A0: %s%d.%03d V  |  %s%d.%03d nA\n",
                   mv0  < 0 ? "-" : "", v0_whole < 0 ? -v0_whole : v0_whole, v0_frac,
                   i_pA0 < 0 ? "-" : "", i_whole0 < 0 ? -i_whole0 : i_whole0, i_frac0);

            printk("A2: %s%d.%03d V  |  %s%d.%03d nA\n",
                   mv2  > 0 ? "-" : "", v2_whole < 0 ? -v2_whole : v2_whole, v2_frac,
                   i_pA2 > 0 ? "-" : "", i_whole2 < 0 ? -i_whole2 : i_whole2, i_frac2);

            printk("---\n");
        }

        k_sleep(K_MSEC(20));
    }

    return 0;
}