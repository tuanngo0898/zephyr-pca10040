#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/pwm.h>

#define FLAGS_OR_ZERO(node)						\
	COND_CODE_1(DT_PHA_HAS_CELL(node, pwms, flags), (DT_PWMS_FLAGS(node)), (0))

#define PWM_NODE		DT_ALIAS(pwm_led0)
#define PWM_LABEL		DT_PWMS_LABEL(PWM_NODE)
#define PWM_CHANNEL		DT_PWMS_CHANNEL(PWM_NODE)
#define PWM_FLAGS		FLAGS_OR_ZERO(PWM_NODE)

#define PERIOD_MS		1000

void main(void)
{
	struct device *dev_pwm;
	uint32_t pulse = 0;
	uint8_t dir = 1;
	int ret;

	printk("PWM-based blinky\n");

	dev_pwm = device_get_binding(PWM_LABEL);
	if (!dev_pwm) {
		printk("Error: didn't find %s device\n", PWM_LABEL);
		return;
	}

	while(1){
		ret = pwm_pin_set_usec(dev_pwm, PWM_CHANNEL, PERIOD_MS, pulse, PWM_FLAGS);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return;
		}

		pulse = dir ? (pulse + 1) : (pulse - 1);
		if(pulse==0 || pulse==PERIOD_MS) dir = !dir;

		k_sleep(K_MSEC(1U));
	}
}
