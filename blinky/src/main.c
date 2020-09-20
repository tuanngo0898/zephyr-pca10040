#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE 		DT_ALIAS(led0)

#define LED_LABEL		DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED_PIN	    	DT_GPIO_PIN(LED0_NODE, gpios)
#define LED_FLAGS		DT_GPIO_FLAGS(LED0_NODE, gpios)

void main(void)
{
	struct device *dev_led;
	bool led_is_on = true;

	dev_led = device_get_binding(LED_LABEL);
	if (dev_led == NULL) return;

	int ret = gpio_pin_configure(dev_led, LED_PIN, GPIO_OUTPUT_ACTIVE | LED_FLAGS);
	if (ret < 0) return;

	while (1) {
		gpio_pin_set(dev_led, LED_PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME_MS);
	}
}