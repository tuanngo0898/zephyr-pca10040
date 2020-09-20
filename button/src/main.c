#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>

#define SLEEP_TIME_MS	1

#define FLAGS_OR_ZERO(node)	\
	COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags), (DT_GPIO_FLAGS(node, gpios)), (0))

#define BTN_NODE		DT_ALIAS(sw0)
#define BTN_LABEL		DT_GPIO_LABEL(BTN_NODE, gpios)
#define BTN_PIN			DT_GPIO_PIN(BTN_NODE, gpios)
#define BTN_FLAGS		(GPIO_INPUT | FLAGS_OR_ZERO(BTN_NODE))

#define LED_NODE		DT_ALIAS(led0)
#define LED_LABEL		DT_GPIO_LABEL(LED_NODE, gpios)
#define LED_PIN			DT_GPIO_PIN(LED_NODE, gpios)
#define LED_FLAGS		(GPIO_OUTPUT | FLAGS_OR_ZERO(LED_NODE))

static struct gpio_callback button_cb_data;

void button_pressed(struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

void main(void)
{
	struct device *dev_btn;
	struct device *dev_led;
	int ret;

	/* Button */
	dev_btn = device_get_binding(BTN_LABEL);
	if (dev_btn == NULL) {
		printk("Error: didn't find %s device\n", BTN_LABEL);
		return;
	}

	ret = gpio_pin_configure(dev_btn, BTN_PIN, BTN_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, BTN_LABEL, BTN_PIN);
		return;
	}

	ret = gpio_pin_interrupt_configure(dev_btn, BTN_PIN, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, BTN_LABEL, BTN_PIN);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(BTN_PIN));
	gpio_add_callback(dev_btn, &button_cb_data);
	printk("Set up button at %s pin %d\n", BTN_LABEL, BTN_PIN);

	/* LED */
	dev_led = device_get_binding(LED_LABEL);
	if (dev_led == NULL) {
		printk("Didn't find LED device %s\n", LED_LABEL);
		return;
	}
	ret = gpio_pin_configure(dev_led, LED_PIN, LED_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure LED device %s pin %d\n", ret, LED_LABEL, LED_PIN);
		return;
	}
	printk("Set up LED at %s pin %d\n", LED_LABEL, LED_PIN);

	printk("Press the button\n");
	while (1) {
		gpio_pin_set(dev_led, LED_PIN, gpio_pin_get(dev_btn, BTN_PIN));
		k_msleep(SLEEP_TIME_MS);
	}
};
