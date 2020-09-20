#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)

#define FLAGS_OR_ZERO(node)	\
	COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags), (DT_GPIO_FLAGS(node, gpios)), (0))

struct printk_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	uint32_t led;
	uint32_t cnt;
};

K_FIFO_DEFINE(printk_fifo);

struct led {
	const char *gpio_dev_name;
	const char *gpio_pin_name;
	unsigned int gpio_pin;
	unsigned int gpio_flags;
};

void blink(const struct led *led, uint32_t sleep_ms, uint32_t id)
{
	struct device *gpio_dev;
	int cnt = 0;
	int ret;

	gpio_dev = device_get_binding(led->gpio_dev_name);
	if (gpio_dev == NULL) {
		printk("Error: didn't find %s device\n",
		       led->gpio_dev_name);
		return;
	}

	ret = gpio_pin_configure(gpio_dev, led->gpio_pin, led->gpio_flags);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d '%s'\n", ret, led->gpio_pin, led->gpio_pin_name);
		return;
	}

	while(1){
		gpio_pin_set(gpio_dev, led->gpio_pin, cnt % 2);

		size_t size = sizeof(struct printk_data_t);
		struct printk_data_t *tx_data = (struct printk_data_t*)k_malloc(size);
		__ASSERT_NO_MSG(tx_data != 0);
		tx_data->led = id;
		tx_data->cnt = cnt;

		k_fifo_put(&printk_fifo, tx_data);

		k_msleep(sleep_ms);
		cnt++;
	}
}

void blink0(void)
{
	const struct led led = {
		.gpio_dev_name 	= DT_GPIO_LABEL(LED0_NODE, gpios),
		.gpio_pin_name 	= DT_LABEL(LED0_NODE),
		.gpio_pin 		= DT_GPIO_PIN(LED0_NODE, gpios),
		.gpio_flags 	= GPIO_OUTPUT | FLAGS_OR_ZERO(LED0_NODE),
	};

	blink(&led, 100, 0);
}

void blink1(void)
{
	const struct led led = {
		.gpio_dev_name 	= DT_GPIO_LABEL(LED1_NODE, gpios),
		.gpio_pin_name 	= DT_LABEL(LED1_NODE),
		.gpio_pin 		= DT_GPIO_PIN(LED1_NODE, gpios),
		.gpio_flags 	= GPIO_OUTPUT | FLAGS_OR_ZERO(LED1_NODE),
	};

	blink(&led, 500, 1);
}

void blink2(void)
{
	const struct led led = {
		.gpio_dev_name 	= DT_GPIO_LABEL(LED2_NODE, gpios),
		.gpio_pin_name 	= DT_LABEL(LED2_NODE),
		.gpio_pin 		= DT_GPIO_PIN(LED2_NODE, gpios),
		.gpio_flags 	= GPIO_OUTPUT | FLAGS_OR_ZERO(LED2_NODE),
	};

	blink(&led, 1000, 1);
}

void blink3(void)
{
	const struct led led = {
		.gpio_dev_name 	= DT_GPIO_LABEL(LED3_NODE, gpios),
		.gpio_pin_name 	= DT_LABEL(LED3_NODE),
		.gpio_pin 		= DT_GPIO_PIN(LED3_NODE, gpios),
		.gpio_flags 	= GPIO_OUTPUT | FLAGS_OR_ZERO(LED3_NODE),
	};

	blink(&led, 2000, 1);
}

void uart_out(void)
{
	while (1) {
		struct printk_data_t *rx_data = k_fifo_get(&printk_fifo, K_FOREVER);
		printk("Toggled led%d; counter=%d\n", rx_data->led, rx_data->cnt);
		k_free(rx_data);
	}
}

K_THREAD_DEFINE(blink0_id, STACKSIZE, blink0, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(blink1_id, STACKSIZE, blink1, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(blink2_id, STACKSIZE, blink2, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(blink3_id, STACKSIZE, blink3, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(uart_out_id, STACKSIZE, uart_out, NULL, NULL, NULL, PRIORITY, 0, 0);
