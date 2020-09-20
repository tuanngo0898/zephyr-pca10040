#include <sys/printk.h>
#include <settings/settings.h>
#include <sys/byteorder.h>
#include <device.h>
#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/mesh.h>
#include <stdio.h>

/* Model Operation Codes */
#define BT_MESH_MODEL_OP_GEN_ONOFF_GET			BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET			BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS		BT_MESH_MODEL_OP_2(0x82, 0x04)

static void gen_onoff_get(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf);

static void gen_onoff_set(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf);

static void gen_onoff_set_unack(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf);

static void gen_onoff_status(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf);

static struct bt_mesh_cfg_srv cfg_srv = {
	.relay 				= BT_MESH_RELAY_DISABLED,
	.beacon 			= BT_MESH_BEACON_ENABLED,
	.frnd 				= BT_MESH_FRIEND_DISABLED,
	.gatt_proxy 		= BT_MESH_GATT_PROXY_ENABLED,
	.default_ttl 		= 7,
	.net_transmit 		= BT_MESH_TRANSMIT(2, 20),
	.relay_retransmit 	= BT_MESH_TRANSMIT(2, 20),
};

// Client Configuration Declaration
static struct bt_mesh_cfg_cli cfg_cli = {
};

BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv_s_0, NULL, 2 + 2);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli_s_0, NULL, 2 + 2);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv_s_1, NULL, 2 + 2);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli_s_1, NULL, 2 + 2);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv_s_2, NULL, 2 + 2);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli_s_2, NULL, 2 + 2);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv_s_3, NULL, 2 + 2);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli_s_3, NULL, 2 + 2);

// OnOff Model Server Op Dispatch Table
static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ BT_MESH_MODEL_OP_GEN_ONOFF_GET, 0, gen_onoff_get },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET, 2, gen_onoff_set },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK, 2, gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};

// OnOff Model Client Op Dispatch Table
static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
	{ BT_MESH_MODEL_OP_GEN_ONOFF_STATUS, 1, gen_onoff_status },
	BT_MESH_MODEL_OP_END,
};

struct onoff_state {
	uint8_t current;
	uint8_t previous;
	uint8_t led_gpio_pin;
	struct device *led_device;
};

static struct onoff_state onoff_state[] = {
	{ .led_gpio_pin = DT_GPIO_PIN(DT_ALIAS(led0), gpios) },
	{ .led_gpio_pin = DT_GPIO_PIN(DT_ALIAS(led1), gpios) },
	{ .led_gpio_pin = DT_GPIO_PIN(DT_ALIAS(led2), gpios) },
	{ .led_gpio_pin = DT_GPIO_PIN(DT_ALIAS(led3), gpios) },
};

// Element Model Declarations
static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
};

static struct bt_mesh_model led0_models[] = {
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
		      &gen_onoff_pub_srv_s_0, &onoff_state[0]),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
		      &gen_onoff_pub_cli_s_0, &onoff_state[0]),
};

static struct bt_mesh_model led1_models[] = {
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
		      &gen_onoff_pub_srv_s_1, &onoff_state[1]),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
		      &gen_onoff_pub_cli_s_1, &onoff_state[1]),
};

static struct bt_mesh_model led2_models[] = {
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
		      &gen_onoff_pub_srv_s_2, &onoff_state[2]),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
		      &gen_onoff_pub_cli_s_2, &onoff_state[2]),
};

static struct bt_mesh_model led3_models[] = {
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
		      &gen_onoff_pub_srv_s_3, &onoff_state[3]),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
		      &gen_onoff_pub_cli_s_3, &onoff_state[3]),
};

struct bt_mesh_model *mod_cli_sw[] = {
		&led0_models[1],
		&led1_models[1],
		&led2_models[1],
		&led3_models[1]
};

struct bt_mesh_model *mod_srv_sw[] = {
		&led0_models[0],
		&led1_models[0],
		&led2_models[0],
		&led3_models[0]
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
	BT_MESH_ELEM(0, led0_models, BT_MESH_MODEL_NONE),
	BT_MESH_ELEM(0, led1_models, BT_MESH_MODEL_NONE),
	BT_MESH_ELEM(0, led2_models, BT_MESH_MODEL_NONE),
	BT_MESH_ELEM(0, led3_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

struct sw {
	uint8_t sw_num;
	uint8_t onoff_state;
	struct k_work button_work;
	struct k_timer button_timer;
};
static struct sw sw;

static uint8_t button_press_cnt;
struct device *sw_device;
static struct gpio_callback button_cb;

static uint8_t trans_id;
static uint32_t time, last_time;
static uint16_t primary_addr;
static uint16_t primary_net_idx;

// Generic OnOff Model Server Message Handlers
static void gen_onoff_get(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	NET_BUF_SIMPLE_DEFINE(msg, 2 + 1 + 4);
	struct onoff_state *onoff_state = model->user_data;

	printk("addr 0x%04x onoff 0x%02x\n", bt_mesh_model_elem(model)->addr, onoff_state->current);
	
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
	net_buf_simple_add_u8(&msg, onoff_state->current);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("Unable to send On Off Status response\n");
	}
}

static void gen_onoff_set_unack(struct bt_mesh_model *model,
				struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf)
{
	struct net_buf_simple *msg = model->pub->msg;
	struct onoff_state *onoff_state = model->user_data;
	int err;

	onoff_state->current = net_buf_simple_pull_u8(buf);
	printk("addr 0x%02x state 0x%02x\n", bt_mesh_model_elem(model)->addr, onoff_state->current);

	gpio_pin_set(onoff_state->led_device, onoff_state->led_gpio_pin, onoff_state->current);

	if (onoff_state->previous != onoff_state->current && model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		printk("publish last 0x%02x cur 0x%02x\n", onoff_state->previous, onoff_state->current);
		onoff_state->previous = onoff_state->current;
		bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
		net_buf_simple_add_u8(msg, onoff_state->current);
		err = bt_mesh_model_publish(model);
		if (err) {
			printk("bt_mesh_model_publish err %d\n", err);
		}
	}
}

static void gen_onoff_set(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	printk("gen_onoff_set\n");

	gen_onoff_set_unack(model, ctx, buf);
	gen_onoff_get(model, ctx, buf);
}

static void gen_onoff_status(struct bt_mesh_model *model,
			     struct bt_mesh_msg_ctx *ctx,
			     struct net_buf_simple *buf)
{
	uint8_t	state;

	state = net_buf_simple_pull_u8(buf);

	printk("Node 0x%04x OnOff status from 0x%04x with state 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, ctx->addr, state);
}

static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
	printk("OOB Number %06u\n", number);
	return 0;
}

static int output_string(const char *str)
{
	printk("OOB String %s\n", str);
	return 0;
}

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	printk("provisioning complete for net_idx 0x%04x addr 0x%04x\n",
	       net_idx, addr);
	primary_addr = addr;
	primary_net_idx = net_idx;
}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static uint8_t dev_uuid[16] = { 0xfd, 0x43 };

#define BUTTON_DEBOUNCE_DELAY_MS 250

static uint8_t pin_to_sw(uint32_t pin_pos)
{
	switch (pin_pos) {
		case BIT(DT_GPIO_PIN(DT_ALIAS(sw0), gpios)): return 0;
		case BIT(DT_GPIO_PIN(DT_ALIAS(sw1), gpios)): return 1;
		case BIT(DT_GPIO_PIN(DT_ALIAS(sw2), gpios)): return 2;
		case BIT(DT_GPIO_PIN(DT_ALIAS(sw3), gpios)): return 3;
	}

	printk("No match for GPIO pin 0x%08x\n", pin_pos);
	return 0;
}

static void button_pressed(struct device *dev, struct gpio_callback *cb,
			   uint32_t pin_pos)
{
	/*
	 * One button press within a 1 second interval sends an on message
	 * More than one button press sends an off message
	 */

	time = k_uptime_get_32();

	/* debounce the switch */
	if (time < last_time + BUTTON_DEBOUNCE_DELAY_MS) {
		last_time = time;
		return;
	}

	if (button_press_cnt == 0U) {
		k_timer_start(&sw.button_timer, K_SECONDS(1), K_NO_WAIT);
	}

	printk("button_press_cnt 0x%02x\n", button_press_cnt);
	button_press_cnt++;

	sw.sw_num = pin_to_sw(pin_pos);
	last_time = time;
}

// Button Count Timer Worker
static void button_cnt_timer(struct k_timer *work)
{
	struct sw *button_sw = CONTAINER_OF(work, struct sw, button_timer);

	button_sw->onoff_state = button_press_cnt == 1U ? 1 : 0;
	printk("button_press_cnt 0x%02x onoff_state 0x%02x\n",
	       button_press_cnt, button_sw->onoff_state);
	button_press_cnt = 0U;
	k_work_submit(&sw.button_work);
}

// Button Pressed Worker Task
static void button_pressed_worker(struct k_work *work)
{
	struct bt_mesh_model *mod_cli, *mod_srv;
	struct bt_mesh_model_pub *pub_cli, *pub_srv;
	struct sw *sw = CONTAINER_OF(work, struct sw, button_work);
	int err;
	uint8_t sw_idx = sw->sw_num;

	mod_cli = mod_cli_sw[sw_idx];
	pub_cli = mod_cli->pub;

	mod_srv = mod_srv_sw[sw_idx];
	pub_srv = mod_srv->pub;

	if (primary_addr == BT_MESH_ADDR_UNASSIGNED) {
		NET_BUF_SIMPLE_DEFINE(msg, 1);
		struct bt_mesh_msg_ctx ctx = {
			.addr = sw_idx + primary_addr,
		};

		net_buf_simple_add_u8(&msg, sw->onoff_state);
		gen_onoff_set_unack(mod_srv, &ctx, &msg);
		return;
	}

	if (pub_cli->addr == BT_MESH_ADDR_UNASSIGNED) {
		return;
	}

	printk("publish to 0x%04x onoff 0x%04x sw_idx 0x%04x\n",
	       pub_cli->addr, sw->onoff_state, sw_idx);
	bt_mesh_model_msg_init(pub_cli->msg, BT_MESH_MODEL_OP_GEN_ONOFF_SET);

	net_buf_simple_add_u8(pub_cli->msg, sw->onoff_state);
	net_buf_simple_add_u8(pub_cli->msg, trans_id++);

	err = bt_mesh_model_publish(mod_cli);
	if (err) {
		printk("bt_mesh_model_publish err %d\n", err);
	}
}

/* Disable OOB security for SILabs Android app */
static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 6,
	.output_actions = (BT_MESH_DISPLAY_NUMBER | BT_MESH_DISPLAY_STRING),
	.output_number = output_number,
	.output_string = output_string,
	.complete = prov_complete,
	.reset = prov_reset,
};

// Bluetooth Ready Callback
static void bt_ready(int err)
{
	struct bt_le_oob oob;

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* Use identity address as device UUID */
	if (bt_le_oob_get_local(BT_ID_DEFAULT, &oob)) {
		printk("Identity Address unavailable\n");
	} else {
		memcpy(dev_uuid, oob.addr.a.val, 6);
	}

	bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);

	printk("Mesh initialized\n");
}

void init_led(uint8_t dev, const char *port, uint32_t pin_num, gpio_flags_t flags)
{
	onoff_state[dev].led_device = device_get_binding(port);
	gpio_pin_configure(onoff_state[dev].led_device, pin_num,
			   flags | GPIO_OUTPUT_INACTIVE);
}

void main(void)
{
	int err;

	printk("Initializing...\n");

	/* Initialize the button debouncer */
	last_time = k_uptime_get_32();

	/* Initialize button worker task*/
	k_work_init(&sw.button_work, button_pressed_worker);

	/* Initialize button count timer */
	k_timer_init(&sw.button_timer, button_cnt_timer, NULL);

	/* Initialize button */
	sw_device = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(sw0), gpios));
	gpio_pin_configure(sw_device, DT_GPIO_PIN(DT_ALIAS(sw0), gpios),
			   GPIO_INPUT | DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios));
	gpio_pin_configure(sw_device, DT_GPIO_PIN(DT_ALIAS(sw1), gpios),
			   GPIO_INPUT | DT_GPIO_FLAGS(DT_ALIAS(sw1), gpios));
	gpio_pin_configure(sw_device, DT_GPIO_PIN(DT_ALIAS(sw2), gpios),
			   GPIO_INPUT | DT_GPIO_FLAGS(DT_ALIAS(sw2), gpios));
	gpio_pin_configure(sw_device, DT_GPIO_PIN(DT_ALIAS(sw3), gpios),
			   GPIO_INPUT | DT_GPIO_FLAGS(DT_ALIAS(sw3), gpios));

	gpio_pin_interrupt_configure(sw_device,
				     DT_GPIO_PIN(DT_ALIAS(sw0), gpios),
				     GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure(sw_device,
				     DT_GPIO_PIN(DT_ALIAS(sw1), gpios),
				     GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure(sw_device,
				     DT_GPIO_PIN(DT_ALIAS(sw2), gpios),
				     GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure(sw_device,
				     DT_GPIO_PIN(DT_ALIAS(sw3), gpios),
				     GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&button_cb, button_pressed,
			   BIT(DT_GPIO_PIN(DT_ALIAS(sw0), gpios)) |
			   BIT(DT_GPIO_PIN(DT_ALIAS(sw1), gpios)) |
			   BIT(DT_GPIO_PIN(DT_ALIAS(sw2), gpios)) |
			   BIT(DT_GPIO_PIN(DT_ALIAS(sw3), gpios)));

	gpio_add_callback(sw_device, &button_cb);

	/* Initialize LED's */
	onoff_state[0].led_device = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
	onoff_state[1].led_device = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led1), gpios));
	onoff_state[2].led_device = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led2), gpios));
	onoff_state[3].led_device = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led3), gpios));

	gpio_pin_configure(onoff_state[0].led_device, DT_GPIO_PIN(DT_ALIAS(led0), gpios),
			   DT_GPIO_FLAGS(DT_ALIAS(led0), gpios) | GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(onoff_state[1].led_device, DT_GPIO_PIN(DT_ALIAS(led1), gpios),
			   DT_GPIO_FLAGS(DT_ALIAS(led1), gpios) | GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(onoff_state[2].led_device, DT_GPIO_PIN(DT_ALIAS(led2), gpios),
			   DT_GPIO_FLAGS(DT_ALIAS(led2), gpios) | GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(onoff_state[3].led_device, DT_GPIO_PIN(DT_ALIAS(led3), gpios),
			   DT_GPIO_FLAGS(DT_ALIAS(led3), gpios) | GPIO_OUTPUT_INACTIVE);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
}
