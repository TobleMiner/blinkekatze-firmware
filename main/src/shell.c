#include "shell.h"

#include <errno.h>
#include <stdio.h>

#include <esp_console.h>
#include <esp_log.h>
#include <esp_system.h>
#include <argtable3/argtable3.h>

#include "color_override.h"
#include "default_color.h"
#include "neighbour.h"
#include "node_info.h"
#include "ota.h"
#include "power_control.h"
#include "rainbow_fade.h"
#include "state_of_charge.h"
#include "uid.h"
#include "usb.h"
#include "util.h"

static const char *TAG = "repl";

void main_loop_lock(void);
void main_loop_unlock(void);

static bonk_t *the_bonk;

static int serve_ota(int argc, char **argv) {
	main_loop_lock();
	esp_err_t err = ota_serve_update(true);
	if (err) {
		printf("Failed to start serving updates: %d\r\n", err);
	} else {
		printf("Serving OTA update to neighbours\r\n");
	}
	main_loop_unlock();
	return 0;
}

static int stop_serving_ota(int argc, char **argv) {
	main_loop_lock();
	ota_serve_update(false);
	main_loop_unlock();
	return 0;
}

static int list_neighbours(int argc, char **argv) {
	main_loop_lock();
	neighbour_print_list();
	main_loop_unlock();
	return 0;
}

static int parse_mac_address(const char *str, uint8_t *address) {
	int num_bytes_out = 0;
	while (*str && num_bytes_out < ESP_NOW_ETH_ALEN) {
		if (*str == ':') {
			str++;
			continue;
		}

		if (!str[1]) {
			return -EINVAL;
		}

		address[num_bytes_out++] = hex_to_byte(str);
		str += 2;
	}

	return num_bytes_out == ESP_NOW_ETH_ALEN ? 0 : -EINVAL;
}

static struct {
	struct arg_str *address;
	struct arg_end *end;
} node_info_args;

static int node_info(int argc, char **argv) {
	node_info_args.address->sval[0] = "";

	int errors = arg_parse(argc, argv, (void **)&node_info_args);
	if (errors) {
		arg_print_errors(stderr, node_info_args.end, argv[0]);
		return 1;
	}

	if (node_info_args.address->sval[0] && strlen(node_info_args.address->sval[0])) {
		uint8_t address[ESP_NOW_ETH_ALEN];
		int err = parse_mac_address(node_info_args.address->sval[0], address);
		if (err) {
			fprintf(stderr, "'%s' is not a valid node address\r\n", node_info_args.address->sval[0]);
			return 1;
		}

		main_loop_lock();
		const neighbour_t *neigh = neighbour_find_by_address(address);
		if (neigh) {
			node_info_print_remote(neigh);
		} else {
			fprintf(stderr, "No neighbour with address '%s' found\r\n", node_info_args.address->sval[0]);
		}
		main_loop_unlock();
	} else {
		main_loop_lock();
		node_info_print_local();
		main_loop_unlock();
	}
	return 0;
}

static int ota_status(int argc, char **argv) {
	main_loop_lock();
	ota_print_status();
	main_loop_unlock();
	return 0;
}

static int ota_ignore_version(int argc, char **argv) {
	ota_set_ignore_version(true);
	return 0;
}

static int reboot(int argc, char **argv) {
	esp_restart();
	return 0;
}

static struct {
	struct arg_str *address;
	struct arg_str *enable;
	struct arg_end *end;
} uid_args;

static int parse_on_off(const char *str, bool *on) {
	if (!strcasecmp(str, "on") ||
	    !strcasecmp(str, "true") ||
	    !strcasecmp(str, "enable") ||
	    !strcasecmp(str, "1")) {
		*on = true;
		return 0;
	}
	if (!strcasecmp(str, "off") ||
	    !strcasecmp(str, "false") ||
	    !strcasecmp(str, "disable") ||
	    !strcasecmp(str, "0")) {
		*on = false;
		return 0;
	}

	return -EINVAL;
}

static int uid(int argc, char **argv) {
	uid_args.address->sval[0] = "";
	uid_args.enable->sval[0] = "";

	int errors = arg_parse(argc, argv, (void **)&uid_args);
	if (errors) {
		arg_print_errors(stderr, uid_args.end, argv[0]);
		return 1;
	}

	uint8_t address[ESP_NOW_ETH_ALEN];
	int err = parse_mac_address(uid_args.address->sval[0], address);
	if (err) {
		fprintf(stderr, "'%s' is not a valid node address\r\n", uid_args.address->sval[0]);
		return 1;
	}

	bool enable;
	err = parse_on_off(uid_args.enable->sval[0], &enable);
	if (err) {
		fprintf(stderr, "'%s' is neither on nor off\r\n", uid_args.enable->sval[0]);
		return 1;
	}

	uid_enable(address, enable);
	uid_enable(address, enable);
	uid_enable(address, enable);

	return 0;
}

static struct {
	struct arg_str *enable;
	struct arg_end *end;
} rainbow_fade_args;

static int rainbow_fade(int argc, char **argv) {
	rainbow_fade_args.enable->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&rainbow_fade_args);
	if (errors) {
		arg_print_errors(stderr, rainbow_fade_args.end, argv[0]);
		return 1;
	}

	bool enable;
	int err = parse_on_off(rainbow_fade_args.enable->sval[0], &enable);
	if (err) {
		fprintf(stderr, "'%s' is neither on nor off\r\n", rainbow_fade_args.enable->sval[0]);
		return 1;
	}

	rainbow_fade_set_enable(enable);

	return 0;
}

static struct {
	struct arg_str *enable;
	struct arg_end *end;
} rainbow_fade_rssi_delay_args;

static int rainbow_fade_rssi_delay(int argc, char **argv) {
	rainbow_fade_rssi_delay_args.enable->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&rainbow_fade_rssi_delay_args);
	if (errors) {
		arg_print_errors(stderr, rainbow_fade_rssi_delay_args.end, argv[0]);
		return 1;
	}

	bool enable;
	int err = parse_on_off(rainbow_fade_rssi_delay_args.enable->sval[0], &enable);
	if (err) {
		fprintf(stderr, "'%s' is neither on nor off\r\n", rainbow_fade_rssi_delay_args.enable->sval[0]);
		return 1;
	}

	rainbow_fade_set_phase_shift_enable(enable);

	return 0;
}

typedef struct rssi_delay_model_args {
	struct arg_int *threshold;
	struct arg_int *limit;
	struct arg_int *delay;
	struct arg_int *delay_limit;
} rssi_delay_model_args_t;

static int handle_rssi_delay_model_args(const rssi_delay_model_args_t *args, neighbour_rssi_delay_model_t *model) {
	int threshold = *args->threshold->ival;
	if (threshold < -128 || threshold > 127) {
		fprintf(stderr, "RSSI threshold must be -128 - 127\r\n");
		return 1;
	}

	int limit = *args->limit->ival;
	if (limit < -128 || limit > 127) {
		fprintf(stderr, "RSSI limit must be -128 - 127\r\n");
		return 1;
	}

	model->us_delay_per_rssi_step = *args->delay->ival;
	model->delay_limit_us = *args->delay_limit->ival;
	model->delay_rssi_threshold = threshold;
	model->delay_rssi_limit = limit;
	return 0;
}

static struct {
	rssi_delay_model_args_t delay_model_args;
	struct arg_end *end;
} rainbow_fade_rssi_delay_model_args;

static int rainbow_fade_rssi_delay_model(int argc, char **argv) {
	int errors = arg_parse(argc, argv, (void **)&rainbow_fade_rssi_delay_model_args);
	if (errors) {
		arg_print_errors(stderr, rainbow_fade_rssi_delay_model_args.end, argv[0]);
		return 1;
	}

	neighbour_rssi_delay_model_t delay_model;
	int err = handle_rssi_delay_model_args(&rainbow_fade_rssi_delay_model_args.delay_model_args, &delay_model);
	if (err) {
		return err;
	}

	rainbow_fade_set_rssi_delay_model(&delay_model);

	return 0;
}

static struct {
	struct arg_int *cycle_time_ms;
	struct arg_end *end;
} rainbow_fade_cycle_time_args;

static int rainbow_fade_cycle_time(int argc, char **argv) {
	int errors = arg_parse(argc, argv, (void **)&rainbow_fade_cycle_time_args);
	if (errors) {
		arg_print_errors(stderr, rainbow_fade_cycle_time_args.end, argv[0]);
		return 1;
	}

	int cycle_time_ms = *rainbow_fade_cycle_time_args.cycle_time_ms->ival;
	if (cycle_time_ms <= 0) {
		fprintf(stderr, "Cycle time must be >= 0\r\n");
		return 1;
	}

	rainbow_fade_set_cycle_time(cycle_time_ms);

	return 0;
}

static struct {
	struct arg_str *enable;
	struct arg_end *end;
} color_override_args;

static int color_override(int argc, char **argv) {
	color_override_args.enable->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&color_override_args);
	if (errors) {
		arg_print_errors(stderr, color_override_args.end, argv[0]);
		return 1;
	}

	bool enable;
	int err = parse_on_off(color_override_args.enable->sval[0], &enable);
	if (err) {
		fprintf(stderr, "'%s' is neither on nor off\r\n", color_override_args.enable->sval[0]);
		return 1;
	}

	color_override_set_enable(enable);

	return 0;
}

typedef struct rgb16_args {
	struct arg_int *r;
	struct arg_int *g;
	struct arg_int *b;
} rgb16_args_t;

static int handle_rgb16_args(const rgb16_args_t *args, rgb16_t *color) {
	int r = *args->r->ival;
	if (r < 0 || r > 65535) {
		fprintf(stderr, "Red portion must be 0 - 65535\r\n");
		return 1;
	}

	int g = *args->g->ival;
	if (g < 0 || g > 65535) {
		fprintf(stderr, "Green portion must be 0 - 65535\r\n");
		return 1;
	}

	int b = *args->b->ival;
	if (b < 0 || b > 65535) {
		fprintf(stderr, "Blue portion must be 0 - 65535\r\n");
		return 1;
	}

	color->r = r;
	color->g = g;
	color->b = b;
	return 0;
}

typedef struct hsv_args {
	struct arg_int *h;
	struct arg_int *s;
	struct arg_int *v;
} hsv_args_t;

static int handle_hsv_args(const hsv_args_t *args, color_hsv_t *color) {
	int h = *args->h->ival;
	if (h < 0 || h > HSV_HUE_MAX) {
		fprintf(stderr, "Hue must be 0 - %u\r\n", HSV_HUE_MAX);
		return 1;
	}

	int s = *args->s->ival;
	if (s < 0 || s > HSV_SAT_MAX) {
		fprintf(stderr, "Saturation must be 0 - %u\r\n", HSV_SAT_MAX);
		return 1;
	}

	int v = *args->v->ival;
	if (v < 0 || v > HSV_VAL_MAX) {
		fprintf(stderr, "Brightness must be 0 - %u\r\n", HSV_VAL_MAX);
		return 1;
	}

	color->h = h;
	color->s = s;
	color->v = v;
	return 0;
}

static struct {
	rgb16_args_t rgb;
	struct arg_end *end;
} color_override_color_args;

static int color_override_color(int argc, char **argv) {
	int errors = arg_parse(argc, argv, (void **)&color_override_color_args);
	if (errors) {
		arg_print_errors(stderr, color_override_color_args.end, argv[0]);
		return 1;
	}

	rgb16_t color;
	int err = handle_rgb16_args(&color_override_color_args.rgb, &color);
	if (err) {
		return err;
	}
	color_override_set_color(&color);

	return 0;
}

static struct {
	struct arg_str *address;
	rgb16_args_t rgb;
	struct arg_int *duration;
	struct arg_end *end;
} color_override_remote_color_args;

static int color_override_remote_color(int argc, char **argv) {
	color_override_remote_color_args.address->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&color_override_remote_color_args);
	if (errors) {
		arg_print_errors(stderr, color_override_remote_color_args.end, argv[0]);
		return 1;
	}

	bool broadcast = true;
	wireless_address_t dst_addr;
	if (strlen(color_override_remote_color_args.address->sval[0])) {
		int err = parse_mac_address(color_override_remote_color_args.address->sval[0], dst_addr);
		if (err) {
			fprintf(stderr, "'%s' is not a valid node address\r\n", color_override_remote_color_args.address->sval[0]);
			return err;
		}
		broadcast = false;
	}

	rgb16_t color;
	int err = handle_rgb16_args(&color_override_remote_color_args.rgb, &color);
	if (err) {
		return err;
	}

	int duration_ms = *color_override_remote_color_args.duration->ival;
	if (duration_ms <= 0) {
		fprintf(stderr, "duration must be >= 0\r\n");
		return 1;
	}

	main_loop_lock();
	int64_t now_global = neighbour_get_global_clock();
	int64_t override_stop = now_global + (int64_t)duration_ms * 1000LL;

	for (int i = 0; i < 3; i++) {
		if (broadcast) {
			color_override_broadcast(&color, now_global, override_stop);
		} else {
			color_override_tx(&color, now_global, override_stop, dst_addr);
		}
	}
	main_loop_unlock();

	return 0;
}

static struct {
	struct arg_str *enable;
	struct arg_end *end;
} bonk_args;

static int bonk(int argc, char **argv) {
	bonk_args.enable->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&bonk_args);
	if (errors) {
		arg_print_errors(stderr, bonk_args.end, argv[0]);
		return 1;
	}

	bool enable;
	int err = parse_on_off(bonk_args.enable->sval[0], &enable);
	if (err) {
		fprintf(stderr, "'%s' is neither on nor off\r\n", bonk_args.enable->sval[0]);
		return 1;
	}

	main_loop_lock();
	bonk_set_enable(the_bonk, enable);
	main_loop_unlock();

	return 0;
}

static struct {
	struct arg_int *bonk_duration_ms;
	struct arg_end *end;
} bonk_duration_args;

static int bonk_duration(int argc, char **argv) {
	int errors = arg_parse(argc, argv, (void **)&bonk_duration_args);
	if (errors) {
		arg_print_errors(stderr, bonk_duration_args.end, argv[0]);
		return 1;
	}

	int bonk_duration_ms = *bonk_duration_args.bonk_duration_ms->ival;
	if (bonk_duration_ms < 0 || bonk_duration_ms > 65535) {
		fprintf(stderr, "Bonk duration must be 0ms - 65535ms\r\n");
		return 1;
	}

	main_loop_lock();
	bonk_set_duration(the_bonk, bonk_duration_ms);
	main_loop_unlock();

	return 0;
}

static struct {
	struct arg_str *enable;
	struct arg_end *end;
} bonk_rssi_delay_args;

static int bonk_rssi_delay(int argc, char **argv) {
	bonk_rssi_delay_args.enable->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&bonk_rssi_delay_args);
	if (errors) {
		arg_print_errors(stderr, bonk_rssi_delay_args.end, argv[0]);
		return 1;
	}

	bool enable;
	int err = parse_on_off(bonk_rssi_delay_args.enable->sval[0], &enable);
	if (err) {
		fprintf(stderr, "'%s' is neither on nor off\r\n", bonk_rssi_delay_args.enable->sval[0]);
		return 1;
	}

	main_loop_lock();
	bonk_set_delay_enable(the_bonk, enable);
	main_loop_unlock();

	return 0;
}

static struct {
	rssi_delay_model_args_t delay_model_args;
	struct arg_end *end;
} bonk_rssi_delay_model_args;

static int bonk_rssi_delay_model(int argc, char **argv) {
	int errors = arg_parse(argc, argv, (void **)&bonk_rssi_delay_model_args);
	if (errors) {
		arg_print_errors(stderr, bonk_rssi_delay_model_args.end, argv[0]);
		return 1;
	}

	neighbour_rssi_delay_model_t delay_model;
	int err = handle_rssi_delay_model_args(&bonk_rssi_delay_model_args.delay_model_args, &delay_model);
	if (err) {
		return err;
	}

	main_loop_lock();
	bonk_set_rssi_delay_model(the_bonk, &delay_model);
	main_loop_unlock();

	return 0;
}

static struct {
	struct arg_str *enable;
	struct arg_end *end;
} bonk_decay_args;

static int bonk_decay(int argc, char **argv) {
	bonk_decay_args.enable->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&bonk_decay_args);
	if (errors) {
		arg_print_errors(stderr, bonk_decay_args.end, argv[0]);
		return 1;
	}

	bool enable;
	int err = parse_on_off(bonk_decay_args.enable->sval[0], &enable);
	if (err) {
		fprintf(stderr, "'%s' is neither on nor off\r\n", bonk_decay_args.enable->sval[0]);
		return 1;
	}

	main_loop_lock();
	bonk_set_decay_enable(the_bonk, enable);
	main_loop_unlock();

	return 0;
}

static struct {
	struct arg_str *enable;
	struct arg_end *end;
} ignore_power_switch_args;

static int ignore_power_switch(int argc, char **argv) {
	ignore_power_switch_args.enable->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&ignore_power_switch_args);
	if (errors) {
		arg_print_errors(stderr, ignore_power_switch_args.end, argv[0]);
		return 1;
	}

	bool enable;
	int err = parse_on_off(ignore_power_switch_args.enable->sval[0], &enable);
	if (err) {
		fprintf(stderr, "'%s' is neither on nor off\r\n", ignore_power_switch_args.enable->sval[0]);
		return 1;
	}

	main_loop_lock();
	power_control_set_ignore_power_switch(enable);
	main_loop_unlock();

	return 0;
}

static struct {
	hsv_args_t hsv;
	struct arg_end *end;
} default_color_args;

static int default_color(int argc, char **argv) {
	int errors = arg_parse(argc, argv, (void **)&default_color_args);
	if (errors) {
		arg_print_errors(stderr, default_color_args.end, argv[0]);
		return 1;
	}

	color_hsv_t color;
	int err = handle_hsv_args(&default_color_args.hsv, &color);
	if (err) {
		return err;
	}
	main_loop_lock();
	default_color_set_color(&color);
	main_loop_unlock();

	return 0;
}

static struct {
	struct arg_str *enable;
	struct arg_end *end;
} soc_display_args;

static int soc_display(int argc, char **argv) {
	soc_display_args.enable->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&soc_display_args);
	if (errors) {
		arg_print_errors(stderr, soc_display_args.end, argv[0]);
		return 1;
	}

	bool enable;
	int err = parse_on_off(soc_display_args.enable->sval[0], &enable);
	if (err) {
		fprintf(stderr, "'%s' is neither on nor off\r\n", soc_display_args.enable->sval[0]);
		return 1;
	}

	main_loop_lock();
	state_of_charge_set_display_enable(enable);
	main_loop_unlock();

	return 0;
}

static struct {
	struct arg_str *enable;
	struct arg_end *end;
} wireless_encryption_args;

static int wireless_encryption(int argc, char **argv) {
	wireless_encryption_args.enable->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&wireless_encryption_args);
	if (errors) {
		arg_print_errors(stderr, wireless_encryption_args.end, argv[0]);
		return 1;
	}

	bool enable;
	int err = parse_on_off(wireless_encryption_args.enable->sval[0], &enable);
	if (err) {
		fprintf(stderr, "'%s' is neither on nor off\r\n", wireless_encryption_args.enable->sval[0]);
		return 1;
	}

	wireless_set_encryption_enable(enable);

	return 0;
}

static struct {
	struct arg_str *disable;
	struct arg_end *end;
} usb_disable_args;

static int usb_disable(int argc, char **argv) {
	usb_disable_args.disable->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&usb_disable_args);
	if (errors) {
		arg_print_errors(stderr, usb_disable_args.end, argv[0]);
		return 1;
	}

	bool disable;
	int err = parse_on_off(usb_disable_args.disable->sval[0], &disable);
	if (err) {
		fprintf(stderr, "'%s' is neither on nor off\r\n", usb_disable_args.disable->sval[0]);
		return 1;
	}

	main_loop_lock();
	usb_set_enable(!disable);
	main_loop_unlock();

	return 0;
}

static struct {
	struct arg_str *enable;
	struct arg_end *end;
} usb_enable_override_args;

static int usb_enable_override(int argc, char **argv) {
	usb_enable_override_args.enable->sval[0] = "";
	int errors = arg_parse(argc, argv, (void **)&usb_enable_override_args);
	if (errors) {
		arg_print_errors(stderr, usb_enable_override_args.end, argv[0]);
		return 1;
	}

	if (usb_enable_override_args.enable->sval[0] && strlen(usb_enable_override_args.enable->sval[0])) {
		bool enable;
		int err = parse_on_off(usb_enable_override_args.enable->sval[0], &enable);
		if (err) {
			fprintf(stderr, "'%s' is neither on nor off\r\n", usb_enable_override_args.enable->sval[0]);
			return 1;
		}

		main_loop_lock();
		usb_set_enable_override(enable);
		main_loop_unlock();
	} else {
		bool enable = usb_is_enable_overriden();
		printf("USB enable override is %s\n", enable ? "enabled" : "disabled");
	}

	return 0;
}

#define ADD_COMMAND(name_, help_, func_) \
	ADD_COMMAND_ARGS(name_, help_, func_, NULL)

#define ADD_COMMAND_ARGS(name_, help_, func_, arg_) do {\
	const esp_console_cmd_t cmd = { 		\
		.command = (name_),			\
		.help = (help_),			\
		.func = &(func_),			\
		.hint = NULL,				\
		.argtable = (arg_)			\
	};						\
	esp_err_t err =					\
		esp_console_cmd_register(&cmd);		\
	if (err) {					\
		ESP_LOGE(TAG, "Failed to register %s",	\
			 (name_));			\
		return err;				\
	}						\
} while (0)

static void delay_model_args_init(rssi_delay_model_args_t *args) {
	args->threshold = arg_int1(NULL, NULL, "min rssi", "RSSI to start delaying at");
	args->limit = arg_int1(NULL, NULL, "max rssi", "RSSI to limit delay calculation to");
	args->delay = arg_int1(NULL, NULL, "us", "Delay in us per RSSI step");
	args->delay_limit = arg_int1(NULL, NULL, "us", "Maximum delay in us (set to 0 to disable)");
}

static void rgb16_args_init(rgb16_args_t *args) {
	args->r = arg_int1(NULL, NULL, "r", "Red color portion, 0 - 65535");
	args->g = arg_int1(NULL, NULL, "g", "Green color portion, 0 - 65535");
	args->b = arg_int1(NULL, NULL, "b", "Blue color portion, 0 - 65535");
}

static void hsv_args_init(hsv_args_t *args) {
	args->h = arg_int1(NULL, NULL, "h", "Hue, 0 - 49151");
	args->s = arg_int1(NULL, NULL, "s", "Saturation, 0 - 65535");
	args->v = arg_int1(NULL, NULL, "v", "Brightness, 0 - 65535");
}

esp_err_t shell_init(bonk_t *bonk_) {
	the_bonk = bonk_;

	esp_console_repl_t *repl = NULL;
	esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
	repl_config.prompt = "blinkekatze >";
	repl_config.max_cmdline_length = 255;
	esp_console_register_help_command();

	ADD_COMMAND("list_neighbours",
		    "List wireless neighbours",
		    list_neighbours);

	ADD_COMMAND("serve_ota",
		    "Serve own firmware via OTA update to neighbours",
		    serve_ota);

	ADD_COMMAND("stop_serving_ota",
		    "Stop serving own firmware via OTA",
		    stop_serving_ota);

	node_info_args.address = arg_str0(NULL, NULL, "address", "Address of target node. Local node if omitted");
	node_info_args.end = arg_end(1);

	ADD_COMMAND_ARGS("node_info",
    			 "Show information about local node",
    			 node_info,
			 &node_info_args);

	ADD_COMMAND("ota_status",
		    "Show OTA update status",
		    ota_status);

	ADD_COMMAND("ota_ignore_version",
		    "Force OTA update even if version already installed",
		    ota_ignore_version);

	ADD_COMMAND("reboot",
		    "Reboot local node",
		    reboot);

	uid_args.address = arg_str1(NULL, NULL, "address", "Address of target node");
	uid_args.enable = arg_str1(NULL, NULL, "on|off", "Switch uid light on or off");
	uid_args.end = arg_end(2);

	ADD_COMMAND_ARGS("uid",
			 "Identify device",
			 uid,
			 &uid_args);

	rainbow_fade_args.enable = arg_str1(NULL, NULL, "on|off", "Disable/enable rainbow fade");
	rainbow_fade_args.end = arg_end(1);

	ADD_COMMAND_ARGS("rainbow_fade",
			 "Enable or disable rainbow fade",
			 rainbow_fade,
			 &rainbow_fade_args);

	rainbow_fade_rssi_delay_args.enable = arg_str1(NULL, NULL, "on|off", "Disable/enable rainbow fade phase shift based on RSSI");
	rainbow_fade_rssi_delay_args.end = arg_end(1);

	ADD_COMMAND_ARGS("rainbow_fade_rssi_delay",
			 "Enable or disable rainbow fade phase shift based on RSSI",
			 rainbow_fade_rssi_delay,
			 &rainbow_fade_rssi_delay_args);

	delay_model_args_init(&rainbow_fade_rssi_delay_model_args.delay_model_args);
	rainbow_fade_rssi_delay_model_args.end = arg_end(4);

	ADD_COMMAND_ARGS("rainbow_fade_rssi_delay_model",
			 "Configure model used to calculate rainbow fade phase shift",
			 rainbow_fade_rssi_delay_model,
			 &rainbow_fade_rssi_delay_model_args);

	rainbow_fade_cycle_time_args.cycle_time_ms = arg_int1(NULL, NULL, "cycle time ms", "Rainbow fade cycle time in milliseconds");
	rainbow_fade_cycle_time_args.end = arg_end(1);

	ADD_COMMAND_ARGS("rainbow_fade_cycle_time",
			 "Set rainbow fade cycle time",
			 rainbow_fade_cycle_time,
			 &rainbow_fade_cycle_time_args);

	color_override_args.enable = arg_str1(NULL, NULL, "on|off", "Disable/enable local color override");
	color_override_args.end = arg_end(1);

	ADD_COMMAND_ARGS("color_override",
			 "Enable or disable local color override",
			 color_override,
			 &color_override_args);

	rgb16_args_init(&color_override_color_args.rgb);
	color_override_color_args.end = arg_end(3);

	ADD_COMMAND_ARGS("color_override_color",
			 "Set local color override color",
			 color_override_color,
			 &color_override_color_args);

	rgb16_args_init(&color_override_remote_color_args.rgb);
	color_override_remote_color_args.address = arg_str0("a", "address", "address", "Target a specific node");
	color_override_remote_color_args.duration = arg_int1(NULL, NULL, "duration ms", "Duration of the override in ms");
	color_override_remote_color_args.end = arg_end(4);

	ADD_COMMAND_ARGS("color_override_remote_color",
			 "Set temporary global remote color override",
			 color_override_remote_color,
			 &color_override_remote_color_args);

	bonk_args.enable = arg_str1(NULL, NULL, "on|off", "Disable/enable bonk");
	bonk_args.end = arg_end(1);

	ADD_COMMAND_ARGS("bonk",
			 "Enable or disable bonk",
			 bonk,
			 &bonk_args);

	bonk_duration_args.bonk_duration_ms = arg_int1(NULL, NULL, "bonk duration ms", "Bonk duration in milliseconds");
	bonk_duration_args.end = arg_end(1);

	ADD_COMMAND_ARGS("bonk_duration",
			 "Set rainbow fade cycle time",
			 bonk_duration,
			 &bonk_duration_args);

	bonk_rssi_delay_args.enable = arg_str1(NULL, NULL, "on|off", "Disable/enable bonk delay based on RSSI");
	bonk_rssi_delay_args.end = arg_end(1);

	ADD_COMMAND_ARGS("bonk_rssi_delay",
			 "Enable or disable bonk delay based on RSSI",
			 bonk_rssi_delay,
			 &bonk_rssi_delay_args);

	delay_model_args_init(&bonk_rssi_delay_model_args.delay_model_args);
	bonk_rssi_delay_model_args.end = arg_end(4);

	ADD_COMMAND_ARGS("bonk_rssi_delay_model",
			 "Configure model used to calculate bonk delay",
			 bonk_rssi_delay_model,
			 &bonk_rssi_delay_model_args);

	bonk_decay_args.enable = arg_str1(NULL, NULL, "on|off", "Disable/enable bonk brightness decay");
	bonk_decay_args.end = arg_end(1);

	ADD_COMMAND_ARGS("bonk_decay",
			 "Enable or disable bonk brightness decay",
			 bonk_decay,
			 &bonk_decay_args);

	ignore_power_switch_args.enable = arg_str1(NULL, NULL, "on|off", "Enable/disable ignoring the physical power switch");
	ignore_power_switch_args.end = arg_end(1);

	ADD_COMMAND_ARGS("ignore_power_switch",
			 "Enable or disable physical power switch",
			 ignore_power_switch,
			 &ignore_power_switch_args);

	hsv_args_init(&default_color_args.hsv);
	default_color_args.end = arg_end(3);

	ADD_COMMAND_ARGS("default_color",
			 "Set default color",
			 default_color,
			 &default_color_args);

	soc_display_args.enable = arg_str1(NULL, NULL, "on|off", "Disable/enable SOC display");
	soc_display_args.end = arg_end(1);

	ADD_COMMAND_ARGS("soc_display",
			 "Enable or disable SOC display",
			 soc_display,
			 &soc_display_args);

	wireless_encryption_args.enable = arg_str1(NULL, NULL, "on|off", "Disable/enable wireless encryption");
	wireless_encryption_args.end = arg_end(1);

	ADD_COMMAND_ARGS("wireless_encryption",
			 "Enable or disable wireless encryption",
			 wireless_encryption,
			 &wireless_encryption_args);

	usb_enable_override_args.enable = arg_str0(NULL, NULL, "on|off", "Disable/enable USB enable override");
	usb_enable_override_args.end = arg_end(1);

	ADD_COMMAND_ARGS("usb_enable_override",
			 "Enable or disable local USB port enable override",
			 usb_enable_override,
			 &usb_enable_override_args);

	usb_disable_args.disable = arg_str1(NULL, NULL, "on|off", "Disable/enable USB enable override");
	usb_disable_args.end = arg_end(1);

	ADD_COMMAND_ARGS("usb_disable",
			 "Enable or disable local USB port enable override",
			 usb_disable,
			 &usb_disable_args);

	esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
	esp_err_t err = esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl);
	if (err) {
		return err;
	}

	return esp_console_start_repl(repl);
}
