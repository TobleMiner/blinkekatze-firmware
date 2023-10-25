#include "shell.h"

#include <errno.h>
#include <stdio.h>

#include <esp_console.h>
#include <esp_log.h>
#include <esp_system.h>
#include <argtable3/argtable3.h>

#include "color_override.h"
#include "neighbour.h"
#include "node_info.h"
#include "ota.h"
#include "rainbow_fade.h"
#include "uid.h"
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

static struct {
	struct arg_int *r;
	struct arg_int *g;
	struct arg_int *b;
	struct arg_end *end;
} color_override_color_args;

static int color_override_color(int argc, char **argv) {
	int errors = arg_parse(argc, argv, (void **)&color_override_color_args);
	if (errors) {
		arg_print_errors(stderr, color_override_color_args.end, argv[0]);
		return 1;
	}

	int r = *color_override_color_args.r->ival;
	if (r < 0 || r > 65535) {
		fprintf(stderr, "Red portion must be 0 - 65535\r\n");
		return 1;
	}

	int g = *color_override_color_args.g->ival;
	if (g < 0 || g > 65535) {
		fprintf(stderr, "Green portion must be 0 - 65535\r\n");
		return 1;
	}

	int b = *color_override_color_args.b->ival;
	if (b < 0 || b > 65535) {
		fprintf(stderr, "Blue portion must be 0 - 65535\r\n");
		return 1;
	}

	rgb16_t color = { r, g, b};
	color_override_set_color(&color);

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

	bonk_set_enable(the_bonk, enable);

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

	bonk_set_duration(the_bonk, bonk_duration_ms);

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

	bonk_set_decay_enable(the_bonk, enable);

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

	color_override_color_args.r = arg_int1(NULL, NULL, "r", "Red color portion, 0 - 65535");
	color_override_color_args.g = arg_int1(NULL, NULL, "g", "Green color portion, 0 - 65535");
	color_override_color_args.b = arg_int1(NULL, NULL, "b", "Blue color portion, 0 - 65535");
	color_override_color_args.end = arg_end(3);

	ADD_COMMAND_ARGS("color_override_color",
			 "Set color override color",
			 color_override_color,
			 &color_override_color_args);

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

	bonk_decay_args.enable = arg_str1(NULL, NULL, "on|off", "Disable/enable bonk brightness decay");
	bonk_decay_args.end = arg_end(1);

	ADD_COMMAND_ARGS("bonk_decay",
			 "Enable or disable bonk brightness decay",
			 bonk_decay,
			 &bonk_decay_args);

	esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
	esp_err_t err = esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl);
	if (err) {
		return err;
	}

	return esp_console_start_repl(repl);
}
