#include "shell.h"

#include <errno.h>
#include <stdio.h>

#include <esp_console.h>
#include <esp_log.h>
#include <esp_system.h>
#include <argtable3/argtable3.h>

#include "neighbour.h"
#include "node_info.h"
#include "ota.h"
#include "rainbow_fade.h"
#include "uid.h"
#include "util.h"

static const char *TAG = "repl";

void main_loop_lock(void);
void main_loop_unlock(void);

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

static int node_info(int argc, char **argv) {
	node_info_print_local();
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
	}

	rainbow_fade_set_cycle_time(cycle_time_ms);

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

esp_err_t shell_init(void) {
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

	ADD_COMMAND("node_info",
		    "Show information about local node",
		    node_info);

	ADD_COMMAND("ota_status",
		    "Show OTA update status",
		    ota_status);

	ADD_COMMAND("ota_ignore_version",
		    "Force OTA update even if version already installed",
		    ota_ignore_version);

	ADD_COMMAND("reboot",
		    "Reboot local node",
		    reboot);

	uid_args.address = arg_str1(NULL, NULL, "<address>", "Address of target node");
	uid_args.enable = arg_str1(NULL, NULL, "<on|off>", "Switch uid light on or off");
	uid_args.end = arg_end(2);

	ADD_COMMAND_ARGS("uid",
			 "Identify device",
			 uid,
			 &uid_args);

	rainbow_fade_args.enable = arg_str1(NULL, NULL, "<on|off>", "Disable/enable rainbow fade");
	rainbow_fade_args.end = arg_end(1);

	ADD_COMMAND_ARGS("rainbow_fade",
			 "Enable or disable rainbow fade",
			 rainbow_fade,
			 &rainbow_fade_args);

	rainbow_fade_cycle_time_args.cycle_time_ms = arg_int1(NULL, NULL, "<cycle time ms>", "Rainbow fade cycle time in milliseconds");
	rainbow_fade_cycle_time_args.end = arg_end(1);

	ADD_COMMAND_ARGS("rainbow_fade_cycle_time",
			 "Set rainbow fade cycle time",
			 rainbow_fade_cycle_time,
			 &rainbow_fade_cycle_time_args);

	esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
	esp_err_t err = esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl);
	if (err) {
		return err;
	}

	return esp_console_start_repl(repl);
}
