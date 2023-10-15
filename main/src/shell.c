#include "shell.h"

#include <esp_console.h>
#include <esp_log.h>
#include <esp_system.h>
#include <argtable3/argtable3.h>

#include "neighbour.h"
#include "node_info.h"
#include "ota.h"
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

#define ADD_COMMAND(name_, help_, func_) do {		\
	const esp_console_cmd_t cmd = { 		\
		.command = (name_),			\
		.help = (help_),			\
		.func = &(func_),			\
		.hint = NULL,				\
		.argtable = NULL			\
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

	esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
	esp_err_t err = esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl);
	if (err) {
		return err;
	}

	return esp_console_start_repl(repl);
}

