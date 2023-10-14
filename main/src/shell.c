#include "shell.h"

#include <esp_console.h>
#include <argtable3/argtable3.h>

#include "neighbour.h"
#include "node_info.h"
#include "ota.h"

void main_loop_lock(void);
void main_loop_unlock(void);

static int serve_ota(int argc, char **argv) {
	main_loop_lock();
	ota_serve_update(true);
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

esp_err_t shell_init(void) {
	esp_console_repl_t *repl = NULL;
	esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
	repl_config.prompt = "blinkekatze >";
	repl_config.max_cmdline_length = 255;
	esp_console_register_help_command();

	const esp_console_cmd_t list_neighbours_cmd = {
		.command = "list_neighbours",
		.help = "List wireless neighbours",
		.hint = NULL,
		.func = &list_neighbours,
		.argtable = NULL,
	};

	esp_err_t err = esp_console_cmd_register(&list_neighbours_cmd);
	if (err) {
		return err;
	}

	const esp_console_cmd_t serve_ota_cmd = {
		.command = "serve_ota",
		.help = "Serve own firmware via OTA update to neighbours",
		.hint = NULL,
		.func = &serve_ota,
		.argtable = NULL,
	};

	err = esp_console_cmd_register(&serve_ota_cmd);
	if (err) {
		return err;
	}

	const esp_console_cmd_t stop_serving_ota_cmd = {
		.command = "stop_serving_ota",
		.help = "Stop serving own firmware via OTA",
		.hint = NULL,
		.func = &stop_serving_ota,
		.argtable = NULL,
	};

	err = esp_console_cmd_register(&stop_serving_ota_cmd);
	if (err) {
		return err;
	}

	const esp_console_cmd_t node_info_cmd = {
		.command = "node_info",
		.help = "Show information on local node",
		.hint = NULL,
		.func = &node_info,
		.argtable = NULL,
	};

	err = esp_console_cmd_register(&node_info_cmd);
	if (err) {
		return err;
	}

	const esp_console_cmd_t ota_status_cmd = {
		.command = "ota_status",
		.help = "Show OTA update status",
		.hint = NULL,
		.func = &ota_status,
		.argtable = NULL,
	};

	err = esp_console_cmd_register(&ota_status_cmd);
	if (err) {
		return err;
	}

	esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
	err = esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl);
	if (err) {
		return err;
	}

	return esp_console_start_repl(repl);
}

