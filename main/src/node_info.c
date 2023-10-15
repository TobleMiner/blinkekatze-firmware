#include "node_info.h"

#include <stdio.h>

#include <esp_app_desc.h>
#include <esp_mac.h>

#include "util.h"
#include "wireless.h"

void node_info_print_local() {
	const uint8_t *address = wireless_get_mac_address();
	printf("Node address:     "MACSTR"\r\n", MAC2STR(address));
	const esp_app_desc_t *app_desc = esp_app_get_description();
	printf("Firmware version: %s\r\n", app_desc->version);
	char firmware_hash_str[32 * 2 + 1] = { 0 };
	hex_encode(app_desc->app_elf_sha256, sizeof(app_desc->app_elf_sha256), firmware_hash_str, sizeof(firmware_hash_str) - 1);
	printf("Firmware SHA256:  %s\r\n", firmware_hash_str);
}
