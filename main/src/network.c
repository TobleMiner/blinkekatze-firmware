#include "network.h"

#include <esp_event.h>
#include <esp_netif.h>
#include <nvs_flash.h>

static bool network_initialized = false;

static esp_err_t network_init_(void) {
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase();
		err = nvs_flash_init();
	}
	if (err) {
		return err;
	}

	err = esp_netif_init();
	if (err) {
		return err;
	}

	return esp_event_loop_create_default();
}

esp_err_t network_init() {
	esp_err_t err = 0;

	if (!network_initialized) {
		err = network_init_();
		network_initialized = true;
	}

	return err;
}
