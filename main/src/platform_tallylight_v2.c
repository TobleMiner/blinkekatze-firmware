#include "platform_tallylight_v2.h"

#include <stdlib.h>

#include <driver/gpio.h>
#include <esp_eth.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_mac.h>
#include <esp_rom_gpio.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <hal/gpio_hal.h>
#include <hal/gpio_ll.h>
#include <sdkconfig.h>
#include <soc/gpio_sig_map.h>
#include <soc/io_mux_reg.h>

#include "color.h"
#include "embedded_files.h"
#include "gpio.h"
#include "network.h"
#include "util.h"

#define REV_ID_GPIO	4
#define MAC_CS_GPIO	2
#define MAC_SI_GPIO	8
#define MAC_RST_GPIO	10

#define NUM_LEDS	10
#define BITS_PER_SYMBOL	4
#define SYMBOL_ZERO	0b0001
#define SYMBOL_ONE	0b0011

#define BYTES_DATA	((NUM_LEDS) * 24 * (BITS_PER_SYMBOL)) / 8
#define BYTES_RESET	(250 / 8)

static const char *TAG = "TallyLight v2";
//static const size_t dma_buf_len = ALIGN_UP(BYTES_RESET + BYTES_DATA + BYTES_RESET, 4);

static const rgb16_t *colorcal_table = (const rgb16_t *)EMBEDDED_FILE_PTR(colorcal_16x16x16_9bit_bin);

#define COLOR_TABLE_SIZE	16UL
#define LOOKUP_DIV		((1 << 16) / COLOR_TABLE_SIZE)

static void lookup_color(const rgb16_t *in, rgb16_t *out) {
	uint32_t idx = ((in->g * COLOR_TABLE_SIZE) + in->b) * COLOR_TABLE_SIZE + in->r;
	*out = colorcal_table[idx];
}

static void apply_color_correction_per_channel(const rgb16_t *in, rgb16_t *out) {
	rgb16_t color_min_lookup = { in->r / LOOKUP_DIV, in->g / LOOKUP_DIV, in->b / LOOKUP_DIV };
	rgb16_t color_ref_lookup = {
		MIN(in->r / LOOKUP_DIV + DIV_ROUND(in->r % LOOKUP_DIV, LOOKUP_DIV), COLOR_TABLE_SIZE - 1),
		MIN(in->g / LOOKUP_DIV + DIV_ROUND(in->g % LOOKUP_DIV, LOOKUP_DIV), COLOR_TABLE_SIZE - 1),
		MIN(in->b / LOOKUP_DIV + DIV_ROUND(in->b % LOOKUP_DIV, LOOKUP_DIV), COLOR_TABLE_SIZE - 1),
	};
	rgb16_t color_lookup_r = {
		MIN(MAX((int)color_ref_lookup.r + (color_ref_lookup.r == color_min_lookup.r ? 1 : -1), 0), COLOR_TABLE_SIZE - 1),
		color_ref_lookup.g,
		color_ref_lookup.b
	};
	rgb16_t color_lookup_g = {
		color_ref_lookup.r,
		MIN(MAX((int)color_ref_lookup.g + (color_ref_lookup.g == color_min_lookup.g ? 1 : -1), 0), COLOR_TABLE_SIZE - 1),
		color_ref_lookup.b
	};
	rgb16_t color_lookup_b = {
		color_ref_lookup.r,
		color_ref_lookup.g,
		MIN(MAX((int)color_ref_lookup.b + (color_ref_lookup.b == color_min_lookup.b ? 1 : -1), 0), COLOR_TABLE_SIZE - 1)
	};
	rgb16_t color_ref = {
		color_ref_lookup.r * LOOKUP_DIV,
		color_ref_lookup.g * LOOKUP_DIV,
		color_ref_lookup.b * LOOKUP_DIV
	};
	rgb16_t color_max = {
		color_lookup_r.r * LOOKUP_DIV,
		color_lookup_g.g * LOOKUP_DIV,
		color_lookup_b.b * LOOKUP_DIV
	};
	rgb16_t color_corrected_ref;
	rgb16_t color_corrected_r;
	rgb16_t color_corrected_g;
	rgb16_t color_corrected_b;
	lookup_color(&color_ref_lookup, &color_corrected_ref);
	lookup_color(&color_lookup_r, &color_corrected_r);
	lookup_color(&color_lookup_g, &color_corrected_g);
	lookup_color(&color_lookup_b, &color_corrected_b);

	int32_t delta_in_r = (int32_t)in->r - (int32_t)color_ref.r;
	int32_t delta_in_g = (int32_t)in->g - (int32_t)color_ref.g;
	int32_t delta_in_b = (int32_t)in->b - (int32_t)color_ref.b;
	int32_t delta_ref_r = (int32_t)color_max.r - (int32_t)color_ref.r;
	int32_t delta_ref_g = (int32_t)color_max.g - (int32_t)color_ref.g;
	int32_t delta_ref_b = (int32_t)color_max.b - (int32_t)color_ref.b;
	rgb16_t color_out = color_corrected_ref;

	if (delta_ref_r) {
		color_out.r += DIV_ROUND(((int32_t)color_corrected_r.r - (int32_t)color_corrected_ref.r) * delta_in_r, delta_ref_r);
		color_out.g += DIV_ROUND(((int32_t)color_corrected_r.g - (int32_t)color_corrected_ref.g) * delta_in_r, delta_ref_r);
		color_out.b += DIV_ROUND(((int32_t)color_corrected_r.b - (int32_t)color_corrected_ref.b) * delta_in_r, delta_ref_r);
	}
	if (delta_ref_g) {
		color_out.r += DIV_ROUND(((int32_t)color_corrected_g.r - (int32_t)color_corrected_ref.r) * delta_in_g, delta_ref_g);
		color_out.g += DIV_ROUND(((int32_t)color_corrected_g.g - (int32_t)color_corrected_ref.g) * delta_in_g, delta_ref_g);
		color_out.b += DIV_ROUND(((int32_t)color_corrected_g.b - (int32_t)color_corrected_ref.b) * delta_in_g, delta_ref_g);
	}
	if (delta_ref_b) {
		color_out.r += DIV_ROUND(((int32_t)color_corrected_b.r - (int32_t)color_corrected_ref.r) * delta_in_b, delta_ref_b);
		color_out.g += DIV_ROUND(((int32_t)color_corrected_b.g - (int32_t)color_corrected_ref.g) * delta_in_b, delta_ref_b);
		color_out.b += DIV_ROUND(((int32_t)color_corrected_b.b - (int32_t)color_corrected_ref.b) * delta_in_b, delta_ref_b);
	}

	*out = color_out;
}

static void leds_set_color(uint8_t *data, uint16_t r, uint16_t g, uint16_t b) {
	rgb16_t color_in = { r, g, b };

	rgb16_t color_out;
	apply_color_correction_per_channel(&color_in, &color_out);

	uint16_t r_corrected = color_out.r;
	uint16_t g_corrected = color_out.g;
	uint16_t b_corrected = color_out.b;

	uint8_t local_r1 = data[1] = MIN(r_corrected, 255);
	uint8_t local_g1 = data[0] = MIN(g_corrected, 255);
	uint8_t local_b1 = data[2] = MIN(b_corrected, 255);
	r_corrected -= (uint16_t)local_r1;
	g_corrected -= (uint16_t)local_g1;
	b_corrected -= (uint16_t)local_b1;

	uint8_t local_r2 = data[4] = MIN(r_corrected, 255);
	uint8_t local_g2 = data[3] = MIN(g_corrected, 255);
	uint8_t local_b2 = data[5] = MIN(b_corrected, 255);
	r_corrected -= (uint16_t)local_r2;
	g_corrected -= (uint16_t)local_g2;
	b_corrected -= (uint16_t)local_b2;
}

static void set_rdb_led_color(platform_t *platform, uint16_t r, uint16_t g, uint16_t b) {
	platform_tallylight_v2_t *plat = container_of(platform, platform_tallylight_v2_t, base);

	leds_set_color(plat->led_color_data, r, g, b);

	static const rmt_transmit_config_t tx_config = {
		.loop_count = DIV_ROUND_UP(NUM_LEDS, 2),
		.flags = {
			.eot_level = 0
		}
	};
	rmt_transmit(plat->rmt_channel, plat->rmt_encoder, plat->led_color_data, sizeof(plat->led_color_data), &tx_config);
}

static const platform_ops_t lacklight_ops = {
	.probe = platform_tallylight_v2_probe,
	.set_rgb_led_color = set_rdb_led_color
};

platform_def_t platform_tallylight_v2 = {
	.ops = &lacklight_ops,
	.name = "tallylight_v2"
};

static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
	uint8_t mac_addr[6] = {0};
	esp_eth_handle_t eth_handle = *(esp_eth_handle_t*)event_data;

	switch (event_id) {
	case ETHERNET_EVENT_CONNECTED:
		esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
		ESP_LOGI(TAG, "Ethernet Link Up");
		ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
			 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
		break;
	case ETHERNET_EVENT_DISCONNECTED:
		ESP_LOGI(TAG, "Ethernet Link Down");
		break;
	case ETHERNET_EVENT_START:
		ESP_LOGI(TAG, "Ethernet Started");
		break;
	case ETHERNET_EVENT_STOP:
		ESP_LOGI(TAG, "Ethernet Stopped");
	}
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
	ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
	const esp_netif_ip_info_t *ip_info = &event->ip_info;

	ESP_LOGI(TAG, "Ethernet Got IP Address");
	ESP_LOGI(TAG, "~~~~~~~~~~~");
	ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
	ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
	ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
	ESP_LOGI(TAG, "~~~~~~~~~~~");
}

static const rmt_symbol_word_t sk6812_symbol_zero = {
	.level0 = 1,
	.duration0 = 3,
	.level1 = 0,
	.duration1 = 9
};

static const rmt_symbol_word_t sk6812_symbol_one = {
	.level0 = 1,
	.duration0 = 6,
	.level1 = 0,
	.duration1 = 6
};

static size_t rmt_led_data_encoder_callback(const void *data, size_t data_size, size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg) {
	size_t data_pos = symbols_written / 8;
	size_t num_symbols = 0;
	while (data_pos < data_size && symbols_free >= 8) {
		const uint8_t *data_u8 = data;
		uint8_t byt = data_u8[data_pos];
		for (uint8_t bit = 0x80; bit; bit >>= 1) {
			if (byt & bit) {
				symbols[num_symbols] = sk6812_symbol_one;
			} else {
				symbols[num_symbols] = sk6812_symbol_zero;
			}
			num_symbols++;
			symbols_free--;
		}
		data_pos++;
	}
	if (data_pos >= data_size) {
		*done = true;
	}
	return num_symbols;
}

esp_err_t platform_tallylight_v2_probe(platform_t **ret) {
	platform_tallylight_v2_t *tally = calloc(1, sizeof(platform_tallylight_v2_t));
	if (!tally) {
		return ESP_ERR_NO_MEM;
	}

	if (gpio_get_level(REV_ID_GPIO) != 0) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_CS_GPIO) != 1) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_SI_GPIO) != 1) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_RST_GPIO) != 1) {
		return ESP_FAIL;
	}

	gpio_set_pull_mode(REV_ID_GPIO, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(MAC_CS_GPIO, GPIO_PULLDOWN_ONLY);
	gpio_set_pull_mode(MAC_SI_GPIO, GPIO_PULLDOWN_ONLY);
	gpio_set_pull_mode(MAC_RST_GPIO, GPIO_PULLDOWN_ONLY);
	vTaskDelay(pdMS_TO_TICKS(100));

	if (gpio_get_level(REV_ID_GPIO) != 0) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_CS_GPIO) != 1) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_SI_GPIO) != 1) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_RST_GPIO) != 1) {
		return ESP_FAIL;
	}

	static const rmt_tx_channel_config_t tx_chan_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.gpio_num = 6,
		.mem_block_symbols = 48 * 2,
		.resolution_hz = 10 * 1000 * 1000,
		.trans_queue_depth = 4
	};
	rmt_channel_handle_t led_chan;
	ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

	rmt_encoder_handle_t led_encoder;
	static const rmt_simple_encoder_config_t led_encoder_cfg = {
		.callback = rmt_led_data_encoder_callback
	};
	ESP_ERROR_CHECK(rmt_new_simple_encoder(&led_encoder_cfg, &led_encoder));
	ESP_ERROR_CHECK(rmt_enable(led_chan));
	tally->rmt_channel = led_chan;
	tally->rmt_encoder = led_encoder;

	gpio_init();
	ESP_ERROR_CHECK(network_init());

	esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
	esp_netif_config.if_key = "ETH_SPI_0";
	esp_netif_config.if_desc = "eth0";
	esp_netif_config.route_prio = 30;

	esp_netif_config_t cfg_spi = {
		.base = &esp_netif_config,
		.stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
	};

	esp_netif_t *eth_netif_spi = esp_netif_new(&cfg_spi);

	eth_mac_config_t mac_config_spi = ETH_MAC_DEFAULT_CONFIG();
	eth_phy_config_t phy_config_spi = ETH_PHY_DEFAULT_CONFIG();

	spi_bus_config_t spi_bus_cfg = {
		.mosi_io_num = 8,
		.miso_io_num = 3,
		.sclk_io_num = 1,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS
	};
	ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));

	esp_eth_mac_t *mac_spi;;
	esp_eth_phy_t *phy_spi;
	esp_eth_handle_t eth_handle_spi;

	spi_device_interface_config_t spi_devcfg = {
		.mode = 0,
		.clock_speed_hz = 10 * 1000 * 1000,
		.queue_size = 20,
	        .spics_io_num = 2
	};

	phy_config_spi.phy_addr = 1;
	phy_config_spi.reset_gpio_num = 10;

	eth_ksz8851snl_config_t ksz8851snl_config = ETH_KSZ8851SNL_DEFAULT_CONFIG(SPI2_HOST, &spi_devcfg);
	ksz8851snl_config.int_gpio_num = 7;
	mac_spi = esp_eth_mac_new_ksz8851snl(&ksz8851snl_config, &mac_config_spi);
	phy_spi = esp_eth_phy_new_ksz8851snl(&phy_config_spi);

	esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac_spi, phy_spi);
	ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config_spi, &eth_handle_spi));

	uint8_t eth_mac_addr[ETH_ADDR_LEN];
	ESP_ERROR_CHECK(esp_read_mac(eth_mac_addr, ESP_MAC_ETH));
	ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle_spi, ETH_CMD_S_MAC_ADDR, eth_mac_addr));

	ESP_ERROR_CHECK(esp_netif_attach(eth_netif_spi, esp_eth_new_netif_glue(eth_handle_spi)));

	ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

	ESP_ERROR_CHECK(esp_eth_start(eth_handle_spi));

	platform_init(&tally->base, &platform_tallylight_v2);
	*ret = &tally->base;

	return 0;
}
