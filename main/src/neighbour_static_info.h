#pragma once

#include <stdint.h>

#include "neighbour.h"
#include "wireless.h"

void neighbour_static_info_rx(const wireless_packet_t *packet, const neighbour_t *neigh);
void neighbour_static_info_update(void);
