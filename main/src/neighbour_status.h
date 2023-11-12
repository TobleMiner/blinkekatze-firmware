#pragma once

#include <stdint.h>

#include "bq27546.h"
#include "neighbour.h"
#include "wireless.h"

void neighbour_status_init(bq27546_t *battery_gauge);
void neighbour_status_rx(const wireless_packet_t *packet, const neighbour_t *neigh);
