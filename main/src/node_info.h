#pragma once

#include "bq27546.h"
#include "neighbour.h"

void node_info_init(bq27546_t *gauge);
void node_info_print_local(void);
void node_info_print_remote(const neighbour_t *neigh);
