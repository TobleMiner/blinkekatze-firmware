#pragma once

#include "platform.h"
#include "neighbour.h"

void node_info_init(platform_t *platform);
void node_info_print_local(void);
void node_info_print_remote(const neighbour_t *neigh);
