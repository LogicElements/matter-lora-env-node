#pragma once

#include <sdkconfig.h>

// Increase secure session pool so more simultaneous CASE sessions can exist
#define CHIP_CONFIG_SECURE_SESSION_POOL_SIZE 10

// Interaction Model handler pools (see spec 8.5.1 minimums)
#define CHIP_IM_MAX_NUM_READS 6
#define CHIP_IM_MAX_NUM_SUBSCRIPTIONS (CHIP_CONFIG_MAX_FABRICS * 3) // 15 when MAX_FABRICS=5
#define CHIP_IM_MAX_NUM_WRITE_HANDLER 3
