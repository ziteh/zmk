#include <zephyr/types.h>
#include <stdlib.h>
#include "paw3395.h"

const size_t paw3395_pwrup_registers_length1 = 137;
const size_t paw3395_pwrup_registers_length2 = 5;
const size_t paw3395_runmode_registers_length[RUN_MODE_COUNT] = {
    [HP_MODE] = 21,
    [LP_MODE] = 21,
    [OFFICE_MODE] = 21,
    [GAME_MODE] = 20,
};

/* power up registers init: group1 */
const uint8_t paw3395_pwrup_register_addr1[] = {
};
const uint8_t paw3395_pwrup_register_data1[] = {
};

/* power up registers init: group2 */
const uint8_t paw3395_pwrup_register_addr2[] = {
};
const uint8_t paw3395_pwrup_register_data2[] = {
};

/* hp mode registers */
const uint8_t paw3395_hp_register_addr[] = {
};
const uint8_t paw3395_hp_register_data[] = {
};

/* lp mode registers */
const uint8_t paw3395_lp_register_addr[] = {
};
const uint8_t paw3395_lp_register_data[] = {
};

/* office mode registers */
const uint8_t paw3395_office_register_addr[] = {
};
const uint8_t paw3395_office_register_data[] = {
};

/* game mode registers */
const uint8_t paw3395_game_register_addr[] = {
};
const uint8_t paw3395_game_register_data[] = {
};

/* aggregation of all run modes registers */
const uint8_t* paw3395_rumode_register_addr[] = {
  [HP_MODE] = paw3395_hp_register_addr,
  [LP_MODE] = paw3395_lp_register_addr,
  [OFFICE_MODE] = paw3395_office_register_addr,
  [GAME_MODE] = paw3395_game_register_addr,
};
const uint8_t* paw3395_rumode_register_data[] = {
  [HP_MODE] = paw3395_hp_register_data,
  [LP_MODE] = paw3395_lp_register_data,
  [OFFICE_MODE] = paw3395_office_register_data,
  [GAME_MODE] = paw3395_game_register_data,
};
