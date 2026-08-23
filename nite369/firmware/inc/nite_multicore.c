/**
 * @file nite_multicore.c
 * @brief Storage for multicore shared state.
 *
 * Storage must be in a .c file (not a header) to prevent duplicate instances
 * when nite_multicore.h is included from multiple translation units.
 * The header declares these as extern.
 */
#include "nite_multicore.h"

astra_shared_state_t _shared;
volatile spin_lock_t *_lock;
