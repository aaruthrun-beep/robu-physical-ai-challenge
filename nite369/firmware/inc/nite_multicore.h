#ifndef NITE_MULTICORE_H
#define NITE_MULTICORE_H

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "nite_transmission.h"

/**
 * @brief Thread-safe data sharing between Core 0 (Comm) and Core 1 (PID).
 * 
 * Uses hardware spinlocks to ensure the PID loop never reads a half-written 
 * target command from the SPI/UDP interrupt.
 * 
 * NOTE: Storage is in nite_multicore.c — do NOT declare instances elsewhere.
 */

typedef struct {
    int32_t targets[AXES_PER_SLAVE];
    bool update_ready;
} astra_shared_state_t;

/* Storage defined in nite_multicore.c — extern linkage avoids duplicate instances */
extern astra_shared_state_t _shared;
extern volatile spin_lock_t *_lock;

static inline void astra_sync_init(void) {
    uint lock_num = spin_lock_claim_unused(true);
    _lock = spin_lock_instance(lock_num);
}

/* Core 0 calls this when a new SPI/UDP packet arrives */
static inline void astra_push_targets(int32_t *new_targets) {
    uint32_t irq = spin_lock_blocking(_lock);
    for (int i = 0; i < AXES_PER_SLAVE; i++) {
        _shared.targets[i] = new_targets[i];
    }
    _shared.update_ready = true;
    spin_unlock(_lock, irq);
}

/* Core 1 calls this at the start of every control cycle */
static inline void astra_pull_targets(int32_t *copy_to) {
    uint32_t irq = spin_lock_blocking(_lock);
    for (int i = 0; i < AXES_PER_SLAVE; i++) {
        copy_to[i] = _shared.targets[i];
    }
    _shared.update_ready = false;
    spin_unlock(_lock, irq);
}

#endif // NITE_MULTICORE_H
