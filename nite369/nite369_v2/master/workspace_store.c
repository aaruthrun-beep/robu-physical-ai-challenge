// workspace_store.c — waypoints + macros flash storage for the master RP2040
#include "workspace_store.h"
#include "config_store.h"   // for CONFIG_MAGIC / checksum pattern
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>
#include <stdio.h>

static waypoint_store_t  wp_store;
static macro_store_t     mac_store;
static bool wp_dirty  = false;
static bool mac_dirty = false;

// ── helpers ───────────────────────────────────────────────────────────
static uint32_t calc_checksum(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += p[i];
    return sum;
}

static void flash_write_sector(uint32_t offset, const void *src, size_t len) {
    uint8_t buf[4096];
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, src, len < sizeof(buf) ? len : sizeof(buf));
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(offset, 4096);
    flash_range_program(offset, buf, 4096);
    restore_interrupts(ints);
}

// ── init ──────────────────────────────────────────────────────────────
void workspace_init(void) {
    // Load waypoints from flash
    const waypoint_store_t *flash_wp =
        (const waypoint_store_t *)(XIP_BASE + WP_FLASH_OFFSET);
    if (flash_wp->magic == WP_MAGIC && flash_wp->count <= MAX_WAYPOINTS) {
        uint32_t cs = calc_checksum(&flash_wp->magic,
                        sizeof(uint32_t) * 3 + sizeof(waypoint_t) * flash_wp->count);
        if (cs == flash_wp->checksum) {
            memcpy(&wp_store, flash_wp, sizeof(waypoint_store_t));
            printf("[WP] Loaded %u waypoints from flash\n", wp_store.count);
        } else {
            printf("[WP] Checksum mismatch — starting empty\n");
            memset(&wp_store, 0, sizeof(wp_store));
            wp_store.magic = WP_MAGIC;
        }
    } else {
        printf("[WP] No valid flash — starting empty\n");
        memset(&wp_store, 0, sizeof(wp_store));
        wp_store.magic = WP_MAGIC;
    }

    // Load macros from flash
    const macro_store_t *flash_mac =
        (const macro_store_t *)(XIP_BASE + MAC_FLASH_OFFSET);
    if (flash_mac->magic == MAC_MAGIC && flash_mac->count <= MAX_MACRO_STEPS) {
        uint32_t cs = calc_checksum(&flash_mac->magic,
                        sizeof(uint32_t) * 3 + sizeof(macro_step_t) * flash_mac->count);
        if (cs == flash_mac->checksum) {
            memcpy(&mac_store, flash_mac, sizeof(macro_store_t));
            printf("[MAC] Loaded %u macro steps from flash\n", mac_store.count);
        } else {
            printf("[MAC] Checksum mismatch — starting empty\n");
            memset(&mac_store, 0, sizeof(mac_store));
            mac_store.magic = MAC_MAGIC;
        }
    } else {
        printf("[MAC] No valid flash — starting empty\n");
        memset(&mac_store, 0, sizeof(mac_store));
        mac_store.magic = MAC_MAGIC;
    }
}

// ── waypoints ─────────────────────────────────────────────────────────
int wp_find(const char *name) {
    for (uint32_t i = 0; i < wp_store.count; i++) {
        if (wp_store.items[i].valid && strcmp(wp_store.items[i].name, name) == 0)
            return (int)i;
    }
    return -1;
}

bool wp_save(const char *name, const float pos[6]) {
    // Overwrite existing?
    int idx = wp_find(name);
    if (idx >= 0) {
        memcpy(wp_store.items[idx].pos, pos, sizeof(float) * 6);
        wp_dirty = true;
        return true;
    }
    // Find empty slot
    if (wp_store.count >= MAX_WAYPOINTS) return false;
    waypoint_t *w = &wp_store.items[wp_store.count];
    memset(w, 0, sizeof(*w));
    strncpy(w->name, name, WP_NAME_LEN - 1);
    w->name[WP_NAME_LEN - 1] = '\0';
    memcpy(w->pos, pos, sizeof(float) * 6);
    w->valid = true;
    wp_store.count++;
    wp_dirty = true;
    return true;
}

bool wp_load(int index, float pos[6]) {
    if (index < 0 || index >= (int)wp_store.count) return false;
    if (!wp_store.items[index].valid) return false;
    memcpy(pos, wp_store.items[index].pos, sizeof(float) * 6);
    return true;
}

bool wp_delete(const char *name) {
    int idx = wp_find(name);
    if (idx < 0) return false;
    // Compact: shift remaining items down
    wp_store.items[idx].valid = false;
    for (int i = idx; i < (int)wp_store.count - 1; i++) {
        if (!wp_store.items[i + 1].valid) break;
        wp_store.items[i] = wp_store.items[i + 1];
        wp_store.items[i + 1].valid = false;
    }
    // Trim trailing invalid items
    while (wp_store.count > 0 && !wp_store.items[wp_store.count - 1].valid)
        wp_store.count--;
    wp_dirty = true;
    return true;
}

int wp_count(void) { return (int)wp_store.count; }

const waypoint_t *wp_get(int index) {
    if (index < 0 || index >= (int)wp_store.count) return NULL;
    return &wp_store.items[index];
}

bool wp_persist(void) {
    if (!wp_dirty) return true;
    wp_store.checksum = calc_checksum(&wp_store.magic,
        sizeof(uint32_t) * 3 + sizeof(waypoint_t) * wp_store.count);
    flash_write_sector(WP_FLASH_OFFSET, &wp_store, sizeof(waypoint_store_t));
    wp_dirty = false;
    printf("[WP] Persisted %u waypoints to flash\n", wp_store.count);
    return true;
}

// ── macros ────────────────────────────────────────────────────────────
bool mac_start_record(void) {
    memset(&mac_store, 0, sizeof(mac_store));
    mac_store.magic = MAC_MAGIC;
    mac_store.count = 0;
    mac_dirty = true;
    return true;
}

bool mac_record_step(const char *cmd) {
    if (mac_store.count >= MAX_MACRO_STEPS) return false;
    macro_step_t *s = &mac_store.steps[mac_store.count];
    memset(s, 0, sizeof(*s));
    strncpy(s->cmd, cmd, MAC_CMD_LEN - 1);
    s->cmd[MAC_CMD_LEN - 1] = '\0';
    mac_store.count++;
    mac_dirty = true;
    return true;
}

bool mac_stop_record(void) {
    // Finalize — trim if needed
    mac_dirty = true;
    return true;
}

bool mac_play(int *step_out) {
    if (mac_store.count == 0) return false;
    if (step_out) *step_out = (int)mac_store.count;
    return true;
}

int mac_count(void) { return (int)mac_store.count; }

bool mac_persist(void) {
    if (!mac_dirty) return true;
    mac_store.checksum = calc_checksum(&mac_store.magic,
        sizeof(uint32_t) * 3 + sizeof(macro_step_t) * mac_store.count);
    flash_write_sector(MAC_FLASH_OFFSET, &mac_store, sizeof(macro_store_t));
    mac_dirty = false;
    printf("[MAC] Persisted %u macro steps to flash\n", mac_store.count);
    return true;
}

void mac_clear(void) {
    memset(&mac_store, 0, sizeof(mac_store));
    mac_store.magic = MAC_MAGIC;
    mac_dirty = true;
}

const macro_step_t *wp_get_macro_step(int index) {
    if (index < 0 || index >= (int)mac_store.count) return NULL;
    return &mac_store.steps[index];
}
