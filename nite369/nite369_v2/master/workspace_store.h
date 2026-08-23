#ifndef WORKSPACE_STORE_H
#define WORKSPACE_STORE_H

#include <stdint.h>
#include <stdbool.h>

// ── Flash layout ──────────────────────────────────────────────────────
// Waypoints + macros live in two 4KB sectors just below the config store
// (0x1EF000).  Each sector is erased independently on save.
//
//   0x1ED000  waypoint_store_t  (4KB sector)
//   0x1EE000  macro_store_t     (4KB sector)
//   0x1EF000  config_t          (4KB sector — existing)
//
#define WP_FLASH_OFFSET   (2u * 1024u * 1024u - 64u * 1024u - 8192u)  // 0x1ED000
#define MAC_FLASH_OFFSET  (2u * 1024u * 1024u - 64u * 1024u - 4096u)  // 0x1EE000

#define WP_MAGIC   0x57503031  // "WP01"
#define MAC_MAGIC  0x4D414330  // "MAC0"

// ── Waypoints ─────────────────────────────────────────────────────────
#define WP_NAME_LEN  12
#define MAX_WAYPOINTS 30

typedef struct {
    char    name[WP_NAME_LEN];
    float   pos[6];          // commanded joint positions (degrees)
    bool    valid;
    uint8_t _pad[3];
} waypoint_t;

typedef struct {
    uint32_t    magic;
    uint32_t    version;
    uint32_t    count;
    waypoint_t  items[MAX_WAYPOINTS];
    uint32_t    checksum;
} waypoint_store_t;

// ── Macros ────────────────────────────────────────────────────────────
#define MAC_CMD_LEN    64
#define MAX_MACRO_STEPS 128

typedef struct {
    char cmd[MAC_CMD_LEN];
} macro_step_t;

typedef struct {
    uint32_t       magic;
    uint32_t       version;
    uint32_t       count;
    macro_step_t   steps[MAX_MACRO_STEPS];
    uint32_t       checksum;
} macro_store_t;

// ── API ───────────────────────────────────────────────────────────────
void workspace_init(void);

// Waypoints
int   wp_find(const char *name);                    // index or -1
bool  wp_save(const char *name, const float pos[6]); // save/overwrite
bool  wp_load(int index, float pos[6]);              // load by index
bool  wp_delete(const char *name);
int   wp_count(void);
const waypoint_t *wp_get(int index);                 // read-only access
bool  wp_persist(void);                              // flush to flash

// Macros
bool  mac_start_record(void);                        // clear + begin
bool  mac_record_step(const char *cmd);              // append one command
bool  mac_stop_record(void);                         // finalize
bool  mac_play(int *step_out);                       // replay, return step count
int   mac_count(void);
bool  mac_persist(void);
void  mac_clear(void);
const macro_step_t *wp_get_macro_step(int index);

#endif
