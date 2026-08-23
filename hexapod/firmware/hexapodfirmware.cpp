#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h>
#include <stdlib.h>

#define SDA_PIN 4
#define SCL_PIN 5
#define TCA_ADDR 0x70
#define ADS_ADDR 0x48
#define PCA1_ADDR 0x40
#define PCA2_ADDR 0x41

// ═══════════════════════════════════════
// I2C HELPERS
// ═══════════════════════════════════════
void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(i2c0, addr, buf, 2, false);
}

uint8_t i2c_read_reg(uint8_t addr, uint8_t reg) {
    i2c_write_blocking(i2c0, addr, &reg, 1, true);
    uint8_t val;
    i2c_read_blocking(i2c0, addr, &val, 1, false);
    return val;
}

// ═══════════════════════════════════════
// TCA9548A
// ═══════════════════════════════════════
void tca_select(uint8_t ch) {
    uint8_t buf = (ch > 7) ? 0 : (1 << ch);
    i2c_write_blocking(i2c0, TCA_ADDR, &buf, 1, false);
}

void tca_close() {
    uint8_t buf = 0x00;
    i2c_write_blocking(i2c0, TCA_ADDR, &buf, 1, false);
}

// ═══════════════════════════════════════
// PCA9685
// ═══════════════════════════════════════
void pca_init(uint8_t addr) {
    i2c_write_reg(addr, 0x00, 0x10);   // sleep
    sleep_ms(10);
    i2c_write_reg(addr, 0xFE, 121);    // 50Hz
    sleep_ms(10);
    i2c_write_reg(addr, 0x00, 0x80);   // restart
    sleep_ms(10);
    i2c_write_reg(addr, 0x00, 0x20);   // normal + AI=1 (enable auto-increment)
    sleep_ms(10);
}

void set_servo(uint8_t pca_addr, uint8_t channel, uint16_t pulse) {
    if (pulse < 102) pulse = 102;
    // Upper clamp allows effective angles up to MAX_EFF 200 (see set_servo_gui):
    // a positive cal_offset pushes the commanded range upward, so offset
    // joints must reach eff 190..200 (= pulse ~535..557, 2.6..2.72 ms) to use
    // their full commanded travel. KEEP 560 ≥ pulse at eff 200 if MAX_EFF
    // is ever tuned (pulse = 102 + eff/180*410 → 557.6 at eff 200).
    if (pulse > 560) pulse = 560;
    uint8_t base = 0x06 + channel * 4;
    uint8_t buf[5] = {
        base,
        0x00, 0x00,
        (uint8_t)(pulse & 0xFF),
        (uint8_t)(pulse >> 8)
    };
    i2c_write_blocking(i2c0, pca_addr, buf, 5, false);
}

void set_servo_angle(uint8_t pca_addr, uint8_t channel, float angle) {
    if (angle < 0) angle = 0;
    if (angle > 140) angle = 140;
    uint16_t pulse = (uint16_t)(102 + (angle / 140.0f) * (350 - 102));
    set_servo(pca_addr, channel, pulse);
}

// ── GUI Channel Mapping ────────────────────────────────────────────
// GUI channel 0-31 → PCA# (ch/16) + channel (ch%16), full 0-180° range
// 0-15 → PCA1 (0x40), 16-31 → PCA2 (0x41)
int servo_pos[32];

// ── Calibration ────────────────────────────────────────────────────
// Per-channel: offset added to commanded angle so 90° = mechanically
// straight, plus hard min/max limits so legs can't over-travel.
//
// Limits live in EFFECTIVE space (eff = commanded + offset) and span
// 0..200 — wider than 0..180 so a joint with a positive zero-offset can
// still be commanded through its full range (e.g. offset +20 allows
// commanded 0..180 = eff 20..200 instead of stopping at commanded 160).
#define MAX_EFF 200
int cal_offset[32];
int cal_min[32];
int cal_max[32];

// ── Channel Mirror (upside-down soldered board) ─────────────────────
// If the PCA board is soldered flipped, the header at the "ch0"
// position is electrically ch15. Mirroring remaps:
//   physical = 15 - (ch % 16)  when enabled for that board.
int mirror_pca1 = 0;  // 0x40
int mirror_pca2 = 0;  // 0x41

void cal_init() {
    for (int i = 0; i < 32; i++) {
        cal_offset[i] = 0;
        cal_min[i]    = 0;
        cal_max[i]    = MAX_EFF;
    }
}

void set_servo_gui(int ch, int angle) {
    if (ch < 0 || ch > 31) { printf("ERR ch out of range\n"); return; }
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    // Apply calibration: offset first, then clamp to per-channel limits.
    // eff may exceed 180 — the offset pushes the commanded range up
    // (commanded 180 + offset 20 = eff 200); set_servo accepts the wider
    // pulses, so an offset joint reaches its full commanded travel.
    int eff = angle + cal_offset[ch];
    if (eff < cal_min[ch]) eff = cal_min[ch];
    if (eff > cal_max[ch]) eff = cal_max[ch];
    uint8_t pca = (ch < 16) ? PCA1_ADDR : PCA2_ADDR;
    uint8_t pch = (uint8_t)(ch % 16);
    int mirror = (pca == PCA1_ADDR) ? mirror_pca1 : mirror_pca2;
    if (mirror) pch = (uint8_t)(15 - pch);
    uint16_t pulse = (uint16_t)(102 + (eff / 180.0f) * (512 - 102));
    set_servo(pca, pch, pulse);
    servo_pos[ch] = angle;  // report commanded angle (pre-offset)
}

// ═══════════════════════════════════════
// MOVEMENT — LEG MAP + POSES
// ═══════════════════════════════════════
// Physical wiring (user-verified, with 'mirror all' ON):
//   PCA1: Leg1 = ch 15,14,13 | Leg2 = 12,11,10 | Leg3 = 9,8,7
//   PCA2: Leg4 = ch 31,30,29 | Leg5 = 28,27,26 | Leg6 = 25,24,23
//   (order: coxa, femur, tibia)
int leg_map[6][3] = {
    {15, 14, 13},
    {12, 11, 10},
    { 9,  8,  7},
    {31, 30, 29},
    {28, 27, 26},
    {25, 24, 23}
};

// Pose tables (coxa, femur, tibia) — 0-180 GUI space.
// Starting values; tune per-channel with 'cal' commands.
int pose_home[3]   = { 90,  70, 110 };   // HOME standing pose — the CENTER reference for calibration + IK
int pose_stand[3]  = { 90, 135, 100 };   // body up, legs extended
int pose_sit[3]    = { 90,  70, 160 };   // legs folded, body low
int pose_lift[3]   = { 90, 160,  60 };   // leg raised off ground
int pose_crouch[3] = { 90, 100, 130 };   // low stance

// Movement engine: servo_target[] is where each channel should go.
// movement_update() steps servo_pos[] toward it smoothly, writing via
// set_servo_gui so calibration + mirror always apply.
int servo_target[32];
int move_step = 2;   // degrees per tick (main loop ticks ~20ms)

int pose_lookup(const char* name) {
    if (strcmp(name, "home") == 0)   return 0;
    if (strcmp(name, "stand") == 0)  return 1;
    if (strcmp(name, "sit") == 0)    return 2;
    if (strcmp(name, "lift") == 0)   return 3;
    if (strcmp(name, "crouch") == 0) return 4;
    return -1;
}

const char* pose_name(int idx) {
    switch (idx) {
        case 0: return "home";
        case 1: return "stand";
        case 2: return "sit";
        case 3: return "lift";
        case 4: return "crouch";
    }
    return "?";
}

void set_leg_pose(int leg, int pidx) {
    int* pose = (pidx == 0) ? pose_home : (pidx == 1) ? pose_stand
              : (pidx == 2) ? pose_sit : (pidx == 3) ? pose_lift : pose_crouch;
    printf("Leg %d -> %s: ch %d=%d  ch %d=%d  ch %d=%d\n",
           leg + 1, pose_name(pidx),
           leg_map[leg][0], pose[0],
           leg_map[leg][1], pose[1],
           leg_map[leg][2], pose[2]);
    for (int j = 0; j < 3; j++) servo_target[leg_map[leg][j]] = pose[j];
}

void print_leg_map() {
    printf("\nLEG MAP (coxa, femur, tibia) — valid with mirror ON:\n");
    printf("  Mirror state: PCA1=%d PCA2=%d  (1 = flipped/upside-down board)\n",
           mirror_pca1, mirror_pca2);
    if (!mirror_pca1 || !mirror_pca2) {
        printf("  ⚠ WARNING: leg map assumes BOTH boards mirrored.\n");
        printf("  Type 'mirror all' if you haven't — otherwise wrong channels will move!\n");
    }
    for (int leg = 0; leg < 6; leg++) {
        printf("  Leg %d: ch %2d, %2d, %2d\n", leg + 1,
               leg_map[leg][0], leg_map[leg][1], leg_map[leg][2]);
    }
    printf("Poses: home | stand | sit | lift | crouch   (home = CENTER)\n");
}

void movement_update() {
    for (int ch = 0; ch < 32; ch++) {
        int cur = servo_pos[ch];
        int tgt = servo_target[ch];
        if (cur == tgt) continue;  // idle — no I2C traffic
        int next = (tgt > cur) ? cur + move_step : cur - move_step;
        if ((next - tgt) * (cur - tgt) < 0) next = tgt;  // snap when crossed
        set_servo_gui(ch, next);
    }
}

// ═══════════════════════════════════════
// ADS1115
// ═══════════════════════════════════════
int16_t ads_read(uint8_t ain) {
    uint8_t config[3] = {
        0x01,
        (uint8_t)(0b11000100 | (ain << 4)),
        0b11100011
    };
    i2c_write_blocking(i2c0, ADS_ADDR, config, 3, false);
    sleep_ms(2);
    uint8_t reg = 0x00;
    i2c_write_blocking(i2c0, ADS_ADDR, &reg, 1, true);
    uint8_t raw[2];
    i2c_read_blocking(i2c0, ADS_ADDR, raw, 2, false);
    int16_t val = (raw[0] << 8) | raw[1];
    return val;
}

// ═══════════════════════════════════════
// SCAN
// ═══════════════════════════════════════
void scan_all() {
    printf("\n--- Main I2C Bus ---\n");
    for (int addr = 0x08; addr < 0x78; addr++) {
        uint8_t buf;
        if (i2c_read_blocking(i2c0, addr, &buf, 1, false) >= 0) {
            printf("  Found: 0x%02X\n", addr);
        }
    }

    printf("\n--- TCA Channels ---\n");
    for (int ch = 0; ch < 5; ch++) {
        tca_select(ch);
        sleep_ms(10);
        uint8_t buf;
        int ret = i2c_read_blocking(i2c0, ADS_ADDR, &buf, 1, false);
        printf("  Ch%d: ADS1115 %s\n", ch, ret >= 0 ? "OK ✓" : "NOT found ✗");
        tca_close();
        sleep_ms(10);
    }
    printf("-------------------\n");
}

// ═══════════════════════════════════════
// READ ALL ADC
// ═══════════════════════════════════════
void read_all_adc() {
    printf("\n--- ADC Readings ---\n");
    for (int ch = 0; ch < 5; ch++) {
        tca_select(ch);
        sleep_ms(10);
        printf("TCA Ch%d:\n", ch);
        for (int ain = 0; ain < 4; ain++) {
            int16_t val = ads_read(ain);
            float voltage = val * 2.048f / 32768.0f;  // PGA=010 = ±2.048V
            printf("  A%d: raw=%d  %.3fV\n", ain, val, voltage);
        }
        tca_close();
        sleep_ms(10);
    }
    printf("-------------------\n");
}

// ═══════════════════════════════════════
// FAST ADC FEEDBACK SCAN (pot positions)
// ═══════════════════════════════════════
// Reads all 20 pot channels (5 TCA mux channels × 4 AIN each) with minimal
// blocking and prints ONE compact line per scan:
//     FB <raw0> <raw1> ... <raw19>
// raws are in TCA-major order: tca0.ain0..ain3, tca1.ain0.., ... tca4.ain3
// The host (pico_serial_bridge) converts raw → servo degrees using the
// per-joint calibration in adc_feedback.yaml. The `adc`/`X` commands are
// the verbose human-readable versions; `F` is the machine-parseable one.
#define N_TCA 5
#define N_AIN 4

void scan_adc_fast() {
    printf("FB");
    for (int ch = 0; ch < N_TCA; ch++) {
        tca_select(ch);
        for (int ain = 0; ain < N_AIN; ain++) {
            int16_t val = ads_read(ain);
            printf(" %d", val);
        }
    }
    tca_close();
    printf("\n");
}

// ═══════════════════════════════════════
// PCA9685 REGISTER DUMP
// ═══════════════════════════════════════
void dump_regs() {
    uint8_t addrs[] = {PCA1_ADDR, PCA2_ADDR};
    const char* names[] = {"PCA1 (0x40)", "PCA2 (0x41)"};
    for (int i = 0; i < 2; i++) {
        uint8_t addr = addrs[i];
        // Check if PCA responds
        uint8_t test;
        if (i2c_read_blocking(i2c0, addr, &test, 1, false) < 0) {
            printf("%s: NOT RESPONDING\n", names[i]);
            continue;
        }
        uint8_t m1 = i2c_read_reg(addr, 0x00);
        uint8_t m2 = i2c_read_reg(addr, 0x01);
        // Read LED0_OFF (ch 0 off registers)
        uint8_t off_l = i2c_read_reg(addr, 0x08);
        uint8_t off_h = i2c_read_reg(addr, 0x09);
        uint16_t off = (uint16_t)((off_h << 8) | off_l);
        // Read LED5_OFF (ch 5) for comparison
        uint8_t off5_l = i2c_read_reg(addr, 0x1C);
        uint8_t off5_h = i2c_read_reg(addr, 0x1D);
        uint16_t off5 = (uint16_t)((off5_h << 8) | off5_l);

        printf("\n=== %s ===\n", names[i]);
        printf("  MODE1  (0x00) = 0x%02X\n", m1);
        printf("    SLEEP=%d AI=%d ALLCALL=%d RESTART=%d\n",
            (m1 >> 4) & 1, (m1 >> 5) & 1, m1 & 1, (m1 >> 7) & 1);
        printf("  MODE2  (0x01) = 0x%02X\n", m2);
        printf("    OUTDRV=%d OCH=%d INVRT=%d\n",
            (m2 >> 2) & 1, (m2 >> 3) & 1, (m2 >> 4) & 1);
        printf("  CH0 OFF    = %u (0x%04X) = %.1f us\n", off, off,
            off * (1000000.0f / (4096.0f * 50.0f)));
        printf("  CH5 OFF    = %u (0x%04X) = %.1f us\n", off5, off5,
            off5 * (1000000.0f / (4096.0f * 50.0f)));
        if (off >= 102 && off <= 350) {
            printf("  => CH0 PWM written (VALID pulse: %u)\n", off);
        } else if (off == 0) {
            printf("  => CH0 OFF=0 (always ON - NO servo signal)\n");
        } else if (off >= 4096) {
            printf("  => CH0 OFF>=4096 (always OFF - default state)\n");
        }
    }
    printf("\n");
}

// ═══════════════════════════════════════
// STATUS — GUI handshake info
// ═══════════════════════════════════════
void print_status() {
    uint8_t probe;
    int pca_n = 0;
    uint8_t pca_found[2];
    if (i2c_read_blocking(i2c0, PCA1_ADDR, &probe, 1, false) >= 0) pca_found[pca_n++] = PCA1_ADDR;
    if (i2c_read_blocking(i2c0, PCA2_ADDR, &probe, 1, false) >= 0) pca_found[pca_n++] = PCA2_ADDR;

    printf("FOUND_PCA %d\n", pca_n);
    for (int i = 0; i < pca_n; i++) {
        printf("PCA_INIT %d 0x%02X OK\n", i, pca_found[i]);
    }

    int tca_present = (i2c_read_blocking(i2c0, TCA_ADDR, &probe, 1, false) >= 0);
    printf("TCA_PRESENT %d\n", tca_present ? 1 : 0);

    int ads_n = 0;
    for (int ch = 0; ch < 5; ch++) {
        tca_select(ch);
        sleep_ms(5);
        if (i2c_read_blocking(i2c0, ADS_ADDR, &probe, 1, false) >= 0) {
            printf("ADS_ %d TCA_CH %d 0x%02X\n", ads_n, ch, ADS_ADDR);
            ads_n++;
        }
        tca_close();
        sleep_ms(5);
    }
    printf("ADS_FOUND %d\n", ads_n);
}

// ═══════════════════════════════════════
// HELP MENU
// ═══════════════════════════════════════
void print_help() {
    printf("\n========= HEXAPOD CONTROL =========\n");
    printf("Commands:\n");
    printf("  scan         - scan all I2C devices\n");
    printf("  adc          - read all ADC channels\n");
    printf("  regs         - dump PCA9685 registers (debug)\n");
    printf("  s1 <ch> <ang> - set servo PCA1 channel angle\n");
    printf("  s2 <ch> <ang> - set servo PCA2 channel angle\n");
    printf("  center       - center all servos to 70 degrees\n");
    printf("  sweep <ch>   - sweep PCA1 servo channel\n");
    printf("  cal          - list all calibration\n");
    printf("  cal <ch> <off> - set center offset (-90..+90) for ch\n");
    printf("  cal <ch> min <n> - set min limit (0-%d)\n", MAX_EFF);
    printf("  cal <ch> max <n> - set max limit (0-%d)\n", MAX_EFF);
    printf("  cal <ch> clr - reset channel to default\n");
    printf("  mirror       - show mirror state\n");
    printf("  mirror 1|2|all|off - flip channel order (upside-down board)\n");
    printf("  map          - show leg-to-channel map\n");
    printf("  home         - all legs to HOME (center): coxa 90, femur 70, tibia 110\n");
    printf("  pose         - show poses / leg map\n");
    printf("  pose <name>  - set ALL legs (home|stand|sit|lift|crouch)\n");
    printf("  leg <n> <name> - set one leg 1-6 (home|stand|sit|lift|crouch)\n");
    printf("  help         - show this menu\n");
    printf("===================================\n");
}

// ═══════════════════════════════════════
// COMMAND PARSER
// ═══════════════════════════════════════
void parse_command(char* cmd) {
    // scan
    if (strcmp(cmd, "scan") == 0) {
        scan_all();
    }
    // adc
    else if (strcmp(cmd, "adc") == 0) {
        read_all_adc();
    }
    // center
    else if (strcmp(cmd, "center") == 0) {
        printf("Centering all servos to 90 degrees...\n");
        for (int ch = 0; ch < 16; ch++) {
            set_servo_angle(PCA1_ADDR, ch, 90);
            set_servo_angle(PCA2_ADDR, ch, 90);
        }
        printf("Done!\n");
    }
    // s1 <ch> <angle>
    else if (strncmp(cmd, "s1 ", 3) == 0) {
        int ch, angle;
        if (sscanf(cmd + 3, "%d %d", &ch, &angle) == 2) {
            set_servo_angle(PCA1_ADDR, ch, (float)angle);
            printf("PCA1 CH%d -> %d degrees\n", ch, angle);
        } else {
            printf("Usage: s1 <channel> <angle>\n");
        }
    }
    // s2 <ch> <angle>
    else if (strncmp(cmd, "s2 ", 3) == 0) {
        int ch, angle;
        if (sscanf(cmd + 3, "%d %d", &ch, &angle) == 2) {
            set_servo_angle(PCA2_ADDR, ch, (float)angle);
            printf("PCA2 CH%d -> %d degrees\n", ch, angle);
        } else {
            printf("Usage: s2 <channel> <angle>\n");
        }
    }
    // sweep
    else if (strncmp(cmd, "sweep ", 6) == 0) {
        int ch = atoi(cmd + 6);
        printf("Sweeping PCA1 CH%d...\n", ch);
        for (int a = 0; a <= 140; a += 10) {
            set_servo_angle(PCA1_ADDR, ch, (float)a);
            printf("  -> %d degrees\n", a);
            sleep_ms(300);
        }
        for (int a = 140; a >= 0; a -= 10) {
            set_servo_angle(PCA1_ADDR, ch, (float)a);
            printf("  -> %d degrees\n", a);
            sleep_ms(300);
        }
        printf("Sweep done!\n");
    }
    // regs
    else if (strcmp(cmd, "regs") == 0) {
        dump_regs();
    }
    // P — ping: re-print status + PONG (GUI handshake)
    else if (strcmp(cmd, "P") == 0) {
        print_status();
        printf("OK PONG\n");
    }
    // C<ch> <angle> — GUI servo command (ch 0-31, angle 0-180)
    else if (cmd[0] == 'C' && cmd[1] >= '0' && cmd[1] <= '9') {
        int ch, angle;
        if (sscanf(cmd + 1, "%d %d", &ch, &angle) == 2) {
            set_servo_gui(ch, angle);
            servo_target[ch] = angle;  // keep movement engine in sync
            printf("OK C%d %d\n", ch, angle);
        } else {
            printf("ERR format\n");
        }
    }
    // H — home all servos to 90
    else if (strcmp(cmd, "H") == 0) {
        for (int ch = 0; ch < 32; ch++) {
            set_servo_gui(ch, 90);
            servo_target[ch] = 90;
        }
        printf("OK HOME\n");
    }
    // A <angle> — set all servos
    else if (strncmp(cmd, "A ", 2) == 0) {
        int angle;
        if (sscanf(cmd + 2, "%d", &angle) == 1) {
            for (int ch = 0; ch < 32; ch++) {
                set_servo_gui(ch, angle);
                servo_target[ch] = angle;
            }
            printf("OK A %d\n", angle);
        } else {
            printf("ERR format\n");
        }
    }
    // R — report positions
    else if (strcmp(cmd, "R") == 0) {
        printf("POS");
        for (int ch = 0; ch < 32; ch++) printf(" %d", servo_pos[ch]);
        printf("\n");
    }
    // D — quick servo diagnostic (bus presence check, neutral 90° write)
    else if (strcmp(cmd, "D") == 0) {
        printf("=== SERVO DIAGNOSTIC ===\n");
        for (int ch = 0; ch < 32; ch++) {
            uint8_t pca = (ch < 16) ? PCA1_ADDR : PCA2_ADDR;
            uint8_t pch = (uint8_t)(ch % 16);
            int mirror = (pca == PCA1_ADDR) ? mirror_pca1 : mirror_pca2;
            if (mirror) pch = (uint8_t)(15 - pch);  // same mapping as C/H/A
            uint8_t base = 0x06 + pch * 4;
            uint16_t pulse = 307;  // 90° neutral
            uint8_t buf[5] = {base, 0x00, 0x00,
                              (uint8_t)(pulse & 0xFF), (uint8_t)(pulse >> 8)};
            int ok = i2c_write_blocking(i2c0, pca, buf, 5, false) >= 0;
            printf("Servo %d (PCA%s Ch%d): %s\n", ch,
                   (ch < 16) ? "1" : "2", pch, ok ? "OK" : "FAIL");
        }
        printf("DIAGNOSTIC COMPLETE\n");
    }
    // X — read all ADS1115 across all TCA channels
    else if (strcmp(cmd, "X") == 0) {
        for (int ch = 0; ch < 5; ch++) {
            tca_select(ch);
            sleep_ms(10);
            for (int ain = 0; ain < 4; ain++) {
                int16_t val = ads_read(ain);
                float mv = val * 2048.0f / 32768.0f * 1000.0f;  // ±2.048V PGA
                printf("ADS_VAL %d %d %d %.2f\n", ch, ain, val, mv);
            }
            tca_close();
            sleep_ms(10);
        }
        printf("ADS_SCAN_DONE\n");
    }
    // X <ch> — read one TCA channel's ADS1115
    else if (strncmp(cmd, "X ", 2) == 0) {
        int ch = atoi(cmd + 2);
        tca_select(ch);
        sleep_ms(10);
        for (int ain = 0; ain < 4; ain++) {
            int16_t val = ads_read(ain);
            float mv = val * 4096.0f / 32768.0f * 1000.0f;  // ±4.096V PGA
            printf("ADS_VAL 0 %d %d %.2f\n", ain, val, mv);
        }
        tca_close();
        printf("ADS_DONE\n");
    }
    // F — fast ADC feedback scan (20 pots, one compact line)
    else if (strcmp(cmd, "F") == 0) {
        scan_adc_fast();
    }
    // cal — servo calibration
    else if (strcmp(cmd, "cal") == 0) {
        printf("\nCH  OFFSET  MIN  MAX\n");
        for (int ch = 0; ch < 32; ch++) {
            printf("%2d   %+3d   %3d  %3d\n", ch, cal_offset[ch], cal_min[ch], cal_max[ch]);
        }
    }
    else if (strncmp(cmd, "cal ", 4) == 0) {
        int ch, val;
        char sub[8];
        if (sscanf(cmd + 4, "%d %7s", &ch, &sub) == 2) {
            if (ch < 0 || ch > 31) {
                printf("Ch out of range (0-31)\n");
            } else if (strcmp(sub, "clr") == 0) {
                cal_offset[ch] = 0; cal_min[ch] = 0; cal_max[ch] = 180;
                printf("Ch%d reset: off=0 min=0 max=180\n", ch);
            } else if (strcmp(sub, "min") == 0) {
                if (sscanf(cmd + 4, "%d min %d", &ch, &val) == 2 && val >= 0 && val <= MAX_EFF) {
                    if (val > cal_max[ch]) {
                        printf("ERR: min %d > current max %d — set max first\n", val, cal_max[ch]);
                    } else {
                        cal_min[ch] = val;
                        printf("Ch%d min=%d\n", ch, val);
                    }
                } else printf("Usage: cal <ch> min <0-180>\n");
            } else if (strcmp(sub, "max") == 0) {
                if (sscanf(cmd + 4, "%d max %d", &ch, &val) == 2 && val >= 0 && val <= MAX_EFF) {
                    if (val < cal_min[ch]) {
                        printf("ERR: max %d < current min %d — set min first\n", val, cal_min[ch]);
                    } else {
                        cal_max[ch] = val;
                        printf("Ch%d max=%d\n", ch, val);
                    }
                } else printf("Usage: cal <ch> max <0-180>\n");
            } else {
                // numeric offset — validate it's actually a number
                char* endp;
                val = (int)strtol(sub, &endp, 10);
                if (*endp == '\0' && val >= -90 && val <= 90) {
                    cal_offset[ch] = val;
                    printf("Ch%d offset=%d\n", ch, val);
                } else printf("Offset must be a number -90..+90\n");
            }
        } else {
            printf("Usage: cal <ch> <offset> | cal <ch> min/max <n> | cal <ch> clr\n");
        }
    }
    // mirror — flip channel order for upside-down soldered boards
    else if (strcmp(cmd, "mirror") == 0) {
        printf("Mirror: PCA1=%d PCA2=%d  (1 = flipped/upside-down board)\n",
               mirror_pca1, mirror_pca2);
    }
    else if (strncmp(cmd, "mirror ", 7) == 0) {
        const char* a = cmd + 7;
        if (strcmp(a, "1") == 0) {
            mirror_pca1 = 1;
            printf("PCA1 (0x40) mirrored: ch -> 15-ch\n");
        } else if (strcmp(a, "2") == 0) {
            mirror_pca2 = 1;
            printf("PCA2 (0x41) mirrored: ch -> 15-ch\n");
        } else if (strcmp(a, "all") == 0) {
            mirror_pca1 = 1; mirror_pca2 = 1;
            printf("Both PCAs mirrored\n");
        } else if (strcmp(a, "off") == 0) {
            mirror_pca1 = 0; mirror_pca2 = 0;
            printf("Mirroring disabled\n");
        } else {
            printf("Usage: mirror 1 | 2 | all | off\n");
        }
    }
    // home — all legs to HOME pose (the center reference)
    else if (strcmp(cmd, "home") == 0) {
        if (!mirror_pca1 || !mirror_pca2) {
            printf("⚠ WARNING: leg map assumes mirror all — type 'mirror all' first!\n");
        }
        for (int leg = 0; leg < 6; leg++) set_leg_pose(leg, 0);
        printf("All legs -> HOME (coxa 90, femur 70, tibia 110)\n");
    }
    // map — show leg-to-channel map
    else if (strcmp(cmd, "map") == 0) {
        print_leg_map();
    }
    // pose — show poses + map
    else if (strcmp(cmd, "pose") == 0) {
        print_leg_map();
    }
    // pose <name> — set ALL legs to a pose
    else if (strncmp(cmd, "pose ", 5) == 0) {
        char pname[12];
        if (sscanf(cmd + 5, "%11s", pname) == 1) {
            int pidx = pose_lookup(pname);
            if (pidx < 0) {
                printf("Unknown pose. Use: home, stand, sit, lift, crouch\n");
            } else {
                if (!mirror_pca1 || !mirror_pca2) {
                    printf("⚠ WARNING: leg map assumes mirror all — type 'mirror all' first!\n");
                }
                for (int leg = 0; leg < 6; leg++) set_leg_pose(leg, pidx);
            }
        } else {
            printf("Usage: pose <home|stand|sit|lift|crouch>\n");
        }
    }
    // leg <n> <pose> — set ONE leg (1-6)
    else if (strncmp(cmd, "leg ", 4) == 0) {
        int leg;
        char pname[12];
        if (sscanf(cmd + 4, "%d %11s", &leg, pname) == 2 && leg >= 1 && leg <= 6) {
            int pidx = pose_lookup(pname);
            if (pidx < 0) {
                printf("Unknown pose. Use: home, stand, sit, lift, crouch\n");
            } else {
                if (!mirror_pca1 || !mirror_pca2) {
                    printf("⚠ WARNING: leg map assumes mirror all — type 'mirror all' first!\n");
                }
                set_leg_pose(leg - 1, pidx);
            }
        } else {
            printf("Usage: leg <1-6> <home|stand|sit|lift|crouch>\n");
        }
    }
    // help
    else if (strcmp(cmd, "help") == 0) {
        print_help();
    }
    else {
        printf("Unknown command. Type 'help'\n");
    }
}

// ═══════════════════════════════════════
// MAIN
// ═══════════════════════════════════════
int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(100);
    sleep_ms(1000);

    // Init I2C
    i2c_init(i2c0, 100000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Init position tracking + calibration
    for (int i = 0; i < 32; i++) {
        servo_pos[i] = 90;
        servo_target[i] = 90;
    }
    cal_init();

    // Detect + init present PCA boards
    uint8_t probe;
    uint8_t pca_found[2];
    int pca_n = 0;
    if (i2c_read_blocking(i2c0, PCA1_ADDR, &probe, 1, false) >= 0) pca_found[pca_n++] = PCA1_ADDR;
    if (i2c_read_blocking(i2c0, PCA2_ADDR, &probe, 1, false) >= 0) pca_found[pca_n++] = PCA2_ADDR;
    for (int i = 0; i < pca_n; i++) pca_init(pca_found[i]);

    // GUI handshake — control app waits for READY / PONG
    print_status();
    printf("READY\n");

    printf("=== Hexapod Control System ===\n");
    printf("Type 'help' for commands\n\n");

    // Command buffer
    char buf[64];
    int idx = 0;

    while (1) {
        int c = getchar_timeout_us(20000);   // 20ms tick — movement engine
        if (c == PICO_ERROR_TIMEOUT) {
            movement_update();
            continue;
        }

        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                buf[idx] = '\0';
                printf("\n> %s\n", buf);
                parse_command(buf);
                idx = 0;
            }
        } else if (c == 8 || c == 127) {  // backspace
            if (idx > 0) idx--;
        } else if (idx < 63) {
            buf[idx++] = (char)c;
            putchar(c);  // echo back
        }
    }
}