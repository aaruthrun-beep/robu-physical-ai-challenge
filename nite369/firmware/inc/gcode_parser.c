/**
 * gcode_parser.c — minimal Marlin-style G-code parser.
 *
 * Ported from Marlin 2.1.2.8 gcode/parser.h. Keeps a small static map of
 * parameter letter -> value-pointer, plus the command letter/code.
 */

#include "gcode_parser.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// Parameter offsets per letter (A..Z). -1 = not seen.
static int8_t param_off[26];
static char cmd_letter;
static int32_t cmd_code;
static char *line_buf;

static int letter_index(char c) {
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    if (c < 'A' || c > 'Z') return -1;
    return c - 'A';
}

void gcode_reset(void) {
    memset(param_off, -1, sizeof(param_off));
    cmd_letter = '?';
    cmd_code = 0;
}

static bool is_numeric(char c) {
    return (c >= '0' && c <= '9') || c == '-' || c == '+';
}

bool gcode_parse(char *line) {
    gcode_reset();
    line_buf = line;

    char *p = line;
    // Skip leading spaces
    while (*p == ' ') p++;
    // Optional N<line-number>
    if ((*p == 'N' || *p == 'n') && is_numeric(p[1])) {
        p += 2;
        while (isdigit((unsigned char)*p)) p++;
        while (*p == ' ') p++;
    }

    // Command letter
    char letter = *p;
    if (letter >= 'a' && letter <= 'z') letter = (char)(letter - 'a' + 'A');
    if (letter != 'G' && letter != 'M' && letter != 'T') return false;
    cmd_letter = letter;
    p++;

    // Command number
    cmd_code = 0;
    while (isdigit((unsigned char)*p)) {
        cmd_code = cmd_code * 10 + (*p - '0');
        p++;
    }

    // Parse parameters: <LETTER><optional value> tokens
    while (*p) {
        if (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
            p++;
            continue;
        }
        if (*p == ';') break;  // comment to end of line
        int idx = letter_index(*p);
        if (idx < 0) { p++; continue; }  // skip unknown
        p++;
        // Record offset of the value (even if empty: offset at terminator)
        param_off[idx] = (int8_t)(p - line);
        // Skip the value
        if (is_numeric(*p)) {
            p++;
            while (is_numeric(*p) || *p == '.') p++;
        }
    }
    return true;
}

bool gcode_seen(char letter) {
    int idx = letter_index(letter);
    return idx >= 0 && param_off[idx] >= 0;
}

bool gcode_seenval(char letter) {
    int idx = letter_index(letter);
    if (idx < 0 || param_off[idx] < 0) return false;
    const char *v = line_buf + param_off[idx];
    return is_numeric(*v) || *v == '.';
}

float gcode_floatval(char letter, float dval) {
    int idx = letter_index(letter);
    if (idx < 0 || param_off[idx] < 0) return dval;
    const char *v = line_buf + param_off[idx];
    if (!(is_numeric(*v) || *v == '.')) return dval;
    return (float)strtod(v, NULL);
}

int32_t gcode_intval(char letter, int32_t dval) {
    int idx = letter_index(letter);
    if (idx < 0 || param_off[idx] < 0) return dval;
    const char *v = line_buf + param_off[idx];
    if (!(is_numeric(*v) || *v == '.')) return dval;
    return (int32_t)strtol(v, NULL, 10);
}

char gcode_command(void) { return cmd_letter; }
int32_t gcode_codenum(void) { return cmd_code; }
