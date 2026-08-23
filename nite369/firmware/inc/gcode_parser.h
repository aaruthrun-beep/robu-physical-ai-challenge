#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Minimal Marlin-style G-code parser for the Tesla master.
 *
 * Ported from Marlin 2.1.2.8 (Marlin/src/gcode/parser.h) — the essentials
 * only, no HAL dependencies:
 *   - gcode_parse(line)  tokenize a line like "G1 X10 Y-2.5 F500"
 *   - gcode_seen('X')    true if the letter appears in the line
 *   - gcode_intval/floatval  value of a letter (with default)
 *
 * Command letter + number: gcode_command() returns 'G'/'M'/'T' and
 * gcode_codenum() the number (e.g. 1 for G1, 92 for M92).
 */

// Max line length we accept (same ballpark as Marlin's MAX_CMD_SIZE).
#define GCODE_MAX_LINE 96

// Initialize/reset parser state.
void gcode_reset(void);

// Parse a NUL-terminated G-code line into the parser state.
// Returns true if a valid command letter was found.
bool gcode_parse(char *line);

// True if the given parameter letter appeared in the line.
bool gcode_seen(char letter);

// True if the letter appeared AND has a value after it.
bool gcode_seenval(char letter);

// Value of a parameter (default if absent or valueless).
float gcode_floatval(char letter, float dval);
int32_t gcode_intval(char letter, int32_t dval);

// Command letter ('G'/'M'/'T') and numeric code (1, 92, ...).
char gcode_command(void);
int32_t gcode_codenum(void);

#endif
