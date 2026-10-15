/*
 * Log lines go to Serial and to a small RAM ring buffer, which the /debug
 * page serves - so a board that's only reachable over Wi-Fi can still be
 * diagnosed after the fact. Safe to call from any task.
 */
#pragma once

#include <Arduino.h>

// printf-style; a newline is appended.
void debug_log(const char *format, ...) __attribute__((format(printf, 1, 2)));

// Appends the buffered lines, oldest first.
void debug_log_dump(String &out);
