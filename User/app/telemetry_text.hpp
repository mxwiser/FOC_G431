#pragma once
#include <stdint.h>
#include <stdio.h>

// Fixed storage and float32 arithmetic for the common telemetry range. Passing
// every float to printf promotes it to software double on Cortex-M4 and leaves
// too little foreground time beside the 25 kHz control interrupt.
struct TelemetryText {
    char data[448];
    unsigned length;
    bool valid;

    void reset(const char *prefix) { length = 0u; valid = true; append(prefix); }
    void character(char value) {
        if (length < sizeof(data)) data[length++] = value;
        else valid = false;
    }
    void append(const char *value) { while (*value) character(*value++); }
    void number(uint32_t value) {
        char digits[10]; unsigned count = 0u;
        do { digits[count++] = static_cast<char>('0' + value % 10u); value /= 10u; } while (value);
        while (count) character(digits[--count]);
    }
    void signed_number(int32_t value) {
        if (value < 0) { character('-'); number(0u - static_cast<uint32_t>(value)); }
        else number(static_cast<uint32_t>(value));
    }
    void key(const char *name) { character(','); append(name); character('='); }
    void integer(const char *name, uint32_t value) { key(name); number(value); }
    void letter(const char *name, char value) { key(name); character(value); }

    void fixed(float value, unsigned decimals) {
        static const uint32_t scale[] = {1u, 10u, 100u, 1000u, 10000u, 100000u,
            1000000u, 10000000u, 100000000u, 1000000000u};
        if (decimals >= sizeof(scale) / sizeof(scale[0])) { valid = false; return; }
        const bool negative = value < 0.0f;
        const float magnitude = negative ? -value : value;
        const float scaled = magnitude * static_cast<float>(scale[decimals]) + 0.5f;
        // Rare very large values retain printf's original fixed notation.
        // The normal current, speed and motor-parameter ranges never use it.
        if (!(scaled < 4294967040.0f)) {
            char text[64];
            const int count = snprintf(text, sizeof(text), "%.*f", static_cast<int>(decimals),
                                       static_cast<double>(value));
            if (count <= 0 || count >= static_cast<int>(sizeof(text))) { valid = false; return; }
            append(text); return;
        }
        const uint32_t rounded = static_cast<uint32_t>(scaled);
        if (negative) character('-');
        number(rounded / scale[decimals]);
        if (!decimals) return;
        character('.');
        uint32_t remainder = rounded % scale[decimals];
        for (uint32_t divisor = scale[decimals] / 10u; divisor; divisor /= 10u) {
            character(static_cast<char>('0' + remainder / divisor));
            remainder %= divisor;
        }
    }
    void real(const char *name, float value, unsigned decimals) { key(name); fixed(value, decimals); }
    void significant(const char *name, float value) {
        key(name);
        const float magnitude = value < 0.0f ? -value : value;
        if (magnitude == 0.0f) { character('0'); return; }
        // Match %.7g for normal command values without software double.
        if (magnitude >= 0.0001f && magnitude < 9999999.0f) {
            unsigned decimals = 6u;
            float digits = magnitude;
            while (digits >= 10.0f) { digits *= 0.1f; --decimals; }
            while (digits < 1.0f) { digits *= 10.0f; ++decimals; }
            // Seven significant digits can require ten places near 1e-4.
            if (decimals <= 9u) {
                const unsigned start = length;
                fixed(value, decimals);
                if (decimals) {
                    while (length > start && data[length - 1u] == '0') --length;
                    if (length > start && data[length - 1u] == '.') --length;
                }
                return;
            }
        }
        // Preserve the existing full float command range and scientific syntax.
        char text[24];
        const int count = snprintf(text, sizeof(text), "%.7g", static_cast<double>(value));
        if (count <= 0 || count >= static_cast<int>(sizeof(text))) { valid = false; return; }
        append(text);
    }
    void finish() { character('\r'); character('\n'); }
};
