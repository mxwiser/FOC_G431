#pragma once
#include <stdint.h>

enum SerialCommandType {
    SERIAL_NONE, SERIAL_START, SERIAL_STOP, SERIAL_DETECT,
    SERIAL_SET_IQ, SERIAL_SET_SPEED, SERIAL_SET_SPEED_LIMIT, SERIAL_SET_PHASES,
    SERIAL_SET_MOTOR_MODE, SERIAL_MUSIC_NOTE, SERIAL_MUSIC_LIVE, SERIAL_MUSIC_STATUS, SERIAL_INVALID
};
struct SerialCommand { SerialCommandType type; float value, amplitude, duration_ms; };
struct SerialCommandParser { char text[64]; uint8_t length; bool discarding; };

// One decimal field, with the same whitespace/sign rules as the IQ protocol.
inline bool serial_command_number(const char *text, uint8_t index, uint8_t end, float &result)
{
    while (index < end && (text[index] == ' ' || text[index] == '\t')) ++index;
    bool negative = false;
    if (index < end && (text[index] == '-' || text[index] == '+')) {
        negative = text[index] == '-'; ++index;
    }
    bool digit = false, point = false;
    float value = 0.0f, fraction = 0.1f;
    for (; index < end; ++index) {
        const char c = text[index];
        if (c >= '0' && c <= '9') {
            digit = true;
            if (point) { value += (c - '0') * fraction; fraction *= 0.1f; }
            else value = value * 10.0f + (c - '0');
        } else if (c == '.' && !point) point = true;
        else if (c == ' ' || c == '\t') {
            while (index < end && (text[index] == ' ' || text[index] == '\t')) ++index;
            if (index != end) return false;
            break;
        } else return false;
    }
    if (!digit) return false;
    result = negative ? -value : value;
    return true;
}

// R/D/S are single characters; IQ=<A>, V=<rpm>, ILIM=<A>, M=<0|1|2> use CR/LF.
// N=<Hz>,<A_peak>,<ms> queues a music note; N? queries music capability/ACK.
// S also stops inside an incomplete/overlong command.
inline bool serial_command_feed(SerialCommandParser &parser, uint8_t byte,
                                 SerialCommand &command)
{
    command.type = SERIAL_NONE;
    command.value = command.amplitude = command.duration_ms = 0.0f;
    if (byte == 'S' || byte == 's') {
        parser.length = 0; parser.discarding = false;
        command.type = SERIAL_STOP; return true;
    }
    if (byte == '\r' || byte == '\n') {
        const uint8_t length = parser.length;
        parser.length = 0;
        if (parser.discarding) {
            parser.discarding = false; command.type = SERIAL_INVALID; return true;
        }
        if (!length) return false;
        command.type = SERIAL_INVALID;
        if (length == 2 && (parser.text[0] == 'N' || parser.text[0] == 'n') && parser.text[1] == '?') {
            command.type = SERIAL_MUSIC_STATUS; return true;
        }
        if (length >= 3 && (parser.text[0] == 'N' || parser.text[0] == 'n') && parser.text[1] == '=') {
            uint8_t first = 2u, second;
            while (first < length && parser.text[first] != ',') ++first;
            second = first + 1u;
            while (second < length && parser.text[second] != ',') ++second;
            if (first == length || second == length ||
                !serial_command_number(parser.text, 2u, first, command.value) ||
                !serial_command_number(parser.text, first + 1u, second, command.amplitude) ||
                !serial_command_number(parser.text, second + 1u, length, command.duration_ms)) return true;
            command.type = SERIAL_MUSIC_NOTE; return true;
        }
        if (length >= 3 && (parser.text[0] == 'T' || parser.text[0] == 't') && parser.text[1] == '=') {
            uint8_t comma = 2u;
            while (comma < length && parser.text[comma] != ',') ++comma;
            if (comma == length ||
                !serial_command_number(parser.text, 2u, comma, command.value) ||
                !serial_command_number(parser.text, comma + 1u, length, command.amplitude)) return true;
            command.type = SERIAL_MUSIC_LIVE; return true;
        }
        uint8_t index;
        SerialCommandType type;
        if (length >= 4 && (parser.text[0]=='I' || parser.text[0]=='i') &&
            (parser.text[1]=='Q' || parser.text[1]=='q') && parser.text[2]=='=') {
            index = 3; type = SERIAL_SET_IQ;
        } else if (length >= 3 && (parser.text[0]=='V' || parser.text[0]=='v') && parser.text[1]=='=') {
            index = 2; type = SERIAL_SET_SPEED;
        } else if (length >= 6 && (parser.text[0]=='I' || parser.text[0]=='i') &&
            (parser.text[1]=='L' || parser.text[1]=='l') && (parser.text[2]=='I' || parser.text[2]=='i') &&
            (parser.text[3]=='M' || parser.text[3]=='m') && parser.text[4]=='=') {
            index = 5; type = SERIAL_SET_SPEED_LIMIT;
        } else if (length >= 3 && (parser.text[0]=='M' || parser.text[0]=='m') && parser.text[1]=='=') {
            index = 2; type = SERIAL_SET_MOTOR_MODE;
        } else if (length >= 3 && (parser.text[0]=='P' || parser.text[0]=='p') &&
            (parser.text[1]=='H' || parser.text[1]=='h') && parser.text[2]=='=') {
            index = 3; type = SERIAL_SET_PHASES;
            // Invalid phase values leave the current selection and motor ACK intact.
            command.type = type; command.value = -1.0f;
        } else return true;
        if (!serial_command_number(parser.text, index, length, command.value)) return true;
        command.type = type;
        return true;
    }
    if (parser.discarding) return false;
    if (parser.length == 0) {
        if (byte == ' ' || byte == '\t') return false;
        if (byte == 'R' || byte == 'r') { command.type = SERIAL_START; return true; }
        if (byte == 'D' || byte == 'd') { command.type = SERIAL_DETECT; return true; }
    }
    if (parser.length >= sizeof(parser.text)) { parser.discarding = true; return false; }
    parser.text[parser.length++] = static_cast<char>(byte);
    return false;
}
