#pragma once
#include <stdint.h>

enum SerialCommandType {
    SERIAL_NONE, SERIAL_START, SERIAL_STOP, SERIAL_DETECT,
    SERIAL_SET_IQ, SERIAL_SET_SPEED, SERIAL_SET_SPEED_LIMIT, SERIAL_INVALID
};
struct SerialCommand { SerialCommandType type; float value; };
struct SerialCommandParser { char text[24]; uint8_t length; bool discarding; };

// R/D/S are single characters; IQ=<A>, V=<mechanical rpm>, ILIM=<A> use CR/LF.
// S also stops inside an incomplete/overlong command.
inline bool serial_command_feed(SerialCommandParser &parser, uint8_t byte,
                                 SerialCommand &command)
{
    command.type = SERIAL_NONE;
    command.value = 0.0f;
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
        } else return true;
        while (index < length && (parser.text[index] == ' ' || parser.text[index] == '\t')) ++index;
        bool negative = false;
        if (index < length && (parser.text[index] == '-' || parser.text[index] == '+')) {
            negative = parser.text[index] == '-'; ++index;
        }
        bool digit = false, point = false;
        float value = 0.0f, fraction = 0.1f;
        for (; index < length; ++index) {
            const char c = parser.text[index];
            if (c >= '0' && c <= '9') {
                digit = true;
                if (point) { value += (c - '0') * fraction; fraction *= 0.1f; }
                else value = value * 10.0f + (c - '0');
            } else if (c == '.' && !point) point = true;
            else if (c == ' ' || c == '\t') {
                while (index < length && (parser.text[index] == ' ' || parser.text[index] == '\t')) ++index;
                if (index != length) return true;
                break;
            } else return true;
        }
        if (!digit) return true;
        command.type = type;
        command.value = negative ? -value : value;
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
