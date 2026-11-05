#pragma once
#include <stdint.h>

// Hardware-independent transport for the existing OBS/CUR and command protocol.
// Callbacks must keep transmitted storage alive until tx_ready() becomes true.
namespace data_port {
struct Driver {
    bool (*tx_ready)();
    bool (*write)(const uint8_t *data, uint16_t size);
    bool (*recover_receive)();
    uint32_t (*enter_critical)();
    void (*leave_critical)(uint32_t token);
    void (*memory_barrier)();
    uint32_t (*milliseconds)();
};
struct ReceiveState {
    volatile uint8_t data[256];
    volatile uint8_t write, read;
    volatile bool overflow, error;
};
void bind(const Driver &driver);
ReceiveState &receive_state();
void receive_byte(uint8_t byte);
void receive(const uint8_t *data, uint16_t size);
void receive_error();
bool tx_ready();
bool write(const uint8_t *data, uint16_t size);
bool recover_receive();
uint32_t enter_critical();
void leave_critical(uint32_t token);
void memory_barrier();
uint32_t milliseconds();
}
