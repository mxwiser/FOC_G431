#include "data_port.hpp"

namespace data_port {
namespace {
Driver driver = {};
ReceiveState rx = {};
}

void bind(const Driver &value) { driver = value; }
ReceiveState &receive_state() { return rx; }

void receive_byte(uint8_t byte)
{
    const uint8_t write = rx.write;
    const uint8_t next = static_cast<uint8_t>(write + 1u);
    if (next == rx.read) { rx.overflow = true; return; }
    rx.data[write] = byte;
    memory_barrier();
    rx.write = next;
}

void receive(const uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; ++i) receive_byte(data[i]);
}

void receive_error() { rx.error = true; }
bool tx_ready() { return driver.tx_ready && driver.tx_ready(); }
bool write(const uint8_t *data, uint16_t size)
{
    return data && size && driver.write && tx_ready() && driver.write(data, size);
}
bool recover_receive() { return driver.recover_receive && driver.recover_receive(); }
uint32_t enter_critical() { return driver.enter_critical ? driver.enter_critical() : 0u; }
void leave_critical(uint32_t token) { if (driver.leave_critical) driver.leave_critical(token); }
void memory_barrier() { if (driver.memory_barrier) driver.memory_barrier(); }
uint32_t milliseconds() { return driver.milliseconds ? driver.milliseconds() : 0u; }
}
