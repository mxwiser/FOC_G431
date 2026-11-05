#pragma once
// Private implementation, included only by foc/motor_control_impl.hpp.

enum CommandResult { COMMAND_NONE, COMMAND_OK, COMMAND_FORMAT, COMMAND_RANGE, COMMAND_BUSY, COMMAND_RX,
    COMMAND_FEEDBACK, COMMAND_UNSUPPORTED };
CommandResult command_result = COMMAND_NONE;
SerialCommandParser command_parser = {};
// Telemetry selection only; ADC sampling remains active for FOC and protection.
volatile uint8_t phase_telemetry_mask = 0u;
// ADC is the sole producer; foreground is the sole consumer. The producer
// never overwrites unread slots, including while DMA is transmitting.
struct PhaseSample { uint32_t frame; int16_t a, b, c; };
PhaseSample phase_samples[128];
volatile uint8_t phase_write = 0u, phase_read = 0u;
volatile uint32_t phase_dropped = 0u;
const uint32_t kPhaseStride = 12u;
