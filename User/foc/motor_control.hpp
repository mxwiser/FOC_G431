#pragma once
namespace motor_control {
// Startup and foreground service. This function owns the main loop.
void run();
// Set one coherent input frame in SI units, then call adc_complete() once.
// Currents are calibrated amperes; Ic is reconstructed as -Ia-Ib.
void set_phase_currents(float ia, float ib);
void set_phase_voltages(float vu, float vv, float vw);
void set_bus_voltage(float volts);
// Optional electrical angle in radians. Sensorless modes retain their observer.
void set_encoder_angle(float electrical_radians);
void adc_complete();
void adc_error();
// Calibration and ADC rail checks belong to the board adapter, not this module.
enum InputError { BAD_CURRENT_ZERO, CURRENT_ADC_SATURATED };
bool calibrating();
void calibration_complete();
void input_error(InputError error);
}
