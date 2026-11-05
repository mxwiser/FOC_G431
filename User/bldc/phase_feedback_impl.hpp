#pragma once
// Private implementation, included only by foc/motor_control_impl.hpp.

extern "C" void bldc_publish_phase_voltages(float a, float b, float c)
{
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    bldc_feedback_published = foc_finite(a) && foc_finite(b) && foc_finite(c) &&
        a >= -0.5f && b >= -0.5f && c >= -0.5f &&
        a <= Udc_quant + 0.5f && b <= Udc_quant + 0.5f && c <= Udc_quant + 0.5f;
    if (!bldc_feedback_published && !bldc_feedback_fault_capture.frame &&
        motor_mode == MOTOR_BLDC && motor_fault == NO_FAULT &&
        (motor_state == BOOTSTRAP || motor_state == OPEN_LOOP || motor_state == RUN)) {
        bldc_feedback_fault_capture.phase[0] = a;
        bldc_feedback_fault_capture.phase[1] = b;
        bldc_feedback_fault_capture.phase[2] = c;
        bldc_feedback_fault_capture.bus = Udc_quant;
        bldc_feedback_fault_capture.frame = sample_frames;
    }
    bldc_phase_voltage[0] = a; bldc_phase_voltage[1] = b; bldc_phase_voltage[2] = c;
    bldc_feedback_frame = sample_frames;
    __DMB();
    __set_PRIMASK(mask);
}
