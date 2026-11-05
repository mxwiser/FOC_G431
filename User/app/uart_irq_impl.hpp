#pragma once
// Private implementation, included only by foc/motor_control_impl.hpp.

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2 || huart->hdmarx) return;
    const uint8_t next = static_cast<uint8_t>(uart_rx_write + 1u);
    if (next == uart_rx_read) uart_rx_overflow = true;
    else { uart_rx_data[uart_rx_write] = uart_rx_byte; uart_rx_write = next; }
    if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1) != HAL_OK) uart_rx_error = true;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t)
{
    if (huart != &huart2 || !huart->hdmarx) return;
    // Use the current hardware position rather than the event's Size. A
    // delayed half/full interrupt may be delivered after a newer idle event.
    // Both IRQs must run within half a DMA buffer time: NDTR cannot count
    // complete unseen laps if interrupts are blocked for an entire buffer.
    const uint16_t remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (remaining > sizeof(uart_rx_dma_data)) { uart_rx_error = true; return; }
    const uint16_t end = (sizeof(uart_rx_dma_data) - remaining) % sizeof(uart_rx_dma_data);
    while (uart_rx_dma_read != end) {
        const uint8_t next = static_cast<uint8_t>(uart_rx_write + 1u);
        if (next == uart_rx_read) { uart_rx_overflow = true; break; }
        uart_rx_data[uart_rx_write] = uart_rx_dma_data[uart_rx_dma_read];
        uart_rx_write = next;
        uart_rx_dma_read = (uart_rx_dma_read + 1u) % sizeof(uart_rx_dma_data);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) uart_rx_error = true;
}
