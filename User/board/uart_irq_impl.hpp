#pragma once
// STM32 callbacks. Forward only bytes/errors to the portable app/data_port.

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2 || huart->hdmarx) return;
    data_port::receive_byte(uart_rx_byte);
    if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1) != HAL_OK) data_port::receive_error();
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t)
{
    if (huart != &huart2 || !huart->hdmarx) return;
    // Use the current hardware position rather than the event's Size. A
    // delayed half/full interrupt may be delivered after a newer idle event.
    // Both IRQs must run within half a DMA buffer time: NDTR cannot count
    // complete unseen laps if interrupts are blocked for an entire buffer.
    const uint16_t remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (remaining > sizeof(uart_rx_dma_data)) { data_port::receive_error(); return; }
    const uint16_t end = (sizeof(uart_rx_dma_data) - remaining) % sizeof(uart_rx_dma_data);
    while (uart_rx_dma_read != end) {
        data_port::receive_byte(uart_rx_dma_data[uart_rx_dma_read]);
        if (data_port::receive_state().overflow) break;
        uart_rx_dma_read = (uart_rx_dma_read + 1u) % sizeof(uart_rx_dma_data);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) data_port::receive_error();
}
