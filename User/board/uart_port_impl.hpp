#pragma once
// STM32 UART2 transport backend. OBS/CUR and commands stay in app/.
uint8_t uart_rx_byte;
volatile uint8_t uart_rx_dma_data[128];
uint16_t uart_rx_dma_read = 0;
void bind_board_data_port();

HAL_StatusTypeDef start_uart_receive()
{
    bind_board_data_port();
    if (huart2.hdmarx) {
        if (huart2.hdmarx->Init.Mode != DMA_CIRCULAR) return HAL_ERROR;
        uart_rx_dma_read = 0;
        return HAL_UARTEx_ReceiveToIdle_DMA(&huart2, const_cast<uint8_t *>(uart_rx_dma_data), sizeof(uart_rx_dma_data));
    }
    return HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
}


bool board_tx_ready() { return huart2.gState == HAL_UART_STATE_READY; }
bool board_write(const uint8_t *data, uint16_t size)
{
    return HAL_UART_Transmit_DMA(&huart2, const_cast<uint8_t *>(data), size) == HAL_OK;
}
bool board_recover_receive()
{
    HAL_UART_AbortReceive(&huart2);
    data_port::ReceiveState &rx = data_port::receive_state();
    rx.read = rx.write;
    return start_uart_receive() == HAL_OK;
}
uint32_t board_enter_critical()
{
    const uint32_t token = __get_PRIMASK(); __disable_irq(); return token;
}
void board_leave_critical(uint32_t token) { __set_PRIMASK(token); }
void board_memory_barrier() { __DMB(); }
uint32_t board_milliseconds() { return HAL_GetTick(); }
void bind_board_data_port()
{
    const data_port::Driver driver = {board_tx_ready, board_write, board_recover_receive,
        board_enter_critical, board_leave_critical, board_memory_barrier, board_milliseconds};
    data_port::bind(driver);
}
