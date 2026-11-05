#pragma once
// Included after uart_port_impl.hpp in this board's setup translation unit.
namespace board_uart {
inline bool init() { return start_uart_receive() == HAL_OK; }
}
