/****************************************************************************
 * 	Copyright (C) 2023 Alex Shepherd
 *
 * 	This library is free software; you can redistribute it and/or
 * 	modify it under the terms of the GNU Lesser General Public
 * 	License as published by the Free Software Foundation; either
 * 	version 2.1 of the License, or (at your option) any later version.
 *
 * 	This library is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * 	Lesser General Public License for more details.
 *
 * 	You should have received a copy of the GNU Lesser General Public
 * 	License along with this library; if not, write to the Free Software
 * 	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 *****************************************************************************
 * 	DESCRIPTION
 * 	This module provides functions that manage the sending and receiving of LocoNet packets.
 *
 * 	As bytes are received from the LocoNet, they are stored in a circular
 * 	buffer and after a valid packet has been received it can be read out.
 *
 * 	When packets are sent successfully, they are also appended to the Receive
 * 	circular buffer so they can be handled like they had been received from
 * 	another device.
 *
 * 	Statistics are maintained for both the send and receiving of packets.
 *
 * 	Any invalid packets that are received are discarded and the stats are
 * 	updated approproately.
 *
 *****************************************************************************/
#pragma once

#ifdef ARDUINO_ARCH_ESP32

#include <LocoNetStream.h>
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "hal/uart_types.h"
#include "esp32-hal.h"

class LocoNetStreamESP32: public LocoNetStream
{
public:
    LocoNetStreamESP32 (HardwareSerial &serial, int8_t rxPin, int8_t txPin, bool rxInvert, bool txInvert, LocoNetBus *bus) : LocoNetStream (bus)
    {
        _uart_nr = (uart_port_t) getUartNum(&serial);
        _rxPin = rxPin;
        _rxInvert = rxInvert;
        _txPin = txPin;
        _txInvert = txInvert;
        _serialPort = &serial;
    };

    void start (void)
    {
        _serialPort->begin (LOCONET_BAUD, SERIAL_8N1, _rxPin, _txPin, false);

        if (_rxInvert || _txInvert)
        {
            uint32_t newInvertState = UART_SIGNAL_INV_DISABLE;

            if (_rxInvert)
                newInvertState |= UART_SIGNAL_RXD_INV;

            if (_txInvert)
                newInvertState |= UART_SIGNAL_TXD_INV;

            uart_set_line_inverse (_uart_nr, newInvertState);
        };

        begin (_serialPort);
    }

    void end (bool fullyTerminate = true);

    bool isBusy (void)
    {
        uart_dev_t *hw = getUart (_uart_nr);

        #if defined(CONFIG_IDF_TARGET_ESP32)
        return hw->status.st_urx_out != 0;
        #else
        return hw->fsm_status.st_urx_out != 0;
        #endif
    };

    void beforeSend (void)
    {
        _tempRxFifoThreshold = updateRxFifoFullThreshold (1);
    };

    void afterSend (void)
    {
        updateRxFifoFullThreshold (_tempRxFifoThreshold);
    };

    void sendBreak (void)
    {
        _serialPort->updateBaudRate (LOCONET_BREAK_BAUD);
        _serialPort->write ( (uint8_t) 0);
        _serialPort->flush();
        _serialPort->updateBaudRate (LOCONET_BAUD);
    };

private:
    uart_port_t			_uart_nr;
    HardwareSerial * 	_serialPort;
    uint32_t 			_tempRxFifoThreshold;
    int8_t				_rxPin;
    bool				_rxInvert;
    int8_t				_txPin;
    bool				_txInvert;

    uint32_t updateRxFifoFullThreshold (uint32_t newThreshold)
    {
        uart_dev_t *hw = getUart (_uart_nr);

        uint32_t oldThreshold = hw->conf1.rxfifo_full_thrhd;
        hw->conf1.rxfifo_full_thrhd = newThreshold;

        return oldThreshold;
    };

    static uart_dev_t* getUart(const int uartNum) {
        if(uartNum == 0) return &UART0;
    #if SOC_UART_NUM > 1
        if(uartNum == 1) return &UART1;
    #endif
    #if SOC_UART_NUM > 2
        if(uartNum == 2) return &UART2;
    #endif
    #if SOC_UART_NUM > 3
        if(uartNum == 3) return &UART3;
    #endif
    #if SOC_UART_NUM > 4
        if(uartNum == 4) return &UART4;
    #endif
    #if SOC_UART_NUM > 5
        if(uartNum == 5) return &UART5;
    #endif
        assert(false);  // fail fast
        return nullptr;
    }

    static int getUartNum(const HardwareSerial *serial) {
        if(serial == &Serial0) return 0;
    #if SOC_UART_NUM > 1
        if(serial == &Serial1) return 1;
    #endif
    #if SOC_UART_NUM > 2
        if(serial == &Serial2) return 2;
    #endif
    #if SOC_UART_NUM > 3
        if(serial == &Serial3) return 3;
    #endif
    #if SOC_UART_NUM > 4
        if(serial == &Serial4) return 4;
    #endif
    #if SOC_UART_NUM > 5
        if(serial == &Serial5) return 5;
    #endif
        assert(false);  // fail fast
        return 0;
    }

};
#endif	// ARDUINO_ARCH_ESP32
