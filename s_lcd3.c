/**
Copyright 2017 Lucas Pleß		hello@lucas-pless.com

This file is part of the VESC firmware.

The VESC firmware is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

The VESC firmware is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
**/

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "s_lcd3.h"
#include "mc_interface.h"

#include <string.h>

// settings
#define BAUDRATE						9600
#define SERIAL_RX_BUFFER_SIZE			48

// Threads
static THD_FUNCTION(packet_process_thread, arg);
static THD_WORKING_AREA(packet_process_thread_wa, 4096);
static thread_t *process_tp = 0;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static volatile bool is_running = false;

static volatile lcd_rx_data lcd_data_rx;
static volatile lcd_tx_data lcd_data_tx;



/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
	(void)uartp;
	serial_rx_buffer[serial_rx_write_pos++] = c;

	if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
		serial_rx_write_pos = 0;
	}

	chSysLockFromISR();
	chEvtSignalI(process_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
		txend1,
		txend2,
		rxend,
		rxchar,
		rxerr,
		BAUDRATE,
		0,
		USART_CR2_LINEN,
		0
};


void s_lcd3_start() {

	const volatile mc_configuration *mcconf = mc_interface_get_configuration();

	lcd_data_tx.power = GET_INPUT_VOLTAGE() * mc_interface_get_tot_current_in_filtered();
	lcd_data_tx.battery_lvl = BORDER_FLASHING;
	lcd_data_tx.error_info  = 0;
	lcd_data_tx.motor_temperature = 23;
	lcd_data_tx.wheel_rotation_period = 0;
	lcd_data_tx.show_animated_circle = false;
	lcd_data_tx.show_assist = false;
	lcd_data_tx.show_cruise_control = false;


	serial_rx_read_pos = 0;
	serial_rx_write_pos = 0;

	if (!is_running) {
		chThdCreateStatic(packet_process_thread_wa, sizeof(packet_process_thread_wa),
				NORMALPRIO, packet_process_thread, NULL);
		is_running = true;
	}

	uartStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);


}

void s_lcd3_stop() {

	uartStop(&HW_UART_DEV);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);

}


const volatile lcd_rx_data* s_lcd3_get_data() {
	return &lcd_data_rx;
}

void lcd_set_data(uint16_t wheel_rotation_period, uint8_t error_display,
				  bool anim_throttle, bool cruise, bool assist) {

	lcd_data_tx.wheel_rotation_period = wheel_rotation_period;
	lcd_data_tx.error_info = error_display;
	lcd_data_tx.show_animated_circle = anim_throttle;
	lcd_data_tx.show_assist = assist;
	lcd_data_tx.show_cruise_control = cruise;
}



static THD_FUNCTION(packet_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("s-lcd3 uart process");

	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (serial_rx_read_pos != serial_rx_write_pos) {

			uint8_t byte = serial_rx_buffer[serial_rx_read_pos++];

			if(byte == 0x0e) {
				//probably last byte. check previous byte
				int8_t pos = serial_rx_read_pos-1;
				if(pos == -1) {
					pos = SERIAL_RX_BUFFER_SIZE-1;
				}

				if(serial_rx_buffer[pos] == 0x32) {


				}

			}



			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}
