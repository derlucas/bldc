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
#include "commands.h"

#include <string.h>

// settings
#define BAUDRATE						9600
#define SERIAL_RX_BUFFER_SIZE			48
#define PACKET_MAX_PL_LEN				13

// Threads
static THD_FUNCTION(lcd_process_thread, arg);
static THD_WORKING_AREA(lcd_process_thread_wa, 4096);
static thread_t *process_tp = 0;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static volatile bool is_running = false;
static volatile bool stop_now = true;

static volatile lcd_rx_data lcd_data_rx;
static volatile lcd_tx_data lcd_data_tx;

unsigned char rx_buffer[PACKET_MAX_PL_LEN];
unsigned char tx_buffer[PACKET_MAX_PL_LEN];
unsigned int rx_data_ptr;
unsigned char crc;

// Prototypes
void send_lcd_data(void);


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

	stop_now = false;

	lcd_data_tx.power = 0;
	lcd_data_tx.battery_lvl = BORDER_FLASHING;
	lcd_data_tx.error_info  = 0;
	lcd_data_tx.motor_temperature = 23;
	lcd_data_tx.wheel_rotation_period = 0xffff;
	lcd_data_tx.show_animated_circle = false;
	lcd_data_tx.show_assist = false;
	lcd_data_tx.show_cruise_control = false;

	lcd_data_rx.assist_level = 1;
	lcd_data_rx.headlight = false;


	serial_rx_read_pos = 0;
	serial_rx_write_pos = 0;
	rx_data_ptr = 0;


	chThdCreateStatic(lcd_process_thread_wa, sizeof(lcd_process_thread_wa),
			NORMALPRIO, lcd_process_thread, NULL);

	uartStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);

	commands_printf("lcd started");

}

void s_lcd3_stop() {
	stop_now = true;

	if (is_running) {
		chEvtSignalI(process_tp, (eventmask_t) 1);
	}

	uartStop(&HW_UART_DEV);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);

	while (is_running) {
		chThdSleepMilliseconds(1);
	}
	commands_printf("lcd stopped");
}


const volatile lcd_rx_data* s_lcd3_get_data() {
	return &lcd_data_rx;
}

void lcd_set_data(uint16_t power, uint16_t wheel_rotation_period, uint8_t error_display,
				  battery_level battery,
				  bool anim_throttle, bool cruise, bool assist) {
	lcd_data_tx.power = power;
	lcd_data_tx.wheel_rotation_period = wheel_rotation_period;
	lcd_data_tx.error_info = error_display;
	lcd_data_tx.battery_lvl = battery;
	lcd_data_tx.show_animated_circle = anim_throttle;
	lcd_data_tx.show_assist = assist;
	lcd_data_tx.show_cruise_control = cruise;
}

void send_lcd_data() {
	tx_buffer[0] = 0x41;
	tx_buffer[1] = lcd_data_tx.battery_lvl;
	tx_buffer[2] = 0x24;
	tx_buffer[3] = lcd_data_tx.wheel_rotation_period >> 8;
	tx_buffer[4] = lcd_data_tx.wheel_rotation_period & 0xff;
	tx_buffer[5] = lcd_data_tx.error_info;

	tx_buffer[7] = lcd_data_tx.show_animated_circle | lcd_data_tx.show_cruise_control << 3 | lcd_data_tx.show_assist << 4;
	tx_buffer[8] = lcd_data_tx.power / 7;
	tx_buffer[9] = lcd_data_tx.motor_temperature - 15;
	tx_buffer[10] = 0x00;
	tx_buffer[11] = 0x00;

	// CRC
	tx_buffer[6] = tx_buffer[1] ^ tx_buffer[2] ^ tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5] ^ tx_buffer[7] ^ tx_buffer[8] ^ tx_buffer[9];

	/*
	commands_printf("send: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", tx_buffer[0], tx_buffer[1], tx_buffer[2], tx_buffer[3], tx_buffer[4],
																			   tx_buffer[5], tx_buffer[6], tx_buffer[7], tx_buffer[8], tx_buffer[9]);
 	*/

	// Wait for the previous transmission to finish.
	while (HW_UART_DEV.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	uartStartSend(&HW_UART_DEV, 12, tx_buffer);
}


void process_byte(uint8_t rx_data) {

	rx_buffer[rx_data_ptr++] = rx_data;
	//commands_printf("b: %02x", rx_data);

	if (rx_data_ptr >= 13) {

		if(rx_buffer[11] == 0x32 && rx_buffer[12] == 0x0E) {


			lcd_data_rx.power_monitoring_p5 = rx_buffer[0];
			lcd_data_rx.headlight = (rx_buffer[1] & 0x80) >> 7;
			lcd_data_rx.assist_level = (rx_buffer[1] & 0x07);

			lcd_data_rx.max_speed = ((rx_buffer[2] & 0xF8) >> 3 | (rx_buffer[4] & 0x20)) + 10;
			lcd_data_rx.wheel_size = (rx_buffer[2] & 0x07) << 2 | (rx_buffer[4] & 0xC0) >> 6;

			lcd_data_rx.pulse_per_revolution_p2 = (rx_buffer[4] & 0x07);
			lcd_data_rx.pas_mode_p3 = (rx_buffer[4] & 0x08) >> 3;
			lcd_data_rx.throttle_needs_pas_p4 = (rx_buffer[4] & 0x10) >> 4;

			lcd_data_rx.motor_characteristic_p1 = rx_buffer[3];

			//uint8_t crc = rx_buffer[5];


			lcd_data_rx.pas_sensor_mode_c1 = (rx_buffer[6] & 0x38) >> 3;
			lcd_data_rx.motor_phase_classification_c2 = rx_buffer[6] & 0x07;

			lcd_data_rx.max_current_adjust_c5 = rx_buffer[7] & 0x0f;
			lcd_data_rx.pas_tuning_c14 = (rx_buffer[7] & 0x60) >> 5;

			lcd_data_rx.throttle_mode_c4 = (rx_buffer[8] & 0xE0) >> 5;

			/*
			commands_printf("p5: %d, light: %d  assist: %d, maxspd: %d, wheelsize: %d, ppr_p2: %d, pasmode_p3: %d, "
					"throttleneedpas_p4: %d, motorchar_p1: %d, passensmode_c1: %d, phase_c2: %d, maxcurre_c5: %d, thromo_c4: %d, "
					"pastune_c14: %d ", lcd_data_rx.power_monitoring_p5,
										  lcd_data_rx.headlight,
										  lcd_data_rx.assist_level, lcd_data_rx.max_speed, lcd_data_rx.wheel_size, lcd_data_rx.pulse_per_revolution_p2,
										  lcd_data_rx.pas_mode_p3, lcd_data_rx.throttle_needs_pas_p4, lcd_data_rx.motor_characteristic_p1,
										  lcd_data_rx.pas_sensor_mode_c1, lcd_data_rx.motor_phase_classification_c2, lcd_data_rx.max_current_adjust_c5,
										  lcd_data_rx.throttle_mode_c4, lcd_data_rx.pas_tuning_c14);
			*/


			send_lcd_data();


		}


		rx_data_ptr = 0;
	}

}


static THD_FUNCTION(lcd_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("s-lcd3");

	process_tp = chThdGetSelfX();

	is_running = true;

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		if (stop_now) {
			is_running = false;
			return;
		}

		while (serial_rx_read_pos != serial_rx_write_pos) {

			process_byte(serial_rx_buffer[serial_rx_read_pos++]);


			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}
