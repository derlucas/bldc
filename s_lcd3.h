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

#ifndef _S_LCD3_H
#define _S_LCD3_h

#include <stdint.h>
#include <stdbool.h>


typedef enum {
	WS_10_INCH,
	WS_12_INCH,
	WS_14_INCH,
	WS_16_INCH,
	WS_18_INCH,
	WS_20_INCH,
	WS_22_INCH,
	WS_24_INCH,
	WS_26_INCH,
	WS_700_C,
	WS_28_INCH
} wheel_size;

typedef enum {
	TORQUE,
	SPEED,
} pas_mode;



typedef struct
{
    // Parameters received from display in operation mode:
	uint8_t power_monitoring_p5;
    uint8_t assist_level;              // 0..6 Power Assist Level
    bool headlight;                 	// head- and backlight on/off
    uint8_t motor_characteristic_p1;	// Motor Characteristic Parameter Setting Mode
    									// P1 = motor gear reduction ratio x number of rotor magnets,
    									// just round if there’s any decimal

    uint8_t max_speed;
    wheel_size wheel_size;
    uint8_t	pulse_per_revolution_p2;
    pas_mode pas_mode_p3;
    bool throttle_needs_pas_p4;			// if true, throttle may only work if user is pedaling
    uint8_t pas_sensor_mode_c1;			// 0-6
    uint8_t motor_phase_classification_c2;
    uint8_t throttle_mode_c4;
    uint8_t max_current_adjust_c5;
    uint8_t pas_tuning_c14;


} lcd_rx_data;

typedef enum {
	EMPTY_BOX = 0,
	BORDER_FLASHING = 1,
	ANIMATED_CHARGING = 2,
	EMPTY = 3,
	B1_BAR = 4,
	B2_BARS = 8,
	B3_BARS = 12,
	B4_BARS = 16
} battery_level;

typedef struct {
	battery_level battery_lvl;
	uint16_t wheel_rotation_period;
	uint8_t error_info;
	bool show_animated_circle;
	bool show_cruise_control;
	bool show_assist;
	uint16_t power; 			// in watts
	int8_t motor_temperature;
} lcd_tx_data;

void s_lcd3_start(void);
void s_lcd3_stop(void);

void s_lcd3_work(void);
const volatile lcd_rx_data* s_lcd3_get_data(void);
void lcd_set_data(uint16_t power, uint16_t wheel_rotation_period, uint8_t error_display,
				  battery_level battery, bool anim_throttle, bool cruise, bool assist);

#endif
