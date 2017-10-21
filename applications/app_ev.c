/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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
 */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "servo_dec.h"
#include "mc_interface.h"
#include "commands.h"
#include "timeout.h"
#include "utils.h"
#include "hw.h"
#include "s_lcd3.h"
#include <math.h>
#include <stdio.h>

// Settings
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8

// Threads
static THD_FUNCTION(ev_thread, arg);
static THD_WORKING_AREA(ev_thread_wa, 1024);

// Private variables
static volatile ev_config config;
static volatile float ms_without_power = 0.0;
static volatile float decoded_level1 = 0.0;
static volatile float read_voltage1 = 0.0;
static volatile float decoded_level2 = 0.0;
static volatile float read_voltage2 = 0.0;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile bool throttle_released_after_brake = true;
static volatile int runmode = 1;

// prototypes
float get_mode_value(void);


void app_ev_configure(ev_config *conf) {
	config = *conf;
	ms_without_power = 0.0;

	if(!config.use_pas) {
		runmode = 1; // PAS 1
	}
}

void app_ev_start(bool use_display, bool use_swdport) {
	stop_now = false;

	chThdCreateStatic(ev_thread_wa, sizeof(ev_thread_wa), NORMALPRIO, ev_thread, NULL);

	if(use_display) {
		s_lcd3_start();
	}

	if(use_swdport) {
		// configure SWDIO and SWDCLK pins as Input. Note: Debugging will not be possible
		// until this is configured back to AF(0). Use ESC Terminal "swdio" command to restore
		palSetPadMode(HW_SWD_PORT_IO, HW_SWD_PIN_IO, PAL_MODE_INPUT_PULLUP);
		palSetPadMode(HW_SWD_PORT_CLK, HW_SWD_PIN_CLK, PAL_MODE_INPUT_PULLUP);
	}
}

void app_ev_stop(void) {
	stop_now = true;

	s_lcd3_stop();

	if(is_running) {
		servodec_stop();
	}

	while (is_running) {
		chThdSleepMilliseconds(1);
	}

	// reset limits
	mc_interface_set_current_limit_app(0);
	mc_interface_set_rpm_limit_app(0xffffff);
}

float app_ev_get_decoded_level1(void) {
	return decoded_level1;
}

float app_ev_get_voltage1(void) {
	return read_voltage1;
}

float app_ev_get_decoded_level2(void) {
	return decoded_level2;
}

float app_ev_get_voltage2(void) {
	return read_voltage2;
}


float get_adc_voltage(uint8_t adc_index) {
	// Read the external ADC pin and convert the value to a voltage.
	float val = (float)ADC_Value[adc_index];
	val /= 4095;
	val *= V_REG;
	return val;
}

float get_mode_value() {
	switch (runmode) {
		case 0: return 0;
		case 1: return config.mode_1_current;
		case 2: return config.mode_2_current;
		case 3: return config.mode_3_current;
		case 4: return config.mode_4_current;
		case 5: return config.mode_5_current;
		case 6: return config.mode_6_current;
		default: return config.mode_1_current;
	}
}



static THD_FUNCTION(ev_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_EV");

	// Set servo pin as an input with pullup
	palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);



	servodec_set_pulse_options(10, 500, false);
	servodec_init(0);
	is_running = true;

	uint8_t divider = 0;

	for(;;) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			ms_without_power = 0;
		}

		if(divider++ > 100 && config.use_pulse) {
			bool padpad = palReadPad(GPIOA, 13);
			commands_printf("SWDIO: %d", padpad ? 1 : 0);
			divider = 0;
		}


		read_voltage1 = get_adc_voltage(ADC_IND_EXT);
		read_voltage2 = get_adc_voltage(ADC_IND_EXT2);
		float throttle = 0, brake = 0;

		// Optionally apply a mean value filter
		if (config.use_filter) {
			static float adc_filter_buffer1[FILTER_SAMPLES];
			static float adc_filter_buffer2[FILTER_SAMPLES];
			static int filter_ptr1 = 0;
			static int filter_ptr2 = 0;

			adc_filter_buffer1[filter_ptr1++] = read_voltage1;
			if (filter_ptr1 >= FILTER_SAMPLES) {
				filter_ptr1 = 0;
			}

			for (int i = 0;i < FILTER_SAMPLES;i++) {
				throttle += adc_filter_buffer1[i];
			}

			throttle /= FILTER_SAMPLES;

			adc_filter_buffer2[filter_ptr2++] = read_voltage2;
			if (filter_ptr2 >= FILTER_SAMPLES) {
				filter_ptr2 = 0;
			}

			for (int i = 0;i < FILTER_SAMPLES;i++) {
				brake += adc_filter_buffer2[i];
			}

			brake /= FILTER_SAMPLES;
		} else {
			throttle = read_voltage1;
			brake = read_voltage2;
		}

		// Linear mapping between the start and end voltages
		throttle = utils_map(throttle, config.voltage1_start, config.voltage1_end, 0.0, 1.0);
		brake = utils_map(brake, config.voltage2_start, config.voltage2_end, 0.0, 1.0);

		// Truncate the read voltages
		utils_truncate_number(&throttle, 0.0, 1.0);
		utils_truncate_number(&brake, 0.0, 1.0);

		decoded_level1 = throttle;
		decoded_level2 = brake;


		if(config.use_display) {
			// read LCD data + config
			// we could use the wheel size data + p1 value instead of wheel_factor.
			// But setting via computer is simpler

			const volatile lcd_rx_data *lcd_data = s_lcd3_get_data();
			//TODO: add validation / timeout for this data
			runmode = lcd_data->assist_level;

		}


		bool is_pedaling = false;

		float pwr = 0;

		if(config.use_pas) {
			uint32_t last_servo_time = servodec_get_time_since_update();

			//commands_printf("val: %.4f   time: %d", (double)servo_val, last_servo_time);
			if(last_servo_time < 1000) {
				is_pedaling = true;
			}


			if(is_pedaling) {
				pwr = get_mode_value();

			}

			// Apply PAS ramping (TODO: make this configurable)
			static systime_t last_time_pas = 0;
			static float pwr_ramp_pas = 0.0;
			const float ramp_time_pas = fabsf(pwr) > fabsf(pwr_ramp_pas) ? 5 : 1;

			if (ramp_time_pas > 0.01) {
				const float ramp_step_pas = (float)ST2MS(chVTTimeElapsedSinceX(last_time_pas)) / (ramp_time_pas * 1000.0);
				utils_step_towards(&pwr_ramp_pas, pwr, ramp_step_pas);
				last_time_pas = chVTGetSystemTimeX();
				pwr = pwr_ramp_pas;
			}
		}

		if(config.use_throttle) {
			if(config.use_pas) {
				if(is_pedaling | config.use_throttle_wo_pas) {
					// throttle allows giving more power than pas mode would normally be
					pwr = utils_max_abs(pwr, throttle);
				}
			} else {
				pwr = throttle;
			}
		}

		if(config.use_throttle_brake && brake > 0.01) {	// if brake is at least 1% use brake for power
			pwr = -brake;
			throttle_released_after_brake = false;
		} else {
			// make sure the user released the throttle after braking, otherwise disable pwr

			if(!throttle_released_after_brake) {
				if(throttle <= 0.01) {
					throttle_released_after_brake = true;
				} else {
					pwr = 0;
				}
			}
		}


		// Apply throttle curve
		pwr = utils_throttle_curve(pwr, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		// now pwr contains a value between -1.0 (braking) and 1.0 (full throttle)

		// Apply global ramping
		static systime_t last_time = 0;
		static float pwr_ramp = 0.0;
		const float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&pwr_ramp, pwr, ramp_step);
			last_time = chVTGetSystemTimeX();
			pwr = pwr_ramp;
		}





		float current = 0.0;

		bool current_mode_brake = false;
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		//const float rpm_now = mc_interface_get_rpm();

		// Use the filtered and mapped voltage for control according to the configuration.

		if (pwr >= 0.0) {
			current = pwr * mcconf->lo_current_motor_max_now;
		} else {
			current = fabsf(pwr * mcconf->lo_current_motor_min_now);
			current_mode_brake = true;
		}

		if (pwr < 0.001) {
			ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		}


		if(config.use_display) {
			// display output
			battery_level battery = BORDER_FLASHING;

			if(mcconf->l_min_vin < mcconf->l_max_vin) {
				float bat_level = (GET_INPUT_VOLTAGE() - mcconf->l_min_vin) / (mcconf->l_max_vin - mcconf->l_min_vin);

				if(bat_level >= 0.7) {
					battery = B4_BARS;
				} else if(bat_level >= 0.5) {
					battery = B3_BARS;
				} else if(bat_level >= 0.4) {
					battery = B2_BARS;
				} else if(bat_level >= 0.3) {
					battery = B1_BAR;
				} else if(bat_level < 0) {
					battery = BORDER_FLASHING;
				} else {
					battery = EMPTY_BOX;
				}
			}

			uint16_t wheel_rot_period = 0xffff;

			if(config.use_pulse) {
				// use SWDio pin as input for pulse count
				//TODO

			} else {
				// use ERPM + config.wheel_factor
				float erpm = fabs(mc_interface_get_rpm());
				if(erpm > 100) {
					wheel_rot_period = (1 / (erpm / config.wheel_factor)) * 1000;
				}
			}

			//lcd_set_data(GET_INPUT_VOLTAGE() * mc_interface_get_tot_current_in_filtered(), 500, 0, pwr > 0.01, false, false);
			lcd_set_data(GET_INPUT_VOLTAGE() * mc_interface_get_tot_current_in_filtered(),
						 wheel_rot_period, 0, battery, pwr > 0.01, false, false);
		}




		// check safe start, if the output has not been zero for long enough
		if (ms_without_power < MIN_MS_WITHOUT_POWER ) {
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before) {
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			mc_interface_set_brake_current(timeout_get_brake_current());

			continue;
		}



		// Reset timeout
		timeout_reset();


		if(runmode == 6) {
			mc_interface_set_current_limit_app(config.mode_6_current * mcconf->l_current_max);

			mc_interface_set_pid_speed(config.mode_6_speed);

		} else {
			mc_interface_set_current_limit_app(0);
			mc_interface_set_rpm_limit_app(3000);


			if (current_mode_brake) {
				mc_interface_set_brake_current(current);
			} else {
				mc_interface_set_current(current);
			}
		}


	}
}
