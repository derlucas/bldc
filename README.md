# BLDC - Electronic Speed Controller Firmware

This is the source code for the Electronic Speed Controllers(ESC) like
VESC&copy;, FOCBOX, ESK1.1, OLLIN ESC1.1 and probably others.

This fork is based on Benjamin Vedders bldc Firmware. So mostly his work and
kudos to him.


In this fork i removed the trademark stuff i don't like and add my wanted
features. Please see my esc_tool repository for a compatible \*ESC tool to
configure your \*ESC.


## eBike - Pedelec App

Currently i'm implementing a new application for Electric vehicles (app_ev).
There are options for analog inputs for Accelleration and Braking.
Inputs for PAS Detector and Wheel Rotation detectors.

The goal is to be able to replace regular eBike controllers with this \*ESC and
have all the nice config options we like.

With this App, you are able to make a legal eBike (some countries need detection
for pedaling etc).


### Current working status (roughly):
 - Support for Analog Throttle + Braking
 - Support for PAS Sensor (on PPM connector)
 - Support for s-LCD3 Display
   - Display Speed, Wattage, Battery
   - "Gear" Selection (Mode 0-5)
   - Power Assisted Push Mode
 - Support for config via my esc_tool (see https://github.com/derlucas/esc_tool)


### Current todo:
 - Testing everything
 - LCD Implementation sometimes hangs/crashes/fails
 - implement wheel rotation sensor on some spare IO (e.g. SWDIO) (needed for motors with freewheel)
 - implement max speed from Display (read wheel diameter from display + wheel_factor)
 - even more testing


### Maybe features in future:
 - enable more display config parameters to change stuff in controller (P and C params)
 - make the code cleaner and the display stuff more modular
 - maybe add another Display Type (send me some)


#### Documentation

You need to download the bldc firmware and the esc_tool from github.
Currently there is no release, so you need to compile the code by yourself. (See vedders Documentation about compiling)

Start the new esc_tool and connect to you \*ESC. It will warn you about old firmware. Upload the firmware of this repo, reboot \*ESC and reconnect.
You can then select the EV application (it is enabled by default by now) and make your settings to your needs.


##### Settings:

- General
 - Ramping: Global Ramping (incl PAS and Thumb Throttle)
 - Display Enable: If using the s-LCD3
 - Wheel factor: factor between eRPM and real wheel RPM (eRPM/RPM)
 - Enable PAS Mode: select if you want to use PAS. If enabled you connect a PAS sensor to PPM input. Only if pedaling is detected, the motor will spin. (except of cause power assisted push)
 - Use Pulses Input: select if you have a motor with freewheel and want to use a speed sensor. If disabled, the motor eRPM will be used for displaying the speed. (option only needed with a Display)
 - Use throttle: if enabled, the ADC1 pin is used as Throttle
 - Use throttle brake: if enabled, the ADC2 pin is used as brake. This only makes sense if you are not using a motor with freewheel.
 - Use display Vmax: if enabled, the \*ESC will use the Speed limit configured into the Display.
 - Power Mode 1-5:
    Set the wanted percentage of current (relative to max motor  current) for each "gear"(mode).
 - Push Assist mode 6: configure percentage of current and max speed for the power assisted push function. You have to enter eRPM. (TODO: fill in a formula wheelsize+km/h+gearratio+motorpoles = eRPM)
- Mapping
  - the usual analog input mapping like in the ADC app
- Throttle curve
  - the usual throttle curve like in the other apps.
    Be aware: the curve also affects the current levels in the Power Modes 1-5

##### Wiring / Connections:

- Throttle Accelleration: GND - ADC1 - 3.3V
- Braking (analog): GND - ADC2 - 3.3V
- PAS Sensor: GND - PPM (3pin header) - 3.3V + Pullup between Signal and 3.3V
- Display (s-LCD3):
  - Red: +48V
  - Blue: Control (use with AntiSpark Switch to turn on \*ESC)
  - Black: GND
  - Green (Data1): RX
  - Yellow (Data2): TX
