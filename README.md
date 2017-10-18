BLDC - Electronic Speed Controller Firmware
=====================

This is the source code for the Electronic Speed Controllers(ESC) like
VESC&copy;, FOCBOX, ESK1.1, OLLIN ESC1.1 and probably others.

This fork is based on Benjamin Vedders bldc Firmware. So mostly his work and
kudos to him.


In this fork i removed the trademark stuff i don't like and add my wanted
features. Please see my esc_tool repository for a compatible ESC tool to
configure your ESC.


eBike - Pedelec Features
--------------

Currently i'm implementing a new application for Electric vehicles (app_ev).
There are options for analog inputs for Accelleration and Braking.
Inputs for PAS Detector and Wheel Rotation detectors.

The goal is to be able to replace regular eBike controllers with this ESC and
have all the nice config options we like.

With this App, you are able to make a legal eBike (some countries need detection
for pedaling etc).


Current working status (roughly):
 - Support for Analog Throttle + Braking
 - Support for PAS Sensor (on PPM connector)
 - Support for s-LCD3 Display
  - Display Speed, Wattage, Battery
  - "Gear" Selection (Mode 0-5)
  - Power Assisted Push Mode
 - Support for config via my esc_tool (see https://github.com/derlucas/esc_tool)


Current todo:
 - Testing everything
 - LCD Implementation sometimes hangs/crashes/fails
 - implement wheel rotation sensor on some spare IO (e.g. SWDIO) (needed for motors with freewheel)
 - implement max speed from Display (read wheel diameter from display + wheel_factor)
 - even more testing


Maybe features in future:
 - enable more display config parameters to change stuff in controller (P and C params)
 - make the code cleaner and the display stuff more modular
 - maybe add another Display Type (send me some)
