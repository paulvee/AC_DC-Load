# AC_DC-Load
Files for my AC_DC Dynamic Load development project

I will update files as I go through the development.
The files will be development files and not a final version until declared that way.
There will be no version control, what you see is the latest, look at the dates for more information.

Nano-rms-test is the latest functioning prototype for the complete dynamic AC/DC load system, as far as I got. It is very experimental!

Nano-rms-test-V2 is the version that supports the True RMS LTC1966 chip that I now use for the DUT voltage measurements. This version supports the Power calculation, but the display code for the OLED only works for the DC mode. I did not finish the AC mode. That is done in V3.

Nano_rms_test-V4.6 is the version that supports the Version 4 prototype PCB. Pressing the rotary decoder button will cycle through the Constant Current, Constant Power and Constant Resistance modes. It does not have any calibration data in it. This is the version that I'm working with now and went through quite a bit of reconstructing to make the code cleaner. It supports the CC, CP and CR modes, but the CR mode needs more work to make it user friendly. This version is now using the software rms calculations, the LTC1966 is no longer required for this level of precision.
Based on the many findings of this prototype, I'm now in the process of designing another PCB.

Measure_example_rms is a fully functional, interrupt driven rms calculator using the Nano ADC0 input
