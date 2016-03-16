# MPI-Particle
IoT for audio projects using Particle Photon/Spark Core
Firmware and Android dev code

v.01 (3/13/16) - Code for KG I2C digital attenuators balanced operation, used with Spark Core MCU interfaced with 4D Systems uLCD-32PTU front end. No balance control, will be added in future version. Added timer interrupt library, which controls LED on D7 blinking. 

v.02 (3/14/16) - Code now includes a balance control, which allows a "gain" of up to 128 steps in each direction. The balance decreases when the outer limits of a channel are met. Added Adafruit ADS1x15 ADC library. 
