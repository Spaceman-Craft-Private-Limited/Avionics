Here the explanation of the uploaded .ino files content.
1. 80kv_BLDC.ino - In this file the code is written for a 80kv BLDC motor controlled by a 7Semi BLDC Motor Driver using ESP32 Dev kit.
The GPIO pin connections from 7semi BLDC driver to ESP32 are mentioned in the Code itself as Commented line.
2. all_in_one-damper_repulsion.ino - In this file the code is written for whole the prototype circuit, and in this file two types of code were written.
The first type is written that the stepper motor is controlled by the TB6600 motor driver and the whole code is commented.
The Second type is the code that the stepper motor is controlled by A4988 stepper motor driver and the whole code uncommented.
All the circuit connection is mentioned in the code itself and the Microcontroller used for this whole system is ESP32 Dev kit.
3. small_prototype_system_code_70rpm_noattraction.ino -  In this file the code is written to control the stepper motor by A4988 motor driver using Arduino uno microcontroller.
The term noattraction means the relay is always on, So the electromagnet connected in relay is always energized.
All the pins configurations are mentioned in the code itself.
4. small_prototype_system_code_70rpm_withattraction.ino - In this file the code is written to control the stepper motor by A4988 motor driver using Arduino uno microcontroller.
The term withattraction means the relay will turn on for few sec and turn off based on the logic given in code, So the electromagnet connected in relay will energized based on time.
All the pins configurations are mentioned in the code itself.
