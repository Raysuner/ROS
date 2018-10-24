/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define RESET_ENCODERS 'r'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define READ_PIDIN     'i'
#define READ_PIDOUT    'o'
#define A_WHEEL         1
#define B_WHEEL         2
#define C_WHEEL         3
#define D_WHEEL         4

#define FORWARDS        true
#define BACKWARDS       false

#endif


