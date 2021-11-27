# UART controlled Servo

send commands from Serial port to STM to controll Servos.
Message protocoll:
> "!" ... start message character
> uint16_t ... unsigned int, should be between 250 adn 1250, 250 = 0 degree, 1250 = 180 degree
> "#" ... end of message character

Serial port settings:
> Baudrate    9600
> Data Bits   8
> Parity      even

Movment slowes down when reaching goal with
`servo1Smoothed = (goal_pos * 0.05) + (current_pos * 0.95);`



![alt text](https://github.com/Dannyrevenger/RoboMaster-Internal_Competition/blob/main/image/Uart_controlled_Servo_bb.png)