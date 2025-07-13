# STM32F407VG FreeRTOS UART example
This example is designed for the STEVE V1.1.0 PCB to allow for UART communication when a radio is connected to the 5V UART connector at the top of the board
The STEVE PCB uses the UART2 (USART) TX and RX signals on pins PA2 and PA3 (pins number 25 and 26)

This code is also compatible with the following STM Microcontrollers:
- STM32F405
- STM32F415
- STM32F417

## Generate UART2 setup code using the STM32CubeIDE
(instructions should be identical for STM32CubeMX)

- create a project for the STM32F407VG (if not already created)
    - ensure that a .ioc file is generated
- open the .ioc file with the interactive user interface so that it can be edited
- in the Pinout & Configuration window > Pinout view
    - select the PA2 pin and assign the "USART2_TX" mode to it
    - select the PA3 pin and assign the "USART2_RX" mode to it
- in the Pinout & Configuration window, select Connectivity > USART2 on the left "Categories" tab
    - in the USART2 Mode area, set the USART2 mode to "Asynchronous" so that the pins operate in UART mode instead of USART mode
    - the RS232 Hardware Flow Control can remain disabled
    - optionally assign a user chosen name for each pin by right clicking the pin in the pinout view and selecting "Enter User Label"
        - PA2 can be called "UART2_TX"
        - PA3 can be called "UART2_RX"
    - in the USART2 Configuration area, go to the Parameter Settings tab and set the Baud Rate to 57600 Bits/s
        - the Word Length (8), Parity (None), and Stop Bits (1) can all remain as default
- save the file using the user interface, the app should prompt you to generate code
    - select yes
    - handle any warnings on a case-by-case basis to allow for code to be generated
- the changes made to the main.h, main.c, stm32f4xx_hal_conf.h, and stm32f4xx_hal_msp.c files can be brought over to this example
