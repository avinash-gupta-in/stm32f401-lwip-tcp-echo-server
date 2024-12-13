This is a part of code samples for course available on
www.stm32tutorials.com

## SIMPLE TCP ECHO SERVER USING LWIP

This tcp server listens on TCP port 5000 you can then connect using
Hercules TCP client and send data to this server. The server will
echo back the same data to the client which will be visible on
Hercules.
The aim of this sample code is to teach the students how to write a
TCP Server in lwIP
 

## Hardware

  
  MCU: STM32F401CC (48 PIN)
  Board: STM32F401CC Black-pill
  Crystal: 25MHz
  CPU Speed: 84MHz
  RAM: 64KB
  Core: ARM Cortx-M4
 
  **Ethernet Controller**
  
  ENC28J60 (From Microchip)
  
  Interface: SPI
  SPI Instance: SPI1
  
  **PINS**
       
  	
|  MCU  | ENC28J60 |
|--|--|
|PA5 (SPI1_SCK)|SCK|
|PA6 (SPI1_MISO)|MISO (SO)
|PA7 (SPI1_MOSI)|MOSI (SI)
|PB2 (EXTI) |INT
|PA4|CS|
|PA8|RESET|

 
  **USART Debug**
  
  Debug messages are printed on USART2
  Baud Rate: 115200 (8N1)
  PA2 (TX)
  PA3 (RX)
  
![Output of the program (on Serial Terminal)](http://extremeelectronics.co.in/github/stm32-lwip/tcp-echo-server-stm32-lwip-realterm.png)

Using Hercules TCP Client to send data to our server for testing.

![Hercules used for testing our TCP server ](http://extremeelectronics.co.in/github/stm32-lwip/tcp-echo-server-stm32-lwip-hercules.png)

