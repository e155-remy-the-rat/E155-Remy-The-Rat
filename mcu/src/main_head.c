#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "main.h"

void updateRatState(uint8_t recievedState)
{
    digitalWrite(RP_1, (recievedState & (1 << 0)) ? 1 : 0); // Bit 0 -> RP_1
    digitalWrite(RP_2, (recievedState & (1 << 1)) ? 1 : 0); // Bit 1 -> RP_2
    digitalWrite(RP_3, (recievedState & (1 << 2)) ? 1 : 0); // Bit 2 -> RP_3
    digitalWrite(RP_4, (recievedState & (1 << 3)) ? 1 : 0); // Bit 3 -> RP_4
}


void RX(void)
{
  uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the address for RX pipe
  nRF24_DisableAA(0xFF); // disable ShockBurst
  nRF24_SetRFChannel(90); // set RF channel to 2490MHz
  nRF24_SetDataRate(nRF24_DR_2Mbps); // 2Mbit/s data rate
  nRF24_SetCRCScheme(nRF24_CRC_1byte); // 1-byte CRC scheme
  nRF24_SetAddrWidth(5); // address width is 5 bytes
  nRF24_SetAddr(nRF24_PIPE1, ADDR); // program pipe address
  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 10); // enable RX pipe#1 with Auto-ACK: disabled, payload length: 10 bytes
  nRF24_SetOperationalMode(nRF24_MODE_RX); // switch transceiver to the RX mode
  nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
  // then pull CE pin to HIGH, and the nRF24 will start a receive...

  uint8_t nRF24_received_payload[32]; // buffer for payload
  uint8_t payload_length; // variable to store a length of received payload
  uint8_t pipe; // pipe number
  nRF24_CE_H; // start receiving
  while (1) {
      // constantly poll the status of RX FIFO...
      if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
          // the RX FIFO have some data, take a note what nRF24 can hold up to three payloads of 32 bytes...
          pipe = nRF24_ReadPayload(nRF24_received_payload, &payload_length); // read a payload to buffer
          nRF24_ClearIRQFlags(); // clear any pending IRQ bits
          // now the nRF24_payload buffer holds received data
          // payload_length variable holds a length of received data
          // pipe variable holds a number of the pipe which has received the data
          // ... do something with received data ...

          // We assume the first byte in the payload contains the rat state to send to the fpga
          updateRatState(nRF24_received_payload[0]);
          for (volatile int i = 0; i < 10000000; i++); // Only update once a delay
      }
  }

}

 int main_head(void) {

  GPIO_TypeDef * GPIO_PORT_PTR = gpioPinToBase(PB4);
  int pin_offset = gpioPinOffset(PB4);
  GPIO_PORT_PTR->PUPDR &= ~(0b11 << 8);

  // Configure flash latency and set clock to run at 84 MHz
  configureFlash();
  configureClock();

  gpioEnable(GPIO_PORT_A);
  gpioEnable(GPIO_PORT_B);
  gpioEnable(GPIO_PORT_C);


  pinMode(RF_CE, GPIO_OUTPUT); 

  pinMode(RP_1, GPIO_OUTPUT); 
  pinMode(RP_2, GPIO_OUTPUT); 
  pinMode(RP_3, GPIO_OUTPUT); 
  pinMode(RP_4, GPIO_OUTPUT); 


  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
  initTIM(TIM2);
  initTIM(TIM6);

  // Enable GPIO clock
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);

  // Configure onboard LED
  pinMode(LED_PIN, GPIO_OUTPUT);
  digitalWrite(LED_PIN, 0);


  initSPI(0b100, 0, 0);


  while (1) {
    RX(); // set this to either RX or TX and then upload to corresponding MCU
  }

}
