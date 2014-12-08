#include "rumbleTxRx.h"

#ifndef RUMBLECATION_H_
#include "rumblecation.h"
#endif



void rumblecationInit(void){
	rumbleInit();
}

void receiveCallback(uint16_t);
uint8_t calculateOddParity(uint8_t in);
void (*callBackByte)(uint8_t);

void rumbleListen(void (*returnByte)(uint8_t)) {
	rumbleRx(receiveCallback);
	callBackByte = returnByte;
}

void rumbleEnd(void) {
	rumbleStop();
}

uint8_t rumbleSend(uint8_t sendByte) {
	uint16_t send;
	uint16_t parity = calculateOddParity(sendByte);
	send = sendByte;
	send |= (parity << 8);
	send <<= 1;
	send |= 1;//Add start bit
	rumbleTx(send);
}

void receiveCallback(uint16_t rxByte) {
	uint8_t par = calculateOddParity(rxByte & 0xFF);
	if(par == (rxByte & 0x0100)) {
		rumbleAck();
		callBackByte(rxByte & 0xFF);
	}
}

uint8_t calculateOddParity(uint8_t calcPar) {
	uint8_t count = 0;
	for(uint8_t i; i < 8; i++) {
		if(calcPar & 0x01) {
			count++;
		}
		calcPar >>= 1;
	}
	
	return !(count%2);
}

