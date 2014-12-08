#ifndef RUMBLECATION_H_
#define RUMBLECATION_H_

#include <asf.h>

void rumblecationInit(void);
void rumbleListen(void (*returnByte)(uint8_t));
void rumbleStop(void);

uint8_t rumbleSend(uint8_t);



#endif /* RUMBLECATION_H_ */