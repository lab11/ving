
#ifndef RUMBLETXRX_H_
#define RUMBLETXRX_H_

#include <asf.h>


//=================Function Definitions================
void rumbleInit(void);
void rumbleRx(void (*returnByte)(uint16_t));
void rumbleTx(uint16_t);
void rumbleAck(void);
void writeRegister(uint8_t reg, uint8_t val);
status_code_t readRegister(uint8_t reg, uint8_t* val, int bytesToRead);
void spi_init(void);
void console_write(char* input, int len);
void shiftLeft(void);
void copyArray(void);
void toggle(uint32_t LED_INPUT);
void toggle_motor(uint32_t MOTOR_INPUT);
void pwm_init_start(void);
void on_off_key(void);
void tc_config(uint32_t freq_desired, Tc* TCn, uint32_t CHANNEL, uint32_t PER_ID);
void fft_exe(void);
void moving_average(void);
void start_off(void);
void find_first_peak(void);
void system_clock_begin(void);
void ack(void);
void clear_Rx(void);
void console_write(char* input, int len);


//=================Ring Buffer Definitions================
//q31_t ring_buffer[GOAL_SAMPLES];

#endif 