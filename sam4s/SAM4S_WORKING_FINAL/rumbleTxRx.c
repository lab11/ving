#include "rumbleTxRx.h"
#include "asf.h"
#include "spi.h"
#include "spi_master.h"
#include "arm_math.h"
#include "pio.h"
#include "tc.h"

int RX = 0;
int TX = 0;

//Rate of acceleromter sampling
#define SAMPLE_RATE_HZ 2000
//Rate of motor pulse
#define PULSE_RATE_HZ 20
//Size of FFT
#define FFT_SIZE 64
//Set the bins to print
#define BOTTOM_BIN 6
#define THRESHOLD 3000000
#define BAUD_LEN_SAMPLES (SAMPLE_RATE_HZ/PULSE_RATE_HZ)
#define BIN_SIZE (SAMPLE_RATE_HZ/FFT_SIZE)

q31_t holder[2000];
q31_t samp_holder[4000];
int counting = 0;
int global_counter = 0;

//Message
#define MESSAGE_LEN 16
//const int message[MESSAGE_LEN] = { 1, 0, 1, 0, 1, 0, 1, 0 };
//const int message[MESSAGE_LEN] = { 1, 1, 1, 0, 1, 0, 1, 0,  1, 0, 1, 0, 1, 0, 1, 0 };
uint16_t message = 0xF0F0;
//const int message[3] = {0, 1, 0};
int message_counter = 0;

int ack_counter = 0;
int ack_len = 2;
const int ack_message[2] = {1, 0};
int ack_done = 0;
int acking = 0;

int message_done = 0;
int sample_counter = 0;
int samples_done = 0;

int message_to_send = 1;
int transmitting = 0;

int wait_counter = 0;
q31_t wait_buffer[10] = {0};
int wait_index = 0;
q31_t wait_mean = 0;
q31_t started_mean = 0;

q31_t ring_buffer[BAUD_LEN_SAMPLES];
int head = 0;
int tail = BAUD_LEN_SAMPLES;

uint16_t message_trans = 0;
int summer[MESSAGE_LEN] = {0};

q31_t zero_buffer[10] = {0};
int zero_counter = 0;

q31_t start_edge_buffer[10];
int start_edge_head = 0;
int start_edge_tail = 10;
int start_edge_detected = 0;

int BIT_HIGH_LEVEL = 0;
int BIT_LOW_LEVEL = 0;
int BIT_VARY = 0;
int MESSAGE_START_INDEX = 0;
int start_not_found = 1;

q31_t low_mean = 0;
q31_t high_mean = 0;
q31_t sum_mean = 0;

char write[255];
int n = 0;

//#define BAUD_LEN_SAMPLES 250
#define READ_RES 16

//Channel definitions
#define CHANNEL0 0
#define CHANNEL1 1

int HIGH_COUNTER = 0;
int HIGH_DIV = (SAMPLE_RATE_HZ/PULSE_RATE_HZ);
int TOTAL_COUNTER = 0;

status_code_t status;

#define ERROR_MESSAGE "INIT ERROR \r\n"
#define INIT_SUCCESS "INIT SUCCESS \r\n"
#define BREAKER "\r\n--------------------------------------\r\n"

//=================SPI==================
#define SPI_BAUD 4000000
#define Z_LOW 64
#define Z_HIGH 63
#define PINS_SPI          (PIO_PA12A_MISO | PIO_PA13A_MOSI|PIO_PA11A_NPCS0 |PIO_PA14A_SPCK)
#define PINS_SPI_FLAGS    (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_SPI_MASK     (PIO_PA12A_MISO | PIO_PA13A_MOSI| PIO_PA11A_NPCS0 | PIO_PA14A_SPCK)
#define PINS_SPI_PIO      PIOA
#define PINS_SPI_ID       ID_PIOA
#define PINS_SPI_TYPE     PIO_PERIPH_A
#define PINS_SPI_ATTR     PIO_DEFAULT
struct spi_device CHANNEL_0 = {.id = 0};
uint32_t SPIselect = 1;
char buffer[255];
char print_buf[255];
uint8_t rx_data[2];


//=================PWM============================
//PWM, pa0 LRA, pa1 ERM PWM
#define ERM_on_off PIO_PA30
#define LRA_on_off PIO_PA31

pwm_channel_t g_pwm_channel_led;

#define PINS_PWM0          (PIO_PA0A_PWMH0)
#define PINS_PWM0_FLAGS    (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_PWM0_MASK     (PIO_PA0A_PWMH0)
#define PINS_PWM0_PIO      PIOA
#define PINS_PWM0_ID       ID_PIOA
#define PINS_PWM0_TYPE     PIO_PERIPH_A
#define PINS_PWM0_ATTR     PIO_DEFAULT

/** PWM frequency in Hz */
#define PWM_FREQUENCY      50000
/** Period value of PWM output waveform */
#define PERIOD_VALUE      100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    30 //40 is off, 30, is slightly on, 0 is full on


//=================FFT Globals================


q31_t samples_fix[FFT_SIZE*2] = {0};
q31_t samples_hold[FFT_SIZE*2] = {0};
q31_t magnitudes_fix[FFT_SIZE] = {0};
q31_t max_finder[1] = {0};

int full = 0;


volatile uint8_t high = 0;
volatile uint8_t low = 0;
q31_t val = 0;
q31_t max_val = 0;
uint32_t max_index = 0;
volatile int dom_freq_h = 0;
volatile int dom_freq_l = 0;
const arm_cfft_radix4_instance_q31 fft_inst_fix;
volatile int hasSampled = false;
volatile int isDone = false;
int sampleCounter = 0;
void (*byte_return)(uint16_t);
int len = 0;
int startup = 0;
int zero_level_est = 0;

void rumbleInit(void){
	//Initialize FFT Sampling
	int status = 1;
	status = arm_cfft_radix4_init_q31(&fft_inst_fix, FFT_SIZE, 0, 1); //Selects forward transform and bit reversal of output
	if (status == ARM_MATH_ARGUMENT_ERROR){
		//console_write(ERROR_MESSAGE, sizeof(ERROR_MESSAGE));
	}
	
	else if(status == ARM_MATH_SUCCESS) {
		//console_write(INIT_SUCCESS, sizeof(INIT_SUCCESS));
	}
	
	pmc_enable_periph_clk(ID_PIOA);
	pio_configure(PINS_PWM0_PIO, PINS_PWM0_TYPE, PINS_PWM0_MASK, PINS_PWM0_ATTR);
	pio_set_output(PIOA, PIO_PA30, LOW, DISABLE, ENABLE);
	pio_set_output(PIOA, PIO_PA31, LOW, DISABLE, ENABLE);
	
	//Initizlize PWM
	pwm_init_start();
	
	//Init SPI
	spi_init();
	pio_clear(PIOA, PIO_PA31);//LRA
	
	//Set up Interrupt
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn,0);
}

void rumbleRx(void (*returnByte)(uint16_t)) {
	TX = 0;
	RX = 1;
	message_trans = 0;	
	pio_clear(PIOA, PIO_PA31);//LRA
	tc_config(SAMPLE_RATE_HZ, TC0, CHANNEL0, ID_TC0);
	//enable interrupt
	byte_return = returnByte;
 	NVIC_EnableIRQ(TC0_IRQn);
}

void rumbleTx(uint16_t data) {
	TX = 1;
	RX = 0;
	if(!transmitting) {
		message = data;
}
	pio_set(PIOA, PIO_PA31);//LRA
	g_pwm_channel_led.channel = PWM_CHANNEL_0;
	pwm_channel_update_duty(PWM, &g_pwm_channel_led, 50);
	//Initialzie Timer Interrupt
	tc_config(SAMPLE_RATE_HZ, TC0, CHANNEL0, ID_TC0);
	//enable interrupt
 	NVIC_EnableIRQ(TC0_IRQn);
}
	

void rumbleAck(void) {
	acking = 1;
	NVIC_EnableIRQ(TC0_IRQn);
	//pulses high for 1
}

void rumbleStop(void){
	NVIC_DisableIRQ(TC0_IRQn);
	//disable interrupt
}

void TC0_Handler(void){
	if(acking){
		if(ack_counter > HIGH_DIV) {
			ack_counter = 0;			if(!ack_done){
				ack();
			}
			else{
				NVIC_DisableIRQ(TC0_IRQn);
				pio_clear(PIOA, PIO_PA31);//LRA
				return;
			}
		}
	}
	else {
		if(tc_get_status(TC0,CHANNEL0)){
			if(RX){
				if(message_done){
					NVIC_DisableIRQ(TC0_IRQn);
					clear_Rx();
					byte_return(message_trans);
					NVIC_EnableIRQ(TC0_IRQn);
					//return state
				}
			
				readRegister(Z_HIGH, &rx_data, 2);
				val = (q31_t)((rx_data[0]<<8)+rx_data[1]); //high << 8 + low
				val = (val << 30-READ_RES);
			
				shiftLeft();
				samples_fix[FFT_SIZE*2-2] = val;
				samples_fix[FFT_SIZE*2-1] = (q31_t)0;
				copyArray();
			
				wait_counter++;
				full+=2;
				if (full>FFT_SIZE*2){
					fft_exe();
				}
			}
			if(TX){
				HIGH_COUNTER++;
				if(HIGH_COUNTER > HIGH_DIV) {
					HIGH_COUNTER = 0;
					pio_set(PIOB, PIO_PB12);					if(message_to_send){
						transmitting = 1;
						on_off_key();
					}
					else{
						
						pio_clear(PIOA, PIO_PA31);//LRA
						HIGH_COUNTER = 0;
						transmitting = 0;
						message_to_send = 1;
						message_counter = 0;
						NVIC_DisableIRQ(TC0_IRQn);
						//delay_ms(1000);
						//pio_set(PIOA, PIO_PA31);//LRA
						//message_to_send = 1;
						//message_counter = 0;
					}
				}//End if HIGH_COUNTER
			}
		}
		NVIC_ClearPendingIRQ(TC0_IRQn);
	}//End get status
}

void spi_init(void) {
	//Initialize SPI
	pmc_enable_periph_clk(PIOB);
	pmc_enable_periph_clk(ID_SPI);
	pio_configure(PINS_SPI_PIO, PINS_SPI_TYPE, PINS_SPI_MASK, PINS_SPI_ATTR);

	//Choose appropriate SPI_MODE_n
	// 	Mode CPOL NCPHA SHIFTEDGE CAPTUREEDGE INACTIVELEVEL
	// 	0 0 1 Falling Rising Low
	// 	1 0 0 Rising Falling Low
	// 	2 1 1 Rising Falling High
	// 	3 1 0 Falling Rising High

	spi_master_init(SPI);
	spi_master_setup_device(SPI, &CHANNEL_0, SPI_MODE_3, SPI_BAUD, SPIselect); //mySPIselect is not used
	spi_enable(SPI);

	writeRegister(26, 0x00); //Config reg
	writeRegister(28, 0x00); //Accel config
	writeRegister(29, 0x08); //Config2
	writeRegister(35, 0x00); //FIFO enable 0x08
	writeRegister(106, 0x50); //User control 0101 0000
	writeRegister(107, 0x00);//Power managment 0000 0000
	//writeRegister(108, 0x1F); //Disable all axis and gryo except ACC_X 0001 1111
	writeRegister(108, 0x00); //Enable all axis and gryo
}

void writeRegister(uint8_t reg, uint8_t val) {
	//uint8_t data_Tx = reg;
	spi_select_device(SPI, &CHANNEL_0);
	uint8_t data_Tx[2] = {reg, val};
	spi_write_packet(SPI, data_Tx, 2);
	//spi_write_single(SPI, reg);
	//spi_write_single(SPI, val);
	spi_deselect_device(SPI, &CHANNEL_0);
	return status;
}

status_code_t readRegister(uint8_t reg, uint8_t* val, int bytesToRead){
	uint8_t inByte = 0;
	uint8_t REG_ADDR_R = reg | 0x80;

	spi_select_device(SPI, &CHANNEL_0);
	//spi_write_single(SPI, REG_ADDR_R);
	status = spi_read_packet(SPI, val, bytesToRead);
	spi_deselect_device(SPI, &CHANNEL_0);
	return status;
}


// Configuration function for the Timer.
void tc_config(uint32_t freq_desired, Tc* TCn, uint32_t CHANNEL, uint32_t PER_ID)
{
	// INPUTS:
	//	freq_desired	The desired rate at which to call the ISR, Hz.
	
	uint32_t ul_div;
	uint32_t ul_tcclks;
	//uint32_t ul_sysclk = sysclk_get_cpu_hz();
	uint32_t ul_sysclk = 120000000;
	uint32_t counts;	
	
	// Configure PMC				
	pmc_enable_periph_clk(PER_ID);

	// Configure TC for a frequency and trigger on RC compare.
	tc_find_mck_divisor(
		(uint32_t)freq_desired,	// The desired frequency as a uint32. 
		ul_sysclk,				// Master clock freq in Hz.
		&ul_div,				// Pointer to register where divisor will be stored.
		&ul_tcclks,				// Pointer to reg where clock selection number is stored.
		ul_sysclk);				// Board clock freq in Hz.
	tc_init(TCn, CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	
	// Find the best estimate of counts, then write it to TC register C.
	counts = (ul_sysclk/ul_div)/freq_desired;	
	tc_write_rc(TCn, 0, counts);						

	// Enable interrupts for this TC, and start the TC.		
	tc_enable_interrupt(TCn, CHANNEL,TC_IER_CPCS);				// Enable interrupt.
	tc_start(TCn,CHANNEL);										// Start the TC.

}

void shiftLeft(void) {
  for(int i = 0; i < FFT_SIZE*2; i++)
  {
    samples_fix[i] = samples_fix[i+2];
  }
}

void copyArray(void) {
  for(int i = 0; i < FFT_SIZE*2; i++) {
    samples_hold[i] = samples_fix[i];
  }
}



void pwm_init_start(void) {
	//pio_set(PIOA, PIO_PA30);//ERM
	pio_set(PIOA, PIO_PA31);//LRA
		//Enable PIO
	pmc_enable_periph_clk(PIOA);
	pmc_enable_periph_clk(ID_PWM);
	
	pwm_channel_disable(PWM, PWM_CHANNEL_0);

	
	pwm_clock_t clock_setting = {
	.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
	.ul_clkb = 0,
	.ul_mck = 120000000//sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &clock_setting);
	
	
	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_LOW;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel_led.channel = PWM_CHANNEL_0;
	pwm_channel_init(PWM, &g_pwm_channel_led);
	
	NVIC_DisableIRQ(PWM_IRQn);
	NVIC_ClearPendingIRQ(PWM_IRQn);
	NVIC_SetPriority(PWM_IRQn, 0);
	NVIC_EnableIRQ(PWM_IRQn);

	/* Enable channel counter event interrupt */
	pwm_channel_disable_interrupt(PWM, PWM_CHANNEL_0, 0);
	//pwm_channel_enable_interrupt(PWM, PWM_CHANNEL_0, 0);
	pwm_channel_enable(PWM, PWM_CHANNEL_0);
	
}

void on_off_key(void){
	uint32_t ul_duty;
	if(message & (1 << message_counter)){
		ul_duty = 20;
		message_counter++; 
	}
	else{
		ul_duty = 50;
		message_counter++;
	}//End else
	
	if(message_counter > MESSAGE_LEN) {
		//message_counter = 0;
		message_to_send = 0;
		//pio_clear(PIOA, PIO_PA31);//LRA
	}
	
	g_pwm_channel_led.channel = PWM_CHANNEL_0;
	pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty);
	
}

void ack(void){
	pio_set(PIOA, PIO_PA31);//LRA
	
	uint32_t ul_duty;
	if(ack_message[ack_counter]){
		ul_duty = 20;
		ack_counter++;
	}
	else{
		ul_duty = 50;
		ack_counter++;
	}//End else
	
	if(ack_counter > ack_len) {
		//message_counter = 0;
		ack_done = 0;
		//pio_clear(PIOA, PIO_PA31);//LRA
	}
	
	g_pwm_channel_led.channel = PWM_CHANNEL_0;
	pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty);
	
}

void console_write(char* input, int len){
	for(int i = 0; i < len; i++) {
		while(!uart_is_tx_buf_empty(UART0));
		uart_write(UART0, input[i]);
	}
	return;
}


void fft_exe(void){
	//Disable timer interrupt so that the FFT can complete before being interrupted
	NVIC_DisableIRQ(TC0_IRQn);
		
	//Perform the FFT transform
	arm_cfft_radix4_q31(&fft_inst_fix, samples_hold);
//  		len = sprintf(buffer, "SAM: %d \t %d \r\n", samples_hold[12], samples_hold[13]);
//  		console_write(buffer, len);
			 
	// Calculate magnitude of complex numbers outq31put by the FFT.
 	arm_cmplx_mag_q31(&samples_hold[2*BOTTOM_BIN], magnitudes_fix, 1);

	 
//  	if(!start_edge_detected){
// 		// len = sprintf(buffer, "%d \r\n", *magnitudes_fix);
// 		//console_write(buffer, len);
// 	 }
	 
	 
// 		if(!zero_level_est) {
// 			wait_buffer[wait_index++] = *magnitudes_fix;
// 			arm_mean_q31(wait_buffer, sizeof(wait_buffer), &wait_mean);
// 			if(wait_mean > THRESHOLD) {
// 				 len = sprintf(buffer, "Waiting for zero \r\n", *magnitudes_fix);
// 				//console_write(buffer, len);
// 			}
// 			else{
// 				zero_buffer[zero_counter++] = *magnitudes_fix;
// 				if(zero_counter > 10) {
// 					arm_mean_q31(zero_buffer, sizeof(zero_buffer), &low_mean);
// 					//BIT_LOW_LEVEL = low_mean;
// 					BIT_LOW_LEVEL = 0;
// 					len = sprintf(buffer, "LOW_LEVEL_EST: %d", BIT_LOW_LEVEL);
// 					console_write(buffer, len);
// 					zero_level_est = 1;
// 				}//End if > 10
// 			}
// 		}//End if zero_level_est
// 		
// 		else {
// 			if(!start_edge_detected){
// 				start_edge_buffer[start_edge_head++] = *magnitudes_fix;
// 				arm_mean_q31(start_edge_buffer, sizeof(start_edge_buffer), &high_mean);
// 				if(high_mean > THRESHOLD) {
// 					start_edge_detected = 1;
// 					BIT_HIGH_LEVEL = high_mean;
// 					BIT_VARY = BIT_HIGH_LEVEL - BIT_LOW_LEVEL;
// 					len = sprintf(buffer, "HIGH_LEVEL_EST: %d \r\n", BIT_HIGH_LEVEL);
// 					console_write(buffer, len);
// 				}//End if mean > threshold
// 				if(start_edge_head == start_edge_tail){
// 					start_edge_head = 0;
// 				}//End if starge_edge_head
// 			}//End if start_edge_detected
// 			
// 			if(start_edge_detected){
// 				ring_buffer[head++] = *magnitudes_fix;
// 				if(head == tail){
// 					head = 0;
// 					moving_average();
// 				}//End if head == tail
// 			}//If start_edge_detected
// 		}//End else

		if(!start_edge_detected){
			tail = BAUD_LEN_SAMPLES/10;
			ring_buffer[head++] = *magnitudes_fix;
			if(head == tail){
				head = 0;
				wait_counter = 0;

				arm_mean_q31(ring_buffer, BAUD_LEN_SAMPLES/10, &started_mean);
				if (started_mean > THRESHOLD){
					//len = sprintf(buffer, "---------------------------------\r\n");
					//console_write(buffer, len);
					start_edge_detected = 1;
					BIT_HIGH_LEVEL = started_mean;
					BIT_LOW_LEVEL = 0;
					BIT_VARY = BIT_HIGH_LEVEL- BIT_LOW_LEVEL;
					head = 0;
					//len = sprintf(buffer, "H: %d", started_mean);
					//console_write(buffer, len);
				}
			}//End if head == tail
		}//If start_edge_detected
		
		if(start_edge_detected){
			//if(wait_counter > (BAUD_LEN_SAMPLES-(BAUD_LEN_SAMPLES/10))-30){
			if(wait_counter > 90){
				tail = BAUD_LEN_SAMPLES;
				ring_buffer[head++] = *magnitudes_fix;
				holder[global_counter++] = *magnitudes_fix;
				if(head == tail){
					head = 0;
					moving_average();
				}//End if head == tail
			}//End if wait counter
		}//End else
					
		//Uncomment to output accelerometer data
		//sprintf(buffer, "Z_HIGH: %x\tZ_LOW: %x\tZ_VAL: %d \r\n", rx_data[0], rx_data[1], total);
		//console_write(buffer, sizeof(buffer));
			
		if(message_done) {
			
			for(int i = 0; i < 1000; i++){
				len = sprintf(buffer, "%d \r\n ", holder[i]);
				console_write(buffer, len);
			}
		
			len = sprintf(buffer, "HIGH_LEVEL: %d \r\n", started_mean);
			console_write(buffer, len);
		
			len = sprintf(buffer, "LOW_LEVEL: %d \r\n", BIT_LOW_LEVEL);
			console_write(buffer, len);
			
			
			len = sprintf(buffer, "VARY: %d \r\n", BIT_VARY);
			console_write(buffer, len);
			
			
			for(int i = 0; i< 9; i++) {
				sprintf(buffer, "%d, ", i);
				console_write(buffer, 2);
			}
			console_write("\r\n", 2);
			for(int i = 0; i< 9; i++) {
				sprintf(buffer, "%d, ", (message_trans >> i)&1);
				console_write(buffer, 2);
			}
			console_write("\r\n", 2);
			len = sprintf(buffer, "CHAR: %c \r\n", message_trans);
			console_write(buffer, len); 
			console_write("\r\n", 2);
		
			for(int i = 0; i <9; i++){
				len = sprintf(buffer, "sum[%d]: %d \r\n", i, summer[i]);
				console_write(buffer, len);			}
			//for(int i = 0; i< MESSAGE_LEN; i++) {
			//	sprintf(buffer, "%d, ", message & (1<<i));
			//	console_write(buffer, 2);
			//}
			console_write("\r\n", 2);
			//pio_set(PIOA, PIO_PA31);//LRA
			//delay_ms(1000);
			//pio_clear(PIOA, PIO_PA31);//LRA
			//NVIC_DisableIRQ(TC0_IRQn);

			
			NVIC_DisableIRQ(TC0_IRQn);
			return;
		}
		
		if(global_counter >= 1000){
			for(int i = 0; i < 400; i++){
				len = sprintf(buffer, "%d \r\n ", holder[i]);
				console_write(buffer, len);
			}
			NVIC_DisableIRQ(TC0_IRQn);
			return;
		}//End if global counter
		
		
		NVIC_EnableIRQ(TC0_IRQn);
		
// 		if(!message_done) {
// 			NVIC_EnableIRQ(TC0_IRQn);
// 			return;
// 		}
		
		
}//End of fft_exe

void moving_average(void){
	
	int sum;
	
	arm_mean_q31(ring_buffer, BAUD_LEN_SAMPLES, &sum_mean);
	summer[message_counter] = sum_mean;
	//sum_mean >>= 3;
	
	//len = sprintf(buffer, "sum[%d] = %d \r\n ", message_counter, sum_mean);
	//console_write(buffer, len);
	//if (sum > 2500000)
	if (sum_mean > started_mean>>1) {
		message_trans |= (1 << message_counter);
		//len = sprintf(buffer, "sum[%d] = %d \r\n ", message_trans);
		//console_write(buffer, len);
		/*if((summer[message_counter-1] - sum_mean) > BIT_VARY/5){			
				message_trans &= ~(1 << message_counter);
		}*/
	}
	else {
		//message_trans &= ~(1 << message_counter);
		/*if((sum_mean -summer[message_counter-1]) > BIT_VARY/5) {
			message_trans |= (1 << message_counter);
		}*/
	}
	
	message_counter++;
	if (message_counter > 9) {
		//len = sprintf(buffer, "MESSAGE DONE @ %d\r\n ", message_counter);
		//console_write(buffer, len);
		message_done = 1; 
		NVIC_DisableIRQ(TC0_IRQn);
	}
	//message_trans[0] = 1;
}

void clear_Rx(void) {
	global_counter = 0;
	message_counter = 0;
	ack_counter = 0;
	ack_len = 2;
	ack_done = 0;
	acking = 0;
	message_done = 0;
	sample_counter = 0;
	samples_done = 0;
	wait_counter = 0;
	wait_index = 0;
	wait_mean = 0;
	started_mean = 0;
	head = 0;
	message_trans = 0;
	zero_counter = 0;
	start_edge_head = 0;
	start_edge_tail = 10;
	start_edge_detected = 0;
	BIT_HIGH_LEVEL = 0;
	BIT_LOW_LEVEL = 0;
	BIT_VARY = 0;
	MESSAGE_START_INDEX = 0;
	start_not_found = 1;
	low_mean = 0;
	high_mean = 0;
	sum_mean = 0;
	TOTAL_COUNTER = 0;
	full = 0;
}
