/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */

#include "main.h"
//Interrupt service routine called by timer interrupt at frequency SAMPLE_RATE_HZ
void TC0_Handler(void)
{
	if(tc_get_status(TC0,CHANNEL0)){
		if(RX){
			if(message_done){
				NVIC_DisableIRQ(TC0_IRQn);
				//Josh callback
			}
			
			readRegister(Z_HIGH, &rx_data, 2);
			val = (q31_t)((rx_data[0]<<8)+rx_data[1]); //high << 8 + low
			val = (val << 30-READ_RES);
			
			shiftLeft();
			samples_fix[FFT_SIZE*2-2] = val;
			samples_fix[FFT_SIZE*2-1] = (q31_t)0;
			copyArray();
			
			fft_exe();
			
			wait_counter++;
			
		}
		if(TX){
			HIGH_COUNTER++;
			if(HIGH_COUNTER > HIGH_DIV) {
				HIGH_COUNTER = 0;
				pio_set(PIOB, PIO_PB12);				if(message_to_send){
					on_off_key();
				}
				else{
					pio_clear(PIOA, PIO_PA31);//LRA
					//delay_ms(1000);
					//pio_set(PIOA, PIO_PA31);//LRA
					//message_to_send = 1;
					//message_counter = 0;
				}
			}//End if HIGH_COUNTER
		}
		NVIC_ClearPendingIRQ(TC0_IRQn);
	}//End get status
}//End handler

int main (void)
{
	//system_clock_begin();
	sysclk_init();
	board_init();
	
	// set the pins to use the uart peripheral
	pmc_enable_periph_clk(ID_UART0);
	pmc_enable_periph_clk(ID_PIOA);

	pio_configure(PINS_UART0_PIO, PINS_UART0_TYPE, PINS_UART0_MASK, PINS_UART0_ATTR);
	pio_configure(PINS_PWM0_PIO, PINS_PWM0_TYPE, PINS_PWM0_MASK, PINS_PWM0_ATTR);

	//enable the uart peripherial clock
	pmc_enable_periph_clk(ID_UART0);
	pmc_enable_periph_clk(PIOB); 

	const sam_uart_opt_t uart1_settings =
	{ 120000000, UART_SERIAL_BAUDRATE, UART_SERIAL_MODE };

	uart_init(UART0,&uart1_settings);      //Init UART1 and enable Rx and Tx
	 
	spi_init();
	
	//Initialize FFT Sampling
 	int status = 1;
	status = arm_cfft_radix4_init_q31(&fft_inst_fix, FFT_SIZE, 0, 1); //Selects forward transform and bit reversal of output
 	if (status == ARM_MATH_ARGUMENT_ERROR){
		console_write(ERROR_MESSAGE, sizeof(ERROR_MESSAGE));
 	}
	 else if(status == ARM_MATH_SUCCESS) {
		console_write(INIT_SUCCESS, sizeof(INIT_SUCCESS));
	 }
	
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO10;
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO11;
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO12;
	
	//Disable watch dog
	WDT->WDT_MR |= WDT_MR_WDDIS; 
	
	//gpio_configure_pin(PIO_PB10_IDX, PIO_PERIPH_B|PIO_DEFAULT);
	pio_set_output(PIOB, PIO_PB10, HIGH, DISABLE, ENABLE);
	pio_set_output(PIOB, PIO_PB11, HIGH, DISABLE, ENABLE);
	pio_set_output(PIOB, PIO_PB12, HIGH, DISABLE, ENABLE);
	pio_set_output(PIOA, PIO_PA30, LOW, DISABLE, ENABLE);
	pio_set_output(PIOA, PIO_PA31, LOW, DISABLE, ENABLE);
	pio_set_input(PIOB, PIO_PB1, PIO_PULLUP);
	
	//Initizlize PWM
	pwm_init_start();
	
	//Initialzie Timer Interrupt
	tc_config(SAMPLE_RATE_HZ, TC0, CHANNEL0, ID_TC0);
	//tc_config(PULSE_RATE_HZ, TC0, CHANNEL1, ID_TC0);
	
	if(TX) {
		pio_clear(PIOA, PIO_PA31);//LRA
		delay_ms(1000);
		pio_set(PIOA, PIO_PA31);//LRA
	}
	//delay_ms(100);

	if(RX) {
		pio_clear(PIOA, PIO_PA31);//LRA
		//delay_ms(1000);
	}
	
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn,0);
 	NVIC_EnableIRQ(TC0_IRQn);
	 
	while(1) {
		
		gpio_set_pin_high(PIO_PB10_IDX);

		

		
	}//End while
}//End main

void console_write(char* input, int len){
	for(int i = 0; i < len; i++) {
		while(!uart_is_tx_buf_empty(UART0));
		uart_write(UART0, input[i]);
	}
	return;
}

void spi_init(void) {
	//Initialize SPI
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

void toggle(uint32_t LED_INPUT){
	if(TOGGLE_S){
		pio_set(PIOB, LED_INPUT);
		TOGGLE_S = 0;
	}
	else{
		pio_clear(PIOB, LED_INPUT);
		TOGGLE_S = 1;
	}//End else
	
}

void toggle_motor(uint32_t MOTOR_INPUT){
	if(TOGGLE_M){
		pio_set(PIOA, MOTOR_INPUT);
		TOGGLE_M = 0;
	}
	else{
		pio_clear(PIOA, MOTOR_INPUT);
		TOGGLE_M = 1;
	}//End else
	

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
	if(message[message_counter]){
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

void fft_exe(void){
	//Disable timer interrupt so that the FFT can complete before being interrupted
	NVIC_DisableIRQ(TC0_IRQn);
		
	//Perform the FFT transform
	arm_cfft_radix4_q31(&fft_inst_fix, samples_hold);
//  		len = sprintf(buffer, "SAM: %d \t %d \r\n", samples_hold[12], samples_hold[13]);
//  		console_write(buffer, len);
			 
	// Calculate magnitude of complex numbers outq31put by the FFT.
 	arm_cmplx_mag_q31(&samples_hold[2*BOTTOM_BIN], magnitudes_fix, 1);
	holder[global_counter++] = *magnitudes_fix;
	 
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
			ring_buffer[head++] = *magnitudes_fix;
			if(head == tail){
				head = 0;
				arm_mean_q31(ring_buffer, BAUD_LEN_SAMPLES, &started_mean);
				if (started_mean > THRESHOLD){
					start_edge_detected = 1;
				}
			}//End if head == tail
		}//If start_edge_detected
		
		if(start_edge_detected){
				ring_buffer[head++] = *magnitudes_fix;
				if(head == tail){
					head = 0;
					moving_average();
				}//End if head == tail
			}//If start_edge_detected
		}//End else
					
		//Uncomment to output accelerometer data
		//sprintf(buffer, "Z_HIGH: %x\tZ_LOW: %x\tZ_VAL: %d \r\n", rx_data[0], rx_data[1], total);
		//console_write(buffer, sizeof(buffer));
			
		if(message_done) {
		
			console_write("\r\n", 2);
			console_write("\r\n", 2);
		
			len = sprintf(buffer, "HIGH_LEVEL: %d \r\n", high_mean);
			console_write(buffer, len);
		
			len = sprintf(buffer, "LOW_LEVEL: %d \r\n", BIT_LOW_LEVEL);
			console_write(buffer, len);
			
			
			len = sprintf(buffer, "VARY: %d \r\n", BIT_VARY);
			console_write(buffer, len);
			
			
			for(int i = 0; i< MESSAGE_LEN; i++) {
				sprintf(buffer, "%d, ", i);
				console_write(buffer, 2);
			}
			console_write("\r\n", 2);
			for(int i = 0; i< MESSAGE_LEN; i++) {
				sprintf(buffer, "%d, ", message_trans[i]);
				console_write(buffer, 2);
			}
			console_write("\r\n", 2);
		
			for(int i = 0; i< MESSAGE_LEN; i++) {
				sprintf(buffer, "%d, ", message[i]);
				console_write(buffer, 2);
			}
			console_write("\r\n", 2);
			//pio_set(PIOA, PIO_PA31);//LRA
			//delay_ms(1000);
			//pio_clear(PIOA, PIO_PA31);//LRA
			//NVIC_DisableIRQ(TC0_IRQn);
			for(int i = 0; i < 4000; i++){
				len = sprintf(buffer, "%d \r\n ", holder[i]);
				console_write(buffer, len);
			}
			NVIC_DisableIRQ(TC0_IRQn);
			return;
		}		
			
		//}*/
		if(global_counter >= 4000){
			for(int i = 0; i < 4000; i++){
				len = sprintf(buffer, "%d \r\n ", holder[i]);
				console_write(buffer, len);
			}
			NVIC_DisableIRQ(TC0_IRQn);
			return;
		}//End if global counter
		
		if(!message_done ) {
			NVIC_EnableIRQ(TC0_IRQn);
			return;
		}
}//End moving average

void moving_average(void){
	
	int sum;
	
	arm_mean_q31(ring_buffer, BAUD_LEN_SAMPLES, &sum_mean);
	summer[message_counter] = sum_mean;
	
	len = sprintf(buffer, "sum[%d] = %d \r\n ", message_counter, sum_mean);
	console_write(buffer, len);
	//if (sum > 2500000)
	if (sum_mean > high_mean/2)
	{
		message_trans[message_counter] = 1;
		//len = sprintf(buffer, "sum[%d] = %d \r\n ", message_trans);
		//console_write(buffer, len);
		/*if((summer[message_counter-1] - sum_mean) > BIT_VARY/5){
				len = sprintf(buffer, "overrdiing to zero \r\n ");
				console_write(buffer, len);
			
				message_trans[message_counter] = 0;
		}*/
	}
	else {
		message_trans[message_counter] = 0;
		/*if((sum_mean -summer[message_counter-1]) > BIT_VARY/5) {
			message_trans[message_counter] = 1;
				len = sprintf(buffer, "overrdiing to one \r\n ");
				console_write(buffer, len);
		}*/
	}
	
	message_counter++;
	if (message_counter > MESSAGE_LEN) {
		len = sprintf(buffer, "MESSAGE DONE @ %d\r\n ", message_counter);
		console_write(buffer, len);
		message_done = 1; 
		NVIC_DisableIRQ(TC0_IRQn);
	}
	//message_trans[0] = 1;
	return;
}//End moving average

void system_clock_begin(void){
	//Disable watch dog
	WDT->WDT_MR |= WDT_MR_WDDIS;
	
	//initialize flash to be 5 wait states
	EFC0->EEFC_FMR = EEFC_FMR_FWS(5) | EEFC_FMR_CLOE;
	
	//enable cache and configure to count instruction hits
	//CMCC->CMCC_CTRL |= 1;
	//CMCC->CMCC_MCFG = 0;
	//CMCC->CMCC_MEN |= 1;
	
	//sysclk_init();
	//disable write protection of PMC registers
	PMC->PMC_WPMR = (0x504d43 << 8) | 1;
	PMC->PMC_WPMR = (0x504d43 << 8) | 0;
	PMC->PMC_WPMR &= (0x504d43 << 8);
	
	//use the 12 mhz rc oscillator
	//switch over to the slow RC clock
	//PMC->PMC_MCKR &= ~0x03;
	//while(!(PMC->PMC_SR & PMC_SR_MCKRDY));
	
	//enable the fast crystal oscillator and wait for it to stabiliz
	//PMC->CKGR_MOR = ((0x37 << 16) | (0x40 << 8) | 1);
	//while(!(PMC->PMC_SR & (1 << 0)));
	
	//change the rc oscillator to 12 mhz
	//PMC->CKGR_MOR = (PMC->CKGR_MOR & ~(0x07 << 4)) | (0x02 << 4) | (0x40 << 8) | (0x37 << 16);
	//while(!(PMC->PMC_SR & (1 << 17)));
	
	//select the crystal oscillator as the main clock source
	//PMC->CKGR_MOR |= ((0x37 << 16) | (1 << 24));
	//while(!(PMC->PMC_SR & (1 << 16)));
	
	//calc the main clock frequency
	PMC->CKGR_MCFR |= (1 << 20);
	volatile uint32_t xtal_meas;
	do 
	{
		xtal_meas = PMC->CKGR_MCFR & (0xffff);
	}while(!(PMC->CKGR_MCFR & (1 << 16)));
	
	
	while(!(PMC->PMC_SR & PMC_SR_MOSCRCS));
	
	//enable the plla
	PMC->PMC_MCKR |= PMC_MCKR_PLLADIV2;
	PMC->CKGR_PLLAR = (1 << 29) | (60 << 16) | (0x3f << 8) | 1;
	while(!(PMC->PMC_SR & PMC_SR_LOCKA));
	volatile int bar;
	
	//switch to plla
	PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(0x07 << 4)) | (0x00 << 4);
	while(!(PMC->PMC_SR & PMC_SR_MCKRDY));
	PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(0x03)) | 0x02;
	while(!(PMC->PMC_SR & PMC_SR_MCKRDY));
}