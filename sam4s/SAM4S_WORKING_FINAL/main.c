
#include <asf.h>
#include "rumblecation.h"
#include "rumbleTxRx.h"

#define UART_SERIAL_BAUDRATE        57600
#define UART_SERIAL_CHANNEL_MODE	UART_MR_CHMODE_NORMAL
#define UART_SERIAL_MODE			UART_MR_PAR_NO


#define PINS_UART0          (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)
#define PINS_UART0_FLAGS    (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART0_MASK     (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)
#define PINS_UART0_PIO      PIOA
#define PINS_UART0_ID       ID_PIOA
#define PINS_UART0_TYPE     PIO_PERIPH_A
#define PINS_UART0_ATTR     PIO_DEFAULT


void initLeds(void);
void uartInit(void);
void callback(uint8_t);
void spi_init(void);

uint8_t transmitBuf[256];
uint8_t transmitBufRead = 0;
uint8_t transmitBufWrite = 0;
char buffer[255];
int indexer = 0;


int main (void)
{
	sysclk_init();
	wdt_disable(WDT);
	initLeds();
	rumblecationInit();
	uartInit();
	
	volatile int i;
	delay_ms(3000);
	indexer = sprintf(buffer, "Starting \r\n");
	console_write(buffer, indexer);
	
	
	rumbleListen(callback);
	while(1) {

		//uart_write(UART0, 's');
		//i++;
		/*if(transmitBufRead != transmitBufWrite) {
			uart_write(UART0, transmitBuf[transmitBufRead]);
			transmitBufRead++;
		}*/
// 		if(uart_is_rx_ready(UART0)) {
// 			uint8_t in;
// 			uart_read(UART0, &in);
// 			uart_write(UART0, in);
// 			rumbleSend(in);
// 		}
	}

}




void callback(uint8_t character){
	uart_write(UART0, character);
}

void initLeds(void) {
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO12;
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO11;
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO10;
	
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOA);
	pio_set_output(PIOB, PIO_PB10, LOW, DISABLE, ENABLE);
	pio_set_output(PIOB, PIO_PB11, LOW, DISABLE, ENABLE);
	pio_set_output(PIOB, PIO_PB12, LOW, DISABLE, ENABLE);
}

void uartInit() {
	
	pio_configure(PINS_UART0_PIO, PINS_UART0_TYPE, PINS_UART0_MASK, PINS_UART0_ATTR);
	
	pmc_enable_periph_clk(ID_UART0);

	const sam_uart_opt_t uart1_settings =
	{ 120000000, UART_SERIAL_BAUDRATE, UART_SERIAL_MODE };

	uart_init(UART0,&uart1_settings);
	uart_enable_interrupt(UART0,UART_IER_RXRDY);
	uart_enable_rx(UART0);
}

/*void UART0_Handler(void) {
	uint8_t in;
	uart_read(UART0,&in);
	transmitBuf[transmitBufWrite] = in;
	transmitBufWrite++;
}*/