#include <asf.h>
#include "board.h"
#include "gpio.h"
#include "power_clocks_lib.h"
#include "et024006dhu.h"
#include "delay.h"
#include "avr32_logo.h"
#include "conf_clock.h"
#include "time.h"
#include "usart.h"
#include "pdca.h"
#include "spi.h"
#include "print_funcs.h"
#include "conf_sd_mmc_spi.h"
#include "sd_mmc_spi.h"
#include <stdio.h>


#define pdca_usart_ch 2

#define USART_RX_PIN			0
#define USART_RX_FN				0
#define USART_TX_PIN			1
#define USART_TX_FN				0
#define USART_CLOCK_MASK		AVR32_USART0_CLK_PBA
#define TARGET_PBACLK_FREQ_HZ	12000000

#define PBA_HZ                FOSC0
#define BUFFERSIZE            64
#define PDCA_CHANNEL_USED_RX 7  //SPI_RX
#define PDCA_CHANNEL_USED_TX 15 //SPI_TX
#define PDCA_CHANNEL_SPI_RX		0
#define PDCA_CHANNEL_SPI_TX		1

volatile avr32_pdca_channel_t* pdca_channelrx ;
volatile avr32_pdca_channel_t* pdca_channeltx ; 
const char dummy_data[]="xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
volatile char ram_buffer[512];
volatile bool end_of_transfer;

int llegomsj=0;
volatile flag=0;

volatile char mensaje_rx[50];
volatile char ram_buffer_x[50];
volatile char ram_bufferd[50];
volatile char ram_buffer_last_message[50];


#if BOARD == EVK1105
#include "pwm.h"
avr32_pwm_channel_t pwm_channel6 = {
  .cdty = 0,
  .cprd = 100
};

static void tft_bl_init(void)
{

  pwm_opt_t opt = {
    .diva = 0,
    .divb = 0,
    .prea = 0,
    .preb = 0
  };

  pwm_init(&opt);
  pwm_channel6.CMR.calg = PWM_MODE_LEFT_ALIGNED;
  pwm_channel6.CMR.cpol = PWM_POLARITY_HIGH; //PWM_POLARITY_LOW;//PWM_POLARITY_HIGH;
  pwm_channel6.CMR.cpd = PWM_UPDATE_DUTY;
  pwm_channel6.CMR.cpre = AVR32_PWM_CMR_CPRE_MCK_DIV_2;

  pwm_channel_init(6, &pwm_channel6);
  pwm_start_channels(AVR32_PWM_ENA_CHID6_MASK);

}
#endif

__attribute__ ((__interrupt__))
void touch(void){
	if (gpio_get_pin_interrupt_flag (54)==true){//UP
		flag=1;
		gpio_clear_pin_interrupt_flag(54);
	}
	if (gpio_get_pin_interrupt_flag (55)==true){//DOWN
		flag=2;
		gpio_clear_pin_interrupt_flag(55);
	}
	if (gpio_get_pin_interrupt_flag (56)==true){//RIGHT
		flag=3;
		gpio_clear_pin_interrupt_flag(56);
	}
	if (gpio_get_pin_interrupt_flag (57)==true){//LEFT
		flag=4;
		gpio_clear_pin_interrupt_flag(57);
	}
	if (gpio_get_pin_interrupt_flag(58)==true){//CENTER
		flag=5;		
		gpio_clear_pin_interrupt_flag(58);
	}
	
}

void wait()
{
	volatile int i;
	for(i = 0 ; i < 5000; i++);
}

__attribute__ ((__interrupt__))
static void pdca_int_handler (void){
	Disable_global_interrupt();
	 pdca_disable_interrupt_transfer_complete(PDCA_CHANNEL_SPI_RX);// Disable interrupt channel.
	 sd_mmc_spi_read_close_PDCA();//unselects the SD/MMC memory.
	 wait();
	 // Disable unnecessary channel
	 pdca_disable(PDCA_CHANNEL_SPI_TX);
	 pdca_disable(PDCA_CHANNEL_SPI_RX);
	
	 Enable_global_interrupt();
	 end_of_transfer = true;
}

//Inicializa USART
void inicializa_usart(void){
	// USART options.
	static const usart_options_t USART_OPTIONS =
	{
		.baudrate     = 57600,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};
	
	static const gpio_map_t USART_GPIO_MAP =
	{
		{USART_RX_PIN, USART_RX_FN},
		{USART_TX_PIN, USART_TX_FN}
		
	};
	// Assign GPIO to USART.
	gpio_enable_module(USART_GPIO_MAP, sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));
	// Initialize USART in RS232 mode.
	usart_init_rs232(&AVR32_USART0, &USART_OPTIONS, 12000000);
}

//Inicialización de USART por PDCA
void PDCA_init_USART(){
	const pdca_channel_options_t PDCA_OPTIONSUSART = {
		.pid = AVR32_PDCA_PID_USART0_RX, //l}peripheral
		.transfer_size = AVR32_PDCA_BYTE, //byte, half w, w
		.addr = &mensaje_rx, //memory address
		.size = 50, //TRC
		.r_addr = NULL,
		.r_size = 0,
	};

	pdca_init_channel(pdca_usart_ch, &PDCA_OPTIONSUSART);
	pdca_enable(pdca_usart_ch);
}

//Inicialización de SD
static void sd_mmc_resources_init(void)
{
	// GPIO pins used for SD/MMC interface
	static const gpio_map_t SD_MMC_SPI_GPIO_MAP =
	{
		{SD_MMC_SPI_SCK_PIN,  SD_MMC_SPI_SCK_FUNCTION },  // SPI Clock.
		{SD_MMC_SPI_MISO_PIN, SD_MMC_SPI_MISO_FUNCTION},  // MISO.
		{SD_MMC_SPI_MOSI_PIN, SD_MMC_SPI_MOSI_FUNCTION},  // MOSI.
		{SD_MMC_SPI_NPCS_PIN, SD_MMC_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
	};

	// SPI options.
	spi_options_t spiOptions =
	{
		.reg          = SD_MMC_SPI_NPCS,
		.baudrate     = SD_MMC_SPI_MASTER_SPEED,  // Defined in conf_sd_mmc_spi.h.
		.bits         = SD_MMC_SPI_BITS,          // Defined in conf_sd_mmc_spi.h.
		.spck_delay   = 0,
		.trans_delay  = 0,
		.stay_act     = 1,
		.spi_mode     = 0,
		.modfdis      = 1
	};

	// Assign I/Os to SPI.
	gpio_enable_module(SD_MMC_SPI_GPIO_MAP, sizeof(SD_MMC_SPI_GPIO_MAP) / sizeof(SD_MMC_SPI_GPIO_MAP[0]));

	// Initialize as master.
	spi_initMaster(SD_MMC_SPI, &spiOptions);

	// Set SPI selection mode: variable_ps, pcs_decode, delay.
	spi_selectionMode(SD_MMC_SPI, 0, 0, 0);

	// Enable SPI module.
	spi_enable(SD_MMC_SPI);

	// Initialize SD/MMC driver with SPI clock (PBA).
	sd_mmc_spi_init(spiOptions, FOSC0);
}

void local_pdca_init(void)
{
	// this PDCA channel is used for data reception from the SPI
	pdca_channel_options_t pdca_options_SPI_RX ={ // pdca channel options

		.addr = ram_buffer,
		// memory address. We take here the address of the string dummy_data. This string is located in the file dummy.h

		.size = 512,                              // transfer counter: here the size of the string
		.r_addr = NULL,                           // next memory address after 1st transfer complete
		.r_size = 0,                              // next transfer counter not used here
		.pid = PDCA_CHANNEL_USED_RX,        // select peripheral ID - data are on reception from SPI1 RX line
		.transfer_size = PDCA_TRANSFER_SIZE_BYTE  // select size of the transfer: 8,16,32 bits
	};

	// this channel is used to activate the clock of the SPI by sending a dummy variables
	pdca_channel_options_t pdca_options_SPI_TX ={ // pdca channel options

		.addr = (void *)&dummy_data,              // memory address.
		// We take here the address of the string dummy_data.
		// This string is located in the file dummy.h
		.size = 512,                              // transfer counter: here the size of the string
		.r_addr = NULL,                           // next memory address after 1st transfer complete
		.r_size = 0,                              // next transfer counter not used here
		.pid = PDCA_CHANNEL_USED_TX,        // select peripheral ID - data are on reception from SPI1 RX line
		.transfer_size = PDCA_TRANSFER_SIZE_BYTE  // select size of the transfer: 8,16,32 bits
	};

	// Init PDCA transmission channel
	pdca_init_channel(PDCA_CHANNEL_SPI_TX, &pdca_options_SPI_TX);

	// Init PDCA Reception channel
	pdca_init_channel(PDCA_CHANNEL_SPI_RX, &pdca_options_SPI_RX);

	//! \brief Enable pdca transfer interrupt when completed
	INTC_register_interrupt(&pdca_int_handler, AVR32_PDCA_IRQ_0, AVR32_INTC_INT1);  // pdca_channel_spi1_RX = 0

}


int main(void)
{

	pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);
  
	init_dbg_rs232(12000000); // Initialize debug RS232 with PBA clock

	Disable_global_interrupt();
	INTC_init_interrupts();
	INTC_register_interrupt(&touch, 70, 0);
	INTC_register_interrupt(&touch, 71, 0);
	gpio_enable_pin_interrupt(54,GPIO_RISING_EDGE);
	gpio_enable_pin_interrupt(55,GPIO_RISING_EDGE);
	gpio_enable_pin_interrupt(56,GPIO_RISING_EDGE);
	gpio_enable_pin_interrupt(57,GPIO_RISING_EDGE);
	gpio_enable_pin_interrupt(58,GPIO_RISING_EDGE);
	Enable_global_interrupt();
  
	et024006_Init( FOSC0, FOSC0 );
	tft_bl_init();
	et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, WHITE );  
  
	inicializa_usart();
	sd_mmc_resources_init(); // Initialize SD/MMC driver resources: GPIO, SPI and SD/MMC.  
	local_pdca_init();

#if BOARD == EVK1105
  while(pwm_channel6.cdty < pwm_channel6.cprd)
  {
    pwm_channel6.cdty++;
    pwm_channel6.cupd = pwm_channel6.cdty;
    pwm_async_update_channel(AVR32_PWM_ENA_CHID6, &pwm_channel6);
    delay_ms(10);
  }
  
#endif

	int sector=1;
	int mensajeExitoSDMostrado = 0;
	int mensajeErrorSDMostrado = 0;

  while(1){
	  switch (flag){
		  case 0:
			  wait();
			  while (!sd_mmc_spi_mem_check()){
				  
					if (mensajeErrorSDMostrado == 0)
					{
						et024006_DrawFilledRect(120 , 220, 150, 16, WHITE );
						et024006_PrintString("No hay tarjeta SD reconocida", (const unsigned char *)&FONT8x16, 80, 220, RED, -1);
						mensajeErrorSDMostrado = 1;
						mensajeExitoSDMostrado = 0;
					}
				  
			  };
				mensajeErrorSDMostrado = 0;
			  
				if (mensajeExitoSDMostrado == 0)
				{
					et024006_DrawFilledRect(80 , 220, 225, 16, WHITE );
					et024006_PrintString("SD reconocida", (const unsigned char *)&FONT8x16, 120, 220, GREEN, -1);
					mensajeExitoSDMostrado = 1;
				}
			  
			  break;
		  case 1: //UP
			et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, WHITE);
			et024006_PrintString("Recibiendo mensaje...", (const unsigned char *)&FONT8x16, 60, 98, BLUE, -1);
			et024006_PrintString("Pulse DOWN para desplegarlo...", (const unsigned char *)&FONT8x16, 70, 120, BLUE, -1);
			PDCA_init_USART();
			flag=0;
			break;
		  case 2: //DOWN
			et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, WHITE );
			pdca_disable(pdca_usart_ch);
			llegomsj=1;
			for (int i=0 ; i<sizeof(mensaje_rx); i++)
			{
				ram_buffer_x[i]=mensaje_rx[i];
				if(i<37){
					et024006_PrintString(&mensaje_rx[i], (const unsigned char *)&FONT8x16, 10+8*i, 20, BLUE, -1);
					et024006_DrawFilledRect(305 , 0, 15, 40, WHITE );
				}else{
					et024006_PrintString(&mensaje_rx[i], (const unsigned char *)&FONT8x16, (10+8*i-37*8), 38, BLUE, -1);
				}
			}
			
			for (int i=0;i<50;i++)
			{
				mensaje_rx[i]=' ';
			}
			
			flag=0;
		  break;
		  case 3: //RIGHT
			et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, WHITE );
			if (llegomsj==0)
			{
					
				et024006_PrintString("No hay mensaje", (const unsigned char*)&FONT8x16, 30, 80, RED, -1);
				flag=0;
				break;
			}
			et024006_PrintString("Guardando en SD", (const unsigned char *)&FONT8x16, 30, 100, BLUE, -1);
			et024006_PrintString("Sector:", (const unsigned char *)&FONT8x16, 30, 150 , BLUE, -1);
			// Read Card capacity
			int sizesd=capacity>> 20;
			char displ[30];
			char displ1[30];
			sprintf(displ, " Capacidad: %d MBytes ",sizesd);
			et024006_PrintString(displ, (const unsigned char *)&FONT8x16, 80, 40,BLUE, -1);
			if (sector==6)
			{
				sector=1;
			}
			sprintf(displ1, "%u",sector);
			et024006_PrintString(displ1, (const unsigned char *)&FONT8x16, 90, 150,BLUE, -1);
		
			sd_mmc_spi_write_open(sector);
			sd_mmc_spi_write_sector_from_ram(&ram_buffer_x);
			sd_mmc_spi_write_close();
				
			sector++;
			flag=0;
			break;
		  case 4: //LEFT

			et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, WHITE );
			sector--;
			sd_mmc_spi_read_open(sector);
			sd_mmc_spi_read_sector_to_ram(&ram_buffer_last_message);		//BUFFER DE RECEPCION DESDE SDCARD
			sd_mmc_spi_read_close();
			et024006_PrintString (&ram_buffer_last_message, (const unsigned char *)&FONT8x16,  10, 40, BLUE,-1);
			sprintf(displ, "%u",sector);
			et024006_PrintString("Sector:", (const unsigned char *)&FONT8x16, 10, 10, BLUE, -1);
			et024006_PrintString(displ, (const unsigned char *)&FONT8x16, 80, 10,BLUE, -1);
			sector++;
			flag=0;		  
			break;
		  case 5:
				et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, WHITE);
				for(int j=1; j<=5; j++)
				{
					// Configure the PDCA channel: the address of memory ram_buffer to receive the data at sector address j
					pdca_load_channel(PDCA_CHANNEL_SPI_RX,&ram_buffer,512);
					pdca_load_channel(PDCA_CHANNEL_SPI_TX,(void *)&dummy_data,512); //send dummy to activate the clock
					end_of_transfer = false;
					
					// open sector number j
					if(sd_mmc_spi_read_open_PDCA (j))
					{
						spi_write(SD_MMC_SPI,0xFF); // Write a first dummy data to synchronize transfer
						pdca_enable_interrupt_transfer_complete(PDCA_CHANNEL_SPI_RX);
						pdca_channelrx =(volatile avr32_pdca_channel_t*) pdca_get_handler(PDCA_CHANNEL_SPI_RX); // get the correct PDCA channel pointer
						pdca_channeltx =(volatile avr32_pdca_channel_t*) pdca_get_handler(PDCA_CHANNEL_SPI_TX); // get the correct PDCA channel pointer
						pdca_channelrx->cr = AVR32_PDCA_TEN_MASK; // Enable RX PDCA transfer first
						pdca_channeltx->cr = AVR32_PDCA_TEN_MASK; // and TX PDCA transfer

						while(!end_of_transfer);
						
						for(int i = 0; i <50; i++)
						{
							ram_bufferd[i]=ram_buffer[i];
							ram_buffer[i]=" ";
						}
						
					}
					et024006_PrintString(&ram_bufferd, (const unsigned char *)&FONT6x8, 10, 20*j, BLUE, -1);
						
				}
			flag=0;	  
		  break;
		  
	  }	 
  }
}
