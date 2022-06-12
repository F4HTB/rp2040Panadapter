/*****************************************************************************
* | Author      :   F4HTB
#
******************************************************************************/
#include "main.h"
#include "LCD_0in96.h"


#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "kiss_fft.h"


// ====================================================
// ADC setup (@ADC)
// ====================================================
// set this to determine sample rate (we are in I/Q mode so ksamples/s equal the bandwith to see)
// 0     = 500ksamples/s
// 960   = 50ksamples/s
// 9600  = 5ksamples/s
// 600   = 80ksamples/s
#define CLOCK_DIV 600

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP  LCD_0IN96_WIDTH //Set as display width 160px
#define NSAMP_IQ  (NSAMP * 2) //NSAMP * 2

// globals
dma_channel_config cfg;
uint dma_chan;

void sample(uint8_t *capture_buf) {
  adc_fifo_drain();
  adc_run(false);
      
  dma_channel_configure(dma_chan, &cfg,
			capture_buf,    // dst
			&adc_hw->fifo,  // src
			NSAMP_IQ,          // transfer count
			true            // start immediately
			);

  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);

}

void setup() {
	//stdio_init_all();
	adc_init(); // Initialize ADC to known state
	adc_set_clkdiv(CLOCK_DIV); // set sample rate
	adc_gpio_init(26); // GP26 is ADC 0 for Q channel
	adc_gpio_init(27); // GP27 is ADC 1 for I channel
	adc_select_input(0); // Start with ADC0
	adc_set_round_robin(0x01+0x02);			// Sequence ADC 0-1 (GP 26, 27) free running
	
	adc_fifo_setup(
		 true,    // Write each completed conversion to the sample FIFO
		 true,    // Enable DMA data request (DREQ)
		 1,       // DREQ (and IRQ) asserted when at least 1 sample present
		 false,   // We won't see the ERR bit because of 8 bit reads; disable.
		 false     // Shift each sample to 8 bits when pushing to FIFO
		 );

	

	sleep_ms(1000);
	// Set up the DMA to start transferring data as soon as it appears in FIFO
	uint dma_chan = dma_claim_unused_channel(true);
	cfg = dma_channel_get_default_config(dma_chan);

	// Reading from constant address, writing to incrementing byte addresses
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
	channel_config_set_read_increment(&cfg, false);
	channel_config_set_write_increment(&cfg, true);

	// Pace transfers based on availability of ADC samples
	channel_config_set_dreq(&cfg, DREQ_ADC);

}

// ==================================
// === Main
//====================================

int main(void)
{
	
	uint8_t cap_buf[NSAMP_IQ];
	kiss_fft_cpx fft_in[NSAMP]; // kiss_fft_scalar is a float
	kiss_fft_cpx fft_out[NSAMP];
	kiss_fft_cfg cfg = kiss_fft_alloc(NSAMP,false,0,0);
  	
	DEV_Delay_ms(100);
    if(DEV_Module_Init()!=0){
        return -1;
    }

    /* LCD Init */
    LCD_0IN96_Init(HORIZONTAL);
    LCD_0IN96_Clear(WHITE);
    DEV_Delay_ms(1000);
	
	//Byte array for Image cache
    UDOUBLE Imagesize = LCD_0IN96_HEIGHT*LCD_0IN96_WIDTH*2;
    UWORD *ImageToDraw;
    if((ImageToDraw = (UWORD *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        exit(0);
    }
    //Create a new image cache and fill it with white*/
    Paint_NewImage((UBYTE *)ImageToDraw,LCD_0IN96.WIDTH,LCD_0IN96.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);
    Paint_Clear(WHITE);

	//Paint starting logo
    Paint_DrawImage(gImage_Logo_F4HTB,0,0,160,80);
    LCD_0IN96_Display(ImageToDraw);
    DEV_Delay_ms(2000);
     

    //Paint_Clear(WHITE);
	LCD_0IN96_Display(ImageToDraw);
	
	// setup ADC
	setup();
	
	short int time_offset_line = 0; //Use for write ImageToDraw as a circular buffer 
	while (1) {
		// get NSAMP samples
		sample(cap_buf);
		// fill fourier transform input while subtracting DC component
		uint64_t sum = 0;
		for (int i=0;i<NSAMP_IQ;i++) {sum+=cap_buf[i];}
		float avg = (float)sum/NSAMP_IQ;
		

		
		for (int i=0;i<NSAMP;i++) {
			int index_I=i*2;
			int index_Q=(i*2)+1;
			fft_in[i].i=(float)cap_buf[index_I]-avg;
			fft_in[i].r=(float)cap_buf[index_Q]-avg;
			}

		// compute fast fourier transform
		kiss_fft(cfg , fft_in, fft_out);
		int Circular_Buf_offset=(79-time_offset_line); //We gein at end 
		
		//Magnetude power spectrum calculation and sending to ImageToDraw
		for (int i = 0; i < NSAMP; i++) {
			float power = fft_out[i].r*fft_out[i].r+fft_out[i].i*fft_out[i].i;
			uint8_t colorFFT = (uint8_t)(power);
			UDOUBLE Addr = i + Circular_Buf_offset*(160);
			ImageToDraw[Addr] = colormap_rainbow[colorFFT][0];
			ImageToDraw[Addr+1] = colormap_rainbow[colorFFT][1];
		}
		
		LCD_0IN96_Display_circular(ImageToDraw,Circular_Buf_offset);
		time_offset_line++;
		if(time_offset_line>79)time_offset_line=0;
	}

	// should never get here
	kiss_fft_free(cfg);

	/* Module Exit */
	free(ImageToDraw);
	ImageToDraw = NULL;
    
	DEV_Module_Exit();
	return 0;
}