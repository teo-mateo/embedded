#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/spi.h"

//We will be using ALARM 0
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

//DDS parameters
#define two32 4294967296.0 // 2^32 
#define Fs 50000
#define DELAY 20            // 1/Fs (in microseconds)

// the DDS units:
volatile unsigned int phase_accum_main;
volatile unsigned int phase_incr_main = (500.0*two32)/Fs ;// initial value not needed for swoop/chirp

volatile bool is_on = true;

// DDS sine table
#define sine_table_size 256

// SWOOP table
#define swoop_sample_size 5200
volatile int sin_table[sine_table_size] ;
volatile int swoop_samples[swoop_sample_size];
volatile int swoop_sample_index = 0;

#define chirp_sample_size 5200
volatile int chirp_samples[chirp_sample_size];
volatile int chirp_sample_index = 0;

// Single amplitude table when swoop and chirp sample arrays are of the same size
volatile double ampl_table[swoop_sample_size];
volatile int ampl_index = 0;

// Chirp or Swoop
volatile char swoop_or_chirp = 's';

//GPIO for timing the ISR
#define ISR_GPIO 2

//SPI data
uint16_t DAC_data ; // output value
double ampl_factor;

//DAC parameters
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

//SPI configurations
#define PIN_MISO 4 // WHITE
#define PIN_CS   5 // GREEN
#define PIN_SCK  6 // YELLOW
#define PIN_MOSI 7 // ORANGE 
#define SPI_PORT spi0

static void alarm_irq(void){

    //assert a gpio when we enter the interrupt
    gpio_put(ISR_GPIO, 1);

    //clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    //reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    if (swoop_or_chirp == 's'){
        if (swoop_sample_index >= swoop_sample_size){
            swoop_sample_index = 0;
            swoop_or_chirp = 'c';
            gpio_put(ISR_GPIO, 0);
            return;
        } else{
            phase_accum_main += swoop_samples[swoop_sample_index++];
            DAC_data = (DAC_config_chan_A | (
                ((int)((double)(sin_table[phase_accum_main>>24] + 2048) * ampl_table[swoop_sample_index]))
                & 0xffff
                ));
        }
    }

    if (swoop_or_chirp == 'c'){
        if (chirp_sample_index >= chirp_sample_size){
            chirp_sample_index = 0;
            swoop_or_chirp = 's';
            timer_hw->alarm[ALARM_NUM] = timer_hw->alarm[ALARM_NUM] + 250000;
            return;
        } else {
            phase_accum_main += chirp_samples[chirp_sample_index++]; 
            DAC_data = (DAC_config_chan_A | (
                ((int)((double)(sin_table[phase_accum_main>>24] + 2048) * ampl_table[chirp_sample_index]))
                & 0xffff
                ));
        }
    }

    // Perform an SPI transaction
    gpio_put(PIN_CS, 0);
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);
    gpio_put(PIN_CS, 1);

    //De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);
}

int main()
{
    //Initialize stdio
    stdio_init_all();

    //Initialize SPI
    spi_init(SPI_PORT, 20000000);

    //Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    //Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    //Build the sine lookup table
   	//scaled to produce values between 0 and 4096
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = (int)(2047*sin((float)ii*6.283/(float)sine_table_size));
    }    

    // Build the amplitude table
    for (ii = 0; ii < swoop_sample_size; ii++){
        if (ii < 1000) ampl_table[ii] = (double)ii / 1000;
        else if (ii < swoop_sample_size-1000) ampl_table[ii] = 1.0;
        else ampl_table[ii] = 1.0 - (double)(-(swoop_sample_size-1000-ii))/1000;
    }

    //Build the swoop sample lookup table
    double sinfactor; // [0..-1..0]
    int frequency;
    int phase_inc;
    for (ii = 0; ii < swoop_sample_size; ii++){
        sinfactor = sin(-M_PI/swoop_sample_size*ii);
        frequency = -260*sinfactor+1740; 
        phase_inc = (frequency*two32)/Fs;
        swoop_samples[ii] = phase_inc;
    }

    // Build the chirp sample lookup table
    for (ii = 0; ii < chirp_sample_size; ii++)
    {
         frequency = (1.84f *0.0001f)*ii*ii + 2000;
         phase_inc = (frequency*two32)/Fs;
         chirp_samples[ii] = phase_inc;
    }

    //
    // ALARM 0 is used by DDS
    //

    //Enable interrupts for alarm 0
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);

    //Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);

    //Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true);

    //Write the lower 32 bits of the target time to the alarm register, arming it
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY; 

    while(1){
        //bored
    }
    return 0;
}