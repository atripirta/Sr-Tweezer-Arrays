
/*
#######################################################################
#                                                                     #
# dds-sweeper.c                                                       #
#                                                                     #
# Copyright 2023, Ethan Huegler                                       #
#                                                                     #
# Serial communication code based on the PineBlaster and PrawnBlaster #
#   https://github.com/labscript-suite/pineblaster                    #
#   Copyright 2013, Christopher Billington                            #
#   https://github.com/labscript-suite/prawnblaster                   #
#   Copyright 2013, Philip Starkey                                    #
#                                                                     #
# This file is used to flash a Raspberry Pi Pico microcontroller      #
# prototyping board to create a DDS-Sweeper (see readme.txt           #
# This file is licensed under the BSD-2-Clause license.               #
# See the license.txt file for the full license.                      #
#                                                                     #
#######################################################################
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#define MAX_POINTS 13
#include "ad9959.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "trigger_timer.pio.h"

#define VERSION "0.1.1"

// Default Pins to use
#define PIN_MISO 12
#define PIN_MOSI 15
#define PIN_SCK 14
#define PIN_SYNC 10
#define PIN_CLOCK 21
#define PIN_UPDATE 22
#define PIN_RESET 9
#define P0 19
#define P1 18
#define P2 17
#define P3 16
#define TRIGGER 8

#define PIO_TRIG pio0
#define PIO_TIME pio1

// Mutex for status
static mutex_t status_mutex;
static mutex_t wait_mutex;

#define FLASH_TARGET_OFFSET (256 * 1024)

// STATUS flag
#define STOPPED 0
#define RUNNING 1
#define ABORTING 2
int status = STOPPED;

// modes
#define UNDEF_MODE -1
#define SS_MODE 0
#define AMP_MODE 1
#define FREQ_MODE 2
#define PHASE_MODE 3
#define AMP2_MODE 4
#define FREQ2_MODE 5
#define PHASE2_MODE 6


// PIO VALUES IT IS LOOKING FOR
#define UPDATE 0

#define MAX_SIZE 249856
#define TIMERS 5000
#define TIMING_OFFSET (MAX_SIZE - TIMERS * 4)

// minimum wait lengths
#define WAITS_SS_PER 250
#define WAITS_SS_BASE (500 - WAITS_SS_PER)
#define WAITS_SW_PER 500
#define WAITS_SW_BASE (1000 - WAITS_SW_PER)

// For responding OK to successful commands
#define OK() printf("ok\n")

// =============================================================================
// global variables
// =============================================================================
ad9959_config ad9959;
char readstring[256];
bool DEBUG = true;
bool timing = false;

uint triggers;

uint timer_dma;

uint INS_SIZE = 0;
uint8_t instructions[MAX_SIZE];

// =============================================================================
// Utility Functions
// =============================================================================

void init_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

void init_pio() {
    uint offset = pio_add_program(PIO_TRIG, &trigger_program);
    trigger_program_init(PIO_TRIG, 0, offset, TRIGGER, P3, PIN_UPDATE);
    offset = pio_add_program(PIO_TIME, &timer_program);
    timer_program_init(PIO_TIME, 0, offset, TRIGGER);
}

int get_status() {
    mutex_enter_blocking(&status_mutex);
    int temp = status;
    mutex_exit(&status_mutex);
    return temp;
}

void set_status(int new_status) {
    mutex_enter_blocking(&status_mutex);
    status = new_status;
    mutex_exit(&status_mutex);
}

float cubicSplineInterpolation(float x[MAX_POINTS], float y[MAX_POINTS], int n, float resultX) {
    float h[MAX_POINTS - 1];
    float alpha[MAX_POINTS - 1];
    float l[MAX_POINTS];
    float mu[MAX_POINTS - 1];
    float z[MAX_POINTS];
    float c[MAX_POINTS];
    float b[MAX_POINTS];
    float d[MAX_POINTS];

    // Step 1: Compute h and alpha
    for (int i = 0; i < n - 1; i++) {
        h[i] = x[i + 1] - x[i];
        alpha[i] = (3.0 / h[i]) * (y[i + 1] - y[i]) - (3.0 / h[i - 1]) * (y[i] - y[i - 1]);
    }

    // Step 2: Compute l, mu, and z
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;

    for (int i = 1; i < n - 1; i++) {
        l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0;
    z[n - 1] = 0.0;
    c[n - 1] = 0.0;

    // Step 3: Back-substitution
    for (int j = n - 2; j >= 0; j--) {
        c[j] = z[j] - mu[j] * c[j + 1];
        b[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
        d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
    }

    // Step 4: Interpolation
    int interval = -1;
    for (int i = 0; i < n - 1; i++) {
        if (resultX >= x[i] && resultX <= x[i + 1]) {
            interval = i;
            break;
        }
    }

    if (interval == -1) {
        printf("Error: Data point outside the range of interpolation.\n");
        return 0;
    }

    // Evaluate the cubic spline at the desired point
    float deltaX = resultX - x[interval];
    float interpolationResult = y[interval] + b[interval] * deltaX + c[interval] * deltaX * deltaX + d[interval] * deltaX * deltaX * deltaX;

    printf("Interpolated value at x = %.2f: %.4f\n", resultX, interpolationResult);
    return interpolationResult;
}


void measure_freqs(void) {
    // From https://github.com/raspberrypi/pico-examples under BSD-3-Clause License
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

    printf("pll_sys = %dkHz\n", f_pll_sys);
    printf("pll_usb = %dkHz\n", f_pll_usb);
    printf("rosc = %dkHz\n", f_rosc);
    printf("clk_sys = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb = %dkHz\n", f_clk_usb);
    printf("clk_adc = %dkHz\n", f_clk_adc);
    printf("clk_rtc = %dkHz\n", f_clk_rtc);
}

void readline() {
    int i = 0;
    char c;
    while (true) {
        c = getchar();
        if (c == '\n') {
            readstring[i] = '\0';
            return;
        } else {
            readstring[i++] = c;
        }
    }
}

void update() { pio_sm_put(PIO_TRIG, 0, UPDATE); }

void sync() {
    gpio_put(PIN_SYNC, 1);
    sleep_ms(1);
    gpio_put(PIN_SYNC, 0);
    sleep_ms(1);
}

void reset() {
    gpio_put(PIN_RESET, 1);
    sleep_ms(1);
    gpio_put(PIN_RESET, 0);
    sleep_ms(1);

    sync();
    ad9959.sweep_type = 1;
    ad9959.channels = 1;
    INS_SIZE = 14;

    set_pll_mult(&ad9959, ad9959.pll_mult);

    clear();
    update();
}

void wait(uint channel) {
    pio_sm_get_blocking(PIO_TRIG, 0);
    triggers++;
}

void abort_run() {
    if (get_status() == RUNNING) {
        set_status(ABORTING);

        // take control of trigger pin from PIO
        init_pin(TRIGGER);
        gpio_put(TRIGGER, 1);
        sleep_ms(1);
        gpio_put(TRIGGER, 0);

        // reinit PIO to give Trigger pin back
        init_pio();
    }
}


// =============================================================================
// Table Running Loop
// =============================================================================

void background() {
    // let other core know ready
    multicore_fifo_push_blocking(0);

    int hwstart = 0;
    while (true) {
        // wait for a start command
        hwstart = multicore_fifo_pop_blocking();

        set_status(RUNNING);

        // pre-calculate spacing vars
        uint step = INS_SIZE * ad9959.channels + 1;
        uint offset = 0;

        // count instructions to run
        bool repeat = false;
        int num_ins = 0;
        int i = 0;
        while (true) {
            // If an instruction is empty that means to stop
            if (instructions[offset] == 0x00) {
                if (instructions[offset + 1]) {
                    repeat = true;
                }
                break;
            }
            offset = step * ++i;
        }

        num_ins = i;
        offset = i = 0;
        triggers = 0;

        // sync just to be sure
        sync();

        // if this is hwstart, stell the timer pio core and it will handle that on its own
        if (hwstart) {
            pio_sm_put(PIO_TIME, 0, 0);
        }

        while (status != ABORTING) {
            // check if last instruction
            if (i == num_ins) {
                if (repeat) {
                    i = offset = 0;
                } else {
                    break;
                }
            }

            // prime PIO
            pio_sm_put(PIO_TRIG, 0, instructions[offset]);

            // send new instruciton to AD9959
            spi_write_blocking(spi1, instructions + offset + 1, step - 1);

            // if on the first instruction, begin the timer
            if (i == 0 && timing) {
                dma_channel_transfer_from_buffer_now(timer_dma, instructions + TIMING_OFFSET,
                                                     num_ins);
            }

            wait(0);

            offset = step * ++i;
        }

        // clean up
        dma_channel_abort(timer_dma);
        pio_sm_clear_fifos(PIO_TRIG, 0);
        pio_sm_clear_fifos(PIO_TIME, 0);
        set_status(STOPPED);
    }
}

// =============================================================================
// Serial Communication Loop
// =============================================================================

void loop() {
    readline();
    int local_status = get_status();

    if (strncmp(readstring, "version", 7) == 0) {
        printf("%s\n", VERSION);
    } else if (strncmp(readstring, "status", 6) == 0) {
        printf("%d\n", local_status);
    } else if (strncmp(readstring, "debug on", 8) == 0) {
        DEBUG = 1;
        OK();
    } else if (strncmp(readstring, "debug off", 9) == 0) {
        DEBUG = 0;
        OK();
    } else if (strncmp(readstring, "getfreqs", 8) == 0) {
        measure_freqs();
    } else if (strncmp(readstring, "numtriggers", 11) == 0) {
        printf("%u\n", triggers);
    } else if (strncmp(readstring, "reset", 5) == 0) {
        abort_run();
        reset();
        set_status(STOPPED);
        OK();
    } else if (strncmp(readstring, "abort", 5) == 0) {
        abort_run();
        OK();
    }
    // ====================================================
    // Stuff that cannot be done while the table is running
    // ====================================================
    else if (local_status != STOPPED) {
        printf(
            "Cannot execute command \"%s\" during buffered execution. Check "
            "status first and wait for it to return %d (stopped or aborted).\n",
            readstring, STOPPED);
    } else if (strncmp(readstring, "readregs", 8) == 0) {
        single_step_mode();
        update();
        read_all();
        OK();
      }  else if (strncmp(readstring, "load", 4) == 0) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overread"
        memcpy(instructions, ((uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET)), MAX_SIZE);
#pragma GCC diagnostic pop

        OK();
    } else if (strncmp(readstring, "save", 4) == 0) {
        uint32_t ints = save_and_disable_interrupts();
        // erase sections
        flash_range_erase(FLASH_TARGET_OFFSET, MAX_SIZE);
        // reprogram
        flash_range_program(FLASH_TARGET_OFFSET, instructions, MAX_SIZE);
        restore_interrupts(ints);
        OK();
    } else if (strncmp(readstring, "setfreq1", 8) == 0) {
        // setfreq <channel:int> <frequency:float>

        uint channel=0;
        double freq=85500000;
        uint8_t ftw[4];
        freq = get_ftw(&ad9959, freq, ftw);
        send_channel(0x04, channel, ftw, 4);
        update();

        if (DEBUG) {
             printf("set freq: %lf\n", freq);
        }
        OK();
    } else if (strncmp(readstring, "setfreq2", 8) == 0) {
        // setfreq <channel:int> <frequency:float>

        uint channel=0;
        double freq=120500000;
        uint8_t ftw[4];
        freq = get_ftw(&ad9959, freq, ftw);
        send_channel(0x04, channel, ftw, 4);
        update();

        if (DEBUG) {
             printf("set freq: %lf\n", freq);
        }
        OK();
    } else if (strncmp(readstring, "setamp", 6) == 0) {
        // setamp <channel:int> <amp:float>
        uint channel=0;
        double amp=0.681;
        uint8_t asf[3];
        amp = get_asf(amp, asf);
        send_channel(0x06, channel, asf, 3);
        update();
        if (DEBUG) {
            printf("Amp: %12lf\n", amp);
        }
        OK();
    } else if (strncmp(readstring, "checkv", 11) == 0) {
        // setfreq <channel:int> <frequency:float>

        uint channel=0;
        double freq1=92.5*1000000;
        double freq2=99.5*1000000;
        uint8_t ftw1[4];
        uint8_t ftw2[4];
        int j=1;
        double amp1=0.49955;
        double amp2=0.49955;
        uint8_t asf1[3];
        uint8_t asf2[3];
        
        while(j<=30000){
          amp1 = get_asf(0.685, asf1);
          send_channel(0x06, channel, asf1, 3);
          update();
          freq1 = get_ftw(&ad9959, freq1, ftw1);
          send_channel(0x04, channel, ftw1, 4);
          update();
          sleep_ms(1000);
          freq2 = get_ftw(&ad9959, freq2, ftw2);
          send_channel(0x04, channel, ftw2, 4);
          update();
          amp2 = get_asf(0.716, asf2);
          send_channel(0x06, channel, asf2, 3);
          update();
          
          j++;
          sleep_ms(2000);
        }j=1;
        amp2 = get_asf(0, asf2);
        send_channel(0x06, channel, asf2, 3);
        update();
      } else if (strncmp(readstring, "Custom", 6) == 0) {
        // setfreq <channel:int> <frequency:float>

        uint channel=0;
        double freq;
        uint8_t ftw[4];
        int i=0;
        double amp=0.55;
        uint8_t asf[3];
        amp = get_asf(amp, asf);
        send_channel(0x06, channel, asf, 3);
        update();
        int j=1;
        while(j<=30000){
        while(i<=5){    
          if(i%6==0){
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 85500000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          //sleep_ms(1);
          }
          else if(i%6==1){
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92500000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();}
          else if(i%6==2){
          freq = get_ftw(&ad9959, 99500000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();}
          else if(i%6==3){
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106500000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();}
          else if(i%6==4){
          freq = get_ftw(&ad9959, 113500000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.755, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          }else{
          freq = get_ftw(&ad9959, 120500000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.89, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          
          }
          sleep_ms(1);
          amp = get_asf(0.0, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 0.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          i++;
          
          //printf("Frequency in MHz:",i);
        }i=0;j++;}
      } else if (strncmp(readstring, "sweepamp", 11) == 0) {
        // setfreq <channel:int> <frequency:float>

        uint channel=0;
        double freq=85.5*1000000;
        uint8_t ftw[4];
        int j=1;
        double amp=0.65;
        double temp=amp;
        uint8_t asf[3];
        amp = get_asf(amp, asf);
        send_channel(0x06, channel, asf, 3);
        update();
        freq = get_ftw(&ad9959, freq, ftw);
        send_channel(0x04, channel, ftw, 4);
        update();
        while(j<1000){
        int i=1;
        while(i<=50){
            amp=temp+i*0.001;
            amp = get_asf(amp, asf);
            send_channel(0x06, channel, asf, 3);
            update();
            i++;
            sleep_ms(1.5);
        }
        j++;
        }
        send_channel(0x06, channel, 0, 3);
        update();
      } else if (strncmp(readstring, "freq99.5", 8) == 0) {
        // setfreq <channel:int> <frequency:float>
        double amp=1.0;
        double freq=99500000;
        uint8_t ftw[4];
        uint8_t asf[3];
        amp = get_asf(amp, asf);
        send_channel(0x06, 0, asf, 3);
        update();
        amp = get_asf(amp, asf);
        send_channel(0x06, 1, asf, 3);
        update();
        freq = get_ftw(&ad9959, freq, ftw);
        send_channel(0x04, 0, ftw, 4);
        update();
        freq = get_ftw(&ad9959, freq, ftw);
        send_channel(0x04, 1, ftw, 4);
        update();


        if (DEBUG) {
             printf("set freq: %lf\n", freq);
        }
        OK();
    } else if (strncmp(readstring, "Interpolate", 11) == 0) {
        // setfreq <channel:int> <frequency:float>
        float x[MAX_POINTS] ={85.5, 92.5, 99.5, 106.5, 113.5, 120.5};
        float y[MAX_POINTS] = {0.681, 0.688, 0.7349, 0.710, 0.76, 0.9};//0.898
        // Number of data points
        int n = sizeof(x) / sizeof(x[0]);
        uint channel=0;
        double freq;
        uint8_t ftw[4];
        int j=0;
        double resultX;
        double amp=0.5;
        uint8_t asf[3];
        amp = get_asf(amp, asf);
        send_channel(0x06, channel, asf, 3);
        update();
        while(j<=200){
        int i=85;
        while(i<=120){    
          freq=(i+0.5)*1000000.0;
          resultX=i+0.5;
          amp=cubicSplineInterpolation(x, y, n, resultX);
          
          printf("set amp: %.2f\n", amp);
          amp = get_asf(amp, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, freq, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
            //OK();
            sleep_ms(3);
            i++;
          //printf("Frequency in MHz:",i);
        }j++;
        amp = get_asf(0, asf);
        send_channel(0x06, channel, asf, 3);
        update();
        sleep_ms(1);}
      } else if (strncmp(readstring, "freq_and_amp", 12) == 0) {//changing amplitude as well as frequency to get stability under 1 percent.
        // setfreq <channel:int> <frequency:float>

        uint channel=0;
        double freq;
        uint8_t ftw[4];
        int i=85; 
        int j=0;
        double amp=1.0;
        uint8_t asf[3];
        amp = get_asf(amp, asf);
        send_channel(0x06, channel, asf, 3);
        update();
        printf("set amp: %.2f\n", amp);
        while(j<500){
        while(i<=121){    
          freq=(i+0.5)*1000000.0;
          freq = get_ftw(&ad9959, freq, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          //OK();
          sleep_ms(2);
          printf("Frequency in MHz:%2f\n",(i+0.5));
          i++;
        }
        i=85;
        j++;
      }
    } else if (strncmp(readstring, "Cust", 4) == 0) {//changing amplitude as well as frequency to get stability under 1 percent.
        // setfreq <channel:int> <frequency:float>

        uint channel=0;
        double freq;
        uint8_t ftw[4];
        int i=0;
        double amp=0.55;
        uint8_t asf[3];
        amp = get_asf(amp, asf);
        send_channel(0x06, channel, asf, 3);
        update();
        int j=1;
        while(j<=30000){
        while(i<=5){    
          if(i%6==0){
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 85500000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          //sleep_ms(1);
          }
          else if(i%6==1){
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92500000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();}
          else if(i%6==3){
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106500000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();}
          else if(i%6==5){
          freq = get_ftw(&ad9959, 120500000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.89, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          }
          sleep_ms(1);
          amp = get_asf(0.0, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 0.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          i++;
          
          //printf("Frequency in MHz:",i);
        }i=0;j++;}
    }
    else if (strncmp(readstring, "pattern1", 8) == 0) {//changing amplitude as well as frequency to get stability under 1 percent.
        // setfreq <channel:int> <frequency:float>

        uint channel=0;
        uint channel1=1;
        double freq;
        uint8_t ftw[4];
        int i=0; 
        double amp;
        uint8_t asf[3];
        while(i<=10000){
          i++;
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 85.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(1.0, asf);
          send_channel(0x06, channel1, asf, 3);
          update();
          freq = get_ftw(&ad9959, 86*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 113.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.755, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 114*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          amp = get_asf(1.0, asf);
          send_channel(0x06, channel1, asf, 3);
          update();
          sleep_ms(1);
        
        }
    } else if (strncmp(readstring, "pattern2", 8) == 0) {//changing amplitude as well as frequency to get stability under 1 percent.
        // setfreq <channel:int> <frequency:float>
        
        uint channel=0;
        uint channel1=1;
        double freq;
        uint8_t ftw[4];
        int i=0; 
        double amp;
        uint8_t asf[3];
        amp = get_asf(1.0, asf);
        send_channel(0x06, channel1, asf, 3);
        update();//Channel 1 always set to 1.0
        while(i<=10000){
          i++;
          freq = get_ftw(&ad9959, 86*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.755, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 113.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 86*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 93*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 100*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 107*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 114*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 85.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 107*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 100*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 93*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);        
        }
    } else if (strncmp(readstring, "pattern3", 8) == 0) {
        // setfreq <channel:int> <frequency:float>

        uint channel=0;
        uint channel1=1;
        double freq;
        uint8_t ftw[4];
        int i=0; 
        double amp;
        uint8_t asf[3];
        amp = get_asf(1.0, asf);
        send_channel(0x06, channel1, asf, 3);
        update();
        while(i<=10000){
          i++;
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 85.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          freq = get_ftw(&ad9959, 86*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 113.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.755, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          sleep_ms(1);
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          freq = get_ftw(&ad9959, 93*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 100*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 107*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 114*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);
        }
    } else if (strncmp(readstring, "pattern4", 8) == 0) {

        uint channel=0;
        uint channel1=1;
        double freq;
        uint8_t ftw[4];
        int i=0; 
        double amp;
        uint8_t asf[3];
        amp = get_asf(1.0, asf);
        send_channel(0x06, channel1, asf, 3);
        update();
        while(i<=10000){
          i++;
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          freq = get_ftw(&ad9959, 114*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 107*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 100*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          freq = get_ftw(&ad9959, 85.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 113.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.755, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 93*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          freq = get_ftw(&ad9959, 85.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 113.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.755, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 86*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          sleep_ms(1);
        
        }
    } else if (strncmp(readstring, "pattern5", 8) == 0) {//changing amplitude as well as frequency to get stability under 1 percent.
        // setfreq <channel:int> <frequency:float>
        
        uint channel=0;
        uint channel1=1;
        double freq;
        uint8_t ftw[4];
        int i=0; 
        double amp;
        uint8_t asf[3];
        amp = get_asf(1.0, asf);
        send_channel(0x06, channel1, asf, 3);
        update();//Channel 1 always set to 1.0
        while(i<=10000){
          i++;
          freq = get_ftw(&ad9959, 86*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.755, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 113.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          freq = get_ftw(&ad9959, 93*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 100*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 107*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          freq = get_ftw(&ad9959, 114*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 107*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          freq = get_ftw(&ad9959, 85.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 100*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 93*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 107*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
        }
    } else if (strncmp(readstring, "pattern6", 8) == 0) {//changing amplitude as well as frequency to get stability under 1 percent.
        // setfreq <channel:int> <frequency:float>
        
        uint channel=0;
        uint channel1=1;
        double freq;
        uint8_t ftw[4];
        int i=0; 
        double amp;
        uint8_t asf[3];
        amp = get_asf(1.0, asf);
        send_channel(0x06, channel1, asf, 3);
        update();//Channel 1 always set to 1.0
        while(i<=10000){
          i++;
          freq = get_ftw(&ad9959, 86*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.755, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 113.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 85.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next--------------------------
          freq = get_ftw(&ad9959, 100*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.755, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 113.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 85.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next 
          freq = get_ftw(&ad9959, 114*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 107*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 92*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          sleep_ms(1);
        }
    } else if (strncmp(readstring, "pattern7", 8) == 0) {//changing amplitude as well as frequency to get stability under 1 percent.
        // setfreq <channel:int> <frequency:float>
        
        uint channel=0;
        uint channel1=1;
        double freq;
        uint8_t ftw[4];
        int i=0; 
        double amp;
        uint8_t asf[3];
        amp = get_asf(1.0, asf);
        send_channel(0x06, channel1, asf, 3);
        update();//Channel 1 always set to 1.0
        while(i<=10000){
          i++;
          freq = get_ftw(&ad9959, 86*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 85.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.755, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 113.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 93*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 100*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          amp = get_asf(0.717, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 99.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          freq = get_ftw(&ad9959, 107*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          amp = get_asf(0.685, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 92.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.703, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 106.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
          freq = get_ftw(&ad9959, 114*1000000.0, ftw);
          send_channel(0x04, channel1, ftw, 4);
          update();
          amp = get_asf(0.681, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 85.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);//Next
          amp = get_asf(0.755, asf);
          send_channel(0x06, channel, asf, 3);
          update();
          freq = get_ftw(&ad9959, 113.5*1000000.0, ftw);
          send_channel(0x04, channel, ftw, 4);
          update();
          sleep_ms(1);
        }
    }
}

// =============================================================================
// Initial Setup
// =============================================================================

int main() {
    init_pin(PICO_DEFAULT_LED_PIN);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    stdio_init_all();

    set_sys_clock_khz(125 * MHZ / 1000, false);

    // output sys clock on a gpio pin to be used as REF_CLK for AD9959
    clock_gpio_init(PIN_CLOCK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 1);

    // attatch spi to system clock so it runs at max rate
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 125 * MHZ,
                    125 * MHZ);

    // init SPI
    spi_init(spi1, 100 * MHZ);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // launch other core
    multicore_launch_core1(background);
    multicore_fifo_pop_blocking();

    // initialise the status mutex
    mutex_init(&status_mutex);
    mutex_init(&wait_mutex);

    // init the PIO
    init_pio();

    // setup dma
    timer_dma = dma_claim_unused_channel(true);

    // if pico is timing itself, it will use dma to send all the wait
    // lengths to the timer pio program
    dma_channel_config c = dma_channel_get_default_config(timer_dma);
    channel_config_set_dreq(&c, DREQ_PIO1_TX0);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    dma_channel_configure(timer_dma, &c, &PIO_TIME->txf[0], instructions + TIMING_OFFSET, 0, false);

    // put AD9959 in default state
    init_pin(PIN_SYNC);
    init_pin(PIN_RESET);
    set_ref_clk(&ad9959, 125 * MHZ);
    set_pll_mult(&ad9959, 4);
    reset();

    while (true) {
        loop();
    }
    return 0;
}
