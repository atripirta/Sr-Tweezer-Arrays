// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------- //
// trigger //
// ------- //

#define trigger_wrap_target 0
#define trigger_wrap 7

static const uint16_t trigger_program_instructions[] = {
            //     .wrap_target
    0x90a0, //  0: pull   block           side 0     
    0xa027, //  1: mov    x, osr                     
    0x0044, //  2: jmp    x--, 4                     
    0x1f00, //  3: jmp    0               side 1 [7] 
    0x20a0, //  4: wait   1 pin, 0                   
    0x7b04, //  5: out    pins, 4         side 1 [3] 
    0x7004, //  6: out    pins, 4         side 0     
    0x8020, //  7: push   block                      
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program trigger_program = {
    .instructions = trigger_program_instructions,
    .length = 8,
    .origin = -1,
};

static inline pio_sm_config trigger_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + trigger_wrap_target, offset + trigger_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}
#endif

// ----- //
// timer //
// ----- //

#define timer_wrap_target 0
#define timer_wrap 4

static const uint16_t timer_program_instructions[] = {
            //     .wrap_target
    0x80a0, //  0: pull   block           side 0     
    0xa027, //  1: mov    x, osr          side 0     
    0x0025, //  2: jmp    !x, 5           side 0     
    0xb542, //  3: nop                    side 1 [5] 
    0x0044, //  4: jmp    x--, 4          side 0     
            //     .wrap
    0x20a0, //  5: wait   1 pin, 0        side 0     
    0x0000, //  6: jmp    0               side 0     
};

#if !PICO_NO_HARDWARE
static const struct pio_program timer_program = {
    .instructions = timer_program_instructions,
    .length = 7,
    .origin = -1,
};

static inline pio_sm_config timer_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + timer_wrap_target, offset + timer_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

    static inline void trigger_program_init(PIO pio, uint sm, uint offset, 
        uint trigger_pin,
        uint p_pin, 
        uint update_pin
    ) {
        // profile pins
        pio_gpio_init(pio, p_pin + 0);
        pio_gpio_init(pio, p_pin + 1);
        pio_gpio_init(pio, p_pin + 2);
        pio_gpio_init(pio, p_pin + 3);
        // IO_UPDATE to AD9959
        pio_gpio_init(pio, update_pin);
        // External Trigger Pin
        pio_gpio_init(pio, trigger_pin); 
        pio_sm_set_pindirs_with_mask(pio, sm,
            (0xf << p_pin) | (1u << update_pin) | (0u << trigger_pin),    
            (0xf << p_pin) | (1u << update_pin) | (1u << trigger_pin)    
        );
        pio_sm_config c = trigger_program_get_default_config(offset);
        sm_config_set_sideset_pins(&c, update_pin);
        sm_config_set_out_pins(&c, p_pin, 4);
        sm_config_set_in_pins(&c, trigger_pin);
        sm_config_set_out_shift(&c, true, false, 1);
        sm_config_set_in_shift(&c, true, true, 1);
        sm_config_set_clkdiv(&c, 1.f);
        hw_set_bits(&pio->input_sync_bypass, 1u << trigger_pin);
        pio_sm_init(pio, sm, offset, &c);
        pio_sm_set_enabled(pio, sm, true);
    }
    static inline void timer_program_init(PIO pio, uint sm, uint offset,
        uint trigger_pin
    ) {
        pio_sm_config c = timer_program_get_default_config(offset);
        pio_gpio_init(pio, trigger_pin);
        pio_sm_set_pindirs_with_mask(pio, sm, 
            (1u << trigger_pin),
            (1u << trigger_pin)
        );
        sm_config_set_sideset_pins(&c, trigger_pin);
        sm_config_set_in_pins(&c, trigger_pin);
        sm_config_set_out_shift(&c, true, false, 1);
        sm_config_set_in_shift(&c, true, true, 1);
        sm_config_set_clkdiv(&c, 1.f);
        pio_sm_init(pio, sm, offset, &c);
        pio_sm_set_enabled(pio, sm, true);
    }

#endif

