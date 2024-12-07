/**
 * @file beck-view-connector.c
 * @brief Super 8 projector connector using Raspberry Pi Pico and C.
 */

#include "stdio.h"
#include "string.h"
#include "pico.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

#include <stdbool.h>
#include "ili9341/ili9341.h"

#include "pt_v1_3.h"

#include "frame_timing.h"
#include "display.h"

#include "frame_signal.pio.h"

#define ADVANCE_FRAME_PIN 4 ///< GPIO pin for frame advance signal input.
#define END_OF_FILM_PIN 5   ///< GPIO pin for end-of-film signal input.

#define PASS_ON_FRAME_ADVANCE_PIN 2 //< GPIO pin to propagate frame advance signal.
#define PASS_ON_END_OF_FILM_PIN 3   ///< GPIO pin to propagate end-of-film signal.

#define FRAME_ADVANCE_DURATION_US 8000  ///< Duration in microseconds to maintain frame advance signal.
#define END_OF_FILM_DURATION_US 1000000 ///< Duration in microseconds to maintain end-of-film signal.

#define DEBOUNCE_DELAY_US 3000 ///< Debounce duration for signal edges.

#define UPDATE_CMD 1 ///< Command for updating display.
#define EOF_CMD 2    ///< Command for end-of-film handling.

static PIO pio[4];
static uint sm[4];
static uint offset[4];

static uint32_t frame_signal_duration = 0;
static uint32_t eof_signal_duration = 0;

/// Frame counter to track total frames processed.
static uint frame_counter = 0;

/// Stores IRQ status for the frame advance signal.
static volatile uint irq_status = 0;

/// Queue for frame timing data and command exchange between cores.
static queue_t frame_queue;

/// Protothread control structure.
static struct pt pt;

critical_section_t cs1;

/**
 * @brief Protothread for updating the display with frame timing and status information.
 *
 * @param pt Pointer to the protothread control structure.
 * @return Protothread state.
 */
static PT_THREAD(update_display(struct pt *pt))
{
    PT_BEGIN(pt);

    static queue_entry_t queue_entry;

    while (1)
    {
        PT_WAIT_UNTIL(pt, queue_try_remove(&frame_queue, &queue_entry));

        if (queue_entry.cmd == UPDATE_CMD)
        {
            // FPS Calculation
            uint64_t time_diff = absolute_time_diff_us(queue_entry.frame_start, queue_entry.frame_end);
            float fps = 1.0e6 / time_diff;

            // Duration Calculation
            float duration = time_diff / 1.0e6;

            display_frame_info(queue_entry.frame_counter, fps, duration);
        }
        else if (queue_entry.cmd == EOF_CMD)
        {
            uint64_t time_diff = absolute_time_diff_us(queue_entry.start_time, queue_entry.frame_end);
            float avg_fps = (1.0e6 / time_diff) * queue_entry.frame_counter;
            float duration = time_diff / 1.0e6;

            display_eof_info(queue_entry.frame_counter, avg_fps, duration);
        }
        else
        {
            printf("Invalid command %u\n", queue_entry.cmd);
        }
    }

    PT_END(pt);
}

/**
 * @brief Secondary core entry point for display-related tasks.
 */
void core1_entry()
{
    static queue_entry_t queue_entry;

    ili9341_config_t hw_config;
    display_init(&hw_config);

    display_start_info();

    PT_INIT(&pt);

    pt_add_thread(update_display);

    pt_schedule_start;
}

/**
 * @brief Initialize the LED pin and provide a blinking status.
 *
 * @return int Returns PICO_OK on successful initialization.
 */
int led_init(void)
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    for (int i = 0; i < 5; i++)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(500);
    }
    return PICO_OK;
}

/**
 * @brief Initialize a GPIO pin with specified pull and direction.
 *
 * @param gpio Pin number to initialize.
 * @param pull_up Set to true to enable pull-up; false for pull-down.
 * @param is_output Set to true to configure as output; false for input.
 */
void init_gpio_pin(uint gpio, bool pull_up, bool is_output)
{
    gpio_init(gpio);
    gpio_set_pulls(gpio, pull_up, !pull_up);
    gpio_set_dir(gpio, is_output ? GPIO_OUT : GPIO_IN);
    if (is_output)
        gpio_put(gpio, true);
}

/**
 * @brief Initialize the GPIO pins for frame advance and end-of-film signals.
 */
void init_pins()
{
    init_gpio_pin(ADVANCE_FRAME_PIN, true, false);
    init_gpio_pin(END_OF_FILM_PIN, true, false);
    init_gpio_pin(PASS_ON_FRAME_ADVANCE_PIN, true, true);
    init_gpio_pin(PASS_ON_END_OF_FILM_PIN, true, true);
}

void pio_setup(const pio_program_t *program, PIO *pio, uint *sm, uint *offset, uint gpio_base)
{
    // Find a free pio and state machine and add the program
    bool rc = pio_claim_free_sm_and_add_program_for_gpio_range(program,
                                                               pio,
                                                               sm,
                                                               offset,
                                                               gpio_base,
                                                               1,
                                                               true);
    hard_assert(rc);
    frame_signal_program_init(*pio, *sm, *offset, gpio_base);
    pio_sm_set_enabled(*pio, *sm, true);
}

/**
 * @brief Enables edge interrupt for frame advance signal after debounce delay.
 *
 * @param id Alarm ID (unused).
 * @param user_data Additional user data (unused).
 * @return int64_t Always returns 0.
 */
int64_t enable_frame_advance_edge_irq(alarm_id_t id, void *user_data)
{
    critical_section_enter_blocking(&cs1);
    irq_status = gpio_get_all() & (1 << ADVANCE_FRAME_PIN) ? 1 : 0;
    gpio_set_irq_enabled((uint)ADVANCE_FRAME_PIN,
                         irq_status ? (uint32_t)(GPIO_IRQ_EDGE_FALL) : (uint32_t)(GPIO_IRQ_EDGE_RISE),
                         true);
    critical_section_exit(&cs1);
    return 0;
}

void gpio_irq_callback_isr(uint gpio, uint32_t event_mask)
{
    if (gpio == ADVANCE_FRAME_PIN)
    {
        gpio_acknowledge_irq(ADVANCE_FRAME_PIN, event_mask);
        if (event_mask & GPIO_IRQ_EDGE_FALL)
        {
            // Temporarily disable IRQs.
            gpio_set_irq_enabled(ADVANCE_FRAME_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false);
            if (irq_status == 1)
            {
                irq_status = 3;
                if (frame_counter == 0)
                {
                    init_frame_timing();
                    // Enable end of film detection.
                    gpio_set_irq_enabled(END_OF_FILM_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
                }

                queue_entry_t entry;

                process_frame_timing(&entry, frame_counter, UPDATE_CMD);

                register int64_t delta = absolute_time_diff_us(entry.frame_start, entry.frame_end);
                if (delta < DEBOUNCE_DELAY_US)
                {
                    printf("Alert --- too fast, too furious %u %llu us\n", entry.frame_counter, delta);
                }

                queue_add_blocking(&frame_queue, &entry);

                pio[0]->txf[sm[0]] = frame_signal_duration;
                pio[1]->txf[sm[1]] = frame_signal_duration;

                // Debounce edge detection for DEBOUNCE_DELAY_US - enable IRQ again after DEBOUNCE_DELAY_US.
                uint64_t edge_rise_alarm_id = add_alarm_in_us(DEBOUNCE_DELAY_US, enable_frame_advance_edge_irq, NULL, false);
                if (edge_rise_alarm_id < 0)
                {
                    printf("Edge_rise_alarm_id Alarm error %llu\n", edge_rise_alarm_id);
                }

                frame_counter++;
            }
        }
        else if (event_mask & GPIO_IRQ_EDGE_RISE)
        {
            if (irq_status == 0)
            {
                irq_status = 2;
                // Debounce edge detection - enable IRQ again after DEBOUNCE_DELAY_US.
                uint64_t edge_fall_alarm_id = add_alarm_in_us(DEBOUNCE_DELAY_US, enable_frame_advance_edge_irq, NULL, false);
                if (edge_fall_alarm_id < 0)
                {
                    printf("Edge_fall_alarm_id Alarm error %lld\n", edge_fall_alarm_id);
                }
            }
        }
    }
    else if (gpio == END_OF_FILM_PIN)
    {
        gpio_acknowledge_irq(END_OF_FILM_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);

        if (event_mask & GPIO_IRQ_EDGE_RISE)
        {
            // Disable IRQ
            gpio_set_irq_enabled(END_OF_FILM_PIN, GPIO_IRQ_EDGE_RISE, false);

            queue_entry_t entry;

            process_frame_timing(&entry, frame_counter, EOF_CMD);

            queue_add_blocking(&frame_queue, &entry);

            pio[0]->txf[sm[2]] = eof_signal_duration;
            pio[1]->txf[sm[3]] = eof_signal_duration;

            frame_counter = 0;
        }
    }
}
/**
 * @brief Initializes the frame advance signal GPIO and ISR.
 *
 * Configures interrupts on ADVANCE_FRAME_PIN and initializes state.
 */
void init_signals(void)
{
    gpio_set_irq_callback(gpio_irq_callback_isr);

    irq_status = gpio_get_all() & (1 << ADVANCE_FRAME_PIN) ? 1 : 0;
    gpio_set_irq_enabled((uint)ADVANCE_FRAME_PIN,
                         irq_status ? (uint32_t)(GPIO_IRQ_EDGE_FALL) : (uint32_t)(GPIO_IRQ_EDGE_RISE),
                         true);
    if (!irq_is_enabled(IO_IRQ_BANK0))
    {
        irq_set_enabled(IO_IRQ_BANK0, true);
    };
    printf("HELLO pin state %s  OUT_level=%s level=%d\n", gpio_get_dir(ADVANCE_FRAME_PIN) ? "out" : "in", gpio_get_out_level(ADVANCE_FRAME_PIN) ? "high" : "low", gpio_get(ADVANCE_FRAME_PIN));

    printf("HELLO pin PASS_ON_FRAME_ADVANCE_PIN state %s  OUT_level=%s level=%d\n", gpio_get_dir(PASS_ON_FRAME_ADVANCE_PIN) ? "out" : "in", gpio_get_out_level(PASS_ON_FRAME_ADVANCE_PIN) ? "high" : "low", gpio_get(PASS_ON_FRAME_ADVANCE_PIN));
}

/**
 * @brief Main function for initialization and running of threads and interrupts.
 *
 * Initializes GPIO, interrupts, and launches secondary core for display updates.
 *
 * @return int Returns PICO_OK on successful execution.
 */
int main()
{
    // Overclock pico to 250000khz.
    set_sys_clock_khz(250000, true);

    stdio_init_all();

    critical_section_init(&cs1);

    // Initialize frame_queue - only one queue element allowed.
    int lock_num = spin_lock_claim_unused(true);
    queue_init_with_spinlock(&frame_queue, sizeof(queue_entry_t), 1, lock_num);

    multicore_reset_core1();
    // Launch main routine of core 1.
    multicore_launch_core1(core1_entry);

    led_init();

    frame_signal_duration = (clock_get_hz(clk_sys) * (FRAME_ADVANCE_DURATION_US / 1000000.0)) - 1;
    eof_signal_duration = (clock_get_hz(clk_sys) * (END_OF_FILM_DURATION_US / 1000000.0)) - 1;

    pio_setup(&frame_signal_program, &pio[0], &sm[0], &offset[0], PICO_DEFAULT_LED_PIN);
    pio_setup(&frame_signal_program, &pio[1], &sm[1], &offset[1], PASS_ON_FRAME_ADVANCE_PIN);
    pio_setup(&frame_signal_program, &pio[2], &sm[2], &offset[2], PICO_DEFAULT_LED_PIN);
    pio_setup(&frame_signal_program, &pio[3], &sm[3], &offset[3], PASS_ON_END_OF_FILM_PIN);

    // Initialize pins.
    init_pins();

    // Initialize frame advance and end-of-film signals.
    init_signals();

    while (1)
    {
    }

    return 0;
}