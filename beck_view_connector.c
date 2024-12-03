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
#include "pico/divider.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/ticks.h"
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

#define DEBOUNCE_DELAY_US 1500 ///< Debounce duration for signal edges.

#define UPDATE_CMD 1 ///< Command for updating display.
#define EOF_CMD 2    ///< Command for end-of-film handling.

void init_frame_advance_signal();
void init_end_of_film_signal();

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

/// Scale factor for fixed-point calculations (e.g., FPS with precision).
#define SCALE_FACTOR 1000 // For three decimal places

/**
 * @brief Protothread for updating the display with frame timing and status information.
 *
 * @param pt Pointer to the protothread control structure.
 * @return Protothread state.
 */
static PT_THREAD(update_display(struct pt *pt))
{
    PT_BEGIN(pt);

    queue_entry_t entry;

    while (1)
    {
        PT_WAIT_UNTIL(pt, queue_try_peek(&frame_queue, &entry));

        if (!queue_try_remove(&frame_queue, NULL))
        {
            printf("Update display failed to remove element from queue");
        }

        if (entry.cmd == UPDATE_CMD)
        {
            // absolute_time_t tmp = get_absolute_time();

            // FPS Calculation
            uint64_t time_diff = absolute_time_diff_us(entry.frame_start, entry.frame_end);
            uint64_t fps_scaled = (1000000 * SCALE_FACTOR) / time_diff;
            uint64_t fps_integer = fps_scaled / SCALE_FACTOR;
            uint64_t fps_fraction = fps_scaled % SCALE_FACTOR;

            // Duration Calculation
            uint64_t duration_scaled = time_diff / 1000;        // in milliseconds
            uint64_t duration_integer = duration_scaled / 1000; // in seconds
            uint64_t duration_fraction = duration_scaled % 1000;

            display_frame_info(entry.frame_counter, fps_integer, fps_fraction, duration_integer, duration_fraction);

            // absolute_time_t end = get_absolute_time();
            // printf("Update Display for frame=%u took tmp=%10llu %9.6f seconds\n", entry.frame_counter, tmp, (absolute_time_diff_us(tmp, end)) / 1.0e6);
        }
        else if (entry.cmd == EOF_CMD)
        {
            uint64_t time_diff = absolute_time_diff_us(entry.start_time, entry.frame_end);
            float avg_fps = (1.0e6 / time_diff) * entry.frame_counter;
            float duration = time_diff / 1.0e6;

            display_eof_info(entry.frame_counter, avg_fps, duration);
        }
        else
        {
            printf("Invalid command %u\n", entry.cmd);
        }
    }

    PT_END(pt);
}

/// Protothread control structure.
static struct pt pt;

/**
 * @brief Secondary core entry point for display-related tasks.
 */
void core1_entry()
{
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
    init_gpio_pin(ADVANCE_FRAME_PIN, false, false);
    init_gpio_pin(END_OF_FILM_PIN, false, false);
    init_gpio_pin(PASS_ON_FRAME_ADVANCE_PIN, true, true);
    init_gpio_pin(PASS_ON_END_OF_FILM_PIN, true, true);
}

void signal_frame_advance(PIO pio, uint sm, uint offset, uint pin)
{
    frame_signal_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
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
    signal_frame_advance(*pio, *sm, *offset, gpio_base);
}

static struct pt pt;

/**
 * @brief Main function for initialization and running of threads and interrupts.
 *
 * Initializes GPIO, interrupts, and launches secondary core for display updates.
 *
 * @return int Returns PICO_OK on successful execution.
 */
int main()
{
    // overclock pico to 250000khz
    set_sys_clock_khz(250000, true);

    stdio_init_all();

    // initialize frame_queue - only one queue element allowed
    queue_init(&frame_queue, sizeof(queue_entry_t), 1);

    multicore_reset_core1();
    // launch main routine of core 1
    multicore_launch_core1(core1_entry);

    led_init();

    frame_signal_duration = (clock_get_hz(clk_sys) * (FRAME_ADVANCE_DURATION_US / 1000000.0)) - 1;
    eof_signal_duration = (clock_get_hz(clk_sys) * (END_OF_FILM_DURATION_US / 1000000.0)) - 1;

    pio_setup(&frame_signal_program, &pio[0], &sm[0], &offset[0], PICO_DEFAULT_LED_PIN);
    pio_setup(&frame_signal_program, &pio[1], &sm[1], &offset[1], PASS_ON_FRAME_ADVANCE_PIN);
    pio_setup(&frame_signal_program, &pio[2], &sm[2], &offset[2], PICO_DEFAULT_LED_PIN);
    pio_setup(&frame_signal_program, &pio[3], &sm[3], &offset[3], PASS_ON_END_OF_FILM_PIN);

    // initialize pins
    init_pins();

    // initialize frame advance and end-of-film signals
    init_frame_advance_signal();
    init_end_of_film_signal();

    while (1)
    {
    }

    return 0;
}

int64_t enable_frame_advance_edge_fall_irq(alarm_id_t id, void *user_data)
{
    irq_status = 1;

    gpio_set_irq_enabled(ADVANCE_FRAME_PIN, GPIO_IRQ_EDGE_FALL, true);
    return 0;
}

/**
 * @brief Enables rising edge interrupt for frame advance signal after debounce delay.
 *
 * @param id Alarm ID (unused).
 * @param user_data Additional user data (unused).
 * @return int64_t Always returns 0.
 */
int64_t enable_frame_advance_edge_rise_irq(alarm_id_t id, void *user_data)
{
    irq_status = 0;

    gpio_set_irq_enabled(ADVANCE_FRAME_PIN, GPIO_IRQ_EDGE_RISE, true);
    return 0;
}

/**
 * @brief Interrupt Service Routine (ISR) for the frame advance signal.
 *
 * Handles both rising and falling edge events on the ADVANCE_FRAME_PIN.
 */
void advance_frame_signal_isr(void)
{
    uint32_t event_mask = gpio_get_irq_event_mask(ADVANCE_FRAME_PIN);

    if (event_mask & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(ADVANCE_FRAME_PIN, event_mask);

        // Temporarily disable IRQ
        gpio_set_irq_enabled(ADVANCE_FRAME_PIN, GPIO_IRQ_EDGE_FALL, false);

        if (irq_status == 1)
        {
            if (frame_counter == 0)
            {
                init_frame_timing();
                // enable end of film detection
                gpio_set_irq_enabled(END_OF_FILM_PIN, GPIO_IRQ_EDGE_RISE, true);
            }

            // pass timing information to core1
            queue_entry_t entry;

            process_frame_timing(&entry, frame_counter, UPDATE_CMD);

            if (!queue_try_add(&frame_queue, &entry))
            {
                printf("Alert --- Frame queue overflow %9.6f\n", (float)(absolute_time_diff_us(entry.frame_start, entry.frame_end) / 1000.0));
            }

            pio[0]->txf[sm[0]] = frame_signal_duration;
            pio[1]->txf[sm[1]] = frame_signal_duration;

            // debounce edge detection for DEBOUNCE_DELAY_US - enable IRQ again
            uint64_t edge_rise_alarm_id = add_alarm_in_us(DEBOUNCE_DELAY_US, enable_frame_advance_edge_rise_irq, NULL, false);
            if (edge_rise_alarm_id < 0)
            {
                printf("edge_rise_alarm_id Alarm error %llu\n", edge_rise_alarm_id);
            }

            frame_counter++;
        }
    }
    else if (event_mask & GPIO_IRQ_EDGE_RISE)
    {
        gpio_acknowledge_irq(ADVANCE_FRAME_PIN, event_mask);

        // Temporarily disable IRQ
        gpio_set_irq_enabled(ADVANCE_FRAME_PIN, GPIO_IRQ_EDGE_RISE, false);

        if (irq_status == 0)
        {
            // debounce edge detection - enable IRQ again after DEBOUNCE_DELAY_US
            uint64_t alarm_id = add_alarm_in_us(DEBOUNCE_DELAY_US, enable_frame_advance_edge_fall_irq, NULL, false);
            if (alarm_id < 0)
            {
                printf("FALL Alarm error %lld\n", alarm_id);
            }
        }
    }
}

/**
 * @brief Initializes the frame advance signal GPIO and ISR.
 *
 * Configures interrupts on ADVANCE_FRAME_PIN and initializes state.
 */
void init_frame_advance_signal(void)
{
    gpio_add_raw_irq_handler(ADVANCE_FRAME_PIN, advance_frame_signal_isr);
    irq_status = gpio_get_all() & (1 << ADVANCE_FRAME_PIN) ? 1 : 0;
    printf("HELLO irq_status=%u  %lu\n", irq_status, gpio_get_all() & (1 << ADVANCE_FRAME_PIN));
    gpio_set_irq_enabled((uint)ADVANCE_FRAME_PIN,
                         irq_status ? (uint32_t)(GPIO_IRQ_EDGE_FALL) : (uint32_t)(GPIO_IRQ_EDGE_RISE),
                         true);
    printf("HELLO pin state %s  OUT_level=%s level=%d\n", gpio_get_dir(ADVANCE_FRAME_PIN) ? "out" : "in", gpio_get_out_level(ADVANCE_FRAME_PIN) ? "high" : "low", gpio_get(ADVANCE_FRAME_PIN));

    printf("HELLO pin PASS_ON_FRAME_ADVANCE_PIN state %s  OUT_level=%s level=%d\n", gpio_get_dir(PASS_ON_FRAME_ADVANCE_PIN) ? "out" : "in", gpio_get_out_level(PASS_ON_FRAME_ADVANCE_PIN) ? "high" : "low", gpio_get(PASS_ON_FRAME_ADVANCE_PIN));
}

/**
 * @brief Interrupt Service Routine (ISR) for the end-of-film signal.
 *
 * Handles rising edge events on the END_OF_FILM_PIN.
 */
void end_of_film_signal_isr(void)
{
    if (gpio_get_irq_event_mask(END_OF_FILM_PIN) & GPIO_IRQ_EDGE_RISE)
    {
        gpio_acknowledge_irq(END_OF_FILM_PIN, GPIO_IRQ_EDGE_RISE);

        // Disable IRQ
        gpio_set_irq_enabled(END_OF_FILM_PIN, GPIO_IRQ_EDGE_RISE, false);

        queue_entry_t entry;

        process_frame_timing(&entry, frame_counter, EOF_CMD);

        if (!queue_try_add(&frame_queue, &entry))
        {
            printf("Alert --- Frame queue overflow %9.6f\n", (float)(absolute_time_diff_us(entry.frame_start, entry.frame_end) / 1000.0));
        }

        pio[0]->txf[sm[2]] = eof_signal_duration;
        pio[1]->txf[sm[3]] = eof_signal_duration;

        frame_counter = 0;
    }
}

/**
 * @brief Initializes the end-of-film signal GPIO and ISR.
 *
 * Configures interrupts on END_OF_FILM_PIN for rising edges.
 */
void init_end_of_film_signal(void)
{
    gpio_add_raw_irq_handler(END_OF_FILM_PIN, end_of_film_signal_isr);

    if (!irq_is_enabled(IO_IRQ_BANK0))
    {
        irq_set_enabled(IO_IRQ_BANK0, true);
    };
}
