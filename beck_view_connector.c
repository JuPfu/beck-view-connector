/**
 * @file beck-view-connector.c
 * @brief Super 8 projector connector using Raspberry Pi Pico and C.
 */

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/util/queue.h"
#include "stdio.h"
#include "string.h"

#include "ili9341/ili9341.h"
#include <stdbool.h>

#include "pt_v1_3.h"

#include "display.h"
#include "frame_timing.h"

#include "frame_signal.pio.h"

// Define GPIO pins for frame advance and end-of-film signals
#define ADVANCE_FRAME_PIN 4 ///< GPIO pin for frame advance signal input.
#define END_OF_FILM_PIN 5   ///< GPIO pin for end-of-film signal input.

#define PASS_ON_FRAME_ADVANCE_PIN 2 //< GPIO pin to propagate frame advance signal.
#define PASS_ON_END_OF_FILM_PIN 3   ///< GPIO pin to propagate end-of-film signal.

// Define durations for signals and debounce delays
#define FRAME_ADVANCE_DURATION_US 8000  ///< Duration in microseconds to maintain frame advance signal.
#define END_OF_FILM_DURATION_US 1000000 ///< Duration in microseconds to maintain end-of-film signal.

#define DEBOUNCE_DELAY_US 1000 ///< Debounce duration for signal edges.

// Commands for the queue to manage updates and events
#define UPDATE_CMD 1 ///< Command for updating display.
#define EOF_CMD 2    ///< Command for end-of-film handling.

// PIO (Programmable I/O) setup for signal handling
static PIO pio[4];     ///< Array of PIO instances.
static uint sm[4];     ///< Array of state machines for PIO.
static uint offset[4]; ///< Offset addresses for PIO programs.

static uint32_t frame_signal_duration = 0; ///< Frame advance signal duration in system clock cycles.
static uint32_t eof_signal_duration = 0;   ///< End-of-film signal duration in system clock cycles.

static uint frame_counter = 0; ///< Frame counter to track total frames processed.

static volatile uint irq_status = 0; ///< Stores IRQ status for frame advance signal.

static queue_t frame_queue; ///< Queue for frame timing data and command exchange between cores.

static struct pt pt; ///< Protothread control structure.

critical_section_t cs1; ///< Critical section to protect shared resources.

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
        // Wait for an entry to be available in the queue
        PT_WAIT_UNTIL(pt, queue_try_remove(&frame_queue, &queue_entry));

        if (queue_entry.cmd == UPDATE_CMD)
        {
            // Calculate FPS and duration for the frame
            uint64_t time_diff_us = absolute_time_diff_us(queue_entry.frame_start, queue_entry.frame_end);
            float fps = 1.0e6 / time_diff_us;
            float duration = time_diff_us / 1.0e6;

            // Display frame information
            display_frame_info(queue_entry.frame_counter, fps, duration);
        }
        else if (queue_entry.cmd == EOF_CMD)
        {
            // Calculate average FPS and total duration for end-of-film
            uint64_t time_diff = absolute_time_diff_us(queue_entry.start_time, queue_entry.frame_end);
            float avg_fps = (1.0e6 / time_diff) * queue_entry.frame_counter;
            float duration = time_diff / 1.0e6;

            // Display end-of-film information
            display_eof_info(queue_entry.frame_counter, avg_fps, duration);
        }
        else
        {
            // Handle invalid commands
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
    ili9341_config_t hw_config;
    display_init(&hw_config);      // Initialize the display
    display_start_info();          // Display startup information
    PT_INIT(&pt);                  // Initialize the protothread
    pt_add_thread(update_display); // Add the display update thread
    pt_schedule_start;             // Start protothread scheduling
}

/**
 * @brief Initialize the LED pin and provide a blinking status.
 *
 * @return int Returns PICO_OK on successful initialization.
 */
int led_init(void)
{
    gpio_init(PICO_DEFAULT_LED_PIN);              // Initialize default LED pin
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT); // Set as output

    for (int i = 0; i < 5; i++)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, true);  // Turn LED on
        sleep_ms(500);                         // Wait 500ms
        gpio_put(PICO_DEFAULT_LED_PIN, false); // Turn LED off
        sleep_ms(500);                         // Wait 500ms
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
    gpio_init(gpio);                                    // Initialize the GPIO pin
    gpio_set_dir(gpio, is_output ? GPIO_OUT : GPIO_IN); // Configure as input or output
    gpio_set_pulls(gpio, pull_up, !pull_up);
    if (is_output)
    {
        gpio_put(gpio, true); // Set output level to high if output
    }
}

/**
 * @brief Initialize the GPIO pins for frame advance and end-of-film signals.
 */
void init_pins()
{
    init_gpio_pin(ADVANCE_FRAME_PIN, false, false);       // Configure frame advance pin as input
    init_gpio_pin(END_OF_FILM_PIN, false, false);         // Configure end-of-film pin as input
    init_gpio_pin(PASS_ON_FRAME_ADVANCE_PIN, true, true); // Configure pass-on frame advance pin as output
    init_gpio_pin(PASS_ON_END_OF_FILM_PIN, true, true);   // Configure pass-on end-of-film pin as output
}

/**
 * @brief Configures a PIO instance and state machine for signal generation.
 *
 * @param program Pointer to the PIO program.
 * @param pio Pointer to the PIO instance.
 * @param sm Pointer to the state machine.
 * @param offset Pointer to the program offset.
 * @param gpio_base Base GPIO pin for the PIO program.
 */
void pio_setup(const pio_program_t *program, PIO *pio, uint *sm, uint *offset, uint gpio_base)
{
    // Claim a free PIO and state machine, and load the program for the specified GPIO range
    bool rc = pio_claim_free_sm_and_add_program_for_gpio_range(program, pio, sm, offset, gpio_base, 1, true);
    hard_assert(rc);                                          // Ensure PIO program setup succeeded
    frame_signal_program_init(*pio, *sm, *offset, gpio_base); // Initialize the PIO program
    pio_sm_set_enabled(*pio, *sm, true);                      // Enable the state machine
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
    // Enter critical section to protect shared resources
    critical_section_enter_blocking(&cs1);

    // Determine the current IRQ status and set the appropriate edge detection
    irq_status = gpio_get(ADVANCE_FRAME_PIN);
    gpio_set_irq_enabled(ADVANCE_FRAME_PIN, irq_status ? GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE, true);

    critical_section_exit(&cs1); // Exit critical section
    return 0;
}

/**
 * @brief GPIO interrupt service routine to handle signal events.
 *
 * Handles frame advance and end-of-film signals, processes timing data,
 * and queues updates for the display.
 *
 * @param gpio GPIO pin triggering the interrupt.
 * @param event_mask Mask indicating the type of event (rise/fall).
 */
void gpio_irq_callback_isr(uint gpio, uint32_t event_mask)
{
    gpio_acknowledge_irq(gpio, event_mask); // Acknowledge the interrupt

    if (gpio == ADVANCE_FRAME_PIN)
    {
        critical_section_enter_blocking(&cs1);
        gpio_set_irq_enabled(ADVANCE_FRAME_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
        if (event_mask & GPIO_IRQ_EDGE_FALL)
        {
            if (irq_status == 1) // If a valid signal is detected
            {
                irq_status = 3; // Update IRQ status
                if (frame_counter == 0)
                {
                    init_frame_timing(); // Initialize frame timing at the start
                    // Enable end-of-film detection
                    gpio_set_irq_enabled(END_OF_FILM_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
                }

                queue_entry_t entry;
                process_frame_timing(&entry, frame_counter, UPDATE_CMD); // Process frame timing data

                // Check if the frame timing is too fast (debounce protection)
                int64_t delta = absolute_time_diff_us(entry.frame_start, entry.frame_end);
                if (delta < DEBOUNCE_DELAY_US)
                {
                    printf("Alert --- too fast, too furious %u %llu us\n", entry.frame_counter, delta);
                }

                queue_add_blocking(&frame_queue, &entry); // Add processed data to the queue

                // Trigger PIO for frame advance signal
                pio[0]->txf[sm[0]] = frame_signal_duration;
                pio[1]->txf[sm[1]] = frame_signal_duration;

                // Re-enable IRQ after debounce delay
                uint64_t edge_rise_alarm_id = add_alarm_in_us(DEBOUNCE_DELAY_US, enable_frame_advance_edge_irq, NULL, false);
                if (edge_rise_alarm_id < 0)
                {
                    printf("Edge_rise_alarm_id Alarm error %llu\n", edge_rise_alarm_id);
                }

                frame_counter++; // Increment the frame counter
            }
        }
        else if (event_mask & GPIO_IRQ_EDGE_RISE)
        {
            if (irq_status == 0) // Detect valid rising edge
            {
                irq_status = 2; // Update IRQ status
                // Re-enable IRQ after debounce delay
                uint64_t edge_fall_alarm_id = add_alarm_in_us(DEBOUNCE_DELAY_US, enable_frame_advance_edge_irq, NULL, false);
                if (edge_fall_alarm_id < 0)
                {
                    printf("Edge_rise_alarm_id Alarm error %lld\n", edge_fall_alarm_id);
                }
            }
        }

        critical_section_exit(&cs1); // Exit critical section
    }
    else if (gpio == END_OF_FILM_PIN)
    {
        if (event_mask & GPIO_IRQ_EDGE_RISE)
        {
            // Disable further end-of-film interrupts
            gpio_set_irq_enabled(END_OF_FILM_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);

            queue_entry_t entry;
            process_frame_timing(&entry, frame_counter, EOF_CMD); // Process end-of-film timing data

            queue_add_blocking(&frame_queue, &entry); // Add processed data to the queue

            // Trigger PIO for end-of-film signal
            pio[0]->txf[sm[2]] = eof_signal_duration;
            pio[1]->txf[sm[3]] = eof_signal_duration;

            frame_counter = 0; // Reset frame counter
        }
    }
    else
    {
        printf("Unhandled GPIO interrupt on pin %u\n", gpio);
    }
}

/**
 * @brief Initializes the frame advance signal GPIO and ISR.
 *
 * Configures interrupts on ADVANCE_FRAME_PIN and initializes state.
 */
void init_signals(void)
{
    gpio_set_irq_callback(gpio_irq_callback_isr); // Set GPIO ISR callback

    // Initialize IRQ status and configure edge detection
    irq_status = gpio_get(ADVANCE_FRAME_PIN);
    gpio_set_irq_enabled(ADVANCE_FRAME_PIN, irq_status ? GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE, true);

    // Enable IRQ for the bank if not already enabled
    if (!irq_is_enabled(IO_IRQ_BANK0))
    {
        irq_set_enabled(IO_IRQ_BANK0, true);
    }
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
    // Overclock Pico to 250 MHz
    set_sys_clock_khz(250000, true);

    stdio_init_all(); // Initialize standard I/O

    critical_section_init(&cs1); // Initialize critical section

    // Initialize frame queue with a spinlock
    int lock_num = spin_lock_claim_unused(true);
    queue_init_with_spinlock(&frame_queue, sizeof(queue_entry_t), 1, lock_num);

    multicore_reset_core1();             // Reset core 1
    multicore_launch_core1(core1_entry); // Launch core 1 entry function

    led_init(); // Initialize and blink LED as a status indicator

    // Calculate signal durations in clock cycles
    frame_signal_duration = (clock_get_hz(clk_sys) * (FRAME_ADVANCE_DURATION_US / 1.0e6)) - 2;
    eof_signal_duration = (clock_get_hz(clk_sys) * (END_OF_FILM_DURATION_US / 1.0e6)) - 2;

    // Initialize GPIO pins
    init_pins();

    // Setup PIO for signal generation
    pio_setup(&frame_signal_program, &pio[0], &sm[0], &offset[0], PICO_DEFAULT_LED_PIN);
    pio_setup(&frame_signal_program, &pio[1], &sm[1], &offset[1], PASS_ON_FRAME_ADVANCE_PIN);
    pio_setup(&frame_signal_program, &pio[2], &sm[2], &offset[2], PICO_DEFAULT_LED_PIN);
    pio_setup(&frame_signal_program, &pio[3], &sm[3], &offset[3], PASS_ON_END_OF_FILM_PIN);

    // gpio_put(ADVANCE_FRAME_PIN, true);
    // Initialize signals and their interrupts
    init_signals();

    // Main loop (can add application-specific tasks here)
    while (1)
    {
    }

    return 0;
}
