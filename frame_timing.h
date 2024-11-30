#ifndef FRAME_TIMING_H
#define FRAME_TIMING_H

#include "pico/time.h"

typedef struct Queue_Entry
{
    absolute_time_t frame_start;
    absolute_time_t frame_end;
    absolute_time_t start_time;
    uint frame_counter;
    uint cmd;
} queue_entry_t;

// Initialize frame timing
void init_frame_timing(void);

// Handle frame timing calculations and update
void process_frame_timing(queue_entry_t *entry, uint frame_counter, uint cmd);

#endif // FRAME_TIMING_H
