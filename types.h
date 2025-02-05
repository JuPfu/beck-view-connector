#ifndef BECK_VIEW_CONTROLLER_TYPES_H
#define BECK_VIEW_CONTROLLER_TYPES_H

#include "pico/time.h"

typedef struct __attribute__((aligned(4))) Queue_Entry
{
    absolute_time_t frame_start;
    absolute_time_t frame_end;
    absolute_time_t start_time;
    uint frame_counter;
    uint cmd;
} queue_entry_t;
#endif