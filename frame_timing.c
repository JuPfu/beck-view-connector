#include "stdio.h"

#include "frame_timing.h"

static absolute_time_t start_time, frame_start, frame_end;

void init_frame_timing()
{
    start_time = frame_start = frame_end = get_absolute_time();
}

void process_frame_timing(queue_entry_t *entry, uint frame_counter, uint cmd)
{
    frame_start = frame_end;
    frame_end = get_absolute_time();

    entry->frame_start = frame_start;
    entry->frame_end = frame_end;
    entry->start_time = start_time;
    entry->frame_counter = frame_counter;
    entry->cmd = cmd;
}
