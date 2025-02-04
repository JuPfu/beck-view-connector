#ifndef FRAME_TIMING_H
#define FRAME_TIMING_H

#ifdef __cplusplus
extern "C" {
#endif
#include "types.h"

// Initialize frame timing
void init_frame_timing(void);

// Handle frame timing calculations and update
void process_frame_timing(queue_entry_t *entry, uint frame_counter, uint cmd);

#ifdef __cplusplus
}
#endif

#endif // FRAME_TIMING_H
