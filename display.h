#ifndef DISPLAY_H
#define DISPLAY_H

#include "pico/stdlib.h"

// Initialize the display module
void display_init();

void display_start_info(void);

// Update the display with frame and timing information
void display_frame_info(uint frame_counter, float fps, float duration);

// Show the end-of-film statistics
void display_eof_info(uint frame_counter, float avg_fps, float duration);

#endif // DISPLAY_H
