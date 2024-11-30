#ifndef DISPLAY_H
#define DISPLAY_H

#include "ili9341/ili9341.h"

// Initialize the display module
void display_init(ili9341_config_t *hw_config);

void display_start_info(void);

// Update the display with frame and timing information
void display_frame_info(uint frame_counter, uint64_t fps_integer, uint64_t fps_fraction, uint64_t duration_integer, uint64_t duration_fraction);

// Show the end-of-film statistics
void display_eof_info(uint frame_counter, float avg_fps, float duration);

#endif // DISPLAY_H
