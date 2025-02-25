# Beck View Connector

**Beck View Connector** is a sub-project of the [Beck View Digitize](https://github.com/JuPfu/beck-view-digitize) project. This project connects the photoelectric sensors attached to the film projector to the computer running  [Beck View Digitize](https://github.com/JuPfu/beck-view-digitize). It enables the detection, synchronization, and handling of frame advancements and end-of-film events, with real-time data displayed on a connected screen.

![Prototype](./assets/img/beck-view-connector.png)
*Image: Prototype of the beck-view-connector which is located between the film projector and [Beck View Digitize](https://github.com/JuPfu/beck-view-digitize) 

---

## Overview

[beck-view-connector](https://github.com/JuPfu/beck-view-connector) connects the photoelectric sensor emitting frame advance signals and the photoelectric sensor emitting an end-of-film signal to the computer running [beck-view-digitize](https://github.com/JuPfu/beck-view-digitize). The generated binary program [beck-view-connector](https://github.com/JuPfu/beck-view-connector) has to be flashed to the Rasperry Pi Pico.

![beck-view](./assets/img/beck-view-overview.png)

## Features

**Frame Advancement Detection:**  
A photoelectric sensor triggers when a [rotating shutter](https://github.com/user-attachments/assets/5ff01cb6-593f-48e8-9ff4-b41a6fde65f2) does not block the sensor any more, which coincides with the current frame at rest in front of the film projector lens.
The frame advance signals from the photoelectric sensor are passed on to [Beck View Digitize](https://github.com/JuPfu/beck-view-digitize). Each frame advance signal is held for 8ms. Based on the frame advance signals [Beck View Digitize](https://github.com/JuPfu/beck-view-digitize) triggers the camera mounted in front of the film projector lens.

Here some more details of the process:
The projector shutter has a single blade of about 70 degrees. The maximum speed of the shutter is
24 frames per second. The time for a single frame is 1/24 = 41.67 ms. The shutter blade covers the
projector lens for about 70 degrees of the 360 degrees area. The time for the shutter blade
to cover the lens is 41.67 ms * 70 / 360 = 8.10 ms.
The shutter blade begins to cover the lens (the optocoupler). This is when the rising edge of the frame advance signal is emitted. During the EDGE_RISE_DEBOUNCE_DELAY_US time (2000 us) rising edge interrupts are disabled.
As soon as the lens is uncovered by the shutter blade, the falling edge of the frame advance signal is emitted and the frame advance signal is passed on to the FT232H chip (see [beck-view-digitalize](https://github.com/JuPfu/beck-view-digitalize)).
The signal to the FT232H chip is held high for FRAME_ADVANCE_DURATION_US (8000 us). The FT232H chip sends the frame advance signal to the PC.
There shall be no further edge fall interrupts while the frame advance signal is emitted to the FT232H chip. Therefore the debounce delay for the falling edge (EDGE_FALL_DEBOUNCE_DELAY_US) must be greater than FRAME_ADVANCE_DURATION_US.

**End-of-Film Detection:**  
A second photoelectric sensor is used to detect the end of film. The end of film signal is passed on to [Beck View Digitize](https://github.com/JuPfu/beck-view-digitize). The end of film signal is held for 1 second. On receiving the end of film signal [Beck View Digitize](https://github.com/JuPfu/beck-view-digitize) terminates.

**Real-Time Display:**  
Integrates with an ILI9341 TFT screen to provide frame timing, frames-per-second (FPS), and other statistics. The frame count displayed on the TFT screen after end of film is reached can be compared with the number of frames stored on disc.

**Debounced GPIO Handling:**  
Ensures stable signal processing with debouncing for all GPIO pin events.

**Dual-Core Utilization:**  
Leverages both cores of the Raspberry Pi Pico, dedicating one core for display updates.

**Customizable Timing Configurations:**  
Configurable delays for frame and end-of-film signal processing.

**Modular and Extensible:**  
Designed with a modular structure to allow easy integration and future enhancements.

---

## Requirements

**Hardware:**
- Raspberry Pi Pico
- ILI9341 TFT Display (or compatible display)
- A photoelectric sensor to trigger frame advance signals and a photoelectric sensor to emit an end-of-film signal
- GPIO wiring

**Software:**
- Recommended IDE for this sub-project is Visual Studio Code 
- The plugin <em>Raspberry Pi Pico</em> which also cares for the installation of a compatible toolchain for Raspberry Pi Pico (e.g., `arm-none-eabi-gcc`)

---

## Setup and Installation

### 1. Clone the Repository

  ```bash
  git clone https://github.com/JuPfu/beck-view-connector.git
  cd beck-view-connector
  ```

### 2. Build the Project

Ensure the **Pico SDK** is set up and its environment variables are configured. Then:

  ```bash
  mkdir build
  cd build
  cmake ..
  make
  ```

### 3. Flash to the Raspberry Pi Pico

After building, flash the `beck-view-connector.uf2` file to your Pico:

1. Hold the **BOOTSEL** button on the Pico while plugging it into your computer.
2. Copy the `.uf2` file onto the Pico's storage device.

  ---

## GPIO Configuration

  | GPIO Pin | Function                    |
  |----------|-----------------------------|
  | 4        | Frame Advance Signal Input  |
  | 5        | End-of-Film Signal Input    |
  | 2        | Pass Frame Advance Signal   |
  | 3        | Pass End-of-Film Signal     |
  | 25       | Status LED                  |

  ---

## Usage

1. **Power Up:** Connect the Raspberry Pi Pico to power. The LED should blink during initialization.
2. **Connect Signals:** Ensure the Super 8 projector signals are correctly connected to the specified GPIO pins.
3. **Observe Display:** The TFT screen will display real-time frame information and indicate when the film ends.

---

## Key Components

### Core Functionalities:

- **`advance_frame_signal_isr`**: Handles frame advance signal processing.
- **`end_of_film_signal_isr`**: Manages end-of-film detection.
- **`update_display`**: Protothread for refreshing the display with timing and status updates.

### Protothread System:

Leverages a lightweight threading system for efficient multitasking.

### Timing Configurations:

- `FRAME_ADVANCE_DELAY_US`: Delay to maintain the frame advance signal.
- `END_OF_FILM_DELAY_US`: Delay to maintain the end-of-film signal.
- `DEBOUNCE_DELAY_US`: Debounce timing for signal edges.

---

## Project Structure

```plaintext
  beck-view-connector/
  ├── src/
  │   ├── beck-view-connector.c   # Main implementation file
  │   ├── display.c               # Display handling logic
  │   ├── frame_timing.c          # Frame timing calculations
  │   └── pt_v1_3.h               # Protothreading system
  ├── include/
  │   └── *.h                     # Header files
  ├── CMakeLists.txt              # Build configuration
  └── README.md                   # Project documentation
```

---

## Contribution

Contributions are welcome! Please fork the repository, create a feature branch, and submit a pull request.

---

## License

This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- The **Raspberry Pi Foundation** for the hardware and SDK.
- The open-source community for providing valuable tools and libraries.

For more details on the parent project, visit the [Beck View Digitize repository](https://github.com/JuPfu/beck-view-digitize).

  ---
