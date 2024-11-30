Here’s a draft for the `README.md` file for the **beck-view-connector** sub-project:

---

# Beck View Connector

**Beck View Connector** is a sub-project within the broader [Beck View Digitize](https://github.com/JuPfu/beck-view-digitize) initiative. This project provides an interface and processing platform for a **Super 8 projector** using a **Raspberry Pi Pico**. It enables the detection, synchronization, and handling of frame advancements and end-of-film events, with real-time data displayed on a connected screen.

---

## Features

- **Frame Advancement Detection:**  
  Processes signals from the projector to track frame advancements accurately.

- **End-of-Film Detection:**  
  Automatically detects when the film ends and updates the display accordingly.

- **Real-Time Display:**  
  Integrates with an ILI9341 TFT screen to provide frame timing, frames-per-second (FPS), and other statistics.

- **Debounced GPIO Handling:**  
  Ensures stable signal processing with debouncing for all GPIO pin events.

- **Dual-Core Utilization:**  
  Leverages both cores of the Raspberry Pi Pico, dedicating one core for display updates.

- **Customizable Timing Configurations:**  
  Configurable delays for frame and end-of-film signal processing.

---

## Requirements

- **Hardware:**
  - Raspberry Pi Pico
  - ILI9341 TFT Display (or compatible display)
  - Super 8 projector with frame and end-of-film signal outputs
  - GPIO wiring for projector integration

- **Software:**
  - Pico SDK
  - CMake (for building)
  - A compatible toolchain for Raspberry Pi Pico (e.g., `arm-none-eabi-gcc`)

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
- Contributors to the **Beck View Digitize** project.

For more details on the parent project, visit the [Beck View Digitize repository](https://github.com/JuPfu/beck-view-digitize).

--- 

Let me know if you need any adjustments!
