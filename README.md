# Spinnaker Stereo Camera System with Jetson Orin Hardware Trigger

A modular, production-ready C++ system utilizing the **Teledyne FLIR Spinnaker SDK** (`/opt/spinnaker/`) and **Jetson Orin GPIO** to achieve sub-microsecond hardware-synchronized stereo image capture.

---

## Key Features

1. **YAML-Driven Configuration (`config/stereo_config.yaml`)**:
   - Camera selection by serial number (or auto-enumeration).
   - Independent or unified manual exposure time (in µs) and auto-exposure toggle.
   - Independent or unified manual gain (in dB) and auto-gain toggle.
   - Hardware trigger configuration:
     - `mode`: Enable/disable trigger mode.
     - `source`: `Line0` (opto-isolated input), `Line1`, `Line2`, `Line3`, `Software`.
     - `selector`: `FrameStart` (standard for FLIR machine vision cameras).
     - `activation`: `RisingEdge`, `FallingEdge`, `AnyEdge`, `LevelHigh`, `LevelLow`.
     - `overlap`: `ReadOut` (sensor exposes next frame while reading current frame) or `Off`.
     - `delay_us`: Programmable trigger delay.
   - Acquisition and stream buffering: `StreamBufferHandlingMode = NewestOnly` (eliminates stale buffer latency).

2. **Synchronized Stereo Capture**:
   - Parallel asynchronous frame acquisition from Left and Right cameras using `std::async`.
   - Complete image integrity check (`IsIncomplete()` validation).
   - Timestamp and Frame ID extraction with hardware synchronization delta reporting in milliseconds.

3. **Jetson Orin GPIO Hardware Trigger Class (`stereo::JetsonGPIO`)**:
   - Supports NVIDIA Jetson AGX Orin, Jetson Orin NX, and Jetson Orin Nano 40-pin expansion headers.
   - Translates header pin numbers (e.g. Pin 7, Pin 11, Pin 12, etc.) to internal Linux GPIO numbers.
   - Direct Linux GPIO number specification supported.
   - Utilizes Linux sysfs with pre-opened and cached file descriptors (`pwrite`) to minimize trigger latency and jitter.
   - Sub-microsecond precision pulse generation via high-resolution busy-wait timer for pulses $\le 200\,\mu\text{s}$.

---

## Jetson Orin 40-Pin Header Pinout Mapping

| Header Pin | AGX Orin Linux GPIO | Orin Nano / NX Linux GPIO | Default Function |
| :---: | :---: | :---: | :---: |
| **Pin 7** | `398` | `348` | GPIO9 / PAC.06 (Recommended for Trigger) |
| **Pin 11** | `446` | `349` | UART1_RTS / PR.04 |
| **Pin 12** | `389` | `350` | I2S0_SCLK / PN.01 |
| **Pin 13** | `392` | `351` | SPI1_SCK / PAC.00 |
| **Pin 15** | `444` | `352` | GPIO12 / PR.02 |
| **Pin 18** | `394` | `354` | SPI1_CS0 / PAC.02 |
| **Pin 29** | `434` | `361` | GPIO01 / PQ.05 |
| **Pin 31** | `435` | `362` | GPIO11 / PQ.06 |
| **Pin 32** | `424` | `363` | GPIO07 / PP.00 |
| **Pin 33** | `436` | `364` | GPIO13 / PQ.07 |

---

## Wiring Diagram (Jetson Orin to FLIR Cameras)

```
 Jetson Orin 40-Pin Header                  FLIR Blackfly S / Oryx / Flea3
+--------------------------+               +--------------------------------+
|                          |               | Left Camera (GPIO / Hirose HR10)|
| Pin 7 (GPIO Out) --------+---------------+---> Pin 2 (Line 0 / Opto In)     |
|                          |   |           |     Pin 5 (Opto Ground)        |
| Pin 9 or 39 (GND) -------+---+-----------+---> Pin 6 (GND)                |
|                          |   |           +--------------------------------+
|                          |   |
|                          |   |           +--------------------------------+
|                          |   |           | Right Camera                   |
|                          |   +----------->---> Pin 2 (Line 0 / Opto In)     |
|                          +---------------+---> Pin 6 (GND)                |
+--------------------------+               +--------------------------------+
```

> **Note**: For FLIR Blackfly S cameras with opto-isolated inputs (Line0), verify the voltage requirement (typically 3.3V–24V). Jetson Orin 40-pin header logic level is 3.3V.

---

## Building the Project

Ensure dependencies are installed:
```bash
sudo apt update
sudo apt install -y libyaml-cpp-dev g++ cmake
```

Build using CMake:
```bash
cd /home/daniel/Documents/codes/fringe_process
cmake -B build -S .
cmake --build build -j$(nproc)
```

---

## Configuration (`config/stereo_config.yaml`)

```yaml
stereo_system:
  left_camera:
    serial_number: "21045678"      # Left camera serial number
    auto_exposure: false
    exposure_time_us: 10000.0      # 10 ms exposure
    auto_gain: false
    gain_db: 0.0                   # 0 dB
    trigger:
      mode: true                   # Hardware trigger enabled
      source: "Line0"              # Hardware opto-isolated Line 0
      selector: "FrameStart"
      activation: "RisingEdge"
      overlap: "ReadOut"
      delay_us: 0.0

  right_camera:
    serial_number: "21045679"      # Right camera serial number
    auto_exposure: false
    exposure_time_us: 10000.0
    auto_gain: false
    gain_db: 0.0
    trigger:
      mode: true
      source: "Line0"
      selector: "FrameStart"
      activation: "RisingEdge"
      overlap: "ReadOut"
      delay_us: 0.0

  gpio_trigger:
    enabled: true                  # Enable Jetson GPIO pulse generation
    pin: 7                         # Header pin 7
    pin_type: "HEADER_PIN"         # "HEADER_PIN" or "LINUX_GPIO_NUM"
    jetson_model: "JETSON_ORIN_NANO_NX" # or "JETSON_AGX_ORIN"
    pulse_duration_us: 100         # 100 µs pulse
    active_low: false              # Active High (0 -> 1 -> 0)

  acquisition:
    timeout_ms: 2000
    buffer_handling_mode: "NewestOnly"
    pixel_format: ""               # Empty for default sensor format
```

---

## Running the Application

Grant GPIO permissions (or run with `sudo` for `/sys/class/gpio` access on Jetson):
```bash
# Using sudo for GPIO access
sudo ./build/stereo_capture --config config/stereo_config.yaml --frames 10

# Or without GPIO trigger pulses (listening for external signals or software trigger)
./build/stereo_capture --config config/stereo_config.yaml --no-gpio --frames 5
```

---

## C++ API Usage in Your Own Application

```cpp
#include "StereoCameraSystem.hpp"
#include "JetsonGPIO.hpp"

// 1. Initialize Stereo System from YAML
stereo::StereoCameraSystem stereoSystem;
if (!stereoSystem.initializeFromYaml("config/stereo_config.yaml")) {
    std::cerr << "Initialization failed!" << std::endl;
    return -1;
}

// 2. Initialize Jetson Orin GPIO
stereo::JetsonGPIO triggerPin(7, stereo::PinType::HEADER_PIN, stereo::JetsonModel::JETSON_ORIN_NANO_NX);

// 3. Trigger pulse and acquire synchronized stereo pair
stereo::StereoFrame frame = stereoSystem.triggerAndReceive(triggerPin, 100 /* us pulse */);

if (frame.valid) {
    std::cout << "Captured synchronized pair! Delta: " << frame.syncDeltaMs << " ms" << std::endl;
    frame.save("left.jpg", "right.jpg");
    
    // Release Spinnaker image buffers
    frame.leftImage->Release();
    frame.rightImage->Release();
}

// 4. Teardown
stereoSystem.release();
```
