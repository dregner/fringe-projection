#include "JetsonGPIO.hpp"
#include <iostream>
#include <unordered_map>
#include <vector>
#include <thread>
#include <gpiod.h>

namespace stereo {

// Jetson Orin Nano / Orin NX 40-pin header mapping to Tegra Line IDs (used by gpiod)
const std::unordered_map<int, int> ORIN_NANO_NX_PIN_MAP = {
    {7,  144}, {11, 112}, {12, 50},  {13, 122},
    {15, 85},  {16, 126}, {18, 125}, {19, 135},
    {21, 134}, {22, 123}, {23, 133}, {24, 136},
    {26, 137}, {29, 105}, {31, 106}, {32, 41},
    {33, 43},  {35, 53},  {36, 113}, {37, 124},
    {38, 52},  {40, 51}
};

int JetsonGPIO::mapHeaderPinToGpio(int headerPin, JetsonModel model) {
    if (model == JetsonModel::JETSON_ORIN_NANO_NX) {
        auto it = ORIN_NANO_NX_PIN_MAP.find(headerPin);
        if (it != ORIN_NANO_NX_PIN_MAP.end()) return it->second;
    }
    return headerPin; // Fallback
}

JetsonGPIO::JetsonGPIO() = default;

JetsonGPIO::JetsonGPIO(const std::vector<int>& pins, PinType pinType, JetsonModel model, bool activeLow) {
    init(pins, pinType, model, activeLow);
}

JetsonGPIO::~JetsonGPIO() {
    release();
}

JetsonGPIO::JetsonGPIO(JetsonGPIO&& other) noexcept
    : m_chip(other.m_chip),
      m_lines(std::move(other.m_lines)),
      m_original_pins(std::move(other.m_original_pins)),
      m_activeLow(other.m_activeLow),
      m_initialized(other.m_initialized) {
    other.m_chip = nullptr;
    other.m_initialized = false;
}

JetsonGPIO& JetsonGPIO::operator=(JetsonGPIO&& other) noexcept {
    if (this != &other) {
        release();
        m_chip = other.m_chip;
        m_lines = std::move(other.m_lines);
        m_original_pins = std::move(other.m_original_pins);
        m_activeLow = other.m_activeLow;
        m_initialized = other.m_initialized;
        
        other.m_chip = nullptr;
        other.m_initialized = false;
    }
    return *this;
}

bool JetsonGPIO::init(const std::vector<int>& pins, PinType pinType, JetsonModel model, bool activeLow) {
    release();
    m_activeLow = activeLow;
    m_original_pins = pins;

    // Open gpiochip0
    m_chip = gpiod_chip_open_by_name("gpiochip0");
    if (!m_chip) {
        std::cerr << "[JetsonGPIO] Error: Failed to open gpiochip0. Make sure libgpiod is installed and you have permissions." << std::endl;
        return false;
    }

    bool success = true;

    for (int pin : pins) {
        int line_offset = pin;
        if (pinType == PinType::HEADER_PIN) {
            line_offset = mapHeaderPinToGpio(pin, model);
        }

        struct gpiod_line* line = gpiod_chip_get_line(m_chip, line_offset);
        if (!line) {
            std::cerr << "[JetsonGPIO] Error: Failed to get line " << line_offset << std::endl;
            success = false;
            break;
        }

        // Request line for output
        int ret = gpiod_line_request_output(line, "StereoTrigger", m_activeLow ? 1 : 0);
        if (ret < 0) {
            std::cerr << "[JetsonGPIO] Error: Failed to request output on line " << line_offset << std::endl;
            success = false;
            break;
        }

        m_lines.push_back(line);
        std::cout << "[JetsonGPIO] Successfully acquired line " << line_offset << " via libgpiod." << std::endl;
    }

    if (!success || m_lines.empty()) {
        release();
        return false;
    }

    m_initialized = true;
    return true;
}

void JetsonGPIO::release() {
    if (m_initialized || !m_lines.empty()) {
        for (auto line : m_lines) {
            if (line) gpiod_line_release(line);
        }
        m_lines.clear();
    }
    if (m_chip) {
        gpiod_chip_close(m_chip);
        m_chip = nullptr;
    }
    m_initialized = false;
}

bool JetsonGPIO::write(bool high) {
    if (!m_initialized || m_lines.empty()) return false;
    
    int val = high ? 1 : 0;
    if (m_activeLow) val = !val;

    bool success = true;
    for (auto line : m_lines) {
        if (gpiod_line_set_value(line, val) < 0) {
            success = false;
        }
    }
    return success;
}

bool JetsonGPIO::setHigh() {
    return write(true);
}

bool JetsonGPIO::setLow() {
    return write(false);
}

bool JetsonGPIO::generatePulse(unsigned int duration_us) {
    if (!m_initialized || m_lines.empty()) return false;

    int high_val = m_activeLow ? 0 : 1;
    int low_val  = m_activeLow ? 1 : 0;

    // Set HIGH
    for (auto line : m_lines) {
        gpiod_line_set_value(line, high_val);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(duration_us));

    // Set LOW
    for (auto line : m_lines) {
        gpiod_line_set_value(line, low_val);
    }

    return true;
}

} // namespace stereo
