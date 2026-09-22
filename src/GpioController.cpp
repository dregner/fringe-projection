#include "GpioController.hpp"
#include <iostream>
#include <unordered_map>
#include <thread>
#include <cmath>
#include <algorithm>

#ifdef HAS_GPIOD
#include <gpiod.h>
#endif

namespace stereo {

const std::unordered_map<int, int> ORIN_NANO_PIN_MAP = {
    {7,  144}, {11, 112}, {12, 50},  {13, 122},
    {15, 85},  {16, 126}, {18, 125}, {19, 135},
    {21, 134}, {22, 123}, {23, 133}, {24, 136},
    {26, 137}, {29, 105}, {31, 106}, {32, 41},
    {33, 43},  {35, 53},  {36, 113}, {37, 124},
    {38, 52},  {40, 51}
};
const std::unordered_map<int, int> ORIN_AGX_PIN_MAP = {
    {7,  106}, {11, 112}, {12, 50},  {13, 108},
    {15, 85},  {16, 9}, {18, 43}, {19, 135},
    {21, 134}, {22, 96}, {23, 133}, {24, 136},
    {26, 137}, {29, 1}, {31, 0}, {32, 8},
    {33, 2},  {35, 53},  {36, 113}, {37, 3},
    {38, 52},  {40, 51}
};

static int mapHeaderPinToGpio(int headerPin, const std::string& model) {
    if (model == "JETSON_ORIN_NANO_NX") {
        auto it = ORIN_NANO_PIN_MAP.find(headerPin);
        if (it != ORIN_NANO_PIN_MAP.end()) return it->second;
    }
    return headerPin;
}

GpioController::GpioController() {
    m_step_sequence = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
}

GpioController::~GpioController() {
    release();
}

bool GpioController::init(const GPIOTriggerConfig& trigCfg, const ZNCCConfig& znccCfg) {
    release();
    m_activeLow = trigCfg.activeLow;
    m_delay_us = znccCfg.delayUs;
    m_steps_per_rev = znccCfg.stepsPerRev;
    std::cout << m_steps_per_rev << std::endl;
#ifdef HAS_GPIOD
    m_chip0 = gpiod_chip_open_by_name("gpiochip0");
    if (!m_chip0) {
        std::cerr << "[GpioController] Error: Failed to open gpiochip0\n";
        return false;
    }
    m_chip1 = gpiod_chip_open_by_name("gpiochip1");
    if (!m_chip1) {
        std::cerr << "[GpioController] Error: Failed to open gpiochip1\n";
        gpiod_chip_close(m_chip0);
        m_chip0 = nullptr;
        return false;
    }

    // 1. Setup Trigger Pins (on chip0)
    for (int pin : trigCfg.pins) {
        int line_offset = pin;
        if (trigCfg.pinType == "HEADER_PIN") {
            line_offset = mapHeaderPinToGpio(pin, trigCfg.jetsonModel);
        }
        struct gpiod_line* line = gpiod_chip_get_line(m_chip0, line_offset);
        if (line) {
            if (gpiod_line_request_output(line, "StereoTrigger", m_activeLow ? 1 : 0) == 0) {
                m_trigger_lines.push_back(line);
                std::cout << "[GpioController] Trigger line " << line_offset << " configured.\n";
            }
        }
    }

    // 2. Setup Laser Pin (on chip0)
    m_laser_line = gpiod_chip_get_line(m_chip0, znccCfg.laserPin);
    if (m_laser_line) {
        if (gpiod_line_request_output(m_laser_line, "ZNCC_Laser", 0) == 0) {
            std::cout << "[GpioController] Laser line " << znccCfg.laserPin << " configured.\n";
        } else {
            m_laser_line = nullptr;
        }
    }

    // 3. Setup Motor Pins (on chip1)
    for (int pin : znccCfg.motorPins) {
        struct gpiod_line* line = gpiod_chip_get_line(m_chip1, pin);
        if (line) {
            if (gpiod_line_request_output(line, "ZNCC_Motor", 0) == 0) {
                m_motor_lines.push_back(line);
                std::cout << "[GpioController] Motor line " << pin << " configured.\n";
            }
        }
    }

    m_initialized = true;
    return true;
#else
    std::cout << "[GpioController] HAS_GPIOD not defined. Mock mode.\n";
    m_initialized = true;
    return true;
#endif
}

void GpioController::release() {
#ifdef HAS_GPIOD
    for (auto line : m_trigger_lines) {
        if (line) gpiod_line_release(line);
    }
    m_trigger_lines.clear();

    for (auto line : m_motor_lines) {
        if (line) {
            gpiod_line_set_value(line, 0);
            gpiod_line_release(line);
        }
    }
    m_motor_lines.clear();

    if (m_laser_line) {
        gpiod_line_set_value(m_laser_line, 0);
        gpiod_line_release(m_laser_line);
        m_laser_line = nullptr;
    }

    if (m_chip0) {
        gpiod_chip_close(m_chip0);
        m_chip0 = nullptr;
    }
    if (m_chip1) {
        gpiod_chip_close(m_chip1);
        m_chip1 = nullptr;
    }
#endif
    m_initialized = false;
}

bool GpioController::generatePulse(unsigned int duration_us) {
    if (!m_initialized) return false;

#ifdef HAS_GPIOD
    int high_val = m_activeLow ? 0 : 1;
    int low_val  = m_activeLow ? 1 : 0;

    for (auto line : m_trigger_lines) {
        gpiod_line_set_value(line, high_val);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(duration_us));
    for (auto line : m_trigger_lines) {
        gpiod_line_set_value(line, low_val);
    }
#else
    std::cout << "[GpioController] Mock Trigger Pulse\n";
#endif
    return true;
}

void GpioController::setLaser(bool on) {
    if (!m_initialized) return;
#ifdef HAS_GPIOD
    if(m_laser_line) {
        gpiod_line_set_value(m_laser_line, on ? 1 : 0);
    }
#else
    std::cout << "[GpioController] Mock Laser " << (on ? "ON" : "OFF") << "\n";
#endif
}

void GpioController::moveMotor(float angle) {
    if (!m_initialized) return;
    int steps = static_cast<int>((angle / 360.0f) * m_steps_per_rev);
    if (steps == 0) {
#ifdef HAS_GPIOD
        for (auto line : m_motor_lines) {
            gpiod_line_set_value(line, 0);
        }
#endif
        return;
    }

    bool clockwise = (steps > 0);
    steps = std::abs(steps);
    
    std::vector<std::vector<int>> current_seq = m_step_sequence;
    if (!clockwise) {
        std::reverse(current_seq.begin(), current_seq.end());
    }

    for (int i = 0; i < steps; ++i) {
        const auto& step = current_seq[i % current_seq.size()];
#ifdef HAS_GPIOD
        for (size_t j = 0; j < m_motor_lines.size() && j < step.size(); ++j) {
            gpiod_line_set_value(m_motor_lines[j], step[j]);
        }
#endif
        std::this_thread::sleep_for(std::chrono::microseconds(m_delay_us));
    }
}

} // namespace stereo
