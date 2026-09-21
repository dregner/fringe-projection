#pragma once
#include <string>
#include <chrono>
#include <cstdint>
#include <vector>
#include "StereoConfig.hpp"

struct gpiod_chip;
struct gpiod_line;

namespace stereo {

class GpioController {
public:
    GpioController();
    ~GpioController();

    bool init(const GPIOTriggerConfig& trigCfg, const ZNCCConfig& znccCfg);
    void release();

    // Trigger functionality
    bool generatePulse(unsigned int duration_us = 100);

    // ZNCC functionality
    void setLaser(bool on);
    void moveMotor(float angle);

    bool isInitialized() const { return m_initialized; }

private:
    struct gpiod_chip* m_chip0{nullptr};
    struct gpiod_chip* m_chip1{nullptr};
    
    // Trigger lines (from chip0)
    std::vector<struct gpiod_line*> m_trigger_lines;
    
    // ZNCC lines
    struct gpiod_line* m_laser_line{nullptr};
    std::vector<struct gpiod_line*> m_motor_lines;
    
    bool m_activeLow{false};
    bool m_initialized{false};
    
    std::vector<std::vector<int>> m_step_sequence;
    int m_delay_us;
    int m_steps_per_rev;
};

} // namespace stereo
