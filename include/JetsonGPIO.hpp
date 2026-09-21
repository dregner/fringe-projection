#pragma once

#include <string>
#include <chrono>
#include <cstdint>
#include <vector>

struct gpiod_chip;
struct gpiod_line;

namespace stereo {

enum class JetsonModel {
    JETSON_ORIN_NANO_NX,
    JETSON_AGX_ORIN,
    CUSTOM_OR_DIRECT_GPIO
};

enum class PinType {
    HEADER_PIN,
    LINUX_GPIO_NUM
};

class JetsonGPIO {
public:
    JetsonGPIO();
    explicit JetsonGPIO(const std::vector<int>& pins, 
                       PinType pinType = PinType::HEADER_PIN, 
                       JetsonModel model = JetsonModel::JETSON_ORIN_NANO_NX,
                       bool activeLow = false);
    ~JetsonGPIO();

    JetsonGPIO(const JetsonGPIO&) = delete;
    JetsonGPIO& operator=(const JetsonGPIO&) = delete;

    JetsonGPIO(JetsonGPIO&& other) noexcept;
    JetsonGPIO& operator=(JetsonGPIO&& other) noexcept;

    bool init(const std::vector<int>& pins, 
              PinType pinType = PinType::HEADER_PIN, 
              JetsonModel model = JetsonModel::JETSON_ORIN_NANO_NX,
              bool activeLow = false);

    bool write(bool high);
    bool setHigh();
    bool setLow();
    bool generatePulse(unsigned int duration_us = 100);
    void release();

    std::vector<int> getGpioNumbers() const { return m_original_pins; }
    bool isInitialized() const { return m_initialized; }

    static int mapHeaderPinToGpio(int headerPin, JetsonModel model);

private:
    struct gpiod_chip* m_chip{nullptr};
    std::vector<struct gpiod_line*> m_lines;
    std::vector<int> m_original_pins;

    bool m_activeLow{false};
    bool m_initialized{false};
};

} // namespace stereo
