#pragma once

#include <string>
#include <chrono>
#include <cstdint>

namespace stereo {

/**
 * @brief Jetson Orin model variant for pin mapping resolution.
 */
enum class JetsonModel {
    JETSON_ORIN_NANO_NX,
    JETSON_AGX_ORIN,
    CUSTOM_OR_DIRECT_GPIO
};

/**
 * @brief Pin specification type: direct Linux GPIO number vs 40-pin header pin.
 */
enum class PinType {
    HEADER_PIN,
    LINUX_GPIO_NUM
};

/**
 * @brief GPIO Pin Direction
 */
enum class PinDirection {
    IN,
    OUT
};

/**
 * @brief Low-latency, high-precision GPIO output controller for Jetson Orin.
 *
 * Supports hardware trigger pulse generation for synchronization of camera systems.
 * Uses Linux sysfs with cached open file descriptors to eliminate file open/close
 * overhead and minimize jitter during camera triggering.
 */
class JetsonGPIO {
public:
    JetsonGPIO();
    explicit JetsonGPIO(const std::vector<int>& pins, 
                       PinType pinType = PinType::HEADER_PIN, 
                       JetsonModel model = JetsonModel::JETSON_ORIN_NANO_NX,
                       bool activeLow = false);
    ~JetsonGPIO();

    // Prevent accidental copying to preserve file descriptor ownership
    JetsonGPIO(const JetsonGPIO&) = delete;
    JetsonGPIO& operator=(const JetsonGPIO&) = delete;

    // Allow move semantics
    JetsonGPIO(JetsonGPIO&& other) noexcept;
    JetsonGPIO& operator=(JetsonGPIO&& other) noexcept;

    /**
     * @brief Initialize and configure the GPIO pins as output.
     * @param pins Vector of header pin numbers or direct Linux GPIO numbers.
     * @param pinType HEADER_PIN or LINUX_GPIO_NUM.
     * @param model Jetson Orin model variant for header pin resolution.
     * @param activeLow Whether the logic is active-low (default: false = active high).
     * @return true on success, false on error.
     */
    bool init(const std::vector<int>& pins, 
              PinType pinType = PinType::HEADER_PIN, 
              JetsonModel model = JetsonModel::JETSON_ORIN_NANO_NX,
              bool activeLow = false);

    /**
     * @brief Set output level directly for all pins.
     * @param high true for HIGH, false for LOW.
     * @return true on success.
     */
    bool write(bool high);

    /**
     * @brief Set all pins HIGH.
     */
    bool setHigh();

    /**
     * @brief Set all pins LOW.
     */
    bool setLow();

    /**
     * @brief Generate a hardware trigger pulse with microsecond precision.
     *
     * @param duration_us Pulse duration in microseconds (default: 100 µs).
     * @return true on success.
     */
    bool generatePulse(unsigned int duration_us = 100);

    /**
     * @brief Release and unexport the GPIO pins.
     */
    void release();

    std::vector<int> getGpioNumbers() const {
        std::vector<int> nums;
        for (const auto& p : m_pins) nums.push_back(p.gpioNumber);
        return nums;
    }

    /**
     * @brief Check if GPIO is successfully initialized.
     */
    bool isInitialized() const { return m_initialized; }

    static int mapHeaderPinToGpio(int headerPin, JetsonModel model);

private:
    struct PinInfo {
        int gpioNumber;
        std::string gpioPath;
        int valueFd{-1};
    };

    bool exportPin(const PinInfo& pin);
    bool unexportPin(const PinInfo& pin);
    bool setDirection(const PinInfo& pin, PinDirection dir);
    bool setActiveLow(const PinInfo& pin, bool activeLow);
    bool openValueFd(PinInfo& pin);
    void closeValueFd(PinInfo& pin);

    std::vector<PinInfo> m_pins;
    bool m_activeLow{false};
    bool m_initialized{false};
};

} // namespace stereo
