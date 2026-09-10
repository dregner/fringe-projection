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
    explicit JetsonGPIO(int pin, 
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
     * @brief Initialize and configure the GPIO pin as output.
     * @param pin Header pin number (e.g. 7, 12, 13) or direct Linux GPIO number.
     * @param pinType HEADER_PIN or LINUX_GPIO_NUM.
     * @param model Jetson Orin model variant for header pin resolution.
     * @param activeLow Whether the logic is active-low (default: false = active high).
     * @return true on success, false on error.
     */
    bool init(int pin, 
              PinType pinType = PinType::HEADER_PIN, 
              JetsonModel model = JetsonModel::JETSON_ORIN_NANO_NX,
              bool activeLow = false);

    /**
     * @brief Set output level directly.
     * @param high true for HIGH, false for LOW.
     * @return true on success.
     */
    bool write(bool high);

    /**
     * @brief Set pin HIGH.
     */
    bool setHigh();

    /**
     * @brief Set pin LOW.
     */
    bool setLow();

    /**
     * @brief Read current state of the pin.
     * @return true if HIGH, false if LOW or error.
     */
    bool read();

    /**
     * @brief Generate a hardware trigger pulse with microsecond precision.
     *
     * For pulses <= 200 µs, utilizes high-resolution clock busy-waiting
     * to eliminate kernel scheduler latency and jitter.
     * For longer pulses, utilizes std::this_thread::sleep_for.
     *
     * @param duration_us Pulse duration in microseconds (default: 100 µs).
     * @return true on success.
     */
    bool generatePulse(unsigned int duration_us = 100);

    /**
     * @brief Release and unexport the GPIO pin.
     */
    void release();

    /**
     * @brief Get the resolved Linux GPIO number.
     */
    int getGpioNumber() const { return m_gpioNumber; }

    /**
     * @brief Check if GPIO is successfully initialized.
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Map 40-pin expansion header pin to Linux GPIO number for Jetson Orin.
     * @param headerPin Pin number on 40-pin header (1 to 40).
     * @param model Jetson Orin variant.
     * @return Linux GPIO number, or -1 if invalid or not a GPIO.
     */
    static int mapHeaderPinToGpio(int headerPin, JetsonModel model);

private:
    bool exportPin();
    bool unexportPin();
    bool setDirection(PinDirection dir);
    bool setActiveLow(bool activeLow);
    bool openValueFd();
    void closeValueFd();

    int m_gpioNumber{-1};
    int m_valueFd{-1};
    bool m_activeLow{false};
    bool m_initialized{false};
    std::string m_gpioPath;
};

} // namespace stereo
