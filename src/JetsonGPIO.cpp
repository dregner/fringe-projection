#include "JetsonGPIO.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstring>
#include <unordered_map>

namespace stereo {

namespace {

// Jetson AGX Orin 40-pin header mapping to Linux GPIO numbers
const std::unordered_map<int, int> AGX_ORIN_PIN_MAP = {
    {7,  398}, {11, 446}, {12, 389}, {13, 392},
    {15, 444}, {16, 395}, {18, 394}, {19, 396},
    {21, 397}, {22, 393}, {23, 399}, {24, 400},
    {26, 401}, {29, 434}, {31, 435}, {32, 424},
    {33, 436}, {35, 391}, {36, 447}, {37, 429},
    {38, 390}, {40, 388}
};

// Jetson Orin Nano / Orin NX 40-pin header mapping to Linux GPIO numbers
const std::unordered_map<int, int> ORIN_NANO_NX_PIN_MAP = {
    {7,  348}, {11, 349}, {12, 350}, {13, 351},
    {15, 352}, {16, 353}, {18, 354}, {19, 355},
    {21, 356}, {22, 357}, {23, 358}, {24, 359},
    {26, 360}, {29, 361}, {31, 362}, {32, 363},
    {33, 364}, {35, 365}, {36, 366}, {37, 367},
    {38, 368}, {40, 369}
};

bool writeSysfs(const std::string& path, const std::string& value) {
    int fd = ::open(path.c_str(), O_WRONLY);
    if (fd < 0) {
        return false;
    }
    ssize_t written = ::write(fd, value.c_str(), value.size());
    ::close(fd);
    return written == static_cast<ssize_t>(value.size());
}

} // anonymous namespace

JetsonGPIO::JetsonGPIO() = default;

JetsonGPIO::JetsonGPIO(int pin, PinType pinType, JetsonModel model, bool activeLow) {
    init(pin, pinType, model, activeLow);
}

JetsonGPIO::~JetsonGPIO() {
    release();
}

JetsonGPIO::JetsonGPIO(JetsonGPIO&& other) noexcept
    : m_gpioNumber(other.m_gpioNumber),
      m_valueFd(other.m_valueFd),
      m_activeLow(other.m_activeLow),
      m_initialized(other.m_initialized),
      m_gpioPath(std::move(other.m_gpioPath)) {
    other.m_gpioNumber = -1;
    other.m_valueFd = -1;
    other.m_initialized = false;
}

JetsonGPIO& JetsonGPIO::operator=(JetsonGPIO&& other) noexcept {
    if (this != &other) {
        release();
        m_gpioNumber = other.m_gpioNumber;
        m_valueFd = other.m_valueFd;
        m_activeLow = other.m_activeLow;
        m_initialized = other.m_initialized;
        m_gpioPath = std::move(other.m_gpioPath);

        other.m_gpioNumber = -1;
        other.m_valueFd = -1;
        other.m_initialized = false;
    }
    return *this;
}

int JetsonGPIO::mapHeaderPinToGpio(int headerPin, JetsonModel model) {
    if (model == JetsonModel::JETSON_AGX_ORIN) {
        auto it = AGX_ORIN_PIN_MAP.find(headerPin);
        if (it != AGX_ORIN_PIN_MAP.end()) return it->second;
    } else if (model == JetsonModel::JETSON_ORIN_NANO_NX) {
        auto it = ORIN_NANO_NX_PIN_MAP.find(headerPin);
        if (it != ORIN_NANO_NX_PIN_MAP.end()) return it->second;
    }
    return -1;
}

bool JetsonGPIO::init(int pin, PinType pinType, JetsonModel model, bool activeLow) {
    release();

    m_activeLow = activeLow;

    if (pinType == PinType::HEADER_PIN) {
        m_gpioNumber = mapHeaderPinToGpio(pin, model);
        if (m_gpioNumber < 0) {
            std::cerr << "[JetsonGPIO] Error: Header pin " << pin 
                      << " is invalid or not available as GPIO on this Jetson model." << std::endl;
            return false;
        }
        std::cout << "[JetsonGPIO] Mapped header pin " << pin 
                  << " to Linux GPIO " << m_gpioNumber << std::endl;
    } else {
        m_gpioNumber = pin;
        std::cout << "[JetsonGPIO] Using direct Linux GPIO " << m_gpioNumber << std::endl;
    }

    m_gpioPath = "/sys/class/gpio/gpio" + std::to_string(m_gpioNumber);

    // Export pin
    if (!exportPin()) {
        std::cerr << "[JetsonGPIO] Warning: Failed to export GPIO " << m_gpioNumber 
                  << " (it may already be exported or require sudo/gpio permissions)." << std::endl;
    }

    // Small delay to allow udev permissions settling
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Configure active-low
    if (!setActiveLow(m_activeLow)) {
        std::cerr << "[JetsonGPIO] Warning: Failed to set active_low for GPIO " 
                  << m_gpioNumber << std::endl;
    }

    // Set direction to OUT
    if (!setDirection(PinDirection::OUT)) {
        std::cerr << "[JetsonGPIO] Error: Failed to set direction 'out' for GPIO " 
                  << m_gpioNumber << std::endl;
        return false;
    }

    // Open cached file descriptor for fast writing
    if (!openValueFd()) {
        std::cerr << "[JetsonGPIO] Error: Failed to open value file for GPIO " 
                  << m_gpioNumber << std::endl;
        return false;
    }

    // Set initial output state to LOW (inactive)
    setLow();

    m_initialized = true;
    std::cout << "[JetsonGPIO] GPIO " << m_gpioNumber 
              << " initialized successfully as OUTPUT." << std::endl;
    return true;
}

bool JetsonGPIO::exportPin() {
    struct stat st{};
    if (::stat(m_gpioPath.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
        return true; // Already exported
    }
    return writeSysfs("/sys/class/gpio/export", std::to_string(m_gpioNumber));
}

bool JetsonGPIO::unexportPin() {
    return writeSysfs("/sys/class/gpio/unexport", std::to_string(m_gpioNumber));
}

bool JetsonGPIO::setDirection(PinDirection dir) {
    std::string path = m_gpioPath + "/direction";
    std::string value = (dir == PinDirection::OUT) ? "out" : "in";
    return writeSysfs(path, value);
}

bool JetsonGPIO::setActiveLow(bool activeLow) {
    std::string path = m_gpioPath + "/active_low";
    return writeSysfs(path, activeLow ? "1" : "0");
}

bool JetsonGPIO::openValueFd() {
    closeValueFd();
    std::string path = m_gpioPath + "/value";
    m_valueFd = ::open(path.c_str(), O_RDWR | O_SYNC);
    return m_valueFd >= 0;
}

void JetsonGPIO::closeValueFd() {
    if (m_valueFd >= 0) {
        ::close(m_valueFd);
        m_valueFd = -1;
    }
}

bool JetsonGPIO::write(bool high) {
    if (m_valueFd < 0) {
        std::cerr << "[JetsonGPIO] Error: GPIO " << m_gpioNumber 
                  << " value FD is not open." << std::endl;
        return false;
    }
    const char val = high ? '1' : '0';
    // pwrite writes at offset 0 without modifying seek pointer
    ssize_t res = ::pwrite(m_valueFd, &val, 1, 0);
    return res == 1;
}

bool JetsonGPIO::setHigh() {
    return write(true);
}

bool JetsonGPIO::setLow() {
    return write(false);
}

bool JetsonGPIO::read() {
    if (m_valueFd < 0) return false;
    char val = '0';
    ssize_t res = ::pread(m_valueFd, &val, 1, 0);
    if (res == 1) {
        return (val == '1');
    }
    return false;
}

bool JetsonGPIO::generatePulse(unsigned int duration_us) {
    if (!m_initialized) {
        std::cerr << "[JetsonGPIO] Error: Cannot pulse uninitialized GPIO." << std::endl;
        return false;
    }

    // Step 1: Assert trigger signal HIGH (or LOW if activeLow)
    if (!setHigh()) {
        return false;
    }

    // Step 2: High precision delay
    if (duration_us <= 200) {
        // High-precision busy wait avoids kernel context switches
        auto start = std::chrono::steady_clock::now();
        auto target = start + std::chrono::microseconds(duration_us);
        while (std::chrono::steady_clock::now() < target) {
            #if defined(__x86_64__) || defined(_M_X64)
            __builtin_ia32_pause();
            #elif defined(__aarch64__)
            asm volatile("yield");
            #endif
        }
    } else {
        // Longer delay can yield to OS
        std::this_thread::sleep_for(std::chrono::microseconds(duration_us));
    }

    // Step 3: De-assert trigger signal LOW
    return setLow();
}

void JetsonGPIO::release() {
    if (m_initialized) {
        setLow();
        closeValueFd();
        // Keep export active or unexport; leaving pin exported is typical on embedded
        // but we can unexport if desired.
        m_initialized = false;
    }
}

} // namespace stereo
