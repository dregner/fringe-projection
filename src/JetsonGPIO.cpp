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

JetsonGPIO::JetsonGPIO(const std::vector<int>& pins, PinType pinType, JetsonModel model, bool activeLow) {
    init(pins, pinType, model, activeLow);
}

JetsonGPIO::~JetsonGPIO() {
    release();
}

JetsonGPIO::JetsonGPIO(JetsonGPIO&& other) noexcept
    : m_pins(std::move(other.m_pins)),
      m_activeLow(other.m_activeLow),
      m_initialized(other.m_initialized) {
    other.m_initialized = false;
    other.m_pins.clear();
}

JetsonGPIO& JetsonGPIO::operator=(JetsonGPIO&& other) noexcept {
    if (this != &other) {
        release();
        m_pins = std::move(other.m_pins);
        m_activeLow = other.m_activeLow;
        m_initialized = other.m_initialized;

        other.m_initialized = false;
        other.m_pins.clear();
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

bool JetsonGPIO::init(const std::vector<int>& pins, PinType pinType, JetsonModel model, bool activeLow) {
    release();
    m_activeLow = activeLow;
    bool all_ok = true;

    for (int pin : pins) {
        PinInfo pi;
        if (pinType == PinType::HEADER_PIN) {
            pi.gpioNumber = mapHeaderPinToGpio(pin, model);
            if (pi.gpioNumber < 0) {
                std::cerr << "[JetsonGPIO] Error: Header pin " << pin 
                          << " is invalid or not available as GPIO on this Jetson model." << std::endl;
                all_ok = false;
                continue;
            }
            std::cout << "[JetsonGPIO] Mapped header pin " << pin 
                      << " to Linux GPIO " << pi.gpioNumber << std::endl;
        } else {
            pi.gpioNumber = pin;
            std::cout << "[JetsonGPIO] Using direct Linux GPIO " << pi.gpioNumber << std::endl;
        }

        pi.gpioPath = "/sys/class/gpio/gpio" + std::to_string(pi.gpioNumber);

        // Export pin
        if (!exportPin(pi)) {
            std::cerr << "[JetsonGPIO] Warning: Failed to export GPIO " << pi.gpioNumber 
                      << " (it may already be exported or require sudo/gpio permissions)." << std::endl;
        }

        // Small delay to allow udev permissions settling
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // Configure active-low
        if (!setActiveLow(pi, m_activeLow)) {
            std::cerr << "[JetsonGPIO] Warning: Failed to set active_low for GPIO " 
                      << pi.gpioNumber << std::endl;
        }

        // Set direction to OUT
        if (!setDirection(pi, PinDirection::OUT)) {
            std::cerr << "[JetsonGPIO] Error: Failed to set direction 'out' for GPIO " 
                      << pi.gpioNumber << std::endl;
            all_ok = false;
            continue;
        }

        // Open cached file descriptor for fast writing
        if (!openValueFd(pi)) {
            std::cerr << "[JetsonGPIO] Error: Failed to open value file for GPIO " 
                      << pi.gpioNumber << std::endl;
            all_ok = false;
            continue;
        }

        m_pins.push_back(pi);
    }

    if (m_pins.empty()) {
        return false;
    }

    // Set initial output state to LOW (inactive)
    setLow();

    m_initialized = all_ok;
    return m_initialized;
}

bool JetsonGPIO::exportPin(const PinInfo& pin) {
    struct stat st{};
    if (::stat(pin.gpioPath.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
        return true; // Already exported
    }
    return writeSysfs("/sys/class/gpio/export", std::to_string(pin.gpioNumber));
}

bool JetsonGPIO::unexportPin(const PinInfo& pin) {
    return writeSysfs("/sys/class/gpio/unexport", std::to_string(pin.gpioNumber));
}

bool JetsonGPIO::setDirection(const PinInfo& pin, PinDirection dir) {
    std::string path = pin.gpioPath + "/direction";
    std::string value = (dir == PinDirection::OUT) ? "out" : "in";
    return writeSysfs(path, value);
}

bool JetsonGPIO::setActiveLow(const PinInfo& pin, bool activeLow) {
    std::string path = pin.gpioPath + "/active_low";
    return writeSysfs(path, activeLow ? "1" : "0");
}

bool JetsonGPIO::openValueFd(PinInfo& pin) {
    closeValueFd(pin);
    std::string path = pin.gpioPath + "/value";
    pin.valueFd = ::open(path.c_str(), O_RDWR | O_SYNC);
    return pin.valueFd >= 0;
}

void JetsonGPIO::closeValueFd(PinInfo& pin) {
    if (pin.valueFd >= 0) {
        ::close(pin.valueFd);
        pin.valueFd = -1;
    }
}

bool JetsonGPIO::write(bool high) {
    bool ok = true;
    const char val = high ? '1' : '0';
    for (auto& pin : m_pins) {
        if (pin.valueFd < 0) {
            ok = false;
            continue;
        }
        // pwrite writes at offset 0 without modifying seek pointer
        ssize_t res = ::pwrite(pin.valueFd, &val, 1, 0);
        if (res != 1) ok = false;
    }
    return ok;
}

bool JetsonGPIO::setHigh() {
    return write(true);
}

bool JetsonGPIO::setLow() {
    return write(false);
}

bool JetsonGPIO::generatePulse(unsigned int duration_us) {
    if (!m_initialized || m_pins.empty()) {
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
    if (m_initialized || !m_pins.empty()) {
        setLow();
        for (auto& pin : m_pins) {
            closeValueFd(pin);
        }
        m_pins.clear();
        m_initialized = false;
    }
}

} // namespace stereo
