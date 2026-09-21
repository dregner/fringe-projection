#include "StereoCameraSystem.hpp"
#include "JetsonGPIO.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <filesystem>

namespace fs = std::filesystem;

static std::atomic<bool> g_running{true};

void signalHandler(int signum) {
    std::cout << "\n[Main] Caught signal " << signum << ", stopping acquisition..." << std::endl;
    g_running = false;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::string configPath = "/home/daniel/Documents/codes/fringe_process/config/stereo_config.yaml";
    int numFramesToCapture = 5;
    bool enableGpio = true;

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
        } else if (arg == "--frames" && i + 1 < argc) {
            numFramesToCapture = std::stoi(argv[++i]);
        } else if (arg == "--no-gpio") {
            enableGpio = false;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --config <path>    Path to stereo_config.yaml (default: config/stereo_config.yaml)\n"
                      << "  --frames <N>       Number of stereo pairs to trigger and capture (default: 5)\n"
                      << "  --no-gpio          Disable Jetson GPIO pulse generation (for software/external trigger)\n"
                      << "  --help, -h         Display this help message\n";
            return 0;
        }
    }

    std::cout << "===============================================================\n"
              << "       Spinnaker Stereo Camera System & Jetson Orin Trigger     \n"
              << "===============================================================\n";

    // Create output directory for captured frames
    fs::create_directories("captured_frames");

    // Initialize Stereo Camera System
    stereo::StereoCameraSystem stereoSystem;
    if (!stereoSystem.initializeFromYaml(configPath)) {
        std::cerr << "[Main] Failed to initialize stereo camera system. Exiting." << std::endl;
        return 1;
    }

    const auto& config = stereoSystem.getConfig();

    // Initialize Jetson GPIO Trigger Output if enabled (ARM64 / Jetson Orin only)
#ifdef STEREO_HAS_JETSON_GPIO
    std::unique_ptr<stereo::JetsonGPIO> gpioTrigger;
    if (enableGpio && config.gpioTrigger.enabled) {
        gpioTrigger = std::make_unique<stereo::JetsonGPIO>();

        stereo::PinType pType = (config.gpioTrigger.pinType == "LINUX_GPIO_NUM")
                                ? stereo::PinType::LINUX_GPIO_NUM
                                : stereo::PinType::HEADER_PIN;

        stereo::JetsonModel jModel = (config.gpioTrigger.jetsonModel == "JETSON_AGX_ORIN")
                                     ? stereo::JetsonModel::JETSON_AGX_ORIN
                                     : stereo::JetsonModel::JETSON_ORIN_NANO_NX;

        std::cout << "[Main] Initializing Jetson GPIO trigger pins: ";
        for (int p : config.gpioTrigger.pins) std::cout << p << " ";
        std::cout << "(" << config.gpioTrigger.pinType << ")..." << std::endl;

        if (!gpioTrigger->init(config.gpioTrigger.pins, pType, jModel, config.gpioTrigger.activeLow)) {
            std::cerr << "[Main] Warning: Failed to initialize Jetson GPIO trigger pin.\n"
                      << "       Ensure the application is run with appropriate permissions or sudo.\n"
                      << "       Falling back to software trigger mode." << std::endl;
            gpioTrigger.reset();
        }
    }
#else
    std::cout << "[Main] Non-ARM64 build: JetsonGPIO not available. "
                 "Cameras will be triggered via Spinnaker software trigger." << std::endl;
#endif

    std::cout << "\n[Main] Ready to acquire " << numFramesToCapture << " triggered stereo image pairs." << std::endl;
    std::cout << "---------------------------------------------------------------" << std::endl;

    int successCount = 0;
    for (int frameIdx = 0; frameIdx < numFramesToCapture && g_running; ++frameIdx) {
        std::cout << "\n[Main] Frame [" << (frameIdx + 1) << "/" << numFramesToCapture << "]: ";

        stereo::StereoFrame frame;
#ifdef STEREO_HAS_JETSON_GPIO
        if (gpioTrigger && gpioTrigger->isInitialized()) {
            std::cout << "Generating GPIO trigger pulse ("
                      << config.gpioTrigger.pulseDurationUs << " µs)..." << std::endl;
            frame = stereoSystem.triggerAndReceive(*gpioTrigger,
                                                   config.gpioTrigger.pulseDurationUs,
                                                   config.acquisition.timeoutMs);
        } else {
            std::cout << "GPIO unavailable – using software trigger..." << std::endl;
            frame = stereoSystem.softwareTriggerAndReceive(config.acquisition.timeoutMs);
        }
#else
        std::cout << "Sending software trigger..." << std::endl;
        frame = stereoSystem.softwareTriggerAndReceive(config.acquisition.timeoutMs);
#endif

        if (frame.valid) {
            std::cout << "  ✓ Captured synchronized stereo pair:\n"
                      << "    - Resolution:      " << frame.width << " x " << frame.height << "\n"
                      << "    - Left Frame ID:   " << frame.leftFrameID << " | Right Frame ID: " << frame.rightFrameID << "\n"
                      << "    - Left Timestamp:  " << frame.leftTimestampNs << " ns\n"
                      << "    - Right Timestamp: " << frame.rightTimestampNs << " ns\n"
                      << "    - Sync Difference: " << std::fixed << std::setprecision(4) 
                      << frame.syncDeltaMs << " ms" << std::endl;

            // Save image pair to disk
            std::ostringstream leftFile, rightFile;
            leftFile << "captured_frames/left_" << std::setw(4) << std::setfill('0') << frameIdx << ".jpg";
            rightFile << "captured_frames/right_" << std::setw(4) << std::setfill('0') << frameIdx << ".jpg";

            frame.save(leftFile.str(), rightFile.str());

            // Release images from Spinnaker buffer pool
            if (frame.leftImage) frame.leftImage->Release();
            if (frame.rightImage) frame.rightImage->Release();

            successCount++;
        } else {
            std::cerr << "  ✗ Failed to receive synchronized frame pair." << std::endl;
        }

        // Delay between successive captures
        if (frameIdx + 1 < numFramesToCapture && g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    std::cout << "\n===============================================================\n"
              << "[Main] Capture complete: " << successCount << "/" << numFramesToCapture 
              << " stereo pairs acquired successfully.\n"
              << "===============================================================" << std::endl;

    // Gracefully stop acquisition and release cameras
    stereoSystem.release();
#ifdef STEREO_HAS_JETSON_GPIO
    if (gpioTrigger) {
        gpioTrigger->release();
    }
#endif

    return (successCount > 0) ? 0 : 1;
}
