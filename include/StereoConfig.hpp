#pragma once
#include <string>
#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace stereo {
    /**
    * @brief Configuration for an individual Spinnaker camera.
    */
    struct CameraConfig {
        std::string serialNumber;
        double exposureTimeUs{10000.0};      // in microseconds
        bool autoExposure{false};            // false = manual exposure
        double gainDb{0.0};                  // in dB
        bool autoGain{false};                // false = manual gain
        // Trigger settings
        bool triggerMode{true};              // true = "On", false = "Off"
        std::string triggerSource{"Line0"};  // e.g. "Line0", "Line1", "Line2", "Line3", "Software"
        std::string triggerSelector{"FrameStart"}; // e.g. "FrameStart", "AcquisitionStart"
        std::string triggerActivation{"RisingEdge"}; // "RisingEdge", "FallingEdge", "AnyEdge", "LevelHigh", "LevelLow"
        std::string triggerOverlap{"ReadOut"}; // "ReadOut", "Off"
        double triggerDelayUs{0.0};          // in microseconds
    };
    /**
    * @brief Configuration for Jetson Orin GPIO hardware trigger generation.
    */
    struct GPIOTriggerConfig {
        bool enabled{true};
        std::vector<int> pins{7};            // Header pin numbers or Linux GPIO numbers
        std::string pinType{"HEADER_PIN"};   // "HEADER_PIN" or "LINUX_GPIO_NUM"
        std::string jetsonModel{"JETSON_ORIN_NANO_NX"}; // "JETSON_ORIN_NANO_NX" or "JETSON_AGX_ORIN"
        unsigned int pulseDurationUs{100};   // Pulse duration in microseconds
        bool activeLow{false};               // false = active high pulse
    };
    /**
    * @brief General acquisition parameters.
    */
    struct AcquisitionConfig {
        uint64_t timeoutMs{2000};            // Spinnaker GetNextImage timeout in ms
        std::string bufferHandlingMode{"NewestOnly"}; // "NewestOnly", "OldestFirst", "NewestFirst"
        std::string pixelFormat{""};         // e.g. "Mono8", "BayerRG8", "" for default
    };
    /**
    * @brief Complete Stereo System Configuration.
    */

    struct ZNCCConfig {
        int numImages{10};
        int motorSteps{20};
        int warmupTriggers{2};
        std::vector<int> motorPins{1, 0, 8, 2};
        int laserPin{106};
        int delayUs{3500};
        int stepsPerRev{2048};
    };

    struct StereoSystemConfig {
        ZNCCConfig zncc;
        CameraConfig leftCamera;
        CameraConfig rightCamera;
        GPIOTriggerConfig gpioTrigger;
        AcquisitionConfig acquisition;

        static StereoSystemConfig loadFromFile(const std::string& yamlFilePath) {
            StereoSystemConfig sysConfig;
            YAML::Node root = YAML::LoadFile(yamlFilePath);
            if (!root["stereo_system"]) {
                throw std::runtime_error("Missing 'stereo_system' root node in YAML config.");
            }
            YAML::Node stereoNode = root["stereo_system"];

            // Parse Left Camera
            if (stereoNode["left_camera"]) {
                YAML::Node left = stereoNode["left_camera"];
                sysConfig.leftCamera.serialNumber = left["serial_number"].as<std::string>("");
                sysConfig.leftCamera.exposureTimeUs = left["exposure_time_us"].as<double>(sysConfig.leftCamera.exposureTimeUs);
                sysConfig.leftCamera.autoExposure = left["auto_exposure"].as<bool>(sysConfig.leftCamera.autoExposure);
                sysConfig.leftCamera.gainDb = left["gain_db"].as<double>(sysConfig.leftCamera.gainDb);
                sysConfig.leftCamera.autoGain = left["auto_gain"].as<bool>(sysConfig.leftCamera.autoGain);
                if (left["trigger"]) {
                    YAML::Node trig = left["trigger"];
                    sysConfig.leftCamera.triggerMode = trig["mode"].as<bool>(sysConfig.leftCamera.triggerMode);
                    sysConfig.leftCamera.triggerSource = trig["source"].as<std::string>(sysConfig.leftCamera.triggerSource);
                    sysConfig.leftCamera.triggerSelector = trig["selector"].as<std::string>(sysConfig.leftCamera.triggerSelector);
                    sysConfig.leftCamera.triggerActivation = trig["activation"].as<std::string>(sysConfig.leftCamera.triggerActivation);
                    sysConfig.leftCamera.triggerOverlap = trig["overlap"].as<std::string>(sysConfig.leftCamera.triggerOverlap);
                    sysConfig.leftCamera.triggerDelayUs = trig["delay_us"].as<double>(sysConfig.leftCamera.triggerDelayUs);
                }
            } else {
                throw std::runtime_error("Missing 'left_camera' in YAML config.");
            }
            // Parse Right Camera
            if (stereoNode["right_camera"]) {
                YAML::Node right = stereoNode["right_camera"];
                sysConfig.rightCamera.serialNumber = right["serial_number"].as<std::string>("");
                sysConfig.rightCamera.exposureTimeUs = right["exposure_time_us"].as<double>(sysConfig.rightCamera.exposureTimeUs);
                sysConfig.rightCamera.autoExposure = right["auto_exposure"].as<bool>(sysConfig.rightCamera.autoExposure);
                sysConfig.rightCamera.gainDb = right["gain_db"].as<double>(sysConfig.rightCamera.gainDb);
                sysConfig.rightCamera.autoGain = right["auto_gain"].as<bool>(sysConfig.rightCamera.autoGain);
                if (right["trigger"]) {
                    YAML::Node trig = right["trigger"];
                    sysConfig.rightCamera.triggerMode = trig["mode"].as<bool>(sysConfig.rightCamera.triggerMode);
                    sysConfig.rightCamera.triggerSource = trig["source"].as<std::string>(sysConfig.rightCamera.triggerSource);
                    sysConfig.rightCamera.triggerSelector = trig["selector"].as<std::string>(sysConfig.rightCamera.triggerSelector);
                    sysConfig.rightCamera.triggerActivation = trig["activation"].as<std::string>(sysConfig.rightCamera.triggerActivation);
                    sysConfig.rightCamera.triggerOverlap = trig["overlap"].as<std::string>(sysConfig.rightCamera.triggerOverlap);
                    sysConfig.rightCamera.triggerDelayUs = trig["delay_us"].as<double>(sysConfig.rightCamera.triggerDelayUs);
                }
            } else {
                throw std::runtime_error("Missing 'right_camera' in YAML config.");
            }
            // Parse GPIO Trigger
            if (stereoNode["gpio_trigger"]) {
                YAML::Node gpio = stereoNode["gpio_trigger"];
                sysConfig.gpioTrigger.enabled = gpio["enabled"].as<bool>(sysConfig.gpioTrigger.enabled);
                
                // Support both single pin and list of pins
                if (gpio["pins"] && gpio["pins"].IsSequence()) {
                    sysConfig.gpioTrigger.pins.clear();
                    for (const auto& p : gpio["pins"]) {
                        sysConfig.gpioTrigger.pins.push_back(p.as<int>());
                    }
                } else if (gpio["pin"]) {
                    sysConfig.gpioTrigger.pins = { gpio["pin"].as<int>() };
                }

                sysConfig.gpioTrigger.pinType = gpio["pin_type"].as<std::string>(sysConfig.gpioTrigger.pinType);
                sysConfig.gpioTrigger.jetsonModel = gpio["jetson_model"].as<std::string>(sysConfig.gpioTrigger.jetsonModel);
                sysConfig.gpioTrigger.pulseDurationUs = gpio["pulse_duration_us"].as<unsigned int>(sysConfig.gpioTrigger.pulseDurationUs);
                sysConfig.gpioTrigger.activeLow = gpio["active_low"].as<bool>(sysConfig.gpioTrigger.activeLow);
            }
            // Parse Acquisition
            if (stereoNode["acquisition"]) {
                YAML::Node acq = stereoNode["acquisition"];
                sysConfig.acquisition.timeoutMs = acq["timeout_ms"].as<uint64_t>(sysConfig.acquisition.timeoutMs);
                sysConfig.acquisition.bufferHandlingMode = acq["buffer_handling_mode"].as<std::string>(sysConfig.acquisition.bufferHandlingMode);
                sysConfig.acquisition.pixelFormat = acq["pixel_format"].as<std::string>(sysConfig.acquisition.pixelFormat);
            }
            
            if (stereoNode["zncc"]) {
                YAML::Node zNode = stereoNode["zncc"];
                sysConfig.zncc.numImages = zNode["num_images"].as<int>(sysConfig.zncc.numImages);
                sysConfig.zncc.motorSteps = zNode["motor_steps"].as<int>(sysConfig.zncc.motorSteps);
                sysConfig.zncc.warmupTriggers = zNode["warmup_triggers"].as<int>(sysConfig.zncc.warmupTriggers);
                sysConfig.zncc.laserPin = zNode["laser_pin"].as<int>(sysConfig.zncc.laserPin);
                sysConfig.zncc.delayUs = zNode["delay_us"].as<int>(sysConfig.zncc.delayUs);
                sysConfig.zncc.stepsPerRev = zNode["steps_per_rev"].as<int>(sysConfig.zncc.stepsPerRev);
                if (zNode["motor_pins"] && zNode["motor_pins"].IsSequence()) {
                    sysConfig.zncc.motorPins.clear();
                    for (const auto& p : zNode["motor_pins"]) {
                        sysConfig.zncc.motorPins.push_back(p.as<int>());
                    }
                }
            }

            return sysConfig;
        }
    };
} // namespace stereo