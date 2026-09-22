#pragma once

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "StereoConfig.hpp"

#include "GpioController.hpp"

#include <memory>
#include <string>
#include <vector>

namespace stereo {

/**
 * @brief Represents a synchronized pair of images captured from the stereo camera system.
 */
struct StereoFrame {
    Spinnaker::ImagePtr leftImage{nullptr};
    Spinnaker::ImagePtr rightImage{nullptr};

    uint64_t leftFrameID{0};
    uint64_t rightFrameID{0};

    uint64_t leftTimestampNs{0};
    uint64_t rightTimestampNs{0};
    double syncDeltaMs{0.0}; // Absolute timestamp difference between left and right in milliseconds

    size_t width{0};
    size_t height{0};
    bool valid{false};

    /**
     * @brief Save both images to disk.
     * @param leftPath Filepath for left image (e.g. "left_001.png" or "left_001.jpg").
     * @param rightPath Filepath for right image.
     * @return true if both saved successfully.
     */
    bool save(const std::string& leftPath, const std::string& rightPath) const;
};

/**
 * @brief Manages a Spinnaker dual-camera stereo system.
 *
 * Handles camera discovery by serial numbers, GenApi configuration
 * (exposure, gain, trigger source/mode, buffer handling), continuous
 * acquisition lifecycle, and synchronized frame acquisition.
 */
class StereoCameraSystem {
public:
    StereoCameraSystem();
    ~StereoCameraSystem();

    // Prevent copying to maintain Spinnaker resource safety
    StereoCameraSystem(const StereoCameraSystem&) = delete;
    StereoCameraSystem& operator=(const StereoCameraSystem&) = delete;

    /**
     * @brief Initialize the stereo camera system using a YAML configuration file.
     * @param yamlFilePath Path to the YAML file.
     * @return true on success, false otherwise.
     */
    bool initializeFromYaml(const std::string& yamlFilePath);

    /**
     * @brief Initialize the stereo camera system using an existing config object.
     * @param config System configuration.
     * @return true on success, false otherwise.
     */
    bool initialize(const StereoSystemConfig& config);

    /**
     * @brief Receive both images when a trigger signal is sent.
     *
     * Waits for and retrieves a frame from both left and right cameras within
     * the configured timeout, verifies integrity, and validates frame synchronization.
     *
     * @param timeoutMs Timeout in milliseconds (0 uses timeout from YAML config).
     * @return Synchronized StereoFrame pair.
     */
    StereoFrame receiveStereoPair(uint64_t timeoutMs = 0);

    /**
     * @brief Send software triggers to both cameras and immediately receive both images.
     *
     * Available on all platforms. Only works when TriggerSource is configured as "Software".
     *
     * @param timeoutMs Timeout in milliseconds (0 uses timeout from YAML config).
     * @return Synchronized StereoFrame pair.
     */
    StereoFrame softwareTriggerAndReceive(uint64_t timeoutMs = 0);

#ifdef STEREO_HAS_JETSON_GPIO
    /**
     * @brief Send a hardware GPIO trigger pulse via GpioController and immediately receive both images.
     * @param gpio Initialized GpioController instance configured as output.
     * @param pulseDurationUs Duration of trigger pulse in microseconds (default: 100 µs).
     * @param timeoutMs Timeout in milliseconds (0 uses timeout from YAML config).
     * @return Synchronized StereoFrame pair.
     */
    StereoFrame triggerAndReceive(GpioController& gpio,
                                  unsigned int pulseDurationUs = 100,
                                  uint64_t timeoutMs = 0);
#endif

    /**
     * @brief End acquisition on both cameras.
     */
    void stopAcquisition();

    /**
     * @brief Release all Spinnaker resources, cameras, and system instance.
     */
    void release();

    /**
     * @brief Check if stereo camera system is initialized and ready.
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Get active configuration.
     */
    const StereoSystemConfig& getConfig() const { return m_config; }

private:
    Spinnaker::CameraPtr findCameraBySerial(Spinnaker::CameraList& camList, const std::string& serial);
    bool configureCamera(Spinnaker::CameraPtr pCam, const CameraConfig& cfg, const AcquisitionConfig& acqCfg);
    bool configureExposure(Spinnaker::GenApi::INodeMap& nodeMap, double exposureTimeUs, bool autoExposure);
    bool configureGain(Spinnaker::GenApi::INodeMap& nodeMap, double gainDb, bool autoGain);
    bool configureTrigger(Spinnaker::GenApi::INodeMap& nodeMap, const CameraConfig& cfg);
    bool configureStream(Spinnaker::CameraPtr pCam, const std::string& bufferHandlingMode);
    void resetCameraTimestamp(Spinnaker::CameraPtr cam);
    StereoSystemConfig m_config;
    Spinnaker::SystemPtr m_pSystem{nullptr};
    Spinnaker::CameraList m_camList;
    Spinnaker::CameraPtr m_pCamLeft{nullptr};
    Spinnaker::CameraPtr m_pCamRight{nullptr};
    bool m_initialized{false};
    bool m_acquiring{false};
};

} // namespace stereo
