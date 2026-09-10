#include "StereoCameraSystem.hpp"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <future>
#include <cmath>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace stereo {

bool StereoFrame::save(const std::string& leftPath, const std::string& rightPath) const {
    if (!valid || !leftImage || !rightImage) {
        std::cerr << "[StereoFrame] Cannot save: frame pair is not valid." << std::endl;
        return false;
    }

    try {
        leftImage->Save(leftPath.c_str());
        rightImage->Save(rightPath.c_str());
        std::cout << "[StereoFrame] Saved left image to: " << leftPath << std::endl;
        std::cout << "[StereoFrame] Saved right image to: " << rightPath << std::endl;
        return true;
    } catch (const Spinnaker::Exception& e) {
        std::cerr << "[StereoFrame] Error saving images: " << e.what() << std::endl;
        return false;
    }
}

StereoCameraSystem::StereoCameraSystem() = default;

StereoCameraSystem::~StereoCameraSystem() {
    release();
}

bool StereoCameraSystem::initializeFromYaml(const std::string& yamlFilePath) {
    try {
        std::cout << "[StereoCameraSystem] Loading YAML configuration from: " << yamlFilePath << std::endl;
        StereoSystemConfig cfg = StereoSystemConfig::loadFromFile(yamlFilePath);
        return initialize(cfg);
    } catch (const std::exception& e) {
        std::cerr << "[StereoCameraSystem] Configuration parsing error: " << e.what() << std::endl;
        return false;
    }
}

bool StereoCameraSystem::initialize(const StereoSystemConfig& config) {
    release();
    m_config = config;

    try {
        std::cout << "[StereoCameraSystem] Initializing Spinnaker SDK..." << std::endl;
        m_pSystem = System::GetInstance();
        if (!m_pSystem) {
            std::cerr << "[StereoCameraSystem] Failed to retrieve Spinnaker System instance." << std::endl;
            return false;
        }

        const LibraryVersion ver = m_pSystem->GetLibraryVersion();
        std::cout << "[StereoCameraSystem] Spinnaker library version: " 
                  << ver.major << "." << ver.minor << "." << ver.type << "." << ver.build << std::endl;

        // Retrieve cameras from system
        m_camList = m_pSystem->GetCameras();
        const unsigned int numCameras = m_camList.GetSize();
        std::cout << "[StereoCameraSystem] Number of cameras detected: " << numCameras << std::endl;

        if (numCameras < 2) {
            std::cerr << "[StereoCameraSystem] Error: At least 2 cameras are required for stereo. Found: " 
                      << numCameras << std::endl;
            return false;
        }

        // Locate left camera by serial
        m_pCamLeft = findCameraBySerial(m_camList, m_config.leftCamera.serialNumber);
        if (!m_pCamLeft) {
            if (m_config.leftCamera.serialNumber.empty() && numCameras >= 1) {
                std::cout << "[StereoCameraSystem] Left camera serial is unspecified. Using camera index 0." << std::endl;
                m_pCamLeft = m_camList.GetByIndex(0);
            } else {
                std::cerr << "[StereoCameraSystem] Left camera with serial '" 
                          << m_config.leftCamera.serialNumber << "' not found." << std::endl;
                return false;
            }
        }

        // Locate right camera by serial
        m_pCamRight = findCameraBySerial(m_camList, m_config.rightCamera.serialNumber);
        if (!m_pCamRight) {
            if (m_config.rightCamera.serialNumber.empty() && numCameras >= 2) {
                std::cout << "[StereoCameraSystem] Right camera serial is unspecified. Using camera index 1." << std::endl;
                m_pCamRight = m_camList.GetByIndex(1);
            } else {
                std::cerr << "[StereoCameraSystem] Right camera with serial '" 
                          << m_config.rightCamera.serialNumber << "' not found." << std::endl;
                return false;
            }
        }

        // Verify that Left and Right are not the same camera
        if (m_pCamLeft == m_pCamRight) {
            std::cerr << "[StereoCameraSystem] Error: Left and Right cameras resolve to the same device." << std::endl;
            return false;
        }

        // Initialize and configure Left Camera
        std::cout << "\n--- Configuring Left Camera ---" << std::endl;
        m_pCamLeft->Init();
        if (!configureCamera(m_pCamLeft, m_config.leftCamera, m_config.acquisition)) {
            std::cerr << "[StereoCameraSystem] Failed to configure Left Camera." << std::endl;
            return false;
        }

        // Initialize and configure Right Camera
        std::cout << "\n--- Configuring Right Camera ---" << std::endl;
        m_pCamRight->Init();
        if (!configureCamera(m_pCamRight, m_config.rightCamera, m_config.acquisition)) {
            std::cerr << "[StereoCameraSystem] Failed to configure Right Camera." << std::endl;
            return false;
        }

        // Begin acquisition on both cameras
        std::cout << "\n[StereoCameraSystem] Starting continuous acquisition streams..." << std::endl;
        m_pCamLeft->BeginAcquisition();
        m_pCamRight->BeginAcquisition();
        m_acquiring = true;
        m_initialized = true;

        std::cout << "[StereoCameraSystem] Stereo camera system initialized and ready for triggers!\n" << std::endl;
        return true;

    } catch (const Spinnaker::Exception& e) {
        std::cerr << "[StereoCameraSystem] Spinnaker Exception during initialization: " << e.what() << std::endl;
        release();
        return false;
    }
}

CameraPtr StereoCameraSystem::findCameraBySerial(CameraList& camList, const std::string& serial) {
    if (serial.empty()) {
        return nullptr;
    }

    for (unsigned int i = 0; i < camList.GetSize(); ++i) {
        CameraPtr pCam = camList.GetByIndex(i);
        INodeMap& tlNodeMap = pCam->GetTLDeviceNodeMap();
        CStringPtr ptrSerial = tlNodeMap.GetNode("DeviceSerialNumber");
        if (IsReadable(ptrSerial)) {
            std::string currentSerial = ptrSerial->GetValue().c_str();
            if (currentSerial == serial) {
                return pCam;
            }
        }
    }
    return nullptr;
}

bool StereoCameraSystem::configureCamera(CameraPtr pCam, const CameraConfig& cfg, const AcquisitionConfig& acqCfg) {
    try {
        INodeMap& nodeMap = pCam->GetNodeMap();
        INodeMap& tlDeviceNodeMap = pCam->GetTLDeviceNodeMap();

        // Print device info
        CStringPtr ptrModel = tlDeviceNodeMap.GetNode("DeviceModelName");
        CStringPtr ptrSerial = tlDeviceNodeMap.GetNode("DeviceSerialNumber");
        std::cout << "Device: " << (IsReadable(ptrModel) ? ptrModel->GetValue().c_str() : "Unknown")
                  << " (S/N: " << (IsReadable(ptrSerial) ? ptrSerial->GetValue().c_str() : "Unknown") << ")" << std::endl;

        // Set acquisition mode to Continuous
        CEnumerationPtr ptrAcqMode = nodeMap.GetNode("AcquisitionMode");
        if (IsReadable(ptrAcqMode) && IsWritable(ptrAcqMode)) {
            CEnumEntryPtr ptrContinuous = ptrAcqMode->GetEntryByName("Continuous");
            if (IsReadable(ptrContinuous)) {
                ptrAcqMode->SetIntValue(ptrContinuous->GetValue());
                std::cout << "  - AcquisitionMode set to Continuous" << std::endl;
            }
        }

        // Configure stream buffer handling (e.g. NewestOnly to prevent frame lag)
        configureStream(pCam, acqCfg.bufferHandlingMode);

        // Configure Exposure
        if (!configureExposure(nodeMap, cfg.exposureTimeUs, cfg.autoExposure)) {
            std::cerr << "  - Warning: Failed to configure exposure settings." << std::endl;
        }

        // Configure Gain
        if (!configureGain(nodeMap, cfg.gainDb, cfg.autoGain)) {
            std::cerr << "  - Warning: Failed to configure gain settings." << std::endl;
        }

        // Configure Trigger
        if (!configureTrigger(nodeMap, cfg)) {
            std::cerr << "  - Error: Failed to configure trigger settings." << std::endl;
            return false;
        }

        // Optional: Configure pixel format if specified
        if (!acqCfg.pixelFormat.empty()) {
            CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
            if (IsReadable(ptrPixelFormat) && IsWritable(ptrPixelFormat)) {
                CEnumEntryPtr ptrFormat = ptrPixelFormat->GetEntryByName(acqCfg.pixelFormat.c_str());
                if (IsReadable(ptrFormat)) {
                    ptrPixelFormat->SetIntValue(ptrFormat->GetValue());
                    std::cout << "  - PixelFormat set to " << acqCfg.pixelFormat << std::endl;
                }
            }
        }

        return true;
    } catch (const Spinnaker::Exception& e) {
        std::cerr << "  - Spinnaker Exception in configureCamera: " << e.what() << std::endl;
        return false;
    }
}

bool StereoCameraSystem::configureStream(CameraPtr pCam, const std::string& bufferHandlingMode) {
    try {
        INodeMap& streamNodeMap = pCam->GetTLStreamNodeMap();
        CEnumerationPtr ptrHandlingMode = streamNodeMap.GetNode("StreamBufferHandlingMode");
        if (IsReadable(ptrHandlingMode) && IsWritable(ptrHandlingMode)) {
            CEnumEntryPtr ptrMode = ptrHandlingMode->GetEntryByName(bufferHandlingMode.c_str());
            if (IsReadable(ptrMode)) {
                ptrHandlingMode->SetIntValue(ptrMode->GetValue());
                std::cout << "  - StreamBufferHandlingMode set to " << bufferHandlingMode << std::endl;
                return true;
            }
        }
    } catch (const Spinnaker::Exception& e) {
        std::cerr << "  - Note: StreamBufferHandlingMode could not be configured: " << e.what() << std::endl;
    }
    return false;
}

bool StereoCameraSystem::configureExposure(INodeMap& nodeMap, double exposureTimeUs, bool autoExposure) {
    try {
        // Set ExposureAuto
        CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        if (IsReadable(ptrExposureAuto) && IsWritable(ptrExposureAuto)) {
            CEnumEntryPtr ptrEntry = ptrExposureAuto->GetEntryByName(autoExposure ? "Continuous" : "Off");
            if (IsReadable(ptrEntry)) {
                ptrExposureAuto->SetIntValue(ptrEntry->GetValue());
                std::cout << "  - ExposureAuto set to " << (autoExposure ? "Continuous" : "Off") << std::endl;
            }
        }

        if (!autoExposure) {
            // Set manual exposure time
            CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
            if (IsReadable(ptrExposureTime) && IsWritable(ptrExposureTime)) {
                double minExp = ptrExposureTime->GetMin();
                double maxExp = ptrExposureTime->GetMax();
                double targetExp = std::max(minExp, std::min(maxExp, exposureTimeUs));

                ptrExposureTime->SetValue(targetExp);
                std::cout << "  - ExposureTime set to " << std::fixed << std::setprecision(1) 
                          << targetExp << " µs (Range: " << minExp << " - " << maxExp << " µs)" << std::endl;
                return true;
            } else {
                std::cerr << "  - ExposureTime node is not writable." << std::endl;
                return false;
            }
        }
        return true;
    } catch (const Spinnaker::Exception& e) {
        std::cerr << "  - Exception configuring exposure: " << e.what() << std::endl;
        return false;
    }
}

bool StereoCameraSystem::configureGain(INodeMap& nodeMap, double gainDb, bool autoGain) {
    try {
        // Set GainAuto
        CEnumerationPtr ptrGainAuto = nodeMap.GetNode("GainAuto");
        if (IsReadable(ptrGainAuto) && IsWritable(ptrGainAuto)) {
            CEnumEntryPtr ptrEntry = ptrGainAuto->GetEntryByName(autoGain ? "Continuous" : "Off");
            if (IsReadable(ptrEntry)) {
                ptrGainAuto->SetIntValue(ptrEntry->GetValue());
                std::cout << "  - GainAuto set to " << (autoGain ? "Continuous" : "Off") << std::endl;
            }
        }

        if (!autoGain) {
            // Set manual gain in dB
            CFloatPtr ptrGain = nodeMap.GetNode("Gain");
            if (IsReadable(ptrGain) && IsWritable(ptrGain)) {
                double minGain = ptrGain->GetMin();
                double maxGain = ptrGain->GetMax();
                double targetGain = std::max(minGain, std::min(maxGain, gainDb));

                ptrGain->SetValue(targetGain);
                std::cout << "  - Gain set to " << std::fixed << std::setprecision(2) 
                          << targetGain << " dB (Range: " << minGain << " - " << maxGain << " dB)" << std::endl;
                return true;
            } else {
                std::cerr << "  - Gain node is not writable." << std::endl;
                return false;
            }
        }
        return true;
    } catch (const Spinnaker::Exception& e) {
        std::cerr << "  - Exception configuring gain: " << e.what() << std::endl;
        return false;
    }
}

bool StereoCameraSystem::configureTrigger(INodeMap& nodeMap, const CameraConfig& cfg) {
    try {
        // Step 1: Ensure TriggerMode is OFF before configuring trigger parameters
        CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
        if (!IsReadable(ptrTriggerMode) || !IsWritable(ptrTriggerMode)) {
            std::cerr << "  - Unable to read/write TriggerMode node." << std::endl;
            return false;
        }

        CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
        if (!IsReadable(ptrTriggerModeOff)) {
            std::cerr << "  - Unable to retrieve TriggerMode 'Off' entry." << std::endl;
            return false;
        }
        ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());
        std::cout << "  - TriggerMode disabled for configuration" << std::endl;

        if (!cfg.triggerMode) {
            std::cout << "  - Free-run (TriggerMode Off) selected." << std::endl;
            return true;
        }

        // Step 2: Set TriggerSelector (e.g. FrameStart)
        CEnumerationPtr ptrTriggerSelector = nodeMap.GetNode("TriggerSelector");
        if (IsReadable(ptrTriggerSelector) && IsWritable(ptrTriggerSelector)) {
            CEnumEntryPtr ptrSelector = ptrTriggerSelector->GetEntryByName(cfg.triggerSelector.c_str());
            if (IsReadable(ptrSelector)) {
                ptrTriggerSelector->SetIntValue(ptrSelector->GetValue());
                std::cout << "  - TriggerSelector set to " << cfg.triggerSelector << std::endl;
            }
        }

        // Step 3: Set TriggerSource (e.g. Line0, Line1, Line2, Line3, Software)
        CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
        if (!IsReadable(ptrTriggerSource) || !IsWritable(ptrTriggerSource)) {
            std::cerr << "  - Unable to read/write TriggerSource node." << std::endl;
            return false;
        }

        CEnumEntryPtr ptrSource = ptrTriggerSource->GetEntryByName(cfg.triggerSource.c_str());
        if (!IsReadable(ptrSource)) {
            std::cerr << "  - Invalid TriggerSource '" << cfg.triggerSource << "'." << std::endl;
            return false;
        }
        ptrTriggerSource->SetIntValue(ptrSource->GetValue());
        std::cout << "  - TriggerSource set to " << cfg.triggerSource << std::endl;

        // Step 4: Set TriggerActivation (RisingEdge, FallingEdge, AnyEdge, LevelHigh, LevelLow)
        CEnumerationPtr ptrTriggerActivation = nodeMap.GetNode("TriggerActivation");
        if (IsReadable(ptrTriggerActivation) && IsWritable(ptrTriggerActivation)) {
            CEnumEntryPtr ptrActivation = ptrTriggerActivation->GetEntryByName(cfg.triggerActivation.c_str());
            if (IsReadable(ptrActivation)) {
                ptrTriggerActivation->SetIntValue(ptrActivation->GetValue());
                std::cout << "  - TriggerActivation set to " << cfg.triggerActivation << std::endl;
            }
        }

        // Step 5: Set TriggerOverlap (ReadOut, Off)
        CEnumerationPtr ptrTriggerOverlap = nodeMap.GetNode("TriggerOverlap");
        if (IsReadable(ptrTriggerOverlap) && IsWritable(ptrTriggerOverlap)) {
            CEnumEntryPtr ptrOverlap = ptrTriggerOverlap->GetEntryByName(cfg.triggerOverlap.c_str());
            if (IsReadable(ptrOverlap)) {
                ptrTriggerOverlap->SetIntValue(ptrOverlap->GetValue());
                std::cout << "  - TriggerOverlap set to " << cfg.triggerOverlap << std::endl;
            }
        }

        // Step 6: Set TriggerDelay if specified
        if (cfg.triggerDelayUs > 0.0) {
            CFloatPtr ptrTriggerDelay = nodeMap.GetNode("TriggerDelay");
            if (IsReadable(ptrTriggerDelay) && IsWritable(ptrTriggerDelay)) {
                ptrTriggerDelay->SetValue(cfg.triggerDelayUs);
                std::cout << "  - TriggerDelay set to " << cfg.triggerDelayUs << " µs" << std::endl;
            }
        }

        // Step 7: Turn TriggerMode ON
        CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On");
        if (!IsReadable(ptrTriggerModeOn)) {
            std::cerr << "  - Unable to retrieve TriggerMode 'On' entry." << std::endl;
            return false;
        }
        ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());
        std::cout << "  - TriggerMode enabled (On)" << std::endl;

        return true;
    } catch (const Spinnaker::Exception& e) {
        std::cerr << "  - Exception configuring trigger: " << e.what() << std::endl;
        return false;
    }
}

StereoFrame StereoCameraSystem::receiveStereoPair(uint64_t timeoutMs) {
    StereoFrame frame;
    frame.valid = false;

    if (!m_initialized || !m_acquiring || !m_pCamLeft || !m_pCamRight) {
        std::cerr << "[StereoCameraSystem] Error: Camera system is not actively acquiring." << std::endl;
        return frame;
    }

    uint64_t timeout = (timeoutMs > 0) ? timeoutMs : m_config.acquisition.timeoutMs;

    // Launch parallel retrieval tasks to prevent serial blocking between left and right streams
    auto fetchLeft = std::async(std::launch::async, [this, timeout]() -> ImagePtr {
        try {
            return m_pCamLeft->GetNextImage(timeout);
        } catch (const Spinnaker::Exception& e) {
            std::cerr << "[StereoCameraSystem] Left camera GetNextImage failed: " << e.what() << std::endl;
            return nullptr;
        }
    });

    auto fetchRight = std::async(std::launch::async, [this, timeout]() -> ImagePtr {
        try {
            return m_pCamRight->GetNextImage(timeout);
        } catch (const Spinnaker::Exception& e) {
            std::cerr << "[StereoCameraSystem] Right camera GetNextImage failed: " << e.what() << std::endl;
            return nullptr;
        }
    });

    ImagePtr pImgLeft = fetchLeft.get();
    ImagePtr pImgRight = fetchRight.get();

    if (!pImgLeft || !pImgRight) {
        std::cerr << "[StereoCameraSystem] Timeout or error acquiring stereo image pair." << std::endl;
        if (pImgLeft) pImgLeft->Release();
        if (pImgRight) pImgRight->Release();
        return frame;
    }

    // Validate completeness of both images
    if (pImgLeft->IsIncomplete()) {
        std::cerr << "[StereoCameraSystem] Left image incomplete (status: " 
                  << pImgLeft->GetImageStatus() << ")" << std::endl;
        pImgLeft->Release();
        pImgRight->Release();
        return frame;
    }

    if (pImgRight->IsIncomplete()) {
        std::cerr << "[StereoCameraSystem] Right image incomplete (status: " 
                  << pImgRight->GetImageStatus() << ")" << std::endl;
        pImgLeft->Release();
        pImgRight->Release();
        return frame;
    }

    // Populate StereoFrame
    frame.leftImage = pImgLeft;
    frame.rightImage = pImgRight;
    frame.leftFrameID = pImgLeft->GetFrameID();
    frame.rightFrameID = pImgRight->GetFrameID();
    frame.leftTimestampNs = pImgLeft->GetTimeStamp();
    frame.rightTimestampNs = pImgRight->GetTimeStamp();

    // Calculate sync delta in milliseconds
    int64_t diffNs = static_cast<int64_t>(frame.leftTimestampNs) - static_cast<int64_t>(frame.rightTimestampNs);
    frame.syncDeltaMs = std::abs(diffNs) / 1.0e6;

    frame.width = pImgLeft->GetWidth();
    frame.height = pImgLeft->GetHeight();
    frame.valid = true;

    return frame;
}

StereoFrame StereoCameraSystem::softwareTriggerAndReceive(uint64_t timeoutMs) {
    if (!m_initialized || !m_acquiring) {
        std::cerr << "[StereoCameraSystem] softwareTriggerAndReceive: system not ready." << std::endl;
        return StereoFrame{};
    }

    // Fire software trigger on both cameras as simultaneously as possible.
    // The acquisition future threads will then block on GetNextImage.
    auto triggerCam = [](Spinnaker::CameraPtr pCam, const std::string& name) {
        try {
            Spinnaker::GenApi::INodeMap& nm = pCam->GetNodeMap();
            Spinnaker::GenApi::CCommandPtr pTrigger = nm.GetNode("TriggerSoftware");
            if (Spinnaker::GenApi::IsWritable(pTrigger)) {
                pTrigger->Execute();
            } else {
                std::cerr << "[StereoCameraSystem] TriggerSoftware node not writable on "
                          << name << std::endl;
            }
        } catch (const Spinnaker::Exception& e) {
            std::cerr << "[StereoCameraSystem] Software trigger error on " << name
                      << ": " << e.what() << std::endl;
        }
    };

    // Trigger both cameras in rapid succession (Left first, then Right within same thread)
    triggerCam(m_pCamLeft,  "Left");
    triggerCam(m_pCamRight, "Right");

    return receiveStereoPair(timeoutMs);
}

#ifdef STEREO_HAS_JETSON_GPIO
StereoFrame StereoCameraSystem::triggerAndReceive(JetsonGPIO& gpio,
                                                  unsigned int pulseDurationUs,
                                                  uint64_t timeoutMs) {
    // Send trigger pulse from Jetson Orin GPIO
    if (!gpio.generatePulse(pulseDurationUs)) {
        std::cerr << "[StereoCameraSystem] Warning: Failed to generate GPIO trigger pulse." << std::endl;
    }

    // Immediately acquire the synchronized stereo frame pair
    return receiveStereoPair(timeoutMs);
}
#endif

void StereoCameraSystem::stopAcquisition() {
    if (m_acquiring) {
        std::cout << "[StereoCameraSystem] Stopping camera acquisition..." << std::endl;
        try {
            if (m_pCamLeft && m_pCamLeft->IsStreaming()) {
                m_pCamLeft->EndAcquisition();
            }
        } catch (const Spinnaker::Exception& e) {
            std::cerr << "[StereoCameraSystem] Error ending left acquisition: " << e.what() << std::endl;
        }

        try {
            if (m_pCamRight && m_pCamRight->IsStreaming()) {
                m_pCamRight->EndAcquisition();
            }
        } catch (const Spinnaker::Exception& e) {
            std::cerr << "[StereoCameraSystem] Error ending right acquisition: " << e.what() << std::endl;
        }
        m_acquiring = false;
    }
}

void StereoCameraSystem::release() {
    stopAcquisition();

    try {
        if (m_pCamLeft) {
            if (m_pCamLeft->IsInitialized()) {
                m_pCamLeft->DeInit();
            }
            m_pCamLeft = nullptr;
        }

        if (m_pCamRight) {
            if (m_pCamRight->IsInitialized()) {
                m_pCamRight->DeInit();
            }
            m_pCamRight = nullptr;
        }

        m_camList.Clear();

        if (m_pSystem) {
            m_pSystem->ReleaseInstance();
            m_pSystem = nullptr;
        }
    } catch (const Spinnaker::Exception& e) {
        std::cerr << "[StereoCameraSystem] Exception during release: " << e.what() << std::endl;
    }

    m_initialized = false;
}

} // namespace stereo
