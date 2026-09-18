/**
 * stereo_fringe_main.cpp
 *
 * Integrated pipeline:
 *   1. Generate fringe + GrayCode patterns via FringeProcess
 *   2. Display each pattern on the projector (fullscreen OpenCV window)
 *   3. Trigger both cameras (GPIO on ARM64 / software trigger otherwise)
 *   4. Store ALL captured stereo pairs in memory
 *   5. After acquisition loop ends, compute absolute phase maps and modulation
 *   6. Save phase_map_left.exr, phase_map_right.exr,
 *          modulation_left.exr, modulation_right.exr  (and PNG previews)
 */

#include "StereoCameraSystem.hpp"
#include "FringeProcess.hpp"   
#include "DebugVisualizer.hpp"  

#ifdef STEREO_HAS_JETSON_GPIO
#  include "JetsonGPIO.hpp"
#endif

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <csignal>
#include <atomic>
#include <filesystem>
#include <chrono>
#include <thread>
#include <cstdio>
#include <regex>

namespace fs = std::filesystem;

static std::atomic<bool> g_running{true};

void signalHandler(int s) {
    std::cout << "\n[stereo_fringe] Caught signal " << s << ", aborting..." << std::endl;
    g_running = false;
}

// ---------------------------------------------------------------------------
// Monitor Helper (using xrandr for Linux)
// ---------------------------------------------------------------------------
struct MonitorInfo {
    std::string name;
    int width, height;
    int x, y;
};

static std::vector<MonitorInfo> get_monitors() {
    std::vector<MonitorInfo> monitors;
    // Execute xrandr command
    FILE* fp = popen("xrandr 2>/dev/null", "r");
    if (!fp) return monitors;
    
    char buffer[1024];
    // Match line like: "HDMI-1 connected 1920x1080+1920+0 (normal left inverted right x axis y axis)..."
    // OR "eDP-1 connected primary 1920x1080+0+0 ..."
    std::regex re("^([A-Za-z0-9\\-]+) connected (?:primary )?([0-9]+)x([0-9]+)\\+([0-9]+)\\+([0-9]+)");
    
    while (fgets(buffer, sizeof(buffer), fp)) {
        std::string line(buffer);
        std::smatch match;
        if (std::regex_search(line, match, re)) {
            MonitorInfo m;
            m.name = match[1].str();
            m.width = std::stoi(match[2].str());
            m.height = std::stoi(match[3].str());
            m.x = std::stoi(match[4].str());
            m.y = std::stoi(match[5].str());
            monitors.push_back(m);
        }
    }
    pclose(fp);
    return monitors;
}

// ---------------------------------------------------------------------------
// Configuration helpers
// ---------------------------------------------------------------------------
struct FringeCaptureConfig {
    // Projector / pattern settings
    cv::Size projectorResolution{1024, 768};
    cv::Size cameraResolution{1600, 1200};
    int      pixelsPerFringe{12};    // px_f
    int      nSteps{4};              // fringe phase steps
    int      projectorDisplayMs{50}; // ms to show each pattern before capture
    int     waitTime{5000};         // ms to wait before starting acquisition
    std::string projectorWindowName{"Projector"};
    int      projectorMonitor{1};    // legacy fallback
    std::string projectorMonitorName{""}; // e.g. "HDMI-0", "DP-1"

    // Output
    std::string outputDir{"fringe_results"};
    bool        saveRawFrames{false};

    static FringeCaptureConfig loadFromYaml(const std::string& path) {
        FringeCaptureConfig c;
        if (!fs::exists(path)) return c;
        YAML::Node root = YAML::LoadFile(path);
        if (!root["fringe_capture"]) return c;
        YAML::Node n = root["fringe_capture"];
        if (n["projector_resolution"]) {
            c.projectorResolution.width  = n["projector_resolution"]["width"].as<int>(c.projectorResolution.width);
            c.projectorResolution.height = n["projector_resolution"]["height"].as<int>(c.projectorResolution.height);
        }
        if (n["camera_resolution"]) {
            c.cameraResolution.width  = n["camera_resolution"]["width"].as<int>(c.cameraResolution.width);
            c.cameraResolution.height = n["camera_resolution"]["height"].as<int>(c.cameraResolution.height);
        }
        c.pixelsPerFringe      = n["pixels_per_fringe"].as<int>(c.pixelsPerFringe);
        c.nSteps               = n["n_steps"].as<int>(c.nSteps);
        c.projectorDisplayMs   = n["projector_display_ms"].as<int>(c.projectorDisplayMs);
        c.waitTime             = n["wait_time_ms"].as<int>(c.waitTime);
        c.projectorWindowName  = n["projector_window_name"].as<std::string>(c.projectorWindowName);
        if (n["projector_monitor_name"]) {
            c.projectorMonitorName = n["projector_monitor_name"].as<std::string>();
        } else if (n["projector_monitor"]) {
            c.projectorMonitor     = n["projector_monitor"].as<int>();
        }
        c.outputDir            = n["output_dir"].as<std::string>(c.outputDir);
        c.saveRawFrames        = n["save_raw_frames"].as<bool>(c.saveRawFrames);
        return c;
    }
};

static bool get_screen_resolution(const std::string& monitor_name, cv::Size& res, cv::Point& pos) {
    auto monitors = get_monitors();

    // 1. Imprime TODOS os monitores encontrados para debug
    std::cout << "[Pipeline] Encontrados " << monitors.size() << " monitores conectados:\n";
    for (const auto& monitor : monitors) {
        std::cout << " -> Monitor " << monitor.name 
                  << ": resolucao " << monitor.width << "x" << monitor.height
                  << ", posicao " << monitor.x << "x" << monitor.y << "\n";
    }

    for (const auto& monitor : monitors) {
        if (monitor.name == monitor_name) {
            res.width = monitor.width;
            res.height = monitor.height;
            pos.x = monitor.x;
            pos.y = monitor.y;
            
            std::cout << "[Pipeline] Monitor '" << monitor_name << "' selecionado com sucesso!\n";
            return true;
        }
    }

    std::cerr << "[Pipeline] ERROR: Monitor '" << monitor_name << "' not found!\n";
    return false;
}

// ---------------------------------------------------------------------------
// Save utilities
// ---------------------------------------------------------------------------
static void saveFloatMap(const std::string& basePath, const cv::Mat& mat,
                         const std::string& tag, double normScale = 1.0 / (2.0 * CV_PI)) {
    // EXR for lossless float32 storage
    const std::string exrPath = basePath + "_" + tag + ".exr";
    cv::Mat f32;
    mat.convertTo(f32, CV_32F);
    cv::imwrite(exrPath, f32);
    std::cout << "  Saved: " << exrPath << std::endl;

    // PNG preview (normalized to [0,255])
    const std::string pngPath = basePath + "_" + tag + ".png";
    cv::Mat preview;
    cv::normalize(mat, preview, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(preview, preview,
                      (tag.find("mod") != std::string::npos) ? cv::COLORMAP_JET : cv::COLORMAP_BONE);
    cv::imwrite(pngPath, preview);
    std::cout << "  Saved: " << pngPath << std::endl;
}

// ---------------------------------------------------------------------------
// Projector display helper
// ---------------------------------------------------------------------------
static void projectPattern(const cv::Mat& pattern, const std::string& winName, int displayMs) {
    // Convert to BGR for imshow if grayscale
    cv::Mat disp;
    if (pattern.channels() == 1)
        cv::cvtColor(pattern, disp, cv::COLOR_GRAY2BGR);
    else
        disp = pattern;

    cv::imshow(winName, disp);
    cv::waitKey(displayMs);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::string stereoCfgPath = "config/stereo_config.yaml";
    bool enableGpio = true;
    bool saveRawOverride = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--config" || arg == "-c") && i + 1 < argc)  stereoCfgPath = argv[++i];
        else if (arg == "--no-gpio")  enableGpio = false;
        else if (arg == "--save-raw") saveRawOverride = true;
        else if (arg == "--help" || arg == "-h") {
            std::cout <<
                "Usage: " << argv[0] << " [options]\n"
                "  --config <path>  Stereo + fringe YAML config (default: config/stereo_config.yaml)\n"
                "  --no-gpio        Force software trigger (ignore GPIO even on ARM64)\n"
                "  --save-raw       Save raw stereo frames to output_dir/raw/\n"
                "  --help, -h       Show this help\n";
            return 0;
        }
    }

    // -----------------------------------------------------------------------
    // Load configurations
    // -----------------------------------------------------------------------
    FringeCaptureConfig fCfg = FringeCaptureConfig::loadFromYaml(stereoCfgPath);
    if (saveRawOverride) fCfg.saveRawFrames = true;
    
    fs::create_directories(fCfg.outputDir);
    if (fCfg.saveRawFrames){
         fs::create_directories(fCfg.outputDir + "/raw/left");
         fs::create_directories(fCfg.outputDir + "/raw/right");
    }

    std::cout << "================================================================\n"
              << "   Stereo Fringe-Projection Acquisition Pipeline\n"
              << "================================================================\n";

    // -----------------------------------------------------------------------
    // Determine projector monitor
    // -----------------------------------------------------------------------
    cv::Point projectorPos(0, 0);
    if (!fCfg.projectorMonitorName.empty()) {
        if (!get_screen_resolution(fCfg.projectorMonitorName, fCfg.projectorResolution, projectorPos)) {
            // If monitor not found, we might want to fallback or exit.
            std::cerr << "         Falling back to default resolution and position.\n";
        }
    } else {
        // legacy fallback based on monitor index
        projectorPos.x = fCfg.projectorResolution.width * fCfg.projectorMonitor;
    }

    // -----------------------------------------------------------------------
    // Build patterns (in memory)
    // -----------------------------------------------------------------------
    std::cout << "[Pipeline] Generating fringe + GrayCode patterns...\n";
    FringeProcess processor(fCfg.projectorResolution, fCfg.cameraResolution,
                            fCfg.pixelsPerFringe, fCfg.nSteps);

    const int totalSteps = processor.get_total_steps();
    std::vector<cv::Mat> gcPatterns  = processor.get_gc_images();
    std::vector<cv::Mat> frPatterns  = processor.get_fr_image();

    // Concatenated display sequence: GrayCode images first, then fringe steps
    // (matches the internal storage order used by FringeProcess::set_images)
    std::vector<cv::Mat> displaySeq;
    displaySeq.insert(displaySeq.end(), gcPatterns.begin(), gcPatterns.end());
    displaySeq.insert(displaySeq.end(), frPatterns.begin(), frPatterns.end());

    std::cout << "  GrayCode patterns : " << gcPatterns.size() << "\n"
              << "  Fringe steps       : " << frPatterns.size() << "\n"
              << "  Total acquisitions : " << totalSteps << "\n";

    // -----------------------------------------------------------------------
    // Initialize camera system
    // -----------------------------------------------------------------------
    stereo::StereoCameraSystem stereoSystem;
    if (!stereoSystem.initializeFromYaml(stereoCfgPath)) {
        std::cerr << "[Pipeline] ERROR: Failed to initialize stereo cameras. Aborting.\n";
        return 1;
    }
    const auto& camCfg = stereoSystem.getConfig();

    // -----------------------------------------------------------------------
    // Initialize GPIO (ARM64 only)
    // -----------------------------------------------------------------------
#ifdef STEREO_HAS_JETSON_GPIO
    std::unique_ptr<stereo::JetsonGPIO> gpioTrigger;
    if (enableGpio && camCfg.gpioTrigger.enabled) {
        gpioTrigger = std::make_unique<stereo::JetsonGPIO>();
        stereo::PinType   pType  = (camCfg.gpioTrigger.pinType == "LINUX_GPIO_NUM")
                                    ? stereo::PinType::LINUX_GPIO_NUM
                                    : stereo::PinType::HEADER_PIN;
        stereo::JetsonModel jMdl = (camCfg.gpioTrigger.jetsonModel == "JETSON_AGX_ORIN")
                                    ? stereo::JetsonModel::JETSON_AGX_ORIN
                                    : stereo::JetsonModel::JETSON_ORIN_NANO_NX;

        if (!gpioTrigger->init(camCfg.gpioTrigger.pins, pType, jMdl, camCfg.gpioTrigger.activeLow)) {
            std::cerr << "[Pipeline] WARNING: GPIO init failed – falling back to software trigger.\n";
            gpioTrigger.reset();
        } else {
            std::cout << "[Pipeline] Jetson GPIO trigger ready on pins: ";
            for (int p : camCfg.gpioTrigger.pins) std::cout << p << " ";
            std::cout << ".\n";
        }
    }
#else
    std::cout << "[Pipeline] Non-ARM64 build: using Spinnaker software trigger.\n";
#endif

    // -----------------------------------------------------------------------
    // Create projector window
    // -----------------------------------------------------------------------
    cv::namedWindow(fCfg.projectorWindowName, cv::WINDOW_NORMAL);
    
    // 1. Force a small window size initially so it doesn't get blocked by the WM
    cv::resizeWindow(fCfg.projectorWindowName, 400, 300);
    
    // 2. Move the window safely inside the target monitor's bounds (offset by 100px)
    // This prevents the X11 Window Manager from snapping it to the primary monitor
    int safeX = projectorPos.x + 100;
    int safeY = projectorPos.y + 100;
    cv::moveWindow(fCfg.projectorWindowName, safeX, safeY);
    
    // 3. Show a black frame to force the OS to render the window
    cv::imshow(fCfg.projectorWindowName, cv::Mat::zeros(fCfg.projectorResolution, CV_8UC3));
    
    // 4. Wait long enough for the Window Manager (GNOME/Mutter) to process the move
    cv::waitKey(300);
    
    // 5. Enforce Fullscreen. It will maximize on the monitor it currently resides on.
    cv::setWindowProperty(fCfg.projectorWindowName,
                          cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    
    std::cout << "\n[Pipeline] Waiting " << fCfg.waitTime / 1000 << " seconds before starting acquisition...\n";
    for (int w = fCfg.waitTime / 1000; w > 0 && g_running; --w) {
        std::cout << "  Starting in " << w << "...\r" << std::flush;
        cv::waitKey(1000); // Wait 1 second while keeping UI responsive
    }
    std::cout << "  Starting now...      \n";
    
    if (!g_running) {
        stereoSystem.release();
        return 0;
    }

    // -----------------------------------------------------------------------
    // Acquisition loop – all images stored in RAM
    // -----------------------------------------------------------------------
    // Storage: index = step counter (0..totalSteps-1), each element is {left, right}
    std::vector<cv::Mat> capturedLeft(totalSteps);
    std::vector<cv::Mat> capturedRight(totalSteps);
    int acquiredCount = 0;

    std::cout << "\n[Pipeline] Starting acquisition of " << totalSteps << " patterns...\n"
              << "----------------------------------------------------------------\n";

    for (int step = 0; step < totalSteps && g_running; ++step) {
        // 1. Display the current pattern on the projector
        projectPattern(displaySeq[step], fCfg.projectorWindowName, fCfg.projectorDisplayMs);

        // 2. Trigger cameras and receive synchronized pair
        stereo::StereoFrame frame;

#ifdef STEREO_HAS_JETSON_GPIO
        if (gpioTrigger && gpioTrigger->isInitialized()) {
            frame = stereoSystem.triggerAndReceive(*gpioTrigger,
                                                   camCfg.gpioTrigger.pulseDurationUs,
                                                   camCfg.acquisition.timeoutMs);
        } else {
            frame = stereoSystem.softwareTriggerAndReceive(camCfg.acquisition.timeoutMs);
        }
#else
        frame = stereoSystem.softwareTriggerAndReceive(camCfg.acquisition.timeoutMs);
#endif

        if (!frame.valid) {
            std::cerr << "[Pipeline] WARN: Step " << step << " – frame pair invalid, skipping.\n";
            // Insert empty mats to keep indices aligned
            capturedLeft[step]  = cv::Mat::zeros(fCfg.cameraResolution, CV_8UC1);
            capturedRight[step] = cv::Mat::zeros(fCfg.cameraResolution, CV_8UC1);
            continue;
        }

        // 3. Convert Spinnaker images → cv::Mat and store in RAM
        // Spinnaker raw data: GetData() / GetWidth() / GetHeight()
        const size_t w = frame.leftImage->GetWidth();
        const size_t h = frame.leftImage->GetHeight();

        cv::Mat leftMat(static_cast<int>(h), static_cast<int>(w), CV_8UC1,
                        frame.leftImage->GetData());
        cv::Mat rightMat(static_cast<int>(h), static_cast<int>(w), CV_8UC1,
                         frame.rightImage->GetData());
        cv::rotate(leftMat, leftMat, 2);
        cv::rotate(rightMat, rightMat, 0);

        // Deep copy before releasing Spinnaker buffers
        leftMat.copyTo(capturedLeft[step]);
        rightMat.copyTo(capturedRight[step]);

        // Feed into FringeProcess for bookkeeping (it keeps its own copies via set_images)
        processor.set_images(capturedLeft[step], capturedRight[step], step);

        ++acquiredCount;
        std::cout << "  [" << std::setw(2) << (step + 1) << "/" << totalSteps << "]"
                  << "  sync Δ=" << std::fixed << std::setprecision(3)
                  << frame.syncDeltaMs << " ms"
                  << "  frameID L=" << frame.leftFrameID
                  << " R=" << frame.rightFrameID << "\n";

        // 4. Optionally save raw frames to disk as we go
        if (fCfg.saveRawFrames) {
            std::ostringstream ol, or_;
            ol << fCfg.outputDir << "/raw/left/L"  << std::setw(3) << std::setfill('0') << step << ".png";
            or_ << fCfg.outputDir << "/raw/right/R" << std::setw(3) << std::setfill('0') << step << ".png";
            cv::imwrite(ol.str(),  capturedLeft[step]);
            cv::imwrite(or_.str(), capturedRight[step]);
        }

        // 5. Release Spinnaker buffers back to driver pool
        frame.leftImage->Release();
        frame.rightImage->Release();
    }

    // Turn off projector (show black)
    cv::imshow(fCfg.projectorWindowName, cv::Mat::zeros(fCfg.projectorResolution, CV_8UC3));
    cv::waitKey(20);
    cv::destroyWindow(fCfg.projectorWindowName);

    // -----------------------------------------------------------------------
    // Stop cameras before heavy processing
    // -----------------------------------------------------------------------
    stereoSystem.stopAcquisition();
#ifdef STEREO_HAS_JETSON_GPIO
    if (gpioTrigger) gpioTrigger->release();
#endif

    std::cout << "\n[Pipeline] Acquisition complete. "
              << acquiredCount << "/" << totalSteps << " frames captured.\n";

    if (acquiredCount == 0) {
        std::cerr << "[Pipeline] No frames were captured. Cannot compute phase. Exiting.\n";
        stereoSystem.release();
        return 1;
    }

    // // -----------------------------------------------------------------------
    // // Phase + modulation computation (all in RAM)
    // // -----------------------------------------------------------------------
    // std::cout << "\n[Pipeline] Computing absolute phase maps and modulation...\n";
    // auto t0 = std::chrono::steady_clock::now();

    // // calculate_abs_phi_images returns {abs_phi_l, abs_phi_r, mod_l, mod_r}
    // std::vector<cv::Mat> results = processor.calculate_abs_phi_images(/*save_data=*/false);

    // auto t1 = std::chrono::steady_clock::now();
    // std::cout << "  Phase computation time: "
    //           << std::chrono::duration<double>(t1 - t0).count() << " s\n";

    // // -----------------------------------------------------------------------
    // // Save outputs
    // // -----------------------------------------------------------------------
    // if (results.size() < 4) {
    //     std::cerr << "[Pipeline] Unexpected result count from calculate_abs_phi_images. Aborting save.\n";
    //     stereoSystem.release();
    //     return 1;
    // }

    // const cv::Mat& phaseLeft   = results[0]; // abs_phi_l  (CV_64FC1, radians)
    // const cv::Mat& phaseRight  = results[1]; // abs_phi_r  (CV_64FC1, radians)
    // const cv::Mat& modLeft     = results[2]; // mod_l      (CV_64FC1, [0..1] normalised)
    // const cv::Mat& modRight    = results[3]; // mod_r      (CV_64FC1)

    // std::cout << "\n[Pipeline] Saving results to: " << fCfg.outputDir << "/\n";
    // const std::string base = fCfg.outputDir + "/";

    // saveFloatMap(base + "phase_map",   phaseLeft,  "left");
    // saveFloatMap(base + "phase_map",   phaseRight, "right");
    // saveFloatMap(base + "modulation",  modLeft,    "left");
    // saveFloatMap(base + "modulation",  modRight,   "right");

    // // Also save the debug mosaic from DebugVisualizer
    // DebugVisualizer::saveDebugMosaic(base + "debug_mosaic.png",
    //                                  phaseLeft, phaseRight, modLeft, modRight);

    // std::cout << "\n[Pipeline] All results saved successfully.\n"
    //           << "================================================================\n";

    stereoSystem.release();
    return 0;
}
