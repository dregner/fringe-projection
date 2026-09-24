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
#include "SDLProjector.hpp"   
#include "DebugVisualizer.hpp"  

#ifdef STEREO_HAS_JETSON_GPIO
#  include "GpioController.hpp"
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
    int     waitTime{2000};         // ms to wait before starting acquisition
    std::string projectorWindowName{"Projector"};
    int      projectorMonitor{1};    // legacy fallback
    std::string projectorMonitorName{""}; // e.g. "HDMI-0", "DP-1"
    
    // Output
    std::string outputDir{"fringe_results"};
    bool        saveRawFrames{false};
    int     nImagesZNCC{5};
    int     motorStep{20};
    int     warmupTrigger{0};
    int     ZNCCdelay{200};

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
        if(n["zncc"]){
            c.nImagesZNCC     = n["num_images"].as<int>(c.nImagesZNCC);
            c.motorStep       = n["motor_steps"].as<int>(c.motorStep);
            c.warmupTrigger   = n["warmup_triggers"].as<int>(c.warmupTrigger);
            c.ZNCCdelay       = n["delay_us"].as<int>(c.ZNCCdelay);
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
// Projector display helper
// ---------------------------------------------------------------------------

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
         fs::create_directories(fCfg.outputDir + "/fringe/left");
         fs::create_directories(fCfg.outputDir + "/fringe/right");
         fs::create_directories(fCfg.outputDir + "/zncc/left");
         fs::create_directories(fCfg.outputDir + "/zncc/right");
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
    std::vector<cv::Mat> gcPatterns  = processor.get_gc_images("blue");
    std::vector<cv::Mat> frPatterns  = processor.get_fr_image("blue");

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
std::unique_ptr<stereo::GpioController> gpioCtrl;
#ifdef STEREO_HAS_JETSON_GPIO
    if (enableGpio && camCfg.gpioTrigger.enabled) {
        gpioCtrl = std::make_unique<stereo::GpioController>();
        if (!gpioCtrl->init(camCfg.gpioTrigger, camCfg.zncc)) {
            std::cerr << "[Pipeline] WARNING: GPIO init failed – falling back to software trigger.\n";
            gpioCtrl.reset();
        } else {
            std::cout << "[Pipeline] GpioController initialized successfully.\n";
        }
    }
#else
    std::cout << "[Pipeline] Non-ARM64 build: using Spinnaker software trigger.\n";
#endif

    // -----------------------------------------------------------------------
    // Create projector window
    // -----------------------------------------------------------------------
    
    SDLProjector projector(fCfg.projectorResolution.width, fCfg.projectorResolution.height, fCfg.projectorWindowName, fCfg.projectorMonitor);
    if (!projector.init()) {
        std::cerr << "[Pipeline] ERROR: Failed to initialize SDL projector.\n";
        return 1;
    }
    projector.waitMs(300);

    std::cout << "[Pipeline] READY\n" << std::flush;
    
    while (g_running) {
        std::string cmd_line;
        if (!std::getline(std::cin, cmd_line)) {
            break;
        }
        if (cmd_line.empty()) continue;
        
        std::istringstream iss(cmd_line);
        std::string cmd;
        iss >> cmd;
        
        if (cmd == "QUIT") {
            break;
        } else if (cmd == "FRINGE") {
            std::string out_dir;
            iss >> out_dir;
            
            FringeCaptureConfig newCfg = FringeCaptureConfig::loadFromYaml(stereoCfgPath);
            fCfg.saveRawFrames = saveRawOverride ? true : newCfg.saveRawFrames;
            fCfg.pixelsPerFringe = newCfg.pixelsPerFringe;
            fCfg.nSteps = newCfg.nSteps;
            fCfg.projectorDisplayMs = newCfg.projectorDisplayMs;
            
            fs::create_directories(fCfg.outputDir);
            if (fCfg.saveRawFrames){
                 fs::create_directories(fCfg.outputDir + "/fringe/left");
                 fs::create_directories(fCfg.outputDir + "/fringe/right");
                 fs::create_directories(fCfg.outputDir + "/fringe/projected/");
            }

            // Build patterns
            std::cout << "[Pipeline] Generating fringe + GrayCode patterns...\n";
            FringeProcess processor(fCfg.projectorResolution, fCfg.cameraResolution,
                                    fCfg.pixelsPerFringe, fCfg.nSteps);

            const int totalSteps = processor.get_total_steps();
            std::vector<cv::Mat> gcPatterns  = processor.get_gc_images("blue");
            std::vector<cv::Mat> frPatterns  = processor.get_fr_image("blue");

            std::vector<cv::Mat> displaySeq;
            displaySeq.insert(displaySeq.end(), gcPatterns.begin(), gcPatterns.end());
            displaySeq.insert(displaySeq.end(), frPatterns.begin(), frPatterns.end());

            std::vector<cv::Mat> capturedLeft(totalSteps);
            std::vector<cv::Mat> capturedRight(totalSteps);
            int acquiredCount = 0;

            std::cout << "\n[Pipeline] Starting acquisition of " << totalSteps << " patterns...\n"
                      << "----------------------------------------------------------------\n";

            // 0. Flush camera buffer of any stale frames (e.g. from stray triggers or startup)
            // This prevents "losing the first frame" due to the camera serving an old frame.
            std::cout << "[Pipeline] Flushing stale camera frames (ignore timeout warnings)..." << std::endl;
            while (true) {
                stereo::StereoFrame stale = stereoSystem.receiveStereoPair(20);
                if (!stale.valid) break; // Buffer is empty when it times out
            }
            std::cout << "[Pipeline] Buffer flush complete.\n" << std::endl;

            // 1. Show the VERY FIRST pattern (step 0) and wait.
            // This ensures step 0 is physically on the projector screen before the loop starts.
            projector.show(displaySeq[0]);
            std::this_thread::sleep_for(std::chrono::milliseconds(120));

            for (int step = 0; step < totalSteps && g_running; ++step) {
                // At this exact moment, displaySeq[step] is ALREADY on the projector.
                // Trigger the cameras immediately to capture it.
                stereo::StereoFrame frame;

            #ifdef STEREO_HAS_JETSON_GPIO
                if (gpioCtrl && gpioCtrl->isInitialized()) {
                    frame = stereoSystem.triggerAndReceive(*gpioCtrl,
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
                    capturedLeft[step]  = cv::Mat::zeros(fCfg.cameraResolution, CV_8UC1);
                    capturedRight[step] = cv::Mat::zeros(fCfg.cameraResolution, CV_8UC1);
                    continue;
                }

                const size_t w = frame.leftImage->GetWidth();
                const size_t h = frame.leftImage->GetHeight();

                cv::Mat leftMat(static_cast<int>(h), static_cast<int>(w), CV_8UC1,
                                frame.leftImage->GetData());
                cv::Mat rightMat(static_cast<int>(h), static_cast<int>(w), CV_8UC1,
                                 frame.rightImage->GetData());

                leftMat.copyTo(capturedLeft[step]);
                rightMat.copyTo(capturedRight[step]);

                ++acquiredCount;
                std::cout << "  [" << std::setw(2) << (step + 1) << "/" << totalSteps << "]"
                          << "  sync Δ=" << std::fixed << std::setprecision(3)
                          << frame.syncDeltaMs << " ms"
                          << "  frameID L=" << frame.leftFrameID
                          << " R=" << frame.rightFrameID << "\n";

                frame.leftImage->Release();
                frame.rightImage->Release();

                // 3. Prepare the NEXT pattern (if not the last step)
                if (step + 1 < totalSteps) {
                    projector.show(displaySeq[step + 1]);
                    std::this_thread::sleep_for(std::chrono::milliseconds(fCfg.projectorDisplayMs));
                }
            }
            if (fCfg.saveRawFrames) {
                fs::create_directories(fCfg.outputDir + "/fringe/projected");
                for (int i = 0; i < totalSteps; ++i) {
                    std::ostringstream out_l, out_r, out_p;
                    out_l << fCfg.outputDir << "/fringe/left/L"       << std::setw(3) << std::setfill('0') << i << ".png";
                    out_r << fCfg.outputDir << "/fringe/right/R"      << std::setw(3) << std::setfill('0') << i << ".png";
                    out_p << fCfg.outputDir << "/fringe/projected/P"  << std::setw(3) << std::setfill('0') << i << ".png";
                    cv::imwrite(out_l.str(), capturedLeft[i]);
                    cv::imwrite(out_r.str(), capturedRight[i]);
                    cv::imwrite(out_p.str(), displaySeq[i]);
                }
            }

            // Turn off projector (show black)
            projector.show(cv::Mat::zeros(fCfg.projectorResolution, CV_8UC1));
            projector.waitMs(20);
            
            std::cout << "[Pipeline] ACQUIRE_DONE\n" << std::flush;
        } else if (cmd == "ZNCC") {
            fs::create_directories(fCfg.outputDir);
            if (fCfg.saveRawFrames){
                 fs::create_directories(fCfg.outputDir + "/zncc/left");
                 fs::create_directories(fCfg.outputDir + "/zncc/right");
            }
            std::vector<cv::Mat> capturedLeft_zncc(fCfg.nImagesZNCC);
            std::vector<cv::Mat> capturedRight_zncc(fCfg.nImagesZNCC);

            float angle_per_step = (fCfg.motorStep / 2048.0f) * 360.0f;
            std::cout << "[Pipeline] ZNCC Starting... Laser ON\n";
            if(gpioCtrl) gpioCtrl->setLaser(true);
            if (fCfg.warmupTrigger > 0) {
                for (int w = 0; w < fCfg.warmupTrigger; ++w) {
            #ifdef STEREO_HAS_JETSON_GPIO
                    if (gpioCtrl && gpioCtrl->isInitialized()) {
                        gpioCtrl->generatePulse(camCfg.gpioTrigger.pulseDurationUs);
                    }
            #endif
                    std::this_thread::sleep_for(std::chrono::milliseconds(40));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            int acquiredCount = 0;
            for (int k = 0; k < fCfg.nImagesZNCC && g_running; ++k) {
                if(gpioCtrl) gpioCtrl->moveMotor(angle_per_step);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                stereo::StereoFrame frame;
            #ifdef STEREO_HAS_JETSON_GPIO
                if (gpioCtrl && gpioCtrl->isInitialized()) {
                    frame = stereoSystem.triggerAndReceive(*gpioCtrl, camCfg.gpioTrigger.pulseDurationUs, camCfg.acquisition.timeoutMs);
                } else {
                    frame = stereoSystem.softwareTriggerAndReceive(camCfg.acquisition.timeoutMs);
                }
            #else
                frame = stereoSystem.softwareTriggerAndReceive(camCfg.acquisition.timeoutMs);
            #endif
                std::this_thread::sleep_for(std::chrono::milliseconds(30));

                if (!frame.valid) {
                    std::cerr << "[Pipeline] WARN: Step " << k << " – frame pair invalid, skipping.\n";
                    capturedLeft_zncc[k]  = cv::Mat::zeros(fCfg.cameraResolution, CV_8UC1);
                    capturedRight_zncc[k] = cv::Mat::zeros(fCfg.cameraResolution, CV_8UC1);
                    continue;
                }
                const size_t w = frame.leftImage->GetWidth();
                const size_t h = frame.leftImage->GetHeight();

                cv::Mat leftMat(static_cast<int>(h), static_cast<int>(w), CV_8UC1, frame.leftImage->GetData());
                cv::Mat rightMat(static_cast<int>(h), static_cast<int>(w), CV_8UC1, frame.rightImage->GetData());

                leftMat.copyTo(capturedLeft_zncc[k]);
                rightMat.copyTo(capturedRight_zncc[k]);
                ++acquiredCount;
                std::cout << "  [" << std::setw(2) << (k + 1) << "/" << fCfg.nImagesZNCC << "]"
                            << "  sync Δ=" << std::fixed << std::setprecision(3)
                            << frame.syncDeltaMs << " ms"
                            << "  frameID L=" << frame.leftFrameID
                            << " R=" << frame.rightFrameID << "\n";
                            
                
                frame.leftImage->Release();
                frame.rightImage->Release();
            }
            if(gpioCtrl) gpioCtrl->setLaser(false);
            float return_angle = -(fCfg.nImagesZNCC * angle_per_step);
            if(gpioCtrl) gpioCtrl->moveMotor(return_angle);
            if (fCfg.saveRawFrames) {
                for(int i=0; i< fCfg.nImagesZNCC; ++i){
                    std::ostringstream out_l, out_r;
                    out_l << fCfg.outputDir << "/zncc/left/L"  << std::setw(3) << std::setfill('0') << (i) << ".png";
                    out_r << fCfg.outputDir << "/zncc/right/R" << std::setw(3) << std::setfill('0') << (i) << ".png";
                    cv::imwrite(out_l.str(), capturedLeft_zncc[i]);
                    cv::imwrite(out_r.str(), capturedRight_zncc[i]);
                }
            }
            std::cout << "[Pipeline] ZNCC_DONE " << acquiredCount << "\n" << std::flush;
        }
    }

    // -----------------------------------------------------------------------
    // Stop cameras
    // -----------------------------------------------------------------------
    stereoSystem.stopAcquisition();
#ifdef STEREO_HAS_JETSON_GPIO
    if (gpioCtrl) gpioCtrl->release();
#endif
    
    projector.close();
    stereoSystem.release();
    return 0;
}

