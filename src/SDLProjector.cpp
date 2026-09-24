#include "SDLProjector.hpp"
#include <iostream>
#include <chrono>
#include <thread>

SDLProjector::SDLProjector(int width, int height, const std::string& title, int monitor_index)
    : m_width(width), m_height(height), m_title(title), m_monitor_index(monitor_index),
      m_window(nullptr), m_renderer(nullptr), m_texture(nullptr)
{
}

SDLProjector::~SDLProjector() {
    close();
}

bool SDLProjector::init() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "[SDLProjector] SDL could not initialize! SDL_Error: " << SDL_GetError() << "\n";
        return false;
    }

    int displayCount = SDL_GetNumVideoDisplays();
    if (m_monitor_index >= displayCount) {
        std::cerr << "[SDLProjector] Monitor index " << m_monitor_index << " out of bounds. Max: " << (displayCount - 1) << ". Using 0.\n";
        m_monitor_index = 0;
    }

    int x = SDL_WINDOWPOS_UNDEFINED_DISPLAY(m_monitor_index);
    int y = SDL_WINDOWPOS_UNDEFINED_DISPLAY(m_monitor_index);

    m_window = SDL_CreateWindow(m_title.c_str(), x, y, m_width, m_height, SDL_WINDOW_FULLSCREEN_DESKTOP | SDL_WINDOW_SHOWN);
    if (!m_window) {
        std::cerr << "[SDLProjector] Window could not be created! SDL_Error: " << SDL_GetError() << "\n";
        return false;
    }

    // Use hardware acceleration and enable vsync
    m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!m_renderer) {
        std::cerr << "[SDLProjector] Renderer could not be created! SDL_Error: " << SDL_GetError() << "\n";
        return false;
    }

    // Create a texture that we will update with cv::Mat data. We use SDL_PIXELFORMAT_BGR24 for OpenCV BGR
    m_texture = SDL_CreateTexture(m_renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, m_width, m_height);
    if (!m_texture) {
        std::cerr << "[SDLProjector] Texture could not be created! SDL_Error: " << SDL_GetError() << "\n";
        return false;
    }

    SDL_ShowCursor(SDL_DISABLE);
    
    // Clear screen
    SDL_SetRenderDrawColor(m_renderer, 0, 0, 0, 255);
    SDL_RenderClear(m_renderer);
    SDL_RenderPresent(m_renderer);

    return true;
}

void SDLProjector::close() {
    if (m_texture) {
        SDL_DestroyTexture(m_texture);
        m_texture = nullptr;
    }
    if (m_renderer) {
        SDL_DestroyRenderer(m_renderer);
        m_renderer = nullptr;
    }
    if (m_window) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }
    SDL_ShowCursor(SDL_ENABLE);
    SDL_Quit();
}

void SDLProjector::show(const cv::Mat& image) {
    if (!m_texture || !m_renderer) return;

    cv::Mat displayImage = image;

    // Convert grayscale to BGR so we can use the BGR24 texture
    if (image.channels() == 1) {
        cv::cvtColor(image, displayImage, cv::COLOR_GRAY2BGR);
    }
    
    // Resize if dimensions do not match the texture dimensions
    if (displayImage.cols != m_width || displayImage.rows != m_height) {
        cv::resize(displayImage, displayImage, cv::Size(m_width, m_height));
    }

    // Update texture
    SDL_UpdateTexture(m_texture, nullptr, displayImage.ptr(), displayImage.step);

    // Render
    SDL_RenderClear(m_renderer);
    SDL_RenderCopy(m_renderer, m_texture, nullptr, nullptr);
    SDL_RenderPresent(m_renderer);

    // Pump events to keep window responsive
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_QUIT) {
            // handle quit if needed
        }
    }
}

void SDLProjector::waitMs(int ms) {
    auto start = std::chrono::steady_clock::now();
    while (true) {
        auto now = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (diff >= ms) break;

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            // handle events to keep window responsive
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
