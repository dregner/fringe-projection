#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <SDL2/SDL.h>

class SDLProjector {
public:
    SDLProjector(int width, int height, const std::string& title = "Projector", int monitor_index = 0);
    ~SDLProjector();

    bool init();
    void close();

    // Display an OpenCV Mat. Mat must be CV_8UC3 (BGR or RGB) or CV_8UC1 (Grayscale).
    // The projection will scale to fill the screen if dimensions do not match.
    void show(const cv::Mat& image);
    void waitMs(int ms);

private:
    int m_width;
    int m_height;
    std::string m_title;
    int m_monitor_index;

    SDL_Window* m_window;
    SDL_Renderer* m_renderer;
    SDL_Texture* m_texture;
};
