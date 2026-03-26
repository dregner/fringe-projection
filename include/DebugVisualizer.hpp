#ifndef DEBUGVISUALIZER_HPP
#define DEBUGVISUALIZER_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class DebugVisualizer {
public:
    // Cria a composição 3x2 igual ao seu código Python
    static void saveDebugImage(const std::string& filename,
                               const cv::Mat& abs_phi_l, const cv::Mat& abs_phi_r,
                               const cv::Mat& mod_l, const cv::Mat& mod_r,
                               const cv::Mat& qsi_l, const cv::Mat& qsi_r);

private:
    // Auxiliar para converter mapas de calor (jet, gray, etc)
    static cv::Mat visualizeMatrix(const cv::Mat& input, int colormap = -1);
    
    // Desenha um gráfico 1D (linha) em uma Mat
    static cv::Mat plot1D(const cv::Mat& data1, const cv::Mat& data2, 
                          const std::string& title, cv::Scalar color1, cv::Scalar color2);
};

#endif