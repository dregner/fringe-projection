#include "DebugVisualizer.hpp"

cv::Mat DebugVisualizer::visualizeMatrix(const cv::Mat& input, int colormap) {
    cv::Mat normalized, display;
    // Normaliza para 0-255 para exibição
    cv::normalize(input, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    if (colormap != -1) {
        cv::applyColorMap(normalized, display, colormap);
    } else {
        cv::cvtColor(normalized, display, cv::COLOR_GRAY2BGR);
    }
    return display;
}

cv::Mat DebugVisualizer::plot1D(const cv::Mat& data1, const cv::Mat& data2, 
                               const std::string& title, cv::Scalar color1, cv::Scalar color2) {
    int width = 400;
    int height = 300;
    cv::Mat plot = cv::Mat::zeros(height, width, CV_8UC3);
    plot.setTo(cv::Scalar(255, 255, 255)); // Fundo branco

    auto drawData = [&](const cv::Mat& data, cv::Scalar color, bool secondary) {
        double min, max;
        cv::minMaxLoc(data, &min, &max);
        for (int i = 1; i < data.cols; i++) {
            float v1 = data.at<double>(0, i - 1);
            float v2 = data.at<double>(0, i);
            
            int x1 = (i - 1) * width / data.cols;
            int x2 = i * width / data.cols;
            int y1 = height - ((v1 - min) / (max - min + 1e-6) * height);
            int y2 = height - ((v2 - min) / (max - min + 1e-6) * height);
            
            cv::line(plot, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
        }
    };

    drawData(data1, color1, false); // Linha Cinza (Fase)
    drawData(data2, color2, true);  // Linha Vermelha (QSI)
    
    cv::putText(plot, title, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);
    return plot;
}

void DebugVisualizer::saveDebugImage(const std::string& filename,
                                     const cv::Mat& abs_phi_l, const cv::Mat& abs_phi_r,
                                     const cv::Mat& mod_l, const cv::Mat& mod_r,
                                     const cv::Mat& qsi_l, const cv::Mat& qsi_r) {
    
    int mid_l = abs_phi_l.rows / 2;
    int mid_r = abs_phi_r.rows / 2;

    // Linha 1: Gráficos 1D (Middle Index)
    cv::Mat p1 = plot1D(abs_phi_l.row(mid_l), qsi_l.row(mid_l), "Abs Phi L 1D", cv::Scalar(100,100,100), cv::Scalar(0,0,255));
    cv::Mat p2 = plot1D(abs_phi_r.row(mid_r), qsi_r.row(mid_r), "Abs Phi R 1D", cv::Scalar(100,100,100), cv::Scalar(0,0,255));

    // Linha 2: Abs Phi 2D
    cv::Mat p3 = visualizeMatrix(abs_phi_l);
    cv::Mat p4 = visualizeMatrix(abs_phi_r);

    // Linha 3: Modulation Maps (Jet)
    cv::Mat p5 = visualizeMatrix(mod_l, cv::COLORMAP_JET);
    cv::Mat p6 = visualizeMatrix(mod_r, cv::COLORMAP_JET);

    // Redimensionar tudo para o mesmo tamanho antes de concatenar
    cv::Size s(400, 300);
    cv::resize(p1, p1, s); cv::resize(p2, p2, s);
    cv::resize(p3, p3, s); cv::resize(p4, p4, s);
    cv::resize(p5, p5, s); cv::resize(p6, p6, s);

    // Montagem da grade (Grid 3x2)
    cv::Mat row1, row2, row3, full;
    cv::hconcat(p1, p2, row1);
    cv::hconcat(p3, p4, row2);
    cv::hconcat(p5, p6, row3);
    cv::vconcat(std::vector<cv::Mat>{row1, row2, row3}, full);

    cv::imwrite(filename, full);
    std::cout << "Debug image saved as: " << filename << std::endl;
}