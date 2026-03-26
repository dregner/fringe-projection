#include "FringeProcess.hpp"
#include <iostream>

int main() {
    cv::Size proj_res(1024, 768);
    cv::Size cam_res(1024, 768); // Para simplificar o teste, usei a mesma
    int px_f = 80;
    int steps = 4;

    FringeProcess processor(proj_res, cam_res, px_f, steps);

    // Simulação: Pegando os padrões gerados e "fingindo" que a câmera os leu
    // No uso real, aqui você usaria cv::imread ou captura de câmera
    std::vector<cv::Mat> gc_patterns = processor.get_gc_images();
    std::vector<cv::Mat> fr_patterns = processor.get_fr_image();

    // processor.show_fr_image();
    // processor.show_gc_image();

    std::cout << "Fringe images: " << fr_patterns.size() << std::endl;
    std::cout << "GrayCode images: " << gc_patterns.size() << std::endl;
    std::cout << "total: " << processor.get_total_steps() << std::endl;

    int counter = 0;
    for(auto& img : gc_patterns) {
        processor.set_images(img, img, counter++);
    }
    for(auto& img : fr_patterns) {
        processor.set_images(img, img, counter++);
    }

    std::cout << "Receive images" << std::endl;

    // Processamento
    std::vector<cv::Mat> results = processor.calculate_abs_phi_images();

    // Visualização do resultado final (Fase Absoluta Esquerda)
    cv::Mat display;
    results[0].convertTo(display, CV_8U, 255.0 / (2.0 * CV_PI * 10)); // Normalizado para ver algo
    cv::imshow("Absolute Phase L (Normalized)", display);
    cv::waitKey(0);

    return 0;
}