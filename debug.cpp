#include "FringeProcess.hpp"
#include "DebugVisualizer.hpp"
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

int main() {
    cv::Size resolucao_projetor(1024, 768);
    cv::Size resolucao_camera(1024, 768);
    int px_f = 20;
    int steps = 4;

    std::cout << "Inicializando o processador de franjas..." << std::endl;
    FringeProcess processor(resolucao_projetor, resolucao_camera, px_f, steps);

    std::vector<cv::Mat> padroes_graycode = processor.get_gc_images();
    std::vector<cv::Mat> padroes_franjas = processor.get_fr_image();

    std::cout << "Simulando a captura estéreo das imagens..." << std::endl;
    int counter = 0;
    for (const auto& img : padroes_graycode) {
        processor.set_images(img, img, counter++);
    }
    for (const auto& img : padroes_franjas) {
        processor.set_images(img, img, counter++);
    }

    std::cout << "Calculando a fase absoluta..." << std::endl;
    std::vector<cv::Mat> resultados = processor.calculate_abs_phi_images();

    std::cout << "Gerando o mosaico de debug visual..." << std::endl;
    DebugVisualizer::saveDebugImage("grafico_mapa_de_fase_cpp.png", 
                                    resultados[0], resultados[1], 
                                    resultados[2], resultados[3], 
                                    resultados[0], resultados[1]);

    std::cout << "Processamento concluido com sucesso. Verifique o arquivo PNG na pasta." << std::endl;
    return 0;
}