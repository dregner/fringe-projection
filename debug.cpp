#include "FringeProcess.hpp"
#include <iostream>
#include <vector>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <filesystem>

void load_stereo_images(std::string path, std::vector<cv::Mat>& left_imgs, std::vector<cv::Mat>& right_imgs) {
    std::string path_left = path + "/left";
    std::string path_right = path + "/right";

    std::vector<std::string> files_l, files_r;

    // Lista os arquivos
    for (const auto& entry : std::filesystem::directory_iterator(path_left)) 
        files_l.push_back(entry.path().string());
    
    for (const auto& entry : std::filesystem::directory_iterator(path_right)) 
        files_r.push_back(entry.path().string());

    // Ordena (L000, L001...) para não quebrar a sequência da franja
    std::sort(files_l.begin(), files_l.end());
    std::sort(files_r.begin(), files_r.end());

    // Carrega as matrizes
    for (size_t i = 0; i < files_l.size(); i++) {
        left_imgs.push_back(cv::imread(files_l[i], cv::IMREAD_GRAYSCALE));
        right_imgs.push_back(cv::imread(files_r[i], cv::IMREAD_GRAYSCALE));
    }
}


int main() {
    cv::Size resolucao_projetor(1920, 1080);
    cv::Size resolucao_camera(2448, 2048);
    int px_f = 64;
    int steps = 12;
    
    std::string path = "/home/daniel/Pictures/structured-light";
    std::vector<cv::Mat> left_imgs;
    std::vector<cv::Mat> right_imgs;

    load_stereo_images(path, left_imgs, right_imgs);

    std::cout << "Inicializando o processador de franjas..." << std::endl;
    FringeProcess processor(resolucao_projetor, resolucao_camera, px_f, steps);

    std::cout << "Atualizando imagens salvas no processo..." << std::endl;
    for (int i = 0; i < left_imgs.size(); i++) {
        processor.set_images(left_imgs[i], right_imgs[i], i);
    }

    if (left_imgs.empty()) {
        std::cerr << "ERRO CRÍTICO: Vetor de imagens está vazio. Verifique o caminho das pastas!" << std::endl;
        return -1;
    }

    std::cout << "Calculando a fase absoluta..." << std::endl;
    std::vector<cv::Mat> resultados = processor.calculate_abs_phi_images(true);


    std::cout << "Processamento concluido com sucesso. Verifique o arquivo PNG na pasta." << std::endl;
    return 0;
}