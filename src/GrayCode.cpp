#include "GrayCode.hpp"
#include <iostream>
#include <cmath>
#include <set>

GrayCode::GrayCode(cv::Size resolution, int axis, int px_f)
    : width(resolution.width), height(resolution.height), px_f(px_f){
    
    // Aloca as imagens: 2 canais extras baseados no original em Python
    n_bits = static_cast<int>(std::ceil(std::log2(static_cast<double>(width) / px_f) + 1.0)) + 2;
    gc_images.resize(n_bits);

    for (int i = 0; i < n_bits; ++i) {
        gc_images[i] = cv::Mat::zeros(height, width, CV_8UC1);
    }
    create_graycode_images();
}

std::vector<cv::Mat> GrayCode::get_gc_images() const {
    return gc_images;
}

void GrayCode::show_gc_image() const {
    for (size_t i = 0; i < gc_images.size(); ++i) {
        std::cout << i << std::endl;
        cv::imshow("Image", gc_images[i]);
        cv::waitKey(0);
    }
    cv::destroyAllWindows();
}

std::vector<std::string> GrayCode::list_to_graycode_binary(const std::vector<int>& int_list, int bit_length) const {
    std::vector<std::string> graycode_list;
    for (int n : int_list) {
        int graycode = n ^ (n >> 1);
        std::string binary_str = "";
        for (int k = bit_length - 1; k >= 0; --k) {
            binary_str += ((graycode >> k) & 1) ? "1" : "0";
        }
        graycode_list.push_back(binary_str);
    }
    return graycode_list;
}

void GrayCode::create_graycode_images() {
    gc_images[0].setTo(255); // gc_images[:, :, 0] = 255

    std::vector<int> width_list;
    int max_val = 1 << n_bits; // Equivalente a 2 ** n_bits
    int repeat_count = px_f / 2;
    
    for (int element = 0; element < max_val; ++element) {
        for (int r = 0; r < repeat_count; ++r) {
            if (width_list.size() < static_cast<size_t>(width)) {
                width_list.push_back(element);
            }
        }
    }
    
    // Completa caso a lista seja menor que a largura da imagem
    while(width_list.size() < static_cast<size_t>(width)) {
        width_list.push_back(0);
    }

    std::vector<std::string> width_b = list_to_graycode_binary(width_list, n_bits);

    for (int j = 0; j < width; ++j) {
        for (int i = 1; i < n_bits; ++i) {
            uchar pixel_val = (width_b[j][i] == '1') ? 255 : 0;
            for (int h = 0; h < height; ++h) {
                gc_images[i].at<uchar>(h, j) = pixel_val;
            }
        }
    }
}

std::vector<int> GrayCode::get_gc_order_v() const {
    std::vector<int> int_values(width, 0);
    
    // Converte os bits de volta para valores inteiros
    for (int j = 0; j < width; ++j) {
        int current_val = 0;
        for (int i = 0; i < n_bits; ++i) {
            int bit = (gc_images[i].at<uchar>(0, j) > 0) ? 1 : 0;
            current_val += bit * (1 << (n_bits - 1 - i));
        }
        int_values[j] = current_val;
    }

    // Obtém valores únicos preservando a ordem original (similar a np.unique com return_index=True)
    std::vector<int> unique_vals;
    std::set<int> seen;
    for(int val : int_values) {
        if(seen.find(val) == seen.end()) {
            seen.insert(val);
            unique_vals.push_back(val);
        }
    }

    return unique_vals;
}