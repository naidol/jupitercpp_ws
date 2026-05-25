// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#include "jupiter_nav/occupancy_map.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>

bool OccupancyMap::load(const std::string& yaml_path) {
    std::ifstream file(yaml_path);
    if (!file.is_open()) return false;

    std::string pgm_filename;
    float occupied_thresh = 0.65f;
    float free_thresh     = 0.25f;
    int   negate          = 0;

    std::string line;
    while (std::getline(file, line)) {
        auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string key = line.substr(0, colon);
        std::string val = line.substr(colon + 1);
        // Trim leading whitespace from value
        val.erase(0, val.find_first_not_of(" \t"));
        val.erase(val.find_last_not_of(" \t\r\n") + 1);

        if      (key == "image")            pgm_filename    = val;
        else if (key == "resolution")       resolution_     = std::stod(val);
        else if (key == "occupied_thresh")  occupied_thresh = std::stof(val);
        else if (key == "free_thresh")      free_thresh     = std::stof(val);
        else if (key == "negate")           negate          = std::stoi(val);
        else if (key == "origin") {
            // origin: [-3.013, -7.096, 0]
            auto lb = val.find('['), rb = val.find(']');
            if (lb != std::string::npos && rb != std::string::npos) {
                std::istringstream ss(val.substr(lb + 1, rb - lb - 1));
                char comma;
                ss >> origin_x_ >> comma >> origin_y_;
            }
        }
    }

    // Resolve relative image path against the yaml directory
    if (!pgm_filename.empty() && pgm_filename[0] != '/') {
        auto dir = yaml_path.substr(0, yaml_path.find_last_of('/'));
        pgm_filename = dir + '/' + pgm_filename;
    }

    cv::Mat img = cv::imread(pgm_filename, cv::IMREAD_GRAYSCALE);
    if (img.empty()) return false;

    width_  = img.cols;
    height_ = img.rows;
    free_.assign(width_ * height_, false);

    // PGM row 0 = top of image; map row 0 = south (bottom).
    // Flip vertically so cell (row, col) matches world (x = origin_x + col*res,
    //                                                    y = origin_y + row*res).
    for (int map_row = 0; map_row < height_; ++map_row) {
        int img_row = height_ - 1 - map_row;
        for (int col = 0; col < width_; ++col) {
            uint8_t pixel = img.at<uint8_t>(img_row, col);
            float occupancy = negate ? pixel / 255.0f : (255 - pixel) / 255.0f;
            free_[idx(map_row, col)] = (occupancy < free_thresh);
        }
    }
    return true;
}

void OccupancyMap::inflate(double radius_m) {
    const int radius_cells = static_cast<int>(std::ceil(radius_m / resolution_));
    const int r2 = radius_cells * radius_cells;
    std::vector<bool> inflated = free_;

    for (int row = 0; row < height_; ++row) {
        for (int col = 0; col < width_; ++col) {
            if (free_[idx(row, col)]) continue; // only expand obstacles
            for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
                for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
                    if (dr * dr + dc * dc > r2) continue;
                    int nr = row + dr, nc = col + dc;
                    if (in_bounds(nr, nc)) inflated[idx(nr, nc)] = false;
                }
            }
        }
    }
    free_ = std::move(inflated);
}

bool OccupancyMap::is_free(int row, int col) const {
    return in_bounds(row, col) && free_[idx(row, col)];
}

bool OccupancyMap::in_bounds(int row, int col) const {
    return row >= 0 && row < height_ && col >= 0 && col < width_;
}

Cell OccupancyMap::world_to_cell(double x, double y) const {
    return {
        static_cast<int>((y - origin_y_) / resolution_),
        static_cast<int>((x - origin_x_) / resolution_)
    };
}

void OccupancyMap::cell_to_world(int row, int col, double& x, double& y) const {
    x = origin_x_ + (col + 0.5) * resolution_;
    y = origin_y_ + (row + 0.5) * resolution_;
}
