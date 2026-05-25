// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <string>
#include <vector>

struct Cell {
    int row{0}, col{0};
    bool operator==(const Cell& o) const { return row == o.row && col == o.col; }
};

class OccupancyMap {
public:
    bool load(const std::string& yaml_path);

    // Expand every obstacle outward by radius_m so the robot centre
    // never gets closer than radius_m to a wall.
    void inflate(double radius_m);

    bool is_free(int row, int col) const;
    bool in_bounds(int row, int col) const;

    Cell   world_to_cell(double x, double y) const;
    void   cell_to_world(int row, int col, double& x, double& y) const;

    int    width()      const { return width_; }
    int    height()     const { return height_; }
    double resolution() const { return resolution_; }
    double origin_x()   const { return origin_x_; }
    double origin_y()   const { return origin_y_; }

    // Raw access for publishing as OccupancyGrid
    const std::vector<bool>& free_cells() const { return free_; }

private:
    int    width_{0}, height_{0};
    double resolution_{0.05};
    double origin_x_{0.0}, origin_y_{0.0};
    // Row 0 = south edge of map (matches ROS convention).
    // true = navigable free space.
    std::vector<bool> free_;

    int idx(int row, int col) const { return row * width_ + col; }
};
