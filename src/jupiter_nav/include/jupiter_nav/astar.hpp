// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "jupiter_nav/occupancy_map.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <vector>

class AStar {
public:
    struct Result {
        bool              found{false};
        std::vector<Cell> path; // start → goal, in map cells
    };

    Result search(Cell start, Cell goal, const OccupancyMap& map) const {
        if (start == goal)
            return {true, {start}};
        if (!map.is_free(start.row, start.col) || !map.is_free(goal.row, goal.col))
            return {};

        const int width = map.width();
        auto key = [width](const Cell& c) { return c.row * width + c.col; };

        // (f, g, cell) — lazy-deletion open set
        using Entry = std::tuple<float, float, Cell>;
        auto worse = [](const Entry& a, const Entry& b) {
            return std::get<0>(a) > std::get<0>(b);
        };
        std::priority_queue<Entry, std::vector<Entry>, decltype(worse)> open(worse);

        std::unordered_map<int, float> g;
        std::unordered_map<int, int>   parent; // key → parent key

        const int start_key = key(start);
        const int goal_key  = key(goal);
        g[start_key] = 0.0f;
        open.push({octile(start, goal), 0.0f, start});

        static constexpr int   DR[]   = {-1,-1,-1, 0, 0, 1, 1, 1};
        static constexpr int   DC[]   = {-1, 0, 1,-1, 1,-1, 0, 1};
        static constexpr float COST[] = {1.414f,1.f,1.414f,1.f,1.f,1.414f,1.f,1.414f};

        while (!open.empty()) {
            auto [f, g_snap, current] = open.top();
            open.pop();

            const int ck = key(current);
            // Skip stale entries from lazy deletion
            if (g.count(ck) && g_snap > g[ck] + 1e-5f) continue;
            if (ck == goal_key) return {true, backtrack(parent, start_key, goal_key, width)};

            for (int i = 0; i < 8; ++i) {
                Cell nb{current.row + DR[i], current.col + DC[i]};
                if (!map.is_free(nb.row, nb.col)) continue;

                const int   nk = key(nb);
                const float ng = g[ck] + COST[i];
                if (!g.count(nk) || ng < g[nk]) {
                    g[nk]      = ng;
                    parent[nk] = ck;
                    open.push({ng + octile(nb, goal), ng, nb});
                }
            }
        }
        return {};
    }

    // String-pulling: remove waypoints whose neighbours have line-of-sight.
    // Reduces a staircase grid path to straight-line segments.
    static std::vector<Cell> smooth(const std::vector<Cell>& path, const OccupancyMap& map) {
        if (path.size() <= 2) return path;
        std::vector<Cell> out;
        out.push_back(path[0]);
        size_t anchor = 0;
        for (size_t i = 2; i < path.size(); ++i) {
            if (!los(path[anchor], path[i], map)) {
                out.push_back(path[i - 1]);
                anchor = i - 1;
            }
        }
        out.push_back(path.back());
        return out;
    }

private:
    static float octile(const Cell& a, const Cell& b) {
        float dx = std::abs(float(a.col - b.col));
        float dy = std::abs(float(a.row - b.row));
        return std::max(dx, dy) + 0.414f * std::min(dx, dy);
    }

    // Bresenham line-of-sight check on the free map
    static bool los(const Cell& a, const Cell& b, const OccupancyMap& map) {
        int x = a.col, y = a.row;
        int dx = std::abs(b.col - x), dy = std::abs(b.row - y);
        int sx = (b.col > x) ? 1 : -1, sy = (b.row > y) ? 1 : -1;
        int err = dx - dy;
        while (true) {
            if (!map.is_free(y, x)) return false;
            if (x == b.col && y == b.row) return true;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }
    }

    static std::vector<Cell> backtrack(
        const std::unordered_map<int, int>& parent,
        int start_key, int goal_key, int width)
    {
        std::vector<Cell> path;
        int cur = goal_key;
        while (cur != start_key) {
            path.push_back({cur / width, cur % width});
            auto it = parent.find(cur);
            if (it == parent.end()) return {};
            cur = it->second;
        }
        path.push_back({start_key / width, start_key % width});
        std::reverse(path.begin(), path.end());
        return path;
    }
};
