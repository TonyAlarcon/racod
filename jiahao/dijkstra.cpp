#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <chrono>
#include <string>
#include <iomanip>
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

// Map loading
vector<string> loadMap(const string& path) {
    ifstream in(path);
    string line;
    getline(in, line);  // type
    int H, W;
    in >> line >> H;
    in >> line >> W;
    getline(in, line);  // flush
    getline(in, line);  // "map"

    vector<string> grid(H);
    for (int i = 0; i < H; ++i)
        getline(in, grid[i]);
    return grid;
}

// Collision checking
bool checkCollision(const vector<string>& grid, int x0, int y0, int r) {
    int H = grid.size(), W = grid[0].size();
    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            int x = x0 + dx, y = y0 + dy;
            if (x < 0 || x >= W || y < 0 || y >= H)
                return false;
            if (grid[y][x] == '@')
                return false;
        }
    }
    return true;
}

// Neighbor generation
vector<pair<int, int>> getNeighbors(int x, int y, const vector<string>& grid) {
    static const vector<pair<int,int>> deltas = {
        {0,1},{1,0},{0,-1},{-1,0}, {1,1},{-1,-1},{1,-1},{-1,1}
    };
    vector<pair<int, int>> nbrs;
    int H = grid.size(), W = grid[0].size();

    for (auto [dx, dy] : deltas) {
        int nx = x + dx, ny = y + dy;
        if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
        if (abs(dx)==1 && abs(dy)==1 &&
            (grid[y][x+dx] == '@' || grid[y+dy][x] == '@')) continue;
        nbrs.emplace_back(nx, ny);
    }
    return nbrs;
}

// Dijkstra algorithm
vector<pair<int,int>> dijkstra(
    const vector<string>& grid,
    pair<int,int> start,
    pair<int,int> goal,
    int radius)
{
    typedef pair<double, pair<int,int>> QItem;
    priority_queue<QItem, vector<QItem>, greater<>> open;
    unordered_map<int64_t, pair<int,int>> came_from;
    unordered_map<int64_t, double> cost_so_far;

    auto key = [](int x, int y){ return (int64_t(y) << 32) | uint32_t(x); };

    open.emplace(0.0, start);
    cost_so_far[key(start.first, start.second)] = 0.0;

    while (!open.empty()) {
        auto [g, cur] = open.top(); open.pop();
        int x = cur.first, y = cur.second;

        if (cur == goal) {
            vector<pair<int,int>> path;
            while (cur != start) {
                path.push_back(cur);
                cur = came_from[key(cur.first, cur.second)];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }

        for (auto [nx, ny] : getNeighbors(x, y, grid)) {
            if (!checkCollision(grid, nx, ny, radius)) continue;
            double new_cost = g + hypot(nx - x, ny - y);
            int64_t k = key(nx, ny);
            if (!cost_so_far.count(k) || new_cost < cost_so_far[k]) {
                cost_so_far[k] = new_cost;
                came_from[k] = {x, y};
                open.emplace(new_cost, make_pair(nx, ny));
            }
        }
    }
    return {};  // no path
}

// CSV processing
void run(const string& map_path, const string& samples_csv, const string& out_csv) {
    auto grid = loadMap(map_path);
    ifstream inf(samples_csv);
    ofstream outf(out_csv);
    string line;

    getline(inf, line);
    outf << line << ",algo,time_ms,path\n";

    while (getline(inf, line)) {
        istringstream ss(line);
        string tok;
        vector<string> tokens;
        while (getline(ss, tok, ','))
            tokens.push_back(tok);

        int sx = stoi(tokens[0]);
        int sy = stoi(tokens[1]);
        int gx = stoi(tokens[2]);
        int gy = stoi(tokens[3]);
        int radius = stoi(tokens[4]);

        auto t0 = chrono::high_resolution_clock::now();
        auto path = dijkstra(grid, {sx, sy}, {gx, gy}, radius);
        auto t1 = chrono::high_resolution_clock::now();
        double ms = chrono::duration<double, milli>(t1 - t0).count();

        outf << line << ",dijkstra," << fixed << setprecision(3) << ms << ",\"";
        if (!path.empty()) {
            outf << "[";
            for (size_t i = 0; i < path.size(); ++i) {
                outf << "[" << path[i].first << "," << path[i].second << "]";
                if (i + 1 < path.size()) outf << ",";
            }
            outf << "]";
        } else {
            outf << "[]";
        }
        outf << "\"\n";
    }

    cout << "Results written to: " << out_csv << "\n";
}

int main() {
    string map_path = "./dataset/boston/Boston_0_1024.map";
    string samples_csv = "./dataset/boston/Boston_0_1024_samples.csv";
    string out_csv = "./dataset/boston/Boston_0_1024_results_dijkstra.csv";
    run(map_path, samples_csv, out_csv);
    return 0;
}
