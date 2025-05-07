#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include <tuple>
#include <functional>
#include <future>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <cmath>
#include <chrono>
#include <cstdint>   // at the top

using namespace std;

enum class Algo { SERIAL, CREATEJOIN, POOL, CJ_RAS, POOL_RAS };



// ----------------------------------------------------
// Simple fixed-size thread pool
// ----------------------------------------------------
class ThreadPool {
    public:
        explicit ThreadPool(size_t n) : stop(false) {
            for(size_t i = 0; i < n; ++i) {
                workers.emplace_back([this] {
                    while(true) {
                        function<void()> task;
                        {
                            unique_lock<mutex> lock(queue_mutex);
                            condition.wait(lock, [this]{ return stop || !tasks.empty(); });
                            if(stop && tasks.empty()) return;
                            task = move(tasks.front()); tasks.pop();
                        }
                        task();
                    }
                });
            }
        }
    
        template<class F, class... Args>
        auto submit(F&& f, Args&&... args) -> future<decltype(f(args...))> {
            using R = decltype(f(args...));
            auto task_ptr = make_shared<packaged_task<R()>>(bind(forward<F>(f), forward<Args>(args)...));
            future<R> res = task_ptr->get_future();
            {
                lock_guard<mutex> lock(queue_mutex);
                tasks.emplace([task_ptr](){ (*task_ptr)(); });
            }
            condition.notify_one();
            return res;
        }
    
        ~ThreadPool() {
            {
                lock_guard<mutex> lock(queue_mutex);
                stop = true;
            }
            condition.notify_all();
            for(thread &w : workers) w.join();
        }
    
    private:
        vector<thread> workers;
        queue<function<void()>> tasks;
        mutex queue_mutex;
        condition_variable condition;
        bool stop;
    };

// ----------------------------------------------------
// Load map helper
// ----------------------------------------------------
vector<string> loadMap(const string& mapFile) {
    ifstream in(mapFile);
    if (!in) {
        cerr << "ERROR: cannot open map file '" << mapFile << "'\n";
        exit(1);
    }
    string keyword;
    int H = 0, W = 0;
    // Read header lines
    in >> keyword;             // type
    in >> keyword;             // octile
    in >> keyword >> H;        // height <H>
    in >> keyword >> W;        // width  <W>
    in >> keyword;             // map

    if (H <= 0 || W <= 0) {
        cerr << "ERROR: invalid map dimensions " << H << "x" << W << "\n";
        exit(1);
    }

    vector<string> grid(H);
    for (int i = 0; i < H; ++i) {
        in >> grid[i];
        if ((int)grid[i].size() != W) {
            cerr << "ERROR: row " << i << " length " << grid[i].size()
                 << " != expected width " << W << "\n";
            exit(1);
        }
    }
    return grid;
}

// ----------------------------------------------------
// Collision detection: circular footprint radius r
// '.' = free, '@' = obstacle
// ----------------------------------------------------
bool checkCollisionOBB(const vector<string>& grid, int x0, int y0, int r) {
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

// ----------------------------------------------------
// Get 8-connectivity neighbors with corner-cut guard
// ----------------------------------------------------
vector<pair<int,int>> getNeighbors(int x, int y, const vector<string>& grid) {
    static const vector<pair<int,int>> deltas = {
        {0,1},{1,0},{0,-1},{-1,0},  // orthogonal
        {1,1},{-1,-1},{1,-1},{-1,1}  // diagonal
    };
    int H = grid.size(), W = grid[0].size();
    vector<pair<int,int>> nbrs;
    for (auto [dx,dy] : deltas) {
        int nx = x + dx, ny = y + dy;
        if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
        // corner-cut guard
        if (abs(dx)==1 && abs(dy)==1) {
            if (grid[y][x+dx]=='@' || grid[y+dy][x]=='@')
                continue;
        }
        nbrs.emplace_back(nx, ny);
    }
    return nbrs;
}

// ----------------------------------------------------
// A* node
// ----------------------------------------------------
struct Node {
    int x,y;
    double g,h,f;
    Node* parent;
    Node(int _x,int _y,double _g,double _h,Node* p)
      : x(_x), y(_y), g(_g), h(_h), f(_g+_h), parent(p) {}
};

struct NodeCmp {
    bool operator()(Node* a, Node* b) const {
        return a->f > b->f;  // min-heap on f
    }
};



// ----------------------------------------------------
// Serial A* baseline serialized version
// ----------------------------------------------------
vector<pair<int,int>> astarSerial(const vector<string>& grid,
                                  pair<int,int> start,
                                  pair<int,int> goal,
                                  int radius) {
    int sx = start.first, sy = start.second;
    int gx = goal.first,  gy = goal.second;
    auto hfunc = [&](int x,int y){ return hypot(x-gx, y-gy); };
    int H = grid.size(), W = grid[0].size();
    vector<vector<bool>> closed(H, vector<bool>(W,false));
    priority_queue<Node*, vector<Node*>, NodeCmp> open;
    open.push(new Node(sx, sy, 0, hfunc(sx, sy), nullptr));

    while (!open.empty()) {
        Node* cur = open.top(); 
        open.pop();
        int x = cur->x, y = cur->y;
        if (closed[y][x]) { delete cur; continue; }
        closed[y][x] = true;
        if (x == gx && y == gy) {
            vector<pair<int,int>> path;
            for (Node* p = cur; p; p = p->parent)
                path.emplace_back(p->x, p->y);
            reverse(path.begin(), path.end());
            return path;
        }
        for (auto [nx,ny] : getNeighbors(x,y,grid)) {
            if (closed[ny][nx]) continue;
            if (!checkCollisionOBB(grid, nx, ny, radius)) continue;
            double g2 = cur->g + hypot(nx-x, ny-y);
            open.push(new Node(nx, ny, g2, hfunc(nx, ny), cur));
        }
    }
    return {}; // no path
}


//----------------------------------------------------------------
//  astarPersistentThreadPool  –  corrected, leak‑free
//----------------------------------------------------------------
vector<pair<int,int>>
astarPersistentThreadPool(const vector<string>& grid,
                          pair<int,int> start,
                          pair<int,int> goal,
                          int radius,
                          size_t numThreads)
{
    auto h = [&](int x,int y){ return hypot(x-goal.first,y-goal.second); };

    const int H = grid.size(), W = grid[0].size();
    vector<vector<bool>> closed(H, vector<bool>(W,false));

    priority_queue<Node*, vector<Node*>, NodeCmp> open;
    vector<Node*>                              arena;   // owns *all* nodes

    auto *root = new Node(start.first,start.second,0.0,h(start.first,start.second),nullptr);
    open.push(root);  arena.push_back(root);

    ThreadPool pool(numThreads);

    // cache to avoid duplicate collision checks
    unordered_map<uint64_t,bool> cache;
    mutex cache_m;

    auto key = [](int x,int y){ return (uint64_t(y)<<32)|uint32_t(x); };

    while(!open.empty()) {
        Node* cur = open.top();  open.pop();
        if (closed[cur->y][cur->x]) continue;
        closed[cur->y][cur->x] = true;

        if (cur->x==goal.first && cur->y==goal.second) {      // reconstruct
            vector<pair<int,int>> path;
            for(Node* p=cur; p; p=p->parent) path.emplace_back(p->x,p->y);
            reverse(path.begin(),path.end());
            for(Node* n:arena) delete n;                      // tidy‑up
            return path;
        }

        /* --- dispatch neighbour checks -------------------------------- */
        vector<pair<int,int>> nbrs = getNeighbors(cur->x,cur->y,grid);
        vector< future<pair<pair<int,int>,bool>> > futs; futs.reserve(nbrs.size());

        for (auto [nx,ny] : nbrs) {
            if (closed[ny][nx]) continue;
            futs.emplace_back(
                pool.submit([&,nx,ny]{
                    uint64_t k = key(nx,ny);
                    {   lock_guard<mutex> g(cache_m);
                        auto it = cache.find(k);
                        if (it!=cache.end()) return make_pair(make_pair(nx,ny), it->second);
                    }
                    bool free = checkCollisionOBB(grid,nx,ny,radius);
                    {   lock_guard<mutex> g(cache_m);
                        cache[k] = free;
                    }
                    return make_pair(make_pair(nx,ny), free);
                })
            );
        }

        /* --- harvest -------------------------------------------------- */
        for (auto &f : futs) {
            auto [pos, ok] = f.get();
            if (!ok) continue;

            int nx = pos.first, ny = pos.second;
            double g2 = cur->g + hypot(nx-cur->x, ny-cur->y);
            auto *n  = new Node(nx,ny,g2,h(nx,ny),cur);
            open.push(n);
            arena.push_back(n);
        }
    }
    for(Node* n:arena) delete n;
    return {};
}

// == RASExp ADDITIONS ====================================================

// Collision status enum for speculative memoization
enum class CollisionStatus : uint8_t { UNKNOWN, PENDING, FREE, BLOCKED };

// 8-direction deltas for neighbor lookup and run-ahead
static const vector<pair<int,int>> RAS_DELTAS = {
    {0,1},{1,0},{0,-1},{-1,0},  // orthogonal moves
    {1,1},{-1,-1},{1,-1},{-1,1}  // diagonal moves
};

// Determine which direction index led to current Node from its parent
static int getDirIndex(const Node* cur) {
    if (!cur->parent) return 0;
    int dx = cur->x - cur->parent->x;
    int dy = cur->y - cur->parent->y;
    for (int i = 0; i < (int)RAS_DELTAS.size(); ++i) {
        if (RAS_DELTAS[i].first == dx && RAS_DELTAS[i].second == dy)
            return i;
    }
    return 0;
}



// ----------------------------------------------------------------------
// astarPoolRAS: uses a persistent ThreadPool and applies RASExp
// ----------------------------------------------------------------------
vector<pair<int,int>> astarPoolRAS(
    const vector<string>& grid,
    pair<int,int> start,
    pair<int,int> goal,
    int radius,
    size_t numThreads,
    int maxDepth
) {
    auto h = [&](int x,int y){ return hypot(x-goal.first, y-goal.second); };
    int H = grid.size(), W = grid[0].size();

    vector<vector<bool>> closed(H, vector<bool>(W, false));
    vector<vector<CollisionStatus>> status(H, vector<CollisionStatus>(W, CollisionStatus::UNKNOWN));

    priority_queue<Node*, vector<Node*>, NodeCmp> open;
    vector<Node*> arena;
    Node* root = new Node(start.first, start.second, 0.0, h(start.first, start.second), nullptr);
    open.push(root); arena.push_back(root);

    ThreadPool pool(numThreads);
    unordered_map<uint64_t,bool> cache;
    mutex cache_m;
    auto key = [&](int x,int y){ return (uint64_t(y)<<32) | uint32_t(x); };

    while (!open.empty()) {
        Node* cur = open.top(); open.pop();
        if (closed[cur->y][cur->x]) continue;
        closed[cur->y][cur->x] = true;
        if (cur->x == goal.first && cur->y == goal.second) {
            vector<pair<int,int>> path;
            for (Node* p = cur; p; p = p->parent) path.emplace_back(p->x, p->y);
            reverse(path.begin(), path.end());
            for (Node* n : arena) delete n;
            return path;
        }

        // Demand collision checks
        vector<future<pair<pair<int,int>,bool>>> futs;
        for (auto [nx,ny] : getNeighbors(cur->x, cur->y, grid)) {
            if (closed[ny][nx] || status[ny][nx] != CollisionStatus::UNKNOWN) continue;
            status[ny][nx] = CollisionStatus::PENDING;
            futs.emplace_back(pool.submit([&, nx, ny](){
                uint64_t k = key(nx, ny);
                { lock_guard<mutex> lk(cache_m);
                    if (auto it = cache.find(k); it != cache.end())
                        return make_pair(make_pair(nx, ny), it->second);
                }
                bool free = checkCollisionOBB(grid, nx, ny, radius);
                { lock_guard<mutex> lk(cache_m); cache[k] = free; }
                return make_pair(make_pair(nx, ny), free);
            }));
        }

        // Speculative run-ahead
        if (!futs.empty()) {
            int depth = maxDepth;
            int dir   = getDirIndex(cur);
            // start from the real current position
            int px    = cur->x;
            int py    = cur->y;
            // precompute the step delta
            int dx    = RAS_DELTAS[dir].first;
            int dy    = RAS_DELTAS[dir].second;

            while (futs.size() < numThreads && depth-- > 0) {
                // step forward
                px += dx;
                py += dy;
                if (px < 0 || px >= W || py < 0 || py >= H) break;

                // speculative neighbors of (px,py)
                for (auto [nx,ny] : getNeighbors(px, py, grid)) {
                    if (closed[ny][nx] ||
                        status[ny][nx] != CollisionStatus::UNKNOWN)
                        continue;
                    status[ny][nx] = CollisionStatus::PENDING;
                    futs.emplace_back(pool.submit([&, nx, ny]() {
                        uint64_t k = key(nx, ny);
                        { lock_guard<mutex> lk(cache_m);
                        if (auto it = cache.find(k); it != cache.end())
                            return make_pair(make_pair(nx, ny), it->second);
                        }
                        bool free = checkCollisionOBB(grid, nx, ny, radius);
                        { lock_guard<mutex> lk(cache_m); cache[k] = free; }
                        return make_pair(make_pair(nx, ny), free);
                    }));
                    if (futs.size() >= numThreads) break;
                }
            }
        }


        // Collect
        for (auto &f : futs) {
            auto [pos, ok] = f.get();
            auto [nx,ny] = pos;
            status[ny][nx] = ok ? CollisionStatus::FREE : CollisionStatus::BLOCKED;
        }

        // Enqueue
        for (auto [nx,ny] : getNeighbors(cur->x, cur->y, grid)) {
            if (closed[ny][nx] || status[ny][nx] != CollisionStatus::FREE) continue;
            double g2 = cur->g + hypot(nx - cur->x, ny - cur->y);
            Node* n = new Node(nx, ny, g2, h(nx, ny), cur);
            open.push(n);
            arena.push_back(n);
        }
    }
    for (Node* n : arena) delete n;
    return {};
}


// ------------------------------------------------------------------------
// Baseline multithreaded A* (create/join per neighbor)
// ------------------------------------------------------------------------
vector<pair<int,int>> astarCreateJoin(
    const vector<string>& grid,
    pair<int,int> start,
    pair<int,int> goal,
    int radius,
    size_t numThreads)
{
    int sx = start.first, sy = start.second;
    int gx = goal.first, gy = goal.second;
    auto hfunc = [&](int x, int y) { return hypot(x - gx, y - gy); };
    int H = grid.size(), W = grid[0].size();

    vector<vector<bool>> closed(H, vector<bool>(W, false));
    priority_queue<Node*, vector<Node*>, NodeCmp> open;
    vector<Node*> arena;

    // initialize root
    Node* root = new Node(sx, sy, 0.0, hfunc(sx, sy), nullptr);
    open.push(root);
    arena.push_back(root);

    while (!open.empty()) {
        Node* cur = open.top(); open.pop();
        int x = cur->x, y = cur->y;
        // skip duplicates (already expanded)
        if (closed[y][x]) continue;
        closed[y][x] = true;

        // goal check
        if (x == gx && y == gy) {
            vector<pair<int,int>> path;
            for (Node* p = cur; p; p = p->parent)
                path.emplace_back(p->x, p->y);
            reverse(path.begin(), path.end());
            for (Node* n : arena) delete n;
            return path;
        }

        auto nbrs = getNeighbors(x, y, grid);
        vector<bool> freeCheck(nbrs.size(), false);
        vector<thread> threads;
        vector<size_t> indices;
        threads.reserve(nbrs.size());

        // launch collision checks
        for (size_t i = 0; i < nbrs.size(); ++i) {
            int nx = nbrs[i].first, ny = nbrs[i].second;
            if (closed[ny][nx]) continue;
            indices.push_back(i);
            threads.emplace_back([&, i, nx, ny] {
                freeCheck[i] = checkCollisionOBB(grid, nx, ny, radius);
            });
        }
        // join threads
        for (auto& t : threads) t.join();

        // enqueue collision-free neighbors
        for (size_t idx : indices) {
            if (!freeCheck[idx]) continue;
            int nx = nbrs[idx].first, ny = nbrs[idx].second;
            double g2 = cur->g + hypot(nx - x, ny - y);
            Node* n = new Node(nx, ny, g2, hfunc(nx, ny), cur);
            open.push(n);
            arena.push_back(n);
        }
    }

    // no path found
    for (Node* n : arena) delete n;
    return {};
}

// ------------------------------------------------------------------------
// Multithreaded A* with RASExp extension (create/join)
// ------------------------------------------------------------------------
vector<pair<int,int>> astarCreateJoinRAS(
    const vector<string>& grid,
    pair<int,int> start,
    pair<int,int> goal,
    int radius,
    size_t numThreads,
    int maxDepth)
{
    int H = grid.size(), W = grid[0].size();
    int sx = start.first, sy = start.second;
    int gx = goal.first, gy = goal.second;
    auto h = [&](int x, int y) { return hypot(x - gx, y - gy); };

    vector<vector<bool>> closed(H, vector<bool>(W, false));
    vector<vector<CollisionStatus>> status(H, vector<CollisionStatus>(W, CollisionStatus::UNKNOWN));
    priority_queue<Node*, vector<Node*>, NodeCmp> open;
    vector<Node*> arena;

    // initialize root
    Node* root = new Node(sx, sy, 0.0, h(sx, sy), nullptr);
    open.push(root);
    arena.push_back(root);

    while (!open.empty()) {
        Node* cur = open.top(); open.pop();
        int x = cur->x, y = cur->y;
        if (closed[y][x]) continue;
        closed[y][x] = true;

        if (x == gx && y == gy) {
            vector<pair<int,int>> path;
            for (Node* p = cur; p; p = p->parent)
                path.emplace_back(p->x, p->y);
            reverse(path.begin(), path.end());
            for (Node* n : arena) delete n;
            return path;
        }

        // demand collision checks
        auto nbrs = getNeighbors(x, y, grid);
        vector<pair<int,int>> demand;
        for (auto& nb : nbrs) {
            int nx = nb.first, ny = nb.second;
            if (closed[ny][nx] || status[ny][nx] != CollisionStatus::UNKNOWN) continue;
            status[ny][nx] = CollisionStatus::PENDING;
            demand.emplace_back(nx, ny);
        }

        vector<thread> threads;
        threads.reserve(numThreads);

        // launch demand threads
        for (auto& p : demand) {
            int nx = p.first, ny = p.second;
            threads.emplace_back([&, nx, ny] {
                bool ok = checkCollisionOBB(grid, nx, ny, radius);
                status[ny][nx] = ok ? CollisionStatus::FREE : CollisionStatus::BLOCKED;
            });
        }


        // speculative run‐ahead 
        if (!threads.empty()) {
            int depth = maxDepth;
            int dir   = getDirIndex(cur);
            // start from the real current position
            int px    = cur->x;
            int py    = cur->y;
            // precompute the step delta for this direction
            int dx    = RAS_DELTAS[dir].first;
            int dy    = RAS_DELTAS[dir].second;

            while (threads.size() < numThreads && depth-- > 0) {
                // step one cell forward in the predicted direction
                px += dx;
                py += dy;
                if (px < 0 || px >= W || py < 0 || py >= H) break;

                // gather neighbors of this speculative cell
                auto specNbrs = getNeighbors(px, py, grid);
                for (auto& nb : specNbrs) {
                    int nx2 = nb.first, ny2 = nb.second;
                    // skip if already closed or already checked
                    if (closed[ny2][nx2] ||
                        status[ny2][nx2] != CollisionStatus::UNKNOWN)
                        continue;
                    // mark pending and launch collision‐check thread
                    status[ny2][nx2] = CollisionStatus::PENDING;
                    threads.emplace_back([&, nx2, ny2] {
                        bool ok2 = checkCollisionOBB(grid, nx2, ny2, radius);
                        status[ny2][nx2] = ok2
                            ? CollisionStatus::FREE
                            : CollisionStatus::BLOCKED;
                    });
                    if (threads.size() >= numThreads)
                        break;
                }
            }
        }


        // join all threads
        for (auto& t : threads) t.join();

        // enqueue free neighbors
        for (auto& nb : getNeighbors(x, y, grid)) {
            int nx = nb.first, ny = nb.second;
            if (closed[ny][nx] || status[ny][nx] != CollisionStatus::FREE) continue;
            double g2 = cur->g + hypot(nx - x, ny - y);
            Node* n = new Node(nx, ny, g2, h(nx, ny), cur);
            open.push(n);
            arena.push_back(n);
        }
    }

    // cleanup
    for (Node* n : arena) delete n;
    return {};
}



/* ───── main ───── */
int main(int argc, char* argv[])
{
    if (argc != 2) {
        cerr << "Usage: " << argv[0] << " <config-file>\n"; return 1;
    }

    string mapFile;   pair<int,int> start, goal;
    int radius = 1, maxDepth = 8;
    size_t threads = thread::hardware_concurrency();
    Algo algo = Algo::POOL;

    auto trim=[](string s){
        size_t a=s.find_first_not_of(" \t"), b=s.find_last_not_of(" \t");
        return (a==string::npos)? "" : s.substr(a,b-a+1);
    };

    ifstream fin(argv[1]);  if(!fin){ cerr<<"Cannot open cfg\n"; return 1; }
    string line;
    while(getline(fin,line)){
        line = line.substr(0,line.find('#'));              // strip comments
        auto eq=line.find('='); if(eq==string::npos) continue;
        string key = trim(line.substr(0,eq));
        string val = trim(line.substr(eq+1));

        if      (key=="map")        mapFile = val;
        else if (key=="start")      sscanf(val.c_str(),"%d,%d",&start.first,&start.second);
        else if (key=="goal")       sscanf(val.c_str(),"%d,%d",&goal.first ,&goal.second );
        else if (key=="radius")     radius   = stoi(val);
        else if (key=="threads")    threads  = stoul(val);
        else if (key=="max_depth")  maxDepth = stoi(val);
        else if (key=="algorithm") {
            if      (val=="serial")        algo=Algo::SERIAL;
            else if (val=="createjoin")    algo=Algo::CREATEJOIN;
            else if (val=="pool")          algo=Algo::POOL;
            else if (val=="createjoin_ras")algo=Algo::CJ_RAS;
            else if (val=="pool_ras")      algo=Algo::POOL_RAS;
        }
    }

    auto grid = loadMap(mapFile);

    cout << "Algorithm: ";
    auto t0 = chrono::high_resolution_clock::now();
    vector<pair<int,int>> path;

    switch(algo){
        case Algo::SERIAL:
            path = astarSerial(grid,start,goal,radius); 
            cout << "serial\n";
            break;
        case Algo::CREATEJOIN:
            path = astarCreateJoin(grid,start,goal,radius,threads); 
            cout << "createjoin\n";
            break;
        case Algo::POOL:
            path = astarPersistentThreadPool(grid,start,goal,radius,threads); 
            cout << "pool\n";
            break;
        case Algo::CJ_RAS:
            path = astarCreateJoinRAS(grid,start,goal,radius,threads,maxDepth); 
            cout << "createjoin_ras\n";
            break;
        case Algo::POOL_RAS:
            path = astarPoolRAS(grid,start,goal,radius,threads,maxDepth); 
            cout << "pool_ras\n";
            break;
    }
    auto t1 = chrono::high_resolution_clock::now();

    double ms = chrono::duration<double,milli>(t1-t0).count();
    

    if(path.empty()) cout << "No path  time=" << ms << " ms\n";
    else {
        cout << "Path len=" << path.size() << "  time=" << ms << " ms\n";  
    }
    return 0;
}


