#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <iostream>

struct Node {
    int X{0};
    int Y{0};
    int GWeight{0};
    int Parent{-1};
    bool Walkable{true};
    int State{0};
    int Id() const { return Y; }
};

static inline int Index(int x, int y, int sizeX) {
    return y * sizeX + x;
}

static inline int Heuristic(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

std::vector<Node> AStar(std::vector<Node>& Graph, int SizeX, int SizeY, Node Start, Node End) {
    const int N = SizeX * SizeY;
    if (N == 0) return {};

    auto validCoord = [&](int x, int y){ return x >= 0 && x < SizeX && y >= 0 && y < SizeY; };

    int startIdx = Index(Start.X, Start.Y, SizeX);
    int endIdx = Index(End.X, End.Y, SizeX);
    if (!validCoord(Start.X, Start.Y) || !validCoord(End.X, End.Y)) return {};

    const int INF = std::numeric_limits<int>::max() / 4;
    std::vector<int> g(N, INF);
    std::vector<int> f(N, INF);
    std::vector<int> parent(N, -1);
    std::vector<bool> closed(N, false);

    using Pair = std::pair<int,int>;
    struct Cmp { bool operator()(const Pair &a, const Pair &b) const { return a.first > b.first; } };
    std::priority_queue<Pair, std::vector<Pair>, Cmp> open;

    g[startIdx] = 0;
    int hx = Heuristic(Start.X, Start.Y, End.X, End.Y);
    f[startIdx] = hx;
    open.emplace(f[startIdx], startIdx);

    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    while (!open.empty()) {
        auto [curF, cur] = open.top();
        open.pop();
        if (closed[cur]) continue;
        if (cur == endIdx) break;
        closed[cur] = true;

        int cx = cur % SizeX;
        int cy = cur / SizeX;

        for (int k = 0; k < 4; ++k) {
            int nx = cx + dx[k];
            int ny = cy + dy[k];
            if (!validCoord(nx, ny)) continue;
            int ni = Index(nx, ny, SizeX);
            if (!Graph[ni].Walkable) continue;
            if (closed[ni]) continue;

            int tentative = g[cur] + 1;
            if (tentative < g[ni]) {
                parent[ni] = cur;
                g[ni] = tentative;
                int h = Heuristic(nx, ny, End.X, End.Y);
                f[ni] = g[ni] + h;
                open.emplace(f[ni], ni);
            }
        }
    }

    for (int i = 0; i < N; ++i) {
        Graph[i].GWeight = (g[i] == INF) ? -1 : g[i];
        Graph[i].Parent = parent[i];
        if (!Graph[i].Walkable) Graph[i].State = 1;
        else Graph[i].State = 0;
    }
    for (int i = 0; i < N; ++i) if (closed[i] && Graph[i].Walkable) Graph[i].State = 2;

    std::vector<Node> path;
    if (parent[endIdx] == -1 && endIdx != startIdx && g[endIdx] == INF) {
        return {};
    }
    int cur = endIdx;
    while (cur != -1) {
        Node n = Graph[cur];
        n.GWeight = g[cur];
        n.Parent = parent[cur];
        path.push_back(n);
        Graph[cur].State = 3;
        if (cur == startIdx) break;
        cur = parent[cur];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

int main() {
    const int sizeX = 10, sizeY = 10;
    std::vector<Node> graph(sizeX * sizeY);
    for (int y = 0; y < sizeY; ++y) {
        for (int x = 0; x < sizeX; ++x) {
            int idx = Index(x, y, sizeX);
            graph[idx].X = x;
            graph[idx].Y = y;
            graph[idx].Walkable = true;
            graph[idx].State = 0;
        }
    }

    graph[Index(3,3,sizeX)].Walkable = false;
    graph[Index(3,4,sizeX)].Walkable = false;
    graph[Index(3,5,sizeX)].Walkable = false;

    Node start; start.X = 1; start.Y = 1;
    Node end; end.X = 8; end.Y = 8;

    auto path = AStar(graph, sizeX, sizeY, start, end);
    if (path.empty()) {
    } else {
        std::cout << "Chemin (" << path.size() << "):\n";
        for (auto &n : path) {
            std::cout << "(" << n.X << "," << n.Y << ") g=" << n.GWeight << "\n";
        }
    }

    for (int y = 0; y < sizeY; ++y) {
        for (int x = 0; x < sizeX; ++x) {
            int idx = Index(x, y, sizeX);
            std::cout << graph[idx].State;
            if (x < sizeX - 1) std::cout << ' ';
        }
        std::cout << '\n';
    }

    return 0;
}
