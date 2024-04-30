#include <vector>
#include <set>
#include <map>
#include <queue>
#include <cmath>
#include <array>
#include <algorithm>
#include "ThreeDmap.h"   // 假设包含 getSuccs 方法等
#include "HeuristicManager.h"  // 包含多个启发式的管理
#include <limits>
#include <queue>

enum class Resolution { LOW, MEDIUM, HIGH, Invalid };

struct Node {
    std::array<int,3> position; // x, y, z coordinates
    Resolution res;
    double g_cost;
    double h_cost;
    double f_cost() const { return g_cost + h_cost; }
    Node* parent;
    bool operator<(const Node& other) const {
        return f_cost() > other.f_cost(); // Priority queue needs max at top
    }
    bool operator==(const Node& other) const {
        return position == other.position && res == other.res;
    }
};

class CompareNode {
public:
    bool operator()(const Node& n1, const Node& n2) const {
        return n1 < n2;
    }
};

class AMRAstar {
private:
    Node start, goal;
    ThreeDMap grid;
    std::vector<std::set<Node, CompareNode>> openLists;  // One open list per heuristic
    std::vector<std::vector<Node>> closeList;  // One close list per resolution
    std::vector<Node> INCONS;
    HeuristicManager manager;
    int weight1;
    int weight2;

    void addToOpenList(int heuristicIndex, const Node& node) {
        openLists[heuristicIndex].insert(node);
    }

    bool isInCloseList(const Node& node, Resolution res) const {
        const auto& list = closeList[static_cast<int>(res)];
        return std::find(list.begin(), list.end(), node) != list.end();
    }

    void addToCloseList(const Node& node) {
        closeList[static_cast<int>(node.res)].push_back(node);
    }

    void removeFromOpenList(int heuristicIndex, const Node& node) {
        openLists[heuristicIndex].erase(node);
    }

public:
    AMRAstar(int initWeight1, int initWeight2, std::string path, std::array<int,3> taskStart, std::array<int,3> taskGoal, int resolutionScale)
    : grid(path, resolutionScale), weight1(initWeight1), weight2(initWeight2) {
        manager.registerHeuristic("Euclidean", std::make_unique<EuclideanDistance>());
        manager.registerHeuristic("Manhattan", std::make_unique<ManhattanDistance>());
        openLists.resize(manager.countHeuristics());
        closeList.resize(3); // Assuming there are three resolutions: LOW, MEDIUM, HIGH

        start = {taskStart, Resolution::HIGH, 0, manager.getHeuristic("Euclidean")->calculate(taskStart, taskGoal), nullptr};
        goal = {taskGoal, Resolution::HIGH, manager.getHeuristic("Euclidean")->calculate(taskStart, taskGoal), 0, nullptr};
    }

    void search() {
        addToOpenList(0, start);  // Start with one heuristic for simplicity

        while (!openLists[0].empty()) {  // Simplified to use one open list
            Node current = *openLists[0].begin();
            removeFromOpenList(0, current);

            if (current == goal) {
                reconstructPath(current);
                return;
            }

            expand(current);
        }
    }

    void expand(const Node& node) {
        auto successors = grid.getSuccs(node.position, node.res);
        for (const auto& successorPosition : successors) {
            Node successor = {successorPosition, node.res, node.g_cost + 1, 0, &node}; // Assuming cost = 1 for simplicity

            successor.h_cost = manager.getHeuristic("Euclidean")->calculate(successorPosition, goal.position);
            successor.g_cost = node.g_cost + 1; // Simplified cost function

            if (!isInCloseList(successor, successor.res)) {
                addToOpenList(0, successor); // Simplified to use one open list
            }
        }

        addToCloseList(node);
    }



    bool improvePath() {
    while (!openLists[0].empty()) {
        Node current = *openLists[0].begin(); // 获取拥有最小 f_cost 的节点
        removeFromOpenList(0, current);

        // 如果当前节点是目标节点
        if (current.position == goal.position) {
            reconstructPath(current);
            return true; // 找到路径，返回 true
        }

        // 将当前节点添加到对应解析度的 closed list
        addToCloseList(current, current.res);

        // 扩展当前节点
        auto successors = grid.getSuccs(current.position, current.res);
        for (const auto& successorPosition : successors) {
            Node successor = {
                successorPosition,
                current.res,
                current.g_cost + 1,  // 假设所有转移成本为1
                manager.getHeuristic("Euclidean")->calculate(successorPosition, goal.position),
                &current  // 设置前驱节点为当前节点
            };

            if (!isInCloseList(successor, successor.res)) {
                double new_f_cost = successor.g_cost + weight1 * successor.h_cost;
                successor.f_cost = new_f_cost; // 更新 f_cost

                auto it = std::find_if(openLists[0].begin(), openLists[0].end(), 
                    [&](const Node& n) { return n.position == successor.position && n.res == successor.res; });
                
                // 如果后继节点不在 open list 或者发现了一条更好的路径
                if (it == openLists[0].end() || new_f_cost < it->f_cost()) {
                    if (it != openLists[0].end()) {
                        removeFromOpenList(0, *it);  // 从 open list 中移除旧的节点
                    }
                    addToOpenList(0, successor);  // 将后继节点添加到 open list
                }
            }
        }
    }
    return false; // 没有找到路径
}

void reconstructPath(const Node& node) {
    std::vector<Node> path;
    const Node* current = &node;
    while (current != nullptr) {
        path.push_back(*current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());

    // 这里可以添加代码来处理或显示路径
}

    void reconstructPath(const Node& node) {
        std::vector<Node> path;
        const Node* current = &node;
        while (current != nullptr) {
            path.push_back(*current);
            current = current->parent;
        }
        std::reverse(path.begin(), path.end());

        // Output path or process it further
    }
};
