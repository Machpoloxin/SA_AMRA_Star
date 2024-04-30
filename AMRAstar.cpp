#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include "ThreeDmap.h"
#include <array>
#include <set>



struct Node 
{
    std::array<int,3> position; //x,y,z
    Resolution::Level res;
    double g_cost;
    double h_cost;
    double f_cost() const { return g_cost + h_cost; } // function!
    Node* parent;
    bool operator<(const Node& other) const {
        return f_cost() < other.f_cost() || (f_cost() == other.f_cost() && position < other.position);
    }
};

using Grid = std::vector<std::vector<Node*>>;


double EuclideanDistance(const std::array<int, 3>& a, const std::array<int, 3>& b) {
    int sum = 0;
    for (int i = 0; i < 3; ++i) {
        int diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

int ManhattanDistance(const std::array<int, 3>& a, const std::array<int, 3>& b) 
{
    int sum = 0;
    for (int i = 0; i < 3; ++i) {
        int diff = std::abs(a[i] - b[i]);
        sum  += diff;
    }
    return sum; 
}



class AMRAstar
{
private:
    Node start, goal;
    ThreeDMap Grid;
    std::set<Node> openList;
    std::vector<Node> closeList;
    
    
    void addToOpenList(const Node& node) 
    {
        openList.insert(node);
    }

    bool isInCloseList(const Node& node) const 
    {
        return std::find_if(closeList.begin(), closeList.end(),
                            [node](const Node& n) { return n.position == node.position && n.res == node.res; }) != closeList.end();
    }

    void addToCloseList(const Node& node) 
    {
        closeList.push_back(node);
    }

    void removeFromOpenList(const Node& node) {
         openList.erase(node);
    }



public:
    AMRAstar(std::string path, std::array<int,3> taskStart, std::array<int,3> taskGoal, int resolutionScale):Grid(path,resolutionScale)
    {
        start = {
            taskStart,
            Resolution::HIGH,
            0,
            EuclideanDistance(taskStart,taskGoal),
            nullptr
        };
        goal = {
            taskGoal,
            Resolution::HIGH,
            EuclideanDistance(taskStart,taskGoal),
            0,
            nullptr
        };

    };
    ~AMRAstar(){};

   void expand(Node* node) {
        // 获取后继节点
        auto successors = Grid.getSuccs(node->position, node->res);
        
       for (auto& successorPosition : successors) {
        Node successor = {successorPosition, node->res, node->g_cost + EuclideanDistance(node->position, successorPosition), EuclideanDistance(successorPosition, goal.position), node};
        if (!isInCloseList(successor) && openList.find(successor) == openList.end()) {
            addToOpenList(successor);
        } else {
            auto it = openList.find(successor);
            if (it != openList.end() && successor.g_cost < it->g_cost) {
                openList.erase(it);
                addToOpenList(successor);
            }
        }
    }
    
    addToCloseList(*node);
    }

    Node getNextOpenListNode()
    {
        if (!openList.empty()) {
            Node nextNode = openList.top();
            return nextNode;
        } else {
            throw std::runtime_error("Open list is empty!");
        }
    }

    // 一个新的公共函数，用来判断 openList 是否为空
    bool isOpenListEmpty() const {
        return openList.empty();
    }



};




int main() {
    std::array<int, 3> start_pos = {0, 0, 0};  // 起始点坐标
    std::array<int, 3> goal_pos = {10, 10, 10};  // 目标点坐标
    std::string mapPath = "sparsemap.map";  // 地图文件路径
    int resolutionScale = 1;  // 分辨率缩放因子

    // 创建 AMRAstar 实例
    AMRAstar amrAStar(mapPath, start_pos, goal_pos, resolutionScale);

    // 设置起始节点
    Node startNode = {
        start_pos,
        Resolution::HIGH,
        0,  // g_cost
        EuclideanDistance(start_pos, goal_pos),  // h_cost, 根据您的距离计算函数
        nullptr  // 没有父节点
    };

    // 将起始节点加入 openList
    amrAStar.addToOpenList(startNode);

    // 展开起始节点
    amrAStar.expand(&startNode);

    // 输出展开的节点
    while (!amrAStar.isOpenListEmpty()) {
        Node nextNode = amrAStar.getNextOpenListNode();
        amrAStar.removeFromOpenList(nextNode.position);
        std::cout << "Expanded Node Position: (" << nextNode.position[0] << ", " << nextNode.position[1] << ", " << nextNode.position[2] << ")\n";
        std::cout << "G Cost: " << nextNode.g_cost << ", H Cost: " << nextNode.h_cost << ", F Cost: " << nextNode.f_cost() << std::endl;
    }

    return 0;
}

















