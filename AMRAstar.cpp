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
    bool operator>(const Node& other) const {
        return f_cost() > other.f_cost();
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
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    std::vector<Node> closeList;
    std::set<std::array<int, 3>> openSetPositions;
    
    void addToOpenList(const Node& node) 
    {
        openList.push(node);
    }

    bool isInCloseList(const Node* node) const 
    {
        return std::find_if(closeList.begin(), closeList.end(),
                            [node](const Node& n) { return n.position == node->position && n.res == node->res; }) != closeList.end();
    }

    bool isInOpenList(const std::array<int, 3>& position) const {
        return openSetPositions.find(position) != openSetPositions.end();
    }

    void addToCloseList(const Node& node) 
    {
        closeList.push_back(node);
    }

    void removeFromOpenList(const std::array<int, 3>& position) {
        openSetPositions.erase(position);
        // 重新构建优先队列，除了要删除的元素
        auto tempQueue = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>();
        while (!openList.empty()) {
            auto node = openList.top();
            openList.pop();
            if (node.position != position) {
                tempQueue.push(node);
            }
        }
        openList = tempQueue;
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
        
        for (auto const& successorPosition : successors) {
            Node successor;
            successor.position = successorPosition;
            successor.res = node->res;
            successor.parent = node;
            successor.g_cost = node->g_cost + EuclideanDistance(node->position, successor.position);
            successor.h_cost = EuclideanDistance(successor.position, goal.position);

            if (!isInCloseList(&successor) && !isInOpenList(successor.position)) {
                // 如果后继节点不在关闭列表和开放列表中，添加到开放列表
                addToOpenList(successor);
                openSetPositions.insert(successor.position);
            } else if (isInOpenList(successor.position) && successor.g_cost < node->g_cost) {
                // 如果在开放列表中找到了更好的路径，更新该节点
                removeFromOpenList(successor.position); // 从优先队列中移除节点
                addToOpenList(successor); // 用新成本添加回去
                openSetPositions.insert(successor.position);
            }
        }
        
        // 将当前节点移动到关闭列表
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




int main() 
{
    std::string path = "sparsemap.map";
    std::array<int, 3> start_pos = {0, 0, 0}; // 一个示例起点位置
    std::array<int, 3> goal_pos = {10, 10, 10}; // 一个示例目标位置
    int resolutionScale = 1;

    // 创建 AMRAstar 实例
    AMRAstar amrastar(path, start_pos, goal_pos, resolutionScale);

    // 展开起始节点
    Node start_node = {start_pos, Resolution::HIGH, 0, EuclideanDistance(start_pos, goal_pos), nullptr};
    amrastar.expand(&start_node);

    // 输出 openList 的内容以验证 expand 函数的结果
     
    Node current = amrastar.getNextOpenListNode();
    std::cout << "Node Position: (" << current.position[0] << ", " << current.position[1] << ", " << current.position[2] << ")\n";
    std::cout << "G Cost: " << current.g_cost << ", H Cost: " << current.h_cost << ", F Cost: " << current.f_cost() << std::endl;
    

    return 0;
}

















