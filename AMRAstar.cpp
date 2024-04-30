#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include "ThreeDmap.h"
#include "HeuristicManger.h"
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



class AMRAstar 
{
private:
    Node start, goal;
    ThreeDMap grid;
    std::vector<std::set<Node>> openLists;  // for each heuristic, an openlist
    std::vector<std::vector<Node>> closeList;
    std::vector<Node> INCONS;
    HeuristicManager manager;
    int weight1;
    int weight2;

    void addToOpenList(int heuristicIndex, const Node& node) {
        openLists[heuristicIndex].insert(node);
    }

    bool isInCloseList(const Node& node) const 
    {
        return std::find_if(closeList.begin(), closeList.end(),
                            [node](const Node& n) { return n.position == node.position && n.res == node.res; }) != closeList.end();
    }

    void addToCloseList(const Node& node, Resolution::Level res) 
    {
        if (res!=Resolution::Invalid){closeList[res].push_back(node);}
    }

    bool isInCloseList(const Node& node, Resolution::Level res) const 
    {
        if (res!=Resolution::Invalid){
        const auto& list = closeList[res];
        return std::find_if(list.begin(), list.end(),
                        [node](const Node& n) { return n.position == node.position && n.res == node.res; }) != list.end();}
    }

    void removeFromOpenList(int heuristicIndex, const Node& node) {
        openLists[heuristicIndex].erase(node);
    }

public:
    AMRAstar(int initWeight1, int initWeight2, std::string path, std::array<int,3> taskStart, std::array<int,3> taskGoal, int resolutionScale): grid(path, resolutionScale),weight1(initWeight1),weight2(initWeight2)
    {
        
        manager.registerHeuristic("Euclidean", std::make_unique<EuclideanDistance>());
        manager.registerHeuristic("Manhattan", std::make_unique<ManhattanDistance>());

        for (size_t i = 0; i < manager.countHeuristics(); ++i) {
            openLists.emplace_back();
        }

        start = {taskStart, Resolution::HIGH, 0, manager.getHeuristic("Euclidean")->calculate(taskStart, taskGoal), nullptr};
        goal = {taskGoal, Resolution::HIGH, manager.getHeuristic("Euclidean")->calculate(taskStart, taskGoal), 0, nullptr};
    }

    double key(Node* node,int i)
    {
        const Heuristic* heuristic = manager.getHeuristicByIndex(i);
        return node->g_cost +  weight1*heuristic->calculate(node->position, goal.position);
    }
    
    void expand(Node* node, Resolution::Level res) 
    {
        // 获取后继节点
        auto successors = grid.getSuccs(node->position, res);

        // 遍历每个后继节点，根据所有启发式更新
        for (auto& successorPosition : successors) {
            Node successor;
            successor.position = successorPosition;
            successor.res = node->res;
            successor.parent = node;

            for (size_t i = 0; i < manager.countHeuristics(); ++i) {
                const Heuristic* heuristic = manager.getHeuristicByIndex(i);
                double g_cost_new = node->g_cost + heuristic->calculate(node->position, successorPosition);
                double h_cost_new = heuristic->calculate(successorPosition, goal.position);

                successor.g_cost = g_cost_new;
                successor.h_cost = h_cost_new;

                auto& openList = openLists[i];
                auto it = openList.find(successor);
                if (it == openList.end() && !isInCloseList(successor)) {
                    addToOpenList(i, successor);
                } else if (it != openList.end() && g_cost_new < it->g_cost) {
                    removeFromOpenList(i, *it);
                    addToOpenList(i, successor);
                }
            }
        }

        addToCloseList(*node,res);
    }



};




int main() {


    return 0;
}

















