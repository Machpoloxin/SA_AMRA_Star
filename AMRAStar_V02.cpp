#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include "ThreeDmap.h"
#include "HeuristicManager.h"
#include <array>
#include <set>
#include <chrono>






struct Node {
    std::array<int, 3> position; // x, y, z
    RotationQuaternion orientation;
    Resolution::Level res;
    double g_cost, h_cost, f_cost;
    std::shared_ptr<Node> parent; 

    Node(std::array<int, 3> pos, RotationQuaternion ori, Resolution::Level resolution, double gCost, double hCost, double fCost, std::shared_ptr<Node> parentNode)
        : position(pos), orientation(ori), res(resolution), g_cost(gCost), h_cost(hCost), f_cost(fCost), parent(parentNode) {}


    bool operator<(const Node& other) const {
        return f_cost < other.f_cost || (f_cost == other.f_cost && position < other.position);
    }
    bool operator==(const Node& other) const {
        return (position == other.position)&&(orientation == other.orientation);
    }
    void operator=(const std::shared_ptr<Node> other){
        position=other->position; orientation == other->orientation; res = other->res; g_cost = other->g_cost; h_cost = other->h_cost; f_cost = other->f_cost;
    }
};

struct NodeCompare {
    bool operator()(const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) const {
        if (lhs->f_cost < rhs->f_cost) return true;
        if (rhs->f_cost < lhs->f_cost) return false;
        return lhs->position < rhs->position;
    }
};



class AMRAstar 
{
private:
    std::shared_ptr<Node> start, goal;
    ThreeDMap grid;
    std::vector<std::set<std::shared_ptr<Node>,NodeCompare>> openLists;
    std::vector<std::vector<std::shared_ptr<Node>>> closeList;
    std::vector<std::shared_ptr<Node>> INCONS;
    std::vector<Node> solutionPath;
    std::vector<std::shared_ptr<Node>> Explored;
    HeuristicManager manager;
    int weight1;
    int weight2;
    int weight1Step;
    int weight2step;
    std::vector<std::pair<Resolution::Level, int>> heurs_map;
    double startTime;
    double searchTime;
    double timeLimit; //ms


    void addNodeToExplored(const std::shared_ptr<Node>& node) {
        auto it = std::find_if(Explored.begin(), Explored.end(), 
                           [&node](const std::shared_ptr<Node>& n) { return *n == *node; });
        if (it == Explored.end()) {
            Explored.push_back(node);
     }
    }

    void addToOpenList(int heuristicIndex, std::shared_ptr<Node> node) {
        openLists[heuristicIndex].insert(node);
    }

    void removeFromOpenList(int heuristicIndex, std::shared_ptr<Node> node) {
        std::shared_ptr<Node> existingNode = findNodeByPosition(openLists, heuristicIndex, node->position);
        if (existingNode) {
             openLists[heuristicIndex].erase(existingNode);
        }
    }


    std::shared_ptr<Node> findNodeByPosition(const std::vector<std::set<std::shared_ptr<Node>,NodeCompare>>& openLists, int heuristicIndex, const std::array<int, 3>& position) {
        auto& openList = openLists[heuristicIndex];
        for (const auto& node : openList) {
            if (node->position == position) {
                return node;
            }
        }
        return nullptr; 
    }

    void updateOpenList(int heuristicIndex, std::shared_ptr<Node> node) {
        std::shared_ptr<Node> existingNode = findNodeByPosition(openLists, heuristicIndex, node->position);
        if (existingNode) {
            if (existingNode->f_cost > node->f_cost) {
                removeFromOpenList(heuristicIndex, existingNode);
                addToOpenList(heuristicIndex, node);
            }
        } else {
            addToOpenList(heuristicIndex, node);
        }
    }


    void clearAllOpen() 
    {
        for (auto& openList : openLists) 
        {
            openList.clear();
        }
    }

    std::set<Resolution::Level> getUniqueResolutionLevels(const std::vector<std::set<Node>>& openLists, size_t index) //also usable for closeLists 
    {
        std::set<Resolution::Level> uniqueLevels;
        if (index < openLists.size()) {
            for (const auto& node : openLists[index]) {
                uniqueLevels.insert(node.res);
            }
    }
    return uniqueLevels;
    }   

    void addToCloseList(const std::shared_ptr<Node>& node, Resolution::Level res) {
        if (res != Resolution::Invalid) {
            int resLevel = static_cast<int>(res);
            if (resLevel < closeList.size())
            {
                closeList[resLevel].push_back(node);
            }
        }
    }

    bool isInCloseList(const std::shared_ptr<Node>& node, Resolution::Level res) const {
        if (res != Resolution::Invalid) {
            int resLevel = static_cast<int>(res);
            const auto& list = closeList[resLevel];
            return std::any_of(list.begin(), list.end(),
                            [node](const std::shared_ptr<Node>& n) { return *n == *node; });
        }
        return false;
    }

    std::shared_ptr<Node> whereInCloseList(const std::shared_ptr<Node>& node, Resolution::Level res) const {
        if (res != Resolution::Invalid) {
            int resLevel = static_cast<int>(res);
            const auto& list = closeList[resLevel];
            auto it = std::find_if(list.begin(), list.end(),
                            [node](const std::shared_ptr<Node>& n) { return *n == *node; });
            if (it != list.end())
            {
                return *it;
            }
        }
        return nullptr;
    }

    void clearAllClose() 
    {
        for (auto& list : closeList) 
        {
            list.clear();
        }
    }

    void addToINCONS(const std::shared_ptr<Node>& node) {
        auto it = std::find(INCONS.begin(), INCONS.end(), node);
        if (it == INCONS.end()) {
            INCONS.push_back(node);  
        }
    }

    void clearINCONS() {
        INCONS.clear(); 
    }

    void reintegrateINCONSIntoOpenList() 
    {
        for (const auto& node : INCONS) {
            updateOpenList(0, node);  
        }
        clearINCONS();  
    }

    double getTime()
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        double milliseconds = std::chrono::duration<double, std::milli>(duration).count();
        //std::cout << "Milliseconds since epoch: " << milliseconds << " ms" << std::endl;
        return milliseconds;
    }



public:
    //std::vector<Node> Explored;
    AMRAstar(int initWeight1, int initWeight2, const std::string& path, const std::array<int, 3>& taskStart, const std::array<int, 3>& taskGoal, int resolutionScale, double timeLimit)
        : grid(path, resolutionScale), weight1(initWeight1), weight2(initWeight2), timeLimit(timeLimit) {
        
        manager.registerHeuristic("Euclidean", std::make_unique<EuclideanDistance>());
        heurs_map.emplace_back(Resolution::HIGH, manager.countHeuristics() - 1);

        manager.registerHeuristic("Manhattan", std::make_unique<ManhattanDistance>());
        heurs_map.emplace_back(Resolution::MID, manager.countHeuristics() - 1);

        openLists.resize(manager.countHeuristics());
        closeList.resize(4); // anchor,high,mid,low

        //double taskGoalGvalue = decodeGvalue(grid.getMapValue(taskGoal));
        double taskStartHvalue = manager.getHeuristic("Euclidean")->calculate(taskStart, taskGoal);
        start = std::make_shared<Node>(Node {taskStart, Resolution::HIGH, 0, taskStartHvalue, weight1*taskStartHvalue, nullptr});
        goal = std::make_shared<Node>(Node {taskGoal, Resolution::HIGH, taskStartHvalue, 0, taskStartHvalue, nullptr});

        weight1Step = 0.1 * weight1;
        weight2step = 0.1 * weight2;

        INCONS.push_back(start);
    }

    double key(const Node& node, int i)
    {
        const Heuristic* heuristic = manager.getHeuristicByIndex(i);
        return node.g_cost +  weight1*heuristic->calculate(node.position, goal->position);
    }
    


    void printOpen(int heuristicIndex) {
        if (heuristicIndex < openLists.size()) {
            std::cout << "Open List [" << heuristicIndex << "] Nodes:" << std::endl;
            for (const auto& node : openLists[heuristicIndex]) {
                std::cout << '(' << node->position[0] << ',' << node->position[1] << ',' << node->position[2] << ',' << node->f_cost << ')';
                if (node->parent) {  // check empty?
                    std::cout << " Parent: (" << node->parent->position[0] << ',' << node->parent->position[1] << ',' << node->parent->position[2] << ')';
                } else {
                    std::cout << " Parent: (null)";
                }
                std::cout << std::endl;
            }
        } else {
            std::cout << "Invalid heuristic index." << std::endl;
        }
    }

    void printClose(Resolution::Level res) {
        int resLevel = static_cast<int>(res);
        if (resLevel < closeList.size()) {
            std::cout << "Close List [" << resLevel << "] Nodes:" << std::endl;
            for (const auto& node : closeList[resLevel]) {
                std::cout << '(' << node->position[0] << ',' << node->position[1] << ',' << node->position[2] << ',' << node->f_cost << ')';
                if (node->parent) {  // check empty
                    std::cout << " Parent: (" << node->parent->position[0] << ',' << node->parent->position[1] << ',' << node->parent->position[2] << ')';
                } else {
                    std::cout << " Parent: (null)";
                }
                std::cout << std::endl;
            }
        } else {
            std::cout << "Invalid resolution level index." << std::endl;
        }
    }

    void expand(std::shared_ptr<Node> node, int heuristicIndex) {
        Resolution::Level res = heurs_map.at(heuristicIndex).first;
        std::vector<std::pair<std::array<int, 3>, std::string>> successors;

        if (node->position == goal->position)
            return;

        successors = grid.getSuccs(node->position, res);

        for (const auto& successorPosition : successors) {
            std::shared_ptr<Node> successor = std::make_shared<Node>(successorPosition.first, node->res, 0.0,0.0,0.0,nullptr);
            successor->parent = node;
            const Heuristic* heuristic = manager.getHeuristicByIndex(0);
            double g_cost_new = node->g_cost + heuristic->calculate(node->position, successorPosition.first);
            successor->g_cost = g_cost_new;
            double h_cost_new = heuristic->calculate(successorPosition.first, goal->position);
            successor->h_cost = h_cost_new;
            successor->f_cost = successor->g_cost + weight1 * successor->h_cost;

            if (res == Resolution::ANCHOR)
            {
                auto it = whereInCloseList(successor,res);
                if (it!= nullptr)
                {
                    if (it->g_cost <= successor->g_cost)
                    {
                        successor->g_cost = it->g_cost;
                        successor->parent = it->parent;
                        addToINCONS(successor);
                    }
                    addToINCONS(successor);
                }
            }
            else
            {
                for (size_t jj = 1; jj < manager.countHeuristics(); ++jj)
                {
                    Resolution::Level jres = heurs_map.at(jj).first;
                    if (jj!=heuristicIndex && jres==res)
                    {
                        removeFromOpenList(jj,successor);
                    }
                }
                updateOpenList(0,successor);
            }
            for (size_t j = 1; j < manager.countHeuristics(); ++j) 
            {
                Resolution::Level lres = heurs_map.at(j).first;
                if (lres != successor->res)
                {
                    continue;
                }
                if (!whereInCloseList(successor,lres))
                {
                    double newKey = key(*successor, j);
                    if(newKey <= weight2*successor->f_cost)
                    {
                        successor->f_cost = newKey;
                        //updateOpenList(j,successor);
                        addToOpenList(j,successor);
                    }
                }
            }
        }
    }

    
    bool improvePath(const double& startTime) 
    {
        std::cout << "count" << std::endl;
        bool checkImprove = false;
        std::cout << "size of Heuristics" << manager.countHeuristics() << std::endl;

        for (size_t i = 0; i < manager.countHeuristics(); ++i) {
            while (!openLists[i].empty()) {
                std::cout << i << std::endl;
                double elapsedTime = getTime() - startTime;
                std::cout << "elapsedTime:" << elapsedTime << std::endl;
                if (elapsedTime >= timeLimit) {
                    return false;
                }
                double fCheck = weight2 * (*openLists[i].begin())->f_cost;
                if ((*openLists[i].begin())->f_cost > fCheck) {
                    i = 0;
                }
                std::shared_ptr<Node> x = *openLists[i].begin();
                openLists[i].erase(openLists[i].begin());

                std::cout << "x_now: " << '(' << x->position[0] << ',' << x->position[1] << ',' << x->position[2] << ',' << x->h_cost << ')' << std::endl;
                if (x->position == goal->position) {
                    std::cout << "beginToreconstruct" << std::endl;
                    reconstructPath(x);
                    checkImprove = true;
                    return true;
                }

                expand(x, i);  // Expand using smart pointers
                Resolution::Level res = heurs_map.at(i).first;
                addToCloseList(x, res);
                std::cout << "After expansion openList size: " << openLists[i].size() << std::endl;
                std::cout << "open" << i << ':' << std::endl;
                printOpen(i);
            }
        }
        return checkImprove;
    }
    



    void reconstructPath(const std::shared_ptr<Node>& node) {
        solutionPath.clear();
        for (std::shared_ptr<Node> current = node; current != nullptr; current = current->parent) {
            solutionPath.push_back(*current);
        }
        std::reverse(solutionPath.begin(), solutionPath.end());

        // Display the path
        for (const auto& step : solutionPath) {
            std::cout << '(' << step.position[0] << ',' << step.position[1] << ',' << step.position[2] << ')' << std::endl;
        }
    }
/*
   void search() 
   {
    startTime = getTime();
    clearAllOpen();
    addToOpenList(0, start); 

    while (weight1 >= 1 && weight2 >= 1) {
        std::cout << "weight1: " << weight1 << ", weight2: " << weight2 << std::endl;

        while (!openLists[0].empty()) {
            bool checkaa = improvePath(startTime);
            std::cout << "improvement result: " << checkaa << std::endl;
            if (checkaa) {
                std::cout << "Path found!" << std::endl;
                break;
            }
        }

        weight1 -= weight1Step;
        weight2 -= weight2step;
        clearAllOpen();
        clearAllClose();
        reintegrateINCONSIntoOpenList();
    }
    }
*/


    void search()
    {
        startTime = getTime();
        clearAllOpen();
        addToOpenList(0, start);

        while (weight1 >= 1 && weight2 >= 1) {
            std::cout << "weight1: " << weight1 << ", weight2: " << weight2 << std::endl;

            // Update nodes in INCONS
            for (const auto& node : INCONS) {
                updateOpenList(0, node);  
            }
            clearINCONS();

            // Update all open lists based on resolution
            for (size_t i = 0; i < openLists[0].size(); ++i) {
                auto& node = *std::next(openLists[0].begin(), i);
                for (size_t j = 1; j < manager.countHeuristics(); ++j) {
                    Resolution::Level res = heurs_map.at(j).first;
                    if (res == node->res) {
                        //updateOpenList(j, node);
                        addToOpenList(j,node);
                    }
                }
            }

            clearAllClose();

            if (improvePath(startTime)) {
                //reconstructPath(goal);
                std::cout << "Path found!" << std::endl;
                return;
            }

            weight1 -= weight1Step;
            weight2 -= weight2step;
    }
    }
};


int main() 
{
    std::array<int, 3> start = {1, 1, 0};
    std::array<double, 4> start_ori = {0.707, 0.707, 0, 0};
    std::array<int, 3> goal = {4, 1, 0};
    std::array<double, 4> goal_ori = {0.707, 0, 0.707, 0};
    //std::cout << start[0];

    AMRAstar amraStar(10, 10, "path_to_map_file", start, goal, 1, 100);
    amraStar.search();


    return 0; 
}