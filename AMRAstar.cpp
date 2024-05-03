#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include "ThreeDmap.h"
#include "HeuristicManager.h"
#include <array>
#include <set>


/*

struct Node 
{
    std::array<int,3> position; //x,y,z
    Resolution::Level res;
    double g_cost;
    double h_cost;
    double f_cost() const { return g_cost + h_cost; } // function!
    Node* parent;
    bool operator<(const Node& other) const { //begin is the smallest 
        return f_cost() < other.f_cost() || (f_cost() == other.f_cost() && position < other.position);
    }
};
*/

struct Node 
{
    std::array<int,3> position; //x,y,z
    Resolution::Level res;
    double g_cost;
    double h_cost;
    double f_cost; // function!
    Node* parent;
    bool operator<(const Node& other) const { //begin is the smallest 
        return f_cost < other.f_cost || (f_cost == other.f_cost && position < other.position);
    }
    bool operator==(const Node& other) const {
        return position == other.position;}
};

//using Grid = std::vector<std::vector<Node*>>;




class AMRAstar 
{
private:
    Node start, goal;
    ThreeDMap grid;
    std::vector<std::set<Node>> openLists;  // for each heuristic, an openlist
    std::vector<std::vector<Node>> closeList;
    std::vector<Node> INCONS;
    //std::vector<Node> Explored;
    HeuristicManager manager;
    int weight1;
    int weight2;
    std::vector<std::pair<Resolution::Level, int>> heurs_map;

    int findNodePosition(const Node& node) {
        auto it = std::find(Explored.begin(), Explored.end(), node);
        if (it != Explored.end()) {
        // Calculate the index
            return std::distance(Explored.begin(), it);
        }
        return -1; // Return -1 if not found
    }

    void addNodeToExplored(const Node& node) {
        if (findNodePosition(node)==-1) {
            Explored.push_back(node);
        }
    }

    void addToOpenList(int heuristicIndex, const Node& node) {
        openLists[heuristicIndex].insert(node);
    }

    void removeFromOpenList(int heuristicIndex, const Node& node) 
    {
        openLists[heuristicIndex].erase(node);
    }

    void updateOpenList(int heuristicIndex, const Node& node)
    {
        auto& openList = openLists[heuristicIndex];
        auto it = openList.find(node);
        if (it == openList.end()) 
        {
            addToOpenList(heuristicIndex, node);
        }   
        else
        {
            removeFromOpenList(heuristicIndex, *it);
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
        else{return false;}
    }

    void clearAllClose() 
    {
        for (auto& list : closeList) 
        {
            list.clear();
        }
    }

    void addToINCONS(const Node& node) 
    {
        INCONS.push_back(node);
    }

    void clearINCONS() 
    {
        INCONS.clear();
    }



public:
    std::vector<Node> Explored;
    AMRAstar(int initWeight1, int initWeight2, std::string path, std::array<int,3> taskStart, std::array<int,3> taskGoal, int resolutionScale): grid(path, resolutionScale),weight1(initWeight1),weight2(initWeight2)
    {

        //heurs_map.emplace_back(Resolution::ANCHOR, 0);//CAN CHANGE, 0: Heuristic Index!!!
        manager.registerHeuristic("Euclidean", std::make_unique<EuclideanDistance>());
        heurs_map.emplace_back(Resolution::HIGH, manager.countHeuristics()-1);//CAN CHANGE
        
        manager.registerHeuristic("Manhattan", std::make_unique<ManhattanDistance>());
        heurs_map.emplace_back(Resolution::MID, manager.countHeuristics()-1);

        for (size_t i = 0; i < manager.countHeuristics(); ++i) {
            openLists.emplace_back();
        }
        closeList.resize(4);
        double taskGoalGvalue = decodeGvalue(grid.getMapValue(taskGoal));
        double taskStartHvalue = manager.getHeuristic("Euclidean")->calculate(taskStart, taskGoal);

        start = {taskStart, Resolution::HIGH, 0, taskStartHvalue,0 + weight1 * taskStartHvalue, nullptr};
        goal = {taskGoal, Resolution::HIGH, taskGoalGvalue, 0, taskGoalGvalue + 0  , nullptr};
    }

    double key(const Node& node, int i)
    {
        const Heuristic* heuristic = manager.getHeuristicByIndex(i);
        return node.g_cost +  weight1*heuristic->calculate(node.position, goal.position);
    }
    
   

    void expand(Node* node, int heuristicIndex) 
    {
        Resolution::Level res = heurs_map.at(heuristicIndex).first;
        std::cout<< res<<std::endl;

        auto successors = grid.getSuccs(node->position, res);
        grid.printNodes(successors);
        grid.printBoundary();

        for (auto& successorPosition : successors) 
        {
            Node successor;
            successor.position = successorPosition.first;
            successor.res = node->res;
            int iter =findNodePosition(successor);
            if (iter!=-1)
            {
                successor.g_cost = Explored.at(iter).g_cost;
            }
            else
            {
                successor.g_cost = std::numeric_limits<double>::max();
            }
            const Heuristic* heuristic = manager.getHeuristicByIndex(0);
            double g_cost_new = node->g_cost + heuristic->calculate(node->position, successorPosition.first);
            if (successor.g_cost > g_cost_new)
            {  
                successor.g_cost = g_cost_new; 
                successor.parent = node;
                double h_cost_new = heuristic->calculate(successorPosition.first, goal.position);
                successor.h_cost = h_cost_new;
                successor.f_cost = key(successor, 0);// 0 anchor heuristic
                addNodeToExplored(successor);
               
                if (isInCloseList(successor, Resolution::ANCHOR))
                {
                    addToINCONS(successor);
                }
                else
                {
                    updateOpenList(0,successor);
                    for (size_t j = 0; j < manager.countHeuristics(); ++j) 
                    {
                        Resolution::Level lres = heurs_map.at(j).first;
                        
                        if (lres != successor.res)
                        {
                            continue;
                        }
                        if (!isInCloseList(successor,lres))
                        {
                            double newKey = key(successor, j);
                            if (newKey <= weight2*successor.f_cost)
                            {
                                successor.f_cost = newKey;
                                updateOpenList(j, successor);
                            }
                        }

                    }
                }

            }

           
        }



    }




};



int main() 
{



    // Start and goal positions
    std::array<int, 3> start = {1, 1, 0};
    std::array<int, 3> goal = {4, 1, 0};

    // Initialize AMRAstar algorithm
    AMRAstar amraStar(1, 1, "path_to_map_file", start, goal, 1);

    // Node to expand
    Node testNode;
    testNode.position = start;
    testNode.res = Resolution::Level::HIGH;
    testNode.g_cost = 0;
    testNode.h_cost = 0;
    testNode.f_cost = 0; // Assuming f_cost is g_cost + h_cost for simplicity
    testNode.parent = nullptr;
    //std::cout<<"point1"<<std::endl;
    // Simulate expansion
    amraStar.expand(&testNode, 0);  // Use the first heuristic

    // Output results
    std::cout << "Explored nodes after expansion:" << std::endl;
    for (const auto& node : amraStar.Explored) {
        std::cout << "(" << node.position[0] << ", " << node.position[1] << ", " << node.position[2] << ") g_cost: " << node.g_cost << std::endl;
    }

    return 0;
}


















