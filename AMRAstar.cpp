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
    std::vector<Node> solutionPath;
    //std::vector<Node> Explored;
    HeuristicManager manager;
    int weight1;
    int weight2;
    int weight1Step;
    int weight2step;
    std::vector<std::pair<Resolution::Level, int>> heurs_map;
    double startTime;
    double searchTime;
    double timeLimit; //ms

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
    
    Node* findNodeByPosition(const std::vector<std::set<Node>>& openLists, int heuristicIndex, const std::array<int, 3>& position) 
    {
        auto& openList = openLists[heuristicIndex];
        for (const auto& node : openList) {
            if (node.position == position) {
                return const_cast<Node*>(&node);
            }
        }
        return nullptr; 
    }

    void removeFromOpenListByPosition(int heuristicIndex, const std::array<int, 3>& position) 
    {
        Node* node = findNodeByPosition(openLists, heuristicIndex, position);
        if (node != nullptr) {
            removeFromOpenList(heuristicIndex, *node);
        }   
    }

    void printOPEN( int heuristicIndex)
    {
        for (auto const& opens: openLists[heuristicIndex])
        {
           std::cout << '(' << opens.position[0] << ',' << opens.position[1] << ',' <<opens.position[2]<< ',' << opens.h_cost << ')' <<std::endl;
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

    double getTime()
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        double milliseconds = std::chrono::duration<double, std::milli>(duration).count();
        //std::cout << "Milliseconds since epoch: " << milliseconds << " ms" << std::endl;
        return milliseconds;
    }



public:
    std::vector<Node> Explored;
    AMRAstar(int initWeight1, int initWeight2, std::string path, std::array<int,3> taskStart, std::array<int,3> taskGoal, int resolutionScale, double timeLimit): 
                                                                grid(path, resolutionScale),weight1(initWeight1),weight2(initWeight2),timeLimit(timeLimit)
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
        goal = {taskGoal, Resolution::HIGH, taskStartHvalue , 0, taskStartHvalue  + 0  , nullptr};
        weight1Step = 0.1 * weight1;
        weight2step = 0.1 * weight2;

    }

    double key(const Node& node, int i)
    {
        const Heuristic* heuristic = manager.getHeuristicByIndex(i);
        return node.g_cost +  weight1*heuristic->calculate(node.position, goal.position);
    }
    
   

    void expand(Node* node, int heuristicIndex) 
    {
        Resolution::Level res = heurs_map.at(heuristicIndex).first;
        std::vector<std::pair<std::array<int, 3>, std::string>> successors;

        //std::cout<< res<<std::endl;
        if (node->position==goal.position)
        {
            successors.push_back({node->position, grid.getMapValue(node->position)});
        }
        else
        {
            successors = grid.getSuccs(node->position, res);
        }
        
        //grid.printNodes(successors);
        //grid.printBoundary();

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

    /*
    bool improvePath(const double& startTime)
    {
        std::cout<<"count"<<std::endl;
        bool checkImprove = false;
        std::cout<<"size of Heuristics" << manager.countHeuristics() <<std::endl;
        for (size_t i = 0; i < manager.countHeuristics(); ++i)
        {
            while (!openLists[i].empty()) 
            { 
                std::cout << i << std::endl;
                double elapsedTime = getTime() - startTime; 
                std::cout << "elapsedTime:" << elapsedTime << std::endl;
                if (elapsedTime >= timeLimit)
                {
                    return false;
                }
                double fCheck = weight2 * openLists[0].begin()->f_cost;
                if (openLists[i].begin()->f_cost > fCheck ) 
                {
                    i = 0;
                }
                Node x = *openLists[i].begin();
                std::cout << "x_now: "<< '(' << x.position[0] << ',' << x.position[1] << ',' <<x.position[2]<< ',' << x.h_cost << ')' <<std::endl;
                if (x.position == goal.position){
                    std::cout << "beginToreconstruct"<<std::endl;
                    std::cout << "x_parent: "<< '(' << x.parent->position[0] << ',' << x.parent->position[1] << ',' <<x.parent->position[2]<< ')' <<std::endl;
                    //reconstructPath(x);
                    checkImprove = true;
                    return true;
                }
                openLists[i].erase(openLists[i].begin());
                std::cout << "Before_Opensize: " <<openLists[i].size() << std::endl;
                expand(&x,i);
                std::cout << "After_Opensize: " <<openLists[i].size() << std::endl;
                std::cout << "open" << i << ':'<< std::endl;
                printOPEN(i);
                Resolution::Level res = heurs_map.at(i).first;
                addToCloseList(x, res);

            }
        }
        return checkImprove;
    }
    */

    bool improvePath(const double& startTime)
    {
        while (!openLists[0].empty())
        {
            //printOPEN(0);
            double elapsedTime = getTime() - startTime; 
            if (elapsedTime > timeLimit)
            {
                return false; 
            }
            for (int i = 0; i < manager.countHeuristics(); ++i) 
            {
                if (openLists[0].empty())
                {
                    return false;
                }
                double fCheck = weight2 * openLists[0].begin()->f_cost;
                //if (goal.g_cost <= fCheck)
                //{
                //    std::cout<<"Cost of Goal: "<<goal.g_cost<<','<< "Cost of fcheck2: "<<fCheck<<std::endl;
                //    return true;
                //}

                if (!openLists[i].empty() &&
                        openLists[i].begin()->f_cost <= fCheck)
                {
                    Node x = *openLists[i].begin();
                    openLists[i].erase(openLists[i].begin());
                    if(x.position == goal.position)
                    {
                        reconstructPath(x);
                        return true;
                    }
                    printOPEN(i);
                    expand(&x,i);
                }
                else
                {
                    Node x = *openLists[i].begin();
                    openLists[i].erase(openLists[i].begin());
                    if(x.position == goal.position)
                    {
                        reconstructPath(x);
                        return true;
                    }
                    expand(&x,0);
                }
            }
        }
    }




    void reconstructPath(Node& node) {
        solutionPath.clear();
        for (Node* current = &node; current != nullptr; current = current->parent) {
            solutionPath.push_back(*current);
        }
        std::reverse(solutionPath.begin(), solutionPath.end());
        // Display or process the path
    }

    void search()
    {
        startTime = getTime();
        clearAllOpen();
        addToINCONS(start);
        while((weight1 >= 1) && (weight2 >= 1))
        {   
            std:: cout << "weight1: "<< weight1<< std::endl;
            std:: cout << "weight2: " <<weight2<< std::endl;
            for(const auto& incon: INCONS){updateOpenList(0, incon);}
            INCONS.clear();
            std::set<Resolution::Level> ResolutionInOpenList = getUniqueResolutionLevels(openLists,0);
            for (const auto& node_0: openLists[0])
            {
                for (size_t j = 0; j < manager.countHeuristics(); ++j)
                {
                    Resolution::Level res = heurs_map.at(j).first;
                    if (ResolutionInOpenList.find(res)!= ResolutionInOpenList.end())
                    {
                        updateOpenList(j,node_0);
                    }
                }
            }
            clearAllClose();
            if (improvePath(startTime))
            {
                for (const auto &solution: solutionPath)
                {
                    std::cout << '(' << solution.position[0] << ',' << solution.position[1] << ',' <<solution.position[2]<< ')' <<std::endl;
                }
                
            }
            weight1 = weight1 - weight1Step;
            weight2 = weight2 - weight2step; 

        }

    }

};



int main() 
{



    // Start and goal positions
    std::array<int, 3> start = {1, 1, 0};
    std::array<int, 3> goal = {4, 1, 0};
    //std::cout << start[0];

    // Initialize AMRAstar algorithm
    AMRAstar amraStar(10, 10, "path_to_map_file", start, goal, 1, 1000);
    /*
    
    
    // Node to expand
    Node testNode;
    testNode.position = start;
    testNode.res = Resolution::Level::HIGH;
    testNode.g_cost = 0;
    testNode.h_cost = 0;
    testNode.f_cost = 0; // Assuming f_cost is g_cost + h_cost for simplicity
    testNode.parent = nullptr;

    // Simulate expansion
    amraStar.expand(&testNode, 0);  // Use the first heuristic

    // Output results
    std::cout << "Explored nodes after expansion:" << std::endl;
    for (const auto& node : amraStar.Explored) {
        std::cout << "(" << node.position[0] << ", " << node.position[1] << ", " << node.position[2] << ") g_cost: " << node.g_cost << std::endl;
    }
    */

    amraStar.search();
    
    return 0;
}


















