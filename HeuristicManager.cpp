#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <iostream>
#include <vector>

class Heuristic {
public:
    virtual ~Heuristic() {}
    virtual double calculate(const std::array<int, 3>& a, const std::array<int, 3>& b) const = 0;
};


class EuclideanDistance : public Heuristic {
public:
    double calculate(const std::array<int, 3>& a, const std::array<int, 3>& b) const override {
        int sum = 0;
        for (int i = 0; i < 3; ++i) {
            int diff = a[i] - b[i];
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }
};


class ManhattanDistance : public Heuristic {
public:
    double calculate(const std::array<int, 3>& a, const std::array<int, 3>& b) const override {
        int sum = 0;
        for (int i = 0; i < 3; ++i) {
            int diff = std::abs(a[i] - b[i]);
            sum += diff;
        }
        return sum;
    }
};


class HeuristicManager {
private:
    std::map<std::string, std::unique_ptr<Heuristic>> heuristics;
    std::vector<std::string> heuristicIndex;

public:
    
    void registerHeuristic(const std::string& name, std::unique_ptr<Heuristic> heuristic) {
        heuristics[name] = std::move(heuristic);
        heuristicIndex.push_back(name);
    }

    
    const Heuristic* getHeuristic(const std::string& name) const {
        auto it = heuristics.find(name);
        if (it != heuristics.end()) {
            return it->second.get();
        }
        return nullptr;
    }

    const Heuristic* getHeuristicByIndex(int i) const {
        std::string name = heuristicIndex.at(i);
        auto it = heuristics.find(name);
        if (it != heuristics.end()) {
            return it->second.get();
        }
        return nullptr;
    }

    
    size_t countHeuristics() const {
        return heuristics.size();
    }

    void printAllHeuristic()
    {
        //for (auto const & heurist: heuristicIndex)
        //{
        //    std::cout << heurist << std::endl;
        //}
        for (int i = 0; i < countHeuristics();++i)
        {
            std::cout << heuristicIndex[i] << std::endl;
        }
    }
};

int main() {
    HeuristicManager manager;
    manager.registerHeuristic("Euclidean", std::make_unique<EuclideanDistance>());
    manager.registerHeuristic("Manhattan", std::make_unique<ManhattanDistance>());
    manager.printAllHeuristic();
    std::array<int, 3> point1 = {1, 2, 3};
    std::array<int, 3> point2 = {4, 5, 6};

    
    auto euclidean = manager.getHeuristic("Euclidean");
    if (euclidean) {
        std::cout << "Euclidean Distance: " << euclidean->calculate(point1, point2) << std::endl;
    }

   
    auto manhattan = manager.getHeuristic("Manhattan");
    if (manhattan) {
        std::cout << "Manhattan Distance: " << manhattan->calculate(point1, point2) << std::endl;
    }

    
    std::cout << "Total heuristics registered: " << manager.countHeuristics() << std::endl;

    return 0;
}