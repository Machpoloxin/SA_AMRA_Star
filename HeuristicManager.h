#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <iostream>
#include <vector>




class RotationQuaternion {
private:
    double w, x, y, z;

public:
    // Constructor
    RotationQuaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
    RotationQuaternion(std::array<double,4> rotQuat) : w(rotQuat[0]), x(rotQuat[1]),y(rotQuat[2]),z(rotQuat[3]){}

    // Normalize the quaternion
    void normalize() {
        double norm = std::sqrt(w*w + x*x + y*y + z*z);
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

    // Quaternion multiplication
    RotationQuaternion operator*(const RotationQuaternion& q) const {
        return RotationQuaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }

    // Inverse of the quaternion
    RotationQuaternion inverse() const {
        return RotationQuaternion(w, -x, -y, -z);
    }

    // Distance between two quaternions
    static double distance(const RotationQuaternion& q1, const RotationQuaternion& q2) {
        double dot_product = std::fabs(q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z);
        return 2 * std::acos(dot_product);
    }

    bool operator==(const RotationQuaternion& q) const {
        return (std::fabs(w - q.w) < 1e-9 && std::fabs(x - q.x) < 1e-9 && std::fabs(y - q.y) < 1e-9 && std::fabs(z - q.z) < 1e-9) ||
               (std::fabs(w + q.w) < 1e-9 && std::fabs(x + q.x) < 1e-9 && std::fabs(y + q.y) < 1e-9 && std::fabs(z + q.z) < 1e-9);
    }

    // Print the quaternion
    void print() const {
        std::cout << "Quaternion: (" << w << ", " << x << ", " << y << ", " << z << ")" << std::endl;
    }

    // Getters
    double getW() const { return w; }
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }
};




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
