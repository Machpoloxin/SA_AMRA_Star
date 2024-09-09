#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <random>




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
    RotationQuaternion operator*(double scalar) const {
        return RotationQuaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    RotationQuaternion operator+(const RotationQuaternion& q) const {
        return RotationQuaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    RotationQuaternion operator-(const RotationQuaternion& q) const {
        return RotationQuaternion(w - q.w, x - q.x, y - q.y, z - q.z);
    }

    // Inverse of the quaternion
    RotationQuaternion inverse() const {
        return RotationQuaternion(w, -x, -y, -z);
    }

    // Distance between two quaternions
    static double distance(const RotationQuaternion& q1, const RotationQuaternion& q2) {
        double dot_product = std::fabs(q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z);
        dot_product = std::clamp(dot_product, -1.0, 1.0);
        return 2 * std::acos(dot_product);
    }

    static RotationQuaternion slerp(const RotationQuaternion& q1, const RotationQuaternion& q2, double t) {
        double dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
        const double THRESHOLD = 0.9995;

        if (dot > THRESHOLD) {
            // Linear interpolation for very close quaternions
            RotationQuaternion result = RotationQuaternion(
                q1.w + t * (q2.w - q1.w),
                q1.x + t * (q2.x - q1.x),
                q1.y + t * (q2.y - q1.y),
                q1.z + t * (q2.z - q1.z)
            );
            result.normalize();
            return result;
        }

        dot = std::clamp(dot, -1.0, 1.0);
        double theta_0 = std::acos(dot);        // theta_0 is the angle between input quaternions
        double theta = theta_0 * t;             // theta is the angle between q1 and the result

        RotationQuaternion q3 = q2 - q1 * dot;
        q3.normalize();

        return q1 * std::cos(theta) + q3 * std::sin(theta);
    }

    bool operator==(const RotationQuaternion& q) const {
        return (std::fabs(w - q.w) < 1e-9 && std::fabs(x - q.x) < 1e-9 && std::fabs(y - q.y) < 1e-9 && std::fabs(z - q.z) < 1e-9) ||
               (std::fabs(w + q.w) < 1e-9 && std::fabs(x + q.x) < 1e-9 && std::fabs(y + q.y) < 1e-9 && std::fabs(z + q.z) < 1e-9);
    }


        // 封装函数1：生成 xy 平面上均匀分布的 k 个方向
    static std::vector<RotationQuaternion> generateDirectionsInPlane(int k) {
        std::vector<RotationQuaternion> directions;
        double deltaTheta = 360.0 / k;  // 每个方向的角度间隔
        for (int i = 0; i < k; ++i) {
            double theta = i * deltaTheta * M_PI / 180.0;  // 将角度转换为弧度
            // 在 xy 平面上，旋转绕 z 轴的四元数
            RotationQuaternion direction(
                std::cos(theta / 2),  // w 分量
                0.0,                  // x 分量（xy 平面内不涉及绕 x 轴旋转）
                0.0,                  // y 分量（xy 平面内不涉及绕 y 轴旋转）
                std::sin(theta / 2)   // z 分量（绕 z 轴旋转）
            );
            direction.normalize();
            directions.push_back(direction);
        }
        return directions;
    }

    // 封装函数2：根据角度围绕 y 轴旋转平面
    static RotationQuaternion rotatePlaneAroundYAxis(double angle) {
        double radians = angle * M_PI / 180.0;  // 将角度转换为弧度
        return RotationQuaternion(
            std::cos(radians / 2),  // w 分量
            0.0,                    // x 分量（绕 y 轴旋转不涉及 x 分量）
            std::sin(radians / 2),  // y 分量（绕 y 轴的旋转）
            0.0                     // z 分量
        );
    }

    // 封装函数3：扩展函数，结合平面旋转生成方向
    static std::vector<RotationQuaternion> expandDirections(int k, int m) {
        std::vector<RotationQuaternion> expandedDirections;

        // 生成初始平面上的方向
        std::vector<RotationQuaternion> initialDirections = generateDirectionsInPlane(k);

        // 生成 m 个旋转平面
        double deltaPhi = 90.0 / m;  // 每个平面之间的角度间隔
        for (int i = 0; i < m; ++i) {
            double phi = i * deltaPhi;  // 每次绕 y 轴旋转的角度
            RotationQuaternion planeRotation = rotatePlaneAroundYAxis(phi);

            // 对每个方向应用平面旋转
            for (const auto& dir : initialDirections) {
                RotationQuaternion rotatedDirection = planeRotation * dir;
                rotatedDirection.normalize();
                expandedDirections.push_back(rotatedDirection);
            }
        }

        return expandedDirections;
    }

    static std::vector<std::array<double, 3>> fibonacciSphereSampling(int numPoints) {
        std::vector<std::array<double, 3>> points;
        double phi = M_PI * (3.0 - std::sqrt(5.0));  // 黄金角度

        for (int i = 0; i < numPoints; ++i) {
            double y = 1 - (i / (double)(numPoints - 1)) * 2;  // y 坐标在 [-1, 1] 之间均匀分布
            double radius = std::sqrt(1 - y * y);  // 根据 y 计算半径
            double theta = phi * i;  // 计算方位角 theta

            double x = std::cos(theta) * radius;
            double z = std::sin(theta) * radius;

            // 将点存入结果集
            points.push_back({x, y, z});
        }

        return points;
    }


    static std::vector<RotationQuaternion> generateQuaternions(int numPoints, double minAngle, double maxAngle) {
        std::vector<RotationQuaternion> quaternions;
        std::vector<std::array<double, 3>> axes = fibonacciSphereSampling(numPoints);  // 生成旋转轴
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> angleDist(minAngle, maxAngle);  // 生成随机角度

        for (const auto& axis : axes) {
            double theta = angleDist(gen);  // 随机选择旋转角度
            double halfTheta = theta / 2.0;

            double w = std::cos(halfTheta);  // 四元数的 w 分量
            double x = axis[0] * std::sin(halfTheta);  // 四元数的 x 分量
            double y = axis[1] * std::sin(halfTheta);  // 四元数的 y 分量
            double z = axis[2] * std::sin(halfTheta);  // 四元数的 z 分量

            RotationQuaternion q(w, x, y, z);
            q.normalize();  // 归一化四元数
            quaternions.push_back(q);  // 将生成的四元数存入结果集中
        }

        return quaternions;
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

class InadManhattanDistance : public Heuristic {
public:
    double calculate(const std::array<int, 3>& a, const std::array<int, 3>& b) const override {
        int sum = 0;
        for (int i = 0; i < 3; ++i) {
            int diff = std::abs(a[i] - b[i]);
            sum += diff;
        }
        return 10*sum;
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
