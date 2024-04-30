#include <iostream>
#include <unordered_map>
#include <string>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <algorithm>


std::string extractFileName(const std::string& path)
{
    std::filesystem::path p = path;
    return p.stem().string(); 
}

std::vector<int> findMapSize(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) 
    {
        std::cerr << "Failed to open file for writing.\n";
        return {};
    }

    std::string line;
    int maxX, maxY, maxZ = 0; 
    while (std::getline(file, line)) 
    {
        int x, y, z, value;
        char ch1, ch2, ch3, ch4, ch5, ch6, ch7;
        std::istringstream iss(line);
        if (iss >> ch1 >> x >> ch2 >> y >> ch3 >> z >> ch4 >> value >> ch5) 
        {
            if (ch1 == '(' && ch2 == ',' && ch3 == ',' && ch4 == ',' && ch5 == ')') 
            {
                maxX = std::max(maxX, x);
                maxY = std::max(maxY, x);
                maxZ = std::max(maxZ, x);
            }
        }
    }
    
    file.close();
    return {maxX,maxY,maxZ}; 

}



class ThreeDMap 
{
private:
    std::unordered_map<std::string, std::string> map;
    int sizeX, sizeY, sizeZ;
    std::string mapName; 

    std::string generateKey(int x, int y, int z) 
    {
        return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(z);
    }

public:
    ThreeDMap(int sizeX, int sizeY, int sizeZ) : sizeX(sizeX), sizeY(sizeY), sizeZ(sizeZ)
    {
        std::srand(std::time(nullptr)); // random seed
    }

    ThreeDMap(const std:: string & MapAddress) : mapName(extractFileName(MapAddress)+"_path.map")          
    {
        std::vector<int> coordinates = findMapSize(MapAddress);
        sizeX = coordinates[0];
        sizeY = coordinates[1];
        sizeZ = coordinates[2];
        std::srand(std::time(nullptr)); // random seed
    }


    void generateRandomMap() 
    {
        for (int x = 0; x < sizeX; ++x) {
            for (int y = 0; y < sizeY; ++y) {
                for (int z = 0; z < sizeZ; ++z) {
                    std::string key = generateKey(x, y, z);
                    std::string value = (std::rand() % 100 < 20) ? "ob" : "non"; // 20%
                    map[key] = value;
                }
            }
        }
    }

    std::string getMapValue(int x, int y, int z) 
    {
        std::string key = generateKey(x, y, z);
        auto it = map.find(key);
        if (it != map.end()) 
        {
            return it->second;
        } 
        else 
        {
            return "non"; //without obstacle
        }
    }

    void setMapValue(int x, int y, int z, const std::string& value)
    {
        std::string key = generateKey(x, y, z);
        map[key] = value;
    }

    void removeMapValue(int x, int y, int z) 
    {
        std::string key = generateKey(x, y, z);
        map.erase(key);
    }

    void saveToFile(const std::string& filename) const // path, extension list...
    {
        std::ofstream file(filename);
        if (!file.is_open()) 
        {
            std::cerr << "Failed to open file for writing.\n";
            return;
        }
        // Titles:
        file << "Map_Name: " << mapName << std::endl;

        for (auto& entry : map) 
        {
            std::string key = entry.first; //coordinate "x_y_z"
            std::string value = entry.second; // value
            int x, y, z;
            sscanf(key.c_str(), "%d_%d_%d", &x, &y, &z); // Parse the key back to coordinates
            file << "(" << x << ", " << y << ", " << z << ", " << value << ")\n";
        }

        file.close();
    }
};


int main() 
{
    ThreeDMap myMap(10, 10, 10);
    myMap.generateRandomMap(); 
    std::cout << "Value at (1,1,1): " << myMap.getMapValue(1, 1, 1) << std::endl;
    myMap.setMapValue(1, 1, 1, "ob");
    std::cout << "New value at (1,1,1): " << myMap.getMapValue(1, 1, 1) << std::endl;
    std::cout << "Check 101" << myMap.getMapValue(101,101,101)<<std::endl; 
    myMap.saveToFile("sparsemap.map"); 

    //ThreeDMap myMap2 ("sparsemap.map");
    //std::cout << "Value at (1,1,1): " << myMap2.getMapValue(1, 1, 1) << std::endl;


    return 0;
}

