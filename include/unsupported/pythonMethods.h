#ifndef pythonMethods_h
#define pythonMethods_h

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <utility>
#include <map>
#include <numeric>


inline void linspace(float start, float end, int numPoints, std::vector<float>& result, float power = 0.0f) {
    for (int i = 0; i < numPoints; i++) {
        float t = static_cast<float>(i) / static_cast<float>(numPoints - 1); // t ranges from 0 to 1
        float s = start + (end - start) * std::pow(t, power);
        result.push_back(s);
    }
}

// template<typename type> size_t findMinMaxValueInVector(std::vector<type>& vec, bool minMax = false) {   // for min, (bool)minMax = false
//     if (vec.empty()) {
//         return -1;
//     }
//     size_t minIndex = 0;
//     type minValue = vec[minIndex];
//
//     for (size_t i = 1; i < vec.size(); i++) {
//         if (((vec[i] < minValue) && (!minMax)) || ((vec[i] > minValue) && (minMax))) {
//             minValue = vec[i];
//             minIndex = i;
//         }
//     }
//     return minIndex;
// }

inline float getEuclideanNorm(const std::vector<float>& vec) {
    float norm = 0.0f;
    for (size_t i = 0; i < vec.size(); i++) {
        norm = norm + (vec[i] * vec[i]);
    }
    norm = std::sqrt(norm);

    return norm;
}


// inline void writeVectorToCsv(const std::string& filePath, const std::vector<float>& data) {
//     std::ofstream outputFile(filePath);
//
//     if (outputFile.is_open()) {
//         for (const float& value : data) {
//             outputFile << value << "\n";
//         }
//         // Remove the trailing comma and add a newline character
//         outputFile << "\n";
//
//         outputFile.close();
//         std::cout << "Data written to " << filePath << std::endl;
//     } else {
//         std::cerr << "Unable to open file for writing: " << filePath << std::endl;
//     }
// }

// inline void writeVectorToCsv(const std::string& filePath, const std::vector<std::vector<float>>& data) {
//     std::ofstream outputFile(filePath);
//     size_t rows = data.size();
//
//     bool allSizesSame = true;
//     size_t expectedSize = data[0].size();
//
//     for (const std::vector<float>& innerVector : data) {
//         if (innerVector.size() != expectedSize) {
//             allSizesSame = false;
//             break;
//         }
//     }
//
//     if (!allSizesSame) {
//         std::cout << "Not all inner vectors have the same size." << std::endl;
//         return;
//     }
//
//     if (outputFile.is_open()) {
//         for (size_t i = 0; i < data[0].size(); i++) {
//             for (size_t j = 0; j < rows; j++) {
//                 outputFile << data[j][i] << ",";
//             }
//             outputFile << "\n";
//         }
//
//         outputFile.close();
//         std::cout << "Data written to " << filePath << std::endl;
//     } else {
//         std::cerr << "Unable to open file for writing: " << filePath << std::endl;
//     }
// }

inline float getMeanOfVector (std::vector<float>vec) {
    float sum = 0;
    size_t size = vec.size();

    for (float value : vec) {
        sum = sum + value;
    }

    sum = sum / static_cast<float>(size);

    return sum;
}

// CUSTOM DATATYPES

// template <typename AeroType>
// struct AeroSimulationResults {
//     AeroType Drag, Lift, D_profile, CLa, CL, CLzero, Area;
//     explicit AeroSimulationResults(AeroType drag = 0, AeroType lift = 0, AeroType d_profile = 0, AeroType cl = 0, AeroType cla = 0, AeroType angle = 0, AeroType area = 0) {
//         Drag = drag;
//         Lift = lift;
//         D_profile = d_profile;
//         CLa = cla;
//         CL = cl;
//         CLzero = cl - cla * angle;
//         Area = area;
//     }
//
//     void assign(AeroType drag, AeroType lift, AeroType d_profile, AeroType cl, AeroType cla, AeroType angle, AeroType area) {
//         Drag = drag;
//         Lift = lift;
//         D_profile = d_profile;
//         CLa = cla;
//         CL = cl;
//         CLzero = cl - cla * angle;
//         Area = area;
//     }
//
//     void assign(AeroSimulationResults<AeroType> copyFrom) {
//         Drag = copyFrom.Drag;
//         Lift = copyFrom.Lift;
//         D_profile = copyFrom.D_profile;
//         CLa = copyFrom.CLa;
//         CL = copyFrom.CL;
//         CLzero = copyFrom.CLzero;
//         Area = copyFrom.Area;
//     }
//
//     void printData() {
//         std::cout << "Drag: " << Drag << std::endl;
//         std::cout << "Lift" << Lift << std::endl;
//         std::cout << "D_profile" << D_profile << std::endl;
//         std::cout << "CL: " << CL << std::endl;
//         std::cout << "CLa: " << CLa << std::endl;
//         std::cout << "CLzero: " << CLzero << std::endl;
//         std::cout << "Area" << Area << std::endl;
//     }
// };



#endif