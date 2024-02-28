//
//  operationalPoint.h
//  Conceptual Design Kit
//
//  Created by Harshavardhan Karnati on 31/08/2023.
//

#ifndef operationalPoint_h
#define operationalPoint_h

#include <iostream>
#include <fstream>
#include "nlohmann/json.hpp"

class operationalPoint {
public:
    float density, altitude, temperature, viscosity, speedOfSound, pressure;
    float velocity, velocityAoA, velocityBeta;
    float pitchRate, rollRate, yawRate;
    float refSurfaceArea, refLength;
    float temperatureOffset;

     operationalPoint (const float In_altitude, const float In_velocity, const float In_temperatureOffset = 0.0f, const float In_velocityAoA = 0.0f, const float In_velocityBeta = 0.0f, const float In_pitchRate = 0.0f, const float In_rollRate = 0.0f, const float In_yawRate = 0.0f, const float In_refSurfaceArea = 0.0f, const float In_refLength = 0.0f) {
        this->altitude = In_altitude;
        this->temperatureOffset = In_temperatureOffset;
        this->velocity = In_velocity;
        this->velocityAoA = In_velocityAoA;
        this->velocityBeta = In_velocityBeta;
        this->pitchRate = In_pitchRate;
        this->rollRate = In_rollRate;
        this->yawRate = In_yawRate;
        this->refSurfaceArea = In_refSurfaceArea;
        this->refLength = In_refLength;
        this->temperature = this->getTemperature();
        this->viscosity = this->getViscosity();
        this->speedOfSound = this->getSpeedOfSound();
        this->pressure = this->getPressure();
        this->density = this->getDensity();
    }

    void recalc() {
        this->temperature = this->getTemperature();
        this->viscosity = this->getViscosity();
        this->speedOfSound = this->getSpeedOfSound();
        this->pressure = this->getPressure();
        this->density = this->getDensity();
    }

    void print_results_to_json (const std::string path = "operationL_point_h.json", const std::string run = "run_1/") {
         const std::string file_path = "../assets/" + run + path;
         std::ofstream operational_file(file_path);
         const nlohmann::json json_data = this->convert_operational_to_json();
         try {
             if(!operational_file.is_open()) {
                 throw std::runtime_error("Could not open file for writing: " + file_path);
             }
             std::cout << "Writing to " << path  << std::endl;
             operational_file << json_data.dump(4);
             operational_file.close();
         } catch (const std::runtime_error& e) {
             std::cerr << "An error occurred while writting to 'parametric' JSON: " << e.what() << std::endl;
             std::cerr << "Parametric JSON DATA NOT STORED" << std::endl;
         }
    }

    void read_results_from_json (const std::string path = "operationL_point_h.json", const std::string run = "run_1/") {
         const std::string file_path = "../assets/" + run + path;
         std::ifstream operational_file(file_path, std::ios::ate);

         if (!operational_file.is_open() && !operational_file.tellg() == 0) {
             throw std::runtime_error("Could not open file for reading or it is empty: " + file_path);
         }

         operational_file.seekg(0, std::ios::beg);

         const nlohmann::json json_data = nlohmann::json::parse(operational_file);
         operational_file.close();

         try {
             this->convert_json_to_operational(json_data);
         } catch (std::exception& e) {
             throw;
         }
     }


private:

    const float R = 8.31f;
    const float M = 28.96f * 0.001f;
    const float gamma = 1.4;

    [[nodiscard]] float getDensity () const {
        return (1.225f * pow((1.0f - 22.558e-6f * this->altitude), 4.2559f));
    }

    [[nodiscard]] float getTemperature () const {
        return ((15.0f - 6.5f * (this->altitude / 1000.0f)) + 273.15f + this->temperatureOffset);
    }

    [[nodiscard]] float getViscosity () const {
        return 1.48e-06f * (pow(this->temperature, 1.5f)) * (1.0f / (this->temperature + 110.4f));
    }

    [[nodiscard]] float getSpeedOfSound () const {
        return (sqrt(this->gamma * (this->R / this->M) * this->temperature));
    }

    [[nodiscard]] float getPressure (const float basePressure = 101325.0f) const {
        return basePressure * pow((1.0f - 0.0065f * (this->altitude / this->temperature)), 5.2561f);
    }

    nlohmann::json convert_operational_to_json () {
        nlohmann::json j;
        //public
        j["density"] = this->density;
        j["altitude"] = this->altitude;
        j["temperature"] = this->temperature;
        j["viscosity"] = this->viscosity;
        j["speedOfSound"] = this->speedOfSound;
        j["pressure"] = this->pressure;
        j["velocity"] = this->velocity;
        j["velocityAoA"] =this-> velocityAoA;
        j["velocityBeta"] = this->velocityBeta;
        j["pitchRate"] = this->pitchRate;
        j["rollRate"] = this->rollRate;
        j["yawRate"] = this->yawRate;
        j["refSurfaceArea"] = this->refSurfaceArea;
        j["refLength"] = this->refLength;
        j["temperatureOffset"] = this->temperatureOffset;

        return j;
    }

    void convert_json_to_operational (const nlohmann::json& j) {
        try {
            this->density = j.at("density").get<float>();
            this->altitude = j.at("altitude").get<float>();
            this->temperature = j.at("temperature").get<float>();
            this->viscosity = j.at("viscosity").get<float>();
            this->speedOfSound = j.at("speedOfSound").get<float>();
            this->pressure = j.at("pressure").get<float>();
            this->velocity = j.at("velocity").get<float>();
            this->velocityAoA = j.at("velocityAoA").get<float>();
            this->velocityBeta = j.at("velocityBeta").get<float>();
            this->pitchRate = j.at("pitchRate").get<float>();
            this->rollRate = j.at("rollRate").get<float>();
            this->yawRate = j.at("yawRate").get<float>();
            this->refSurfaceArea = j.at("refSurfaceArea").get<float>();
            this->refLength = j.at("refLength").get<float>();
            this->temperatureOffset = j.at("temperatureOffset").get<float>();
        } catch (std::exception& e) {
            std::cerr << "Converting from JSON to Operational Failed: " << e.what() << std::endl;
            throw;
        }
    }

};


#endif /* operationalPoint_h */