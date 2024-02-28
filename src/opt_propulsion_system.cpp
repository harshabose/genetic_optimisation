//
// Created by Harshavardhan Karnati on 13/02/2024.
//

#include <iostream>
#include <string>
#include <fstream>
#include <memory>

#include "../include/propulsion_system.h"
#include "../include/operationalPoint.h"
#include "../include/genetic_optimisation.h"


// optimising for the climb segment
static constexpr float altitude = 3941.37869f;
static constexpr float velocity = 61.92303103f;
static constexpr float angle_of_attack = 3.080505517f;
static constexpr float thrust_required = 3687.74f;
static constexpr float gamma = 4.695810749f;
static size_t opt_count = 0;

static constexpr size_t number_of_propellers_ = 4;
static constexpr size_t number_of_blades_ = 3;
static constexpr float radius_ = 1.0f;
static constexpr float chord_ = 0.3f;

float get_propulsion_system_power (const std::size_t number_of_propellers, const std::size_t number_of_blades, const float radius, const float chord, PROPULSION_SYSTEM_H::propulsion_system* propulsion_system) {
    propulsion_system->set_propeller_root_pitch(0.0f);
    propulsion_system->set_propeller_RPM(100.0f);
    propulsion_system->update_propulsion_system_design (
        number_of_propellers, radius, number_of_blades, chord,
        std::vector<float>{1.0f},
        std::vector<float>{0.0f});

    propulsion_system->set_total_thrust_required(thrust_required);
    propulsion_system->simulate_propulsion_system();

    return propulsion_system->total_POWER_required;
}

float thrust_constraint_eq (const std::size_t number_of_propellers, const std::size_t number_of_blades, const float radius, const float chord, PROPULSION_SYSTEM_H::propulsion_system* propulsion_system) {
    return propulsion_system->get_diff_thrust_coeffecient();
}

void genetic_propulstion_system () {
    const auto atmosphere = std::make_shared<operationalPoint_h::operationalPoint>(altitude, velocity, 0, angle_of_attack);
    auto propulsion_system = std::make_shared<PROPULSION_SYSTEM_H::propulsion_system>(atmosphere.get(), true, true);

    GENETIC_OPTIMISATION_H::create_constraint<decltype(thrust_constraint_eq), float, const std::size_t, const std::size_t, const float, const float, PROPULSION_SYSTEM_H::propulsion_system*>
    constraint(thrust_constraint_eq, "=", 0, 0.00001f);

    std::tuple<std::size_t, std::size_t, float, float, PROPULSION_SYSTEM_H::propulsion_system*> lower_bounds = {2, 2, 0.25, 0.1};
    std::tuple<std::size_t, std::size_t, float, float, PROPULSION_SYSTEM_H::propulsion_system*> upper_bounds = {10, 6, 2, 0.5};

    auto genetic_operator = GENETIC_OPTIMISATION_H::genetic_algorithm<decltype(get_propulsion_system_power), const std::size_t, const std::size_t, const float, const float, PROPULSION_SYSTEM_H::propulsion_system*>(get_propulsion_system_power);

    genetic_operator.add_constraints(constraint);
    genetic_operator.set_lower_bounds(lower_bounds);
    genetic_operator.set_upper_bounds(upper_bounds);
    genetic_operator.enable_search_space_reduction(true);

    genetic_operator.perform_genetic_optimisation();
}