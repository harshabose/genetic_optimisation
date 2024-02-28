//
// Created by Harshavardhan Karnati on 04/02/2024.
//

#ifndef PROPULSION_SYSTEM_H
#define PROPULSION_SYSTEM_H


#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <shared_mutex>

#include "operationalPoint.h"
#include "nlopt/nlopt.h"
#include "unsupported/pythonMethods.h"
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include "/opt/homebrew/opt/python@3.11/Frameworks/Python.framework/Versions/3.11/include/python3.11/Python.h"


class propulsion_system {
public:
    // thread_manager_h::thread_manager *propulsion_threads;
    operationalPoint_h::operationalPoint *atmosphere;
    float total_POWER_required;
    propulsion_system (operationalPoint_h::operationalPoint *IN_ATMOSPHERE, const bool IN_CONTROL_WITH_RPM, const bool IN_CONTROL_WITH_BLADE_PITCH_ANGLE = false, const float IN_FIXED_RPM = 0.0f, const float IN_FIXED_PITCH = 0.0f) {
        this->atmosphere = IN_ATMOSPHERE;
        this->controll_with_RPM = IN_CONTROL_WITH_RPM;
        this->controll_with_pitch = IN_CONTROL_WITH_BLADE_PITCH_ANGLE;
        this->propellers = new PROPULSION_SYSTEM_H::propulsion_system::propeller_parameters(this, "naca652415", this->controll_with_RPM, this->controll_with_pitch, 1, IN_FIXED_RPM, IN_FIXED_PITCH);
        // this->propulsion_threads = new thread_manager_h::thread_manager(7);
    }

    ~propulsion_system () {
        delete this->propellers;
        // delete this->propulsion_threads;
    }

    void simulate_propulsion_system () {
        this->propellers->run_propeller();
        this->total_POWER_required = static_cast<float>(this->number_of_propellers) * this->propellers->total_power_required;
    }

    void change_atmospheric_conditions (const float IN_ALTITUDE, const float IN_VELOCITY, const float IN_ANGLEOF_ATTACK) {
        this->atmosphere->altitude = IN_ALTITUDE;
        this->atmosphere->velocity = IN_VELOCITY;
        this->atmosphere->velocityAoA = IN_ANGLEOF_ATTACK;

        this->atmosphere->recalc();
    }

    void set_total_thrust_required (const float IN_THRUST_REQUIRED) {
        this->total_THRUST_required = IN_THRUST_REQUIRED;
        const float thrust_required_per_propeller = this->total_THRUST_required / static_cast<float>(this->number_of_propellers);
        propellers->set_thrust_required(thrust_required_per_propeller);
    }

    void update_propulsion_system_design (const size_t IN_NUMBER_OF_PROPULSION_UNITS, const float IN_RADIUS, const size_t IN_NUMBER_OF_BLADES, const float IN_ROOT_CHORD, const std::vector<float>& IN_TAPER_EQUATION, const std::vector<float>& IN_TWIST_EQUATION) {
        this->number_of_propellers = IN_NUMBER_OF_PROPULSION_UNITS;
        this->propellers->update_variables(IN_RADIUS, IN_NUMBER_OF_BLADES, IN_ROOT_CHORD, IN_TAPER_EQUATION, IN_TWIST_EQUATION);
    }

    [[nodiscard]] size_t get_max_equation_coeffecients () const {
        return this->propellers->max_equation_coeffecients;
    }

    [[nodiscard]] float get_propeller_root_pitch () const {
        return this->propellers->root_pitch;
    }

    void set_propeller_root_pitch (const float pitch) {
        this->propellers->root_pitch = pitch;
    }

    [[nodiscard]] float get_obtained_thrust_coeff_per_propeller () const {
        return this->propellers->obtained_thrust_coeffecient;
    }

    [[nodiscard]] float get_required_thrust_coeff_per_propeller () const {
        return this->propellers->get_thrust_coeffecient();
    }

    [[nodiscard]] float get_average_blade_angle_of_attack () const {
        return this->propellers->average_angle_of_attack;
    }

    [[nodiscard]] float get_diff_thrust_coeffecient () const {
        return this->get_obtained_thrust_coeff_per_propeller() - this->propellers->get_thrust_coeffecient();
    }

    [[nodiscard]] float get_propeller_RPM () const {
        return this->propellers->RPM;
    }

    void set_propeller_RPM (const float IN_RPM) const {
        this->propellers->RPM = IN_RPM;
    }



private:

    const float M_PIF = M_PI;
    size_t number_of_propellers;
    float total_THRUST_required;
    bool controll_with_RPM = false;
    bool controll_with_pitch = false;

    class propeller_parameters {
    public:
        float radius;
        float average_angle_of_attack;
        size_t number_of_blades;
        float root_chord;
        float root_pitch;
        float obtained_thrust_coeffecient;
        float total_power_required;
        float RPM;
        size_t max_equation_coeffecients;

        propeller_parameters(PROPULSION_SYSTEM_H::propulsion_system *IN_PROPULSION_SYSTEM_OBJECT, const std::string &IN_AIRFOIL, const bool IN_CONTROLL_WITH_RPM, const bool IN_CONTROLL_WITH_PITCH, const size_t IN_MAX_EQUATION_COEFFECIENTS, const float IN_FIXED_RPM = 0.0f, const float IN_FIXED_ROOT_PITCH = 0.0f) {
            this->propulsion_system_object = IN_PROPULSION_SYSTEM_OBJECT;
            if (IN_CONTROLL_WITH_RPM) {
                if (IN_CONTROLL_WITH_PITCH) this->controller_variable = "HYBRID";
                else this->controller_variable = "RPM";
            } else this->controller_variable = "PITCH";

            this->airfoil = IN_AIRFOIL;
            this->max_equation_coeffecients = IN_MAX_EQUATION_COEFFECIENTS;
            this->number_of_blades = 0;

            for (size_t i = 0; i < this->number_of_psi_angles_divisions; i++) {
                if (this->number_of_psi_angles_divisions - 1 != 0) this->psi_angle_iterator.push_back(static_cast<float>(i) * 360.0f / (static_cast<float>(this->number_of_psi_angles_divisions) - 1.0f));
                else this->psi_angle_iterator.push_back(0.0f);
            }

            this->RPM = IN_FIXED_RPM == 0.0f ? 100.0f : IN_FIXED_RPM;
            this->root_pitch = IN_FIXED_ROOT_PITCH == 0.0f ? 20.0f : IN_FIXED_ROOT_PITCH;
        }

        void set_thrust_required(const float IN_THRUST_REQUIRED) {
            this->thrust_required = IN_THRUST_REQUIRED;
        }

        [[nodiscard]] float get_thrust_coeffecient () const {
            return this->thrust_effective_coeffecient;
        }

        void update_variables(const float IN_RADIUS, const size_t IN_NUMBER_OF_BLADES, const float IN_ROOT_CHORD, const std::vector<float> &IN_TAPER_EQUATION, const std::vector<float> &IN_TWIST_EQUATION) {
            this->radius = IN_RADIUS;
            this->number_of_blades = IN_NUMBER_OF_BLADES;
            this->root_chord = IN_ROOT_CHORD;

            if (IN_TAPER_EQUATION.size() == this->max_equation_coeffecients && IN_TWIST_EQUATION.size() == this->max_equation_coeffecients) {
                this->taper_equation_coeffecients = IN_TAPER_EQUATION;
                this->twist_equation_coeffecients = IN_TWIST_EQUATION;
            }
        }

        float run_propeller() {
            unsigned int number_of_variables;
            double control_variables[2];
            double lower_bounds[2];
            double upper_bounds[2];

            if (this->controller_variable == "HYBRID") {    // set up control and bounds for "HYBRID"
                number_of_variables = 2;
                control_variables[0] = this->RPM; control_variables[1] = this->root_pitch;
                lower_bounds[0] = 60; lower_bounds[1] = -90.0;
                upper_bounds[0] = 200.0; upper_bounds[1] = 90.0;
            } else {
                number_of_variables = 1;
                if (this->controller_variable == "RPM") {   // set up control and bounds for "RPM"
                    control_variables[0] = this->RPM; control_variables[1] = this->root_pitch;
                    lower_bounds[0] = 60; lower_bounds[1] = -90.0;
                    upper_bounds[0] = 200.0; upper_bounds[1] = 90.0;
                } else {                                    // set up control and bounds for "PITCH"
                    control_variables[0] = this->root_pitch; control_variables[1] = this->RPM;
                    lower_bounds[0] = -90.0; lower_bounds[1] = 60;
                    upper_bounds[0] = 90.0; upper_bounds[1] = 200.0;
                }
            }

            nlopt_opt run_propeller = nlopt_create(NLOPT_LN_COBYLA, number_of_variables);
            nlopt_set_min_objective(run_propeller, PROPULSION_SYSTEM_H::propulsion_system::propeller_parameters::nlopt_get_power_to_run_propeller, this);
            nlopt_set_lower_bounds(run_propeller, lower_bounds);
            nlopt_set_upper_bounds(run_propeller, upper_bounds);

            nlopt_add_equality_constraint(run_propeller, PROPULSION_SYSTEM_H::propulsion_system::propeller_parameters::nlopt_get_obtained_thrust_coeffecient, this, 1e-8);
            // nlopt_add_equality_constraint(run_propeller, PROPULSION_SYSTEM_H::propulsion_system::propeller_parameters::nlopt_get_positive_power_constraint, this, 1e-5);
            nlopt_set_xtol_rel(run_propeller, 1e-6);
            nlopt_set_maxeval(run_propeller, 500);
            double minimum_power;

            try {
                nlopt_optimize(run_propeller, control_variables, &minimum_power);
                // std::cout << this->opt_call << "\tPROPELLER CONVERGED: " << minimum_power << ", RPM:" << this->RPM << ", pitch: " << this->root_pitch << "\n\n\n" << std::endl;
                this->opt_call = 0;
            } catch (std::exception &e) {
                std::cerr << "NLOPT failed: " << e.what() << std::endl;
            }
            nlopt_destroy(run_propeller);
            return static_cast<float>(minimum_power);
        }

    private:
        std::string airfoil;
        float propeller_static_area;
        float propeller_effective_area;
        std::string controller_variable;
        float thrust_required;
        float thrust_coeffecient;
        float thrust_effective_coeffecient;
        float advanced_velocity;
        float forward_velocity;
        float inflow_velocity;
        const float root_cut_factor = 0.1f;
        float tip_loss_factor;
        std::vector<float> taper_equation_coeffecients;
        std::vector<float> twist_equation_coeffecients;
        const size_t number_of_blade_sections = 10;
        const size_t number_of_psi_angles_divisions = 1;
        std::vector<float> psi_angle_iterator;
        std::vector<float> radius_iterator;
        const float M_PIF = M_PI;
        PROPULSION_SYSTEM_H::propulsion_system *propulsion_system_object;
        size_t opt_call = 0;

        void calculate_thrust_coeffecient() {
            this->propeller_static_area = this->M_PIF * this->radius * this->radius;
            this->thrust_coeffecient = this->thrust_required / (this->propulsion_system_object->atmosphere->density * this->propeller_static_area * (this->RPM * this->radius) * (this->RPM * this->radius));

            this->tip_loss_factor = 1.0f - sqrt(2.0f * this->thrust_coeffecient) / static_cast<float>(this->number_of_blades);
            if (tip_loss_factor < 0.0f) throw std::runtime_error("Negative 'tip loss factor' encountered... Skipping the iteration...");
            this->propeller_effective_area = this->propeller_static_area * ((tip_loss_factor * tip_loss_factor) - (this->root_cut_factor * this->root_cut_factor));
            this->thrust_effective_coeffecient = this->thrust_required / (this->propulsion_system_object->atmosphere->density * this->propeller_effective_area * (this->RPM * this->radius) * (this->RPM * this->radius));
            this->radius_iterator.clear();
            for (size_t i = 0; i < this->number_of_blade_sections; i++) {
                const float root_radius = this->root_cut_factor * this->radius;
                const float tip_radius = this->radius * this->tip_loss_factor - root_radius;
                const float increment = static_cast<float>(i) / (static_cast<float>(this->number_of_blade_sections) - 1.0f);

                this->radius_iterator.push_back(root_radius + tip_radius * increment);
                if (this->radius_iterator[i] < 0.0f) {
                    std::cout << "NEGATIVE RADIUS: " << this->radius_iterator[i] << "RPM: " << this->RPM << std::endl;
                    std::exit(1);
                }
            }
        }

        void calculate_induced_velocities() {
            //RPM need a guess
            this->advanced_velocity = this->propulsion_system_object->atmosphere->velocity * std::cos(this->propulsion_system_object->atmosphere->velocityAoA * this->M_PIF / 180.0f);
            this->forward_velocity =  this->propulsion_system_object->atmosphere->velocity * std::sin(this->propulsion_system_object->atmosphere->velocityAoA * this->M_PIF / 180.0f);

            const float advanced_velocity_ratio = this->advanced_velocity / (this->RPM * this->radius);
            const float forward_velocity_ratio = this->forward_velocity / (this->RPM * this->radius);

            float inflow_velocity_guess_alpha = std::sqrt(this->thrust_effective_coeffecient / 2.0f);
            float inflow_velocity_guess_beta = 2.0f * inflow_velocity_guess_alpha;

            size_t iterative_safety_counter = 0;
            constexpr size_t iterative_safety_counter_MAX = 100;

            do {
                const float store_guess = inflow_velocity_guess_beta;

                const float resultant_velocity_ratio_alpha = pythonMethods_h::getEuclideanNorm(std::vector<float>{advanced_velocity_ratio, inflow_velocity_guess_alpha});
                const float resultant_velocity_ratio_beta = pythonMethods_h::getEuclideanNorm(std::vector<float>{advanced_velocity_ratio, inflow_velocity_guess_beta});
                const float temporary_alpha = (advanced_velocity_ratio + (this->thrust_effective_coeffecient / (2.0f * resultant_velocity_ratio_alpha)) - inflow_velocity_guess_alpha);
                const float temporary_beta = (advanced_velocity_ratio + (this->thrust_effective_coeffecient / (2.0f * resultant_velocity_ratio_beta)) - inflow_velocity_guess_beta);

                const float derivative = (temporary_alpha - temporary_beta) / (inflow_velocity_guess_alpha - inflow_velocity_guess_beta);
                inflow_velocity_guess_beta = inflow_velocity_guess_beta - (temporary_beta / derivative);
                inflow_velocity_guess_alpha = store_guess;
            } while ((std::abs(inflow_velocity_guess_alpha - inflow_velocity_guess_beta) > 1e-5f) && (iterative_safety_counter++ < iterative_safety_counter_MAX));

            const float inflow_velocity_ratio = inflow_velocity_guess_alpha;
            this->inflow_velocity = inflow_velocity_ratio * (this->RPM * this->radius);
        }

        [[nodiscard]] float get_instantaneous_inflow_ratio(const float relative_radius, const float psi_angle) const {
            const float advanced_velocity_ratio = this->advanced_velocity / (this->RPM * this->radius);
            const float inflow_velocity_ratio = this->inflow_velocity / (this->RPM * this->radius);
            const float psi_angle_rads = psi_angle * this->M_PIF / 180.0f;

            const float X = std::atan(advanced_velocity_ratio / inflow_velocity_ratio);
            const float Kx = (15.0f * this->M_PIF / 32.0f) * std::tan(X / 2.0f);
            const float Kz = 0.0f;

            return inflow_velocity_ratio * (1.0f + Kx * relative_radius * std::cos(psi_angle_rads) + Kz * relative_radius * std::sin(psi_angle_rads));
        }

        std::array<float, 2> get_sectional_aerodynamics(const float angle_of_attack, const float sectional_velocity, const float sectional_chord, const float sectional_span, const bool run_with_aerosandbox = true) {
            if (run_with_aerosandbox == true) {
                namespace py = pybind11;

                const double local_reynolds_number = this->propulsion_system_object->atmosphere->density * sectional_velocity * sectional_chord / this->propulsion_system_object->atmosphere->viscosity;
                const double local_mach = sectional_velocity / this->propulsion_system_object->atmosphere->speedOfSound;

                py::module asb = py::module::import("aerosandbox");
                py::module np = py::module::import("aerosandbox.numpy");

                py::object sectional_airfoil = asb.attr("Airfoil")(
                    py::arg("name") = py::cast(this->airfoil)
                );

                py::dict sectional_aero_results = sectional_airfoil.attr("get_aero_from_neuralfoil")(
                    py::arg("alpha") = py::cast(angle_of_attack),
                    py::arg("Re") = py::cast(local_reynolds_number),
                    py::arg("mach") = py::cast(local_mach),
                    py::arg("include_360_deg_effects") = py::cast(true)
                );

                const float sectional_area = sectional_chord * sectional_span;
                const float dynamic_pressure = 0.5f * this->propulsion_system_object->atmosphere->density * (sectional_velocity * sectional_velocity) * sectional_area;
                const float lift = sectional_aero_results["CL"].cast<float>() * dynamic_pressure;
                const float drag = sectional_aero_results["CD"].cast<float>() * dynamic_pressure;

                return std::array<float, 2>{lift, drag};
            } else {
                const float lift = 0.0f;
                const float drag = 0.0f;

                return std::array<float, 2>{lift, drag};
            }
        }

        void calculate_propeller_forces_moments(const float guess_RPM, const float guess_Pitch) {
            this->RPM = guess_RPM;
            this->root_pitch = guess_Pitch;

            float total_lift = 0.0f, total_drag = 0.0f, force_x = 0.0f, force_z = 0.0f, total_torque = 0.0f;
            this->average_angle_of_attack = 0.0f;

            try {
                auto lambda_wrapper = [&, this] (const size_t start_index, const size_t end_index) {
                    for (size_t i = start_index; i < end_index; i++) {
                        const float sectional_span = this->radius_iterator[1] - this->radius_iterator[0];
                        for (size_t j = 0; j < this->number_of_blade_sections; j++) {
                            const float tangential_sectional_velocity = this->radius_iterator[j] * this->RPM + this->forward_velocity * std::cos(this->psi_angle_iterator[i] * this->M_PIF / 180.0f);
                            const float radial_sectional_velocity = this->advanced_velocity * std::sin(this->psi_angle_iterator[i] * this->M_PIF / 180.0f);
                            const float axial_sectional_velocity = this->RPM * this->radius * this->get_instantaneous_inflow_ratio(this->radius_iterator[j] / this->radius, this->psi_angle_iterator[i]);

                            const float resultant_sectional_velocity = pythonMethods_h::getEuclideanNorm(std::vector<float>{axial_sectional_velocity, tangential_sectional_velocity});
                            const float phi = std::atan(axial_sectional_velocity / tangential_sectional_velocity);

                            const float sectional_angle_of_attack = this->root_pitch + this->solve_polynomial(this->twist_equation_coeffecients, this->radius_iterator[j] / this->radius);
                            const float sectional_chord = this->root_chord * this->solve_polynomial(this->taper_equation_coeffecients, this->radius_iterator[j] / this->radius);
                            const float effective_sectional_angle_of_attack = sectional_angle_of_attack - (phi * 180.0f / this->M_PIF);

                            auto [sectional_lift, sectional_drag] = this->get_sectional_aerodynamics(effective_sectional_angle_of_attack, resultant_sectional_velocity, sectional_chord, sectional_span, true);
                            this->average_angle_of_attack =+ effective_sectional_angle_of_attack / static_cast<float>(this->number_of_blade_sections * this->number_of_psi_angles_divisions);
                            // std::cout << effective_sectional_angle_of_attack << "\t -> " << sectional_lift << ", " << sectional_drag << std::endl;
                            {
                                // std::shared_lock<std::shared_mutex> write_lock(mutex_);
                                total_lift += sectional_lift;
                                total_drag += sectional_drag;
                                force_x += sectional_lift * std::sin(phi) + sectional_drag * std::cos(phi);
                                force_z += sectional_lift * std::cos(phi) - sectional_drag * std::sin(phi);
                                total_torque += this->radius_iterator[j] * (sectional_lift * std::sin(phi) + sectional_drag * std::cos(phi));
                            }
                        }
                    }
                };
                this->calculate_thrust_coeffecient();
                this->calculate_induced_velocities();

                // pybind11::gil_scoped_release release;
                lambda_wrapper(0, this->number_of_psi_angles_divisions);    //TODO some error with loops_per_block in thread_manager.h

                // this->propulsion_system_object->propulsion_threads->push_loop(static_cast<size_t>(0), this->number_of_psi_angles_divisions, 5, lambda_wrapper);
                // this->propulsion_system_object->propulsion_threads->wait_for_all_tasks_in_que();
                // pybind11::gil_scoped_acquire acquire;
                total_lift *= static_cast<float>(this->number_of_blades) / static_cast<float>(this->number_of_psi_angles_divisions);
                total_drag *= static_cast<float>(this->number_of_blades) / static_cast<float>(this->number_of_psi_angles_divisions);
                force_x *= static_cast<float>(this->number_of_blades) / static_cast<float>(this->number_of_psi_angles_divisions);
                force_z *= static_cast<float>(this->number_of_blades) / static_cast<float>(this->number_of_psi_angles_divisions);
                total_torque *= static_cast<float>(this->number_of_blades) / static_cast<float>(this->number_of_psi_angles_divisions);

                this->obtained_thrust_coeffecient = force_z / (this->propulsion_system_object->atmosphere->density * this->propeller_effective_area * (this->RPM * this->radius) * (this->RPM * this->radius));
                this->total_power_required = total_torque * this->RPM;
            } catch (std::exception& e) {
                std::cerr << "WARNING: " << e.what() << std::endl;
                this->total_power_required = HUGE_VAL;
                this->obtained_thrust_coeffecient = HUGE_VAL;
            }
        }

        static double nlopt_get_obtained_thrust_coeffecient(unsigned n, const double *x, double *grad, void *data) {
            const auto cast_data = static_cast<PROPULSION_SYSTEM_H::propulsion_system::propeller_parameters *>(data);
            const auto diff_thrust_coeffecient = static_cast<double>(cast_data->obtained_thrust_coeffecient - cast_data->thrust_effective_coeffecient);
            // std::cout << " \tDIFFERENCE IN THRUST COEFFS: obtained = " << cast_data->obtained_thrust_coeffecient << "; needed = " << cast_data->thrust_effective_coeffecient << "; DIFFERENCE = " << diff_thrust_coeffecient << std::endl;
            return diff_thrust_coeffecient;
        }

        static double nlopt_get_positive_power_constraint(unsigned n, const double *x, double *grad, void *data) {
            const auto cast_data = static_cast<PROPULSION_SYSTEM_H::propulsion_system::propeller_parameters *>(data);
            return -1.0 * cast_data->total_power_required;
        }

        static double nlopt_get_power_to_run_propeller(unsigned n, const double *x, double *grad, void *data) {
            const auto start = std::chrono::high_resolution_clock::now();
            const auto cast_data = static_cast<PROPULSION_SYSTEM_H::propulsion_system::propeller_parameters *>(data);

            float guess_RPM, guess_root_pitch;
            if (cast_data->controller_variable == "HYBRID") {
                guess_RPM = static_cast<float>(x[0]); guess_root_pitch = static_cast<float>(x[1]);
            } else if (cast_data->controller_variable == "RPM") {
                guess_RPM = static_cast<float>(x[0]);
                guess_root_pitch = cast_data->root_pitch;
            }else {
                guess_root_pitch = static_cast<float>(x[0]);
                guess_RPM = cast_data->RPM;
            }

            cast_data->calculate_propeller_forces_moments(guess_RPM, guess_root_pitch);
            const auto end = std::chrono::high_resolution_clock::now();
            const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            // nlopt_get_obtained_thrust_coeffecient(1, x, grad, cast_data);
            cast_data->opt_call++;
            // std::cout << "\n" << cast_data->opt_call;
            // std::cout << "\tRPM: " << guess_RPM << ", pitch: " << guess_root_pitch << " -> " << cast_data->total_power_required << ", TIME TAKEN: " << duration.count() << " milliseconds." << std::endl;
            return static_cast<double>(cast_data->total_power_required);
        }

        [[nodiscard]] static float solve_polynomial(const std::vector<float> &polynomial_coeffecients, const float x) {
            const size_t polynomial_degree = polynomial_coeffecients.size();
            float solved_value = 0.0f;
            for (size_t i = 0; i < polynomial_degree; i++) {
                solved_value += polynomial_coeffecients[i] * std::pow(x, static_cast<float>(i));
            }
            return solved_value;
        }
    }  *propellers;

    static double get_total_power_to_run_propulsion_system (unsigned int n, const double *x, double *grad, void *data) {
        auto cast_data = static_cast<PROPULSION_SYSTEM_H::propulsion_system*>(data);
        const auto NUMBER_OF_PROPULSION_UNITS = static_cast<size_t>(x[0]);
        const auto RADIUS = static_cast<float>(x[1]);
        const auto NUMBER_OF_BLADES = static_cast<size_t>(x[2]);
        const auto ROOT_CHORD = static_cast<float>(x[3]);
        std::vector<float> TAPER, TWIST;
        for (size_t i = 4; i < 4 + cast_data->propellers->max_equation_coeffecients; i++) {
            TAPER.push_back(static_cast<float>(x[i]));
            TWIST.push_back(static_cast<float>(x[i + cast_data->propellers->max_equation_coeffecients]));
        }
        cast_data->update_propulsion_system_design(NUMBER_OF_PROPULSION_UNITS, RADIUS, NUMBER_OF_BLADES, ROOT_CHORD, TAPER, TWIST);

    }
};

void genetic_propulstion_system ();


#endif //PROPULSION_SYSTEM_H
