//
// Created by Harshavardhan Karnati on 15/02/2024.
//

#ifndef GENETIC_OPTIMISATION_H
#define GENETIC_OPTIMISATION_H

#define VERBOSITY 1
#define GRAPH 0

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <numeric>
#include <limits>
#include <cmath>
// #include <type_traits>
#include <functional>
#include <random>
#include <tuple>
#include <set>
#include <algorithm>
#include <any>


#if VERBOSITY
#define VERBOSE_PRINT(x)  do { std::ostringstream oss; oss << x; std::cout << oss.str() << std::endl; } while (0)
#else
#define VERBOSE_PRINT(x)
#endif

#if GRAPH
#define WITHOUT_NUMPY
#include "matplotlib-cpp/matplotlibcpp.h"
#endif



template <class type>
using remove_all_qual = std::remove_pointer_t<std::decay_t<type>>;

template <class type>
static constexpr bool is_string_v = std::is_constructible_v<std::string, std::decay_t<type>>;

template <class return_type, class func_type, class... args_type>
static constexpr bool check_func_v = std::is_invocable_r_v<return_type, remove_all_qual<func_type>, args_type...>;

template <class... types>
static constexpr bool is_number_v = ((std::is_arithmetic_v<std::remove_pointer_t<types>> && !is_string_v<std::remove_pointer_t<types>>) && ...);

template <std::size_t i, class tuple_type> using tuple_args_type_at = std::decay_t<std::tuple_element_t<i, std::remove_reference_t<tuple_type>>>;

template <class... types>
struct are_same : std::true_type{};
template <class T1, class... first_remaining_type, class T2, class... second_remaining_type>
struct are_same<std::tuple<T1, first_remaining_type...>, std::tuple<T2, second_remaining_type...>> :
    std::conditional_t<std::is_same_v<T1, T2>, are_same<std::tuple<first_remaining_type...>, std::tuple<second_remaining_type...>>, std::false_type>{};

template <class tuple_type>
static bool is_tuple_default_initialized (tuple_type&& tup) {
    auto check = [] <class tuple_type_, std::size_t... i_> (tuple_type_&& tup_, std::index_sequence<i_...>) -> bool {
        auto check_at = [] <typename _type_> (_type_&& _arg_) -> bool {
            return _arg_ == _type_{};
        };
        return (check_at.operator()(
            std::forward<tuple_args_type_at<i_, tuple_type_>>(std::get<i_>(std::forward<tuple_type_>(tup_))))
            && ...);
    };
    return check.operator()(std::forward<tuple_type>(tup), std::index_sequence_for<std::tuple_size<tuple_type>>{});
}

template <class constraint_func_types, class value_type, class... args_type>
struct create_constraint {
    std::function<constraint_func_types> func;
    std::string operator_;
    value_type value;
    float tolerance = 0.00001f;

    template <class constraint_func_types_, class operator_type_, class value_type_, class tolerance_type_>
    requires (check_func_v<value_type_, constraint_func_types_, args_type...> && is_string_v<operator_type_> && std::is_floating_point_v<tolerance_type_>)
    create_constraint (constraint_func_types_&& IN_FUNC, operator_type_&& IN_OPERATOR, value_type_&& IN_VALUE, tolerance_type_&& IN_TOLERANCE) {
        this->func = std::forward<constraint_func_types_>(IN_FUNC);
        this->operator_ = std::forward<operator_type_>(IN_OPERATOR);
        this->value = std::forward<value_type_>(IN_VALUE);
        this->tolerance = std::forward<tolerance_type_>(IN_TOLERANCE);
    }
};







template<class func_type, class... args_type>
class genetic_algorithm {
public:
    using return_type =  std::invoke_result_t<std::remove_pointer_t<std::decay_t<func_type>>, args_type...> ;
    std::tuple<args_type...> optimal_point;
    return_type optimal_fitness{};

    template <class func_type_>
    requires (check_func_v<return_type, func_type_, args_type...>)
    explicit genetic_algorithm (func_type_&& IN_FUNC, bool IN_MINIMISE = true) {
        this->minimise = IN_MINIMISE;
        this->objective_function = std::forward<func_type>(IN_FUNC);
        this->number_of_variables = (sizeof...(args_type));
        this->population_size = 50 * (number_of_variables + 1);
        this->chromosome_table.resize(this->population_size);
        this->fitness_table.resize(this->population_size);
    }

    ~genetic_algorithm () {
        delete constraint_manager_;
    }

    template <
        template <class, class, class...> class... create_constraint_type,
    class constraint_func_type, typename value_type, class... args_type_>
    void add_constraints(create_constraint_type<constraint_func_type, value_type, args_type_...>&... constraints) {
        this->constraints_on = true;
        this->constraint_manager_ = new constraint_manager<decltype(constraints.func)...>(std::move(constraints.func)..., (constraints.value)...); // constraints.value...
        this->constraint_manager_->add_operators(std::vector<std::string>{constraints.operator_...});
        this->constraint_manager_->add_tolerances(std::vector<float>{constraints.tolerance...});
    }

    void set_population_size (const std::size_t &IN_POPULATION_SIZE) {
        std::cout << "Changing population from '" << this->population_size << "' to '" << IN_POPULATION_SIZE << std::endl;
        this->population_size = IN_POPULATION_SIZE;
        this->chromosome_table.resize(this->population_size);
        this->fitness_table.resize(this->population_size);
    }

    template <class... args_type_>
    requires (are_same<std::tuple<args_type_...>, std::tuple<args_type...>>::value)
    void set_lower_bounds (args_type_&&... IN_LOWER_BOUNDS) {
        this->lower_bounds = std::make_tuple(std::forward<args_type_>(IN_LOWER_BOUNDS)...);
    }

    void set_lower_bounds (const std::tuple<args_type...> &IN_LOWER_BOUNDS) {
        this->lower_bounds = IN_LOWER_BOUNDS;
    }

    template <class... args_type_>
    requires (are_same<std::tuple<args_type_...>, std::tuple<args_type...>>::value)
    void set_upper_bounds (args_type_&&... IN_UPPER_BOUNDS) {
        this->upper_bounds = std::make_tuple(std::forward<args_type_>(IN_UPPER_BOUNDS)...);
    }

    void set_upper_bounds (const std::tuple<args_type...> &IN_UPPER_BOUNDS) {
        this->upper_bounds = IN_UPPER_BOUNDS;
    }

    void set_max_eval (const std::size_t &IN_MAX_EVAL) {
        this->max_eval = IN_MAX_EVAL;
    }

    void set_max_no_improvement_count (const std::size_t &IN_MAX_COUNT) {
        this->max_no_improvement_count = IN_MAX_COUNT;
        if (this->max_no_improvement_count > this->max_eval) {
            std::cerr << "max_no_improvement_count needs to be lower than max_eval_count." << std::endl;
            std::cerr << "max_eval_count is: " << this->max_eval << ". Use (public) 'set_max_eval' method to set max_eval accordingly" << std::endl;
        }
    }

    void set_crossover_probability (const float &IN_CROSSOVER_PROBABILITY = 0) {
        if (IN_CROSSOVER_PROBABILITY < 0 || IN_CROSSOVER_PROBABILITY > 1) {
            generate_random_arthmetic randomiser = generate_random_arthmetic();
            this->crossover_probability = randomiser.get_random_value(static_cast<float>(0), static_cast<float>(1));
        } else this->crossover_probability = IN_CROSSOVER_PROBABILITY;

        if (this->mutation_probability + this->crossover_probability != 1) {
            this->mutation_probability = 1 - this->crossover_probability;
        }
    }

    void set_mutation_probability (const float IN_MUTATION_PROBABILITY = 0) {
        if (IN_MUTATION_PROBABILITY < 0 || IN_MUTATION_PROBABILITY > 1) {
            generate_random_arthmetic randomiser = generate_random_arthmetic();
            this->mutation_probability = randomiser.get_random_value(static_cast<float>(0), static_cast<float>(1));
        } else this->mutation_probability = IN_MUTATION_PROBABILITY;

        if (this->mutation_probability + this->crossover_probability != 1) {
            this->crossover_probability = 1 - this->mutation_probability;
        }
    }

    void enable_search_space_reduction (const bool &IN_BOOL, const float &IN_REDUCTION_RATE = 0.2f,
        const float &IN_TOLERANCE = 0.001f, const std::size_t &IN_MAX_EVAL = 10) {
        this->search_space_reduction = IN_BOOL;
        this->search_space_reduction_rate = IN_REDUCTION_RATE;
        this->search_space_reduction_fitness_tolerance = IN_TOLERANCE;
        this->search_space_reduction_eval = IN_MAX_EVAL;
    }

    void perform_genetic_optimisation (std::size_t IN_SEARCH_SPACE_EVAL = 0) {
        check_bounds(this->lower_bounds, this->upper_bounds);
        this->current_generation = 1;
        std::size_t improvement_counter = 0;
        this->generate_population();
        if (IN_SEARCH_SPACE_EVAL > 0) {
            this->chromosome_table.at(0) = this->optimal_point;
            this->fitness_table.at(0) = this->optimal_fitness;
        }
        return_type old_optimal_fitness = this->optimal_fitness;
        do {
            VERBOSE_PRINT("@ ITERATION: " << this->current_generation << "...");
            if (this->survivor_selector(this->rank_chromosome_table())) improvement_counter++;
            else improvement_counter = 0;
            VERBOSE_PRINT("TOP FITNESS VALUE: " << this->fitness_table.at(0) << "..." << "\n");
        } while (this->current_generation++ < this->max_eval && (improvement_counter < this->max_no_improvement_count));
        this->optimal_point = this->chromosome_table.at(0);
        this->optimal_fitness = this->fitness_table.at(0);

        VERBOSE_PRINT("TERMINATION CRITERIA HIT...");
        if (this->current_generation >= this->max_eval) VERBOSE_PRINT("MAX EVAL CRITERIA HIT...");
        if (improvement_counter >= this->max_no_improvement_count) VERBOSE_PRINT("MAX NO IMPROVEMENT CRITERIA HIT...");

#if GRAPH
        this->graph.pushback(this->fitness_table.at(0));
#endif

        if (this->search_space_reduction &&
            std::abs(old_optimal_fitness - this->optimal_fitness) > this->search_space_reduction_fitness_tolerance &&
            IN_SEARCH_SPACE_EVAL < this->search_space_reduction_eval) {

            VERBOSE_PRINT("Performing SEARCH SPACE REDUCTION");
            this->hypercube_reduction();
            this->perform_genetic_optimisation(++IN_SEARCH_SPACE_EVAL);
        }
    }

    std::pair<return_type, std::tuple<args_type...>> get_optimised_values () {
        std::cout << "Optimal Fitness found -> " << this->optimal_fitness << std::endl;
        auto print_chromosome = [this] <std::size_t... i> (std::index_sequence<i...>) {
            auto print_at = [this] <std::size_t i_> () {
                std::cout << std::get<i_>(this->chromosome_table.at(0));
                if (i_ < this->population_size - 1) std::cout << ", ";
            };
            (print_at.template operator()<i>(),...);
            std::cout << std::endl;
        };
        std::cout << "at -> "; print_chromosome.operator()(std::index_sequence_for<args_type...>{});
        return std::make_pair(this->fitness_table.at(0), this->chromosome_table.at(0));
    }

protected:

    struct generate_random_arthmetic {
        std::random_device rd;
        mutable std::mt19937 gen;

        explicit generate_random_arthmetic () : gen(rd()) {};

        template <typename type>
        type get_random_value (const type min, const type max, const float bias = 0.5) {
            if constexpr (std::numeric_limits<type>::is_integer && !std::is_same_v<type, bool>) {
                std::uniform_int_distribution<type> engine(min, max);
                return engine(gen);
            } else if constexpr (!std::numeric_limits<type>::is_integer && std::is_floating_point_v<type>) {
                std::uniform_real_distribution<type> engine(min, max);
                return engine(gen);
            } else if constexpr (std::is_same_v<type, bool>) {
                std::bernoulli_distribution engine(bias);
                return engine(gen);
            }
            return min;
        }

        template <typename type>
        void fill_vector_with_unique_random (std::vector<type> &vector_, const type min, const type max, const float bias = 0.5) const {
            std::set<type> unique_numbers;
            for (size_t i = 0; i < vector_.size(); i++) {
                type random_number;
                do {
                    random_number = this->template get_random_value<std::size_t>(static_cast<std::size_t>(min), static_cast<std::size_t>(max), bias);
                } while (!unique_numbers.insert(random_number).second); // Ensure uniqueness
                vector_[i] = random_number;
            }
        }

        template <typename type>
        void fill_vector_with_unique_random (std::vector<type> &vector_, const type min, const type max, generate_random_arthmetic* this_ptr, const float bias = 0.5) const {
            std::set<type> unique_numbers;
            for (size_t i = 0; i < vector_.size(); i++) {
                type random_number;
                do {
                    random_number = this_ptr->get_random_value<type>(min, max, bias);
                } while (!unique_numbers.insert(random_number).second); // Ensure uniqueness
                vector_[i] = random_number;
            }
        }

        static void fill_bool_vector_with_random (std::vector<bool>& vector_, generate_random_arthmetic* this_ptr, const float bias = 0.5) {
            for (auto && bool_ : vector_) bool_ = this_ptr->get_random_value<bool>(false, true, bias);
        }
    };

    struct constraint_manager_base {
        return_type penalty{};
        virtual ~constraint_manager_base() = default;
        virtual void add_operators (const std::vector<std::string>& IN_OPERATORS) = 0;
        virtual void get_penalty (std::tuple<args_type...> &IN_ARGS_TUPLE, std::size_t max_eval, std::size_t current_gen) = 0;
        virtual void add_tolerances (const std::vector<float>& IN_TOLERANCES) = 0;
    };

    template<class... constraint_func_types>
    struct constraint_manager final : constraint_manager_base {
        template <std::size_t i, class tuple_type> using tuple_args_type_at = std::decay_t<std::tuple_element_t<i, std::remove_reference_t<tuple_type>>>;
        template <class constraint_func_type> using return_type_t = std::invoke_result_t<std::decay_t<constraint_func_type>, args_type...>;

        std::vector<std::function<void(args_type...)>> vector_of_constraints;
        std::tuple<std::shared_ptr<return_type_t<constraint_func_types>>...> return_tuple{};
        std::tuple<return_type_t<constraint_func_types>...> constraint_values;
        std::vector<std::string>operators;
        std::vector<float>tolerances;

        template<class... value_types>
        explicit constraint_manager (constraint_func_types&&... IN_FUNCS, value_types&&... IN_VALUES) { //,
            static_assert((std::is_lvalue_reference_v<value_types> && ...));
            auto create = [this] <std::size_t... i, class...constraint_func_types_> (constraint_func_types&&... IN_FUNCS_, std::index_sequence<i...>) -> void {
                auto create_at = [this] <std::size_t i_, class constraint_func_type> (constraint_func_type&& FUNC) -> void {
                    using return_at = tuple_args_type_at<i_, std::tuple<return_type_t<constraint_func_types>...>>;
                    auto RETURN_POINTER = std::make_shared<return_at>();
                    std::get<i_>(return_tuple) = RETURN_POINTER;

                    auto constraint_at = [FUNC = std::forward<constraint_func_type>(FUNC), RETURN_POINTER] (args_type... args) -> void {
                        *RETURN_POINTER = FUNC(args...);
                    };

                    this->vector_of_constraints.push_back(constraint_at);
                };
                (create_at.template operator()<i>(std::forward<constraint_func_types>(IN_FUNCS_)),...);
            };
            create(std::forward<constraint_func_types>(IN_FUNCS)..., std::index_sequence_for<constraint_func_types...>{});
            this->add_constraint_values(std::forward<value_types>(IN_VALUES)...);
        }

        template<class... value_types>
        void add_constraint_values (value_types&&... IN_VALUES) {
            std::tuple<std::decay_t<std::remove_reference_t<value_types>>...> tuple = std::make_tuple(std::move(IN_VALUES)...);

            auto add = [this, &tuple] <std::size_t...i_> (std::index_sequence<i_...>) {
                auto add_at = [this, &tuple] <std::size_t i> () {
                    try {
                        if(std::is_same_v<tuple_args_type_at<i, decltype(this->constraint_values)>, tuple_args_type_at<i, std::tuple<value_types...>>>) {
                            std::get<i>(this->constraint_values) = std::move(std::get<i>(tuple));
                        } else throw std::runtime_error("Constraint values are not same as previously defined");
                    } catch (std::exception& e) {
                        std::cerr << e.what() << std::endl;
                        std::cerr << "Skipping initialisation of value.. defaulted to" << std::get<i>(this->constraint_values) << std::endl;
                    }
                };
                (add_at.template operator()<i_>(),...);
            };
            add.operator()(std::index_sequence_for<value_types...>{});
        }

        void add_operators (const std::vector<std::string> &IN_OPERATORS) override {
            const std::size_t size = sizeof...(constraint_func_types);
            try {
                if (size == IN_OPERATORS.size()) {
                    this->operators = IN_OPERATORS;
                } else throw std::runtime_error("Length of Operator vector does not match number of constraints");
            } catch (std::exception &e) {
                std::cerr << e.what() << std::endl;
                std::cerr << "Declaring all operator to '<='" << std::endl;
                this->operators.assign(size, "<=");
            }
        }

        void add_tolerances (const std::vector<float> &IN_TOLERANCES) override {
            const std::size_t size = sizeof...(constraint_func_types);
            try {
                if (size == IN_TOLERANCES.size()) {
                    this->tolerances = IN_TOLERANCES;
                } else throw std::runtime_error("Length of Operator vector does not match number of constraints");
            } catch (std::exception &e) {
                std::cerr << e.what() << std::endl;
                std::cerr << "Declaring all operator to '0.001f'" << std::endl;
                this->tolerances.assign(size, 0.001f);
            }
        }

        auto get_constraint_violation (const auto& IN_OBTAINED, const auto& IN_REQUIRED, const std::string& IN_OPERATOR, const auto& IN_TOLERANCE) {
            std::remove_reference_t<std::decay_t<decltype(IN_REQUIRED)>> diff = IN_OBTAINED - IN_REQUIRED;
            if (IN_OPERATOR == "<" && !(diff < 0) && std::abs(diff) > IN_TOLERANCE) return std::abs(diff);
            if (IN_OPERATOR == "<=" && !(diff <= 0) && std::abs(diff) > IN_TOLERANCE) return std::abs(diff);
            if (IN_OPERATOR == ">" && !(diff > 0) && std::abs(diff) > IN_TOLERANCE) return std::abs(diff);
            if (IN_OPERATOR == ">=" && !(diff >= 0) && std::abs(diff) > IN_TOLERANCE) return std::abs(diff);
            if (IN_OPERATOR == "=" && std::abs(diff) > IN_TOLERANCE) return std::abs(diff);
            if (IN_OPERATOR == "!=" && std::abs(diff) < IN_TOLERANCE) return std::numeric_limits<std::decay_t<decltype(IN_REQUIRED)>>::max();
            return decltype(IN_REQUIRED){};
        }

        void get_penalty (std::tuple<args_type...> &IN_ARGS_TUPLE, std::size_t max_eval, std::size_t current_gen) override {
            try {
                this->penalty = {};
                // const std::size_t number_of_constraints = this->vector_of_constraints.size();

                for (auto& each_constraint : this->vector_of_constraints) std::apply(each_constraint, IN_ARGS_TUPLE);   // always calls with lvalue

                auto get_penalty_ = [this] <std::size_t... i> (std::index_sequence<i...>) {
                    auto get_penalty_at = [this] <std::size_t i_> () {
                        auto obt_value_at = *(std::get<i_>(this->return_tuple));
                        auto req_value_at = std::get<i_>(this->constraint_values);
                        this->penalty += static_cast<return_type>(this->get_constraint_violation(obt_value_at, req_value_at, this->operators[i_], this->tolerances[i_]));
                    };
                    (get_penalty_at.template operator()<i>(),...);
                };
                get_penalty_.operator()(std::index_sequence_for<constraint_func_types...>{});
                // if (this->penalty > 0) VERBOSE_PRINT(this->penalty);
                this->penalty = static_cast<double>(current_gen) * 100000000000 * this->penalty * this->penalty;
                // if(this->penalty > 0) VERBOSE_PRINT("ONE OR MORE CONSTRAINTS VIOLATED. PENALTY of " << this->penalty << " introduced");
            } catch (std::exception &e) {
                std::cerr << "Error while calculating constraint penalty..." << e.what() << std::endl;
                std::cerr << "Ignoring constraints for current generation..." << std::endl;
                this->penalty = {};
            }
        }
    };

    template <class tuple_type>
    static void check_bounds (tuple_type &&IN_LOWER_BOUNDS, tuple_type &&IN_UPPER_BOUNDS) {

        auto are_lower_bounds_lower_than_upper = []<size_t... i_>(tuple_type &&lower_bounds_, tuple_type &&upper_bounds_, std::index_sequence<i_...>) -> bool {
            auto check_at = [] <std::size_t _i_> (tuple_type &&_lower_bounds_, tuple_type &&_upper_bounds_) -> bool {
                return std::get<_i_>(std::forward<tuple_type>(_lower_bounds_)) <= std::get<_i_>(std::forward<tuple_type>(_upper_bounds_));
            };
            return (check_at.template operator()<i_>
                (std::forward<tuple_type>(lower_bounds_), std::forward<tuple_type>(upper_bounds_))
                && ...);
        };

        if (is_tuple_default_initialized(IN_UPPER_BOUNDS) || is_tuple_default_initialized(IN_LOWER_BOUNDS)) {
            throw std::runtime_error("One of the Bounds are not initialsed. Use 'set_lower_bounds' or 'set_upper_bounds' to set bounds");
        }
        if (constexpr size_t tuple_size = std::tuple_size_v<std::remove_reference_t<tuple_type>>;
            !are_lower_bounds_lower_than_upper.operator()(std::forward<tuple_type>(IN_LOWER_BOUNDS), std::forward<tuple_type>(IN_UPPER_BOUNDS), std::make_index_sequence<tuple_size>{})) {
            throw std::runtime_error("One or more 'lower_bounds' are not lower than 'upper_bounds'. Use 'set_lower_bounds' or 'set_upper_bounds' to set bounds");
        }
    }

private:
    std::vector<std::tuple<args_type...>> chromosome_table;
    std::vector<return_type>graph;
    std::vector<return_type> fitness_table;
    std::function<return_type(args_type...)> objective_function;
    std::size_t number_of_variables = 0, population_size {};
    std::tuple<args_type...> lower_bounds{}, upper_bounds{};
    size_t max_eval = 10000;
    return_type search_space_reduction_fitness_tolerance;
    size_t max_no_improvement_count = 2000;
    float search_space_reduction_rate = 0.2;
    std::size_t search_space_reduction_eval{};
    bool minimise = true;
    bool search_space_reduction = false;
    float crossover_probability = 0.6f, mutation_probability = 0.4f;
    constraint_manager_base* constraint_manager_ = nullptr;    //intended to be used for "constraint_manager"
    bool constraints_on = false;
    std::size_t current_generation = 1;

    void generate_population () {   // try catch blocks
        try {
            generate_random_arthmetic randomiser = generate_random_arthmetic();
            auto update_tuple = [&randomiser, this]<std::size_t... indices> (std::tuple<args_type...>& current_chromosome,
                                                                             std::index_sequence<indices...>) -> void {
                auto insert_random_value = [&] <std::size_t index> () -> void {
                    std::get<index>(current_chromosome) = randomiser.get_random_value(std::get<index>(this->lower_bounds), std::get<index>(this->upper_bounds));
                };
                (insert_random_value.template operator()<indices>(), ...);
            };

            auto print_chromosome = [this] <std::size_t... i> (auto& chromosome, std::index_sequence<i...>) {
                auto print_at = [this] <std::size_t i_> (auto& chromosome) {
                    std::cout << std::get<i_>(chromosome);
                    if (i_ < this->population_size - 1) std::cout << ", ";
                };
                (print_at.template operator()<i>(chromosome),...);
            };

            std::size_t i = 0;
            for (auto& chromosome : this->chromosome_table) {
                update_tuple(chromosome, std::index_sequence_for<args_type...>{});
                this->fitness_table.at(i++) = this->get_fit_for_chromosome(chromosome);
                std::cout << "created chromosome " << i << " :";
                print_chromosome (chromosome, std::index_sequence_for<args_type...>{});
                std::cout << "fitness: " << this->fitness_table.at(i - 1) << std::endl;

            }
        } catch (std::exception& e) {
            std::cerr << "ERROR: " << e.what() << std::endl;
            throw;
        }
    }

    [[nodiscard]] return_type get_fit_for_chromosome (std::tuple<args_type...> &chromosome) {
        try {
            return_type fitness = std::apply(this->objective_function, chromosome);
            if (this->constraints_on) {
                this->constraint_manager_->get_penalty(chromosome, this->max_eval, this->current_generation);
                fitness += this->constraint_manager_->penalty;
            }
            return std::isnan(fitness) ? std::numeric_limits<return_type>::max() : fitness;

        } catch (std::exception &e) {
            std::cerr << "Error when calculating fit. " << e.what() << std::endl;
            throw;
        }
    }

    bool rank_chromosome_table () {
        auto get_standard_deviation = [this] () -> return_type {
            return_type deviation = 0;
            return_type mean = std::accumulate(
                this->fitness_table.begin(), this->fitness_table.end(), 0) / this->fitness_table.size();
            for (const return_type& fit : this->fitness_table) {
                const return_type difference = fit - mean;
                deviation += difference * difference;
            }
            deviation = deviation / this->fitness_table.size();
            return std::sqrt(deviation);
        };

        std::vector<std::size_t>indices(this->population_size, 0);
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(), [this](const std::size_t i, const std::size_t j)
            {return this->compare_two_fit_values(this->fitness_table[i], this->fitness_table[j]);});

        std::vector<return_type> new_fitness_table(this->population_size);
        std::vector<std::tuple<args_type...>> new_chromosom_table(this->population_size);
        auto new_fitness_it = new_fitness_table.begin();
        auto new_chromosom_it = new_chromosom_table.begin();
        for (std::size_t i = 0; i < this->population_size; ++i) {
            *new_fitness_it++ = std::move(this->fitness_table[indices[i]]);
            *new_chromosom_it++ = std::move(this->chromosome_table[indices[i]]);
        }

        std::swap(this->fitness_table, new_fitness_table);
        std::swap(this->chromosome_table, new_chromosom_table);

        return (get_standard_deviation() > 0.01);
    }

    void crossover (const std::size_t first_parent_index, const std::size_t second_parent_index,
                    const std::size_t first_child_index, const std::size_t second_child_index) {

        VERBOSE_PRINT("Performing CROSSOVER OPERATION...");
        generate_random_arthmetic randomisor;

        switch (randomisor.template get_random_value<std::size_t>(static_cast<std::size_t>(0), static_cast<std::size_t>(3))) {
            case 0: // one point crossover
            {
                VERBOSE_PRINT("Using ONE POINT CROSSOVER METHOD...");
                auto create_bool_vector = [] (const std::size_t size, const std::size_t position) -> std::vector<bool> {
                    std::vector<bool> result(size, false);
                    if (position < size) std::fill(result.begin() + position + 1, result.end(), true);
                    return result;
                };
                const std::size_t crossover_point = randomisor.template get_random_value<std::size_t>(static_cast<std::size_t>(0), this->number_of_variables - 2);
                std::vector<bool> where_swap = create_bool_vector(this->number_of_variables, crossover_point);

                this->chromosome_table[first_child_index] = this->chromosome_table[first_parent_index];
                this->chromosome_table[second_child_index] = this->chromosome_table[second_parent_index];

                this->swap_range_tuple(this->chromosome_table[first_child_index],
                                            this->chromosome_table[second_child_index], where_swap);
                break;
            }

            case 1: // multi point crossover
            {
                VERBOSE_PRINT("Using MULTI POINT CROSSOVER METHOD...");
                auto create_bool_vector = [] (const size_t vector_size, const std::vector<std::size_t> &positions) -> std::vector<bool> {
                    std::vector<bool> vector_(vector_size, false);
                    bool bool_ = false;
                    size_t flip_counter = 0;
                    for (std::size_t i = 0; i < vector_size; i++) {
                        if (positions[flip_counter] == i) {
                            bool_ = !bool_;
                            flip_counter++;
                        }
                        vector_[i] = bool_;
                    }
                    return vector_;
                };
                const std::size_t number_of_crossover_points = randomisor.template get_random_value<std::size_t>(static_cast<std::size_t>(1), this->number_of_variables - 1);
                std::vector<std::size_t> point_positions(number_of_crossover_points);
                randomisor.template fill_vector_with_unique_random<std::size_t>(point_positions, static_cast<std::size_t>(1),
                    static_cast<std::size_t>(this->number_of_variables - 1), &randomisor);
                std::ranges::sort(point_positions);
                std::vector<bool> where_swap = create_bool_vector(this->number_of_variables, point_positions);

                this->chromosome_table[first_child_index] = this->chromosome_table[first_parent_index];
                this->chromosome_table[second_child_index] = this->chromosome_table[second_parent_index];

                this->swap_range_tuple(this->chromosome_table[first_child_index], this->chromosome_table[second_child_index], where_swap);
                break;
            }

            case 2: // uniform crossover
            {
                VERBOSE_PRINT("Using UNIFORM CROSSOVER METHOD...");
                this->chromosome_table[first_child_index] = this->chromosome_table[first_parent_index];
                this->chromosome_table[second_child_index] = this->chromosome_table[second_parent_index];

                std::vector<bool>where_swap(this->number_of_variables, false);
                randomisor.fill_bool_vector_with_random(where_swap, &randomisor);

                this->swap_range_tuple(this->chromosome_table[first_child_index], this->chromosome_table[second_child_index], where_swap);
                break;
            }

            case 3: //whole arthmetic recombination error
            {
                VERBOSE_PRINT("Using WHOLE ARTHMETIC RECOMBINATION CROSSOVER METHOD...");
                auto perform = [] <std::size_t... i> (std::tuple<args_type...> &first_tuple,
                    std::tuple<args_type...> &second_tuple, const float weight, std::index_sequence<i...>) -> void {

                    auto perform_at = [] <std::size_t i_> (std::tuple<args_type...> &first_tuple_,
                        std::tuple<args_type...> &second_tuple_, const float weight_) -> void {

                        if constexpr (std::is_arithmetic_v<std::tuple_element_t<i_, std::tuple<args_type...>>>) {
                            //first_child = weight.first_parent + (1-weight).second_parent    (first_child is put in the place of first_parent)
                            //second_child = (1-weight).first_parent + weight.second_parent   (second_child is put in the place of second_parent)
                            std::exchange(std::get<i_>(second_tuple_),
                                std::exchange(std::get<i_>(first_tuple_),
                                       std::get<i_>(first_tuple_) * weight_ + std::get<i_>(second_tuple_) * (1.0f - weight_))
                                    * (1.0f - weight_) + std::get<i_>(second_tuple_) * weight_);
                        }
                    };
                    (perform_at.template operator()<i>(first_tuple, second_tuple, weight),...);
                };

                this->chromosome_table[first_child_index] = this->chromosome_table[first_parent_index];
                this->chromosome_table[second_child_index] = this->chromosome_table[second_parent_index];
                const float weight_to_first = randomisor.template get_random_value<float>(0.0f, 1.0f);
                perform(this->chromosome_table[first_child_index], this->chromosome_table[second_child_index],
                                                    weight_to_first, std::index_sequence_for<args_type...>{});
                break;
            }

            default:
                break;
        }
    }

    void mutation (const size_t parent_index, const size_t child_index) {
        VERBOSE_PRINT("Performing MUTATION...");
        generate_random_arthmetic randomiser = generate_random_arthmetic();
        std::vector<bool>positions_of_mutation(this->number_of_variables, false);
        std::generate(positions_of_mutation.begin(), positions_of_mutation.end(), [&randomiser] {
            const float bias = randomiser.template get_random_value<float>(0.0f, 1.0f);
            return randomiser.template get_random_value<bool>(false, true, bias);
        });

        this->chromosome_table[child_index] = this->chromosome_table[parent_index];
        auto mutate = [this, &positions_of_mutation, &randomiser, child_index] <std::size_t... i> (std::index_sequence<i...>) {
            auto mutate_at = [&] <std::size_t i_> () -> void {
                if (positions_of_mutation[i_] == true &&
                    std::is_arithmetic_v<std::tuple_element_t<i_, std::tuple<args_type...>>>) {
                    std::get<i_>(this->chromosome_table[child_index]) = randomiser.get_random_value(std::get<i_>(this->lower_bounds),
                        std::get<i_>(this->upper_bounds));
                }
            };
            (mutate_at.template operator()<i>(),...);
        };
        mutate.operator()(std::index_sequence_for<args_type...>{});
    }

    std::size_t parent_selector (const bool IN_USE_TOURNAMENT = true) {
        //assuming the tables are sorted...
        VERBOSE_PRINT("SELECTING PARENTS for genetic operations...");
        if (IN_USE_TOURNAMENT) {   // Tournament Selection Method
            VERBOSE_PRINT("TOURNAMENT SELECTION METHOD selected for index parents...");
            const std::size_t k = this->population_size / 10;

            generate_random_arthmetic randomiser = generate_random_arthmetic();
            std::vector<std::size_t>parent_indices(this->population_size);
            std::iota(parent_indices.begin(), parent_indices.end(), 0);
            std::shuffle(parent_indices.begin(), parent_indices.end(), randomiser.gen);

            std::sort(parent_indices.begin(), parent_indices.begin() + k,
                [this](const std::size_t i, const std::size_t j) -> bool {return this->fitness_table[i] < this->fitness_table[j];});

            VERBOSE_PRINT("SELECTED PARENTS: " << parent_indices[0] << " and " << parent_indices[1]);
            return parent_indices[0];
        }
        VERBOSE_PRINT("Encountering LOW standard DEVIATION in FITNESS TABLE...");
        VERBOSE_PRINT("Selected Parents: " << 0 << " and " << 1);
        return 0;
    }

    static void swap_range_tuple (std::tuple<args_type...> &first_tuple, std::tuple<args_type...> &second_tuple, std::vector<bool> &where_swap) {
        auto swap = [&first_tuple, &second_tuple, &where_swap]<std::size_t... i>(std::index_sequence<i...>) {
            auto swap_at = [&]<std::size_t i_> () {
                if(where_swap[i_] == true) {
                    std::swap(std::get<i_>(first_tuple), std::get<i_>(second_tuple));
                }
            };
            (swap_at.template operator()<i>(),...);
        };
        swap.operator()(std::index_sequence_for<args_type...>{});
    }

    bool survivor_selector (const bool IN_STANDARD_DEVIATION) {
        // assuming the table is ranked
        VERBOSE_PRINT("Performing SURVIVOR SELECTION....");
        const std::size_t first_parent_index = this->parent_selector(IN_STANDARD_DEVIATION);
        const std::size_t second_parent_index = this->parent_selector(IN_STANDARD_DEVIATION);
        const size_t first_child_index = this->population_size - 1, second_child_index = this->population_size - 2;
        VERBOSE_PRINT("SELECTED least fit chromosomes to be replaced by CHILDREN");
        bool crossover_mutation_;
        {
            generate_random_arthmetic randomiser = generate_random_arthmetic();
            crossover_mutation_ = randomiser.template get_random_value<bool>(false, true, this->crossover_probability);
        }

        if (crossover_mutation_ == true) { //crossover
            this->crossover(first_parent_index, second_parent_index, first_child_index, second_child_index);

            VERBOSE_PRINT("LEAST FIT CHROMOSOMES DELETED with fitness: " << this->fitness_table[first_child_index] << " and " << this->fitness_table[second_child_index] << " fit at 0: " << this->fitness_table[0]);
            this->fitness_table[first_child_index] = this->get_fit_for_chromosome(this->chromosome_table[first_child_index]);
            this->fitness_table[second_child_index] = this->get_fit_for_chromosome(this->chromosome_table[second_child_index]);
            VERBOSE_PRINT("CHILD CHROMOSOMES CREATED with fitness: " << this->fitness_table[first_child_index] << " and " << this->fitness_table[second_child_index]);

            return this->compare_two_fit_values(this->fitness_table[0], this->fitness_table[first_child_index])
                || this->compare_two_fit_values(this->fitness_table[0], this->fitness_table[second_child_index]);
        } // mutation
        this->mutation(first_parent_index, first_child_index);

        VERBOSE_PRINT("LEAST FIT CHROMOSOMES DELETED with fitness: " << this->fitness_table[first_child_index] << " fit at 0: " << this->fitness_table[0]);
        this->fitness_table[first_child_index] = this->get_fit_for_chromosome(this->chromosome_table[first_child_index]);
        VERBOSE_PRINT("CHILD CHROMOSOMES CREATED with fitness: " << this->fitness_table[first_child_index]);

        return this->compare_two_fit_values(this->fitness_table[0], this->fitness_table[first_child_index]);
    }

    bool compare_two_fit_values (return_type fit_main, return_type fit_should_be_less_fit) {
        if (this->minimise) return (fit_main < fit_should_be_less_fit);
        return  fit_main > fit_should_be_less_fit;
    }

    void hypercube_reduction () {
        auto reduce = [this] <std::size_t... i> (std::index_sequence<i...>) {
            auto reduce_at = [this] <std::size_t i_> () {
                if constexpr (std::is_arithmetic_v<std::tuple_element_t<i_, std::tuple<args_type...>>>) {
                    std::get<i_>(this->lower_bounds) +=
                        this->search_space_reduction_rate * (std::get<i_>(this->optimal_point) - std::get<i_>(this->lower_bounds));
                    std::get<i_>(this->upper_bounds) -=
                        this->search_space_reduction_rate * (std::get<i_>(this->upper_bounds) - std::get<i_>(this->optimal_point));
                }
            };
            (reduce_at.template operator()<i>(),...);
        };
        reduce.operator()(std::index_sequence_for<args_type...>{});
    }
};

#endif //GENETIC_OPTIMISATION_H
