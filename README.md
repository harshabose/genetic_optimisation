# genetic_optimisation C++ header file
**A versatile genetic optimisation tool for engineering design optimisation.**

Introducing an adaptable genetic optimization tool tailored for engineering design optimization challenges. The need for a custom solution arises from the limitations of existing public optimization libraries, like NLOPT, which struggle when dealing with a high number of variables and diverse data types, making them less effective for engineering design optimization.

My header file, located at [include/genetic_optimisation.h](https://github.com/harshabose/genetic_optimisation/blob/main/include/genetic_optimisation.h), overcomes these challenges. It empowers you with the ability to handle any number of optimization variables, each with different data types. Moreover, it seamlessly accommodates an unlimited number of linear and non-linear equality and inequality constraints – a perfect fit for engineering design optimization scenarios. The header file employes advanced C++ features and techniques such as template metaprogramming and compilte-time delegation to optimise the workflow allowing for high performance with minimal memory overhead.

The implementation is user-friendly – just copy and paste the file into your project directory, and include it in your code.

The tool has undergone rigorous testing on various benchmark optimization test functions, including the Rastrigin and Ackley functions with accurate convergence achieved within a few hundred iterations, with a tolerance of 0.00001.

Additionally, the tool has been applied to real-world engineering challenges - conceptual optimization tasks of propulsion system configuration and propeller design of eVTOL in climb segment. The outcomes are documented for your reference.

To perform the above mentioned engineering design optimization tests, several supporting tools were developed. Note that these auxiliary files heavily rely on popular public libraries and tools like python, pybind11 and aerospacesandbox. As a result, compatibility may vary depending on your setup.

In future version, I will be integrating my [lightweight C++ thread manager](https://github.com/harshabose/thread_manager) tool to genetic optimisation allowing for even more performance and speed.

