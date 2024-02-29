# genetic_optimisation C++ header file
**A versatile genetic optimisation tool for engineering design optimisation.**

Introducing an adaptable genetic optimization tool tailored for engineering design optimization challenges. The need for a custom solution arises from the limitations of existing public optimization libraries, like NLOPT, which struggle when dealing with a high number of variables and diverse data types, making them less effective for engineering design optimization.

THe header file, overcomes these challenges. It empowers you with the ability to handle any number of optimization variables, each with different data types. Moreover, it seamlessly accommodates an unlimited number of linear and non-linear equality and inequality constraints – a perfect fit for engineering design optimization scenarios. The header file employes advanced C++ features and techniques such as template metaprogramming and compilte-time delegation to optimise the workflow allowing for high performance with minimal memory overhead.

The implementation is user-friendly – just copy and paste the file into your project directory, and include it in your code.

The tool has undergone rigorous testing on various benchmark optimization test functions, including the Rastrigin and Ackley functions with accurate convergence achieved within a few hundred iterations, with a tolerance of 0.00001.

Additionally, the tool has been applied to real-world engineering challenges - conceptual optimization tasks of propulsion system configuration and propeller design of eVTOL in climb segment. ~~The outcomes are documented for your reference.~~ (document removed for condidentiality, the [propulsion system analysis](https://github.com/harshabose/propulsion_system_analysis) tool could be found in the link and a test case is given below)

[https://github.com/harshabose/genetic_optimisation/blob/b20ad53db3ee4b89fa86efee5ebad1a261825045/Test%20Propeller%20Genetic%20Optimisation.png]


In future version, I will be integrating my [lightweight C++ thread manager](https://github.com/harshabose/thread_manager) tool to genetic optimisation allowing for even more performance and speed.

