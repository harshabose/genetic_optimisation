# genetic_optimisation C++ header file
**A versatile genetic optimisation tool for engineering design optimisation.**

The motivation to create a custom genetic optimisation tool is that many public optimisation libraries, such as NLOPT, lack the ability to handle
high number of optimisation variables and different data types, making them sub-optimal for engineering design optimisation.

This header file allows for practically limitless number of optimisation variables with different data type and supports unlimited linear and
non-linear equality and inequality constraints, ideal for engineering design optimisation.

The header file is located at [include/genetic_optimisation.h](https://github.com/harshabose/genetic_optimisation/blob/main/include/genetic_optimisation.h). Just copy paste the file into your directory and include in your code. Simple as that.

The tool was tested on several benchmark optimsation test functions including rastrigin function and ackley function and resulted in accurate converge within few thousand iterations with tolerance 0.00001.
The tool was also tested on conceptual optimisation of propulsion system configuration and propeller design on eVTOL in climb. The results are as below.



To facilitate the above engineering design optimisation problem, several other tools were developed which heavily rely on public libraries and tools such as pybind11 and aerospacesandbox. So do not expect other files to work on your setup. 

