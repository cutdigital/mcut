Thank you for purchasing MCUT. This provided package contains development files 
which you can use to work with MCUT.

There are two types of files, which you need - headers and libraries. 

There are exactly two header files, which you must include as part of your C 
or C++ application software for developing with MCUT. These header files 
can be found in the "include" directory. 

The libraries are distributed as shared (.so/.dll) and static/archive (.a/.lib)
files, which you must link against to build your application using MCUT. These
library files are also categorised as either "exact" or "fp". Exact library 
files are implemented with arbitrary precision arithmetic in place of standard
(mchine precision) floating point values. Conversely The "fp" library files
are implemented with standard floating point arithmetic. All library files can 
be found in the "lib" directory.

CutDigital provides two categories of MCUT implementations (libraries), allowing
users the ability to choose. The arbitrary precision arithmetic implementation 
(lib/exact) improves the robustness of geometric predicates against rounding error 
but have the disadvantage of being slow. On the other hand, the floating point 
arithmetic libraries (lib/fp) are faster but more susceptible to numerical error. 
Both options are generally robust.   

For more information, visit the project page: https://cutdigital.github.io/

===============================================================================

A SUMMARY OF HOW TO USE MCUT IN YOUR PROJECT

This is what you must do in your C or C++ project:

1. Include the file "include/mcut/mcut.h"
2. Link against one of the provided library files.

For more information on using the API, refer to the documentation and examples.
    - documentation: https://github.com/cutdigital/mcut-docs/
    - examples:
        * tutorials: https://cutdigital.github.io/