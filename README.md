# [M]esh [CUT]ting library (MCUT) #

_A simple & robust mesh cutting tool._ 

MCUT is a  library for _cutting_ surface meshes. It is a simple and versatile tool, which can be used for operations like mesh _slicing_, _boolean operations_ (CSG), _stencilling_ and more! . 

Project homepage:    TODO

Copyright (c) 2020- CutDigital Ltd

# Building the library

* Open terminal
* Change directory to your preferred folder 
    - `cd <path/to/preferred/folder>` 
* Download the MCUT library 
    - `git clone https://github.com/cutdigital/mcut.git`
    - Enter your credentials when prompted
* Download the public MCUT headers
    - `git clone https://github.com/cutdigital/mcut-headers.git`
* Change directory into the cloned MCUT folder. 
    - `cd mcut`
* Create and enter the `build` folder (you can also use a different but unused name)
    - `mkdir build && cd build`
* Configure the build system using [CMake](https://cmake.org/)
    - `cmake -DMCUT_INCLUDE_DIR:STRING="<path/to/mcut/headers>" ..`

# Integrating MCUT into your CMake project

For a list of CMake variables (e.g. MCUT target library path), see the topmatter of `mcut/CMakeLists.txt`.

The public headers are provided separately from this project. They can be downloaded [here](https://github.com/cutdigital/mcut-headers.git).