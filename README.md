# What is this?

_A simple and fast C++ library for mesh booleans and more..._ 

[![Windows](https://github.com/cutdigital/mcut/actions/workflows/windows.yml/badge.svg)](https://github.com/cutdigital/mcut/actions/workflows/windows.yml)
[![MacOS](https://github.com/cutdigital/mcut/actions/workflows/macos.yml/badge.svg)](https://github.com/cutdigital/mcut/actions/workflows/macos.yml) [![Linux](https://github.com/cutdigital/mcut/actions/workflows/linux.yaml/badge.svg)](https://github.com/cutdigital/mcut/actions/workflows/linux.yaml)

This project is designed for a broad range of real-world problems relating to structural-design tasks. Application areas include aerospace and automotive engineering, mining, civil and mechanical engineering amongst others. 

The software solution is called "MCUT", and ut provides functionality to perform fast and robust geometry operations, as shown below (images rendered with Blender):

<p align="center">
  <img src="https://github.com/cutdigital/mcut.github.io/blob/master/docs/media/repo-teaser/github-teaser.png?raw=true">
</p>

Figure 1: Generate, slice and perform Booleans without errors. Competitive with the best solids and polygon Boolean tools on the market (for example, [Deswik.CAD](https://www.deswik.com/product-detail/deswik-cad/)).

The repo provides a comprehensive tool for ensuring that computer-aided planning tasks for e.g. mine-design, rock strata boring (e.g. underground-tunnel excavations), oil-well drilling and general 3D modelling for animation are achievable with robustness. The tool is developed to take advantage of modern high-performance parallel computing hardware for you, and is demonstrably robust by using precise geometric algorithms that are implemented in C++ and accessed through an intuitive API that resembles the all-familiar C programming language.

## Capabilities

MCUT is a tool for partitioning meshes that represent solids or open surfaces: It is a code library for cutting 3D mesh objects using their geometry to produce crisp fragments at fine scale, which is useful for operations like slicing and boolean operations (union, subtraction and intersection). Supported features include (see images below):

* stencilling: exact cut-outs of the cutting surface
* intersection curve access: geometry representing lines of intersection-contour points
* partial cuts: producing valid results where an open-surface (sheet of connected polygons) is not necessarily completely cutting through a solid.
* concatenation: merging a solids or open-surfaces with another.
* sectioning: elimination of material/volume on one side of a specified surface (e.g. a plane) 
* splitting: partitioning one mesh using another that might be open (a sheet of polygons) or solid (connected set of watertight polygons). 

What is being offered is a general solution to the problem of resolving solid- and surface-mesh intersections. It is a solution that is sought by many companies and individuals for its ability to address extremely difficult problems relating to computational geometry in 3D. A classic application is constructive solid geometry (CSG) i.e. the “boolean operation”, which is shown below, where the resulting meshes/objects are produced with MCUT:

<p align="center">
  <img src="https://github.com/cutdigital/mcut.github.io/blob/master/docs/media/repo-teaser/teaser2.png?raw=true">
</p>

Figure 2: Generate solids / polygons using a robust Boolean engine, where other technologies fail, MCUT solids will be valid.

## The practical benefits users

The expert capabilities of MCUT will allow companies, individuals and researchers-alike to develop robust (and fast) Computer-Aided Design (CAD) and Manufacturing (CAM) tools. For example, these tools could cater to the design of industry-specific structural models like tunnels, drill holes, mechanical instruments and rock-block models. All this alongside the ability to handle general 3D modelling tasks that are typical in industries and academic-fields related to computer graphics (e.g. game-engine level design) and mechanical engineering (e.g. fracture simulation). In essence, users of MCUT are provided with the capability to create robust derivative products for generating (and testing) structural designs in a virtual setting for short- and long-term production operations and feasibility tests/studies.

The following images show more examples of what users can achieve with MCUT:

<p align="center">
  <img src="https://github.com/cutdigital/mcut.github.io/blob/master/docs/media/repo-teaser/extra-images/eg-teaser.jpg?raw=true">
</p>

Figure 3: Fracture simulation using the [Extended Finite Element Method (XFEM)](https://onlinelibrary.wiley.com/doi/abs/10.1111/cgf.13953), where MCUT is used to create fragment geometry by intersecting the simulation domain with propagated cracks.

<p align="center">
  <img src="https://github.com/cutdigital/mcut.github.io/blob/master/docs/media/repo-teaser/extra-images/image156.png?raw=true">
</p>

Figure 4: Intersecting a gear cog with a surface to model the fracturing of steel.

<p align="center">
  <img src="https://github.com/cutdigital/mcut.github.io/blob/master/docs/media/repo-teaser/extra-images/path1471.png?raw=true">
</p>

Figure 5: Merging an engine with the axle shaft to model their connectivity.

<p align="center">
  <img src="https://github.com/cutdigital/mcut.github.io/blob/master/docs/media/repo-teaser/extra-images/arm-sphere.png?raw=true">
</p>

Figure 6: Intersecting a hand model and a sphere, showing how MCUT can also be useful for planning and designing molding processes for e.g. 3D printing

<p align="center">
  <img src="https://github.com/cutdigital/mcut.github.io/blob/master/docs/media/repo-teaser/extra-images/arma-bunn.png?raw=true">
</p>

Figure 8: Assorted results produced by intersecting the Stanford bunny model and armadillo.

<p align="center">
  <img src="https://github.com/cutdigital/mcut.github.io/blob/master/docs/media/repo-teaser/extra-images/path1471-2.png?raw=true">
</p>
Figure 9: Tunnel excavation of a mountainous terrain for modelling underground construction with a boring machine (represented with cylinder). Note how the input meshes need not be solids.

<p align="center">
  <img src="https://github.com/cutdigital/mcut.github.io/blob/master/docs/media/repo-teaser/extra-images/image111.png?raw=true">
</p>

Figure 10: Creating an Open-Pit mine model on a rough terrain for e.g. pre-planning operations.

<p align="center">
  <img src="https://github.com/cutdigital/mcut.github.io/blob/master/docs/media/repo-teaser/extra-images/path1471-5.png?raw=true">
</p>
Figure 11: An example of sectioning with a flat plane, which can be used to eliminate material/volume on either side of this plane or create hollow carve-outs.

## Source code and test applications

The source code is available for your perusal and evaluation. You can access right here. This is a trusted opportunity for you to trial and experiment with MCUT for you needs and requirments.

* `git clone https://github.com/cutdigital/mcut.git` 
* `mkdir build`
* `cd build`
* `cmake ..` (see `CMakeLists.txt` for available build configuration options) 
* run `make -j4` *IF* you are on Linux/MacOS terminal, *ELSE* open the generated `.sln` with e.g. Visual Studio

Next, try out one of the tutorials!

## Licensing

MCUT is available under an Open Source license as well as a commercial license. Users choosing to use MCUT under the free-of-charge Open Source license (e.g. for academic purposes) simply need to comply to its terms, otherwise a commercial license is required. The Open Source license is the "GNU General Public License" (GPL). In cases where the constraints of the Open source license prevent you from using MCUT, a commercial license can be purchased. The library is licensed with an attractively low price which is a one-off sum, requiring no further loyalty fees with guarranteed future updates for free.   

_For more information about this library, you can refer to the project page [here](https://cutdigital.github.io/mcut.site/) or [email](floyd.m.chitalu@gmail.com) the author_.

|:warning: MCUT may not be sold or included in commercial products without a license. |
| --- |
