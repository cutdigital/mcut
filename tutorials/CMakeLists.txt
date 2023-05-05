#
# Copyright (c) 2021-2022 Floyd M. Chitalu.
# All rights reserved.
# 
# NOTE: This file is licensed under GPL-3.0-or-later (default). 
# A commercial license can be purchased from Floyd M. Chitalu. 
#  
# License details:
# 
# (A)  GNU General Public License ("GPL"); a copy of which you should have 
#      recieved with this file.
# 	    - see also: <http://www.gnu.org/licenses/>
# (B)  Commercial license.
#      - email: floyd.m.chitalu@gmail.com
# 
# The commercial license options is for users that wish to use MCUT in 
# their products for comercial purposes but do not wish to release their 
# software products under the GPL license. 
# 
# Author(s)     : Floyd M. Chitalu
#

cmake_minimum_required(VERSION 3.1)
project(mcut_tutorials)

#add_subdirectory(BasicGuiApp)
add_subdirectory(BasicCmdLineApp)
add_subdirectory(HelloWorld)
add_subdirectory(TextureCoordinates)
add_subdirectory(VertexNormals)
add_subdirectory(FaceNormals)
add_subdirectory(CSGBoolean)
add_subdirectory(AdjacentFaces)
add_subdirectory(Triangulation)
add_subdirectory(TriangulatedFaceMaps)
add_subdirectory(EventCallback)
add_subdirectory(MultipleContextsInParallel)

