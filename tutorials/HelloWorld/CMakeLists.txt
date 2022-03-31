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

#get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
#project(${PROJECT_NAME})

add_executable(HelloWorld HelloWorld.cpp)
target_link_libraries(HelloWorld mcut ${extra_libs})
target_include_directories(HelloWorld PRIVATE ${MCUT_INCLUDE_DIR})
target_compile_options(HelloWorld PRIVATE ${compilation_flags})
target_compile_definitions(HelloWorld PRIVATE ${preprocessor_defs})