#include "off.h"
#include <string>
#include <cstdlib>
#include <fstream>

void writeOBJ(const std::string& fname, const std::vector<float>& vertices, const std::vector<uint32_t>& triangles)
{
    printf("writeOBJ: %s\n", fname.c_str());

    std::ofstream file(fname);

    for(uint32_t i =0; i < (uint32_t)vertices.size(); i+=3)
    {
        file << "v " << vertices[i+0]<< " " << vertices[i+1] << " " <<  vertices[i+2]<< "\n"; 
    }

    for(uint32_t i =0; i < (uint32_t)triangles.size(); i+=3)
    {
        file << "f " << triangles[i+0]+1<< " " << triangles[i+1]+1 << " " <<  triangles[i+2]+1<< "\n"; 
    }

    file.close();
}


void MCAPI_PTR mcDebugOutput(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam)
{
    
    //printf("Debug message ( %d ), length=%zu\n%s\n--\n", id, length, message);
    //printf("userParam=%p\n", userParam);

    std::string debug_src;
    switch (source) {
    case MC_DEBUG_SOURCE_API:
        debug_src = "API";
        break;
    case MC_DEBUG_SOURCE_KERNEL:
        debug_src = "KERNEL";
        break;
    case MC_DEBUG_SOURCE_ALL:
        break;
    }
    std::string debug_type;
    switch (type) {
    case MC_DEBUG_TYPE_ERROR:
        debug_type = "ERROR";
        break;
    case MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
         debug_type = "DEPRECATION";
        break;
    case MC_DEBUG_TYPE_OTHER:
        //printf("Type: Other");
        debug_type = "OTHER";
        break;
    case MC_DEBUG_TYPE_ALL:
        break;
       
    }

    std::string severity_str;

    switch (severity) {
    case MC_DEBUG_SEVERITY_HIGH:
        severity_str = "HIGH";
        break;
    case MC_DEBUG_SEVERITY_MEDIUM:
        severity_str = "MEDIUM";
        break;
    case MC_DEBUG_SEVERITY_LOW:
        severity_str = "LOW";
        break;
    case MC_DEBUG_SEVERITY_NOTIFICATION:
        severity_str = "NOTIFICATION";
        break;
    case MC_DEBUG_SEVERITY_ALL:
        break;
    }

    printf("MCUT[%d:%p,%s:%s:%s:%zu] %s\n", id, userParam, debug_src.c_str(), debug_type.c_str(),severity_str.c_str(), length, message);
}