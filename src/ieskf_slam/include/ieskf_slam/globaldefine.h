#pragma once
#include <algorithm>
#include <string>

#ifndef PROJ_DIR
#define PROJ_DIR " "
#endif

const std::string WORKD_DIR =  PROJ_DIR;
const std::string CONFIG_DIR = WORKD_DIR+"/config/";
const std::string RESULT_DIR = WORKD_DIR + "/result/";

/// Strip directory separators and path-traversal sequences from a file name
/// so that it can be safely concatenated with a trusted base directory.
inline std::string sanitizeFileName(const std::string& name) {
    // Find last separator to extract the basename.
    auto pos = name.find_last_of("/\\");
    std::string base = (pos == std::string::npos) ? name : name.substr(pos + 1);
    // Reject names that are empty or consist solely of dots (e.g. ".", "..").
    if (base.empty() || base.find_first_not_of('.') == std::string::npos) {
        return "untrusted_file";
    }
    return base;
}  