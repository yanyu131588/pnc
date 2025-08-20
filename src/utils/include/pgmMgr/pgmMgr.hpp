#ifndef PGM_MGR_HPP_
#define PGM_MGR_HPP_


#include <vector>
#include <string>


typedef struct pgmDef
{
    int width_;
    int height_;
    int maxGrayVal_;
    std::vector<unsigned char> mapData_;
    pgmDef() : width_(0), height_(0), maxGrayVal_(0), mapData_() {}
} pgmDef;

bool readPGM(const std::string& filename, pgmDef& image);

#endif