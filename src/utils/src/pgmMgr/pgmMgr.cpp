#include "pgmMgr/pgmMgr.hpp"

#include <fstream>
#include <sstream>
#include <limits>

#include <iostream>

bool readPGM(const std::string& filename, pgmDef& image)
{
    std::ifstream file(filename, std::ios::binary);

    if (!file) {
        std::cerr << "Error: Could not open file!" << std::endl;
        return 1;
    }

    char magicNumber[3];
    file.read(magicNumber, 2);

    if (std::string(magicNumber, 2) != "P5" && std::string(magicNumber, 2) != "P2") {
        std::cerr << "Error: Invalid magic number, not a PGM file!" << std::endl;
        file.close();
        return false;
    }

    char whitespace;
    while (file.peek() == ' ' || file.peek() == '\t' || file.peek() == '\n' || file.peek() == '\r'|| file.peek() == '\f') {
        file.get(whitespace);
    }

    std::string line;
    while (file.peek() == '#') {
        std::getline(file, line);
    }

    file >> image.width_;
    file.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
    file >> image.height_;
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); 
    file >> image.maxGrayVal_;
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); 

    image.mapData_.resize(image.width_ * image.height_ * sizeof(unsigned char));
    if (std::string(magicNumber, 2) == "P5") {
       
        file.read(reinterpret_cast<char*>(image.mapData_.data()), image.mapData_.size());
    } 

    file.close();    

    return true;
}