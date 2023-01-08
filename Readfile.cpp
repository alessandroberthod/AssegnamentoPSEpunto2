#include <fstream>
#include <iostream>
#include <vector>
using std::vector;
#include <string>
using std::string;

#include "Coordinate.h"



vector<Coordinate> read_from_file(const string& filename) {

    vector<Coordinate> coords_from_text;

    // std::ifstream encapsulates the functionality of an 
    // input file stream as a class, is created by
    // providing a filename as its constructor argument
    std::ifstream infile{filename};

    while (!infile.eof()) {
        double x, y;
        infile >> x >> y;
        if (infile.eof() || infile.fail() || infile.bad()) {
            std::cerr << "Error in input or eof  \n";
            break;
        }
        coords_from_text.push_back({x,y});
        
    }

    return coords_from_text;

}
