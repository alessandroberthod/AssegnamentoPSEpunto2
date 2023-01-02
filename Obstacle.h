#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <ostream> // se necessario stampare gli ostacoli
#include <vector>
//#include "Robot.h"
//#include "Coordinate.h"
using std::vector;

class Coordinate; //forward declaration della classe Coordinate per poter impiegare tipo Coordinate nelle fz membro

class Obstacle {
public:
    //costruttore e costruttore di default
    Obstacle (double x1, double y1, double x2, double y2);
    Obstacle();
    
    //funzioni membro, che non modificano l'oggetto
    double x1o() const { return x1;};
    double y1o() const {return y1;};
    double x2o() const {return x2;};
    double y2o() const {return y2;};
    vector<Coordinate> outline_obstacle_coordinates(double dimGrid) const;

    //funzioni membro, che modificano l'oggetto
    void adapt_obstacle_to_grid(double dimGrid);
    

private:
    double x1, y1, x2, y2;
};



std::ostream& operator<<(std::ostream& os, const Obstacle& obs_os);

#endif