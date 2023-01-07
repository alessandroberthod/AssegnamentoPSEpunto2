#ifndef COORDINATE_H
#define COORDINATE_H

#include<ostream>
#include<vector>

#include "Obstacle.h"


class Coordinate{

public:
    //costruttore e costruttore di default
    Coordinate(double xC, double yC);
    Coordinate();

    //funzioni membro, che non modificano l'oggetto
    double xCoord() const { return xC; };
    double yCoord() const { return yC; };
    double distance_btw_two_coords (const Coordinate& c) const;
    double distance_currentrobotcoord_goalrobotcoord(const Coordinate& cgoal) const;
    double min_distance_currentrobotcoord_one_obstacle_coords(double dimGrid, const Obstacle& obst) const;
    double min_distance_currentrobotcoord_all_obstacles_coords(double dimGrid, const std::vector<Obstacle>& vecobst) const;
    double potential_tot_btw_currentgoalrobcoords_currentrobobstcoords(double _eta, double _zeta, double _max_dist_infl, double dimGrid, const Coordinate& cgoal, const std::vector<Obstacle>& vecobst_pot) const;
    Coordinate path_planning_robot(double _eta, double _zeta, double _max_dist_infl, double dimGrid, const Coordinate& cgoal, const std::vector<Obstacle>& vecobst_pp) const;
    Coordinate coords_cell_of_particular_position(double dimGrid) const;

private:
    double xC, yC;


};


Coordinate operator - (const Coordinate& c1, const Coordinate & c2);

#endif
