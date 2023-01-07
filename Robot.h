#ifndef ROBOT_H
#define ROBOT_H

#include <ostream>
#include "Coordinate.h"
#include "Obstacle.h"

class Robot {

public:
    //costruttore e costruttore di default
    Robot(double xRinitial, double yRinitial, double xRcurrent, double yRcurrent, double xRgoal, double yRgoal);
    Robot();

    //funzioni membro, che non modificano l'oggetto
    double pos_xRinitial() const { return xRinitial; };
    double pos_yRinitial() const {return yRinitial; };
    double pos_xRcurrent() const { return xRcurrent; };
    double pos_yRcurrent() const { return yRcurrent; };
    double pos_xRgoal() const { return xRgoal; };
    double pos_yRgoal() const { return yRgoal; };

    //funzioni membro, che modificano l'oggetto
    Coordinate curr_rob_lower_left_corner_cell_coord(double dimGrid);
    Coordinate goal_rob_lower_left_corner_cell_coord(double dimGrid);
    void move_robot_to_goal(double _eta, double _zeta, double _max_dist_infl, double dimGrid, const std::vector<Obstacle>& vecobst_pp);
    void update_robot_to_new_sample_goalcoords(const Coordinate& coordsgoal);
    void adapt_final_currrobcoords_to_goalcoords(double dimGrid, const Coordinate& finalcurrcords, const Coordinate& goalcoords);

private:
    double xRinitial, yRinitial, xRcurrent, yRcurrent, xRgoal, yRgoal;
    //current non actual!

};




std::ostream& operator<<(std::ostream& os, const Robot& rob_os);

#endif
