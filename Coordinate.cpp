#include <cmath>
#include <algorithm>
using std::min_element;
#include <vector>
using std::vector;

#include "Coordinate.h"
#include "Robot.h"
#include "Obstacle.h"




//Costruttore coordinata
Coordinate::Coordinate(double xCe, double yCe)
    : xC{xCe}, yC{yCe}
{
}
//Costruttore di default coordinata
const Coordinate& default_coordinate()
{
	static Coordinate ct {0,0};
	return ct;
}

Coordinate::Coordinate()
	:xC{default_coordinate().xCoord()},
	yC{default_coordinate().yCoord()}
{
}


Coordinate Coordinate::coords_cell_of_particular_position(double dimGrid) const 
{
	double coord_x_cell_of_pos, coord_y_cell_of_pos;

	if (xC > 0)
	{
		coord_x_cell_of_pos = ((static_cast<int>(xC/dimGrid))*dimGrid);
	}
	else 
	{
		coord_x_cell_of_pos = ((static_cast<int>(xC/dimGrid))*dimGrid) - dimGrid;
	}

	if (yC > 0)
	{
		coord_y_cell_of_pos = ((static_cast<int>(yC/dimGrid))*dimGrid);
	}
	else
	{
		coord_y_cell_of_pos = ((static_cast<int>(yC/dimGrid))*dimGrid) - dimGrid;
	}

	Coordinate cell_coordinates_of_position{coord_x_cell_of_pos, coord_y_cell_of_pos};

	return cell_coordinates_of_position;

}



/*Funzione, che calcola la distanza fra 2 celle generiche dell'area ORA considero di passare 1 cella come parametro mentre l'altra è la
cella a cui è applicato il metodo, altrimenti potrei passare entrambe le celle come parametro però non so se ha senso definire poi la funzione membro
nella classe Cell*/
double Coordinate::distance_btw_two_coords(const Coordinate& c) const
{

	double dist, diff_x2x1_squared, diff_y2y1_squared;
	diff_x2x1_squared = pow((c.xCoord() - xC), 2);
	diff_y2y1_squared = pow((c.yCoord() - yC), 2);
	dist = sqrt(diff_x2x1_squared + diff_y2y1_squared);

	return dist;

}



/*Funzione: calcola partendo dall'oggetto iniziale robot la distanza fra la cella a cui è applicato il metodo
cioè la cella associata alla posizione attuale del robot e la cella associata alla posizione goal*/
double Coordinate::distance_currentrobotcoord_goalrobotcoord(const Coordinate& cgoal) const
{
	
	double dist_currentRobcell_goalRobcell;
	dist_currentRobcell_goalRobcell = distance_btw_two_coords(cgoal);
	return dist_currentRobcell_goalRobcell;


}


double Coordinate::min_distance_currentrobotcoord_one_obstacle_coords(double dimGrid, const Obstacle& obst) const
{
	vector<Coordinate> vec_outline_obstacle_cells;
	vector<double> vec_dist_currrobcell_obstcells;
	double min_dist_currrobcell_obstcells;
	vec_outline_obstacle_cells = obst.outline_obstacle_coordinates(dimGrid);


	for (auto pos = vec_outline_obstacle_cells.cbegin(); pos != vec_outline_obstacle_cells.cend(); ++pos)
	{
		vec_dist_currrobcell_obstcells.push_back(distance_btw_two_coords(*pos)); 
	}

	min_dist_currrobcell_obstcells = *min_element(vec_dist_currrobcell_obstcells.cbegin(), vec_dist_currrobcell_obstcells.cend());


	return min_dist_currrobcell_obstcells;

}




double Coordinate::min_distance_currentrobotcoord_all_obstacles_coords(double dimGrid, const vector<Obstacle>& vecobst) const
{

	vector<double> vec_min_dist_currentrobcell_generic_obst;
	double min_dist_currrobcell_all_obstcells;
	for (auto pos = vecobst.cbegin(); pos != vecobst.cend(); ++pos) 
	{
		vec_min_dist_currentrobcell_generic_obst.push_back(min_distance_currentrobotcoord_one_obstacle_coords(dimGrid, *pos));
	}

	min_dist_currrobcell_all_obstcells = *min_element(vec_min_dist_currentrobcell_generic_obst.cbegin(), vec_min_dist_currentrobcell_generic_obst.cend());

	return min_dist_currrobcell_all_obstcells;

}




//Funzione: calcola il POTENZIALE TOTALE come somma del POTENZIALE ATTRATIVO tra pos.attuale e pos goal e POTENZIALE REPULSIVO tra pos.attuale e ostacoli.
double Coordinate::potential_tot_btw_currentgoalrobcoords_currentrobobstcoords(double _eta, double _zeta, double _max_dist_infl, double dimGrid, const Coordinate& cgoal, const vector<Obstacle>& vecobst_pot) const
{
	double potential_rg;
	double potential_ro;
	double potential_tot;
	double min_dist_currrobcell_allobstacle;

	potential_rg = 1.0/2*(_zeta)*pow(distance_currentrobotcoord_goalrobotcoord(cgoal), 2);

	min_dist_currrobcell_allobstacle = min_distance_currentrobotcoord_all_obstacles_coords(dimGrid, vecobst_pot);
	if (min_dist_currrobcell_allobstacle <= _max_dist_infl)
	{
		potential_ro = 1.0/2*(_eta)*pow(((1/min_dist_currrobcell_allobstacle)-(1/_max_dist_infl)), 2);
	}
	else
	{
		potential_ro = 0;

	}

	potential_tot = potential_rg + potential_ro;
	return potential_tot;

}


Coordinate Coordinate::path_planning_robot(double _eta, double _zeta, double _max_dist_infl, double dimGrid, const Coordinate& cgoal, const vector<Obstacle>& vecobst_pp) const
{
	//Inizio con il definire le 8 coordinate associate alle 8 future celle delle posizioni del robot impiegando il costruttore della classe Coordinate
	vector<Coordinate> possible_robot_next_step_coords; //è fondamentale inizializzare il vettore di coordinate ad ogni ciclo while per avere size=8!!
	possible_robot_next_step_coords.push_back(Coordinate{xC - dimGrid, yC + dimGrid}); //Coordinata delle cella 1
	possible_robot_next_step_coords.push_back(Coordinate{xC, yC + dimGrid}); //Coordinata delle cella 2
	possible_robot_next_step_coords.push_back(Coordinate{xC + dimGrid, yC + dimGrid}); //Coordinata delle cella 3
	possible_robot_next_step_coords.push_back(Coordinate{xC + dimGrid, yC}); //Coordinata delle cella 4
	possible_robot_next_step_coords.push_back(Coordinate{xC + dimGrid, yC - dimGrid}); //Coordinata delle cella 5
	possible_robot_next_step_coords.push_back(Coordinate{xC, yC - dimGrid}); //Coordinata delle cella 6
	possible_robot_next_step_coords.push_back(Coordinate{xC - dimGrid, yC - dimGrid}); //Coordinata delle cella 7
	possible_robot_next_step_coords.push_back(Coordinate{xC - dimGrid, yC}); //Coordinata delle cella 8


	vector<double> potential_possible_robot_next_step_coords;

    //Calcolo del potenziale totale delle possibili celle che il robot può andare ad occupare al movimento successivo
	for (auto pos = possible_robot_next_step_coords.cbegin(); pos != possible_robot_next_step_coords.cend(); ++pos)  
		{

			potential_possible_robot_next_step_coords.push_back((*pos).potential_tot_btw_currentgoalrobcoords_currentrobobstcoords(_eta, _zeta, _max_dist_infl, dimGrid, cgoal, vecobst_pp));

		}

    //Ricerca dell'indice della cella a potenziale minore considerando l'ordine delle celle descritto sopra
	auto idx_min_potential_robot_next_step_coords = min_element(potential_possible_robot_next_step_coords.begin(), potential_possible_robot_next_step_coords.end());
    int idx_ = std::distance(potential_possible_robot_next_step_coords.begin(), idx_min_potential_robot_next_step_coords);
    Coordinate robot_next_step_coords;
    robot_next_step_coords = possible_robot_next_step_coords[idx_];

	return robot_next_step_coords;

}

//operators overloading, che consente di effettuare la differenza fra due coordinate 
Coordinate operator - (const Coordinate& c1, const Coordinate& c2)
{

	double diff_in_x_coord, diff_in_y_coord;
	diff_in_x_coord = c1.xCoord() - c2.xCoord();
	diff_in_y_coord = c1.yCoord() - c2.yCoord();
	Coordinate coords_diff{diff_in_x_coord, diff_in_y_coord};

	return coords_diff;

}