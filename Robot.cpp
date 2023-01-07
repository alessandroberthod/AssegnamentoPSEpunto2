#include <ostream>
using std::ostream;
#include <cmath>
#include <iostream>
using std::cout;
using std::endl;
#include <algorithm>
using std::min_element;
#include <vector>
using std::vector;

#include "Robot.h"
#include "Obstacle.h"
#include "Coordinate.h"




//Costruttore Robot
Robot::Robot(double xRinit, double yRinit, double xRcurr, double yRcurr, double xRg, double yRg)
    : xRinitial{xRinit}, yRinitial{yRinit}, xRcurrent{xRcurr}, yRcurrent{yRcurr}, xRgoal{xRg}, yRgoal{yRg}
{
   /*if di controllo se il robot è all'interno dell'area
   definita da xmax,ymax e xmin,ymin degli ostacoli ->
   altrimenti invalid input*/

}
//Costruttore default Robot
const Robot& default_robot()
{
	static Robot rr{0,0,0,0,0,0};
	return rr;
}

Robot::Robot()
	:xRinitial{default_robot().pos_xRinitial()},
	yRinitial{default_robot().pos_yRinitial()},
	xRcurrent{default_robot().pos_xRcurrent()},
	yRcurrent{default_robot().pos_yRcurrent()},
	xRgoal{default_robot().pos_xRgoal()},
	yRgoal{default_robot().pos_yRgoal()}
{
}


//Costruttore ostacolo
Obstacle::Obstacle(double x1ob, double y1ob, double x2ob, double y2ob)
	: x1{x1ob}, y1{y1ob}, x2{x2ob}, y2{y2ob}
{
}
//Costruttore di default ostacolo
const Obstacle& default_obstacle()
{
	static Obstacle oo{0,0,100,100};
	return oo;
}

Obstacle::Obstacle()
	:x1{default_obstacle().x1o()},
	y1{default_obstacle().y1o()},
	x2{default_obstacle().x2o()},
	y2{default_obstacle().y2o()}
{
}

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




//Funzioni: in base alle posizioni attuali e goal del robot inserite dall'utente associano a quest'ultime le coordinate dell'angolo inferiore a sinistar della cella specifica nello spazio
Coordinate Robot::curr_rob_lower_left_corner_cell_coord(double dimGrid)
{
  	double xRcurrentfz{xRcurrent}; //Variabili fittizie in modo tale che quando chiamo la fz non vario i dati membri dell'oggetto robot anche se non si notava perchè 1 volta ingrigliato l'operazione lo mantiene =
	double yRcurrentfz{yRcurrent};
	Coordinate currRobCell_lower_left_corner_coord;
	xRcurrentfz = ((static_cast<int>(xRcurrentfz/dimGrid))*dimGrid);
	yRcurrentfz = ((static_cast<int>(yRcurrentfz/dimGrid))*dimGrid);
	currRobCell_lower_left_corner_coord = {xRcurrentfz, yRcurrentfz};

	return currRobCell_lower_left_corner_coord;

}

Coordinate Robot::goal_rob_lower_left_corner_cell_coord(double dimGrid)
{

	double xRgoalfz{xRgoal};
	double yRgoalfz{yRgoal};
	Coordinate goalRobCell_lower_left_corner_coord;
	xRgoalfz = ((static_cast<int>(xRgoal/dimGrid))*dimGrid);
	yRgoalfz = ((static_cast<int>(yRgoal/dimGrid))*dimGrid);
	goalRobCell_lower_left_corner_coord = {xRgoalfz, yRgoalfz};

	return goalRobCell_lower_left_corner_coord;

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



//Funzione: adegua le proprietà dell'ostacolo alla dimensione della griglia
void Obstacle::adapt_obstacle_to_grid(double dimGrid) 
{
	
	if (x1 > 0)
	{
		x1 = static_cast<int>((x1/dimGrid))*dimGrid;
	}
	else 
	{
		x1 = static_cast<int>((x1/dimGrid))*dimGrid - dimGrid;
	}

	if (y1 > 0)
	{
		y1 = static_cast<int>((y1/dimGrid))*dimGrid;
	}
	else 
	{
		y1 = static_cast<int>((y1/dimGrid))*dimGrid - dimGrid;
	}

	if (x2 > 0)
	{
		x2 = (static_cast<int>((x2/dimGrid))*dimGrid) + dimGrid;//Aggiunta di dimGrid perchè altrimenti avrei ad esempio 25.2 -> 25 mentre 25.2->26, ricordando che cella occupata parzialmente = cella occupata interamente
	}
	else
	{
		x2 = (static_cast<int>((x2/dimGrid))*dimGrid);
	}

	if (y2 > 0)
	{
		y2 = (static_cast<int>((y2/dimGrid))*dimGrid) + dimGrid;
	}
	else 
	{
		y2 = (static_cast<int>((y2/dimGrid))*dimGrid);
	}
	
}


//Funzione: restituisce le coordinate dell'angolo in basso a sinistra o dell'angolo in alto a destra delle celle di contorno dell'ostacolo considerato già adattato alla griglia!!
vector<Coordinate> Obstacle::outline_obstacle_coordinates(double dimGrid) const
{
	vector<Coordinate> outline_obstacle_c;


	//Coordinate delle celle di contorno dell'ostacolo aventi stessa x = x1 e y compresa fra y1 e y2
	for (int i = 0; i <= (y2-y1)/dimGrid; ++i)
	{
		outline_obstacle_c.push_back(Coordinate{x1, (y1 + i*dimGrid)});
	}

	//Coordinate delle celle del contorno dell'ostacolo aventi stessa y = y2 e x compresa fra x2 e x1
	for (int i = 1; i < (x2-x1)/dimGrid; ++i)
	{
		outline_obstacle_c.push_back(Coordinate{(x1 + i*dimGrid), y2});
	}

	//Coordinate del contorno dell'ostacolo avaneti stessa x = x2 e y compresa fra y1 e y2
	for (int i = 0; i <= (y2-y1)/dimGrid; ++i)
	{
		outline_obstacle_c.push_back(Coordinate{x2, (y1 + i*dimGrid)});
	}

	//Coordinate del contorno dell'ostacolo aventi stessa y = y1 e x compresa fra x1 e x2
	for (int i = 1; i < (x2-x1)/dimGrid; ++i)
	{
		outline_obstacle_c.push_back(Coordinate{(x1 + i*dimGrid), y1});
	}

	return outline_obstacle_c;

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


void Robot::move_robot_to_goal(double _eta, double _zeta, double _max_dist_infl, double dimGrid, const vector<Obstacle>& vecobst_pp) 
{
	Coordinate current_robot_coordinates{xRcurrent, yRcurrent};
	Coordinate goal_robot_coordinates{xRgoal, yRgoal};


	cout << "Coordinate del Robot iniziali in ordine iniziale, attuale e goal: " << '(' << xRinitial << ',' << yRinitial << ')' 
	<< '(' << xRcurrent << ',' << yRcurrent << ')' << '(' << xRgoal << ',' << yRgoal << ')' << endl;


	/*Confronto fra le due posizioni del Robot è possibile solamente definendo +1 operator overloading per i simboli !=
	non serve perchè si confontano 2 double e non 2 oggetti della classe Cell, occhio al confronto perchè all'inizio true && true = true mentre poi false && true = false quando una
	delle due eguaglia il valore della posizione Goal l'algoritmo si ferma!
	*/
	while ( !((std::abs(goal_robot_coordinates.xCoord() - current_robot_coordinates.xCoord()) <= (dimGrid/2 - 1e-9)) && (std::abs(goal_robot_coordinates.yCoord() - current_robot_coordinates.yCoord()) <= (dimGrid/2 - 1e-9))))
	{
		Coordinate robot_next_step_move;
		robot_next_step_move = current_robot_coordinates.path_planning_robot(_eta, _zeta, _max_dist_infl, dimGrid, goal_robot_coordinates, vecobst_pp);
		current_robot_coordinates = robot_next_step_move;
		xRcurrent = current_robot_coordinates.xCoord();
		yRcurrent = current_robot_coordinates.yCoord();
	
		
		//Print del moto del robot all'interno dello spazio verso la pos. goal
		cout << "Coordinate del Robot in ordine iniziale, attuale e goal: " << '(' << xRinitial << ',' << yRinitial << ')' 
		<< '(' << xRcurrent << ',' << yRcurrent << ')' << '(' << xRgoal << ',' << yRgoal << ')' << endl;


	}

	adapt_final_currrobcoords_to_goalcoords(dimGrid, current_robot_coordinates, goal_robot_coordinates);

	cout << "Coordinate del Robot in ordine iniziale, attuale e goal: " << '(' << xRinitial << ',' << yRinitial << ')' 
	<< '(' << xRcurrent << ',' << yRcurrent << ')' << '(' << xRgoal << ',' << yRgoal << ')' << endl;


}
void Robot::adapt_final_currrobcoords_to_goalcoords(double dimGrid, const Coordinate& finalcurrcoords, const Coordinate& goalcoords)
{
	
	Coordinate finalcurrcoords_cell, goalcoords_cell, diff_coords_goal, diff_coords_current;
	finalcurrcoords_cell = finalcurrcoords.coords_cell_of_particular_position(dimGrid);
	goalcoords_cell = goalcoords.coords_cell_of_particular_position(dimGrid);


	diff_coords_current = finalcurrcoords - finalcurrcoords_cell;
	diff_coords_goal = goalcoords - goalcoords_cell;

	//Operazioni considerando le coordinate in x:
	if (finalcurrcoords_cell.xCoord() < goalcoords_cell.xCoord())
	{ 
		xRcurrent = xRcurrent + dimGrid - diff_coords_current.xCoord() + diff_coords_goal.xCoord();
	}
	else if (goalcoords_cell.xCoord() < finalcurrcoords_cell.xCoord())
	{
		xRcurrent = xRcurrent - dimGrid + diff_coords_goal.xCoord() - diff_coords_current.xCoord();
	}
	else
	{
		xRcurrent = xRcurrent + diff_coords_goal.xCoord() - diff_coords_current.xCoord();
	}



	//Operazioni considerando le coordinate in y:
	if (finalcurrcoords_cell.yCoord() < goalcoords_cell.yCoord())
	{ 
		yRcurrent = yRcurrent + dimGrid - diff_coords_current.yCoord() + diff_coords_goal.yCoord();
	}
	else if (goalcoords_cell.yCoord() < finalcurrcoords_cell.yCoord())
	{
		yRcurrent = yRcurrent - dimGrid + diff_coords_goal.yCoord() - diff_coords_current.yCoord();
	}
	else
	{
		yRcurrent = yRcurrent + diff_coords_goal.yCoord() - diff_coords_current.yCoord();
	}


}

void Robot::update_robot_to_new_sample_goalcoords(const Coordinate& coordsgoal)
{
	xRcurrent = xRgoal;
	yRcurrent = yRgoal;
	xRgoal = coordsgoal.xCoord();
	yRgoal = coordsgoal.yCoord();

}



//operators overloading per avere cout Robot e Ostacoli
ostream& operator<<(ostream& os, const Robot& rob_os)
{
	return os << '(' << rob_os.pos_xRinitial()
		<< ',' << rob_os.pos_yRinitial() << ") " << '(' << rob_os.pos_xRcurrent() << ',' << rob_os.pos_yRcurrent() << ") "
		<< '(' << rob_os.pos_xRgoal() << ',' << rob_os.pos_yRgoal() << ')';
}


ostream& operator<<(ostream& os, const Obstacle& obs_os)
{
	return os << '(' << obs_os.x1o()
		<< ',' << obs_os.y1o() << ',' << obs_os.x2o()
		<< ',' << obs_os.y2o() << ')';
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