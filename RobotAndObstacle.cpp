#include <ostream>
using std::ostream;
#include <iostream>
using std::cout;
using std::endl;
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
	while ( !((std::abs(goal_robot_coordinates.xCoord() - current_robot_coordinates.xCoord()) <= (dimGrid/2)) && (std::abs(goal_robot_coordinates.yCoord() - current_robot_coordinates.yCoord()) <= (dimGrid/2))))
	{
		Coordinate robot_next_step_move;
		robot_next_step_move = current_robot_coordinates.path_planning_robot(_eta, _zeta, _max_dist_infl, dimGrid, goal_robot_coordinates, vecobst_pp);
		current_robot_coordinates = robot_next_step_move;
		xRcurrent = current_robot_coordinates.xCoord();
		yRcurrent = current_robot_coordinates.yCoord();
	
		
		//Print del moto del robot all'interno dello spazio verso la pos. goal
		//cout << "Coordinate del Robot in ordine iniziale, attuale e goal: " << '(' << xRinitial << ',' << yRinitial << ')' 
		//<< '(' << xRcurrent << ',' << yRcurrent << ')' << '(' << xRgoal << ',' << yRgoal << ')' << endl;


	}

	adapt_final_currrobcoords_to_goalcoords(dimGrid, current_robot_coordinates, goal_robot_coordinates);

	cout << "Coordinate del Robot finali in ordine iniziale, attuale e goal: " << '(' << xRinitial << ',' << yRinitial << ')' 
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
	xRinitial = xRgoal;
	yRinitial = yRgoal;
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


