#include <ostream>
using std::ostream;
#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>
#include "Robot.h"
#include "Obstacle.h"
#include "Coordinate.h"
using std::cout;
using std::endl;
using std::vector;
using std::min_element;


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
	static Robot rr{0,0,0,0,100,100};
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

//Funzione: adegua le proprietà dell'ostacolo alla dimensione della griglia
void Obstacle::adapt_obstacle_to_grid(double dimGrid) 
{

	x1 = static_cast<int>((x1/dimGrid))*dimGrid;
	y1 = static_cast<int>((y1/dimGrid))*dimGrid;
	x2 = (static_cast<int>((x2/dimGrid))*dimGrid) + dimGrid; //Aggiunta di dimGrid perchè altrimenti avrei ad esempio 25.2 -> 25 mentre 25.2->26, ricordando che cella occupata parzialmente = cella occupata interamente
	y2 = (static_cast<int>((y2/dimGrid))*dimGrid) + dimGrid;

}


//Funzione: restituisce le coordinate dell'angolo in basso a sinistra o dell'angolo in alto a destra delle celle di contorno dell'ostacolo considerato già adattato alla griglia!!
vector<Coordinate> Obstacle::outline_obstacle_coordinates(double dimGrid) const
{
	vector<Coordinate> outline_obstacle_c;


	//Coordinate delle celle di contorno dell'ostacolo aventi stessa x = x1 e y compresa fra y1 e y2
	for (int i = 0; i <= (y2-y1); ++i)
	{
		outline_obstacle_c.push_back(Coordinate{x1, (y1 + i*dimGrid)});
	}

	//Coordinate delle celle del contorno dell'ostacolo aventi stessa y = y2 e x compresa fra x2 e x1
	for (int i = 1; i < (x2-x1); ++i)
	{
		outline_obstacle_c.push_back(Coordinate{(x1 + i*dimGrid), y2});
	}

	//Coordinate del contorno dell'ostacolo avaneti stessa x = x2 e y compresa fra y1 e y2
	for (int i = 0; i <= (y2-y1); ++i)
	{
		outline_obstacle_c.push_back(Coordinate{x2, (y1 + i*dimGrid)});
	}

	//Coordinate del contorno dell'ostacolo aventi stessa y = y1 e x compresa fra x1 e x2
	for (int i = 1; i < (x2-x1); ++i)
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

	/*for (auto pos = vec_outline_obstacle_cells.cbegin(); pos != vec_outline_obstacle_cells.cend(); ++pos)
	{
		cout << (*pos).xCell() << "," << (*pos).yCell() << " ";
	}*/

	//cout << endl;

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
	//cout << "La minima distanza dall'ostacolo risulta: " << min_dist_actrobcell_all_obstcells << endl;
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
	//cout << "Il potenziale attrattivo risulta: " << potential_rg << endl;

	min_dist_currrobcell_allobstacle = min_distance_currentrobotcoord_all_obstacles_coords(dimGrid, vecobst_pot);
	if (min_dist_currrobcell_allobstacle <= _max_dist_infl)
	{
		potential_ro = 1.0/2*(_eta)*pow(((1/min_dist_currrobcell_allobstacle)-(1/_max_dist_infl)), 2);
	}
	else
	{
		potential_ro = 0;

	}
	//cout << "Il potenziale repulsivo risulta: " << potential_ro << endl;

	potential_tot = potential_rg + potential_ro;

	//cout << "Il potenziale totale risulta: " << potential_tot << endl;

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
			//cout << '(' << (*pos).xCoord() << ',' << (*pos).yCoord() << ')' << endl;
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
	//Definisco le celle della posizione attuale/iniziale del robot e della posizione goal del robot 
	Coordinate currrobcell_coords, goalrobcell_coords;
	currrobcell_coords = curr_rob_lower_left_corner_cell_coord(dimGrid); //Coordinate dell'angolo in basso a sinistra della cella associata alla posizione attuale del robot
	double currrobcell_coord_x{currrobcell_coords.xCoord()};
	double currrobcell_coord_y{currrobcell_coords.yCoord()};
	goalrobcell_coords = goal_rob_lower_left_corner_cell_coord(dimGrid); //Coordinate dell'angolo in basso a sinistra della cella associata alla posizione goal del robot
	double goalrobcell_coord_x{goalrobcell_coords.xCoord()};
	double goalrobcell_coord_y{goalrobcell_coords.yCoord()};
	
	
	/*double currrobcellx, currrobcelly, diff_currrobcellx_xRcurrent, diff_currrobcelly_yRcurrent;
	currrobcellx = currrobcell.xCell();
	currrobcelly = currrobcell.yCell();
	diff_currrobcellx_xRcurrent = xRcurrent - currrobcellx;
	diff_currrobcelly_yRcurrent = yRcurrent - currrobcelly;*/


	Coordinate current_robot_coordinates{xRcurrent, yRcurrent};
	Coordinate goal_robot_coordinate{xRgoal, yRgoal};

	double diffx_currrobcellcoords_currrobcoords, diffy_currrobcellcoords_currrobcoords;
	diffx_currrobcellcoords_currrobcoords = current_robot_coordinates.xCoord() - currrobcell_coords.xCoord();
	diffy_currrobcellcoords_currrobcoords = current_robot_coordinates.yCoord() - currrobcell_coords.yCoord();
	cout << diffx_currrobcellcoords_currrobcoords << diffy_currrobcellcoords_currrobcoords << endl;


	/*Confronto fra le due posizioni del Robot è possibile solamente definendo +1 operator overloading per i simboli !=
	non serve perchè si confontano 2 double e non 2 oggetti della classe Cell, occhio al confronto perchè all'inizio true && true = true mentre poi false && true = false quando una
	delle due eguaglia il valore della posizione Goal l'algoritmo si ferma!
	*/
	while ( !((abs(goalrobcell_coord_x - currrobcell_coord_x) < 1e-9) && (abs(goalrobcell_coord_y - currrobcell_coord_y) < 1e-9))) //prima avevo currrobcellx e currrobcelly che non venivano aggiornati ad ogni ciclo!
	//while ( !(((goalrobcell.xCell() == currrobcell.xCell())) && ((goalrobcell.yCell() == currrobcell.yCell()))))
	{
		Coordinate robot_next_step_move;
		robot_next_step_move = current_robot_coordinates.path_planning_robot(_eta, _zeta, _max_dist_infl, dimGrid, goal_robot_coordinate, vecobst_pp);
		current_robot_coordinates = robot_next_step_move;
		currrobcell_coord_x = current_robot_coordinates.xCoord() - diffx_currrobcellcoords_currrobcoords;
		currrobcell_coord_y = current_robot_coordinates.yCoord() - diffy_currrobcellcoords_currrobcoords;
		
		//xRcurrent = currrobcell_coords.xCoord() + diffx_currrobcellcoords_currrobcoords;
		//yRcurrent = currrobcell_coords.yCoord() + diffy_currrobcellcoords_currrobcoords;
		
		//Celle della pos.attuale e pos.goal stampate per verifica
		//cout << "Coordinata dell'angolo inferiore della cella attuale Robot: " << '(' << currrobcell.xCell() << ',' << currrobcell.yCell() << ')' << endl;
		//cout << "Coordinata dell'angolo inferiore della cella goal Robot: " << '(' << goalrobcell.xCell() << ',' << goalrobcell.yCell() << ')' << endl;

		/*if perchè in alcuni casi può essere che il robot sia posizionato in alto nella cella quindi sia +vicino nella cella precedente rispetto alla 
		cella successiva in cui è posizionato il goal se questo è + vicino invece all'angolo in basso*/
		if ((abs(goalrobcell_coord_x - currrobcell_coord_x) < 1e-9) && (abs(goalrobcell_coord_y - currrobcell_coord_y) < 1e-9))
		{
			xRcurrent = xRgoal;
			yRcurrent = yRgoal;
		}
		else
		{
			xRcurrent = current_robot_coordinates.xCoord();
			yRcurrent = current_robot_coordinates.yCoord();
		}
		
		
		//Print del moto del robot all'interno dello spazio verso la pos. goal
		cout << "Coordinate del Robot in ordine iniziale, attuale e goal: " << '(' << xRinitial << ',' << yRinitial << ')' 
		<< '(' << xRcurrent << ',' << yRcurrent << ')' << '(' << xRgoal << ',' << yRgoal << ')' << endl;
		cout << "Coordinate del Robot cella attuale: " << currrobcell_coord_x << ',' << currrobcell_coord_y << endl;
		cout << "Coordinate del Robot cella goal: " << goalrobcell_coord_x << ',' << goalrobcell_coord_y << endl;

		// DA PROBLEMI con dimG=0.2 xRcurrent=0.3 yRcurrent=0.5 xRgoal=60 yRgoal=60
		// DA PROBLEMI con dimG=0.3 xRcurrent=0.5 yRcurrent=0.8 xRgoal=60 yRgoal=60


	}




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
