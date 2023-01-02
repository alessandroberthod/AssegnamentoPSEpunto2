#include <iostream>
#include <fstream>
#include <vector>
using std::cout;
using std::cin;
using std::endl;
using std::vector;
#include "Robot.h"
#include "Obstacle.h"
#include "Coordinate.h"

vector<Coordinate> read_from_file(const std::string& filename) {

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

int main()
{
    double xRini, yRini, xRgoal, yRgoal, dimG;

    //Si fornisce la dimensione della cella desiderata
    cout<<"Fornire la dimensione della cella della griglia: ";
    cin>>dimG;

    //Si definiscono le proprietà del robot:
    cout<<"Fornire la posizione x iniziale del Robot: ";
    cin>>xRini;
    cout<<"Fornire la posizione y iniziale del Robot: ";
    cin>>yRini;
   cout<<"Fornire la posizione x goal del Robot: ";
    cin>>xRgoal;
    cout<<"Fornire la posizione y goal del Robot: ";
    cin>>yRgoal;

    /*Si genera l'oggetto della classe Robot robot_1, il quale
    avrà inizialmnete posizione inziale e attuale coincidenti*/
    Robot robot_1{xRini, yRini, xRini, yRini, xRgoal, yRgoal};
    cout << "Posizione iniziale, attuale e goal del robot all'interno della griglia: " << robot_1 << endl;
    
    //Si definiscono gli ostacoli presenti all'interno dell'area
    Obstacle ostacolo1{20.3, 20.6, 25.9, 25.2};
    Obstacle ostacolo2{110.7, 130.4, 150.3, 190.5};
    //Obstacle ostacolo3{450.8, 540.9, 890.1, 920.1};
    //Obstacle ostacolo4{450.4, 480.7, 660.78, 570.67};
    //Obstacle ostacolo5{750.43, 660.82, 950.5, 900.32};

    //Adatto le coordinate degli ostacoli alle celle della griglia
    ostacolo1.adapt_obstacle_to_grid(dimG);
    ostacolo2.adapt_obstacle_to_grid(dimG);
    //ostacolo3.adapt_obstacle_to_grid(dimG);

    vector <Obstacle> vobs;
    vobs.push_back(ostacolo1);
    vobs.push_back(ostacolo2);
    //vobs.push_back(ostacolo3);
    //vobs.push_back(ostacolo4);
    //vobs.push_back(ostacolo5);

    //Ciclo for: per stampare gli ostacoli adattati alla dimensione della griglia (per verifica)
    for (auto pos = vobs.cbegin(); pos !=vobs.cend(); ++pos){

        cout << *pos << endl;

    }


    
    //Cell prova4{18,32};
    //Cell goalprova4{25,30};
    /*double distanzaprova4 = prova4.distance_btw_cell(goalprova4);
    cout << distanzaprova4 << endl;
    double distanzaprova4_2 = prova4.min_distance_actualrobotcell_one_obstacle_cells(dimG, ostacolo1);
    cout << distanzaprova4_2 << endl;*/
    //double potenzialeprova4 = prova4.potential_tot_btw_actgoalrobcells_actrobobstcells(100.0, 3.0, 10.0, dimG, goalprova4, vobs);
    //cout << potenzialeprova4;
    //Robot robotprova4{0.2,0.2,0.2,0.2,30.5,30.8};
    //robotprova4.move_robot_to_goal(1000.0, 3.0, 10.0, dimG, vobs);

    
    //robot_1.move_robot_to_goal(100.0, 1.0, 10.0, dimG, vobs); //ok per 30 e 30
    /*vector<Cell> cellediconfineostacolo2;
    cellediconfineostacolo2 = ostacolo2.outline_obstacle_cells(dimG);
    for (auto pos = cellediconfineostacolo2.cbegin(); pos !=cellediconfineostacolo2.cend(); ++pos){

        cout << '(' << (*pos).xCell() << ',' << (*pos).yCell() << ')' << endl;

    }*/

    //Cell provapostvisione{141, 125};
    //double distanzaobstprovapostvisione;
    //distanzaobstprovapostvisione = provapostvisione.min_distance_actualrobotcell_all_obstacles_cells(dimG, vobs);
    //robot_1.move_robot_to_goal(10000.0, 1.0, 10.0, dimG, vobs);



    //Salvo le coordinate inviate dai due sataletti all'interno di 2 vettori di coordinate
    vector<Coordinate> goals_coords_from_sat1, goals_coords_from_sat2;
    std::string filename{"sample_coordinates.txt"};
    goals_coords_from_sat1 = read_from_file(filename);

    for (auto pos = goals_coords_from_sat1.cbegin(); pos !=goals_coords_from_sat1.cend(); ++pos){

        cout << '(' << (*pos).xCoord() << ',' << (*pos).yCoord() << ')' << endl;

    }

    return 0;
    
  }
