#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <thread>
using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::string;
#include "Robot.h"
#include "Obstacle.h"
#include "Coordinate.h"
#include "PC.h"

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


//PC q(10);
PC q(3);

void consumer(Robot robot, double dimofgrid, vector<Obstacle> vobstacle, int num_samples)
{

    for (int i = 0; i < num_samples; ++i)
    {
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        Coordinate robot_goal_coords = q.take();

        /*robot_goal_x = robot_goal.xCoord();
        robot_goal_y = robot_goal.yCoord();
        Robot rob{robot.pos_xRgoal(), robot.pos_yRgoal(), robot.pos_xRgoal(), robot.pos_yRgoal(), robot_goal_x, robot_goal_y};*/


        //Impiego una fz che mi aggiorna il robot alle coordinate goal fornite dai satelliti, imponendo quelle del goal precedente come attuali  
        robot.update_robot_to_new_sample_goalcoords(robot_goal_coords);    
        robot.move_robot_to_goal(300.0, 1.0, 5.0, dimofgrid, vobstacle);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
       
    }
    

}

void producer(string sat, vector<Coordinate> sample_coords)
{

    for (auto pos=sample_coords.cbegin(); pos!=sample_coords.cend(); ++pos)
    {
        q.append(*pos); 
        cout << "Cooordinate x e y prodotte sono: " << (*pos).xCoord() << ',' << (*pos).yCoord() << " dal satellite: " << sat << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
       

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


    //robot_1.move_robot_to_goal(10000.0, 1.0, 10.0, dimG, vobs);


    vector<Coordinate> goals_coords_from_sat1, goals_coords_from_sat2;
    
    //Salvo le coordinate inviate dal primo dei due satelliti all'interno di un vettore di coordinate
    std::string filename1{"sample_coordinates_sat1.txt"};
    goals_coords_from_sat1 = read_from_file(filename1);

    for (auto pos = goals_coords_from_sat1.cbegin(); pos !=goals_coords_from_sat1.cend(); ++pos){

        cout << '(' << (*pos).xCoord() << ',' << (*pos).yCoord() << ')' << endl;

    }

    //Salvo le coordinate inviate dal secondo dei due satelliti all'interno di un vettore di coordinate
    std::string filename2{"sample_coordinates_sat2.txt"};
    goals_coords_from_sat2 = read_from_file(filename2);

    for (auto pos = goals_coords_from_sat1.cbegin(); pos !=goals_coords_from_sat1.cend(); ++pos){

        cout << '(' << (*pos).xCoord() << ',' << (*pos).yCoord() << ')' << endl;

    }
    //Definisco il numero dei campioni, che il robot dovrà processare
    int size_of_samples_vec;
    size_of_samples_vec = static_cast<int>((goals_coords_from_sat1).size());
    size_of_samples_vec = size_of_samples_vec + static_cast<int>((goals_coords_from_sat2).size());
    cout << "Il numero dei dati forniti dai due satelitti e': " << size_of_samples_vec << endl;


    std::thread c1(consumer, robot_1, dimG, vobs, size_of_samples_vec);
    std::thread p1(producer, "sat1", goals_coords_from_sat1);
    std::thread p2(producer, "sat2", goals_coords_from_sat2);

    c1.join();
    p1.join();
    p2.join();

    return 0;
    
  }
