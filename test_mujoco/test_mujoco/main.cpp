#include <stdbool.h> //for bool
#include "mojuco_test.h"
#include "mojuco_dbpendulum.h"
#include "mujoco_ik.h"
#include "mujoco_flap.h"
#include <iostream>
using namespace std;

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>
#include <mujoco/mujoco.h>

// set pendulum = 4 represents flap experiment.
// we create mujoco_flap class
// input file is the force and torque from SPHinXsys case, it is already set
// output file: test_validation_dataset.csv, it's output file. need to ensure there is no file in the folder.
// we can get corresponding output time, angle and angle velocity.
// for 1  2  3 this are reference project, i do the refactor, if you have less experience in mujoco, please refer these parts.
// i refer part 1 2 3 to finish the project.
// if you don't want to see simulation part, please uncomment the line: glfwDestroyWindow(window);

 int pendulum = 3;
// //1: fsm  
// //2: db_pendulum
// //3: pendulum_ik
// //4: flap test
// //main function
int main(int argc,  char** argv)
{
    //cout << _getcwd(NULL, 0) << endl; // get run program path
     char datafile[] = "test_validation_dataset.csv";
     char filename[] = "doublependulum_fsm.xml";
     char filename_dbpendulum[] = "doublependulum.xml";
     char filename_test[] = "oscw.xml";
     char filename_ik[] = "doublependulum_ik.xml";
     char filename_flap[] = "oscw.xml";
     char filename_db[] = "dbpendulum.xml";
     char forcefile[] = "input_force.csv";
    if (pendulum == 1) {
        pendulum_fsm fsm(filename);
        fsm.run_mujoco();
    }
    else if (pendulum == 2) {
        dbpendulum pend(filename_dbpendulum);
        pend.run_mujoco();
    }
    else if (pendulum == 3) {
        mujoco_ik ikpen(filename_ik, datafile);
        ikpen.run_mujoco();
    }
    else if (pendulum == 4) {// record the pos and velocity of flaps
        mujoco_flap flap(filename_flap, datafile, forcefile);// xml already keep safe
        flap.run_mujoco();
    }
    return 1;
}