#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <direct.h>

// io header 
#include "io_mujoco.h"

class mujoco_ik {
public:
	//simulation end time
	double simend = 20;
	double qinit[2] = { 0,1.25 };
	static double _r;
	static double omega;
	static double x_0, y_0;

	//related to writing data to a file
	static FILE* _fid;
	static int _loop_index;

	char* _filename;
	char* _datafile;

	// MuJoCo data structures
	static mjModel* _m;                  // MuJoCo model
	static mjData* _d;                   // MuJoCo data
	static mjvCamera _cam;                      // abstract camera
	static mjvOption _opt;                      // visualization options
	static mjvScene _scn;                       // abstract scene
	static mjrContext _con;                     // custom GPU context

	// mouse interaction
	static bool _button_left;
	static bool _button_middle;
	static bool _button_right;
	static double _lastx;
	static double _lasty;

	// holders of one step history of time and position to calculate dertivatives
	mjtNum position_history = 0;
	mjtNum previous_time = 0;

	// controller related variables
	float_t ctrl_update_freq = 100;
	mjtNum last_update = 0.0;
	mjtNum ctrl;

	mujoco_ik(char filename[], char datafile[]);
	void run_mujoco();
	static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
	// mouse button callback
	static void mouse_button(GLFWwindow* window, int button, int act, int mods);
	// mouse move callback
	static void mouse_move(GLFWwindow* window, double xpos, double ypos);
	// scroll callback
	static void scroll(GLFWwindow* window, double xoffset, double yoffset);
	//****************************
	//This function is called once and is used to get the headers
	void init_save_data();
	//***************************
	//This function is called at a set frequency, put data here
	static void save_data(const mjModel* m, mjData* d);
	/******************************/
	void set_torque_control(const mjModel* m, int actuator_no, int flag);
	/******************************/
	void set_position_servo(const mjModel* m, int actuator_no, double kp);
	/******************************/
	void set_velocity_servo(const mjModel* m, int actuator_no, double kv);
	//**************************
	void init_controller(const mjModel* m, mjData* d);
	//**************************
	static void mycontroller(const mjModel* m, mjData* d);

private:
	static WriteMuJocoPinData pin_data;
	GLFWwindow* generate_window();
};
