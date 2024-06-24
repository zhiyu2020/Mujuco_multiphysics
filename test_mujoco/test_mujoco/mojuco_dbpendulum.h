#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <direct.h>


class dbpendulum {
public:
	dbpendulum(char filename[]);
	static mjModel* _m;                  // MuJoCo model
	static mjData* _d;                   // MuJoCo data
	static mjvCamera _cam;                      // abstract camera
	static mjvOption _opt;                      // visualization options
	static mjvScene _scn;                       // abstract scene
	static mjrContext _con;                     // custom GPU context
	// mouse interaction
	static bool button_left;
	static bool button_middle;
	static bool button_right;
	static double lastx;
	static double lasty;
	// file path
	char* _filename;
	// simulation end time 
	double _simend = 5;
	// holders of one step history of time and position to calculate dertivatives
	mjtNum position_history = 0;
	mjtNum previous_time = 0;
	// controller related variables
	float_t ctrl_update_freq = 100;
	mjtNum last_update = 0.0;
	mjtNum ctrl;
	static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
	// mouse button callback
	static void mouse_button(GLFWwindow* window, int button, int act, int mods);
	// mouse move callback
	static void mouse_move(GLFWwindow* window, double xpos, double ypos);
	// scroll callback
	static void scroll(GLFWwindow* window, double xoffset, double yoffset);
	static void mycontroller(const mjModel* m, mjData* d);
	void run_mujoco();
	GLFWwindow* generate_window();
};