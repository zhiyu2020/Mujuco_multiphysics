#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <direct.h>

using namespace std;

class pendulum_fsm {
public:
	pendulum_fsm(char filename[]);
	char* _filename;
	//simulation end time
	double simend = 5;
	//state machine
	static int _fsm_state;
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

	void set_torque_control(const mjModel* m, int actuator_no, int flag);
	void set_position_servo(const mjModel* m, int actuator_no, double kp);
	void set_velocity_servo(const mjModel* m, int actuator_no, double kv);
	static void generate_trajectory(double t0, double tf, double q_0[2], double q_f[2]);
	void init_controller(const mjModel* m, mjData* d);
	static void mycontroller(const mjModel* m, mjData* d);
	void run_mujoco();
	
	static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
	// mouse button callback
	static void mouse_button(GLFWwindow* window, int button, int act, int mods);
	// mouse move callback
	static void mouse_move(GLFWwindow* window, double xpos, double ypos);
	// scroll callback
	static void scroll(GLFWwindow* window, double xoffset, double yoffset);

private:
	GLFWwindow* generate_window();
};

