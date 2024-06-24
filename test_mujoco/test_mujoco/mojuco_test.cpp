
#include "mojuco_test.h"

////////////// static member var init //////////////////
mjModel* pendulum_fsm::_m = NULL;                  // MuJoCo model
mjData* pendulum_fsm::_d = NULL;                   // MuJoCo data
mjvCamera pendulum_fsm::_cam;                      // abstract camera
mjvOption pendulum_fsm::_opt;                      // visualization options
mjvScene pendulum_fsm::_scn;                       // abstract scene
mjrContext pendulum_fsm::_con;                     // custom GPU context

// mouse interaction
bool pendulum_fsm::_button_left = false;
bool pendulum_fsm::_button_middle = false;
bool pendulum_fsm::_button_right = false;
double pendulum_fsm::_lastx = 0;
double pendulum_fsm::_lasty = 0;

// init fsm state///
int pendulum_fsm::_fsm_state;

///////// init finit state parameters //////////
const int fsm_hold = 0;
const int fsm_swing1 = 1;
const int fsm_swing2 = 2;
const int fsm_stop = 3;

const double t_hold = 0.5;
const double t_swing1 = 1;
const double t_swing2 = 1;
///////////////////////////////////////////////
double a0[2] = { 0 }, a1[2] = { 0 }, a2[2] = { 0 }, a3[2] = { 0 };
double qref[2] = { 0 }, uref[2] = { 0 };



void pendulum_fsm::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
        // backspace: reset simulation
        if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
        {
            mj_resetData(_m, _d);
            mj_forward(_m, _d);
        }
}


// mouse button callback
void pendulum_fsm::mouse_button(GLFWwindow* window, int button, int act, int mods)
{
        // update button state
        _button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        _button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        _button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

        // update mouse position
        glfwGetCursorPos(window, &_lastx, &_lasty);
}

// mouse move callback
void pendulum_fsm::mouse_move(GLFWwindow* window, double xpos, double ypos)
{
        // no buttons down: nothing to do
        if (!_button_left && !_button_middle && !_button_right)
            return;

        // compute mouse displacement, save
        double dx = xpos - _lastx;
        double dy = ypos - _lasty;
        _lastx = xpos;
        _lasty = ypos;

        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        // get shift key state
        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        // determine action based on mouse button
        mjtMouse action;
        if (_button_right)
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else if (_button_left)
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else
            action = mjMOUSE_ZOOM;

        // move camera
        mjv_moveCamera(_m, action, dx / height, dy / height, &_scn, &_cam);
}


// scroll callback
void pendulum_fsm::scroll(GLFWwindow* window, double xoffset, double yoffset)
{
   // emulate vertical mouse motion = 5% of window height
   mjv_moveCamera(_m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &_scn, &_cam);
}


void pendulum_fsm::set_torque_control(const mjModel* m, int actuator_no, int flag)
{
    if (flag == 0)
        m->actuator_gainprm[10 * actuator_no + 0] = 0;
    else
        m->actuator_gainprm[10 * actuator_no + 2] = 1;
}

void pendulum_fsm::set_position_servo(const mjModel* m, int actuator_no, double kp)
{
    m->actuator_gainprm[10 * actuator_no + 0] = kp;
    m->actuator_biasprm[10 * actuator_no + 1] = -kp;
}

void pendulum_fsm::set_velocity_servo(const mjModel* m, int actuator_no, double kv)
{
    m->actuator_gainprm[10 * actuator_no + 0] = kv;
    m->actuator_biasprm[10 * actuator_no + 2] = -kv;
}

void pendulum_fsm::generate_trajectory(double t0, double tf, double q_0[2], double q_f[2])
{
    int i;
    double tf_t0_3 = (tf - t0) * (tf - t0) * (tf - t0);
    for (i = 0; i < 2; i++)
    {
        double q0 = q_0[i], qf = q_f[i];
        a0[i] = qf * t0 * t0 * (3 * tf - t0) + q0 * tf * tf * (tf - 3 * t0); a0[i] = a0[i] / tf_t0_3;
        a1[i] = 6 * t0 * tf * (q0 - qf); a1[i] = a1[i] / tf_t0_3;
        a2[i] = 3 * (t0 + tf) * (qf - q0); a2[i] = a2[i] / tf_t0_3;
        a3[i] = 2 * (q0 - qf); a3[i] = a3[i] / tf_t0_3;
    }
}

void pendulum_fsm::init_controller(const mjModel* m, mjData* d)
{
    _fsm_state = fsm_hold;
    set_position_servo(m, 1, 0); //set pservo1 to 0
    set_velocity_servo(m, 2, 0); //set vservo1 to 0
    set_position_servo(m, 4, 0); //set pservo2 to 0
    set_velocity_servo(m, 5, 0); //set vservo2 to 0
}


void pendulum_fsm::mycontroller(const mjModel* m, mjData* d)
{
    //write control here
    int i;
    double t;
    t = d->time;
    //start: q0 = -1; q1 = 0
    //intermediate: q0 = 0; q1 = -1.57 (pi/2)
    //end: q0 = 1; q1 = 0;
    //transitions
    if (_fsm_state == fsm_hold && d->time >= t_hold)
    {
        _fsm_state = fsm_swing1;
        double q_0[2] = { 0 }; double q_f[2] = { 0 };
        q_0[0] = -1; q_0[1] = 0;
        q_f[0] = 0.5;  q_f[1] = -2;
        generate_trajectory(t_hold, t_hold + t_swing1, q_0, q_f);
    }
    if (_fsm_state == fsm_swing1 && d->time >= t_hold + t_swing1)
    {
        _fsm_state = fsm_swing2;
        double q_0[2] = { 0 }; double q_f[2] = { 0 };
        q_0[0] = 0.5; q_0[1] = -2;
        q_f[0] = 1;  q_f[1] = 0;
        generate_trajectory(t_hold + t_swing1, t_hold + t_swing1 + t_swing2, q_0, q_f);
    }
    if (_fsm_state == fsm_swing2 && d->time >= t_hold + t_swing1 + t_swing2)
    {
        _fsm_state = fsm_stop;
    }

    //actions
    //start: q0 = -1; q1 = 0
    //intermediate: q0 = 0; q1 = -1.57 (pi/2)
    //end: q0 = 1; q1 = 0;
    double q0, q1;
    double kp = 500, kv = 50;
    if (_fsm_state == fsm_hold)
    {
        //q0 = -1; q1 = 0;
        qref[0] = -1; qref[1] = 0;
        uref[0] = 0;  uref[1] = 0;
        d->ctrl[0] = -kp * (d->qpos[0] - qref[0]) - kv * d->qvel[0];
        d->ctrl[3] = -kp * (d->qpos[1] - qref[1]) - kv * d->qvel[1];

        // d->ctrl[1] = q0;
        // d->ctrl[4] = q1;
    }
    if (_fsm_state == fsm_swing1) //generate trajectory
    {
        //q0 = 0; q1 = -1.57;


        for (i = 0; i < 2; i++)
        {
            qref[i] = a0[i] + a1[i] * t + a2[i] * t * t + a3[i] * t * t * t;
            uref[i] = a1[i] + 2 * a2[i] * t + 3 * a3[i] * t * t;
        }
        d->ctrl[0] = -kp * (d->qpos[0] - qref[0]) - kv * (d->qvel[0] - uref[0]);
        d->ctrl[3] = -kp * (d->qpos[1] - qref[1]) - kv * (d->qvel[1] - uref[1]);

        // d->ctrl[1] = q0;
        // d->ctrl[4] = q1;
    }
    if (_fsm_state == fsm_swing2) //generate trajectory
    {
        //q0 = 1; q1 = 0;

        //double qref[2]={0}, uref[2]={0};
        for (i = 0; i < 2; i++)
        {
            qref[i] = a0[i] + a1[i] * t + a2[i] * t * t + a3[i] * t * t * t;
            uref[i] = a1[i] + 2 * a2[i] * t + 3 * a3[i] * t * t;
        }
        d->ctrl[0] = -kp * (d->qpos[0] - qref[0]) - kv * (d->qvel[0] - uref[0]);
        d->ctrl[3] = -kp * (d->qpos[1] - qref[1]) - kv * (d->qvel[1] - uref[1]);


        // d->ctrl[1] = q0;
        // d->ctrl[4] = q1;
    }
    if (_fsm_state == fsm_stop)
    {
        qref[0] = 1;  qref[1] = 0;
        uref[0] = 0;  uref[1] = 0;
        d->ctrl[0] = -kp * (d->qpos[0] - qref[0]) - kv * d->qvel[0];
        d->ctrl[3] = -kp * (d->qpos[1] - qref[1]) - kv * d->qvel[1];

        // d->ctrl[1] = q0;
        // d->ctrl[4] = q1;
    }

    //write data here (dont change/dete this function call; instead write what you need to save in save_data)
    //if (loop_index % data_frequency == 0)
    //{
    //    save_data(m, d);
    //}
    //loop_index = loop_index + 1;
}


void pendulum_fsm::run_mujoco()
{
    // activate software
    mj_activate("mjkey.txt");
    // load and compile model
    char error[1000] = "Could not load binary model";
    _m = mj_loadXML(_filename, 0, error, 1000);
    if (!_m) mju_error_s("Load model error: %s", error);
    _d = mj_makeData(_m);
    GLFWwindow*  window = generate_window();
    // install control callback
    mjcb_control = mycontroller;

    _d->qpos[0] = -1;
    init_controller(_m, _d);
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = _d->time;
        while (_d->time - simstart < 1.0 / 60.0)
        {
            mj_step(_m, _d);
        }

        if (_d->time >= simend)
        {
            //fclose(fid_);
            break;
        }

        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(_m, _d, &_opt, NULL, &_cam, mjCAT_ALL, &_scn);
        mjr_render(viewport, &_scn, &_con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);
        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);
        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }
    // free visualization storage
    mjv_freeScene(&_scn);
    mjr_freeContext(&_con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(_d);
    mj_deleteModel(_m);
    mj_deactivate();
    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}


GLFWwindow*  pendulum_fsm::generate_window()
{
    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&_cam);
    mjv_defaultOption(&_opt);
    mjv_defaultScene(&_scn);
    mjr_defaultContext(&_con);
    mjv_makeScene(_m, &_scn, 2000);                // space for 2000 objects
    mjr_makeContext(_m, &_con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, pendulum_fsm::keyboard);
    glfwSetCursorPosCallback(window, pendulum_fsm::mouse_move);
    glfwSetMouseButtonCallback(window, pendulum_fsm::mouse_button);
    glfwSetScrollCallback(window, pendulum_fsm::scroll);

    double arr_view[] = { 89.608063, -11.588379, 5, 0.000000, 0.000000, 1.000000 };
    _cam.azimuth = arr_view[0];
    _cam.elevation = arr_view[1];
    _cam.distance = arr_view[2];
    _cam.lookat[0] = arr_view[3];
    _cam.lookat[1] = arr_view[4];
    _cam.lookat[2] = arr_view[5];
    return window;
}


pendulum_fsm::pendulum_fsm(char filename[])
{
    _filename = filename;
}