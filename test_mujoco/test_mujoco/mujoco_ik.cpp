#include "mujoco_ik.h"
#include <windows.h>
const int data_frequency = 1000;

FILE* mujoco_ik::_fid;
int mujoco_ik::_loop_index = 0;
////////////// static member var init //////////////////
mjModel* mujoco_ik::_m = NULL;                  // MuJoCo model
mjData* mujoco_ik::_d = NULL;                   // MuJoCo data
mjvCamera mujoco_ik::_cam;                      // abstract camera
mjvOption mujoco_ik::_opt;                      // visualization options
mjvScene mujoco_ik::_scn;                       // abstract scene
mjrContext mujoco_ik::_con;                     // custom GPU context

// mouse interaction
bool mujoco_ik::_button_left = false;
bool mujoco_ik::_button_middle = false;
bool mujoco_ik::_button_right = false;
double mujoco_ik::_lastx = 0;
double mujoco_ik::_lasty = 0;

///// parameters for running pendulum
double mujoco_ik::omega = 0.5;
double mujoco_ik::_r = 0.5;
double mujoco_ik::x_0 = 0.0;
double mujoco_ik::y_0 = 0.0;

WriteMuJocoPinData mujoco_ik::pin_data;

mujoco_ik::mujoco_ik(char filename[], char datafile[])
{
    // 检查文件是否存在
    if (GetFileAttributesA(datafile) != INVALID_FILE_ATTRIBUTES) {
        // 如果文件存在，则删除它
        if (DeleteFileA(datafile)) {
            std::cout << "successfuly delete file! " << std::endl;
        }
        else {
            std::cerr << "delete file failed. error code：" << GetLastError() << std::endl;
        }
    }
    _filename = filename;
    _datafile = datafile;
}

void mujoco_ik::run_mujoco() {
    // activate software
    mj_activate("mjkey.txt");
    // load and compile model
    char error[1000] = "Could not load binary model";
    _m = mj_loadXML(_filename, 0, error, 1000);
    if (!_m) mju_error_s("Load model error: %s", error);
    _d = mj_makeData(_m);
    GLFWwindow* window = generate_window();
    // install control callback
    mjcb_control = mycontroller;
    _d->qpos[0] = qinit[0];
    _d->qpos[1] = qinit[1];
    ///////// file open ////////
    _fid = fopen(_datafile, "w");
    pin_data.initFile();
    init_save_data(); // init save data
    init_controller(_m, _d);
    
    // use the first while condition if you want to simulate for a period.
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

        if (_d->time >= simend) // end time 20s
        {
            fclose(_fid); // close it 
            pin_data.close();
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
}

void mujoco_ik::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(_m, _d);
        mj_forward(_m, _d);
    }
}

// mouse button callback
void mujoco_ik::mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    _button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    _button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    _button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    // update mouse position
    glfwGetCursorPos(window, &_lastx, &_lasty);
}

// mouse move callback
void mujoco_ik::mouse_move(GLFWwindow* window, double xpos, double ypos) {
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
void mujoco_ik::scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(_m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &_scn, &_cam);
}

//****************************
//This function is called once and is used to get the headers
void mujoco_ik::init_save_data() { // modify the write data 
    //write name of the variable here (header)
    fprintf(_fid, "time, ");
    fprintf(_fid, "x, y ");
    //Don't remove the newline
    fprintf(_fid, "\n");
}

//***************************
//This function is called at a set frequency, put data here
void mujoco_ik::save_data(const mjModel* m, mjData* d) {
    //data here should correspond to headers in init_save_data()
    //seperate data by a space %f followed by space
    fprintf(_fid, "%f, ", d->time);
    fprintf(_fid, "%f, %f ", d->sensordata[0], d->sensordata[2]);
    //Don't remove the newline
    fprintf(_fid, "\n");
}

/******************************/
void mujoco_ik::set_torque_control(const mjModel* m, int actuator_no, int flag)
{
    if (flag == 0)
        m->actuator_gainprm[10 * actuator_no + 0] = 0;
    else
        m->actuator_gainprm[10 * actuator_no + 0] = 1;
}

/******************************/
void mujoco_ik::set_position_servo(const mjModel* m, int actuator_no, double kp)
{
    m->actuator_gainprm[10 * actuator_no + 0] = kp;
    m->actuator_biasprm[10 * actuator_no + 1] = -kp;
}

/******************************/
void mujoco_ik::set_velocity_servo(const mjModel* m, int actuator_no, double kv)
{
    m->actuator_gainprm[10 * actuator_no + 0] = kv;
    m->actuator_biasprm[10 * actuator_no + 2] = -kv;
}

//**************************//
void mujoco_ik::init_controller(const mjModel* m, mjData* d)
{
    //mj_step(m,d);
    mj_forward(m, d);
    // 1 time intergration;
    printf("position = %f %f \n", d->sensordata[0], d->sensordata[2]);

    //x0+r = d->sensordata[0];
    //y0 = d->sensordata[2]

    x_0 = d->sensordata[0] - _r;
    y_0 = d->sensordata[2];
}

//**************************
void mujoco_ik::mycontroller(const mjModel* m, mjData* d)
{
    //write control here
    //printf("position = %f %f %f \n",d->sensordata[0],d->sensordata[1],d->sensordata[2]);
    //printf("velocity = %f %f %f \n",d->sensordata[3],d->sensordata[4],d->sensordata[5]);

    //void mj_jac(const mjModel* m, const mjData* d,mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body);
    double jacp[6] = { 0 };
    double point[3] = { d->sensordata[0],d->sensordata[1],d->sensordata[2] };
    int body = 2; // 2 link
    // jacp translational jac
    mj_jac(m, d, jacp, NULL, point, body);
    // printf("J = \n");//3x2
    // printf("%f %f \n", jacp[0],jacp[1]);
    // printf("%f %f \n", jacp[2],jacp[3]);
    // printf("%f %f \n", jacp[4],jacp[5]);
    // printf("*********\n");
    // 这四个有值 剩下的没有 2 3为0
    double J[4] = { jacp[0],jacp[1],jacp[4],jacp[5] };
    // qdot is the v of the joint
    double qdot[2] = { d->qvel[0],d->qvel[1] };
    double xdot[2] = { 0 };
    //xdot = J*qdot using Jacobian to compute v
    mju_mulMatVec(xdot, J, qdot, 2, 2);
    // 获得的是x和z轴方向的velocity
    // printf("velocity using jacobian: %f %f \n",xdot[0],xdot[1]);
    // printf("velocity using sensordata= %f %f \n",d->sensordata[3],d->sensordata[5]);

    // d->ctrl[0] = qinit[0];
    // d->ctrl[2] = qinit[1];

    int i;
    double det_J = J[0] * J[3] - J[1] * J[2];
    double J_temp[] = { J[3],-J[1],-J[2],J[0] };
    double J_inv[4] = {};
    for (i = 0; i < 4; i++)
        J_inv[i] = J_temp[i] / det_J;

    double x, y;
    x = x_0 + _r * cos(omega * d->time);
    y = y_0 + _r * sin(omega * d->time);

    // dr the change in the x position
    double dr[] = { x - d->sensordata[0],y - d->sensordata[2] };
    double dq[2] = {};

    //dq = Jinv*dr
    mju_mulMatVec(dq, J_inv, dr, 2, 2);
    printf("%f %f \n", dq[0], dq[1]);

    //q -> q+dq
    //ctrl = q
    d->ctrl[0] = d->qpos[0] + dq[0];
    d->ctrl[2] = d->qpos[1] + dq[1];
    //write data here (dont change/dete this function call; instead write what you need to save in save_data)
    if (_loop_index % data_frequency == 0)
    {
        save_data(m, d);// control and record corresponding data
        pin_data.writeToFile(m, d);
    }
    _loop_index++;
}


GLFWwindow* mujoco_ik::generate_window()
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
    glfwSetKeyCallback(window, mujoco_ik::keyboard);
    glfwSetCursorPosCallback(window, mujoco_ik::mouse_move);
    glfwSetMouseButtonCallback(window, mujoco_ik::mouse_button);
    glfwSetScrollCallback(window, mujoco_ik::scroll);

    double arr_view[] = { 89.608063, -11.588379, 5, 0.000000, 0.000000, 1.000000 };
    _cam.azimuth = arr_view[0];
    _cam.elevation = arr_view[1];
    _cam.distance = arr_view[2];
    _cam.lookat[0] = arr_view[3];
    _cam.lookat[1] = arr_view[4];
    _cam.lookat[2] = arr_view[5];
    return window;
}