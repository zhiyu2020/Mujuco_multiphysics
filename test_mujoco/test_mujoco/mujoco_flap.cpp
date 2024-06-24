#include "mujoco_flap.h"
#include <fstream>
#include <iostream>
#include <windows.h>

// 
void find_str(string s, string del) {
    // Use find function to find 1st position of delimiter.

    int end = s.find(del);
    while (end != -1) { // Loop until no delimiter is left in the string.
        cout << s.substr(0, end) << endl;
        s.erase(s.begin(), s.begin() + end + 1);
        end = s.find(del);
    }
    cout << s.substr(0, end);
}

using namespace std;
const int data_frequency = 2;
FILE* mujoco_flap::_fid;
// init static class parameters
mjModel* mujoco_flap::_m = NULL;                  // MuJoCo model
mjData* mujoco_flap::_d = NULL;                   // MuJoCo data
mjvCamera mujoco_flap::_cam;                      // abstract camera
mjvOption mujoco_flap::_opt;                      // visualization options
mjvScene mujoco_flap::_scn;                       // abstract scene
mjrContext mujoco_flap::_con;                     // custom GPU context

// mouse interaction
bool mujoco_flap::button_left = false;
bool mujoco_flap::button_middle = false;
bool mujoco_flap::button_right = false;
double mujoco_flap::lastx = 0;
double mujoco_flap::lasty = 0;

int mujoco_flap::_loop_index = 0;

WriteMuJocoPinData mujoco_flap::pin_data;
//
std::vector<std::pair<Vec3d, Vec3d>> mujoco_flap::pair_vec_force_torque_;
std::vector<double> mujoco_flap::vec_d_timestamp_;
int i_idx = 0;
// lines count from _forcefile
const int force_torque_count = 5817;

void mujoco_flap::read_force_data()
{
    //std::pair<std::vector<double>, std::vector<double>> pair_vec_force_torque_;
    //double d_timestamp_;
    std::ifstream csv_data(_forcefile, std::ios::in);
    std::string line;

    if (!csv_data.is_open())
    {
        std::cout << "Error: opening file fail" << std::endl;
        std::exit(1);
    }

    std::istringstream sin;         //将整行字符串line读入到字符串istringstream中
    std::vector<std::string> words; //声明一个字符串向量
    std::string word;
    std::string del = ",";

    // 读取标题行
    std::getline(csv_data, line);
    std::cout << "title: " << line << std::endl;
    // 读取数据
    while (std::getline(csv_data, line))
    {
        {
            words.clear();
            int end = line.find(del);
            while (end != -1) { // Loop until no delimiter is left in the string.
                //cout << line.substr(0, end) << endl;
                words.push_back(line.substr(0, end));
                line.erase(line.begin(), line.begin() + end + 1);
                end = line.find(del);
            }
            double d_timestamp = stod(words[0]);
            vec_d_timestamp_.push_back(d_timestamp);
            std::pair<Vec3d, Vec3d> pair_vec_force_torque;
            pair_vec_force_torque.first.x = stod(words[1]);
            pair_vec_force_torque.first.y = stod(words[2]);
            pair_vec_force_torque.first.z = stod(words[3]);
            pair_vec_force_torque.second.x = stod(words[4]);
            pair_vec_force_torque.second.y = stod(words[5]);
            pair_vec_force_torque.second.z = stod(words[6]);
            pair_vec_force_torque_.push_back(pair_vec_force_torque);

            //std::cout << "t: " << d_timestamp_ << " f: " <<
            //    pair_vec_force_torque.first.x << " , " << pair_vec_force_torque.first.y << " , " << pair_vec_force_torque.first.z <<
            //    " t: " << pair_vec_force_torque.second.x << " , " << pair_vec_force_torque.second.y << " , " <<
            //    pair_vec_force_torque.second.z << endl;
        }
    }
    csv_data.close();
}


void mujoco_flap::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(_m, _d);
        mj_forward(_m, _d);
    }
}

// mouse button callback
void mujoco_flap::mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mujoco_flap::mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(_m, action, dx / height, dy / height, &_scn, &_cam);
}

// scroll callback
void mujoco_flap::scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(_m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &_scn, &_cam);
}


void mujoco_flap::mycontroller_flap(const mjModel* m, mjData* d)
{
    // xml seting 0.0001 => 0.00005
    //std::cout << "control time: " << d->time << endl;
    const int n_nbody = 1;
    // update to corresponding force and torque;
    d->xfrc_applied[6 * n_nbody + 0] = 0;
    d->xfrc_applied[6 * n_nbody + 1] = 0;
    d->xfrc_applied[6 * n_nbody + 2] = 0;
    d->xfrc_applied[6 * n_nbody + 3] = 0;
    d->xfrc_applied[6 * n_nbody + 4] = 0;
    d->xfrc_applied[6 * n_nbody + 5] = 0;

    // take the force and torque into the corresponding body part and generate effects.
    if (abs(d->time - vec_d_timestamp_[i_idx]) <= 1e-3 && i_idx < force_torque_count) {
        //std::cout << "d->time: " << d->time << " , csv time" << vec_d_timestamp_[i_idx] << " idx " << i_idx <<  endl;
        d->xfrc_applied[6 * n_nbody + 0] = pair_vec_force_torque_[i_idx].first.x;
        d->xfrc_applied[6 * n_nbody + 1] = pair_vec_force_torque_[i_idx].first.y;
        d->xfrc_applied[6 * n_nbody + 2] = pair_vec_force_torque_[i_idx].first.z;
        d->xfrc_applied[6 * n_nbody + 3] = pair_vec_force_torque_[i_idx].second.x;
        d->xfrc_applied[6 * n_nbody + 4] = pair_vec_force_torque_[i_idx].second.y;
        d->xfrc_applied[6 * n_nbody + 5] = pair_vec_force_torque_[i_idx].second.z;
        //cout << pair_vec_force_torque_[i_idx].first.x << " , " << pair_vec_force_torque_[i_idx].first.y << " , " <<
        //    pair_vec_force_torque_[i_idx].first.z << " , " << pair_vec_force_torque_[i_idx].second.x << " , "
        //    << pair_vec_force_torque_[i_idx].second.y << " , " << pair_vec_force_torque_[i_idx].second.z << endl;
        i_idx++;
    }

    if (_loop_index % data_frequency == 0) // 保存数据  可以用这个调整大小100
    { // 0.005S time step ==> xml setting 0.0002
        save_data(m, d);// control and record corresponding data
        //pin_data.writeToFile(m, d);
    }
    _loop_index++;
}

void mujoco_flap::run_mujoco()
{
    // take csv data into corresponding controller
    // activate software
    mj_activate("mjkey.txt");
    // load and compile model
    char error[1000] = "Could not load binary model";
    _m = mj_loadXML(_filename, 0, error, 1000);
    if (!_m) mju_error_s("Load model error: %s", error);
    // make data
    _d = mj_makeData(_m);
    GLFWwindow* window = generate_window();
     //install control callback
    
    mjcb_control = mycontroller_flap;

    // check fid status; if open -> close first; 
    // else open and write things
    _fid = fopen(_datafile, "w");
    init_save_data();
    //pin_data.initFile();
    //glfwDestroyWindow(window); // 解决关闭仿真环境的效果 没有仿真环境
    // use the first while condition if you want to simulate for a period.
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = _d->time; // d->time global time
        while (_d->time - simstart < 1.0 / 100.0)
        {
            mj_step(_m, _d);
        }

        if (_d->time >= _simend)
        {
            fclose(_fid);
            //pin_data.close();
            break;
        }
        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        //// update scene and render
        mjv_updateScene(_m, _d, &_opt, NULL, &_cam, mjCAT_ALL, &_scn);
        mjr_render(viewport, &_scn, &_con);

        //// swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);
        //// process pending GUI events, call GLFW callbacks
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


void mujoco_flap::init_save_data()
{
    //write name of the variable here (header)
    fprintf(_fid, "t, ");
    fprintf(_fid, "angles,");
    fprintf(_fid, "angle_rates");
    //Don't remove the newline
    fprintf(_fid, "\n");
}

//***************************
//This function is called at a set frequency, put data here
void mujoco_flap::save_data(const mjModel* m, mjData* d)
{
    //data here should correspond to headers in init_save_data()
//seperate data by a space %f followed by space
    fprintf(_fid, "%f, ", d->time);
    fprintf(_fid, "%f, ", d->qpos[0]); // 只有在x軸有影响
    fprintf(_fid, "%f ", d->qvel[0]); // 只有在x軸影响
    fprintf(_fid, "\n");
}


mujoco_flap::mujoco_flap(char filename[],  char datafile[],  char forcefile[])
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
    _forcefile = forcefile;
    // first read time force torque data csv;
    read_force_data();
}


GLFWwindow* mujoco_flap::generate_window()
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
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = { 89.608063, -11.588379, 5, 0.000000, 0.000000, 2.000000 };
    _cam.azimuth = arr_view[0];
    _cam.elevation = arr_view[1];
    _cam.distance = arr_view[2];
    _cam.lookat[0] = arr_view[3];
    _cam.lookat[1] = arr_view[4];
    _cam.lookat[2] = arr_view[5];
    return window;
}