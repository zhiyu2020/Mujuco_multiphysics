#include "mojuco_dbpendulum.h"

// init static class parameters
mjModel* dbpendulum::_m = NULL;                  // MuJoCo model
mjData* dbpendulum::_d = NULL;                   // MuJoCo data
mjvCamera dbpendulum::_cam;                      // abstract camera
mjvOption dbpendulum::_opt;                      // visualization options
mjvScene dbpendulum::_scn;                       // abstract scene
mjrContext dbpendulum::_con;                     // custom GPU context

// mouse interaction
 bool dbpendulum::button_left = false;
 bool dbpendulum::button_middle = false;
 bool dbpendulum::button_right = false;
 double dbpendulum::lastx = 0;
 double dbpendulum::lasty = 0;

 void dbpendulum::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
 {
     // backspace: reset simulation
     if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
     {
         mj_resetData(_m, _d);
         mj_forward(_m, _d);
     }
 }

 // mouse button callback
 void dbpendulum::mouse_button(GLFWwindow* window, int button, int act, int mods)
 {
     // update button state
     button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
     button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
     button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

     // update mouse position
     glfwGetCursorPos(window, &lastx, &lasty);
 }

 // mouse move callback
 void dbpendulum::mouse_move(GLFWwindow* window, double xpos, double ypos)
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
 void dbpendulum::scroll(GLFWwindow* window, double xoffset, double yoffset)
 {
     // emulate vertical mouse motion = 5% of window height
     mjv_moveCamera(_m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &_scn, &_cam);
 }

 //**************************
 void dbpendulum::mycontroller(const mjModel* m, mjData* d)
 {
     //write control here
     mj_energyPos(m, d);
     mj_energyVel(m, d);
     //printf("%f %f %f %f \n",d->time,d->energy[0],d->energy[1],d->energy[0]+d->energy[1]);

     //check equations
     //M*qacc + qfrc_bias = qfrc_applied + ctrl
     //M*qddot + f = qfrc_applied + ctrl
     const int ndof = 2;
     double dense_M[ndof * ndof] = { 0 };
     mj_fullM(m, dense_M, d->qM);
     double M[ndof][ndof] = { 0 };
     M[0][0] = dense_M[0];
     M[0][1] = dense_M[1];
     M[1][0] = dense_M[2];
     M[1][1] = dense_M[3];
     // printf("%f %f \n",M[0][0],M[0][1]);
     // printf("%f %f \n",M[1][0],M[1][1]);
     // printf("******** \n");

     double qddot[ndof] = { 0 };
     qddot[0] = d->qacc[0];
     qddot[1] = d->qacc[1];

     double f[ndof] = { 0 };
     f[0] = d->qfrc_bias[0];
     f[1] = d->qfrc_bias[1];

     double lhs[ndof] = { 0 };
     mju_mulMatVec(lhs, dense_M, qddot, 2, 2); //lhs = M*qddot
     lhs[0] = lhs[0] + f[0]; //lhs = M*qddot + f
     lhs[1] = lhs[1] + f[1];

     //d->qfrc_applied[0] = 0.1 * f[0];
     d->qfrc_applied[0] =  0.05 * f[0];
     //d->qfrc_applied[1] = 0.5 * f[1];
     d->qfrc_applied[1] =  0.01 * f[1];
     //d->qfrc_applied[0] = f[0];
     //d->qfrc_applied[1] = f[1];

     double rhs[ndof] = { 0 };
     rhs[0] = d->qfrc_applied[0];
     rhs[1] = d->qfrc_applied[1];

     // printf("%f %f \n",lhs[0], rhs[0]);
     // printf("%f %f \n",lhs[1], rhs[1]);
     // printf("******\n");

     //control
     double Kp1 = 100, Kp2 = 100;
     double Kv1 = 10, Kv2 = 10;
     double qref1 = -0.5, qref2 = -1.6;

     //PD control
     // d->qfrc_applied[0] = -Kp1*(d->qpos[0]-qref1)-Kv1*d->qvel[0];
     // d->qfrc_applied[1] = -Kp2*(d->qpos[1]-qref2)-Kv2*d->qvel[1];

     //coriolis + gravity + PD control
     // d->qfrc_applied[0] = f[0]-Kp1*(d->qpos[0]-qref1)-Kv1*d->qvel[0];
     // d->qfrc_applied[1] = f[1]-Kp2*(d->qpos[1]-qref2)-Kv2*d->qvel[1];

     //Feedback linearization
     //M*(-kp( ... ) - kv(...) + f)
     //double tau[2] = { 0 };
     //tau[0] = -Kp1 * (d->qpos[0] - qref1) - Kv1 * d->qvel[0];
     //tau[1] = -Kp2 * (d->qpos[1] - qref2) - Kv2 * d->qvel[1];

     //mju_mulMatVec(tau, dense_M, tau, 2, 2); //lhs = M*tau
     //tau[0] += f[0];
     //tau[1] += f[1];
     //d->qfrc_applied[0] = tau[0];
     //d->qfrc_applied[1] = tau[1];
 }

 void dbpendulum::run_mujoco()
 {
     // activate software
     mj_activate("mjkey.txt");
     // load and compile model
     char error[1000] = "Could not load binary model";
     _m = mj_loadXML(_filename, 0, error, 1000);
     // make data
     _d = mj_makeData(_m);
     GLFWwindow* window = generate_window();
     // install control callback
     mjcb_control = mycontroller;
     _d->qpos[0] = 0.5;
     //d->qpos[1] = 0;
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
         if (_d->time >= _simend)
         {
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

 dbpendulum::dbpendulum(char filename[])
 {
     _filename = filename;
 }


 GLFWwindow* dbpendulum::generate_window()
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