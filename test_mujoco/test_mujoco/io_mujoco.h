#pragma once
#include <iostream>
#include <string>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

using namespace std;

class WriteMuJocoPinData {
public:
	WriteMuJocoPinData();
	void initFile();
	void writeToFile(const mjModel* m, mjData* d);
	void close();
private:
	string filepath_;
};