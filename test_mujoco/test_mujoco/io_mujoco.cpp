#include "io_mujoco.h"

WriteMuJocoPinData::WriteMuJocoPinData():
    filepath_("mb_pinbody_data.dat")
{
}

void WriteMuJocoPinData::initFile()
{
    std::ofstream out_file(filepath_.c_str(), std::ios::app);
    out_file << "\"time\""
        << "   ";
    out_file << "  "
        << "angles"
        << " ";
    out_file << "  "
        << "angle_rates"
        << " ";
    out_file << "\n";
    //out_file << "\"time\""
    //    << "   ";
    //out_file << "  "
    //    << "q1"
    //    << " ";
    //out_file << "  "
    //    << "q2"
    //    << " ";
    //out_file << "\n";
    out_file.close();
}

void WriteMuJocoPinData::writeToFile(const mjModel* m, mjData* d)
{
    std::ofstream out_file(filepath_.c_str(), ios::app);
    // for now test only
    out_file << d->time << "   ";
    //out_file << "  " << d->sensordata[0] << "  " << d->sensordata[2] << "  ";
    out_file << "  " << d->qpos[0] << "  " << d->qvel[0] << "  ";
    out_file << "\n";
}

void WriteMuJocoPinData::close()
{
    std::ofstream out_file(filepath_.c_str(), std::ios::app);
    out_file.close();
}