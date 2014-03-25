#ifndef SCANRESULTSTRUCT_H
#define SCANRESULTSTRUCT_H

#include <vector>

typedef struct{
    std::vector<long> distances;
    std::vector<double> jointsValue;

    unsigned int number_of_joints;
    unsigned int number_of_scans;
    unsigned int number_of_points;
    unsigned long number_of_points_per_scan;

} RawScan3dResult;


typedef struct{
    std::vector<long> coordinates2d;
    std::vector<double> jointsValue;

    unsigned int number_of_joints;
    unsigned long number_of_scans;
    unsigned long number_of_points_per_scan;

} Scan3dResult;

#endif // SCANRESULTSTRUCT_H
