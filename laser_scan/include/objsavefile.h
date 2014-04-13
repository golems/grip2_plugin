#ifndef OBJSAVEFILE_H
#define OBJSAVEFILE_H

#include <string>
#include <pcl/PolygonMesh.h>
#include <fstream>
#include <pcl/common/io.h>

class ObjSaveFile
{
public:
    ObjSaveFile();
    static int save(const std::string &file_name, const pcl::PolygonMesh &mesh, unsigned precision = 5);
};

#endif // OBJSAVEFILE_H
