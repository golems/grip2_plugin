#include "objsavefile.h"

ObjSaveFile::ObjSaveFile()
{
}

int ObjSaveFile::save(const std::string &file_name, const pcl::PolygonMesh &mesh, unsigned precision)
{

    if (mesh.cloud.data.empty ())
    {
        PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
        return (-1);
    }
    // Open file
    std::ofstream fs;
    fs.precision (precision);
    fs.open (file_name.c_str ());

    /* Write 3D information */
    // number of points
    int nr_points = mesh.cloud.width * mesh.cloud.height;
    // point size
    unsigned point_size = static_cast<unsigned> (mesh.cloud.data.size () / nr_points);
    // number of faces for header
    unsigned nr_faces = static_cast<unsigned> (mesh.polygons.size ());
    // Do we have vertices normals?
    int normal_index = getFieldIndex (mesh.cloud, "normal_x");

    // Write the header information
    fs << "####" << std::endl;
    fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
    fs << "# Vertices: " << nr_points << std::endl;
    if (normal_index != -1)
        fs << "# Vertices normals : " << nr_points << std::endl;
    fs << "# Faces: " <<nr_faces << std::endl;
    fs << "####" << std::endl;

    // Write vertex coordinates
    fs << "# List of Vertices, with (x,y,z) coordinates, w is optional." << std::endl;
    for (int i = 0; i < nr_points; ++i)
    {
        int xyz = 0;
        for (size_t d = 0; d < mesh.cloud.fields.size (); ++d)
        {
            int c = 0;
            // adding vertex
            if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                        mesh.cloud.fields[d].name == "x" ||
                        mesh.cloud.fields[d].name == "y" ||
                        mesh.cloud.fields[d].name == "z"))
            {
                if (mesh.cloud.fields[d].name == "x")
                    // write vertices beginning with v
                    fs << "v ";

                float value;
                memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
                fs << value;
                if (++xyz == 3)
                    break;
                fs << " ";
            }
        }
        if (xyz != 3)
        {
            PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
            return (-2);
        }
        fs << std::endl;
    }

    fs << "# "<< nr_points <<" vertices" << std::endl;

    if(normal_index != -1)
    {
        fs << "# Normals in (x,y,z) form; normals might not be unit." << std::endl;
        // Write vertex normals
        for (int i = 0; i < nr_points; ++i)
        {
            int nxyz = 0;
            for (size_t d = 0; d < mesh.cloud.fields.size (); ++d)
            {
                int c = 0;
                // adding vertex
                if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                            mesh.cloud.fields[d].name == "normal_x" ||
                            mesh.cloud.fields[d].name == "normal_y" ||
                            mesh.cloud.fields[d].name == "normal_z"))
                {
                    if (mesh.cloud.fields[d].name == "normal_x")
                        // write vertices beginning with vn
                        fs << "vn ";

                    float value;
                    memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
                    fs << value;
                    if (++nxyz == 3)
                        break;
                    fs << " ";
                }
            }
            if (nxyz != 3)
            {
                PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
                return (-2);
            }
            fs << std::endl;
        }

        fs << "# "<< nr_points <<" vertices normals" << std::endl;
    }

    fs << "# Face Definitions" << std::endl;
    // Write down faces
    if(normal_index == -1)
    {
        for(unsigned i = 0; i < nr_faces; i++)
        {
            fs << "f ";
            size_t j = 0;
            for (; j < mesh.polygons[i].vertices.size () - 1; ++j)
                fs << mesh.polygons[i].vertices[j] + 1 << " ";
            fs << mesh.polygons[i].vertices[j] + 1 << std::endl;
        }
    }
    else
    {
        for(unsigned i = 0; i < nr_faces; i++)
        {
            fs << "f ";
            size_t j = 0;
            for (; j < mesh.polygons[i].vertices.size () - 1; ++j)
                fs << mesh.polygons[i].vertices[j] + 1 << "//" << mesh.polygons[i].vertices[j] + 1 << " ";
            fs << mesh.polygons[i].vertices[j] + 1 << "//" << mesh.polygons[i].vertices[j] + 1 << std::endl;
        }
    }
    fs << "# End of File" << std::endl;

    // Close obj file
    fs.close ();
    return 0;
}
