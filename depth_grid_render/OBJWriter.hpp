#ifndef OBJ_WRITER_
#define OBJ_WRITER_

#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>

template<typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
class OBJMesh {
public:
    OBJMesh() {}

    void AddVertex(const Vector3<T> &v) {
        verts.push_back(v);
    }

    void AddTri(const Eigen::Vector3i &f) {
        tris.push_back(f);
    }

    void SaveOBJ(std::ofstream &of) const {
        for (const auto &vert : verts) {
            of << "v " << vert[0] << " " << vert[1] << " " << vert[2] << std::endl;
        }
        for (const auto &tri : tris) {
            of << "f " << tri[0] << " " << tri[1] << " " << tri[2] << std::endl;
        }
    }

    Vector3<T>& GetVertex(int index) {
        return verts[index - 1];
    }

    const Vector3<T>& GetVertex(int index) const {
        return verts[index - 1];
    }

private:
    std::vector<Vector3<T>> verts;
    std::vector<Eigen::Vector3i> tris;

};

#endif