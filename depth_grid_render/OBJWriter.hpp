#ifndef OBJ_WRITER_
#define OBJ_WRITER_

#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>

template<typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;
template<typename T>
using Vector2 = Eigen::Matrix<T, 2, 1>;

template <typename T>
class OBJMesh {
public:
    OBJMesh() {}

    void AddVertex(const Vector3<T> &v, const Vector2<T> &uv) {
        verts.push_back(v);
        uvs.push_back(uv);
    }

    void AddTri(const Eigen::Vector3i &f) {
        tris.push_back(f);
    }

    void SaveOBJ(std::ofstream &of) const {
        for (const auto &vert : verts) {
            of << "v " << vert[0] << " " << vert[1] << " " << vert[2] << std::endl;
        }
        for (const auto &uv : uvs) {
            of << "vt " << uv[0] << " " << uv[1] << std::endl;
        }
        of << "mtllib material.mtl" << std::endl;
        of << "usemtl textured" << std::endl;
        for (const auto &tri : tris) {
            of << "f";
            for (int i = 0; i < 3; i++) {
                of << " " << tri[i] << "/" << tri[i];
            }
            of << std::endl;
        }
        std::ofstream mf("material.mtl");
        mf << "newmtl textured" << std::endl;
        mf << "map_Kd texture.png" << std::endl;
    }

    Vector3<T>& GetVertex(int index) {
        return verts[index - 1];
    }

    const Vector3<T>& GetVertex(int index) const {
        return verts[index - 1];
    }

private:
    std::vector<Vector3<T>> verts;
    std::vector<Vector2<T>> uvs;
    std::vector<Eigen::Vector3i> tris;

};

#endif