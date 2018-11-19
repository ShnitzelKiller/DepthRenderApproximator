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

template<typename T>
struct YTest {
  YTest(T val, std::vector<Vector3<T>> &vertices, bool strict) : val_(val), vertices_(vertices), strict_(strict) {}
  bool operator()(Eigen::Vector3i element) {
      bool pred = strict_;
    for (int i=0; i<3; i++) {
      if (!strict_ && vertices_[element[i]-1][1] < val_) { //if not strict, true if any vertex below threshold
	    return true;
      } else if (strict_ && vertices_[element[i]-1][1] >= val_) { //if strict, true only if all vertices are below threshold
          return false;
      }
    }
    return pred;
  }
private:
  T val_;
  bool strict_;
  std::vector<Vector3<T>> &vertices_;
};

template <typename T>
class OBJMesh {
public:
    OBJMesh() {}

    void AddVertex(const Vector3<T> &v, const Vector2<T> &uv) {
        verts.push_back(v);
        uvs.push_back(uv);
    }

  size_t GetNumVertices() const {
    return verts.size();
  }

  size_t GetNumElements() const {
    return tris.size();
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

  Eigen::Vector3i& GetElement(int index) {
    return tris[index-1];
  }
  const Eigen::Vector3i& GetElement(int index) const {
    return tris[index-1];
  }

  void DeleteBelowY(T threshold, bool strict = true) {
    YTest<T> cond(threshold, verts, strict);
    auto search = std::remove_if(tris.begin(), tris.end(), cond);
    tris.erase(search, tris.end());
  }

private:
    std::vector<Vector3<T>> verts;
    std::vector<Vector2<T>> uvs;
    std::vector<Eigen::Vector3i> tris;

};

#endif
