#ifndef OBJ_WRITER_
#define OBJ_WRITER_

#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include "typedefs.hpp"

template<typename T>
struct YTest {
  YTest(T val, std::vector<Vector3<T>> &vertices, bool strict, T ang=90) : val_(val), vertices_(vertices), strict_(strict) {
    tan2ang_ = tan(ang/360*M_PI);
    tan2ang_*=tan2ang_;
  }
  bool operator()(Eigen::Vector3i &element) {
      bool pred = strict_;
    for (int i=0; i<3; i++) {
      if (!strict_ && vertices_[element[i]-1][1] < val_) { //if not strict, true if any vertex below threshold
	    return true;
      } else if (strict_ && vertices_[element[i]-1][1] >= val_) { //if strict, true only if all vertices are below threshold
          return false;
      }
    }
    if (pred) {
      Vector3<T> e1 = vertices_[element[1]-1] - vertices_[element[0]-1];
      Vector3<T> e2 = vertices_[element[2]-1] - vertices_[element[1]-1];
      Vector3<T> n = e1.cross(e2);
      n.normalize();
      if ((n[0]*n[0]+n[2]*n[2])/(n[1]*n[1]) < tan2ang_) {
	return true;
      }
    }
    return false;
  }
private:
  T val_;
  bool strict_;
  T tan2ang_;
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

  void SaveOBJ(std::ofstream &of, bool flip_normals=false) const {
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
	    if (flip_normals) {
	      for (int i=2; i >= 0; i--) {
		of << " " << tri[i] << "/" << tri[i];
	      }
	    } else {
	      for (int i = 0; i < 3; i++) {
                of << " " << tri[i] << "/" << tri[i];
	      }
	    }
            of << std::endl;
        }
        std::ofstream mf("material.mtl");
        mf << "newmtl textured" << std::endl;
        mf << "map_Kd texture.exr" << std::endl;
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

  void DeleteBelowY(T threshold, bool strict = true, float angle = 90) {
    YTest<T> cond(threshold, verts, strict, angle);
    auto search = std::remove_if(tris.begin(), tris.end(), cond);
    tris.erase(search, tris.end());
  }

  void Transform(Eigen::Matrix<T, 4, 4> &transform) {
    Eigen::Matrix4Xf allverts(4, verts.size());
      for (int i=0; i<verts.size(); i++) {
        allverts.col(i).head(3) = verts[i];
        allverts(3, i) = 1;
      }
      allverts = transform * allverts;
      for (int i=0; i<verts.size(); i++) {
          verts[i] = allverts.col(i).head(3);
      }
    }

private:
    std::vector<Vector3<T>> verts;
    std::vector<Vector2<T>> uvs;
    std::vector<Eigen::Vector3i> tris;

};

#endif
