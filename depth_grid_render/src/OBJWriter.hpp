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
  OBJMesh() : computed_normals(false) {}
  
  void RecomputeNormals() {
    normals.resize(verts.size());
    int counts[verts.size()];
    for (int i=0; i<normals.size(); i++) {
      normals[i] = Vector3<T>::Zero(3);
    }
    for (int i=0; i<tris.size(); i++) {
      Eigen::Vector3i tri = tris[i];
      Vector3<T> v0 = GetVertex(tri(0));
      Vector3<T> v1 = GetVertex(tri(1));
      Vector3<T> v2 = GetVertex(tri(2));
      Vector3<T> n = (v1-v0).cross(v2-v1).normalized();
      normals[tri(0)-1] += n;
      counts[tri(0)-1] += 1;
      normals[tri(1)-1] += n;
      counts[tri(1)-1] += 1;
      normals[tri(2)-1] += n;
      counts[tri(2)-1] += 1;
    }
    for (int i=0; i<normals.size(); i++) {
      if (counts[i] != 0)
	normals[i] /= counts[i];
    }
    computed_normals = true;
  }
  
  const Vector3<T> &GetNormal(int index) const {
    if (!computed_normals) {
      RecomputeNormals();
    }
    return normals[index-1];
  }

  Vector3<T> &GetNormal(int index) {
    if (!computed_normals) {
      RecomputeNormals();
    }
    return normals[index-1];
  }

    void AddVertex(const Vector3<T> &v, const Vector2<T> &uv) {
        verts.push_back(v);
        uvs.push_back(uv);
	computed_normals = false;
    }

  void AddVertex(const Vector3<T> &v) {
    AddVertex(v, v.head(2));
    }

  size_t GetNumVertices() const {
    return verts.size();
  }

  size_t GetNumElements() const {
    return tris.size();
  }

    void AddTri(const Eigen::Vector3i &f) {
        tris.push_back(f);
	computed_normals = false;
    }
  void AddTri(int i1, int i2, int i3) {
    tris.emplace_back(i1, i2, i3);
    computed_normals=false;
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

  void SaveOBJ(std::string fname, bool flip_normals=false) const {
    std::ofstream of(fname);
    SaveOBJ(of, flip_normals);
    of.close();
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
    computed_normals = false;
  }

  void Transform(Eigen::Matrix<T, 4, 4> &transform) {
    Eigen::Matrix4Xf allverts(4, verts.size());
    Eigen::Matrix4Xf allnorms(4, normals.size());
      for (int i=0; i<verts.size(); i++) {
        allverts.col(i).head(3) = verts[i];
        allverts(3, i) = 1;
      }
      allverts = transform * allverts;
      for (int i=0; i<verts.size(); i++) {
          verts[i] = allverts.col(i).head(3);
      }
      for (int i=0; i<normals.size(); i++) {
	allnorms.col(i).head(3) = normals[i];
	allnorms(3,i) = 0;
      }
      allnorms = transform * allnorms;
      for (int i=0; i<normals.size(); i++) {
	normals[i] = allnorms.col(i).head(3);
      }
    }
  void TransformInv(Eigen::Matrix<T,4,4> &transform) {
    Eigen::Matrix4Xf allverts(4, verts.size());
    Eigen::Matrix4Xf allnorms(4, normals.size());
    for (int i=0; i<verts.size(); i++) {
      allverts.col(i).head(3) = verts[i];
      allverts(3,i) = 1;
    }
    auto fac = transform.colPivHouseholderQr();
    allverts = fac.solve(allverts);
    for (int i=0; i<verts.size(); i++) {
      verts[i] = allverts.col(i).head(3);
    }
    for (int i=0; i<normals.size(); i++) {
	allnorms.col(i).head(3) = normals[i];
	allnorms(3,i) = 0;
    }
    allnorms = fac.solve(allnorms);
    for (int i=0; i<normals.size(); i++) {
      normals[i] = allnorms.col(i).head(3);
    }
  }

private:
    std::vector<Vector3<T>> verts;
    std::vector<Vector2<T>> uvs;
    std::vector<Eigen::Vector3i> tris;
    bool computed_normals;
  std::vector<Vector3<T>> normals;
};

#endif
