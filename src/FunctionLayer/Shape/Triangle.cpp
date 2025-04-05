#include "Triangle.h"
#include <FunctionLayer/Acceleration/Linear.h>
#include <CoreLayer/Debug/Debug.h>

// 辅助函数：创建一个正交坐标系
static void createCoordinateSystem(const Vector3f &normal, Vector3f *tangent, Vector3f *bitangent) {
  if (fabs(normal[0]) > fabs(normal[1])) {
    *tangent = Vector3f(normal[2], 0, -normal[0]) / sqrtf(normal[0] * normal[0] + normal[2] * normal[2]);
  } else {
    *tangent = Vector3f(0, -normal[2], normal[1]) / sqrtf(normal[1] * normal[1] + normal[2] * normal[2]);
  }
  *bitangent = cross(normal, *tangent);
}

//--- Triangle ---
Triangle::Triangle(int _primID, int _vtx0Idx, int _vtx1Idx, int _vtx2Idx,
                   const TriangleMesh *_mesh)
    : primID(_primID), vtx0Idx(_vtx0Idx), vtx1Idx(_vtx1Idx), vtx2Idx(_vtx2Idx),
      mesh(_mesh) {
  Point3f vtx0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]),
          vtx1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]),
          vtx2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);
  boundingBox.Expand(vtx0);
  boundingBox.Expand(vtx1);
  boundingBox.Expand(vtx2);
  this->geometryID = mesh->geometryID;
}

bool Triangle::rayIntersectShape(Ray &ray, int *primID, float *u,
                                 float *v) const {
  //* todo 实现三角形与光线求交
  Point3f v0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]);
  Point3f v1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]);
  Point3f v2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);
  
  // 计算边向量
  Vector3f e1 = v1 - v0;
  Vector3f e2 = v2 - v0;
  
  // 计算三角形法线与光线方向的叉积
  Vector3f p = cross(ray.direction, e2);
  
  // 计算分母
  float det = dot(e1, p);
  
  // 如果det接近0，光线与三角形平行，没有交点
  constexpr float kEpsilon = 1e-8f;
  if (fabs(det) < kEpsilon) {
    return false;
  }
  
  float invDet = 1.0f / det;
  
  // 计算从v0到ray.origin的向量
  Vector3f t = ray.origin - v0;
  
  // 计算重心坐标u
  *u = dot(t, p) * invDet;
  
  // 如果u在[0,1]范围外，没有交点
  if (*u < 0.0f || *u > 1.0f) {
    return false;
  }
  
  // 计算q
  Vector3f q = cross(t, e1);
  
  // 计算重心坐标v
  *v = dot(ray.direction, q) * invDet;
  
  // 如果v在[0,1]范围外或u+v>1，没有交点
  if (*v < 0.0f || *u + *v > 1.0f) {
    return false;
  }
  
  // 计算t，即光线方向上的参数
  float distance = dot(e2, q) * invDet;
  
  // 如果t不在射线的有效范围内，没有交点
  if (distance < ray.tNear || distance > ray.tFar) {
    return false;
  }
  
  // 更新光线的参数范围
  ray.tFar = distance;
  
  // 填充primID
  *primID = this->primID;
  
  return true;
}

void Triangle::fillIntersection(float distance, int primID, float u, float v,
                                Intersection *intersection) const {
  // 该函数实际上不会被调用
  return;
}

//--- TriangleMesh ---
TriangleMesh::TriangleMesh(const Json &json) : Shape(json) {
  const auto &filepath = fetchRequired<std::string>(json, "file");
  meshData = MeshData::loadFromFile(filepath);
}

RTCGeometry TriangleMesh::getEmbreeGeometry(RTCDevice device) const {
  RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

  float *vertexBuffer = (float *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float),
      meshData->vertexCount);
  for (int i = 0; i < meshData->vertexCount; ++i) {
    Point3f vertex = transform.toWorld(meshData->vertexBuffer[i]);
    vertexBuffer[3 * i] = vertex[0];
    vertexBuffer[3 * i + 1] = vertex[1];
    vertexBuffer[3 * i + 2] = vertex[2];
  }

  unsigned *indexBuffer = (unsigned *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
      3 * sizeof(unsigned), meshData->faceCount);
  for (int i = 0; i < meshData->faceCount; ++i) {
    indexBuffer[i * 3] = meshData->faceBuffer[i][0].vertexIndex;
    indexBuffer[i * 3 + 1] = meshData->faceBuffer[i][1].vertexIndex;
    indexBuffer[i * 3 + 2] = meshData->faceBuffer[i][2].vertexIndex;
  }
  rtcCommitGeometry(geometry);
  return geometry;
}

bool TriangleMesh::rayIntersectShape(Ray &ray, int *primID, float *u,
                                     float *v) const {
  //* 当使用embree加速时，该方法不会被调用
  int geomID = -1;
  return acceleration->rayIntersect(ray, &geomID, primID, u, v);
}

void TriangleMesh::fillIntersection(float distance, int primID, float u,
                                    float v, Intersection *intersection) const {
  //* todo 填充光线与三角网格求交得到的交点信息
  intersection->distance = distance;
  intersection->shape = this;
  //* 1. 在三角形内部用插值计算交点坐标
  //* 2. 在三角形内部用插值计算法线
  //* 3. 在三角形内部用插值计算纹理坐标
  //* 4. 在三角形内部用插值计算交点的切线和副切线
  
  // 获取三角形的三个顶点索引
  int v0Idx = meshData->faceBuffer[primID][0].vertexIndex;
  int v1Idx = meshData->faceBuffer[primID][1].vertexIndex;
  int v2Idx = meshData->faceBuffer[primID][2].vertexIndex;
  
  // 获取顶点坐标
  Point3f v0 = transform.toWorld(meshData->vertexBuffer[v0Idx]);
  Point3f v1 = transform.toWorld(meshData->vertexBuffer[v1Idx]);
  Point3f v2 = transform.toWorld(meshData->vertexBuffer[v2Idx]);
  
  // 1. 使用重心坐标计算交点位置
  float w = 1.0f - u - v; // 第一个顶点的权重
  intersection->position = v0 + u * (v1 - v0) + v * (v2 - v0);
  
  // 2. 计算法线
  // 获取法线索引
  int n0Idx = meshData->faceBuffer[primID][0].normalIndex;
  int n1Idx = meshData->faceBuffer[primID][1].normalIndex;
  int n2Idx = meshData->faceBuffer[primID][2].normalIndex;
  
  // 如果法线索引有效，使用重心坐标插值计算法线
  if (n0Idx >= 0 && n1Idx >= 0 && n2Idx >= 0) {
    Vector3f n0 = transform.toWorld(meshData->normalBuffer[n0Idx]);
    Vector3f n1 = transform.toWorld(meshData->normalBuffer[n1Idx]);
    Vector3f n2 = transform.toWorld(meshData->normalBuffer[n2Idx]);
    intersection->normal = normalize(w * n0 + u * n1 + v * n2);
  } else {
    // 如果没有法线数据，计算三角形的几何法线
    Vector3f e1 = v1 - v0;
    Vector3f e2 = v2 - v0;
    intersection->normal = normalize(cross(e1, e2));
  }
  
  // 3. 计算纹理坐标
  // 获取纹理坐标索引
  int t0Idx = meshData->faceBuffer[primID][0].texcodIndex;
  int t1Idx = meshData->faceBuffer[primID][1].texcodIndex;
  int t2Idx = meshData->faceBuffer[primID][2].texcodIndex;
  
  // 如果纹理坐标索引有效，使用重心坐标插值计算纹理坐标
  if (t0Idx >= 0 && t1Idx >= 0 && t2Idx >= 0) {
    Vector2f t0 = meshData->texcodBuffer[t0Idx];
    Vector2f t1 = meshData->texcodBuffer[t1Idx];
    Vector2f t2 = meshData->texcodBuffer[t2Idx];
    
    intersection->texCoord = w * t0 + u * t1 + v * t2;
  } else {
    // 如果没有纹理坐标数据，使用默认值
    intersection->texCoord = Vector2f(0.0f, 0.0f);
  }
  
  // 4. 计算切线和副切线
  if (t0Idx >= 0 && t1Idx >= 0 && t2Idx >= 0) {
    // 计算切线空间
    Vector2f t0 = meshData->texcodBuffer[t0Idx];
    Vector2f t1 = meshData->texcodBuffer[t1Idx];
    Vector2f t2 = meshData->texcodBuffer[t2Idx];
    
    // 计算三角形的边
    Vector3f e1 = v1 - v0;
    Vector3f e2 = v2 - v0;
    
    // 计算纹理坐标的差异
    float du1 = t1[0] - t0[0];
    float dv1 = t1[1] - t0[1];
    float du2 = t2[0] - t0[0];
    float dv2 = t2[1] - t0[1];
    
    // 解方程组求切线和副切线
    float det = du1 * dv2 - dv1 * du2;
    if (fabs(det) > 1e-8f) {
      float invDet = 1.0f / det;
      intersection->tangent = normalize((dv2 * e1 - dv1 * e2) * invDet);
      intersection->bitangent = normalize((-du2 * e1 + du1 * e2) * invDet);
    } else {
      // 如果无法计算切线空间，创建一个坐标系
      createCoordinateSystem(intersection->normal, &intersection->tangent, &intersection->bitangent);
    }
  } else {
    // 如果没有纹理坐标数据，创建一个坐标系
    createCoordinateSystem(intersection->normal, &intersection->tangent, &intersection->bitangent);
  }
  
  // 计算dpdu和dpdv
  intersection->dpdu = intersection->tangent;
  intersection->dpdv = intersection->bitangent;
}

void TriangleMesh::initInternalAcceleration() {
  acceleration = Acceleration::createAcceleration();
  int primCount = meshData->faceCount;
  for (int primID = 0; primID < primCount; ++primID) {
    int vtx0Idx = meshData->faceBuffer[primID][0].vertexIndex,
        vtx1Idx = meshData->faceBuffer[primID][1].vertexIndex,
        vtx2Idx = meshData->faceBuffer[primID][2].vertexIndex;
    std::shared_ptr<Triangle> triangle =
        std::make_shared<Triangle>(primID, vtx0Idx, vtx1Idx, vtx2Idx, this);
    acceleration->attachShape(triangle);
  }
  acceleration->build();
  // TriangleMesh的包围盒就是其内部加速结构的包围盒
  boundingBox = acceleration->boundingBox;
}
REGISTER_CLASS(TriangleMesh, "triangle")