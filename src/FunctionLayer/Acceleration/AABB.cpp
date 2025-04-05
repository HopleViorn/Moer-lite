#include "AABB.h"

Point3f minP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::min(p1[0], p2[0]), std::min(p1[1], p2[1]),
                 std::min(p1[2], p2[2])};
}

Point3f maxP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::max(p1[0], p2[0]), std::max(p1[1], p2[1]),
                 std::max(p1[2], p2[2])};
}

AABB AABB::Union(const AABB &other) const {
  Point3f min = minP(other.pMin, pMin), max = maxP(other.pMax, pMax);
  return AABB{min, max};
}

void AABB::Expand(const AABB &other) {
  pMin = minP(pMin, other.pMin);
  pMax = maxP(pMax, other.pMax);
}

AABB AABB::Union(const Point3f &other) const {
  Point3f min = minP(other, pMin), max = maxP(other, pMax);
  return AABB{min, max};
}

void AABB::Expand(const Point3f &other) {
  pMin = minP(pMin, other);
  pMax = maxP(pMax, other);
}

bool AABB::Overlap(const AABB &other) const {
  for (int dim = 0; dim < 3; ++dim) {
    if (pMin[dim] > other.pMax[dim] || pMax[dim] < other.pMin[dim]) {
      return false;
    }
  }
  return true;
}

bool AABB::RayIntersect(const Ray &ray, float *tMin, float *tMax) const {
  float t0 = ray.tNear;
  float t1 = ray.tFar;
  
  for (int i = 0; i < 3; ++i) {
    // 计算与轴对齐平面的交点参数
    float invD = 1.0f / ray.direction[i];

    float tNear = (pMin[i] - ray.origin[i]) * invD;
    float tFar = (pMax[i] - ray.origin[i]) * invD;

    // 处理方向分量为负的情况
    if (invD < 0.0f) std::swap(tNear, tFar);
    
    // 处理非常薄的AABB，增加一个小的epsilon容差
    const float epsilon = 1e-5f;
    // if (fabs(pMax[i] - pMin[i]) < epsilon) {
    //   // 对于极薄的维度，增加一点容差避免数值问题
    // }
    tNear -= epsilon;
    tFar += epsilon;
    
    // 更新交点区间
    t0 = tNear > t0 ? tNear : t0;
    t1 = tFar < t1 ? tFar : t1;
    
    // 如果区间为空，表示不相交
    if (t0 > t1) return false;
  }
  
  // 如果区间不为空，设置输出参数并返回相交结果
  if (tMin) *tMin = t0;
  if (tMax) *tMax = t1;
  
  return true;
}

Point3f AABB::Center() const {
  return Point3f{(pMin[0] + pMax[0]) * .5f, (pMin[1] + pMax[1]) * .5f,
                 (pMin[2] + pMax[2]) * .5f};
}