#pragma once
#include "Acceleration.h"
#include <vector>
#include <memory>
#include <thrust/device_vector.h>
#include <lbvh/lbvh.cuh>

class LBVH : public Acceleration {
public:
    LBVH();
    virtual ~LBVH();
    
    // 实现Acceleration的接口
    virtual void build() override;
    virtual bool rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const override;

    // 定义一个辅助结构，用于存储形状信息
    struct ShapeInfo {
        int shapeIndex;   // 在shapes数组中的索引
        int primitiveIndex; // 图元索引
        AABB bounds;      // 包围盒
        Point3f centroid; // 包围盒中心点
        
        // 添加默认构造函数
        ShapeInfo() : shapeIndex(-1), primitiveIndex(-1) {}
        
        // 添加拷贝构造函数
        ShapeInfo(const ShapeInfo& other) 
            : shapeIndex(other.shapeIndex), 
              primitiveIndex(other.primitiveIndex),
              bounds(other.bounds),
              centroid(other.centroid) {}
        
        // 添加赋值运算符
        ShapeInfo& operator=(const ShapeInfo& other) {
            if (this != &other) {
                shapeIndex = other.shapeIndex;
                primitiveIndex = other.primitiveIndex;
                bounds = other.bounds;
                centroid = other.centroid;
            }
            return *this;
        }
    };
    
private:
    // AABB获取器，用于lbvh库
    struct AABBGetter {
        __device__ __host__
        lbvh::aabb<float> operator()(const ShapeInfo& info) const noexcept {
            lbvh::aabb<float> box;
            box.lower = make_float4(info.bounds.pMin[0], info.bounds.pMin[1], info.bounds.pMin[2], 0.0f);
            box.upper = make_float4(info.bounds.pMax[0], info.bounds.pMax[1], info.bounds.pMax[2], 0.0f);
            return box;
        }
    };
    
    // 管理设备和主机内存
    std::vector<ShapeInfo> h_shapeInfos;  // 主机端形状信息
    
    // LBVH实例（仅在主机端初始化，存储完整结构）
    std::unique_ptr<lbvh::bvh<float, ShapeInfo, AABBGetter>> d_bvh;
    
    // CPU回退方案 - 线性遍历
    bool fallbackRayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const;
    
    // 新增: 使用设备内存向量构建BVH的辅助函数
    void buildWithDeviceVector();
    
    // 设备端内存管理
    bool initialized;
    bool useCPUFallback; // 是否使用CPU回退方案
}; 