#include "BVH.h"
#include <algorithm>
#include <vector>
#include <functional>

struct BVH::BVHNode {
    AABB box;        // 节点的包围盒
    int firstPrimOffset;  // 叶子节点包含的第一个图元在图元数组中的位置
    int nPrimitives;      // 叶子节点所包含的图元数量
    int rightChildOffset; // 该节点的右子节点相对于当前节点的偏移量
    
    // 判断是否为叶子节点
    bool isLeaf() const { return nPrimitives > 0; }
};

// 用于计算AABB的表面积
inline float computeSurfaceArea(const AABB& box) {
    Vector3f diag = box.pMax - box.pMin;
    return 2.0f * (diag[0] * diag[1] + diag[0] * diag[2] + diag[1] * diag[2]);
}

// 析构函数释放BVH节点内存
BVH::~BVH() {
    if (root) {
        delete[] root;
        root = nullptr;
    }
}

void BVH::build() {
    AABB sceneBox;
    for (const auto & shape : shapes) {
        //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
        //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
        //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
        //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
        shape->initInternalAcceleration();
        sceneBox.Expand(shape->getAABB());
    }
    
    boundingBox = sceneBox;
    
    // 如果没有图元，返回
    if (shapes.empty()) return;
    
    // 构建BVH所需的基本信息
    struct PrimitiveInfo {
        PrimitiveInfo() {}
        PrimitiveInfo(int shapeIndex) : shapeIndex(shapeIndex) {}
        
        int shapeIndex;  // 对应shapes数组中的索引
        AABB bounds;     // 图元的AABB
        Point3f centroid; // 图元包围盒的中心
    };
    
    std::vector<PrimitiveInfo> primitiveInfo(shapes.size());
    for (int i = 0; i < shapes.size(); ++i) {
        primitiveInfo[i] = PrimitiveInfo(i);
        primitiveInfo[i].bounds = shapes[i]->getAABB();
        primitiveInfo[i].centroid = primitiveInfo[i].bounds.Center();
    }
    
    // 递归构建BVH树
    int totalNodes = 0;
    std::vector<std::shared_ptr<Shape>> orderedShapes;
    
    // 为BVH树分配内存
    std::vector<BVHNode> nodes;
    nodes.reserve(2 * shapes.size() - 1);
    
    // SAH相关常量
    const int nBuckets = 12;  // 用于SAH的分桶数量
    const float traversalCost = 1.0f;  // 遍历节点的开销
    const float intersectionCost = 1.5f;  // 与图元相交的开销
    
    // 递归构建函数
    std::function<int(int, int)> recursiveBuild = 
        [&](int start, int end) -> int {
            int nodeIdx = nodes.size();
            nodes.emplace_back();
            BVHNode& node = nodes.back();
            
            // 计算当前节点包含所有图元的AABB
            AABB bounds;
            for (int i = start; i < end; ++i)
                bounds.Expand(primitiveInfo[i].bounds);
            
            int nPrimitives = end - start;
            
            if (nPrimitives <= bvhLeafMaxSize) {
                // 创建叶子节点
                int firstPrimOffset = orderedShapes.size();
                for (int i = start; i < end; ++i) {
                    int shapeIndex = primitiveInfo[i].shapeIndex;
                    orderedShapes.push_back(shapes[shapeIndex]);
                }
                
                node.firstPrimOffset = firstPrimOffset;
                node.nPrimitives = nPrimitives;
                node.box = bounds;
                node.rightChildOffset = 0;
            } else {
                // 计算所有图元中心的AABB
                AABB centroidBounds;
                for (int i = start; i < end; ++i)
                    centroidBounds.Expand(primitiveInfo[i].centroid);
                
                // 选择最长的轴作为候选分割轴
                int dim = 0;
                Vector3f diag = centroidBounds.pMax - centroidBounds.pMin;
                if (diag[1] > diag[0]) dim = 1;
                if (diag[2] > diag[dim]) dim = 2;
                
                // 如果包围盒在某维度上是扁平的，则创建叶子节点
                if (centroidBounds.pMax[dim] == centroidBounds.pMin[dim]) {
                    int firstPrimOffset = orderedShapes.size();
                    for (int i = start; i < end; ++i) {
                        int shapeIndex = primitiveInfo[i].shapeIndex;
                        orderedShapes.push_back(shapes[shapeIndex]);
                    }
                    
                    node.firstPrimOffset = firstPrimOffset;
                    node.nPrimitives = nPrimitives;
                    node.box = bounds;
                    node.rightChildOffset = 0;
                } else {
                    // 使用SAH启发式方法找到最佳分割
                    float minCost = std::numeric_limits<float>::max();
                    int minCostSplitBucket = 0;
                    int bestDim = dim;
                    
                    // 尝试所有三个维度作为分割轴
                    for (int d = 0; d < 3; ++d) {
                        // 如果这个维度平坦，跳过
                        if (centroidBounds.pMax[d] == centroidBounds.pMin[d])
                            continue;
                        
                        // 初始化桶
                        struct BucketInfo {
                            int count = 0;
                            AABB bounds;
                        };
                        BucketInfo buckets[nBuckets];
                        
                        // 将图元分配到桶中
                        for (int i = start; i < end; ++i) {
                            int b = nBuckets * ((primitiveInfo[i].centroid[d] - centroidBounds.pMin[d]) /
                                             (centroidBounds.pMax[d] - centroidBounds.pMin[d]));
                            if (b == nBuckets) b = nBuckets - 1;
                            buckets[b].count++;
                            buckets[b].bounds.Expand(primitiveInfo[i].bounds);
                        }
                        
                        // 计算每个可能分割的SAH代价
                        float cost[nBuckets - 1];
                        for (int i = 0; i < nBuckets - 1; ++i) {
                            AABB b0, b1;
                            int count0 = 0, count1 = 0;
                            
                            for (int j = 0; j <= i; ++j) {
                                b0.Expand(buckets[j].bounds);
                                count0 += buckets[j].count;
                            }
                            
                            for (int j = i + 1; j < nBuckets; ++j) {
                                b1.Expand(buckets[j].bounds);
                                count1 += buckets[j].count;
                            }
                            
                            cost[i] = 1.0f + (count0 * computeSurfaceArea(b0) + count1 * computeSurfaceArea(b1)) / 
                                     computeSurfaceArea(bounds);
                            
                            if (cost[i] < minCost) {
                                minCost = cost[i];
                                minCostSplitBucket = i;
                                bestDim = d;
                            }
                        }
                    }

                    // 当前节点的SAH代价
                    float oldCost = (float)nPrimitives;
                    
                    // 如果使用SAH划分不划算，则创建叶子节点
                    if (nPrimitives > 4 && minCost < oldCost) {
                        // 根据SAH进行划分
                        auto midIter = std::partition(&primitiveInfo[start], &primitiveInfo[end],
                                [=](const PrimitiveInfo& pi) {
                                    int b = nBuckets * ((pi.centroid[bestDim] - centroidBounds.pMin[bestDim]) /
                                                     (centroidBounds.pMax[bestDim] - centroidBounds.pMin[bestDim]));
                                    if (b == nBuckets) b = nBuckets - 1;
                                    return b <= minCostSplitBucket;
                                });
                        
                        int mid = midIter - &primitiveInfo[0];
                        
                        // 如果划分失败(全部在一边)，则退化为中点划分
                        if (mid == start || mid == end) {
                            mid = (start + end) / 2;
                            std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
                                       &primitiveInfo[end-1]+1,
                                       [bestDim](const PrimitiveInfo &a, const PrimitiveInfo &b) {
                                           return a.centroid[bestDim] < b.centroid[bestDim];
                                       });
                        }
                        
                        // 递归创建子节点
                        node.box = bounds;
                        node.firstPrimOffset = 0;
                        node.nPrimitives = 0;
                        
                        int leftChildIdx = recursiveBuild(start, mid);
                        int rightChildIdx = recursiveBuild(mid, end);
                        
                        // 右子节点的偏移量是相对于左子节点的
                        node.rightChildOffset = rightChildIdx - leftChildIdx;
                    } else {
                        // 如果SAH代价不划算，创建叶子节点
                        int firstPrimOffset = orderedShapes.size();
                        for (int i = start; i < end; ++i) {
                            int shapeIndex = primitiveInfo[i].shapeIndex;
                            orderedShapes.push_back(shapes[shapeIndex]);
                        }
                        
                        node.firstPrimOffset = firstPrimOffset;
                        node.nPrimitives = nPrimitives;
                        node.box = bounds;
                        node.rightChildOffset = 0;
                    }
                }
            }
            
            return nodeIdx;
        };
    
    // 从根节点开始构建BVH树
    recursiveBuild(0, primitiveInfo.size());
    
    // 分配连续内存并复制节点数据
    if (!nodes.empty()) {
        // 释放旧内存（如果存在）
        if (root) delete[] root;
        
        // 分配新内存并复制节点
        root = new BVHNode[nodes.size()];
        for (size_t i = 0; i < nodes.size(); ++i) {
            root[i] = nodes[i];
        }
    }
    
    // 替换shapes数组为排序后的数组
    shapes.swap(orderedShapes);
}

bool BVH::rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const {
    if (!root) return false;
    
    bool hit = false;
    float tMin, tMax;
    
    // 如果光线与场景包围盒不相交，直接返回
    if (!boundingBox.RayIntersect(ray, &tMin, &tMax)) 
        return false;
    
    // 迭代版本的BVH遍历，避免递归开销
    struct BVHTraversal {
        const BVHNode* node;
        float tMin, tMax;
    };
    
    BVHTraversal stack[64];
    int stackPtr = 0;
    
    stack[stackPtr].node = root;
    stack[stackPtr].tMin = tMin;
    stack[stackPtr].tMax = tMax;
    stackPtr++;
    
    while (stackPtr > 0) {
        const BVHTraversal& top = stack[stackPtr - 1];
        stackPtr--;
        
        const BVHNode* node = top.node;
        float rayTMin = top.tMin;
        float rayTMax = top.tMax;
        
        // 如果当前节点与光线不相交，则跳过
        if (rayTMin > rayTMax) continue;
        
        if (node->isLeaf()) {
            // 对叶子节点中的图元进行求交
            for (int i = 0; i < node->nPrimitives; ++i) {
                int shapeIdx = i + node->firstPrimOffset;
                int localPrimID;
                float localU, localV;
                
                if (shapes[shapeIdx]->rayIntersectShape(ray, &localPrimID, &localU, &localV)) {
                    if (ray.tFar < rayTMax) {
                        hit = true;
                        *geomID = shapeIdx;
                        *primID = localPrimID;
                        *u = localU;
                        *v = localV;
                    }
                }
            }
        } else {
            // 获取子节点
            const BVHNode* leftChild = node + 1;
            const BVHNode* rightChild = node + 1 + node->rightChildOffset;
            
            // 计算与子节点的相交情况
            float t1Min, t1Max, t2Min, t2Max;
            bool hit1 = leftChild->box.RayIntersect(ray, &t1Min, &t1Max);
            bool hit2 = rightChild->box.RayIntersect(ray, &t2Min, &t2Max);
            
            // 根据相交情况决定遍历顺序
            if (hit1 && hit2) {
                if (t1Min < t2Min) {
                    stack[stackPtr].node = rightChild;
                    stack[stackPtr].tMin = t2Min;
                    stack[stackPtr].tMax = t2Max;
                    stackPtr++;
                    
                    stack[stackPtr].node = leftChild;
                    stack[stackPtr].tMin = t1Min;
                    stack[stackPtr].tMax = t1Max;
                    stackPtr++;
                } else {
                    stack[stackPtr].node = leftChild;
                    stack[stackPtr].tMin = t1Min;
                    stack[stackPtr].tMax = t1Max;
                    stackPtr++;
                    
                    stack[stackPtr].node = rightChild;
                    stack[stackPtr].tMin = t2Min;
                    stack[stackPtr].tMax = t2Max;
                    stackPtr++;
                }
            } else if (hit1) {
                stack[stackPtr].node = leftChild;
                stack[stackPtr].tMin = t1Min;
                stack[stackPtr].tMax = t1Max;
                stackPtr++;
            } else if (hit2) {
                stack[stackPtr].node = rightChild;
                stack[stackPtr].tMin = t2Min;
                stack[stackPtr].tMax = t2Max;
                stackPtr++;
            }
        }
    }
    
    return hit;
}


