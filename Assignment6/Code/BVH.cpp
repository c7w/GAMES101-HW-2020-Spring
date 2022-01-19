#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    splitMethod == SplitMethod::SAH;

    root = recursiveBuildSAH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {

        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        if (this->splitMethod == SplitMethod::NAIVE) {
            // assert(false);
            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object *>(beginning, middling);
            auto rightshapes = std::vector<Object *>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        
        } else {
            

            int size = objects.size();
            std::vector<Bounds3> a(size-1);
            std::vector<Bounds3> b(size-1);

            a[0] = objects[0]->getBounds();
            b[size-2] = objects[size-1]->getBounds();

            for(int i = 1; i < size - 1; ++i) {
                a[i] = Union(a[i-1], objects[i]->getBounds());
                b[size-2-i] = Union(objects[size-1-i]->getBounds(), b[size-1-i]);
            }

            int mid = 0; double Mcost = 1145141919810.0;
            for(int i = 0; i < size-1; ++i) {
                auto c = Union(a[i], b[i]);
                auto sa = a[i].SurfaceArea();
                auto sb = b[i].SurfaceArea();
                auto sc = c.SurfaceArea();
                auto cost = sa / sc * (i + 1) + sb / sc * (size - i - 1) + 0.125;
                if ( cost < Mcost ) { mid = i; Mcost = cost; }
            }




            auto beginning = objects.begin();
            auto middling = objects.begin() + mid;
            auto ending = objects.end();

            auto leftshapes = std::vector<Object *>(beginning, middling + 1);
            auto rightshapes = std::vector<Object *>(middling + 1, ending);

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
            
        }


    }

    return node;
}

// Copied from Quanwei1992/GAMES101: https://github.com/Quanwei1992/GAMES101 for further studying
// of an algorithm of O(n)
BVHBuildNode *BVHAccel::recursiveBuildSAH(std::vector<Object *> objects) {
    BVHBuildNode *node = new BVHBuildNode();

    // Compute bounds of all primitives in SVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    } else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    } else {
        Bounds3 centroidBounds;

        constexpr int nBuckets = 12;
        float minCost = std::numeric_limits<float>::max();
        int minCostSplitBucket = 0;

        // 记录按最小代价划分的索引
        int mid;
        // 记录按最小代价划分对应的维度
        int minDim = 0;

        int start = 0;
        int end = objects.size();

        for (int i = 0; i < end; ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());

        // 对三个维度分别计算最小代价
        for (int dim = 0; dim < 3; ++dim) {
            // 如果图元数量小于4，直接取中间即可，减少运算时间
            if (objects.size() <= 4) {
                mid = (start + end) / 2;
                std::nth_element(&objects[start], &objects[mid],
                                 &objects[end - 1] + 1,
                                 [dim](Object *a, Object *b) {
                                     return a->getBounds().Centroid()[dim] <
                                            b->getBounds().Centroid()[dim];
                                 });
            } else {
                // 否则，按最小代价进行划分，首先初始化格子
                BucketInfo buckets[nBuckets];
                for (int i = 0; i < end; ++i) {
                    Vector3f offsetVec = centroidBounds.Offset(
                        objects[i]->getBounds().Centroid());
                    int b = nBuckets * offsetVec[dim];
                    if (b == nBuckets)
                        b = nBuckets - 1;
                    buckets[b].count++;
                    buckets[b].bounds = Union(
                        buckets[b].bounds, objects[i]->getBounds().Centroid());
                }

                // 计算最小代价
                float cost[nBuckets - 1];
                for (int i = 0; i < nBuckets - 1; ++i) {
                    Bounds3 b0, b1;
                    int count0 = 0, count1 = 0;
                    for (int j = 0; j <= i; ++j) {
                        b0 = Union(b0, buckets[j].bounds);
                        count0 += buckets[j].count;
                    }
                    for (int j = i + 1; j < nBuckets; ++j) {
                        b1 = Union(b1, buckets[j].bounds);
                        count1 += buckets[j].count;
                    }
                    cost[i] = .125f + (count0 * b0.SurfaceArea() +
                                       count1 * b1.SurfaceArea()) /
                                          bounds.SurfaceArea();

                    // 更新最小代价参数：最小代价，最小代价的格子索引，最小代价的维度
                    if (cost[i] < minCost) {
                        minCost = cost[i];
                        minCostSplitBucket = i;
                        minDim = dim;
                    }
                }
            }
        }

        // 按最小代价进行划分
        auto pmid =
            std::partition(&objects[0], &objects[end - 1] + 1, [=](Object *pi) {
                int b = nBuckets * centroidBounds.Offset(
                                       pi->getBounds().Centroid())[minDim];
                if (b == nBuckets)
                    b = nBuckets - 1;
                return b <= minCostSplitBucket;
            });
        mid = pmid - &objects[0];

        auto beginning = objects.begin();
        auto middling = objects.begin() + mid;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object *>(beginning, middling);
        auto rightshapes = std::vector<Object *>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildSAH(leftshapes);
        node->right = recursiveBuildSAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    if(node->object) { // Leaf Node
        std::array<int, 3> isNeg = {0,0,0};
        for(int i = 0; i < 3; ++i) {if(ray.direction[i] < 0) { isNeg[i] = 1; }}
        bool intersectWithBound3 = node->bounds.IntersectP(ray, ray.direction_inv, isNeg);
        if(intersectWithBound3) {
            return node->object->getIntersection(ray);
        }
        return Intersection();
    }

    auto left_inter = getIntersection(node->left, ray);
    auto right_inter = getIntersection(node->right, ray);

    if(left_inter.happened && right_inter.happened) {
        if(left_inter.distance < right_inter.distance) return left_inter;
        return right_inter;
    }

    if(left_inter.happened) return left_inter;
    return right_inter;

}