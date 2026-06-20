#include "../include/CollisionDetection.h"
#include "../include/MathUtils.h"
#include <algorithm>
#include <limits>

namespace IcePhysics
{
    BroadPhase::BroadPhase()
        : m_nextProxyId(0)
    {
    }

    BroadPhase::~BroadPhase()
    {
    }

    void BroadPhase::UpdateCollider(uint32_t colliderId, const AABB& aabb)
    {
        auto it = m_colliderToProxy.find(colliderId);
        if (it != m_colliderToProxy.end())
        {
            uint32_t proxyId = it->second;
            m_proxies[proxyId].aabb = aabb;
        }
        else
        {
            Proxy proxy;
            proxy.proxyId = m_nextProxyId;
            proxy.colliderId = colliderId;
            proxy.aabb = aabb;
            m_proxies[m_nextProxyId] = proxy;
            m_colliderToProxy[colliderId] = m_nextProxyId;
            m_nextProxyId++;
        }
    }

    void BroadPhase::RemoveCollider(uint32_t colliderId)
    {
        auto it = m_colliderToProxy.find(colliderId);
        if (it != m_colliderToProxy.end())
        {
            uint32_t proxyId = it->second;
            m_proxies.erase(proxyId);
            m_colliderToProxy.erase(it);
        }
    }

    void BroadPhase::GetPotentialPairs(std::vector<CollisionPair>& pairs, const std::unordered_map<uint32_t, Collider*>& colliders)
    {
        pairs.clear();

        std::vector<uint32_t> proxyIds;
        proxyIds.reserve(m_proxies.size());
        for (const auto& pair : m_proxies)
        {
            proxyIds.push_back(pair.first);
        }

        for (size_t i = 0; i < proxyIds.size(); i++)
        {
            for (size_t j = i + 1; j < proxyIds.size(); j++)
            {
                uint32_t proxyIdA = proxyIds[i];
                uint32_t proxyIdB = proxyIds[j];

                const Proxy& proxyA = m_proxies[proxyIdA];
                const Proxy& proxyB = m_proxies[proxyIdB];

                if (Overlap(proxyA.aabb, proxyB.aabb))
                {
                    auto colliderItA = colliders.find(proxyA.colliderId);
                    auto colliderItB = colliders.find(proxyB.colliderId);

                    if (colliderItA != colliders.end() && colliderItB != colliders.end())
                    {
                        CollisionPair pair;
                        pair.colliderA = proxyA.colliderId;
                        pair.colliderB = proxyB.colliderId;
                        pair.bodyA = colliderItA->second->GetBodyId();
                        pair.bodyB = colliderItB->second->GetBodyId();

                        if (pair.bodyA != pair.bodyB)
                        {
                            pairs.push_back(pair);
                        }
                    }
                }
            }
        }
    }

    uint32_t BroadPhase::GetProxyId(uint32_t colliderId) const
    {
        auto it = m_colliderToProxy.find(colliderId);
        if (it != m_colliderToProxy.end())
        {
            return it->second;
        }
        return 0xFFFFFFFF;
    }

    AABB BroadPhase::GetProxyAABB(uint32_t proxyId) const
    {
        auto it = m_proxies.find(proxyId);
        if (it != m_proxies.end())
        {
            return it->second.aabb;
        }
        AABB empty;
        empty.min = Vector3(0, 0, 0);
        empty.max = Vector3(0, 0, 0);
        return empty;
    }

    bool BroadPhase::Overlap(const AABB& a, const AABB& b) const
    {
        return Math::AABBIntersects(a, b);
    }

    NarrowPhase::NarrowPhase()
    {
    }

    NarrowPhase::~NarrowPhase()
    {
    }

    bool NarrowPhase::TestCollision(const Collider* colliderA, const Collider* colliderB,
                                     const Vector3& posA, const Quaternion& rotA,
                                     const Vector3& posB, const Quaternion& rotB,
                                     std::vector<CollisionManifold>& manifolds)
    {
        ColliderType typeA = colliderA->GetType();
        ColliderType typeB = colliderB->GetType();

        Quaternion worldRotA = rotA * colliderA->GetRotationOffset();
        Quaternion worldRotB = rotB * colliderB->GetRotationOffset();
        Vector3 worldPosA = posA + rotA * colliderA->GetOffset();
        Vector3 worldPosB = posB + rotB * colliderB->GetOffset();

        if (typeA == ColliderType::Box && typeB == ColliderType::Box)
        {
            return TestBoxBox(static_cast<const BoxCollider*>(colliderA),
                              static_cast<const BoxCollider*>(colliderB),
                              worldPosA, worldRotA, worldPosB, worldRotB, manifolds);
        }
        else if ((typeA == ColliderType::Box && typeB == ColliderType::Sphere) ||
                 (typeA == ColliderType::Sphere && typeB == ColliderType::Box))
        {
            if (typeA == ColliderType::Box)
            {
                return TestBoxSphere(static_cast<const BoxCollider*>(colliderA),
                                     static_cast<const SphereCollider*>(colliderB),
                                     worldPosA, worldRotA, worldPosB, worldRotB, manifolds);
            }
            else
            {
                bool result = TestBoxSphere(static_cast<const BoxCollider*>(colliderB),
                                            static_cast<const SphereCollider*>(colliderA),
                                            worldPosB, worldRotB, worldPosA, worldRotA, manifolds);
                for (auto& m : manifolds)
                {
                    m.normal = Negate(m.normal);
                }
                return result;
            }
        }
        else if ((typeA == ColliderType::Box && typeB == ColliderType::Mesh) ||
                 (typeA == ColliderType::Mesh && typeB == ColliderType::Box))
        {
            if (typeA == ColliderType::Box)
            {
                return TestBoxMesh(static_cast<const BoxCollider*>(colliderA),
                                   static_cast<const MeshCollider*>(colliderB),
                                   worldPosA, worldRotA, worldPosB, worldRotB, manifolds);
            }
            else
            {
                bool result = TestBoxMesh(static_cast<const BoxCollider*>(colliderB),
                                          static_cast<const MeshCollider*>(colliderA),
                                          worldPosB, worldRotB, worldPosA, worldRotA, manifolds);
                for (auto& m : manifolds)
                {
                    m.normal = Negate(m.normal);
                }
                return result;
            }
        }
        else if ((typeA == ColliderType::Hull && typeB == ColliderType::Mesh) ||
                 (typeA == ColliderType::Mesh && typeB == ColliderType::Hull))
        {
            if (typeA == ColliderType::Hull)
            {
                return TestHullMesh(static_cast<const HullCollider*>(colliderA),
                                    static_cast<const MeshCollider*>(colliderB),
                                    worldPosA, worldRotA, worldPosB, worldRotB, manifolds);
            }
            else
            {
                bool result = TestHullMesh(static_cast<const HullCollider*>(colliderB),
                                           static_cast<const MeshCollider*>(colliderA),
                                           worldPosB, worldRotB, worldPosA, worldRotA, manifolds);
                for (auto& m : manifolds)
                {
                    m.normal = Negate(m.normal);
                }
                return result;
            }
        }
        else if (typeA == ColliderType::Hull || typeB == ColliderType::Hull ||
                 typeA == ColliderType::Box || typeB == ColliderType::Box ||
                 typeA == ColliderType::Sphere || typeB == ColliderType::Sphere)
        {
            Vector3 normal;
            float penetration;
            Vector3 point;
            if (GJK(colliderA, colliderB, worldPosA, worldRotA, worldPosB, worldRotB, normal, penetration, point))
            {
                CollisionManifold manifold;
                manifold.normal = normal;
                manifold.point = point;
                manifold.penetration = penetration;
                manifold.impulse = 0.0f;
                manifolds.push_back(manifold);
                return true;
            }
            return false;
        }

        return false;
    }

    bool NarrowPhase::TestBoxBox(const BoxCollider* boxA, const BoxCollider* boxB,
                                  const Vector3& posA, const Quaternion& rotA,
                                  const Vector3& posB, const Quaternion& rotB,
                                  std::vector<CollisionManifold>& manifolds)
    {
        Vector3 halfExtentsA = boxA->GetHalfExtents();
        Vector3 halfExtentsB = boxB->GetHalfExtents();

        Matrix3x3 rotMatA = Math::Mat3FromQuat(rotA);
        Matrix3x3 rotMatB = Math::Mat3FromQuat(rotB);

        Vector3 axes[15];
        axes[0] = Vector3(rotMatA.m[0], rotMatA.m[3], rotMatA.m[6]);
        axes[1] = Vector3(rotMatA.m[1], rotMatA.m[4], rotMatA.m[7]);
        axes[2] = Vector3(rotMatA.m[2], rotMatA.m[5], rotMatA.m[8]);
        axes[3] = Vector3(rotMatB.m[0], rotMatB.m[3], rotMatB.m[6]);
        axes[4] = Vector3(rotMatB.m[1], rotMatB.m[4], rotMatB.m[7]);
        axes[5] = Vector3(rotMatB.m[2], rotMatB.m[5], rotMatB.m[8]);

        int axisIndex = 6;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                axes[axisIndex] = Math::Cross(axes[i], axes[3 + j]);
                float len = Math::Length(axes[axisIndex]);
                if (len > Math::EPSILON)
                {
                    axes[axisIndex] = axes[axisIndex] / len;
                }
                else
                {
                    axes[axisIndex] = Vector3(1, 0, 0);
                }
                axisIndex++;
            }
        }

        Vector3 delta = posB - posA;

        float minOverlap = std::numeric_limits<float>::max();
        int minAxis = -1;

        for (int i = 0; i < 15; i++)
        {
            const Vector3& axis = axes[i];

            float projA = halfExtentsA.x * Math::Abs(Math::Dot(axis, axes[0])) +
                          halfExtentsA.y * Math::Abs(Math::Dot(axis, axes[1])) +
                          halfExtentsA.z * Math::Abs(Math::Dot(axis, axes[2]));

            float projB = halfExtentsB.x * Math::Abs(Math::Dot(axis, axes[3])) +
                          halfExtentsB.y * Math::Abs(Math::Dot(axis, axes[4])) +
                          halfExtentsB.z * Math::Abs(Math::Dot(axis, axes[5]));

            float distance = Math::Abs(Math::Dot(delta, axis));
            float overlap = projA + projB - distance;

            if (overlap < 0.0f)
            {
                return false;
            }

            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                minAxis = i;
            }
        }

        if (minAxis == -1)
        {
            return false;
        }

        Vector3 normal = axes[minAxis];
        if (Math::Dot(delta, normal) < 0.0f)
        {
            normal = Math::Negate(normal);
        }

        Vector3 verticesA[8];
        Vector3 verticesB[8];

        Vector3 extA = boxA->GetHalfExtents();
        verticesA[0] = posA + rotA * Vector3(-extA.x, -extA.y, -extA.z);
        verticesA[1] = posA + rotA * Vector3( extA.x, -extA.y, -extA.z);
        verticesA[2] = posA + rotA * Vector3( extA.x, -extA.y,  extA.z);
        verticesA[3] = posA + rotA * Vector3(-extA.x, -extA.y,  extA.z);
        verticesA[4] = posA + rotA * Vector3(-extA.x,  extA.y, -extA.z);
        verticesA[5] = posA + rotA * Vector3( extA.x,  extA.y, -extA.z);
        verticesA[6] = posA + rotA * Vector3( extA.x,  extA.y,  extA.z);
        verticesA[7] = posA + rotA * Vector3(-extA.x,  extA.y,  extA.z);

        Vector3 extB = boxB->GetHalfExtents();
        verticesB[0] = posB + rotB * Vector3(-extB.x, -extB.y, -extB.z);
        verticesB[1] = posB + rotB * Vector3( extB.x, -extB.y, -extB.z);
        verticesB[2] = posB + rotB * Vector3( extB.x, -extB.y,  extB.z);
        verticesB[3] = posB + rotB * Vector3(-extB.x, -extB.y,  extB.z);
        verticesB[4] = posB + rotB * Vector3(-extB.x,  extB.y, -extB.z);
        verticesB[5] = posB + rotB * Vector3( extB.x,  extB.y, -extB.z);
        verticesB[6] = posB + rotB * Vector3( extB.x,  extB.y,  extB.z);
        verticesB[7] = posB + rotB * Vector3(-extB.x,  extB.y,  extB.z);

        Vector3 pointA;
        float minDistA = std::numeric_limits<float>::max();
        for (int i = 0; i < 8; i++)
        {
            float d = Math::Dot(verticesA[i], normal);
            if (d < minDistA)
            {
                minDistA = d;
                pointA = verticesA[i];
            }
        }

        Vector3 pointB;
        float maxDistB = -std::numeric_limits<float>::max();
        for (int i = 0; i < 8; i++)
        {
            float d = Math::Dot(verticesB[i], normal);
            if (d > maxDistB)
            {
                maxDistB = d;
                pointB = verticesB[i];
            }
        }

        CollisionManifold manifold;
        manifold.normal = normal;
        manifold.point = (pointA + pointB) * 0.5f;
        manifold.penetration = minOverlap;
        manifold.impulse = 0.0f;
        manifolds.push_back(manifold);

        return true;
    }

    bool NarrowPhase::TestBoxSphere(const BoxCollider* box, const SphereCollider* sphere,
                                     const Vector3& posA, const Quaternion& rotA,
                                     const Vector3& posB, const Quaternion& rotB,
                                     std::vector<CollisionManifold>& manifolds)
    {
        Vector3 spherePos = posB;
        float radius = sphere->GetRadius();

        Quaternion invRotA = Math::QuatInverse(rotA);
        Vector3 localSpherePos = invRotA * (spherePos - posA);

        Vector3 halfExtents = box->GetHalfExtents();

        Vector3 closestPoint;
        closestPoint.x = Math::Clamp(localSpherePos.x, -halfExtents.x, halfExtents.x);
        closestPoint.y = Math::Clamp(localSpherePos.y, -halfExtents.y, halfExtents.y);
        closestPoint.z = Math::Clamp(localSpherePos.z, -halfExtents.z, halfExtents.z);

        Vector3 localDiff = localSpherePos - closestPoint;
        float distSq = Math::LengthSquared(localDiff);

        if (distSq > radius * radius)
        {
            return false;
        }

        float dist = Math::Sqrt(distSq);
        Vector3 normal;

        if (dist < Math::EPSILON)
        {
            Vector3 absLocalPos(Math::Abs(localSpherePos.x), Math::Abs(localSpherePos.y), Math::Abs(localSpherePos.z));
            if (absLocalPos.x >= absLocalPos.y && absLocalPos.x >= absLocalPos.z)
            {
                normal = Vector3(Math::Sign(localSpherePos.x), 0, 0);
            }
            else if (absLocalPos.y >= absLocalPos.x && absLocalPos.y >= absLocalPos.z)
            {
                normal = Vector3(0, Math::Sign(localSpherePos.y), 0);
            }
            else
            {
                normal = Vector3(0, 0, Math::Sign(localSpherePos.z));
            }
            normal = rotA * normal;
            dist = 0.0f;
        }
        else
        {
            normal = rotA * (localDiff / dist);
        }

        Vector3 worldClosestPoint = posA + rotA * closestPoint;
        Vector3 contactPoint = worldClosestPoint + normal * (dist * 0.5f);

        CollisionManifold manifold;
        manifold.normal = normal;
        manifold.point = contactPoint;
        manifold.penetration = radius - dist;
        manifold.impulse = 0.0f;
        manifolds.push_back(manifold);

        return true;
    }

    bool NarrowPhase::TestBoxMesh(const BoxCollider* box, const MeshCollider* mesh,
                                   const Vector3& posA, const Quaternion& rotA,
                                   const Vector3& posB, const Quaternion& rotB,
                                   std::vector<CollisionManifold>& manifolds)
    {
        AABB boxAABB = box->ComputeAABB(posA, rotA);
        AABB meshAABB = mesh->ComputeAABB(posB, rotB);

        if (!Math::AABBIntersects(boxAABB, meshAABB))
        {
            return false;
        }

        const auto& triangles = mesh->GetTriangles();
        bool hasCollision = false;

        for (uint32_t i = 0; i < triangles.size(); i++)
        {
            Vector3 v0 = mesh->GetTriangleVertex(i, 0, posB, rotB);
            Vector3 v1 = mesh->GetTriangleVertex(i, 1, posB, rotB);
            Vector3 v2 = mesh->GetTriangleVertex(i, 2, posB, rotB);

            AABB triAABB;
            triAABB.min = Vector3(Math::Min(Math::Min(v0.x, v1.x), v2.x), Math::Min(Math::Min(v0.y, v1.y), v2.y), Math::Min(Math::Min(v0.z, v1.z), v2.z));
            triAABB.max = Vector3(Math::Max(Math::Max(v0.x, v1.x), v2.x), Math::Max(Math::Max(v0.y, v1.y), v2.y), Math::Max(Math::Max(v0.z, v1.z), v2.z));

            if (!Math::AABBIntersects(boxAABB, triAABB))
            {
                continue;
            }

            Vector3 triNormal = Math::TriangleNormal(v0, v1, v2);
            Vector3 boxCenter = (boxAABB.min + boxAABB.max) * 0.5f;

            Vector3 halfExtents = box->GetHalfExtents();
            Matrix3x3 rotMat = Math::Mat3FromQuat(rotA);

            Vector3 axes[6];
            axes[0] = Vector3(rotMat.m[0], rotMat.m[3], rotMat.m[6]);
            axes[1] = Vector3(rotMat.m[1], rotMat.m[4], rotMat.m[7]);
            axes[2] = Vector3(rotMat.m[2], rotMat.m[5], rotMat.m[8]);
            axes[3] = triNormal;
            axes[4] = Math::Cross(axes[0], triNormal);
            axes[5] = Math::Cross(axes[1], triNormal);

            for (int a = 0; a < 6; a++)
            {
                if (Math::LengthSquared(axes[a]) < Math::EPSILON)
                {
                    continue;
                }
                axes[a] = Math::Normalize(axes[a]);
            }

            float minOverlap = std::numeric_limits<float>::max();
            Vector3 bestAxis;
            bool separated = false;

            for (int a = 0; a < 6; a++)
            {
                const Vector3& axis = axes[a];

                float boxProj = halfExtents.x * Math::Abs(Math::Dot(axis, axes[0])) +
                                halfExtents.y * Math::Abs(Math::Dot(axis, axes[1])) +
                                halfExtents.z * Math::Abs(Math::Dot(axis, axes[2]));

                float triProj = Math::Max(Math::Max(Math::Dot(v0 - posA, axis), Math::Dot(v1 - posA, axis)), Math::Dot(v2 - posA, axis)) -
                                Math::Min(Math::Min(Math::Dot(v0 - posA, axis), Math::Dot(v1 - posA, axis)), Math::Dot(v2 - posA, axis));

                Vector3 delta = (v0 + v1 + v2) / 3.0f - boxCenter;
                float dist = Math::Abs(Math::Dot(delta, axis));
                float overlap = boxProj + triProj * 0.5f - dist;

                if (overlap < 0.0f)
                {
                    separated = true;
                    break;
                }

                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    bestAxis = axis;
                }
            }

            if (separated)
            {
                continue;
            }

            Vector3 deltaCenter = (v0 + v1 + v2) / 3.0f - boxCenter;
            if (Math::Dot(deltaCenter, bestAxis) < 0.0f)
            {
                bestAxis = Math::Negate(bestAxis);
            }

            Vector3 closestPoint = v0;
            float minDist = std::numeric_limits<float>::max();
            for (int j = 0; j < 3; j++)
            {
                Vector3 point = (j == 0) ? v0 : (j == 1 ? v1 : v2);
                float d = Math::LengthSquared(point - boxCenter);
                if (d < minDist)
                {
                    minDist = d;
                    closestPoint = point;
                }
            }

            CollisionManifold manifold;
            manifold.normal = bestAxis;
            manifold.point = closestPoint;
            manifold.penetration = minOverlap;
            manifold.impulse = 0.0f;
            manifolds.push_back(manifold);
            hasCollision = true;
        }

        return hasCollision;
    }

    bool NarrowPhase::TestHullMesh(const HullCollider* hull, const MeshCollider* mesh,
                                    const Vector3& posA, const Quaternion& rotA,
                                    const Vector3& posB, const Quaternion& rotB,
                                    std::vector<CollisionManifold>& manifolds)
    {
        Vector3 normal;
        float penetration;
        Vector3 point;

        if (GJK(hull, mesh, posA, rotA, posB, rotB, normal, penetration, point))
        {
            CollisionManifold manifold;
            manifold.normal = normal;
            manifold.point = point;
            manifold.penetration = penetration;
            manifold.impulse = 0.0f;
            manifolds.push_back(manifold);
            return true;
        }

        return false;
    }

    Vector3 NarrowPhase::GetSupport(const Collider* colliderA, const Collider* colliderB,
                                     const Vector3& posA, const Quaternion& rotA,
                                     const Vector3& posB, const Quaternion& rotB,
                                     const Vector3& direction, SimplexPoint& outPoint)
    {
        Vector3 supportA = colliderA->GetSupportPoint(direction, posA, rotA);
        Vector3 supportB = colliderB->GetSupportPoint(Math::Negate(direction), posB, rotB);

        outPoint.supportA = supportA;
        outPoint.supportB = supportB;
        outPoint.point = supportA - supportB;

        return outPoint.point;
    }

    bool NarrowPhase::GJK(const Collider* colliderA, const Collider* colliderB,
                           const Vector3& posA, const Quaternion& rotA,
                           const Vector3& posB, const Quaternion& rotB,
                           Vector3& outNormal, float& outPenetration, Vector3& outPoint)
    {
        Simplex simplex;
        simplex.count = 0;

        Vector3 direction = posB - posA;
        if (Math::LengthSquared(direction) < Math::EPSILON)
        {
            direction = Vector3(1, 0, 0);
        }
        direction = Math::Normalize(direction);

        SimplexPoint initialPoint;
        GetSupport(colliderA, colliderB, posA, rotA, posB, rotB, direction, initialPoint);
        simplex.points[0] = initialPoint;
        simplex.count = 1;

        direction = Math::Negate(initialPoint.point);

        const int maxIterations = 32;
        for (int i = 0; i < maxIterations; i++)
        {
            SimplexPoint newPoint;
            GetSupport(colliderA, colliderB, posA, rotA, posB, rotB, direction, newPoint);

            if (Math::Dot(newPoint.point, direction) <= 0.0f)
            {
                return false;
            }

            simplex.points[simplex.count++] = newPoint;

            if (GJKNextSimplex(simplex, direction))
            {
                outPenetration = EPA(simplex, colliderA, colliderB, posA, rotA, posB, rotB, outNormal, outPoint);
                return true;
            }
        }

        return false;
    }

    bool NarrowPhase::GJKNextSimplex(Simplex& simplex, Vector3& direction)
    {
        switch (simplex.count)
        {
            case 2:
                return GJKSimplexLine(simplex, direction);
            case 3:
                return GJKSimplexTriangle(simplex, direction);
            case 4:
                return GJKSimplexTetrahedron(simplex, direction);
            default:
                return false;
        }
    }

    bool NarrowPhase::GJKSimplexLine(Simplex& simplex, Vector3& direction)
    {
        Vector3 a = simplex.points[1].point;
        Vector3 b = simplex.points[0].point;

        Vector3 ab = b - a;
        Vector3 ao = Math::Negate(a);

        Vector3 abCrossAo = Math::Cross(ab, ao);
        if (Math::LengthSquared(abCrossAo) > Math::EPSILON)
        {
            direction = Math::Cross(abCrossAo, ab);
        }
        else
        {
            direction = Vector3(1, 0, 0);
        }
        direction = Math::Normalize(direction);

        return false;
    }

    bool NarrowPhase::GJKSimplexTriangle(Simplex& simplex, Vector3& direction)
    {
        Vector3 a = simplex.points[2].point;
        Vector3 b = simplex.points[1].point;
        Vector3 c = simplex.points[0].point;

        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ao = Math::Negate(a);

        Vector3 abc = Math::Cross(ab, ac);
        Vector3 abCrossAbc = Math::Cross(ab, abc);
        Vector3 abcCrossAc = Math::Cross(abc, ac);

        if (Math::Dot(abCrossAbc, ao) > 0.0f)
        {
            simplex.points[0] = simplex.points[1];
            simplex.count = 2;
            direction = abCrossAbc;
        }
        else if (Math::Dot(abcCrossAc, ao) > 0.0f)
        {
            simplex.points[1] = simplex.points[2];
            simplex.count = 2;
            direction = abcCrossAc;
        }
        else
        {
            if (Math::Dot(abc, ao) > 0.0f)
            {
                direction = abc;
            }
            else
            {
                SimplexPoint temp = simplex.points[0];
                simplex.points[0] = simplex.points[1];
                simplex.points[1] = temp;
                direction = Math::Negate(abc);
            }
        }

        direction = Math::Normalize(direction);
        return false;
    }

    bool NarrowPhase::GJKSimplexTetrahedron(Simplex& simplex, Vector3& direction)
    {
        Vector3 a = simplex.points[3].point;
        Vector3 b = simplex.points[2].point;
        Vector3 c = simplex.points[1].point;
        Vector3 d = simplex.points[0].point;

        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ad = d - a;
        Vector3 ao = Math::Negate(a);

        Vector3 abc = Math::Cross(ab, ac);
        Vector3 acd = Math::Cross(ac, ad);
        Vector3 adb = Math::Cross(ad, ab);

        bool abcSide = Math::Dot(abc, ad) > 0.0f;
        bool acdSide = Math::Dot(acd, ab) > 0.0f;
        bool adbSide = Math::Dot(adb, ac) > 0.0f;

        if ((abcSide && Math::Dot(abc, ao) > 0.0f) || (!abcSide && Math::Dot(abc, ao) < 0.0f))
        {
            simplex.points[0] = simplex.points[1];
            simplex.points[1] = simplex.points[2];
            simplex.points[2] = simplex.points[3];
            simplex.count = 3;
            direction = abcSide ? abc : Math::Negate(abc);
            direction = Math::Normalize(direction);
            return false;
        }

        if ((acdSide && Math::Dot(acd, ao) > 0.0f) || (!acdSide && Math::Dot(acd, ao) < 0.0f))
        {
            simplex.points[2] = simplex.points[3];
            simplex.count = 3;
            direction = acdSide ? acd : Math::Negate(acd);
            direction = Math::Normalize(direction);
            return false;
        }

        if ((adbSide && Math::Dot(adb, ao) > 0.0f) || (!adbSide && Math::Dot(adb, ao) < 0.0f))
        {
            simplex.points[1] = simplex.points[0];
            simplex.points[0] = simplex.points[2];
            simplex.points[2] = simplex.points[3];
            simplex.count = 3;
            direction = adbSide ? adb : Math::Negate(adb);
            direction = Math::Normalize(direction);
            return false;
        }

        return true;
    }

    void NarrowPhase::EPABuildInitialPolytope(const Simplex& simplex)
    {
        m_epaPoints.clear();
        m_epaFaces.clear();

        for (uint32_t i = 0; i < simplex.count; i++)
        {
            m_epaPoints.push_back(simplex.points[i]);
        }

        EPAFace faces[4];

        faces[0].indices[0] = 0; faces[0].indices[1] = 1; faces[0].indices[2] = 2;
        faces[1].indices[0] = 0; faces[1].indices[1] = 3; faces[1].indices[2] = 1;
        faces[2].indices[0] = 0; faces[2].indices[1] = 2; faces[2].indices[2] = 3;
        faces[3].indices[0] = 1; faces[3].indices[1] = 3; faces[3].indices[2] = 2;

        for (int i = 0; i < 4; i++)
        {
            Vector3 a = m_epaPoints[faces[i].indices[0]].point;
            Vector3 b = m_epaPoints[faces[i].indices[1]].point;
            Vector3 c = m_epaPoints[faces[i].indices[2]].point;

            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 normal = Math::Cross(ab, ac);
            float len = Math::Length(normal);

            if (len < Math::EPSILON)
            {
                continue;
            }

            normal = normal / len;
            float distance = Math::Dot(normal, a);

            if (distance < 0.0f)
            {
                normal = Math::Negate(normal);
                distance = -distance;
                uint32_t temp = faces[i].indices[1];
                faces[i].indices[1] = faces[i].indices[2];
                faces[i].indices[2] = temp;
            }

            faces[i].normal = normal;
            faces[i].distance = distance;
            m_epaFaces.push_back(faces[i]);
        }
    }

    uint32_t NarrowPhase::EPAFindClosestFace()
    {
        uint32_t closestIndex = 0;
        float minDistance = m_epaFaces[0].distance;

        for (uint32_t i = 1; i < m_epaFaces.size(); i++)
        {
            if (m_epaFaces[i].distance < minDistance)
            {
                minDistance = m_epaFaces[i].distance;
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    void NarrowPhase::EPAExpand(uint32_t closestFaceIndex, const SimplexPoint& newPoint)
    {
        const EPAFace& closestFace = m_epaFaces[closestFaceIndex];

        std::vector<std::pair<uint32_t, uint32_t>> edges;

        auto addEdge = [&](uint32_t a, uint32_t b)
        {
            for (size_t i = 0; i < edges.size(); i++)
            {
                if ((edges[i].first == b && edges[i].second == a))
                {
                    edges.erase(edges.begin() + i);
                    return;
                }
            }
            edges.push_back(std::make_pair(a, b));
        };

        addEdge(closestFace.indices[0], closestFace.indices[1]);
        addEdge(closestFace.indices[1], closestFace.indices[2]);
        addEdge(closestFace.indices[2], closestFace.indices[0]);

        for (int i = (int)m_epaFaces.size() - 1; i >= 0; i--)
        {
            if (i == (int)closestFaceIndex) continue;

            const EPAFace& face = m_epaFaces[i];
            Vector3 facePoint = m_epaPoints[face.indices[0]].point;
            Vector3 toNew = newPoint.point - facePoint;

            if (Math::Dot(face.normal, toNew) > Math::EPSILON)
            {
                addEdge(face.indices[0], face.indices[1]);
                addEdge(face.indices[1], face.indices[2]);
                addEdge(face.indices[2], face.indices[0]);
                m_epaFaces.erase(m_epaFaces.begin() + i);
            }
        }

        m_epaFaces.erase(m_epaFaces.begin() + closestFaceIndex);

        uint32_t newPointIndex = (uint32_t)m_epaPoints.size();
        m_epaPoints.push_back(newPoint);

        for (const auto& edge : edges)
        {
            EPAFace newFace;
            newFace.indices[0] = edge.first;
            newFace.indices[1] = edge.second;
            newFace.indices[2] = newPointIndex;

            Vector3 a = m_epaPoints[newFace.indices[0]].point;
            Vector3 b = m_epaPoints[newFace.indices[1]].point;
            Vector3 c = m_epaPoints[newFace.indices[2]].point;

            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 normal = Math::Cross(ab, ac);
            float len = Math::Length(normal);

            if (len < Math::EPSILON)
            {
                continue;
            }

            normal = normal / len;
            float distance = Math::Dot(normal, a);

            if (distance < 0.0f)
            {
                normal = Math::Negate(normal);
                distance = -distance;
                uint32_t temp = newFace.indices[1];
                newFace.indices[1] = newFace.indices[2];
                newFace.indices[2] = temp;
            }

            newFace.normal = normal;
            newFace.distance = distance;
            m_epaFaces.push_back(newFace);
        }
    }

    float NarrowPhase::EPA(const Simplex& simplex, const Collider* colliderA, const Collider* colliderB,
                            const Vector3& posA, const Quaternion& rotA,
                            const Vector3& posB, const Quaternion& rotB,
                            Vector3& outNormal, Vector3& outPoint)
    {
        EPABuildInitialPolytope(simplex);

        const int maxIterations = 64;
        const float tolerance = 0.0001f;

        for (int i = 0; i < maxIterations; i++)
        {
            uint32_t closestFaceIndex = EPAFindClosestFace();
            const EPAFace& closestFace = m_epaFaces[closestFaceIndex];

            SimplexPoint newPoint;
            GetSupport(colliderA, colliderB, posA, rotA, posB, rotB, closestFace.normal, newPoint);

            float supportDistance = Math::Dot(closestFace.normal, newPoint.point);

            if (supportDistance - closestFace.distance < tolerance)
            {
                outNormal = closestFace.normal;

                Vector3 a = m_epaPoints[closestFace.indices[0]].supportA;
                Vector3 b = m_epaPoints[closestFace.indices[1]].supportA;
                Vector3 c = m_epaPoints[closestFace.indices[2]].supportA;

                outPoint = (a + b + c) / 3.0f;

                return closestFace.distance;
            }

            EPAExpand(closestFaceIndex, newPoint);
        }

        uint32_t closestFaceIndex = EPAFindClosestFace();
        const EPAFace& closestFace = m_epaFaces[closestFaceIndex];
        outNormal = closestFace.normal;

        Vector3 a = m_epaPoints[closestFace.indices[0]].supportA;
        Vector3 b = m_epaPoints[closestFace.indices[1]].supportA;
        Vector3 c = m_epaPoints[closestFace.indices[2]].supportA;

        outPoint = (a + b + c) / 3.0f;

        return closestFace.distance;
    }

    ContactSolver::ContactSolver()
        : m_velocityIterations(8)
        , m_positionIterations(4)
        , m_baumgarte(0.2f)
        , m_allowedPenetration(0.01f)
        , m_enabled(true)
        , m_lastIterationCount(0)
        , m_velocitySleepThreshold(0.001f)
        , m_positionSleepThreshold(0.001f)
    {
    }

    ContactSolver::~ContactSolver()
    {
    }

    void ContactSolver::SetIterations(uint32_t velocityIterations, uint32_t positionIterations)
    {
        m_velocityIterations = velocityIterations;
        m_positionIterations = positionIterations;
    }

    void ContactSolver::Solve(std::vector<CollisionData>& collisions, RigidBodyManager& bodyManager, float dt)
    {
        std::vector<ContactConstraint> constraints;

        for (auto& collision : collisions)
        {
            RigidBody* bodyA = bodyManager.GetBody(collision.bodyA);
            RigidBody* bodyB = bodyManager.GetBody(collision.bodyB);

            if (!bodyA || !bodyB)
            {
                continue;
            }

            for (auto& manifold : collision.manifolds)
            {
                ContactConstraint constraint;
                SetupContactConstraint(constraint, manifold, bodyA, bodyB, dt);
                constraints.push_back(constraint);
            }
        }

        for (uint32_t iter = 0; iter < m_velocityIterations; iter++)
        {
            for (auto& constraint : constraints)
            {
                RigidBody* bodyA = bodyManager.GetBody(constraint.bodyA);
                RigidBody* bodyB = bodyManager.GetBody(constraint.bodyB);

                if (bodyA && bodyB)
                {
                    SolveVelocityConstraint(constraint, bodyA, bodyB);
                }
            }
        }

        for (uint32_t iter = 0; iter < m_positionIterations; iter++)
        {
            for (auto& constraint : constraints)
            {
                RigidBody* bodyA = bodyManager.GetBody(constraint.bodyA);
                RigidBody* bodyB = bodyManager.GetBody(constraint.bodyB);

                if (bodyA && bodyB)
                {
                    SolvePositionConstraint(constraint, bodyA, bodyB);
                }
            }
        }

        size_t constraintIndex = 0;
        for (auto& collision : collisions)
        {
            RigidBody* bodyA = bodyManager.GetBody(collision.bodyA);
            RigidBody* bodyB = bodyManager.GetBody(collision.bodyB);

            if (!bodyA || !bodyB)
            {
                continue;
            }

            for (size_t i = 0; i < collision.manifolds.size() && constraintIndex < constraints.size(); i++)
            {
                collision.manifolds[i].impulse = constraints[constraintIndex].normalImpulse;
                constraintIndex++;
            }
        }
    }

    void ContactSolver::SetupContactConstraint(ContactConstraint& constraint, const CollisionManifold& manifold,
                                                RigidBody* bodyA, RigidBody* bodyB, float dt)
    {
        constraint.bodyA = bodyA->GetId();
        constraint.bodyB = bodyB->GetId();
        constraint.normal = manifold.normal;
        constraint.penetration = manifold.penetration;
        constraint.restitution = Math::Min(bodyA->GetRestitution(), bodyB->GetRestitution());
        constraint.friction = Math::Sqrt(bodyA->GetDynamicFriction() * bodyB->GetDynamicFriction());
        constraint.normalImpulse = 0.0f;
        constraint.tangent1Impulse = 0.0f;
        constraint.tangent2Impulse = 0.0f;

        Vector3 posA = bodyA->GetPosition();
        Vector3 posB = bodyB->GetPosition();

        constraint.ra = manifold.point - posA;
        constraint.rb = manifold.point - posB;

        constraint.raxN = Math::Cross(constraint.ra, constraint.normal);
        constraint.rbxN = Math::Cross(constraint.rb, constraint.normal);

        float invMassA = bodyA->IsKinematic() ? 0.0f : bodyA->GetInverseMass();
        float invMassB = bodyB->IsKinematic() ? 0.0f : bodyB->GetInverseMass();

        Matrix3x3 invIA = bodyA->IsKinematic() ? Math::Mat3Identity() * 0.0f : bodyA->GetWorldInverseInertiaTensor();
        Matrix3x3 invIB = bodyB->IsKinematic() ? Math::Mat3Identity() * 0.0f : bodyB->GetWorldInverseInertiaTensor();

        float angularPart = Math::Dot(constraint.raxN, Math::Mat3MulVec3(invIA, constraint.raxN)) +
                            Math::Dot(constraint.rbxN, Math::Mat3MulVec3(invIB, constraint.rbxN));

        float normalMass = invMassA + invMassB + angularPart;
        constraint.normalMass = normalMass > Math::EPSILON ? 1.0f / normalMass : 0.0f;

        Vector3 t1, t2;
        if (Math::Abs(constraint.normal.y) > 0.9f)
        {
            t1 = Vector3(1, 0, 0);
        }
        else
        {
            t1 = Vector3(0, 1, 0);
        }
        t1 = Math::Normalize(t1 - constraint.normal * Math::Dot(t1, constraint.normal));
        t2 = Math::Normalize(Math::Cross(constraint.normal, t1));

        constraint.tangent1 = t1;
        constraint.tangent2 = t2;

        constraint.raxT1 = Math::Cross(constraint.ra, t1);
        constraint.rbxT1 = Math::Cross(constraint.rb, t1);
        constraint.raxT2 = Math::Cross(constraint.ra, t2);
        constraint.rbxT2 = Math::Cross(constraint.rb, t2);

        float tan1Angular = Math::Dot(constraint.raxT1, Math::Mat3MulVec3(invIA, constraint.raxT1)) +
                            Math::Dot(constraint.rbxT1, Math::Mat3MulVec3(invIB, constraint.rbxT1));
        float tan1Mass = invMassA + invMassB + tan1Angular;
        constraint.tangent1Mass = tan1Mass > Math::EPSILON ? 1.0f / tan1Mass : 0.0f;

        float tan2Angular = Math::Dot(constraint.raxT2, Math::Mat3MulVec3(invIA, constraint.raxT2)) +
                            Math::Dot(constraint.rbxT2, Math::Mat3MulVec3(invIB, constraint.rbxT2));
        float tan2Mass = invMassA + invMassB + tan2Angular;
        constraint.tangent2Mass = tan2Mass > Math::EPSILON ? 1.0f / tan2Mass : 0.0f;

        Vector3 velA = bodyA->GetLinearVelocity() + Math::Cross(bodyA->GetAngularVelocity(), constraint.ra);
        Vector3 velB = bodyB->GetLinearVelocity() + Math::Cross(bodyB->GetAngularVelocity(), constraint.rb);
        Vector3 relVel = velB - velA;
        float relVelNormal = Math::Dot(relVel, constraint.normal);

        float biasVelocity = 0.0f;
        if (constraint.penetration > m_allowedPenetration)
        {
            biasVelocity = m_baumgarte * (constraint.penetration - m_allowedPenetration) / dt;
        }

        if (relVelNormal < -1.0f)
        {
            biasVelocity -= constraint.restitution * relVelNormal;
        }

        constraint.bias = biasVelocity;
    }

    void ContactSolver::SolveVelocityConstraint(ContactConstraint& constraint, RigidBody* bodyA, RigidBody* bodyB)
    {
        Vector3 velA = bodyA->GetLinearVelocity();
        Vector3 angVelA = bodyA->GetAngularVelocity();
        Vector3 velB = bodyB->GetLinearVelocity();
        Vector3 angVelB = bodyB->GetAngularVelocity();

        Vector3 ra = constraint.ra;
        Vector3 rb = constraint.rb;

        float invMassA = bodyA->IsKinematic() ? 0.0f : bodyA->GetInverseMass();
        float invMassB = bodyB->IsKinematic() ? 0.0f : bodyB->GetInverseMass();
        Matrix3x3 invIA = bodyA->IsKinematic() ? Math::Mat3Identity() * 0.0f : bodyA->GetWorldInverseInertiaTensor();
        Matrix3x3 invIB = bodyB->IsKinematic() ? Math::Mat3Identity() * 0.0f : bodyB->GetWorldInverseInertiaTensor();

        Vector3 relVel = (velB + Math::Cross(angVelB, rb)) - (velA + Math::Cross(angVelA, ra));

        float relVelNormal = Math::Dot(relVel, constraint.normal);
        float normalImpulse = constraint.normalMass * (constraint.bias - relVelNormal);
        float newNormalImpulse = Math::Max(constraint.normalImpulse + normalImpulse, 0.0f);
        float deltaNormalImpulse = newNormalImpulse - constraint.normalImpulse;
        constraint.normalImpulse = newNormalImpulse;

        Vector3 impulse = constraint.normal * deltaNormalImpulse;

        if (!bodyA->IsKinematic())
        {
            velA -= impulse * invMassA;
            angVelA -= Math::Mat3MulVec3(invIA, Math::Cross(ra, impulse));
        }
        if (!bodyB->IsKinematic())
        {
            velB += impulse * invMassB;
            angVelB += Math::Mat3MulVec3(invIB, Math::Cross(rb, impulse));
        }

        relVel = (velB + Math::Cross(angVelB, rb)) - (velA + Math::Cross(angVelA, ra));

        float relVelTan1 = Math::Dot(relVel, constraint.tangent1);
        float tan1Impulse = constraint.tangent1Mass * (-relVelTan1);
        float maxFriction = constraint.friction * constraint.normalImpulse;
        float newTan1Impulse = Math::Clamp(constraint.tangent1Impulse + tan1Impulse, -maxFriction, maxFriction);
        float deltaTan1Impulse = newTan1Impulse - constraint.tangent1Impulse;
        constraint.tangent1Impulse = newTan1Impulse;

        Vector3 tan1ImpulseVec = constraint.tangent1 * deltaTan1Impulse;

        if (!bodyA->IsKinematic())
        {
            velA -= tan1ImpulseVec * invMassA;
            angVelA -= Math::Mat3MulVec3(invIA, Math::Cross(ra, tan1ImpulseVec));
        }
        if (!bodyB->IsKinematic())
        {
            velB += tan1ImpulseVec * invMassB;
            angVelB += Math::Mat3MulVec3(invIB, Math::Cross(rb, tan1ImpulseVec));
        }

        relVel = (velB + Math::Cross(angVelB, rb)) - (velA + Math::Cross(angVelA, ra));

        float relVelTan2 = Math::Dot(relVel, constraint.tangent2);
        float tan2Impulse = constraint.tangent2Mass * (-relVelTan2);
        float newTan2Impulse = Math::Clamp(constraint.tangent2Impulse + tan2Impulse, -maxFriction, maxFriction);
        float deltaTan2Impulse = newTan2Impulse - constraint.tangent2Impulse;
        constraint.tangent2Impulse = newTan2Impulse;

        Vector3 tan2ImpulseVec = constraint.tangent2 * deltaTan2Impulse;

        if (!bodyA->IsKinematic())
        {
            velA -= tan2ImpulseVec * invMassA;
            angVelA -= Math::Mat3MulVec3(invIA, Math::Cross(ra, tan2ImpulseVec));
        }
        if (!bodyB->IsKinematic())
        {
            velB += tan2ImpulseVec * invMassB;
            angVelB += Math::Mat3MulVec3(invIB, Math::Cross(rb, tan2ImpulseVec));
        }

        bodyA->SetLinearVelocity(velA);
        bodyA->SetAngularVelocity(angVelA);
        bodyB->SetLinearVelocity(velB);
        bodyB->SetAngularVelocity(angVelB);
    }

    void ContactSolver::SolvePositionConstraint(ContactConstraint& constraint, RigidBody* bodyA, RigidBody* bodyB)
    {
        if (constraint.penetration <= m_allowedPenetration)
        {
            return;
        }

        Vector3 posA = bodyA->GetPosition();
        Quaternion rotA = bodyA->GetRotation();
        Vector3 posB = bodyB->GetPosition();
        Quaternion rotB = bodyB->GetRotation();

        Vector3 ra = constraint.ra;
        Vector3 rb = constraint.rb;

        float invMassA = bodyA->IsKinematic() ? 0.0f : bodyA->GetInverseMass();
        float invMassB = bodyB->IsKinematic() ? 0.0f : bodyB->GetInverseMass();
        Matrix3x3 invIA = bodyA->IsKinematic() ? Math::Mat3Identity() * 0.0f : bodyA->GetWorldInverseInertiaTensor();
        Matrix3x3 invIB = bodyB->IsKinematic() ? Math::Mat3Identity() * 0.0f : bodyB->GetWorldInverseInertiaTensor();

        Vector3 raxN = Math::Cross(ra, constraint.normal);
        Vector3 rbxN = Math::Cross(rb, constraint.normal);

        float angularPart = Math::Dot(raxN, Math::Mat3MulVec3(invIA, raxN)) +
                            Math::Dot(rbxN, Math::Mat3MulVec3(invIB, rbxN));

        float effectiveMass = invMassA + invMassB + angularPart;
        if (effectiveMass < Math::EPSILON)
        {
            return;
        }

        float penetration = constraint.penetration - m_allowedPenetration;
        float positionalError = Math::Min(penetration, 0.2f);
        float impulse = (positionalError / effectiveMass) * 0.5f;

        Vector3 impulseVec = constraint.normal * impulse;

        if (!bodyA->IsKinematic())
        {
            posA -= impulseVec * invMassA;
            Vector3 deltaRot = Math::Mat3MulVec3(invIA, Math::Cross(ra, impulseVec));
            Quaternion deltaQuat = Math::QuatFromAngleAxis(Math::Length(deltaRot), Math::Normalize(deltaRot));
            rotA = Math::QuatNormalize(deltaQuat * rotA);
        }
        if (!bodyB->IsKinematic())
        {
            posB += impulseVec * invMassB;
            Vector3 deltaRot = Math::Mat3MulVec3(invIB, Math::Cross(rb, impulseVec));
            Quaternion deltaQuat = Math::QuatFromAngleAxis(Math::Length(deltaRot), Math::Normalize(deltaRot));
            rotB = Math::QuatNormalize(deltaQuat * rotB);
        }

        bodyA->SetPosition(posA);
        bodyA->SetRotation(rotA);
        bodyB->SetPosition(posB);
        bodyB->SetRotation(rotB);
    }

    CollisionWorld::CollisionWorld()
        : m_gravity(Vector3(0, -9.81f, 0))
        , m_collisionCallback(nullptr)
        , m_ccdTestCount(0)
        , m_ccdHitCount(0)
        , m_maxSubSteps(4)
    {
    }

    CollisionWorld::~CollisionWorld()
    {
    }

    uint64_t CollisionWorld::GetCollisionKey(uint32_t bodyA, uint32_t bodyB) const
    {
        if (bodyA > bodyB)
        {
            std::swap(bodyA, bodyB);
        }
        return (uint64_t(bodyA) << 32) | uint64_t(bodyB);
    }

    void CollisionWorld::UpdateBroadPhase(RigidBodyManager& bodyManager, ColliderManager& colliderManager)
    {
        const auto& colliders = colliderManager.GetColliders();

        for (const auto& pair : colliders)
        {
            Collider* collider = pair.second;
            RigidBody* body = bodyManager.GetBody(collider->GetBodyId());

            if (body)
            {
                Vector3 worldPos = body->GetPosition() + body->GetRotation() * collider->GetOffset();
                Quaternion worldRot = body->GetRotation() * collider->GetRotationOffset();
                AABB aabb = collider->ComputeAABB(worldPos, worldRot);
                m_broadPhase.UpdateCollider(collider->GetId(), aabb);
            }
        }

        m_potentialPairs.clear();
        m_broadPhase.GetPotentialPairs(m_potentialPairs, colliders);
    }

    void CollisionWorld::PerformNarrowPhase(RigidBodyManager& bodyManager, ColliderManager& colliderManager)
    {
        for (const auto& pair : m_potentialPairs)
        {
            Collider* colliderA = colliderManager.GetCollider(pair.colliderA);
            Collider* colliderB = colliderManager.GetCollider(pair.colliderB);
            RigidBody* bodyA = bodyManager.GetBody(pair.bodyA);
            RigidBody* bodyB = bodyManager.GetBody(pair.bodyB);

            if (!colliderA || !colliderB || !bodyA || !bodyB)
            {
                continue;
            }

            if (!ShouldCollide(bodyA, bodyB))
            {
                continue;
            }

            if (bodyA->IsSensor() || bodyB->IsSensor())
            {
                continue;
            }

            if (!bodyA->IsAwake() && !bodyB->IsAwake())
            {
                continue;
            }

            if (pair.isCCDPair)
            {
                continue;
            }

            std::vector<CollisionManifold> manifolds;
            if (m_narrowPhase.TestCollision(colliderA, colliderB,
                                            bodyA->GetPosition(), bodyA->GetRotation(),
                                            bodyB->GetPosition(), bodyB->GetRotation(),
                                            manifolds))
            {
                CollisionData data;
                data.bodyA = pair.bodyA;
                data.bodyB = pair.bodyB;
                data.colliderA = pair.colliderA;
                data.colliderB = pair.colliderB;
                data.manifolds = manifolds;
                data.friction = Math::Sqrt(colliderA->GetDynamicFriction() * colliderB->GetDynamicFriction());
                data.restitution = Math::Min(colliderA->GetRestitution(), colliderB->GetRestitution());

                m_activeCollisions.push_back(data);
                m_currentFrameCollisions.insert(GetCollisionKey(pair.bodyA, pair.bodyB));
            }
        }
    }

    void CollisionWorld::ApplyGravity(RigidBodyManager& bodyManager)
    {
        auto& bodies = bodyManager.GetBodies();
        for (auto& pair : bodies)
        {
            RigidBody* body = pair.second;
            if (!body->IsKinematic() && body->IsAwake())
            {
                Vector3 gravityForce = m_gravity * body->GetMass();
                body->ApplyForce(gravityForce, body->GetPosition());
            }
        }
    }

    void CollisionWorld::DispatchCollisionEvents()
    {
        if (!m_collisionCallback)
        {
            return;
        }

        for (const auto& collision : m_activeCollisions)
        {
            uint64_t key = GetCollisionKey(collision.bodyA, collision.bodyB);
            CollisionEventType type;

            if (m_previousFrameCollisions.find(key) == m_previousFrameCollisions.end())
            {
                type = CollisionEventType::ContactStart;
            }
            else
            {
                type = CollisionEventType::ContactPersist;
            }

            m_collisionCallback(type, collision.bodyA, collision.bodyB,
                                collision.manifolds.data(), (uint32_t)collision.manifolds.size());
        }

        for (const auto& key : m_previousFrameCollisions)
        {
            if (m_currentFrameCollisions.find(key) == m_currentFrameCollisions.end())
            {
                uint32_t bodyA = (uint32_t)(key >> 32);
                uint32_t bodyB = (uint32_t)(key & 0xFFFFFFFF);
                m_collisionCallback(CollisionEventType::ContactEnd, bodyA, bodyB, nullptr, 0);
            }
        }

        m_previousFrameCollisions = m_currentFrameCollisions;
    }

    void CollisionWorld::QueryPressureDistribution(uint32_t iceSheetBodyId, uint32_t shipBodyId,
                                                    const Vector3& shipPosition, const Quaternion& shipRotation,
                                                    const Collider* iceCollider, const Collider* shipCollider,
                                                    uint32_t& pointCount, PressurePoint* pressurePoints)
    {
        pointCount = 0;

        if (!iceCollider || !shipCollider)
        {
            return;
        }

        const MeshCollider* iceMesh = dynamic_cast<const MeshCollider*>(iceCollider);
        if (!iceMesh)
        {
            return;
        }

        for (const auto& collision : m_activeCollisions)
        {
            if (collision.bodyA == iceSheetBodyId && collision.bodyB == shipBodyId)
            {
                for (const auto& manifold : collision.manifolds)
                {
                    if (pointCount < 64 && pressurePoints)
                    {
                        pressurePoints[pointCount].position = manifold.point;
                        pressurePoints[pointCount].normal = manifold.normal;
                        pressurePoints[pointCount].pressure = manifold.impulse * 1000.0f;
                        pressurePoints[pointCount].area = 0.01f;
                        pointCount++;
                    }
                }
            }
        }

        const auto& triangles = iceMesh->GetTriangles();
        for (uint32_t i = 0; i < triangles.size() && pointCount < 64; i++)
        {
            Vector3 v0 = iceMesh->GetTriangleVertex(i, 0, shipPosition, shipRotation);
            Vector3 v1 = iceMesh->GetTriangleVertex(i, 1, shipPosition, shipRotation);
            Vector3 v2 = iceMesh->GetTriangleVertex(i, 2, shipPosition, shipRotation);

            Vector3 center = (v0 + v1 + v2) / 3.0f;
            Vector3 normal = Math::TriangleNormal(v0, v1, v2);
            float area = Math::TriangleArea(v0, v1, v2);

            Vector3 toShip = shipPosition - center;
            float dist = Math::Length(toShip);

            if (dist < 5.0f)
            {
                if (pressurePoints)
                {
                    pressurePoints[pointCount].position = center;
                    pressurePoints[pointCount].normal = normal;
                    pressurePoints[pointCount].pressure = Math::Max(0.0f, (5.0f - dist) * 1000.0f);
                    pressurePoints[pointCount].area = area;
                }
                pointCount++;
            }
        }
    }

    bool CollisionWorld::ShouldCollide(const RigidBody* bodyA, const RigidBody* bodyB) const
    {
        if (!bodyA || !bodyB) return false;
        if (bodyA->GetId() == bodyB->GetId()) return false;
        if (!bodyA->CanCollideWith(bodyB)) return false;
        if (!bodyA->IsAwake() && !bodyB->IsAwake()) return false;
        return true;
    }

    void CollisionWorld::CollectCCDPairs(RigidBodyManager& bodyManager, ColliderManager& colliderManager)
    {
        m_ccdPairs.clear();

        for (const auto& pair : m_potentialPairs)
        {
            RigidBody* bodyA = bodyManager.GetBody(pair.bodyA);
            RigidBody* bodyB = bodyManager.GetBody(pair.bodyB);

            if (!bodyA || !bodyB) continue;
            if (!ShouldCollide(bodyA, bodyB)) continue;

            bool isCCDPair = bodyA->IsCCDEnabled() || bodyB->IsCCDEnabled();

            if (isCCDPair)
            {
                float speedA = Math::Length(bodyA->GetLinearVelocity());
                float speedB = Math::Length(bodyB->GetLinearVelocity());
                float relSpeed = Math::Length(bodyA->GetLinearVelocity() - bodyB->GetLinearVelocity());

                float minRadius = Math::Min(bodyA->GetCCDRadius(), bodyB->GetCCDRadius());
                float moveDistance = relSpeed * (1.0f / 60.0f);

                if (moveDistance > minRadius * 0.2f)
                {
                    CollisionPair ccdPair = pair;
                    ccdPair.isCCDPair = true;
                    m_ccdPairs.push_back(ccdPair);
                }
            }
        }
    }

    void CollisionWorld::PerformCCDTests(RigidBodyManager& bodyManager, ColliderManager& colliderManager, float dt)
    {
        m_ccdTestCount = 0;
        m_ccdHitCount = 0;

        for (const auto& pair : m_ccdPairs)
        {
            RigidBody* bodyA = bodyManager.GetBody(pair.bodyA);
            RigidBody* bodyB = bodyManager.GetBody(pair.bodyB);
            Collider* colliderA = colliderManager.GetCollider(pair.colliderA);
            Collider* colliderB = colliderManager.GetCollider(pair.colliderB);

            if (!bodyA || !bodyB || !colliderA || !colliderB) continue;

            m_ccdTestCount++;

            SweepInput sweep;
            sweep.startPosA = bodyA->GetPrevPosition();
            sweep.startRotA = bodyA->GetPrevRotation();
            sweep.endPosA = bodyA->GetPosition();
            sweep.endRotA = bodyA->GetRotation();
            sweep.startPosB = bodyB->GetPrevPosition();
            sweep.startRotB = bodyB->GetPrevRotation();
            sweep.endPosB = bodyB->GetPosition();
            sweep.endRotB = bodyB->GetRotation();
            sweep.tolerance = 0.001f;
            sweep.maxIterations = 32;

            TOIResult result = m_narrowPhase.ConservativeAdvancement(colliderA, colliderB, sweep);

            if (result.hasCollision && result.time >= 0.0f && result.time < 1.0f)
            {
                m_ccdHitCount++;

                float toi = result.time;
                Vector3 interpolatedPosA = Math::Lerp(sweep.startPosA, sweep.endPosA, toi);
                Quaternion interpolatedRotA = Math::QuatSlerp(sweep.startRotA, sweep.endRotA, toi);
                Vector3 interpolatedPosB = Math::Lerp(sweep.startPosB, sweep.endPosB, toi);
                Quaternion interpolatedRotB = Math::QuatSlerp(sweep.startRotB, sweep.endRotB, toi);

                bodyA->SetPosition(interpolatedPosA);
                bodyA->SetRotation(interpolatedRotA);
                bodyB->SetPosition(interpolatedPosB);
                bodyB->SetRotation(interpolatedRotB);

                CollisionData ccdCollision;
                ccdCollision.bodyA = pair.bodyA;
                ccdCollision.bodyB = pair.bodyB;
                ccdCollision.colliderA = pair.colliderA;
                ccdCollision.colliderB = pair.colliderB;
                ccdCollision.friction = 0.5f;
                ccdCollision.restitution = 0.1f;

                CollisionManifold manifold;
                manifold.normal = result.normal;
                manifold.point = result.point;
                manifold.penetration = Math::Max(result.penetration, 0.001f);
                manifold.impulse = 0.0f;
                ccdCollision.manifolds.push_back(manifold);

                m_activeCollisions.push_back(ccdCollision);
            }
        }
    }

    void CollisionWorld::Step(float dt, RigidBodyManager& bodyManager, ColliderManager& colliderManager)
    {
        m_ccdTestCount = 0;
        m_ccdHitCount = 0;
        m_activeCollisions.clear();
        m_currentFrameCollisions.clear();

        ApplyGravity(bodyManager);

        bodyManager.IntegrateAwake(dt);

        UpdateBroadPhase(bodyManager, colliderManager);

        CollectCCDPairs(bodyManager, colliderManager);

        if (!m_ccdPairs.empty())
        {
            PerformCCDTests(bodyManager, colliderManager, dt);
        }

        PerformNarrowPhase(bodyManager, colliderManager);

        if (m_contactSolver.IsEnabled())
        {
            m_contactSolver.Solve(m_activeCollisions, bodyManager, dt);
        }

        DispatchCollisionEvents();
    }

    TOIResult NarrowPhase::ConservativeAdvancement(const Collider* colliderA, const Collider* colliderB,
                                                   const SweepInput& sweep)
    {
        TOIResult result;
        result.hasCollision = false;
        result.time = 1.0f;
        result.iterations = 0;

        Vector3 posA = sweep.startPosA;
        Quaternion rotA = sweep.startRotA;
        Vector3 posB = sweep.startPosB;
        Quaternion rotB = sweep.startRotB;

        float currentTime = 0.0f;
        float tolerance = sweep.tolerance;
        uint32_t maxIterations = sweep.maxIterations;

        Vector3 closestPointA, closestPointB;
        float initialDistance = GJKDistance(colliderA, colliderB, posA, rotA, posB, rotB, closestPointA, closestPointB);

        if (initialDistance <= 0.0f)
        {
            result.hasCollision = true;
            result.time = 0.0f;
            result.normal = Math::Normalize(closestPointB - closestPointA);
            result.point = (closestPointA + closestPointB) * 0.5f;
            result.penetration = -initialDistance;
            return result;
        }

        for (uint32_t iter = 0; iter < maxIterations; iter++)
        {
            result.iterations = iter + 1;

            float distance = GJKDistance(colliderA, colliderB, posA, rotA, posB, rotB, closestPointA, closestPointB);

            if (distance <= tolerance)
            {
                result.hasCollision = true;
                result.time = currentTime;
                Vector3 diff = closestPointB - closestPointA;
                float len = Math::Length(diff);
                result.normal = len > Math::EPSILON ? diff / len : Vector3(0, 1, 0);
                result.point = (closestPointA + closestPointB) * 0.5f;
                result.penetration = Math::Max(-distance, 0.0f);
                return result;
            }

            Vector3 direction = closestPointA - closestPointB;
            float dirLen = Math::Length(direction);
            if (dirLen < Math::EPSILON) break;
            direction /= dirLen;

            Vector3 linVelA = (sweep.endPosA - sweep.startPosA);
            Vector3 linVelB = (sweep.endPosB - sweep.startPosB);
            Vector3 relVel = linVelA - linVelB;

            float approachSpeed = Math::Dot(relVel, direction);

            if (approachSpeed <= 0.0f)
            {
                break;
            }

            float stepTime = distance / approachSpeed;
            stepTime = Math::Min(stepTime, 1.0f - currentTime);

            if (stepTime < tolerance * 0.1f)
            {
                break;
            }

            currentTime += stepTime;

            if (currentTime >= 1.0f - tolerance)
            {
                break;
            }

            posA = Math::Lerp(sweep.startPosA, sweep.endPosA, currentTime);
            rotA = Math::QuatSlerp(sweep.startRotA, sweep.endRotA, currentTime);
            posB = Math::Lerp(sweep.startPosB, sweep.endPosB, currentTime);
            rotB = Math::QuatSlerp(sweep.startRotB, sweep.endRotB, currentTime);
        }

        return result;
    }

    TOIResult NarrowPhase::LinearCast(const Collider* colliderA, const Collider* colliderB,
                                      const Vector3& posA, const Quaternion& rotA,
                                      const Vector3& posB, const Quaternion& rotB,
                                      const Vector3& deltaA, const Vector3& deltaB,
                                      float maxDistance)
    {
        TOIResult result;
        result.hasCollision = false;
        result.time = 1.0f;
        result.iterations = 0;

        SweepInput sweep;
        sweep.startPosA = posA;
        sweep.startRotA = rotA;
        sweep.endPosA = posA + deltaA;
        sweep.endRotA = rotA;
        sweep.startPosB = posB;
        sweep.startRotB = rotB;
        sweep.endPosB = posB + deltaB;
        sweep.endRotB = rotB;
        sweep.tolerance = 0.001f;
        sweep.maxIterations = 32;

        return ConservativeAdvancement(colliderA, colliderB, sweep);
    }

    float NarrowPhase::GJKDistance(const Collider* colliderA, const Collider* colliderB,
                                   const Vector3& posA, const Quaternion& rotA,
                                   const Vector3& posB, const Quaternion& rotB,
                                   Vector3& outClosestPointA, Vector3& outClosestPointB)
    {
        Simplex simplex;
        Vector3 direction = Vector3(1, 0, 0);

        SimplexPoint initialPoint;
        GetSupport(colliderA, colliderB, posA, rotA, posB, rotB, direction, initialPoint);
        simplex.points[0] = initialPoint;
        simplex.count = 1;

        direction = initialPoint.point * -1.0f;

        const uint32_t MAX_GJK_ITERATIONS = 64;
        for (uint32_t i = 0; i < MAX_GJK_ITERATIONS; i++)
        {
            SimplexPoint newPoint;
            GetSupport(colliderA, colliderB, posA, rotA, posB, rotB, direction, newPoint);

            float dot = Math::Dot(newPoint.point, direction);
            if (dot > 0.0f)
            {
                float dist = Math::Length(newPoint.point);
                outClosestPointA = newPoint.supportA;
                outClosestPointB = newPoint.supportB;
                return dist;
            }

            if (simplex.count < 4)
            {
                simplex.points[simplex.count++] = newPoint;
            }
            else
            {
                return 0.0f;
            }

            bool containsOrigin = GJKNextSimplex(simplex, direction);
            if (containsOrigin)
            {
                outClosestPointA = (simplex.count > 0) ? simplex.points[0].supportA : posA;
                outClosestPointB = (simplex.count > 0) ? simplex.points[0].supportB : posB;
                return 0.0f;
            }
        }

        if (simplex.count > 0)
        {
            outClosestPointA = simplex.points[0].supportA;
            outClosestPointB = simplex.points[0].supportB;
            return Math::Length(simplex.points[0].point);
        }

        return Math::Length(posA - posB);
    }

    bool ContactSolver::CheckEarlyExit(const std::vector<ContactConstraint>& constraints) const
    {
        if (constraints.empty()) return true;

        float maxImpulse = 0.0f;
        for (const auto& c : constraints)
        {
            if (c.isSleeping) continue;
            maxImpulse = Math::Max(maxImpulse, Math::Abs(c.normalImpulse));
            maxImpulse = Math::Max(maxImpulse, Math::Abs(c.tangent1Impulse));
            maxImpulse = Math::Max(maxImpulse, Math::Abs(c.tangent2Impulse));
        }

        return maxImpulse < m_velocitySleepThreshold;
    }
}
