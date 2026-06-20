#pragma once

#include "IcePhysicsAPI.h"
#include "MathUtils.h"
#include "Collider.h"
#include "RigidBody.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace IcePhysics
{
    struct CollisionPair
    {
        uint32_t bodyA;
        uint32_t bodyB;
        uint32_t colliderA;
        uint32_t colliderB;
        bool isCCDPair;
    };

    struct TOIResult
    {
        bool hasCollision;
        float time;
        Vector3 normal;
        Vector3 point;
        float penetration;
        uint32_t iterations;
    };

    struct SweepInput
    {
        Vector3 startPosA;
        Quaternion startRotA;
        Vector3 endPosA;
        Quaternion endRotA;
        Vector3 startPosB;
        Quaternion startRotB;
        Vector3 endPosB;
        Quaternion endRotB;
        float tolerance;
        uint32_t maxIterations;
    };

    struct CollisionData
    {
        uint32_t bodyA;
        uint32_t bodyB;
        uint32_t colliderA;
        uint32_t colliderB;
        std::vector<CollisionManifold> manifolds;
        float friction;
        float restitution;
    };

    class BroadPhase
    {
    public:
        BroadPhase();
        ~BroadPhase();

        void UpdateCollider(uint32_t colliderId, const AABB& aabb);
        void RemoveCollider(uint32_t colliderId);
        void GetPotentialPairs(std::vector<CollisionPair>& pairs, const std::unordered_map<uint32_t, Collider*>& colliders);

        uint32_t GetProxyId(uint32_t colliderId) const;
        AABB GetProxyAABB(uint32_t proxyId) const;

    private:
        struct Proxy
        {
            uint32_t proxyId;
            uint32_t colliderId;
            AABB aabb;
        };

        std::unordered_map<uint32_t, Proxy> m_proxies;
        std::unordered_map<uint32_t, uint32_t> m_colliderToProxy;
        uint32_t m_nextProxyId;

        bool Overlap(const AABB& a, const AABB& b) const;
    };

    class NarrowPhase
    {
    public:
        NarrowPhase();
        ~NarrowPhase();

        bool TestCollision(const Collider* colliderA, const Collider* colliderB,
                           const Vector3& posA, const Quaternion& rotA,
                           const Vector3& posB, const Quaternion& rotB,
                           std::vector<CollisionManifold>& manifolds);

        bool TestBoxBox(const BoxCollider* boxA, const BoxCollider* boxB,
                        const Vector3& posA, const Quaternion& rotA,
                        const Vector3& posB, const Quaternion& rotB,
                        std::vector<CollisionManifold>& manifolds);

        bool TestBoxSphere(const BoxCollider* box, const SphereCollider* sphere,
                           const Vector3& posA, const Quaternion& rotA,
                           const Vector3& posB, const Quaternion& rotB,
                           std::vector<CollisionManifold>& manifolds);

        bool TestBoxMesh(const BoxCollider* box, const MeshCollider* mesh,
                         const Vector3& posA, const Quaternion& rotA,
                         const Vector3& posB, const Quaternion& rotB,
                         std::vector<CollisionManifold>& manifolds);

        bool TestHullMesh(const HullCollider* hull, const MeshCollider* mesh,
                          const Vector3& posA, const Quaternion& rotA,
                          const Vector3& posB, const Quaternion& rotB,
                          std::vector<CollisionManifold>& manifolds);

        bool GJK(const Collider* colliderA, const Collider* colliderB,
                 const Vector3& posA, const Quaternion& rotA,
                 const Vector3& posB, const Quaternion& rotB,
                 Vector3& outNormal, float& outPenetration, Vector3& outPoint);

        TOIResult ConservativeAdvancement(const Collider* colliderA, const Collider* colliderB,
                                          const SweepInput& sweep);

        TOIResult LinearCast(const Collider* colliderA, const Collider* colliderB,
                             const Vector3& posA, const Quaternion& rotA,
                             const Vector3& posB, const Quaternion& rotB,
                             const Vector3& deltaA, const Vector3& deltaB,
                             float maxDistance);

        float GJKDistance(const Collider* colliderA, const Collider* colliderB,
                          const Vector3& posA, const Quaternion& rotA,
                          const Vector3& posB, const Quaternion& rotB,
                          Vector3& outClosestPointA, Vector3& outClosestPointB);

    private:
        struct SimplexPoint
        {
            Vector3 supportA;
            Vector3 supportB;
            Vector3 point;
        };

        struct Simplex
        {
            SimplexPoint points[4];
            uint32_t count;

            Simplex() : count(0) {}
        };

        bool GJKNextSimplex(Simplex& simplex, Vector3& direction);
        bool GJKSimplexLine(Simplex& simplex, Vector3& direction);
        bool GJKSimplexTriangle(Simplex& simplex, Vector3& direction);
        bool GJKSimplexTetrahedron(Simplex& simplex, Vector3& direction);

        Vector3 GetSupport(const Collider* colliderA, const Collider* colliderB,
                           const Vector3& posA, const Quaternion& rotA,
                           const Vector3& posB, const Quaternion& rotB,
                           const Vector3& direction, SimplexPoint& outPoint);

        float EPA(const Simplex& simplex, const Collider* colliderA, const Collider* colliderB,
                  const Vector3& posA, const Quaternion& rotA,
                  const Vector3& posB, const Quaternion& rotB,
                  Vector3& outNormal, Vector3& outPoint);

        struct EPAFace
        {
            Vector3 normal;
            float distance;
            uint32_t indices[3];
        };

        std::vector<EPAFace> m_epaFaces;
        std::vector<SimplexPoint> m_epaPoints;

        void EPABuildInitialPolytope(const Simplex& simplex);
        uint32_t EPAFindClosestFace();
        void EPAExpand(uint32_t closestFaceIndex, const SimplexPoint& newPoint);
    };

    class ContactSolver
    {
    public:
        ContactSolver();
        ~ContactSolver();

        void Solve(std::vector<CollisionData>& collisions, RigidBodyManager& bodyManager, float dt);
        void SetIterations(uint32_t velocityIterations, uint32_t positionIterations);

        void SetEnabled(bool enabled) { m_enabled = enabled; }
        bool IsEnabled() const { return m_enabled; }

        uint32_t GetLastIterationCount() const { return m_lastIterationCount; }

    private:
        uint32_t m_velocityIterations;
        uint32_t m_positionIterations;
        float m_baumgarte;
        float m_allowedPenetration;
        bool m_enabled;
        uint32_t m_lastIterationCount;
        float m_velocitySleepThreshold;
        float m_positionSleepThreshold;

        struct ContactConstraint
        {
            uint32_t bodyA;
            uint32_t bodyB;
            Vector3 normal;
            Vector3 tangent1;
            Vector3 tangent2;
            Vector3 ra;
            Vector3 rb;
            Vector3 raxN;
            Vector3 rbxN;
            Vector3 raxT1;
            Vector3 rbxT1;
            Vector3 raxT2;
            Vector3 rbxT2;
            float normalMass;
            float tangent1Mass;
            float tangent2Mass;
            float penetration;
            float restitution;
            float friction;
            float normalImpulse;
            float tangent1Impulse;
            float tangent2Impulse;
            float bias;
            bool isSleeping;
        };

        void SetupContactConstraint(ContactConstraint& constraint, const CollisionManifold& manifold,
                                    RigidBody* bodyA, RigidBody* bodyB, float dt);
        void SolveVelocityConstraint(ContactConstraint& constraint, RigidBody* bodyA, RigidBody* bodyB);
        void SolvePositionConstraint(ContactConstraint& constraint, RigidBody* bodyA, RigidBody* bodyB);

        bool CheckEarlyExit(const std::vector<ContactConstraint>& constraints) const;
    };

    class CollisionWorld
    {
    public:
        CollisionWorld();
        ~CollisionWorld();

        void SetGravity(const Vector3& gravity) { m_gravity = gravity; }
        Vector3 GetGravity() const { return m_gravity; }

        void Step(float dt, RigidBodyManager& bodyManager, ColliderManager& colliderManager);

        uint32_t GetActiveCollisionCount() const { return (uint32_t)m_activeCollisions.size(); }
        const std::vector<CollisionData>& GetActiveCollisions() const { return m_activeCollisions; }

        void SetCollisionEventCallback(CollisionEventCallback callback) { m_collisionCallback = callback; }
        void DispatchCollisionEvents();

        void QueryPressureDistribution(uint32_t iceSheetBodyId, uint32_t shipBodyId,
                                        const Vector3& shipPosition, const Quaternion& shipRotation,
                                        const Collider* iceCollider, const Collider* shipCollider,
                                        uint32_t& pointCount, PressurePoint* pressurePoints);

        uint32_t GetCCDTestCount() const { return m_ccdTestCount; }
        uint32_t GetCCDHitCount() const { return m_ccdHitCount; }

        void SetMaxSubSteps(uint32_t steps) { m_maxSubSteps = steps; }
        uint32_t GetMaxSubSteps() const { return m_maxSubSteps; }

    private:
        Vector3 m_gravity;
        BroadPhase m_broadPhase;
        NarrowPhase m_narrowPhase;
        ContactSolver m_contactSolver;

        std::vector<CollisionPair> m_potentialPairs;
        std::vector<CollisionPair> m_ccdPairs;
        std::vector<CollisionData> m_activeCollisions;
        std::unordered_set<uint64_t> m_previousFrameCollisions;
        std::unordered_set<uint64_t> m_currentFrameCollisions;

        CollisionEventCallback m_collisionCallback;

        uint32_t m_ccdTestCount;
        uint32_t m_ccdHitCount;
        uint32_t m_maxSubSteps;

        uint64_t GetCollisionKey(uint32_t bodyA, uint32_t bodyB) const;

        void UpdateBroadPhase(RigidBodyManager& bodyManager, ColliderManager& colliderManager);
        void PerformNarrowPhase(RigidBodyManager& bodyManager, ColliderManager& colliderManager);
        void ApplyGravity(RigidBodyManager& bodyManager);

        void CollectCCDPairs(RigidBodyManager& bodyManager, ColliderManager& colliderManager);
        void PerformCCDTests(RigidBodyManager& bodyManager, ColliderManager& colliderManager, float dt);
        bool ShouldCollide(const RigidBody* bodyA, const RigidBody* bodyB) const;
    };
}
