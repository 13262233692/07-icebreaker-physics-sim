#pragma once

#include "IcePhysicsAPI.h"
#include "MathUtils.h"
#include <vector>
#include <unordered_map>

namespace IcePhysics
{
    enum class ColliderType : uint32_t
    {
        Box = 0,
        Sphere = 1,
        Capsule = 2,
        Hull = 3,
        Mesh = 4
    };

    class Collider
    {
    public:
        Collider(uint32_t bodyId, ColliderType type);
        virtual ~Collider();

        ColliderType GetType() const { return m_type; }
        uint32_t GetBodyId() const { return m_bodyId; }
        uint32_t GetId() const { return m_id; }
        void SetId(uint32_t id) { m_id = id; }

        Vector3 GetOffset() const { return m_offset; }
        void SetOffset(const Vector3& offset) { m_offset = offset; m_isDirty = true; }

        Quaternion GetRotationOffset() const { return m_rotationOffset; }
        void SetRotationOffset(const Quaternion& rotation) { m_rotationOffset = rotation; m_isDirty = true; }

        float GetStaticFriction() const { return m_staticFriction; }
        void SetStaticFriction(float friction) { m_staticFriction = friction; }

        float GetDynamicFriction() const { return m_dynamicFriction; }
        void SetDynamicFriction(float friction) { m_dynamicFriction = friction; }

        float GetRestitution() const { return m_restitution; }
        void SetRestitution(float restitution) { m_restitution = restitution; }

        bool IsDirty() const { return m_isDirty; }
        void ClearDirty() { m_isDirty = false; }

        virtual AABB ComputeAABB(const Vector3& position, const Quaternion& rotation) const = 0;
        virtual Vector3 GetSupportPoint(const Vector3& direction, const Vector3& position, const Quaternion& rotation) const = 0;

    protected:
        uint32_t m_id;
        uint32_t m_bodyId;
        ColliderType m_type;
        Vector3 m_offset;
        Quaternion m_rotationOffset;
        float m_staticFriction;
        float m_dynamicFriction;
        float m_restitution;
        bool m_isDirty;
    };

    class BoxCollider : public Collider
    {
    public:
        BoxCollider(uint32_t bodyId, const Vector3& halfExtents, const Vector3& offset, const Quaternion& rotationOffset);
        ~BoxCollider() override;

        Vector3 GetHalfExtents() const { return m_halfExtents; }
        void SetHalfExtents(const Vector3& extents) { m_halfExtents = extents; m_isDirty = true; }

        AABB ComputeAABB(const Vector3& position, const Quaternion& rotation) const override;
        Vector3 GetSupportPoint(const Vector3& direction, const Vector3& position, const Quaternion& rotation) const override;

    private:
        Vector3 m_halfExtents;
        std::vector<Vector3> m_vertices;
        void UpdateVertices();
    };

    class SphereCollider : public Collider
    {
    public:
        SphereCollider(uint32_t bodyId, float radius, const Vector3& offset);
        ~SphereCollider() override;

        float GetRadius() const { return m_radius; }
        void SetRadius(float radius) { m_radius = radius; m_isDirty = true; }

        AABB ComputeAABB(const Vector3& position, const Quaternion& rotation) const override;
        Vector3 GetSupportPoint(const Vector3& direction, const Vector3& position, const Quaternion& rotation) const override;

    private:
        float m_radius;
    };

    class HullCollider : public Collider
    {
    public:
        HullCollider(uint32_t bodyId, const std::vector<Vector3>& points, const Vector3& offset, const Quaternion& rotationOffset);
        ~HullCollider() override;

        const std::vector<Vector3>& GetVertices() const { return m_vertices; }
        const std::vector<HullFace>& GetFaces() const { return m_faces; }

        AABB ComputeAABB(const Vector3& position, const Quaternion& rotation) const override;
        Vector3 GetSupportPoint(const Vector3& direction, const Vector3& position, const Quaternion& rotation) const override;

    private:
        std::vector<Vector3> m_vertices;
        std::vector<HullFace> m_faces;
        void BuildHull(const std::vector<Vector3>& points);
    };

    class MeshCollider : public Collider
    {
    public:
        MeshCollider(uint32_t bodyId, const std::vector<Vertex>& vertices, const std::vector<Triangle>& triangles, const Vector3& offset, const Quaternion& rotationOffset);
        ~MeshCollider() override;

        const std::vector<Vertex>& GetVertices() const { return m_vertices; }
        const std::vector<Triangle>& GetTriangles() const { return m_triangles; }

        AABB ComputeAABB(const Vector3& position, const Quaternion& rotation) const override;
        Vector3 GetSupportPoint(const Vector3& direction, const Vector3& position, const Quaternion& rotation) const override;

        Vector3 GetTriangleVertex(uint32_t triangleIndex, uint32_t vertexIndex, const Vector3& position, const Quaternion& rotation) const;

    private:
        std::vector<Vertex> m_vertices;
        std::vector<Triangle> m_triangles;
        AABB m_localAABB;
        void ComputeLocalAABB();
    };

    class ColliderManager
    {
    public:
        ColliderManager();
        ~ColliderManager();

        uint32_t CreateBoxCollider(uint32_t bodyId, const Vector3& halfExtents, const Vector3& offset, const Quaternion& rotationOffset);
        uint32_t CreateSphereCollider(uint32_t bodyId, float radius, const Vector3& offset);
        uint32_t CreateHullCollider(uint32_t bodyId, const Vector3* points, uint32_t pointCount, const Vector3& offset, const Quaternion& rotationOffset);
        uint32_t CreateMeshCollider(uint32_t bodyId, const Vertex* vertices, uint32_t vertexCount, const Triangle* triangles, uint32_t triangleCount, const Vector3& offset, const Quaternion& rotationOffset);

        void DestroyCollider(uint32_t colliderId);
        Collider* GetCollider(uint32_t colliderId);
        const Collider* GetCollider(uint32_t colliderId) const;

        uint32_t GetCount() const { return (uint32_t)m_colliders.size(); }
        const std::unordered_map<uint32_t, Collider*>& GetColliders() const { return m_colliders; }

        std::vector<Collider*> GetCollidersForBody(uint32_t bodyId) const;

    private:
        std::unordered_map<uint32_t, Collider*> m_colliders;
        std::unordered_map<uint32_t, std::vector<uint32_t>> m_bodyToColliders;
        uint32_t m_nextColliderId;
    };
}
