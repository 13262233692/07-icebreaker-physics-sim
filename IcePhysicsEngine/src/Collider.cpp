#include "../include/Collider.h"

namespace IcePhysics
{
    Collider::Collider(uint32_t bodyId, ColliderType type)
        : m_id(0)
        , m_bodyId(bodyId)
        , m_type(type)
        , m_offset(Math::Vec3Zero())
        , m_rotationOffset(Math::QuatIdentity())
        , m_staticFriction(0.5f)
        , m_dynamicFriction(0.3f)
        , m_restitution(0.2f)
        , m_isDirty(true)
    {
    }

    Collider::~Collider()
    {
    }

    BoxCollider::BoxCollider(uint32_t bodyId, const Vector3& halfExtents, const Vector3& offset, const Quaternion& rotationOffset)
        : Collider(bodyId, ColliderType::Box)
        , m_halfExtents(halfExtents)
    {
        m_offset = offset;
        m_rotationOffset = Math::QuatNormalize(rotationOffset);
        UpdateVertices();
    }

    BoxCollider::~BoxCollider()
    {
    }

    void BoxCollider::UpdateVertices()
    {
        m_vertices.clear();
        m_vertices.reserve(8);

        m_vertices.push_back(Vector3(-m_halfExtents.x, -m_halfExtents.y, -m_halfExtents.z));
        m_vertices.push_back(Vector3(m_halfExtents.x, -m_halfExtents.y, -m_halfExtents.z));
        m_vertices.push_back(Vector3(m_halfExtents.x, m_halfExtents.y, -m_halfExtents.z));
        m_vertices.push_back(Vector3(-m_halfExtents.x, m_halfExtents.y, -m_halfExtents.z));
        m_vertices.push_back(Vector3(-m_halfExtents.x, -m_halfExtents.y, m_halfExtents.z));
        m_vertices.push_back(Vector3(m_halfExtents.x, -m_halfExtents.y, m_halfExtents.z));
        m_vertices.push_back(Vector3(m_halfExtents.x, m_halfExtents.y, m_halfExtents.z));
        m_vertices.push_back(Vector3(-m_halfExtents.x, m_halfExtents.y, m_halfExtents.z));
    }

    AABB BoxCollider::ComputeAABB(const Vector3& position, const Quaternion& rotation) const
    {
        Quaternion worldRotation = Math::QuatNormalize(rotation * m_rotationOffset);
        Vector3 worldPosition = position + rotation * m_offset;

        AABB localAABB = Math::AABBFromCenterExtents(Math::Vec3Zero(), m_halfExtents);
        return Math::AABBTransform(localAABB, worldPosition, worldRotation);
    }

    Vector3 BoxCollider::GetSupportPoint(const Vector3& direction, const Vector3& position, const Quaternion& rotation) const
    {
        Quaternion worldRotation = Math::QuatNormalize(rotation * m_rotationOffset);
        Vector3 worldPosition = position + rotation * m_offset;

        Vector3 localDir = Math::QuatInverse(worldRotation) * direction;

        Vector3 supportPoint(
            Math::Sign(localDir.x) * m_halfExtents.x,
            Math::Sign(localDir.y) * m_halfExtents.y,
            Math::Sign(localDir.z) * m_halfExtents.z
        );

        return worldPosition + worldRotation * supportPoint;
    }

    SphereCollider::SphereCollider(uint32_t bodyId, float radius, const Vector3& offset)
        : Collider(bodyId, ColliderType::Sphere)
        , m_radius(radius)
    {
        m_offset = offset;
    }

    SphereCollider::~SphereCollider()
    {
    }

    AABB SphereCollider::ComputeAABB(const Vector3& position, const Quaternion& rotation) const
    {
        Vector3 worldPosition = position + rotation * m_offset;
        Vector3 extents(m_radius, m_radius, m_radius);
        return Math::AABBFromCenterExtents(worldPosition, extents);
    }

    Vector3 SphereCollider::GetSupportPoint(const Vector3& direction, const Vector3& position, const Quaternion& rotation) const
    {
        Vector3 worldPosition = position + rotation * m_offset;
        Vector3 normalizedDir = Math::Normalize(direction);
        return worldPosition + normalizedDir * m_radius;
    }

    HullCollider::HullCollider(uint32_t bodyId, const std::vector<Vector3>& points, const Vector3& offset, const Quaternion& rotationOffset)
        : Collider(bodyId, ColliderType::Hull)
    {
        m_offset = offset;
        m_rotationOffset = Math::QuatNormalize(rotationOffset);
        BuildHull(points);
    }

    HullCollider::~HullCollider()
    {
    }

    void HullCollider::BuildHull(const std::vector<Vector3>& points)
    {
        m_vertices.clear();
        m_faces.clear();

        if (points.size() < 4)
        {
            m_vertices = points;
            return;
        }

        std::vector<Vector3> tempPoints = points;

        size_t extremeIndices[4] = { 0, 0, 0, 0 };
        for (size_t i = 1; i < tempPoints.size(); i++)
        {
            if (tempPoints[i].x < tempPoints[extremeIndices[0]].x) extremeIndices[0] = i;
            if (tempPoints[i].x > tempPoints[extremeIndices[1]].x) extremeIndices[1] = i;
            if (tempPoints[i].y < tempPoints[extremeIndices[2]].y) extremeIndices[2] = i;
            if (tempPoints[i].y > tempPoints[extremeIndices[3]].y) extremeIndices[3] = i;
        }

        float maxDist = 0.0f;
        size_t idx0 = extremeIndices[0];
        size_t idx1 = extremeIndices[1];
        size_t idx2 = extremeIndices[2];
        size_t idx3 = extremeIndices[3];

        for (size_t i = 0; i < tempPoints.size(); i++)
        {
            float dist = Math::LengthSquared(tempPoints[i] - tempPoints[idx0]);
            if (dist > maxDist)
            {
                maxDist = dist;
                idx1 = i;
            }
        }

        maxDist = 0.0f;
        for (size_t i = 0; i < tempPoints.size(); i++)
        {
            Vector3 ab = tempPoints[idx1] - tempPoints[idx0];
            Vector3 ac = tempPoints[i] - tempPoints[idx0];
            float dist = Math::LengthSquared(Math::Cross(ab, ac));
            if (dist > maxDist)
            {
                maxDist = dist;
                idx2 = i;
            }
        }

        maxDist = 0.0f;
        Vector3 ab = tempPoints[idx1] - tempPoints[idx0];
        Vector3 ac = tempPoints[idx2] - tempPoints[idx0];
        Vector3 normal = Math::Normalize(Math::Cross(ab, ac));
        for (size_t i = 0; i < tempPoints.size(); i++)
        {
            Vector3 ad = tempPoints[i] - tempPoints[idx0];
            float dist = Math::Abs(Math::Dot(ad, normal));
            if (dist > maxDist)
            {
                maxDist = dist;
                idx3 = i;
            }
        }

        m_vertices.push_back(tempPoints[idx0]);
        m_vertices.push_back(tempPoints[idx1]);
        m_vertices.push_back(tempPoints[idx2]);
        m_vertices.push_back(tempPoints[idx3]);

        struct Face
        {
            uint32_t a, b, c;
            Vector3 normal;
            float offset;
        };

        std::vector<Face> faces;
        faces.push_back({ 0, 1, 2 });
        faces.push_back({ 0, 3, 1 });
        faces.push_back({ 0, 2, 3 });
        faces.push_back({ 1, 3, 2 });

        for (auto& face : faces)
        {
            Vector3 a = m_vertices[face.a];
            Vector3 b = m_vertices[face.b];
            Vector3 c = m_vertices[face.c];
            face.normal = Math::Normalize(Math::Cross(b - a, c - a));
            face.offset = -Math::Dot(face.normal, a);
        }

        for (size_t pointIdx = 0; pointIdx < tempPoints.size(); pointIdx++)
        {
            if (pointIdx == idx0 || pointIdx == idx1 || pointIdx == idx2 || pointIdx == idx3)
                continue;

            const Vector3& p = tempPoints[pointIdx];
            bool inside = true;

            for (const auto& face : faces)
            {
                float dist = Math::Dot(face.normal, p) + face.offset;
                if (dist > Math::EPSILON)
                {
                    inside = false;
                    break;
                }
            }

            if (inside)
                continue;

            std::vector<Face> visibleFaces;
            for (const auto& face : faces)
            {
                float dist = Math::Dot(face.normal, p) + face.offset;
                if (dist > -Math::EPSILON)
                {
                    visibleFaces.push_back(face);
                }
            }

            std::vector<std::pair<uint32_t, uint32_t>> horizonEdges;
            for (const auto& face : visibleFaces)
            {
                std::pair<uint32_t, uint32_t> edges[3] = {
                    { face.a, face.b },
                    { face.b, face.c },
                    { face.c, face.a }
                };

                for (const auto& edge : edges)
                {
                    bool shared = false;
                    for (const auto& otherFace : visibleFaces)
                    {
                        if (&face == &otherFace) continue;
                        std::pair<uint32_t, uint32_t> otherEdges[3] = {
                            { otherFace.a, otherFace.b },
                            { otherFace.b, otherFace.c },
                            { otherFace.c, otherFace.a }
                        };
                        for (const auto& otherEdge : otherEdges)
                        {
                            if ((edge.first == otherEdge.first && edge.second == otherEdge.second) ||
                                (edge.first == otherEdge.second && edge.second == otherEdge.first))
                            {
                                shared = true;
                                break;
                            }
                        }
                        if (shared) break;
                    }
                    if (!shared)
                    {
                        horizonEdges.push_back(edge);
                    }
                }
            }

            faces.erase(std::remove_if(faces.begin(), faces.end(),
                [&visibleFaces](const Face& f) {
                    for (const auto& vf : visibleFaces)
                    {
                        if (vf.a == f.a && vf.b == f.b && vf.c == f.c)
                            return true;
                    }
                    return false;
                }), faces.end());

            uint32_t newVertexIdx = (uint32_t)m_vertices.size();
            m_vertices.push_back(p);

            for (const auto& edge : horizonEdges)
            {
                Face newFace = { edge.first, edge.second, newVertexIdx };
                Vector3 a = m_vertices[newFace.a];
                Vector3 b = m_vertices[newFace.b];
                Vector3 c = m_vertices[newFace.c];
                newFace.normal = Math::Normalize(Math::Cross(b - a, c - a));
                newFace.offset = -Math::Dot(newFace.normal, a);
                faces.push_back(newFace);
            }
        }

        for (const auto& face : faces)
        {
            HullFace hullFace;
            hullFace.normal = face.normal;
            hullFace.distance = -face.offset;
            hullFace.firstVertex = (uint32_t)m_vertices.size();
            hullFace.vertexCount = 3;
            m_faces.push_back(hullFace);

            m_vertices.push_back(m_vertices[face.a]);
            m_vertices.push_back(m_vertices[face.b]);
            m_vertices.push_back(m_vertices[face.c]);
        }
    }

    AABB HullCollider::ComputeAABB(const Vector3& position, const Quaternion& rotation) const
    {
        if (m_vertices.empty())
        {
            Vector3 worldPosition = position + rotation * m_offset;
            return Math::AABBFromCenterExtents(worldPosition, Math::Vec3Zero());
        }

        Quaternion worldRotation = Math::QuatNormalize(rotation * m_rotationOffset);
        Vector3 worldPosition = position + rotation * m_offset;

        Vector3 transformed = worldPosition + worldRotation * m_vertices[0];
        AABB result = Math::AABBCreate(transformed, transformed);

        for (size_t i = 1; i < m_vertices.size(); i++)
        {
            transformed = worldPosition + worldRotation * m_vertices[i];
            result = Math::AABBExpand(result, transformed);
        }

        return result;
    }

    Vector3 HullCollider::GetSupportPoint(const Vector3& direction, const Vector3& position, const Quaternion& rotation) const
    {
        if (m_vertices.empty())
        {
            return position + rotation * m_offset;
        }

        Quaternion worldRotation = Math::QuatNormalize(rotation * m_rotationOffset);
        Vector3 worldPosition = position + rotation * m_offset;

        Vector3 localDir = Math::QuatInverse(worldRotation) * direction;

        float maxDot = Math::Dot(m_vertices[0], localDir);
        uint32_t bestIdx = 0;

        for (uint32_t i = 1; i < (uint32_t)m_vertices.size(); i++)
        {
            float dot = Math::Dot(m_vertices[i], localDir);
            if (dot > maxDot)
            {
                maxDot = dot;
                bestIdx = i;
            }
        }

        return worldPosition + worldRotation * m_vertices[bestIdx];
    }

    MeshCollider::MeshCollider(uint32_t bodyId, const std::vector<Vertex>& vertices, const std::vector<Triangle>& triangles, const Vector3& offset, const Quaternion& rotationOffset)
        : Collider(bodyId, ColliderType::Mesh)
        , m_vertices(vertices)
        , m_triangles(triangles)
    {
        m_offset = offset;
        m_rotationOffset = Math::QuatNormalize(rotationOffset);
        ComputeLocalAABB();
    }

    MeshCollider::~MeshCollider()
    {
    }

    void MeshCollider::ComputeLocalAABB()
    {
        if (m_vertices.empty())
        {
            m_localAABB = Math::AABBCreate(Math::Vec3Zero(), Math::Vec3Zero());
            return;
        }

        m_localAABB = Math::AABBCreate(m_vertices[0].position, m_vertices[0].position);

        for (size_t i = 1; i < m_vertices.size(); i++)
        {
            m_localAABB = Math::AABBExpand(m_localAABB, m_vertices[i].position);
        }
    }

    AABB MeshCollider::ComputeAABB(const Vector3& position, const Quaternion& rotation) const
    {
        Quaternion worldRotation = Math::QuatNormalize(rotation * m_rotationOffset);
        Vector3 worldPosition = position + rotation * m_offset;

        return Math::AABBTransform(m_localAABB, worldPosition, worldRotation);
    }

    Vector3 MeshCollider::GetSupportPoint(const Vector3& direction, const Vector3& position, const Quaternion& rotation) const
    {
        if (m_vertices.empty())
        {
            return position + rotation * m_offset;
        }

        Quaternion worldRotation = Math::QuatNormalize(rotation * m_rotationOffset);
        Vector3 worldPosition = position + rotation * m_offset;

        Vector3 localDir = Math::QuatInverse(worldRotation) * direction;

        float maxDot = Math::Dot(m_vertices[0].position, localDir);
        uint32_t bestIdx = 0;

        for (uint32_t i = 1; i < (uint32_t)m_vertices.size(); i++)
        {
            float dot = Math::Dot(m_vertices[i].position, localDir);
            if (dot > maxDot)
            {
                maxDot = dot;
                bestIdx = i;
            }
        }

        return worldPosition + worldRotation * m_vertices[bestIdx].position;
    }

    Vector3 MeshCollider::GetTriangleVertex(uint32_t triangleIndex, uint32_t vertexIndex, const Vector3& position, const Quaternion& rotation) const
    {
        if (triangleIndex >= (uint32_t)m_triangles.size() || vertexIndex >= 3)
        {
            return position + rotation * m_offset;
        }

        const Triangle& tri = m_triangles[triangleIndex];
        uint32_t vertexIdx = tri.indices[vertexIndex];

        if (vertexIdx >= (uint32_t)m_vertices.size())
        {
            return position + rotation * m_offset;
        }

        Quaternion worldRotation = Math::QuatNormalize(rotation * m_rotationOffset);
        Vector3 worldPosition = position + rotation * m_offset;

        return worldPosition + worldRotation * m_vertices[vertexIdx].position;
    }

    ColliderManager::ColliderManager()
        : m_nextColliderId(1)
    {
    }

    ColliderManager::~ColliderManager()
    {
        for (auto& pair : m_colliders)
        {
            delete pair.second;
        }
        m_colliders.clear();
        m_bodyToColliders.clear();
    }

    uint32_t ColliderManager::CreateBoxCollider(uint32_t bodyId, const Vector3& halfExtents, const Vector3& offset, const Quaternion& rotationOffset)
    {
        BoxCollider* collider = new BoxCollider(bodyId, halfExtents, offset, rotationOffset);
        collider->SetId(m_nextColliderId);
        m_colliders[m_nextColliderId] = collider;
        m_bodyToColliders[bodyId].push_back(m_nextColliderId);
        return m_nextColliderId++;
    }

    uint32_t ColliderManager::CreateSphereCollider(uint32_t bodyId, float radius, const Vector3& offset)
    {
        SphereCollider* collider = new SphereCollider(bodyId, radius, offset);
        collider->SetId(m_nextColliderId);
        m_colliders[m_nextColliderId] = collider;
        m_bodyToColliders[bodyId].push_back(m_nextColliderId);
        return m_nextColliderId++;
    }

    uint32_t ColliderManager::CreateHullCollider(uint32_t bodyId, const Vector3* points, uint32_t pointCount, const Vector3& offset, const Quaternion& rotationOffset)
    {
        std::vector<Vector3> pointList;
        pointList.reserve(pointCount);
        for (uint32_t i = 0; i < pointCount; i++)
        {
            pointList.push_back(points[i]);
        }

        HullCollider* collider = new HullCollider(bodyId, pointList, offset, rotationOffset);
        collider->SetId(m_nextColliderId);
        m_colliders[m_nextColliderId] = collider;
        m_bodyToColliders[bodyId].push_back(m_nextColliderId);
        return m_nextColliderId++;
    }

    uint32_t ColliderManager::CreateMeshCollider(uint32_t bodyId, const Vertex* vertices, uint32_t vertexCount, const Triangle* triangles, uint32_t triangleCount, const Vector3& offset, const Quaternion& rotationOffset)
    {
        std::vector<Vertex> vertexList;
        vertexList.reserve(vertexCount);
        for (uint32_t i = 0; i < vertexCount; i++)
        {
            vertexList.push_back(vertices[i]);
        }

        std::vector<Triangle> triangleList;
        triangleList.reserve(triangleCount);
        for (uint32_t i = 0; i < triangleCount; i++)
        {
            triangleList.push_back(triangles[i]);
        }

        MeshCollider* collider = new MeshCollider(bodyId, vertexList, triangleList, offset, rotationOffset);
        collider->SetId(m_nextColliderId);
        m_colliders[m_nextColliderId] = collider;
        m_bodyToColliders[bodyId].push_back(m_nextColliderId);
        return m_nextColliderId++;
    }

    void ColliderManager::DestroyCollider(uint32_t colliderId)
    {
        auto it = m_colliders.find(colliderId);
        if (it != m_colliders.end())
        {
            uint32_t bodyId = it->second->GetBodyId();
            delete it->second;
            m_colliders.erase(it);

            auto bodyIt = m_bodyToColliders.find(bodyId);
            if (bodyIt != m_bodyToColliders.end())
            {
                auto& colliderList = bodyIt->second;
                colliderList.erase(std::remove(colliderList.begin(), colliderList.end(), colliderId), colliderList.end());
                if (colliderList.empty())
                {
                    m_bodyToColliders.erase(bodyIt);
                }
            }
        }
    }

    Collider* ColliderManager::GetCollider(uint32_t colliderId)
    {
        auto it = m_colliders.find(colliderId);
        if (it != m_colliders.end())
        {
            return it->second;
        }
        return nullptr;
    }

    const Collider* ColliderManager::GetCollider(uint32_t colliderId) const
    {
        auto it = m_colliders.find(colliderId);
        if (it != m_colliders.end())
        {
            return it->second;
        }
        return nullptr;
    }

    std::vector<Collider*> ColliderManager::GetCollidersForBody(uint32_t bodyId) const
    {
        std::vector<Collider*> result;

        auto it = m_bodyToColliders.find(bodyId);
        if (it != m_bodyToColliders.end())
        {
            for (uint32_t colliderId : it->second)
            {
                auto colliderIt = m_colliders.find(colliderId);
                if (colliderIt != m_colliders.end())
                {
                    result.push_back(colliderIt->second);
                }
            }
        }

        return result;
    }
}
