#pragma once

#include "IcePhysicsAPI.h"
#include "MathUtils.h"
#include <vector>
#include <unordered_map>

namespace IcePhysics
{
    struct VoronoiSite
    {
        Vector2 position;
        uint32_t id;
        float weight;
    };

    struct VoronoiEdge
    {
        Vector2 start;
        Vector2 end;
        uint32_t siteA;
        uint32_t siteB;
    };

    struct VoronoiCell
    {
        uint32_t siteId;
        Vector2 sitePosition;
        std::vector<Vector2> vertices;
        std::vector<uint32_t> edges;
        float area;
        AABB bounds;
    };

    struct VoronoiDiagram
    {
        std::vector<VoronoiSite> sites;
        std::vector<VoronoiEdge> edges;
        std::vector<VoronoiCell> cells;
        AABB bounds;
    };

    class VoronoiGenerator
    {
    public:
        VoronoiGenerator();
        ~VoronoiGenerator();

        void Generate(const std::vector<Vector2>& points, const AABB& bounds, VoronoiDiagram& outDiagram);
        void GenerateWeighted(const std::vector<VoronoiSite>& sites, const AABB& bounds, VoronoiDiagram& outDiagram);

        void SetRelaxationIterations(uint32_t iterations) { m_relaxationIterations = iterations; }
        uint32_t GetRelaxationIterations() const { return m_relaxationIterations; }

        void RelaxDiagram(VoronoiDiagram& diagram, uint32_t iterations);

    private:
        uint32_t m_relaxationIterations;
        float m_epsilon;

        struct FortuneEvent
        {
            enum class Type : uint32_t
            {
                Site = 0,
                Circle = 1
            };

            Type type;
            float y;
            uint32_t siteIndex;
            Vector2 point;
            uint32_t arcIndex;
        };

        struct FortuneArc
        {
            uint32_t siteIndex;
            uint32_t prevArc;
            uint32_t nextArc;
            float s0;
            float s1;
            uint32_t edgeIndex;
            uint32_t circleEventIndex;
            bool isActive;
        };

        std::vector<FortuneEvent> m_eventQueue;
        std::vector<FortuneArc> m_arcs;
        std::vector<Vector2> m_vertices;
        std::vector<uint32_t> m_edgeHalfEdges;

        void FortuneAlgorithm(const std::vector<VoronoiSite>& sites, const AABB& bounds, VoronoiDiagram& outDiagram);

        void AddSiteEvent(uint32_t siteIndex, const Vector2& position);
        void AddCircleEvent(uint32_t arcIndex, const Vector2& center, float y);
        bool ProcessNextEvent(const std::vector<VoronoiSite>& sites, const AABB& bounds, VoronoiDiagram& outDiagram);

        uint32_t CreateArc(uint32_t siteIndex);
        void InsertArc(uint32_t newArcIndex, uint32_t existingArcIndex);
        void RemoveArc(uint32_t arcIndex, const std::vector<VoronoiSite>& sites, VoronoiDiagram& outDiagram);

        uint32_t FindArcAbove(float x, float y, const std::vector<VoronoiSite>& sites);
        Vector2 GetIntersection(uint32_t arcIndex1, uint32_t arcIndex2, float y, const std::vector<VoronoiSite>& sites);
        bool CheckCircleEvent(uint32_t arcIndex, const std::vector<VoronoiSite>& sites);

        void CreateEdge(uint32_t siteA, uint32_t siteB, const Vector2& start, VoronoiDiagram& outDiagram);
        void EndEdge(uint32_t edgeIndex, const Vector2& end, VoronoiDiagram& outDiagram);

        void ClipDiagramToBounds(VoronoiDiagram& diagram, const AABB& bounds);
        bool ClipEdgeToBounds(Vector2& start, Vector2& end, const AABB& bounds);
        bool LineIntersection(const Vector2& p1, const Vector2& p2, const Vector2& p3, const Vector2& p4, Vector2& intersection);

        void BuildCells(VoronoiDiagram& diagram);
        float CalculateCellArea(const std::vector<Vector2>& vertices);
        Vector2 CalculateCellCentroid(const std::vector<Vector2>& vertices, float area);

        uint32_t SortEvents();
        void HeapifyUp(uint32_t index);
        void HeapifyDown(uint32_t index);
    };

    struct IceFragment
    {
        uint32_t id;
        Vector3 position;
        Quaternion rotation;
        Vector3 velocity;
        Vector3 angularVelocity;
        Vector3 centerOfMass;
        float mass;
        Matrix3x3 inertiaTensor;
        std::vector<Vertex> vertices;
        std::vector<Triangle> triangles;
        AABB bounds;
        bool isActive;
        float lifetime;
        uint32_t sourceSheetId;
        uint32_t collisionGroup;
        float collisionCooldown;
        uint32_t bodyId;
        uint32_t colliderId;
        bool isStatic;
    };

    struct IceImpactData
    {
        Vector3 impactPoint;
        Vector3 impactNormal;
        float impactVelocity;
        float impactForce;
        float pressure;
        float contactArea;
        float impactTime;
    };

    class IceSheet
    {
    public:
        IceSheet(const IceSheetDesc& desc);
        ~IceSheet();

        uint32_t GetId() const { return m_id; }
        void SetId(uint32_t id) { m_id = id; }

        void GenerateMesh();
        bool GetMesh(uint32_t& vertexCount, Vertex* vertices, uint32_t& triangleCount, Triangle* triangles) const;

        bool CheckImpact(const IceImpactData& impact, float yieldStrength);
        bool BreakAtPoint(const IceImpactData& impact, float yieldStrength,
                          uint32_t& fragmentCount, IceFragmentDesc*& fragments,
                          std::vector<IceFragment>& outFragments);

        void Update(float dt, float waterDensity, const Vector3& gravity);

        uint32_t GetFragmentCount() const { return (uint32_t)m_fragments.size(); }
        bool GetFragmentData(uint32_t fragmentIndex, IceFragmentDesc& desc,
                             uint32_t& vertexCount, Vertex* vertices,
                             uint32_t& triangleCount, Triangle* triangles) const;

        const IceFragment* GetFragment(uint32_t index) const;
        IceFragment* GetFragment(uint32_t index);

        Vector3 GetPosition() const { return m_position; }
        Vector2 GetSize() const { return m_size; }
        float GetThickness() const { return m_thickness; }
        float GetYieldStrength() const { return m_yieldStrength; }
        float GetYoungsModulus() const { return m_youngsModulus; }
        float GetPoissonsRatio() const { return m_poissonsRatio; }
        float GetDensity() const { return m_density; }

        void CalculatePressureDistribution(const Vector3& shipPosition, const Quaternion& shipRotation,
                                            uint32_t& pointCount, PressurePoint* pressurePoints);

        uint32_t GetBodyId() const { return m_rigidBodyId; }
        void SetBodyId(uint32_t id) { m_rigidBodyId = id; }

        uint32_t GetColliderId() const { return m_colliderId; }
        void SetColliderId(uint32_t id) { m_colliderId = id; }

    private:
        uint32_t m_id;
        Vector3 m_position;
        Vector2 m_size;
        float m_thickness;
        float m_yieldStrength;
        float m_youngsModulus;
        float m_poissonsRatio;
        float m_density;
        uint32_t m_subdivisions;

        uint32_t m_rigidBodyId;
        uint32_t m_colliderId;

        std::vector<Vertex> m_vertices;
        std::vector<Triangle> m_triangles;
        std::vector<Vector2> m_gridPoints;

        std::vector<IceFragment> m_fragments;
        VoronoiGenerator m_voronoiGenerator;

        uint32_t m_nextFragmentId;

        void GenerateGridPoints();
        std::vector<Vector2> GenerateVoronoiSites(const Vector2& impactPoint, float impactRadius, uint32_t siteCount);
        void GenerateFragmentMesh(const VoronoiCell& cell, const Vector2& origin, IceFragment& fragment);

        void ExtrudeFragment(IceFragment& fragment, float height);
        void CalculateFragmentMassProperties(IceFragment& fragment);
        void CalculateFragmentInertia(IceFragment& fragment);

        bool CheckStressFailure(float stress, float yieldStrength);
        float CalculateBendingStress(float impactForce, float distance, float thickness);
        float CalculateShearStress(float impactForce, float contactArea);

        void InitializeFragmentPhysics(IceFragment& fragment, const IceImpactData& impact);
        void ApplyBuoyancy(IceFragment& fragment, float waterDensity, const Vector3& gravity);
    };

    class IceSheetManager
    {
    public:
        IceSheetManager();
        ~IceSheetManager();

        uint32_t CreateIceSheet(const IceSheetDesc& desc);
        void DestroyIceSheet(uint32_t iceSheetId);
        IceSheet* GetIceSheet(uint32_t iceSheetId);
        const IceSheet* GetIceSheet(uint32_t iceSheetId) const;

        void Update(float dt, float waterDensity, const Vector3& gravity);

        uint32_t GetActiveCount() const { return (uint32_t)m_iceSheets.size(); }
        const std::unordered_map<uint32_t, IceSheet*>& GetIceSheets() const { return m_iceSheets; }

        void SetIceBreakCallback(IceBreakCallback callback) { m_iceBreakCallback = callback; }

        bool HandleIceImpact(uint32_t iceSheetId, const IceImpactData& impact);

    private:
        std::unordered_map<uint32_t, IceSheet*> m_iceSheets;
        uint32_t m_nextIceSheetId;
        IceBreakCallback m_iceBreakCallback;
    };
}
