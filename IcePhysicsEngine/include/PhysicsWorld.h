#pragma once

#include "IcePhysicsAPI.h"
#include "MathUtils.h"
#include "RigidBody.h"
#include "Collider.h"
#include "CollisionDetection.h"
#include "Hydrodynamics.h"
#include "Voronoi.h"
#include <vector>
#include <unordered_map>
#include <string>

namespace IcePhysics
{
    class DebugLogger
    {
    public:
        DebugLogger();
        ~DebugLogger();

        void SetLevel(DebugLevel level) { m_level = level; }
        DebugLevel GetLevel() const { return m_level; }

        void SetCallback(DebugLogCallback callback) { m_callback = callback; }

        void Log(DebugLevel level, const char* subsystem, const char* format, ...);
        void LogError(const char* subsystem, const char* message);
        void LogWarning(const char* subsystem, const char* message);
        void LogInfo(const char* subsystem, const char* message);
        void LogVerbose(const char* subsystem, const char* message);
        void LogDebug(const char* subsystem, const char* message);

        const char* GetLastError() const { return m_lastError.c_str(); }
        void ClearLastError() { m_lastError.clear(); }
        void SetLastError(const char* error) { m_lastError = error; }

        uint64_t GetTimestamp() const;

    private:
        DebugLevel m_level;
        DebugLogCallback m_callback;
        std::string m_lastError;
        uint64_t m_startTime;
    };

    struct PhysicsWorldStats
    {
        float simulationTime;
        uint64_t frameCount;
        uint32_t rigidBodyCount;
        uint32_t colliderCount;
        uint32_t iceSheetCount;
        uint32_t activeCollisions;
        uint32_t iceFragmentCount;
        float broadPhaseTimeMs;
        float narrowPhaseTimeMs;
        float solverTimeMs;
        float hydrodynamicTimeMs;
        float totalStepTimeMs;
    };

    class PhysicsWorld
    {
    public:
        PhysicsWorld();
        ~PhysicsWorld();

        bool Initialize(float fixedTimestep, DebugLevel debugLevel);
        void Shutdown();

        void Simulate(float deltaTime);

        RigidBodyManager& GetRigidBodyManager() { return m_rigidBodyManager; }
        ColliderManager& GetColliderManager() { return m_colliderManager; }
        CollisionWorld& GetCollisionWorld() { return m_collisionWorld; }
        HydrodynamicSystem& GetHydrodynamicSystem() { return m_hydrodynamicSystem; }
        InertiaMomentumSolver& GetInertiaSolver() { return m_inertiaSolver; }
        IceSheetManager& GetIceSheetManager() { return m_iceSheetManager; }
        DebugLogger& GetDebugLogger() { return m_debugLogger; }

        void SetGravity(const Vector3& gravity) { m_gravity = gravity; m_collisionWorld.SetGravity(gravity); }
        Vector3 GetGravity() const { return m_gravity; }

        void SetFixedTimestep(float timestep) { m_fixedTimestep = timestep; }
        float GetFixedTimestep() const { return m_fixedTimestep; }

        void SetWaterDensity(float density) { m_waterDensity = density; }
        float GetWaterDensity() const { return m_waterDensity; }

        void SetWaterTemperature(float temperature) { m_waterTemperature = temperature; }
        float GetWaterTemperature() const { return m_waterTemperature; }

        void SetIceConcentration(float concentration) { m_iceConcentration = Math::Clamp(concentration, 0.0f, 1.0f); }
        float GetIceConcentration() const { return m_iceConcentration; }

        uint32_t CreateRigidBody(const RigidBodyDesc* desc);
        void DestroyRigidBody(uint32_t bodyId);

        uint32_t CreateBoxCollider(uint32_t bodyId, const Vector3* halfExtents, const Vector3* offset, const Quaternion* rotationOffset);
        uint32_t CreateMeshCollider(uint32_t bodyId, const Vertex* vertices, uint32_t vertexCount, const Triangle* triangles, uint32_t triangleCount, const Vector3* offset, const Quaternion* rotationOffset);
        uint32_t CreateHullCollider(uint32_t bodyId, const Vector3* points, uint32_t pointCount, const Vector3* offset, const Quaternion* rotationOffset);

        uint32_t CreateIceSheet(const IceSheetDesc* desc);
        void DestroyIceSheet(uint32_t iceSheetId);

        void SetDebugLogCallback(DebugLogCallback callback) { m_debugLogger.SetCallback(callback); }
        void SetCollisionEventCallback(CollisionEventCallback callback) { m_collisionWorld.SetCollisionEventCallback(callback); }
        void SetIceBreakCallback(IceBreakCallback callback) { m_iceSheetManager.SetIceBreakCallback(callback); }

        void UpdateShipState(uint32_t bodyId, const ShipState* state);
        void CalculateHydrodynamicForces(uint32_t bodyId, Vector3* totalForce, Vector3* totalTorque);
        float CalculateShipResistance(float speedKnots, float waterTemperature, float iceConcentration);

        bool QueryIcePressureDistribution(uint32_t iceSheetId, const Vector3* shipPosition, const Quaternion* shipRotation, uint32_t* pointCount, PressurePoint* pressurePoints);

        const PhysicsWorldStats& GetStats() const { return m_stats; }
        void ResetStats();

        float GetSimulationTime() const { return m_simulationTime; }
        uint64_t GetFrameCount() const { return m_frameCount; }

        uint32_t GetActiveRigidBodyCount() const { return m_rigidBodyManager.GetActiveCount(); }
        uint32_t GetActiveIceSheetCount() const { return m_iceSheetManager.GetActiveCount(); }
        uint32_t GetActiveCollisionCount() const { return m_collisionWorld.GetActiveCollisionCount(); }

        void CheckIceCollisions();
        void ProcessIceImpacts();

    private:
        RigidBodyManager m_rigidBodyManager;
        ColliderManager m_colliderManager;
        CollisionWorld m_collisionWorld;
        HydrodynamicSystem m_hydrodynamicSystem;
        InertiaMomentumSolver m_inertiaSolver;
        IceSheetManager m_iceSheetManager;
        DebugLogger m_debugLogger;

        Vector3 m_gravity;
        float m_fixedTimestep;
        float m_accumulator;
        float m_simulationTime;
        uint64_t m_frameCount;

        float m_waterDensity;
        float m_waterTemperature;
        float m_iceConcentration;

        PhysicsWorldStats m_stats;

        std::unordered_map<uint32_t, ShipState> m_shipStates;

        void Step(float dt);
        void UpdateStats(float broadPhaseMs, float narrowPhaseMs, float solverMs, float hydroMs, float totalMs);
    };

    extern PhysicsWorld* g_physicsWorld;

    PhysicsWorld* GetPhysicsWorld();
    void SetPhysicsWorld(PhysicsWorld* world);
}
