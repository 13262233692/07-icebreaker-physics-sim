#include "../include/PhysicsWorld.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <chrono>

namespace IcePhysics
{
    PhysicsWorld* g_physicsWorld = nullptr;

    DebugLogger::DebugLogger()
        : m_level(DebugLevel::Info)
        , m_callback(nullptr)
        , m_startTime(0)
    {
        m_startTime = GetTimestamp();
    }

    DebugLogger::~DebugLogger()
    {
    }

    uint64_t DebugLogger::GetTimestamp() const
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    }

    void DebugLogger::Log(DebugLevel level, const char* subsystem, const char* format, ...)
    {
        if (level > m_level || !format) return;

        char message[512];
        va_list args;
        va_start(args, format);
        vsnprintf(message, sizeof(message), format, args);
        va_end(args);

        if (m_callback)
        {
            DebugLogEntry entry;
            entry.level = level;
            entry.timestamp = GetTimestamp();
            strncpy_s(entry.message, message, _TRUNCATE);
            strncpy_s(entry.subsystem, subsystem ? subsystem : "Unknown", _TRUNCATE);
            m_callback(&entry);
        }
    }

    void DebugLogger::LogError(const char* subsystem, const char* message)
    {
        Log(DebugLevel::Error, subsystem, "%s", message);
        SetLastError(message);
    }

    void DebugLogger::LogWarning(const char* subsystem, const char* message)
    {
        Log(DebugLevel::Warning, subsystem, "%s", message);
    }

    void DebugLogger::LogInfo(const char* subsystem, const char* message)
    {
        Log(DebugLevel::Info, subsystem, "%s", message);
    }

    void DebugLogger::LogVerbose(const char* subsystem, const char* message)
    {
        Log(DebugLevel::Verbose, subsystem, "%s", message);
    }

    void DebugLogger::LogDebug(const char* subsystem, const char* message)
    {
        Log(DebugLevel::Debug, subsystem, "%s", message);
    }

    PhysicsWorld::PhysicsWorld()
        : m_gravity(0.0f, -9.81f, 0.0f)
        , m_fixedTimestep(1.0f / 60.0f)
        , m_accumulator(0.0f)
        , m_simulationTime(0.0f)
        , m_frameCount(0)
        , m_waterDensity(1025.0f)
        , m_waterTemperature(15.0f)
        , m_iceConcentration(0.5f)
    {
        memset(&m_stats, 0, sizeof(m_stats));
    }

    PhysicsWorld::~PhysicsWorld()
    {
        Shutdown();
    }

    bool PhysicsWorld::Initialize(float fixedTimestep, DebugLevel debugLevel)
    {
        m_debugLogger.SetLevel(debugLevel);
        m_debugLogger.LogInfo("PhysicsWorld", "Initializing physics world...");

        m_fixedTimestep = fixedTimestep;
        m_accumulator = 0.0f;
        m_simulationTime = 0.0f;
        m_frameCount = 0;

        m_collisionWorld.SetGravity(m_gravity);

        ResetStats();

        m_debugLogger.LogInfo("PhysicsWorld", "Physics world initialized successfully");
        return true;
    }

    void PhysicsWorld::Shutdown()
    {
        m_debugLogger.LogInfo("PhysicsWorld", "Shutting down physics world...");

        m_shipStates.clear();

        m_debugLogger.LogInfo("PhysicsWorld", "Physics world shut down complete");
    }

    void PhysicsWorld::Simulate(float deltaTime)
    {
        if (deltaTime <= 0.0f) return;

        const float maxDeltaTime = 0.25f;
        if (deltaTime > maxDeltaTime)
        {
            deltaTime = maxDeltaTime;
            m_debugLogger.LogWarning("PhysicsWorld", "Delta time clamped to 0.25s");
        }

        m_accumulator += deltaTime;

        while (m_accumulator >= m_fixedTimestep)
        {
            Step(m_fixedTimestep);
            m_accumulator -= m_fixedTimestep;
            m_simulationTime += m_fixedTimestep;
        }

        m_frameCount++;

        m_stats.simulationTime = m_simulationTime;
        m_stats.frameCount = m_frameCount;
        m_stats.rigidBodyCount = m_rigidBodyManager.GetActiveCount();
        m_stats.colliderCount = m_colliderManager.GetCount();
        m_stats.iceSheetCount = m_iceSheetManager.GetActiveCount();
        m_stats.activeCollisions = m_collisionWorld.GetActiveCollisionCount();
    }

    void PhysicsWorld::Step(float dt)
    {
        auto stepStart = std::chrono::high_resolution_clock::now();

        auto hydroStart = std::chrono::high_resolution_clock::now();
        for (auto& pair : m_shipStates)
        {
            uint32_t bodyId = pair.first;
            ShipState& state = pair.second;

            RigidBody* body = m_rigidBodyManager.GetBody(bodyId);
            if (!body) continue;

            state.position = body->GetPosition();
            state.rotation = body->GetRotation();
            state.linearVelocity = body->GetLinearVelocity();
            state.angularVelocity = body->GetAngularVelocity();
            state.speedKnots = Math::Length(state.linearVelocity) * 1.94384f;
            Vector3 euler = Math::QuatToEuler(state.rotation);
            state.headingDegrees = euler.y;
            state.driftAngle = atan2f(state.linearVelocity.x, state.linearVelocity.z) * Math::RAD_TO_DEG - state.headingDegrees;

            Vector3 totalForce, totalTorque;
            CalculateHydrodynamicForces(bodyId, &totalForce, &totalTorque);

            body->ApplyForce(totalForce, state.position);
            body->ApplyTorque(totalTorque);
        }
        m_iceSheetManager.Update(dt, m_waterDensity, m_gravity);
        auto hydroEnd = std::chrono::high_resolution_clock::now();
        float hydroMs = std::chrono::duration<float, std::milli>(hydroEnd - hydroStart).count();

        auto broadStart = std::chrono::high_resolution_clock::now();
        m_collisionWorld.Step(dt, m_rigidBodyManager, m_colliderManager);
        auto broadEnd = std::chrono::high_resolution_clock::now();
        float totalCollisionMs = std::chrono::duration<float, std::milli>(broadEnd - broadStart).count();

        float broadPhaseMs = totalCollisionMs * 0.3f;
        float narrowPhaseMs = totalCollisionMs * 0.5f;
        float solverMs = totalCollisionMs * 0.2f;

        CheckIceCollisions();
        ProcessIceImpacts();

        m_rigidBodyManager.Update(dt);

        auto stepEnd = std::chrono::high_resolution_clock::now();
        float totalMs = std::chrono::duration<float, std::milli>(stepEnd - stepStart).count();

        UpdateStats(broadPhaseMs, narrowPhaseMs, solverMs, hydroMs, totalMs);

        m_collisionWorld.DispatchCollisionEvents();
    }

    uint32_t PhysicsWorld::CreateRigidBody(const RigidBodyDesc* desc)
    {
        if (!desc)
        {
            m_debugLogger.LogError("PhysicsWorld", "CreateRigidBody: Invalid descriptor");
            return 0;
        }
        return m_rigidBodyManager.CreateBody(*desc);
    }

    void PhysicsWorld::DestroyRigidBody(uint32_t bodyId)
    {
        m_shipStates.erase(bodyId);
        m_rigidBodyManager.DestroyBody(bodyId);
    }

    uint32_t PhysicsWorld::CreateBoxCollider(uint32_t bodyId, const Vector3* halfExtents, const Vector3* offset, const Quaternion* rotationOffset)
    {
        if (!halfExtents || !offset || !rotationOffset)
        {
            m_debugLogger.LogError("PhysicsWorld", "CreateBoxCollider: Invalid parameters");
            return 0;
        }
        return m_colliderManager.CreateBoxCollider(bodyId, *halfExtents, *offset, *rotationOffset);
    }

    uint32_t PhysicsWorld::CreateMeshCollider(uint32_t bodyId, const Vertex* vertices, uint32_t vertexCount, const Triangle* triangles, uint32_t triangleCount, const Vector3* offset, const Quaternion* rotationOffset)
    {
        if (!vertices || !triangles || !offset || !rotationOffset)
        {
            m_debugLogger.LogError("PhysicsWorld", "CreateMeshCollider: Invalid parameters");
            return 0;
        }
        return m_colliderManager.CreateMeshCollider(bodyId, vertices, vertexCount, triangles, triangleCount, *offset, *rotationOffset);
    }

    uint32_t PhysicsWorld::CreateHullCollider(uint32_t bodyId, const Vector3* points, uint32_t pointCount, const Vector3* offset, const Quaternion* rotationOffset)
    {
        if (!points || !offset || !rotationOffset)
        {
            m_debugLogger.LogError("PhysicsWorld", "CreateHullCollider: Invalid parameters");
            return 0;
        }
        return m_colliderManager.CreateHullCollider(bodyId, points, pointCount, *offset, *rotationOffset);
    }

    uint32_t PhysicsWorld::CreateIceSheet(const IceSheetDesc* desc)
    {
        if (!desc)
        {
            m_debugLogger.LogError("PhysicsWorld", "CreateIceSheet: Invalid descriptor");
            return 0;
        }

        uint32_t iceSheetId = m_iceSheetManager.CreateIceSheet(*desc);

        IceSheet* iceSheet = m_iceSheetManager.GetIceSheet(iceSheetId);
        if (iceSheet)
        {
            RigidBodyDesc bodyDesc = {};
            bodyDesc.position = desc->position;
            bodyDesc.rotation = Math::QuatIdentity();
            bodyDesc.mass = desc->density * desc->size.x * desc->size.y * desc->thickness;
            bodyDesc.inertiaTensor = Math::Mat3Identity();
            bodyDesc.restitution = 0.1f;
            bodyDesc.staticFriction = 0.5f;
            bodyDesc.dynamicFriction = 0.3f;
            bodyDesc.linearDamping = 0.1f;
            bodyDesc.angularDamping = 0.1f;
            bodyDesc.isKinematic = true;

            uint32_t bodyId = m_rigidBodyManager.CreateBody(bodyDesc);
            iceSheet->SetBodyId(bodyId);

            uint32_t vertexCount = 0;
            uint32_t triangleCount = 0;
            iceSheet->GetMesh(vertexCount, nullptr, triangleCount, nullptr);

            std::vector<Vertex> vertices(vertexCount);
            std::vector<Triangle> triangles(triangleCount);
            iceSheet->GetMesh(vertexCount, vertices.data(), triangleCount, triangles.data());

            Vector3 offset(0, 0, 0);
            Quaternion rotationOffset = Math::QuatIdentity();
            uint32_t colliderId = m_colliderManager.CreateMeshCollider(
                bodyId, vertices.data(), vertexCount, triangles.data(), triangleCount, offset, rotationOffset);
            iceSheet->SetColliderId(colliderId);
        }

        return iceSheetId;
    }

    void PhysicsWorld::DestroyIceSheet(uint32_t iceSheetId)
    {
        IceSheet* iceSheet = m_iceSheetManager.GetIceSheet(iceSheetId);
        if (iceSheet)
        {
            uint32_t colliderId = iceSheet->GetColliderId();
            uint32_t bodyId = iceSheet->GetBodyId();

            m_colliderManager.DestroyCollider(colliderId);
            m_rigidBodyManager.DestroyBody(bodyId);
        }

        m_iceSheetManager.DestroyIceSheet(iceSheetId);
    }

    void PhysicsWorld::UpdateShipState(uint32_t bodyId, const ShipState* state)
    {
        if (!state) return;

        RigidBody* body = m_rigidBodyManager.GetBody(bodyId);
        if (!body)
        {
            m_debugLogger.LogWarning("PhysicsWorld", "UpdateShipState: Body not found");
            return;
        }

        m_shipStates[bodyId] = *state;

        body->SetPosition(state->position);
        body->SetRotation(state->rotation);
        body->SetLinearVelocity(state->linearVelocity);
        body->SetAngularVelocity(state->angularVelocity);
    }

    void PhysicsWorld::CalculateHydrodynamicForces(uint32_t bodyId, Vector3* totalForce, Vector3* totalTorque)
    {
        if (!totalForce || !totalTorque) return;

        *totalForce = Math::Vec3Zero();
        *totalTorque = Math::Vec3Zero();

        auto it = m_shipStates.find(bodyId);
        if (it == m_shipStates.end())
        {
            return;
        }

        ShipState& state = it->second;
        m_hydrodynamicSystem.CalculateForces(state, *totalForce, *totalTorque);
    }

    float PhysicsWorld::CalculateShipResistance(float speedKnots, float waterTemperature, float iceConcentration)
    {
        return m_hydrodynamicSystem.CalculateTotalResistance(speedKnots, waterTemperature, iceConcentration);
    }

    bool PhysicsWorld::QueryIcePressureDistribution(uint32_t iceSheetId, const Vector3* shipPosition, const Quaternion* shipRotation, uint32_t* pointCount, PressurePoint* pressurePoints)
    {
        if (!shipPosition || !shipRotation || !pointCount) return false;

        IceSheet* iceSheet = m_iceSheetManager.GetIceSheet(iceSheetId);
        if (!iceSheet) return false;

        iceSheet->CalculatePressureDistribution(*shipPosition, *shipRotation, *pointCount, pressurePoints);
        return true;
    }

    void PhysicsWorld::ResetStats()
    {
        memset(&m_stats, 0, sizeof(m_stats));
    }

    void PhysicsWorld::CheckIceCollisions()
    {
        m_debugLogger.LogVerbose("PhysicsWorld", "Checking ice collisions...");
    }

    void PhysicsWorld::ProcessIceImpacts()
    {
        const auto& collisions = m_collisionWorld.GetActiveCollisions();

        for (const auto& collision : collisions)
        {
            if (collision.manifolds.empty()) continue;

            IceSheet* iceSheetA = nullptr;
            IceSheet* iceSheetB = nullptr;

            for (const auto& pair : m_iceSheetManager.GetIceSheets())
            {
                if (pair.second->GetBodyId() == collision.bodyA)
                {
                    iceSheetA = pair.second;
                }
                if (pair.second->GetBodyId() == collision.bodyB)
                {
                    iceSheetB = pair.second;
                }
            }

            if (!iceSheetA && !iceSheetB) continue;

            IceSheet* iceSheet = iceSheetA ? iceSheetA : iceSheetB;
            uint32_t shipBodyId = iceSheetA ? collision.bodyB : collision.bodyA;

            RigidBody* shipBody = m_rigidBodyManager.GetBody(shipBodyId);
            if (!shipBody) continue;

            for (const auto& manifold : collision.manifolds)
            {
                IceImpactData impact;
                impact.impactPoint = manifold.point;
                impact.impactNormal = manifold.normal;
                impact.impactVelocity = Math::Length(shipBody->GetPointVelocity(manifold.point));
                impact.impactForce = manifold.impulse / m_fixedTimestep;
                impact.pressure = impact.impactForce / Math::Max(manifold.penetration * manifold.penetration, 0.001f);
                impact.contactArea = manifold.penetration * 10.0f;
                impact.impactTime = m_simulationTime;

                m_iceSheetManager.HandleIceImpact(iceSheet->GetId(), impact);
            }
        }

        uint32_t fragmentCount = 0;
        for (const auto& pair : m_iceSheetManager.GetIceSheets())
        {
            fragmentCount += pair.second->GetFragmentCount();
        }
        m_stats.iceFragmentCount = fragmentCount;
    }

    void PhysicsWorld::UpdateStats(float broadPhaseMs, float narrowPhaseMs, float solverMs, float hydroMs, float totalMs)
    {
        const float alpha = 0.1f;
        m_stats.broadPhaseTimeMs = m_stats.broadPhaseTimeMs * (1.0f - alpha) + broadPhaseMs * alpha;
        m_stats.narrowPhaseTimeMs = m_stats.narrowPhaseTimeMs * (1.0f - alpha) + narrowPhaseMs * alpha;
        m_stats.solverTimeMs = m_stats.solverTimeMs * (1.0f - alpha) + solverMs * alpha;
        m_stats.hydrodynamicTimeMs = m_stats.hydrodynamicTimeMs * (1.0f - alpha) + hydroMs * alpha;
        m_stats.totalStepTimeMs = m_stats.totalStepTimeMs * (1.0f - alpha) + totalMs * alpha;
    }

    PhysicsWorld* GetPhysicsWorld()
    {
        return g_physicsWorld;
    }

    void SetPhysicsWorld(PhysicsWorld* world)
    {
        g_physicsWorld = world;
    }
}
