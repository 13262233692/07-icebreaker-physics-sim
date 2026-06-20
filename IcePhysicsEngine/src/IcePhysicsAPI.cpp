#include "../include/IcePhysicsAPI.h"
#include "../include/PhysicsWorld.h"

namespace IcePhysics
{
    const char* ERROR_NOT_INITIALIZED = "Physics world not initialized";
    const char* ERROR_INVALID_BODY_ID = "Invalid rigid body ID";
    const char* ERROR_INVALID_COLLIDER_ID = "Invalid collider ID";
    const char* ERROR_INVALID_ICE_SHEET_ID = "Invalid ice sheet ID";
    const char* ERROR_NULL_POINTER = "Null pointer provided";
    const char* ERROR_INVALID_PARAMS = "Invalid parameters";

    static void SetError(const char* message)
    {
        PhysicsWorld* world = GetPhysicsWorld();
        if (world)
        {
            world->GetDebugLogger().SetLastError(message);
        }
    }

    extern "C"
    {
        ICEPHYSICS_API bool IP_Initialize(float fixedTimestep, DebugLevel debugLevel)
        {
            if (fixedTimestep <= 0.0f)
            {
                SetError(ERROR_INVALID_PARAMS);
                return false;
            }

            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return false;
            }

            bool result = world->Initialize(fixedTimestep, debugLevel);
            if (!result)
            {
                SetError("Failed to initialize physics world");
            }
            return result;
        }

        ICEPHYSICS_API void IP_Shutdown()
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (world)
            {
                world->Shutdown();
            }
        }

        ICEPHYSICS_API void IP_SetDebugLogCallback(DebugLogCallback callback)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            world->SetDebugLogCallback(callback);
        }

        ICEPHYSICS_API void IP_SetCollisionEventCallback(CollisionEventCallback callback)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            world->SetCollisionEventCallback(callback);
        }

        ICEPHYSICS_API void IP_SetIceBreakCallback(IceBreakCallback callback)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            world->SetIceBreakCallback(callback);
        }

        ICEPHYSICS_API void IP_Simulate(float deltaTime)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (deltaTime <= 0.0f)
            {
                return;
            }

            world->Simulate(deltaTime);
        }

        ICEPHYSICS_API uint32_t IP_CreateRigidBody(const RigidBodyDesc* desc)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0;
            }

            if (!desc)
            {
                SetError(ERROR_NULL_POINTER);
                return 0;
            }

            return world->CreateRigidBody(desc);
        }

        ICEPHYSICS_API void IP_DestroyRigidBody(uint32_t bodyId)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            world->DestroyRigidBody(bodyId);
        }

        ICEPHYSICS_API void IP_SetRigidBodyPosition(uint32_t bodyId, const Vector3* position)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!position)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            body->SetPosition(*position);
        }

        ICEPHYSICS_API void IP_GetRigidBodyPosition(uint32_t bodyId, Vector3* position)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!position)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            *position = body->GetPosition();
        }

        ICEPHYSICS_API void IP_SetRigidBodyRotation(uint32_t bodyId, const Quaternion* rotation)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!rotation)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            body->SetRotation(*rotation);
        }

        ICEPHYSICS_API void IP_GetRigidBodyRotation(uint32_t bodyId, Quaternion* rotation)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!rotation)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            *rotation = body->GetRotation();
        }

        ICEPHYSICS_API void IP_SetRigidBodyLinearVelocity(uint32_t bodyId, const Vector3* velocity)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!velocity)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            body->SetLinearVelocity(*velocity);
        }

        ICEPHYSICS_API void IP_GetRigidBodyLinearVelocity(uint32_t bodyId, Vector3* velocity)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!velocity)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            *velocity = body->GetLinearVelocity();
        }

        ICEPHYSICS_API void IP_SetRigidBodyAngularVelocity(uint32_t bodyId, const Vector3* velocity)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!velocity)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            body->SetAngularVelocity(*velocity);
        }

        ICEPHYSICS_API void IP_GetRigidBodyAngularVelocity(uint32_t bodyId, Vector3* velocity)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!velocity)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            *velocity = body->GetAngularVelocity();
        }

        ICEPHYSICS_API void IP_ApplyForce(uint32_t bodyId, const Vector3* force, const Vector3* point)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!force || !point)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            body->ApplyForce(*force, *point);
        }

        ICEPHYSICS_API void IP_ApplyTorque(uint32_t bodyId, const Vector3* torque)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!torque)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            body->ApplyTorque(*torque);
        }

        ICEPHYSICS_API void IP_ApplyImpulse(uint32_t bodyId, const Vector3* impulse, const Vector3* point)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!impulse || !point)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            body->ApplyImpulse(*impulse, *point);
        }

        ICEPHYSICS_API uint32_t IP_CreateBoxCollider(uint32_t bodyId, const Vector3* halfExtents, const Vector3* offset, const Quaternion* rotationOffset)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return 0;
            }

            if (!halfExtents || !offset || !rotationOffset)
            {
                SetError(ERROR_NULL_POINTER);
                return 0;
            }

            return world->CreateBoxCollider(bodyId, halfExtents, offset, rotationOffset);
        }

        ICEPHYSICS_API uint32_t IP_CreateMeshCollider(uint32_t bodyId, const Vertex* vertices, uint32_t vertexCount, const Triangle* triangles, uint32_t triangleCount, const Vector3* offset, const Quaternion* rotationOffset)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return 0;
            }

            if (!vertices || !triangles || !offset || !rotationOffset)
            {
                SetError(ERROR_NULL_POINTER);
                return 0;
            }

            if (vertexCount == 0 || triangleCount == 0)
            {
                SetError(ERROR_INVALID_PARAMS);
                return 0;
            }

            return world->CreateMeshCollider(bodyId, vertices, vertexCount, triangles, triangleCount, offset, rotationOffset);
        }

        ICEPHYSICS_API uint32_t IP_CreateHullCollider(uint32_t bodyId, const Vector3* points, uint32_t pointCount, const Vector3* offset, const Quaternion* rotationOffset)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return 0;
            }

            if (!points || !offset || !rotationOffset)
            {
                SetError(ERROR_NULL_POINTER);
                return 0;
            }

            if (pointCount < 4)
            {
                SetError(ERROR_INVALID_PARAMS);
                return 0;
            }

            return world->CreateHullCollider(bodyId, points, pointCount, offset, rotationOffset);
        }

        ICEPHYSICS_API void IP_SetColliderFriction(uint32_t colliderId, float staticFriction, float dynamicFriction)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (colliderId == 0)
            {
                SetError(ERROR_INVALID_COLLIDER_ID);
                return;
            }

            Collider* collider = world->GetColliderManager().GetCollider(colliderId);
            if (!collider)
            {
                SetError(ERROR_INVALID_COLLIDER_ID);
                return;
            }

            collider->SetStaticFriction(staticFriction);
            collider->SetDynamicFriction(dynamicFriction);
        }

        ICEPHYSICS_API void IP_SetColliderRestitution(uint32_t colliderId, float restitution)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (colliderId == 0)
            {
                SetError(ERROR_INVALID_COLLIDER_ID);
                return;
            }

            Collider* collider = world->GetColliderManager().GetCollider(colliderId);
            if (!collider)
            {
                SetError(ERROR_INVALID_COLLIDER_ID);
                return;
            }

            collider->SetRestitution(restitution);
        }

        ICEPHYSICS_API uint32_t IP_CreateIceSheet(const IceSheetDesc* desc)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0;
            }

            if (!desc)
            {
                SetError(ERROR_NULL_POINTER);
                return 0;
            }

            return world->CreateIceSheet(desc);
        }

        ICEPHYSICS_API void IP_DestroyIceSheet(uint32_t iceSheetId)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (iceSheetId == 0)
            {
                SetError(ERROR_INVALID_ICE_SHEET_ID);
                return;
            }

            world->DestroyIceSheet(iceSheetId);
        }

        ICEPHYSICS_API bool IP_GetIceSheetMesh(uint32_t iceSheetId, uint32_t* vertexCount, Vertex* vertices, uint32_t* triangleCount, Triangle* triangles)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return false;
            }

            if (iceSheetId == 0)
            {
                SetError(ERROR_INVALID_ICE_SHEET_ID);
                return false;
            }

            if (!vertexCount || !triangleCount)
            {
                SetError(ERROR_NULL_POINTER);
                return false;
            }

            IceSheet* iceSheet = world->GetIceSheetManager().GetIceSheet(iceSheetId);
            if (!iceSheet)
            {
                SetError(ERROR_INVALID_ICE_SHEET_ID);
                return false;
            }

            return iceSheet->GetMesh(*vertexCount, vertices, *triangleCount, triangles);
        }

        ICEPHYSICS_API uint32_t IP_GetIceSheetFragmentCount(uint32_t iceSheetId)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0;
            }

            if (iceSheetId == 0)
            {
                SetError(ERROR_INVALID_ICE_SHEET_ID);
                return 0;
            }

            IceSheet* iceSheet = world->GetIceSheetManager().GetIceSheet(iceSheetId);
            if (!iceSheet)
            {
                SetError(ERROR_INVALID_ICE_SHEET_ID);
                return 0;
            }

            return iceSheet->GetFragmentCount();
        }

        ICEPHYSICS_API bool IP_GetIceFragmentData(uint32_t iceSheetId, uint32_t fragmentIndex, IceFragmentDesc* desc, uint32_t* vertexCount, Vertex* vertices, uint32_t* triangleCount, Triangle* triangles)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return false;
            }

            if (iceSheetId == 0)
            {
                SetError(ERROR_INVALID_ICE_SHEET_ID);
                return false;
            }

            if (!desc || !vertexCount || !triangleCount)
            {
                SetError(ERROR_NULL_POINTER);
                return false;
            }

            IceSheet* iceSheet = world->GetIceSheetManager().GetIceSheet(iceSheetId);
            if (!iceSheet)
            {
                SetError(ERROR_INVALID_ICE_SHEET_ID);
                return false;
            }

            if (fragmentIndex >= iceSheet->GetFragmentCount())
            {
                SetError(ERROR_INVALID_PARAMS);
                return false;
            }

            return iceSheet->GetFragmentData(fragmentIndex, *desc, *vertexCount, vertices, *triangleCount, triangles);
        }

        ICEPHYSICS_API bool IP_QueryIcePressureDistribution(uint32_t iceSheetId, const Vector3* shipPosition, const Quaternion* shipRotation, uint32_t* pointCount, PressurePoint* pressurePoints)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return false;
            }

            if (iceSheetId == 0)
            {
                SetError(ERROR_INVALID_ICE_SHEET_ID);
                return false;
            }

            if (!shipPosition || !shipRotation || !pointCount)
            {
                SetError(ERROR_NULL_POINTER);
                return false;
            }

            return world->QueryIcePressureDistribution(iceSheetId, shipPosition, shipRotation, pointCount, pressurePoints);
        }

        ICEPHYSICS_API void IP_SetHydrodynamicParams(const HydrodynamicParams* params)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (!params)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            world->GetHydrodynamicSystem().SetParams(params);
        }

        ICEPHYSICS_API void IP_GetHydrodynamicParams(HydrodynamicParams* params)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (!params)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            world->GetHydrodynamicSystem().GetParams(params);
        }

        ICEPHYSICS_API void IP_SetEngineThrustParams(const EngineThrustParams* params)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (!params)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            world->GetHydrodynamicSystem().SetEngineParams(params);
        }

        ICEPHYSICS_API void IP_GetEngineThrustParams(EngineThrustParams* params)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (!params)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            world->GetHydrodynamicSystem().GetEngineParams(params);
        }

        ICEPHYSICS_API void IP_UpdateShipState(uint32_t bodyId, const ShipState* state)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!state)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            world->UpdateShipState(bodyId, state);
        }

        ICEPHYSICS_API void IP_CalculateHydrodynamicForces(uint32_t bodyId, Vector3* totalForce, Vector3* totalTorque)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            if (!totalForce || !totalTorque)
            {
                SetError(ERROR_NULL_POINTER);
                return;
            }

            world->CalculateHydrodynamicForces(bodyId, totalForce, totalTorque);
        }

        ICEPHYSICS_API float IP_CalculateShipResistance(float speedKnots, float waterTemperature, float iceConcentration)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0.0f;
            }

            if (speedKnots < 0.0f)
            {
                SetError(ERROR_INVALID_PARAMS);
                return 0.0f;
            }

            return world->CalculateShipResistance(speedKnots, waterTemperature, iceConcentration);
        }

        ICEPHYSICS_API uint32_t IP_GetActiveRigidBodyCount()
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0;
            }

            return world->GetActiveRigidBodyCount();
        }

        ICEPHYSICS_API uint32_t IP_GetActiveIceSheetCount()
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0;
            }

            return world->GetActiveIceSheetCount();
        }

        ICEPHYSICS_API uint32_t IP_GetActiveCollisionCount()
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0;
            }

            return world->GetActiveCollisionCount();
        }

        ICEPHYSICS_API float IP_GetSimulationTime()
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0.0f;
            }

            return world->GetSimulationTime();
        }

        ICEPHYSICS_API uint64_t IP_GetFrameCount()
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return 0;
            }

            return world->GetFrameCount();
        }

        ICEPHYSICS_API const char* IP_GetLastError()
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                return ERROR_NOT_INITIALIZED;
            }

            return world->GetDebugLogger().GetLastError();
        }

        ICEPHYSICS_API void IP_ClearLastError()
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (world)
            {
                world->GetDebugLogger().ClearLastError();
            }
        }

        ICEPHYSICS_API void IP_SetRigidBodyCCDEnabled(uint32_t bodyId, bool enabled)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            body->SetCCDEnabled(enabled);
        }

        ICEPHYSICS_API bool IP_GetRigidBodyCCDEnabled(uint32_t bodyId)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world || bodyId == 0) return false;

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            return body ? body->IsCCDEnabled() : false;
        }

        ICEPHYSICS_API void IP_SetRigidBodyCCDRadius(uint32_t bodyId, float radius)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            body->SetCCDRadius(radius);
        }

        ICEPHYSICS_API float IP_GetRigidBodyCCDRadius(uint32_t bodyId)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world || bodyId == 0) return 0.0f;

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            return body ? body->GetCCDRadius() : 0.0f;
        }

        ICEPHYSICS_API void IP_SetRigidBodyCollisionGroup(uint32_t bodyId, uint32_t group, uint32_t mask)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (!body)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            body->SetCollisionFilter(group, mask);
        }

        ICEPHYSICS_API void IP_GetRigidBodyCollisionGroup(uint32_t bodyId, uint32_t* group, uint32_t* mask)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world || bodyId == 0 || !group || !mask) return;

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (body)
            {
                *group = body->GetCollisionGroup();
                *mask = body->GetCollisionMask();
            }
        }

        ICEPHYSICS_API void IP_SetRigidBodySleepThreshold(uint32_t bodyId, float threshold)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            if (body)
            {
                body->SetSleepThreshold(threshold);
            }
        }

        ICEPHYSICS_API float IP_GetRigidBodySleepThreshold(uint32_t bodyId)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world || bodyId == 0) return 0.0f;

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            return body ? body->GetSleepThreshold() : 0.0f;
        }

        ICEPHYSICS_API void IP_SetRigidBodyAwake(uint32_t bodyId, bool awake)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            world->GetRigidBodyManager().WakeUpBody(bodyId);
        }

        ICEPHYSICS_API bool IP_GetRigidBodyAwake(uint32_t bodyId)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world || bodyId == 0) return false;

            RigidBody* body = world->GetRigidBodyManager().GetBody(bodyId);
            return body ? body->IsAwake() : false;
        }

        ICEPHYSICS_API uint32_t IP_GetCCDTestCount()
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world) return 0;

            return world->GetCollisionWorld().GetCCDTestCount();
        }

        ICEPHYSICS_API uint32_t IP_GetSleepingBodyCount()
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world) return 0;

            return world->GetRigidBodyManager().GetSleepingCount();
        }

        ICEPHYSICS_API void IP_SetWindFieldParams(const WindFieldParams* params)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (!params)
            {
                SetError(ERROR_INVALID_PARAMS);
                return;
            }

            world->SetWindFieldParams(params);
        }

        ICEPHYSICS_API void IP_GetWindFieldParams(WindFieldParams* params)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world || !params) return;

            world->GetWindFieldParams(params);
        }

        ICEPHYSICS_API void IP_SetOceanCurrentParams(const OceanCurrentParams* params)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (!params)
            {
                SetError(ERROR_INVALID_PARAMS);
                return;
            }

            world->SetOceanCurrentParams(params);
        }

        ICEPHYSICS_API void IP_GetOceanCurrentParams(OceanCurrentParams* params)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world || !params) return;

            world->GetOceanCurrentParams(params);
        }

        ICEPHYSICS_API void IP_GetMarineEnvironmentState(MarineEnvironmentState* state)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world || !state) return;

            world->GetMarineEnvironmentState(state);
        }

        ICEPHYSICS_API void IP_CalculateWindForceOnShip(uint32_t bodyId, WindForceResult* result)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0 || !result)
            {
                SetError(ERROR_INVALID_PARAMS);
                return;
            }

            world->CalculateWindForceOnShip(bodyId, result);
        }

        ICEPHYSICS_API Vector3 IP_GetWaveHeightAtPosition(const Vector3* position, float time)
        {
            Vector3 result = { 0.0f, 0.0f, 0.0f };
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world || !position) return result;

            return world->GetWaveHeightAtPosition(*position);
        }

        ICEPHYSICS_API Vector3 IP_GetOceanCurrentAtPosition(const Vector3* position, float time)
        {
            Vector3 result = { 0.0f, 0.0f, 0.0f };
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world || !position) return result;

            return world->GetOceanCurrentAtPosition(*position);
        }

        ICEPHYSICS_API void IP_ApplyOceanForcesToFragment(uint32_t bodyId, float fragmentRadius, float fragmentMass)
        {
            PhysicsWorld* world = GetPhysicsWorld();
            if (!world)
            {
                SetError(ERROR_NOT_INITIALIZED);
                return;
            }

            if (bodyId == 0)
            {
                SetError(ERROR_INVALID_BODY_ID);
                return;
            }

            world->ApplyOceanForcesToFragment(bodyId, fragmentRadius, fragmentMass);
        }
    }
}
