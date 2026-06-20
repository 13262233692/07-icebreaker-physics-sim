#pragma once

#ifdef ICEPHYSICS_EXPORTS
#define ICEPHYSICS_API __declspec(dllexport)
#else
#define ICEPHYSICS_API __declspec(dllimport)
#endif

#include <stdint.h>

namespace IcePhysics
{
    struct Vector3
    {
        float x, y, z;

        Vector3() : x(0), y(0), z(0) {}
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
    };

    struct Vector2
    {
        float x, y;

        Vector2() : x(0), y(0) {}
        Vector2(float x, float y) : x(x), y(y) {}
    };

    struct Matrix3x3
    {
        float m[9];
    };

    struct Quaternion
    {
        float x, y, z, w;

        Quaternion() : x(0), y(0), z(0), w(1) {}
        Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    };

    struct AABB
    {
        Vector3 min;
        Vector3 max;
    };

    struct Vertex
    {
        Vector3 position;
        Vector3 normal;
        Vector2 uv;
    };

    struct Triangle
    {
        uint32_t indices[3];
    };

    struct HullFace
    {
        Vector3 normal;
        float distance;
        uint32_t firstVertex;
        uint32_t vertexCount;
    };

    enum RigidBodyFlags : uint32_t
    {
        RB_FLAG_NONE = 0,
        RB_FLAG_KINEMATIC = 1 << 0,
        RB_FLAG_SENSOR = 1 << 1,
        RB_FLAG_CCD_ENABLED = 1 << 2,
        RB_FLAG_ALWAYS_AWAKE = 1 << 3,
        RB_FLAG_ICE_FRAGMENT = 1 << 4,
        RB_FLAG_SHIP_HULL = 1 << 5,
    };

    struct RigidBodyDesc
    {
        Vector3 position;
        Quaternion rotation;
        Vector3 linearVelocity;
        Vector3 angularVelocity;
        float mass;
        Matrix3x3 inertiaTensor;
        float restitution;
        float staticFriction;
        float dynamicFriction;
        float linearDamping;
        float angularDamping;
        bool isKinematic;
        bool isSensor;
        uint32_t userData;
        uint32_t flags;
        uint32_t collisionGroup;
        uint32_t collisionMask;
        float sleepThreshold;
        float ccdRadius;
    };

    struct CollisionManifold
    {
        Vector3 normal;
        Vector3 point;
        float penetration;
        float impulse;
    };

    struct PressurePoint
    {
        Vector3 position;
        Vector3 normal;
        float pressure;
        float area;
    };

    struct IceSheetDesc
    {
        Vector3 position;
        Vector2 size;
        float thickness;
        float yieldStrength;
        float youngsModulus;
        float poissonsRatio;
        float density;
        uint32_t subdivisions;
    };

    struct IceFragmentDesc
    {
        Vector3 position;
        Quaternion rotation;
        Vector3 velocity;
        Vector3 angularVelocity;
        Vector3 centerOfMass;
        float mass;
        Matrix3x3 inertiaTensor;
        uint32_t vertexCount;
        uint32_t triangleCount;
    };

    struct HydrodynamicParams
    {
        float waterDensity;
        float dragCoefficient;
        float liftCoefficient;
        float addedMassCoefficient;
        float viscousDragCoefficient;
        float waveDragCoefficient;
        float hullWettedArea;
        float hullLength;
        float hullBeam;
        float hullDraft;
        float prismaticCoefficient;
        float blockCoefficient;
        float midshipCoefficient;
    };

    struct EngineThrustParams
    {
        float totalPower;
        float propellerEfficiency;
        float hullEfficiency;
        float maxRudderAngle;
        float rudderArea;
        float rudderAspectRatio;
        float waterDensity;
    };

    struct WindFieldParams
    {
        Vector3 windDirection;
        float windSpeed;
        float windGustFactor;
        float turbulenceIntensity;
        float airDensity;
        float boundaryLayerHeight;
    };

    struct OceanCurrentParams
    {
        Vector3 currentDirection;
        float currentSpeed;
        float waveAmplitude;
        float waveFrequency;
        float waveDirectionX;
        float waveDirectionZ;
        float swellAmplitude;
        float swellFrequency;
        float swellDirectionX;
        float swellDirectionZ;
        float tideHeight;
        float tidePeriod;
    };

    struct MarineEnvironmentState
    {
        Vector3 windVelocity;
        Vector3 currentVelocity;
        float waveHeight;
        float wavePeriod;
        float waveDirection;
        float seaState;
        float visibility;
    };

    struct WindForceResult
    {
        Vector3 windForce;
        Vector3 windTorque;
        float lateralDriftForce;
        float yawMoment;
        float driftAngleDeg;
        float relativeWindAngleDeg;
    };

    struct ShipState
    {
        Vector3 position;
        Quaternion rotation;
        Vector3 linearVelocity;
        Vector3 angularVelocity;
        float throttle;
        float rudderAngle;
        float speedKnots;
        float headingDegrees;
        float driftAngle;
    };

    enum class DebugLevel : uint32_t
    {
        None = 0,
        Error = 1,
        Warning = 2,
        Info = 3,
        Verbose = 4,
        Debug = 5
    };

    enum class CollisionEventType : uint32_t
    {
        ContactStart = 0,
        ContactPersist = 1,
        ContactEnd = 2,
        IceBreak = 3
    };

    struct DebugLogEntry
    {
        DebugLevel level;
        uint64_t timestamp;
        char message[512];
        char subsystem[64];
    };

    typedef void(*DebugLogCallback)(const DebugLogEntry* entry);
    typedef void(*CollisionEventCallback)(CollisionEventType type, uint32_t bodyA, uint32_t bodyB, const CollisionManifold* manifold, uint32_t manifoldCount);
    typedef void(*IceBreakCallback)(uint32_t iceSheetId, Vector3 impactPoint, uint32_t fragmentCount, const IceFragmentDesc* fragments);

    extern "C"
    {
        ICEPHYSICS_API bool IP_Initialize(float fixedTimestep, DebugLevel debugLevel);
        ICEPHYSICS_API void IP_Shutdown();
        ICEPHYSICS_API void IP_SetDebugLogCallback(DebugLogCallback callback);
        ICEPHYSICS_API void IP_SetCollisionEventCallback(CollisionEventCallback callback);
        ICEPHYSICS_API void IP_SetIceBreakCallback(IceBreakCallback callback);

        ICEPHYSICS_API void IP_Simulate(float deltaTime);

        ICEPHYSICS_API uint32_t IP_CreateRigidBody(const RigidBodyDesc* desc);
        ICEPHYSICS_API void IP_DestroyRigidBody(uint32_t bodyId);
        ICEPHYSICS_API void IP_SetRigidBodyPosition(uint32_t bodyId, const Vector3* position);
        ICEPHYSICS_API void IP_GetRigidBodyPosition(uint32_t bodyId, Vector3* position);
        ICEPHYSICS_API void IP_SetRigidBodyRotation(uint32_t bodyId, const Quaternion* rotation);
        ICEPHYSICS_API void IP_GetRigidBodyRotation(uint32_t bodyId, Quaternion* rotation);
        ICEPHYSICS_API void IP_SetRigidBodyLinearVelocity(uint32_t bodyId, const Vector3* velocity);
        ICEPHYSICS_API void IP_GetRigidBodyLinearVelocity(uint32_t bodyId, Vector3* velocity);
        ICEPHYSICS_API void IP_SetRigidBodyAngularVelocity(uint32_t bodyId, const Vector3* velocity);
        ICEPHYSICS_API void IP_GetRigidBodyAngularVelocity(uint32_t bodyId, Vector3* velocity);
        ICEPHYSICS_API void IP_ApplyForce(uint32_t bodyId, const Vector3* force, const Vector3* point);
        ICEPHYSICS_API void IP_ApplyTorque(uint32_t bodyId, const Vector3* torque);
        ICEPHYSICS_API void IP_ApplyImpulse(uint32_t bodyId, const Vector3* impulse, const Vector3* point);

        ICEPHYSICS_API uint32_t IP_CreateBoxCollider(uint32_t bodyId, const Vector3* halfExtents, const Vector3* offset, const Quaternion* rotationOffset);
        ICEPHYSICS_API uint32_t IP_CreateMeshCollider(uint32_t bodyId, const Vertex* vertices, uint32_t vertexCount, const Triangle* triangles, uint32_t triangleCount, const Vector3* offset, const Quaternion* rotationOffset);
        ICEPHYSICS_API uint32_t IP_CreateHullCollider(uint32_t bodyId, const Vector3* points, uint32_t pointCount, const Vector3* offset, const Quaternion* rotationOffset);
        ICEPHYSICS_API void IP_SetColliderFriction(uint32_t colliderId, float staticFriction, float dynamicFriction);
        ICEPHYSICS_API void IP_SetColliderRestitution(uint32_t colliderId, float restitution);

        ICEPHYSICS_API uint32_t IP_CreateIceSheet(const IceSheetDesc* desc);
        ICEPHYSICS_API void IP_DestroyIceSheet(uint32_t iceSheetId);
        ICEPHYSICS_API bool IP_GetIceSheetMesh(uint32_t iceSheetId, uint32_t* vertexCount, Vertex* vertices, uint32_t* triangleCount, Triangle* triangles);
        ICEPHYSICS_API uint32_t IP_GetIceSheetFragmentCount(uint32_t iceSheetId);
        ICEPHYSICS_API bool IP_GetIceFragmentData(uint32_t iceSheetId, uint32_t fragmentIndex, IceFragmentDesc* desc, uint32_t* vertexCount, Vertex* vertices, uint32_t* triangleCount, Triangle* triangles);
        ICEPHYSICS_API bool IP_QueryIcePressureDistribution(uint32_t iceSheetId, const Vector3* shipPosition, const Quaternion* shipRotation, uint32_t* pointCount, PressurePoint* pressurePoints);

        ICEPHYSICS_API void IP_SetHydrodynamicParams(const HydrodynamicParams* params);
        ICEPHYSICS_API void IP_GetHydrodynamicParams(HydrodynamicParams* params);
        ICEPHYSICS_API void IP_SetEngineThrustParams(const EngineThrustParams* params);
        ICEPHYSICS_API void IP_GetEngineThrustParams(EngineThrustParams* params);
        ICEPHYSICS_API void IP_UpdateShipState(uint32_t bodyId, const ShipState* state);
        ICEPHYSICS_API void IP_CalculateHydrodynamicForces(uint32_t bodyId, Vector3* totalForce, Vector3* totalTorque);
        ICEPHYSICS_API float IP_CalculateShipResistance(float speedKnots, float waterTemperature, float iceConcentration);

        ICEPHYSICS_API uint32_t IP_GetActiveRigidBodyCount();
        ICEPHYSICS_API uint32_t IP_GetActiveIceSheetCount();
        ICEPHYSICS_API uint32_t IP_GetActiveCollisionCount();
        ICEPHYSICS_API float IP_GetSimulationTime();
        ICEPHYSICS_API uint64_t IP_GetFrameCount();

        ICEPHYSICS_API void IP_SetRigidBodyCCDEnabled(uint32_t bodyId, bool enabled);
        ICEPHYSICS_API bool IP_GetRigidBodyCCDEnabled(uint32_t bodyId);
        ICEPHYSICS_API void IP_SetRigidBodyCCDRadius(uint32_t bodyId, float radius);
        ICEPHYSICS_API float IP_GetRigidBodyCCDRadius(uint32_t bodyId);

        ICEPHYSICS_API void IP_SetRigidBodyCollisionGroup(uint32_t bodyId, uint32_t group, uint32_t mask);
        ICEPHYSICS_API void IP_GetRigidBodyCollisionGroup(uint32_t bodyId, uint32_t* group, uint32_t* mask);

        ICEPHYSICS_API void IP_SetRigidBodySleepThreshold(uint32_t bodyId, float threshold);
        ICEPHYSICS_API float IP_GetRigidBodySleepThreshold(uint32_t bodyId);
        ICEPHYSICS_API void IP_SetRigidBodyAwake(uint32_t bodyId, bool awake);
        ICEPHYSICS_API bool IP_GetRigidBodyAwake(uint32_t bodyId);

        ICEPHYSICS_API uint32_t IP_GetCCDTestCount();
        ICEPHYSICS_API uint32_t IP_GetSleepingBodyCount();

        ICEPHYSICS_API void IP_SetWindFieldParams(const WindFieldParams* params);
        ICEPHYSICS_API void IP_GetWindFieldParams(WindFieldParams* params);
        ICEPHYSICS_API void IP_SetOceanCurrentParams(const OceanCurrentParams* params);
        ICEPHYSICS_API void IP_GetOceanCurrentParams(OceanCurrentParams* params);
        ICEPHYSICS_API void IP_GetMarineEnvironmentState(MarineEnvironmentState* state);

        ICEPHYSICS_API void IP_CalculateWindForceOnShip(uint32_t bodyId, WindForceResult* result);
        ICEPHYSICS_API Vector3 IP_GetWaveHeightAtPosition(const Vector3* position, float time);
        ICEPHYSICS_API Vector3 IP_GetOceanCurrentAtPosition(const Vector3* position, float time);
        ICEPHYSICS_API void IP_ApplyOceanForcesToFragment(uint32_t bodyId, float fragmentRadius, float fragmentMass);

        ICEPHYSICS_API const char* IP_GetLastError();
        ICEPHYSICS_API void IP_ClearLastError();
    }
}
