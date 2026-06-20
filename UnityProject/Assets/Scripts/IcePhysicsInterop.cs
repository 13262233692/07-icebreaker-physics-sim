using System;
using System.Runtime.InteropServices;

namespace IcePhysicsUnity
{
    [StructLayout(LayoutKind.Sequential)]
    public struct Vector3
    {
        public float x, y, z;

        public Vector3(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static Vector3 zero => new Vector3(0, 0, 0);
        public static Vector3 one => new Vector3(1, 1, 1);
        public static Vector3 right => new Vector3(1, 0, 0);
        public static Vector3 up => new Vector3(0, 1, 0);
        public static Vector3 forward => new Vector3(0, 0, 1);

        public static implicit operator UnityEngine.Vector3(Vector3 v)
        {
            return new UnityEngine.Vector3(v.x, v.y, v.z);
        }

        public static implicit operator Vector3(UnityEngine.Vector3 v)
        {
            return new Vector3(v.x, v.y, v.z);
        }

        public override string ToString()
        {
            return $"({x:F3}, {y:F3}, {z:F3})";
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Vector2
    {
        public float x, y;

        public Vector2(float x, float y)
        {
            this.x = x;
            this.y = y;
        }

        public static Vector2 zero => new Vector2(0, 0);

        public static implicit operator UnityEngine.Vector2(Vector2 v)
        {
            return new UnityEngine.Vector2(v.x, v.y);
        }

        public static implicit operator Vector2(UnityEngine.Vector2 v)
        {
            return new Vector2(v.x, v.y);
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Quaternion
    {
        public float x, y, z, w;

        public Quaternion(float x, float y, float z, float w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        public static Quaternion identity => new Quaternion(0, 0, 0, 1);

        public static implicit operator UnityEngine.Quaternion(Quaternion q)
        {
            return new UnityEngine.Quaternion(q.x, q.y, q.z, q.w);
        }

        public static implicit operator Quaternion(UnityEngine.Quaternion q)
        {
            return new Quaternion(q.x, q.y, q.z, q.w);
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Matrix3x3
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
        public float[] m;

        public Matrix3x3(float[] values)
        {
            m = new float[9];
            if (values != null && values.Length >= 9)
            {
                Array.Copy(values, m, 9);
            }
        }

        public static Matrix3x3 identity
        {
            get
            {
                return new Matrix3x3(new float[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 });
            }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Vertex
    {
        public Vector3 position;
        public Vector3 normal;
        public Vector2 uv;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Triangle
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public uint[] indices;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct RigidBodyDesc
    {
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 linearVelocity;
        public Vector3 angularVelocity;
        public float mass;
        public Matrix3x3 inertiaTensor;
        public float restitution;
        public float staticFriction;
        public float dynamicFriction;
        public float linearDamping;
        public float angularDamping;
        [MarshalAs(UnmanagedType.U1)] public bool isKinematic;
        [MarshalAs(UnmanagedType.U1)] public bool isSensor;
        public uint userData;
        public uint flags;
        public uint collisionGroup;
        public uint collisionMask;
        public float sleepThreshold;
        public float ccdRadius;
    }

    [Flags]
    public enum RigidBodyFlags : uint
    {
        None = 0,
        Kinematic = (1 << 0),
        Sensor = (1 << 1),
        CCDEnabled = (1 << 2),
        AlwaysAwake = (1 << 3),
        IceFragment = (1 << 4),
        ShipHull = (1 << 5),
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct CollisionManifold
    {
        public Vector3 normal;
        public Vector3 point;
        public float penetration;
        public float impulse;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PressurePoint
    {
        public Vector3 position;
        public Vector3 normal;
        public float pressure;
        public float area;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct IceSheetDesc
    {
        public Vector3 position;
        public Vector2 size;
        public float thickness;
        public float yieldStrength;
        public float youngsModulus;
        public float poissonsRatio;
        public float density;
        public uint subdivisions;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct IceFragmentDesc
    {
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 velocity;
        public Vector3 angularVelocity;
        public Vector3 centerOfMass;
        public float mass;
        public Matrix3x3 inertiaTensor;
        public uint vertexCount;
        public uint triangleCount;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct HydrodynamicParams
    {
        public float waterDensity;
        public float dragCoefficient;
        public float liftCoefficient;
        public float addedMassCoefficient;
        public float viscousDragCoefficient;
        public float waveDragCoefficient;
        public float hullWettedArea;
        public float hullLength;
        public float hullBeam;
        public float hullDraft;
        public float prismaticCoefficient;
        public float blockCoefficient;
        public float midshipCoefficient;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct EngineThrustParams
    {
        public float totalPower;
        public float propellerEfficiency;
        public float hullEfficiency;
        public float maxRudderAngle;
        public float rudderArea;
        public float rudderAspectRatio;
        public float waterDensity;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WindFieldParams
    {
        public Vector3 windDirection;
        public float windSpeed;
        public float windGustFactor;
        public float turbulenceIntensity;
        public float airDensity;
        public float boundaryLayerHeight;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct OceanCurrentParams
    {
        public Vector3 currentDirection;
        public float currentSpeed;
        public float waveAmplitude;
        public float waveFrequency;
        public float waveDirectionX;
        public float waveDirectionZ;
        public float swellAmplitude;
        public float swellFrequency;
        public float swellDirectionX;
        public float swellDirectionZ;
        public float tideHeight;
        public float tidePeriod;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct MarineEnvironmentState
    {
        public Vector3 windVelocity;
        public Vector3 currentVelocity;
        public float waveHeight;
        public float wavePeriod;
        public float waveDirection;
        public float seaState;
        public float visibility;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WindForceResult
    {
        public Vector3 windForce;
        public Vector3 windTorque;
        public float lateralDriftForce;
        public float yawMoment;
        public float driftAngleDeg;
        public float relativeWindAngleDeg;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ShipState
    {
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 linearVelocity;
        public Vector3 angularVelocity;
        public float throttle;
        public float rudderAngle;
        public float speedKnots;
        public float headingDegrees;
        public float driftAngle;
    }

    public enum DebugLevel : uint
    {
        None = 0,
        Error = 1,
        Warning = 2,
        Info = 3,
        Verbose = 4,
        Debug = 5
    }

    public enum CollisionEventType : uint
    {
        ContactStart = 0,
        ContactPersist = 1,
        ContactEnd = 2,
        IceBreak = 3
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct DebugLogEntry
    {
        public DebugLevel level;
        public ulong timestamp;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
        public string message;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
        public string subsystem;
    }

    public delegate void DebugLogCallback(ref DebugLogEntry entry);
    public delegate void CollisionEventCallback(CollisionEventType type, uint bodyA, uint bodyB, IntPtr manifold, uint manifoldCount);
    public delegate void IceBreakCallback(uint iceSheetId, Vector3 impactPoint, uint fragmentCount, IntPtr fragments);

    public static class IcePhysicsInterop
    {
        private const string DLL_NAME = "IcePhysicsEngine";

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool IP_Initialize(float fixedTimestep, DebugLevel debugLevel);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_Shutdown();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetDebugLogCallback(DebugLogCallback callback);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetCollisionEventCallback(CollisionEventCallback callback);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetIceBreakCallback(IceBreakCallback callback);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_Simulate(float deltaTime);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint IP_CreateRigidBody(ref RigidBodyDesc desc);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_DestroyRigidBody(uint bodyId);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetRigidBodyPosition(uint bodyId, ref Vector3 position);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_GetRigidBodyPosition(uint bodyId, out Vector3 position);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetRigidBodyRotation(uint bodyId, ref Quaternion rotation);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_GetRigidBodyRotation(uint bodyId, out Quaternion rotation);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetRigidBodyLinearVelocity(uint bodyId, ref Vector3 velocity);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_GetRigidBodyLinearVelocity(uint bodyId, out Vector3 velocity);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetRigidBodyAngularVelocity(uint bodyId, ref Vector3 velocity);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_GetRigidBodyAngularVelocity(uint bodyId, out Vector3 velocity);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_ApplyForce(uint bodyId, ref Vector3 force, ref Vector3 point);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_ApplyTorque(uint bodyId, ref Vector3 torque);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_ApplyImpulse(uint bodyId, ref Vector3 impulse, ref Vector3 point);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetRigidBodyCCDEnabled(uint bodyId, [MarshalAs(UnmanagedType.U1)] bool enabled);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.U1)]
        public static extern bool IP_GetRigidBodyCCDEnabled(uint bodyId);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetRigidBodyCCDRadius(uint bodyId, float radius);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern float IP_GetRigidBodyCCDRadius(uint bodyId);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetRigidBodyCollisionGroup(uint bodyId, uint group, uint mask);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_GetRigidBodyCollisionGroup(uint bodyId, out uint group, out uint mask);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetRigidBodySleepThreshold(uint bodyId, float threshold);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern float IP_GetRigidBodySleepThreshold(uint bodyId);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetRigidBodyAwake(uint bodyId, [MarshalAs(UnmanagedType.U1)] bool awake);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.U1)]
        public static extern bool IP_GetRigidBodyAwake(uint bodyId);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint IP_CreateBoxCollider(uint bodyId, ref Vector3 halfExtents, ref Vector3 offset, ref Quaternion rotationOffset);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint IP_CreateMeshCollider(uint bodyId, Vertex[] vertices, uint vertexCount, Triangle[] triangles, uint triangleCount, ref Vector3 offset, ref Quaternion rotationOffset);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint IP_CreateHullCollider(uint bodyId, Vector3[] points, uint pointCount, ref Vector3 offset, ref Quaternion rotationOffset);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetColliderFriction(uint colliderId, float staticFriction, float dynamicFriction);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetColliderRestitution(uint colliderId, float restitution);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint IP_CreateIceSheet(ref IceSheetDesc desc);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_DestroyIceSheet(uint iceSheetId);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool IP_GetIceSheetMesh(uint iceSheetId, out uint vertexCount, IntPtr vertices, out uint triangleCount, IntPtr triangles);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint IP_GetIceSheetFragmentCount(uint iceSheetId);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool IP_GetIceFragmentData(uint iceSheetId, uint fragmentIndex, out IceFragmentDesc desc, out uint vertexCount, IntPtr vertices, out uint triangleCount, IntPtr triangles);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool IP_QueryIcePressureDistribution(uint iceSheetId, ref Vector3 shipPosition, ref Quaternion shipRotation, ref uint pointCount, IntPtr pressurePoints);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetHydrodynamicParams(ref HydrodynamicParams params);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_GetHydrodynamicParams(out HydrodynamicParams params);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetEngineThrustParams(ref EngineThrustParams params);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_GetEngineThrustParams(out EngineThrustParams params);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_UpdateShipState(uint bodyId, ref ShipState state);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_CalculateHydrodynamicForces(uint bodyId, out Vector3 totalForce, out Vector3 totalTorque);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern float IP_CalculateShipResistance(float speedKnots, float waterTemperature, float iceConcentration);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint IP_GetActiveRigidBodyCount();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint IP_GetActiveIceSheetCount();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint IP_GetActiveCollisionCount();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint IP_GetCCDTestCount();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint IP_GetSleepingBodyCount();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetWindFieldParams(ref WindFieldParams params);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_GetWindFieldParams(out WindFieldParams params);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_SetOceanCurrentParams(ref OceanCurrentParams params);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_GetOceanCurrentParams(out OceanCurrentParams params);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_GetMarineEnvironmentState(out MarineEnvironmentState state);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_CalculateWindForceOnShip(uint bodyId, out WindForceResult result);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern Vector3 IP_GetWaveHeightAtPosition(ref Vector3 position, float time);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern Vector3 IP_GetOceanCurrentAtPosition(ref Vector3 position, float time);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_ApplyOceanForcesToFragment(uint bodyId, float fragmentRadius, float fragmentMass);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern float IP_GetSimulationTime();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern ulong IP_GetFrameCount();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr IP_GetLastError();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IP_ClearLastError();

        public static string GetLastErrorString()
        {
            IntPtr ptr = IP_GetLastError();
            return ptr != IntPtr.Zero ? Marshal.PtrToStringAnsi(ptr) : string.Empty;
        }

        public static T[] MarshalArrayFromIntPtr<T>(IntPtr ptr, int count) where T : struct
        {
            if (ptr == IntPtr.Zero || count <= 0)
                return new T[0];

            T[] result = new T[count];
            int size = Marshal.SizeOf<T>();

            for (int i = 0; i < count; i++)
            {
                IntPtr elementPtr = new IntPtr(ptr.ToInt64() + i * size);
                result[i] = Marshal.PtrToStructure<T>(elementPtr);
            }

            return result;
        }

        public static CollisionManifold[] MarshalManifolds(IntPtr ptr, uint count)
        {
            return MarshalArrayFromIntPtr<CollisionManifold>(ptr, (int)count);
        }

        public static IceFragmentDesc[] MarshalFragments(IntPtr ptr, uint count)
        {
            return MarshalArrayFromIntPtr<IceFragmentDesc>(ptr, (int)count);
        }
    }
}
