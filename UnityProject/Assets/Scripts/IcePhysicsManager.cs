using System;
using System.Collections.Generic;
using UnityEngine;

namespace IcePhysicsUnity
{
    [DisallowMultipleComponent]
    public class IcePhysicsManager : MonoBehaviour
    {
        private static IcePhysicsManager s_instance;
        public static IcePhysicsManager Instance => s_instance;

        [Header("Simulation Settings")]
        [SerializeField] private float m_fixedTimestep = 1.0f / 60.0f;
        [SerializeField] private DebugLevel m_debugLevel = DebugLevel.Info;
        [SerializeField] private Vector3 m_gravity = new Vector3(0, -9.81f, 0);

        [Header("Environment Settings")]
        [SerializeField] private float m_waterDensity = 1025.0f;
        [SerializeField] private float m_waterTemperature = 0.0f;
        [SerializeField][Range(0, 1)] private float m_iceConcentration = 0.8f;

        [Header("Debug")]
        [SerializeField] private bool m_enableDebugLogging = true;
        [SerializeField] private bool m_drawGizmos = true;
        [SerializeField] private Color m_gizmoColor = Color.cyan;

        private bool m_isInitialized;
        private float m_simulationTime;
        private ulong m_frameCount;
        private uint m_activeRigidBodyCount;
        private uint m_activeIceSheetCount;
        private uint m_activeCollisionCount;

        private DebugLogCallback m_debugLogCallback;
        private CollisionEventCallback m_collisionEventCallback;
        private IceBreakCallback m_iceBreakCallback;

        public event Action<DebugLogEntry> OnDebugLog;
        public event Action<CollisionEventType, uint, uint, CollisionManifold[]> OnCollisionEvent;
        public event Action<uint, Vector3, IceFragmentDesc[]> OnIceBreak;

        public float FixedTimestep => m_fixedTimestep;

        public float SimulationTime => m_simulationTime;

        public ulong FrameCount => m_frameCount;

        public uint ActiveRigidBodyCount => m_activeRigidBodyCount;

        public uint ActiveIceSheetCount => m_activeIceSheetCount;

        public uint ActiveCollisionCount => m_activeCollisionCount;

        public float WaterDensity
        {
            get => m_waterDensity;
            set => m_waterDensity = value;
        }

        public float WaterTemperature
        {
            get => m_waterTemperature;
            set => m_waterTemperature = value;
        }

        public float IceConcentration
        {
            get => m_iceConcentration;
            set => m_iceConcentration = Mathf.Clamp01(value);
        }

        public Vector3 Gravity
        {
            get => m_gravity;
            set => m_gravity = value;
        }

        private void Awake()
        {
            if (s_instance = this;
        }

        private void Start()
        {
            InitializePhysics();
        }

        private void OnDestroy()
        {
            ShutdownPhysics();
        }

        private void Update()
        {
            if (!m_isInitialized)
            {
                IcePhysicsInterop.IP_Simulate(Time.deltaTime);
                UpdateStats();
            }
        }

        private void InitializePhysics()
        {
            try
            {
                if (IcePhysicsInterop.IP_Initialize(m_fixedTimestep, m_debugLevel);

                m_debugLogCallback = OnDebugLogHandler;
                m_collisionEventCallback = OnCollisionEventHandler;
                m_iceBreakCallback = OnIceBreakHandler;

                IcePhysicsInterop.IP_SetDebugLogCallback(m_debugLogCallback);
                IcePhysicsInterop.IP_SetCollisionEventCallback(m_collisionEventCallback);
                IcePhysicsInterop.IP_SetIceBreakCallback(m_iceBreakCallback);

                m_isInitialized = true;

                if (m_enableDebugLogging)
                {
                    Debug.Log("[IcePhysics] Physics engine initialized successfully.");
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[IcePhysics] Failed to initialize physics engine: {e.Message}");
                m_isInitialized = false;
            }
        }

        private void ShutdownPhysics()
        {
            if (m_isInitialized)
            {
                try
                {
                    IcePhysicsInterop.IP_Shutdown();
                    m_isInitialized = false;

                    if (m_enableDebugLogging)
                    {
                        Debug.Log("[IcePhysics] Physics engine shutdown successfully.");
                    }
                }
                catch (Exception e)
                {
                    Debug.LogError($"[IcePhysics] Error during shutdown: {e.Message}");
                }
            }

            if (s_instance == this)
            {
                s_instance = null;
            }
        }

        private void UpdateStats()
        {
            m_simulationTime = IcePhysicsInterop.IP_GetSimulationTime();
            m_frameCount = IcePhysicsInterop.IP_GetFrameCount();
            m_activeRigidBodyCount = IcePhysicsInterop.IP_GetActiveRigidBodyCount();
            m_activeIceSheetCount = IcePhysicsInterop.IP_GetActiveIceSheetCount();
            m_activeCollisionCount = IcePhysicsInterop.IP_GetActiveCollisionCount();
        }

        private void OnDebugLogHandler(ref DebugLogEntry entry)
        {
            if (m_enableDebugLogging)
            {
                string logMessage = $"[IcePhysics.{entry.subsystem}] {entry.message}");

                switch (entry.level)
                {
                    case DebugLevel.Error:
                        Debug.LogError(logMessage);
                        break;
                    case DebugLevel.Warning:
                        Debug.LogWarning(logMessage);
                        break;
                    case DebugLevel.Info:
                    case DebugLevel.Verbose:
                    case DebugLevel.Debug:
                        Debug.Log(logMessage);
                        break;
                }
            }

            OnDebugLog?.Invoke(entry);
        }

        private void OnCollisionEventHandler(CollisionEventType type, uint bodyA, uint bodyB, IntPtr manifoldPtr, uint manifoldCount)
        {
            CollisionManifold[] manifolds = IcePhysicsInterop.MarshalManifolds(manifoldPtr, manifoldCount);
            OnCollisionEvent?.Invoke(type, bodyA, bodyB, manifolds);
        }

        private void OnIceBreakHandler(uint iceSheetId, Vector3 impactPoint, uint fragmentCount, IntPtr fragmentsPtr)
        {
            IceFragmentDesc[] fragments = IcePhysicsInterop.MarshalFragments(fragmentsPtr, fragmentCount);
            OnIceBreak?.Invoke(iceSheetId, impactPoint, fragments);

            if (m_enableDebugLogging)
            {
                Debug.Log($"[IcePhysics] Ice sheet {iceSheetId} broke at {impactPoint} into {fragmentCount} fragments");
            }
        }

        public uint CreateRigidBody(RigidBodyDesc desc)
        {
            if (!m_isInitialized)
            {
                Debug.LogError("[IcePhysics] Engine not initialized");
                return 0;
            }

            return IcePhysicsInterop.IP_CreateRigidBody(ref desc);
        }

        public void DestroyRigidBody(uint bodyId)
        {
            if (m_isInitialized && bodyId != 0)
            {
                IcePhysicsInterop.IP_DestroyRigidBody(bodyId);
            }
        }

        public uint CreateIceSheet(IceSheetDesc desc)
        {
            if (!m_isInitialized)
            {
                Debug.LogError("[IcePhysics] Engine not initialized");
                return 0;
            }

            return IcePhysicsInterop.IP_CreateIceSheet(ref desc);
        }

        public void DestroyIceSheet(uint iceSheetId)
        {
            if (m_isInitialized && iceSheetId != 0)
            {
                IcePhysicsInterop.IP_DestroyIceSheet(iceSheetId);
            }
        }

        public void SetHydrodynamicParams(HydrodynamicParams params)
        {
            if (m_isInitialized)
            {
                IcePhysicsInterop.IP_SetHydrodynamicParams(ref params);
            }
        }

        public void SetEngineThrustParams(EngineThrustParams params)
        {
            if (m_isInitialized)
            {
                IcePhysicsInterop.IP_SetEngineThrustParams(ref params);
            }
        }

        public float CalculateShipResistance(float speedKnots)
        {
            return IcePhysicsInterop.IP_CalculateShipResistance(speedKnots, m_waterTemperature, m_iceConcentration);
        }

        private void OnDrawGizmos()
        {
            if (!m_drawGizmos || !m_isInitialized)
                return;

            Gizmos.color = m_gizmoColor;
            Gizmos.DrawWireSphere(Vector3.zero, 1.0f;

            Gizmos.color = Color.blue;
            Gizmos.DrawLine(Vector3.zero, m_gravity.normalized * 5.0f;
        }
    }
}
