using System;
using UnityEngine;

namespace IcePhysicsUnity
{
    [RequireComponent(typeof(Rigidbody))]
    public class IcebreakerShip : MonoBehaviour
    {
        [Header("Ship Properties")]
        [SerializeField] private float m_shipMass = 20000.0f;
        [SerializeField] private float m_shipLength = 135.0f;
        [SerializeField] private float m_shipBeam = 28.0f;
        [SerializeField] private float m_shipDraft = 8.5f;
        [SerializeField] private float m_shipDisplacement = 23000.0f;

        [Header("Engine Parameters")]
        [SerializeField] private float m_maxSpeedKnots = 21.0f;
        [SerializeField] private float m_maxThrottle = 1.0f;
        [SerializeField] private float m_maxRudderAngle = 35.0f;
        [SerializeField] private float m_enginePowerKW = 75000.0f;
        [SerializeField] private float m_propellerEfficiency = 0.75f;
        [SerializeField] private float m_hullEfficiency = 1.15f;

        [SerializeField] private float m_rudderArea = 35.0f;
        [SerializeField] private float m_rudderAspectRatio = 2.0f;

        [Header("Hydrodynamic Parameters")]
        [SerializeField] private float m_wettedSurfaceArea = 4000.0f;
        [SerializeField] private float m_prismaticCoefficient = 0.65f;
        [SerializeField] private float m_blockCoefficient = 0.85f;
        [SerializeField] private float m_midshipCoefficient = 0.98f;
        [SerializeField] private float m_dragCoefficient = 0.08f;
        [SerializeField] private float m_addedMassCoefficient = 0.1f;

        [Header("Control")]
        [SerializeField] private string m_throttleInputAxis = "Vertical";
        [SerializeField] private string m_rudderInputAxis = "Horizontal";
        [SerializeField] private float m_throttleSmoothTime = 2.0f;
        [SerializeField] private float m_rudderSmoothTime = 1.5f;

        [SerializeField] private bool m_useKeyboardControl = true;

        [Header("Physics Integration")]
        [SerializeField] private bool m_useNativePhysics = true;
        [SerializeField] private uint m_physicsBodyId;

        [Header("Debug")]
        [SerializeField] private bool m_drawDebugInfo = true;
        [SerializeField] private Color m_debugColor = Color.yellow;

        private float m_currentThrottle;
        private float m_targetThrottle;
        private float m_currentRudderAngle;
        private float m_targetRudderAngle;
        private float m_currentSpeedKnots;
        private float m_headingDegrees;
        private float m_driftAngle;

        private Vector3 m_linearVelocity;
        private Vector3 m_angularVelocity;

        private uint m_colliderId;
        private ShipState m_shipState;

        public float CurrentThrottle => m_currentThrottle;

        public float CurrentRudderAngle => m_currentRudderAngle;

        public float CurrentSpeedKnots => m_currentSpeedKnots;

        public float HeadingDegrees => m_headingDegrees;

        public uint PhysicsBodyId => m_physicsBodyId;

        public event Action<float> OnSpeedChanged;

        private void Start()
        {
            InitializeShip();
        }

        private void Update()
        {
            if (m_useKeyboardControl)
            {
                HandleInput();
            }

            UpdateControls();
            UpdateShipState();
        }

        private void FixedUpdate()
        {
            if (m_useNativePhysics && IcePhysicsManager.Instance != null)
            {
                UpdateNativePhysics();
            }
        }

        private void InitializeShip()
        {
            if (IcePhysicsManager.Instance == null)
            {
                Debug.LogError("IcePhysicsManager not found!");
                return;
            }

            if (m_useNativePhysics)
            {
                CreateNativeRigidBody();
                SetupHydrodynamicParams();
                SetupEngineParams();
            }
        }

        private void CreateNativeRigidBody()
        {
            RigidBodyDesc desc = new RigidBodyDesc
            {
                position = transform.position,
                rotation = transform.rotation,
                linearVelocity = Vector3.zero,
                angularVelocity = Vector3.zero,
                mass = m_shipMass,
                inertiaTensor = CalculateInertiaTensor(),
                restitution = 0.1f,
                staticFriction = 0.3f,
                dynamicFriction = 0.2f,
                linearDamping = 0.05f,
                angularDamping = 0.1f,
                isKinematic = false,
                isSensor = false,
                userData = 0,
                flags = (uint)(RigidBodyFlags.CCDEnabled | RigidBodyFlags.ShipHull | RigidBodyFlags.AlwaysAwake),
                collisionGroup = 0x00000001,
                collisionMask = 0xFFFFFFFF,
                sleepThreshold = 0.1f,
                ccdRadius = Mathf.Max(m_shipLength, m_shipBeam) * 0.5f
            };

            m_physicsBodyId = IcePhysicsManager.Instance.CreateRigidBody(desc);

            if (m_physicsBodyId != 0)
            {
                CreateShipCollider();
            }
        }

        private Matrix3x3 CalculateInertiaTensor()
        {
            float Ix = (1.0f / 12.0f) * m_shipMass * (m_shipBeam * m_shipBeam + m_shipDraft * m_shipDraft);
            float Iy = (1.0f / 12.0f) * m_shipMass * (m_shipLength * m_shipLength + m_shipDraft * m_shipDraft);
            float Iz = (1.0f / 12.0f) * m_shipMass * (m_shipLength * m_shipLength + m_shipBeam * m_shipBeam);

            return new Matrix3x3(new float[] { Ix, 0, 0, 0, Iy, 0, 0, 0, Iz });
        }

        private void CreateShipCollider()
        {
            Vector3 halfExtents = new Vector3(m_shipBeam * 0.5f, m_shipDraft * 0.5f, m_shipLength * 0.5f);
            Vector3 offset = new Vector3(0, -m_shipDraft * 0.25f, 0);
            Quaternion rotOffset = Quaternion.identity;

            m_colliderId = IcePhysicsInterop.IP_CreateBoxCollider(m_physicsBodyId, ref halfExtents, ref offset, ref rotOffset);

            if (m_colliderId != 0)
            {
                IcePhysicsInterop.IP_SetColliderFriction(m_colliderId, 0.2f, 0.15f);
                IcePhysicsInterop.IP_SetColliderRestitution(m_colliderId, 0.05f);
            }
        }

        private void SetupHydrodynamicParams()
        {
            HydrodynamicParams hydroParams = new HydrodynamicParams
            {
                waterDensity = IcePhysicsManager.Instance.WaterDensity,
                dragCoefficient = m_dragCoefficient,
                liftCoefficient = 0.05f,
                addedMassCoefficient = m_addedMassCoefficient,
                viscousDragCoefficient = 1.5f,
                waveDragCoefficient = 1.2f,
                hullWettedArea = m_wettedSurfaceArea,
                hullLength = m_shipLength,
                hullBeam = m_shipBeam,
                hullDraft = m_shipDraft,
                prismaticCoefficient = m_prismaticCoefficient,
                blockCoefficient = m_blockCoefficient,
                midshipCoefficient = m_midshipCoefficient
            };

            IcePhysicsManager.Instance.SetHydrodynamicParams(hydroParams);
        }

        private void SetupEngineParams()
        {
            EngineThrustParams engineParams = new EngineThrustParams
            {
                totalPower = m_enginePowerKW * 1000.0f,
                propellerEfficiency = m_propellerEfficiency,
                hullEfficiency = m_hullEfficiency,
                maxRudderAngle = m_maxRudderAngle,
                rudderArea = m_rudderArea,
                rudderAspectRatio = m_rudderAspectRatio,
                waterDensity = IcePhysicsManager.Instance.WaterDensity
            };

            IcePhysicsManager.Instance.SetEngineThrustParams(engineParams);
        }

        private void HandleInput()
        {
            float throttleInput = Input.GetAxis(m_throttleInputAxis);
            float rudderInput = Input.GetAxis(m_rudderInputAxis);

            m_targetThrottle = Mathf.Clamp(throttleInput, -1.0f, m_maxThrottle);
            m_targetRudderAngle = Mathf.Clamp(rudderInput, -1.0f, 1.0f) * m_maxRudderAngle;
        }

        private void UpdateControls()
        {
            m_currentThrottle = Mathf.MoveTowards(m_currentThrottle, m_targetThrottle, Time.deltaTime * m_throttleSmoothTime);
            m_currentRudderAngle = Mathf.MoveTowards(m_currentRudderAngle, m_targetRudderAngle, Time.deltaTime * m_rudderSmoothTime);
        }

        private void UpdateShipState()
        {
            if (m_headingDegrees = transform.eulerAngles.y;

            Vector3 worldVelocity = m_linearVelocity;
            m_currentSpeedKnots = worldVelocity.magnitude * 1.94384f;

            if (m_currentSpeedKnots > 0.5f)
            {
                Vector3 forward = transform.forward;
                Vector3 velocityDir = worldVelocity.normalized;
                float dot = Vector3.Dot(forward, velocityDir);
                m_driftAngle = Mathf.Acos(Mathf.Clamp(dot, -1.0f, 1.0f) * Mathf.Rad2Deg;
                if (Vector3.Cross(forward, velocityDir).y < 0)
                {
                    m_driftAngle = -m_driftAngle;
                }
            }
            else
            {
                m_driftAngle = 0.0f;
            }

            OnSpeedChanged?.Invoke(m_currentSpeedKnots);
        }

        private void UpdateNativePhysics()
        {
            m_shipState = new ShipState
            {
                position = transform.position,
                rotation = transform.rotation,
                linearVelocity = m_linearVelocity,
                angularVelocity = m_angularVelocity,
                throttle = m_currentThrottle,
                rudderAngle = m_currentRudderAngle,
                speedKnots = m_currentSpeedKnots,
                headingDegrees = m_headingDegrees,
                driftAngle = m_driftAngle
            };

            IcePhysicsInterop.IP_UpdateShipState(m_physicsBodyId, ref m_shipState);

            Vector3 totalForce;
            Vector3 totalTorque;
            IcePhysicsInterop.IP_CalculateHydrodynamicForces(m_physicsBodyId, out totalForce, out totalTorque);

            IcePhysicsInterop.IP_GetRigidBodyPosition(m_physicsBodyId, out m_shipState.position);
            IcePhysicsInterop.IP_GetRigidBodyRotation(m_physicsBodyId, out m_shipState.rotation);
            IcePhysicsInterop.IP_GetRigidBodyLinearVelocity(m_physicsBodyId, out m_linearVelocity);
            IcePhysicsInterop.IP_GetRigidBodyAngularVelocity(m_physicsBodyId, out m_angularVelocity);

            transform.position = m_shipState.position;
            transform.rotation = m_shipState.rotation;
        }

        public void SetThrottle(float throttle)
        {
            m_targetThrottle = Mathf.Clamp(throttle, -1.0f, m_maxThrottle);
        }

        public void SetRudderAngle(float angle)
        {
            m_targetRudderAngle = Mathf.Clamp(angle, -m_maxRudderAngle, m_maxRudderAngle);
        }

        private void OnGUI()
        {
            if (!m_drawDebugInfo)
                return;

            GUILayout.BeginArea(new Rect(10, 10, 300, 200));
            GUILayout.Label("=== Icebreaker Ship Status", GUILayout.Width(280));
            GUILayout.Space(10);
            GUILayout.Label($"Speed: {m_currentSpeedKnots:F2} knots");
            GUILayout.Label($"Throttle: {m_currentThrottle * 100:F0}%");
            GUILayout.Label($"Rudder: {m_currentRudderAngle:F1}°");
            GUILayout.Label($"Heading: {m_headingDegrees:F1}°");
            GUILayout.Label($"Drift: {m_driftAngle:F1}°");
            GUILayout.Space(10);

            Vector3 vel = m_linearVelocity;
            GUILayout.Label($"Velocity: ({vel.x:F2}, {vel.y:F2}, {vel.z:F2})");
            GUILayout.Label($"Position: ({transform.position.x:F1}, {transform.position.y:F1}, {transform.position.z:F1})");
            GUILayout.EndArea();
        }

        private void OnDrawGizmos()
        {
            if (!m_drawDebugInfo)
                return;

            Gizmos.color = m_debugColor;
            Gizmos.matrix = transform.localToWorldMatrix;

            Vector3 shipCenter = new Vector3(0, 0, 0);
            Vector3 shipSize = new Vector3(m_shipBeam, m_shipDraft, m_shipLength);
            Gizmos.DrawWireCube(shipCenter, shipSize);

            Gizmos.color = Color.green;
            Gizmos.DrawLine(Vector3.zero, Vector3.forward * 10.0f);

            Gizmos.color = Color.red;
            Gizmos.DrawLine(Vector3.zero, Vector3.right * 5.0f);

            Gizmos.color = Color.blue;
            Gizmos.DrawLine(Vector3.zero, Vector3.up * 3.0f);
        }
    }
}
