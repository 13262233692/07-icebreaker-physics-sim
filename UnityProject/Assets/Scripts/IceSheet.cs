using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

namespace IcePhysicsUnity
{
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(MeshCollider))]
    public class IceSheet : MonoBehaviour
    {
        [Header("Ice Sheet Properties")]
        [SerializeField] private Vector2 m_size = new Vector2(200.0f, 200.0f);
        [SerializeField] private float m_thickness = 1.5f;
        [SerializeField] private float m_yieldStrength = 1.5e6f;
        [SerializeField] private float m_youngsModulus = 9.0e9f;
        [SerializeField] private float m_poissonsRatio = 0.33f;
        [SerializeField] private float m_density = 917.0f;
        [SerializeField] private uint m_subdivisions = 64;

        [Header("Visualization")]
        [SerializeField] private Material m_iceMaterial;
        [SerializeField] private Color m_iceColor = new Color(0.85f, 0.95f, 1.0f, 0.9f);
        [SerializeField] private bool m_showPressureDistribution = true;
        [SerializeField] private Gradient m_pressureGradient;

        [Header("Debug")]
        [SerializeField] private bool m_drawGizmos = true;
        [SerializeField] private Color m_gizmoColor = Color.cyan;

        private uint m_iceSheetId;
        private Mesh m_iceMesh;
        private MeshFilter m_meshFilter;
        private MeshRenderer m_meshRenderer;
        private MeshCollider m_meshCollider;

        private List<IceFragment> m_fragments = new List<IceFragment>();
        private PressurePoint[] m_pressurePoints;

        [SerializeField] private IcebreakerShip m_targetShip;

        public uint IceSheetId => m_iceSheetId;

        public Vector2 Size => m_size;

        public float Thickness => m_thickness;

        public float YieldStrength => m_yieldStrength;

        public int FragmentCount => m_fragments.Count;

        public event Action<uint, Vector3, IceFragmentDesc[]> OnIceBroken;

        private void Awake()
        {
            m_meshFilter = GetComponent<MeshFilter>();
            m_meshRenderer = GetComponent<MeshRenderer>();
            m_meshCollider = GetComponent<MeshCollider>();
        }

        private void Start()
        {
            InitializeIceSheet();
        }

        private void OnDestroy()
        {
            if (m_iceSheetId != 0 && IcePhysicsManager.Instance != null)
            {
                IcePhysicsManager.Instance.DestroyIceSheet(m_iceSheetId);
            }

            ClearFragments();
        }

        private void Update()
        {
            if (m_iceSheetId != 0 && m_targetShip != null && m_showPressureDistribution)
            {
                UpdatePressureDistribution();
            }

            UpdateFragments();
        }

        private void InitializeIceSheet()
        {
            if (IcePhysicsManager.Instance == null)
            {
                Debug.LogError("IcePhysicsManager not found!");
                return;
            }

            IceSheetDesc desc = new IceSheetDesc
            {
                position = transform.position,
                size = m_size,
                thickness = m_thickness,
                yieldStrength = m_yieldStrength,
                youngsModulus = m_youngsModulus,
                poissonsRatio = m_poissonsRatio,
                density = m_density,
                subdivisions = m_subdivisions
            };

            m_iceSheetId = IcePhysicsManager.Instance.CreateIceSheet(desc);

            if (m_iceSheetId != 0)
            {
                GenerateIceMesh();
                SetupMaterial();

                IcePhysicsManager.Instance.OnIceBreak += OnIceBreakHandler;
            }
            else
            {
                Debug.LogError("Failed to create ice sheet in physics engine");
            }
        }

        private void GenerateIceMesh()
        {
            uint vertexCount = 0;
            uint triangleCount = 0;

            if (!IcePhysicsInterop.IP_GetIceSheetMesh(m_iceSheetId, out vertexCount, IntPtr.Zero, out triangleCount, IntPtr.Zero))
            {
                Debug.LogError("Failed to get ice sheet mesh size");
                return;
            }

            if (vertexCount == 0 || triangleCount == 0)
            {
                Debug.LogError("Ice sheet mesh is empty");
                return;
            }

            int vertexSize = Marshal.SizeOf<Vertex>();
            int triangleSize = Marshal.SizeOf<Triangle>();

            IntPtr vertexPtr = Marshal.AllocHGlobal((int)vertexCount * vertexSize);
            IntPtr trianglePtr = Marshal.AllocHGlobal((int)triangleCount * triangleSize);

            try
            {
                if (IcePhysicsInterop.IP_GetIceSheetMesh(m_iceSheetId, out vertexCount, vertexPtr, out triangleCount, trianglePtr))
                {
                    Vertex[] vertices = IcePhysicsInterop.MarshalArrayFromIntPtr<Vertex>(vertexPtr, (int)vertexCount);
                    Triangle[] triangles = IcePhysicsInterop.MarshalArrayFromIntPtr<Triangle>(trianglePtr, (int)triangleCount);

                    m_iceMesh = new Mesh
                    {
                        name = "IceSheetMesh",
                        indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
                    };

                    Vector3[] meshVertices = new Vector3[vertices.Length];
                    Vector3[] meshNormals = new Vector3[vertices.Length];
                    Vector2[] meshUVs = new Vector2[vertices.Length];

                    for (int i = 0; i < vertices.Length; i++)
                    {
                        meshVertices[i] = vertices[i].position;
                        meshNormals[i] = vertices[i].normal;
                        meshUVs[i] = vertices[i].uv;
                    }

                    int[] meshTriangles = new int[triangles.Length * 3];
                    for (int i = 0; i < triangles.Length; i++)
                    {
                        meshTriangles[i * 3] = (int)triangles[i].indices[0];
                        meshTriangles[i * 3 + 1] = (int)triangles[i].indices[1];
                        meshTriangles[i * 3 + 2] = (int)triangles[i].indices[2];
                    }

                    m_iceMesh.vertices = meshVertices;
                    m_iceMesh.normals = meshNormals;
                    m_iceMesh.uv = meshUVs;
                    m_iceMesh.triangles = meshTriangles;
                    m_iceMesh.RecalculateBounds();
                    m_iceMesh.RecalculateTangents();

                    m_meshFilter.sharedMesh = m_iceMesh;
                    m_meshCollider.sharedMesh = m_iceMesh;
                }
            }
            finally
            {
                Marshal.FreeHGlobal(vertexPtr);
                Marshal.FreeHGlobal(trianglePtr);
            }
        }

        private void SetupMaterial()
        {
            if (m_iceMaterial != null)
            {
                m_meshRenderer.material = m_iceMaterial;
            }
            else
            {
                Shader iceShader = Shader.Find("Standard");
                Material material = new Material(iceShader);
                material.color = m_iceColor;
                material.SetFloat("_Glossiness", 0.3f);
                material.SetFloat("_Metallic", 0.0f);
                m_meshRenderer.material = material;
            }
        }

        private void UpdatePressureDistribution()
        {
            if (m_targetShip == null)
                return;

            Vector3 shipPos = m_targetShip.transform.position;
            Quaternion shipRot = m_targetShip.transform.rotation;

            uint pointCount = 0;
            IcePhysicsInterop.IP_QueryIcePressureDistribution(m_iceSheetId, ref shipPos, ref shipRot, ref pointCount, IntPtr.Zero);

            if (pointCount > 0)
            {
                int pointSize = Marshal.SizeOf<PressurePoint>();
                IntPtr pointsPtr = Marshal.AllocHGlobal((int)pointCount * pointSize);

                try
                {
                    if (IcePhysicsInterop.IP_QueryIcePressureDistribution(m_iceSheetId, ref shipPos, ref shipRot, ref pointCount, pointsPtr))
                    {
                        m_pressurePoints = IcePhysicsInterop.MarshalArrayFromIntPtr<PressurePoint>(pointsPtr, (int)pointCount);
                        UpdatePressureVisualization();
                    }
                }
                finally
                {
                    Marshal.FreeHGlobal(pointsPtr);
                }
            }
            else
            {
                m_pressurePoints = null;
            }
        }

        private void UpdatePressureVisualization()
        {
            if (m_pressurePoints == null || m_pressurePoints.Length == 0)
                return;

            MeshRenderer renderer = GetComponent<MeshRenderer>();
            if (renderer != null && renderer.material != null)
            {
                float maxPressure = 0f;
                foreach (var point in m_pressurePoints)
                {
                    if (point.pressure > maxPressure)
                        maxPressure = point.pressure;
                }

                if (maxPressure > 0f)
                {
                    float pressureRatio = Mathf.Clamp01(maxPressure / m_yieldStrength);
                    Color pressureColor = m_pressureGradient.Evaluate(pressureRatio);
                    renderer.material.color = Color.Lerp(m_iceColor, pressureColor, pressureRatio * 0.5f);
                }
            }
        }

        private void OnIceBreakHandler(uint iceSheetId, Vector3 impactPoint, IceFragmentDesc[] fragments)
        {
            if (iceSheetId != m_iceSheetId)
                return;

            OnIceBroken?.Invoke(iceSheetId, impactPoint, fragments);
            SpawnFragments(fragments);

            Debug.Log($"Ice sheet broken at {impactPoint}, created {fragments.Length} fragments");
        }

        private void SpawnFragments(IceFragmentDesc[] fragments)
        {
            foreach (var desc in fragments)
            {
                GameObject fragmentObj = new GameObject($"IceFragment_{desc.vertexCount}");
                fragmentObj.transform.SetParent(transform.parent);

                IceFragment fragment = fragmentObj.AddComponent<IceFragment>();
                fragment.Initialize(desc, m_density);

                m_fragments.Add(fragment);
            }

            gameObject.SetActive(false);
        }

        private void UpdateFragments()
        {
            for (int i = m_fragments.Count - 1; i >= 0; i--)
            {
                if (m_fragments[i] == null)
                {
                    m_fragments.RemoveAt(i);
                }
            }
        }

        private void ClearFragments()
        {
            foreach (var fragment in m_fragments)
            {
                if (fragment != null && fragment.gameObject != null)
                {
                    Destroy(fragment.gameObject);
                }
            }
            m_fragments.Clear();
        }

        public void SetTargetShip(IcebreakerShip ship)
        {
            m_targetShip = ship;
        }

        public PressurePoint[] GetPressurePoints()
        {
            return m_pressurePoints;
        }

        private void OnDrawGizmos()
        {
            if (!m_drawGizmos)
                return;

            Gizmos.color = m_gizmoColor;
            Gizmos.matrix = transform.localToWorldMatrix;

            Vector3 center = Vector3.zero;
            Vector3 size = new Vector3(m_size.x, m_thickness, m_size.y);
            Gizmos.DrawWireCube(center, size);

            if (m_pressurePoints != null && m_pressurePoints.Length > 0)
            {
                foreach (var point in m_pressurePoints)
                {
                    float pressureRatio = Mathf.Clamp01(point.pressure / m_yieldStrength);
                    Gizmos.color = m_pressureGradient.Evaluate(pressureRatio);
                    Gizmos.DrawSphere(point.position, 0.3f + pressureRatio * 0.5f);

                    Gizmos.color = Color.white;
                    Gizmos.DrawLine(point.position, point.position + point.normal * (1.0f + pressureRatio * 2.0f));
                }
            }
        }
    }
}
