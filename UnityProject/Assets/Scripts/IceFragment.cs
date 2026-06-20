using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace IcePhysicsUnity
{
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(Rigidbody), typeof(MeshCollider))]
    public class IceFragment : MonoBehaviour
    {
        [SerializeField] private float m_lifetime = 30.0f;
        [SerializeField] private float m_sinkSpeed = 0.5f;
        [SerializeField] private float m_rotationDamping = 0.1f;
        [SerializeField] private Material m_fragmentMaterial;
        [SerializeField] private Color m_fragmentColor = new Color(0.8f, 0.9f, 0.95f, 0.85f);

        private IceFragmentDesc m_desc;
        private float m_iceDensity;
        private float m_waterDensity;
        private float m_buoyancy;
        private float m_age;
        private bool m_isSinking;
        private float m_fragmentRadius;

        private Mesh m_mesh;
        private new Rigidbody rigidbody;

        public IceFragmentDesc Description => m_desc;

        public float Age => m_age;

        public bool IsSinking => m_isSinking;

        private void Awake()
        {
            rigidbody = GetComponent<Rigidbody>();
        }

        public void Initialize(IceFragmentDesc desc, float iceDensity)
        {
            m_desc = desc;
            m_iceDensity = iceDensity;
            m_waterDensity = IcePhysicsManager.Instance != null ? IcePhysicsManager.Instance.WaterDensity : 1025.0f;

            transform.position = desc.position;
            transform.rotation = desc.rotation;

            GenerateFragmentMesh();
            SetupRigidBody();
            CalculateBuoyancy();
        }

        private void GenerateFragmentMesh()
        {
            if (m_desc.vertexCount == 0 || m_desc.triangleCount == 0)
            {
                GenerateSimpleBoxMesh();
                return;
            }

            IceSheet iceSheet = transform.parent?.GetComponent<IceSheet>();
            if (iceSheet == null)
            {
                GenerateSimpleBoxMesh();
                return;
            }

            uint iceSheetId = iceSheet.IceSheetId;
            uint fragmentIndex = 0;

            uint vertexCount = m_desc.vertexCount;
            uint triangleCount = m_desc.triangleCount;

            int vertexSize = Marshal.SizeOf<Vertex>();
            int triangleSize = Marshal.SizeOf<Triangle>();

            IntPtr vertexPtr = Marshal.AllocHGlobal((int)vertexCount * vertexSize);
            IntPtr trianglePtr = Marshal.AllocHGlobal((int)triangleCount * triangleSize);

            try
            {
                IceFragmentDesc fragmentDesc;
                if (IcePhysicsInterop.IP_GetIceFragmentData(iceSheetId, fragmentIndex, out fragmentDesc, out vertexCount, vertexPtr, out triangleCount, trianglePtr))
                {
                    Vertex[] vertices = IcePhysicsInterop.MarshalArrayFromIntPtr<Vertex>(vertexPtr, (int)vertexCount);
                    Triangle[] triangles = IcePhysicsInterop.MarshalArrayFromIntPtr<Triangle>(trianglePtr, (int)triangleCount);

                    CreateMeshFromData(vertices, triangles);
                }
                else
                {
                    GenerateSimpleBoxMesh();
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Failed to get fragment mesh data: {e.Message}, using fallback box mesh");
                GenerateSimpleBoxMesh();
            }
            finally
            {
                Marshal.FreeHGlobal(vertexPtr);
                Marshal.FreeHGlobal(trianglePtr);
            }
        }

        private void GenerateSimpleBoxMesh()
        {
            float size = Mathf.Max(1.0f, Mathf.Pow(m_desc.mass / m_iceDensity, 1.0f / 3.0f) * 0.5f);

            m_mesh = new Mesh
            {
                name = "IceFragmentMesh"
            };

            Vector3[] vertices = new Vector3[8]
            {
                new Vector3(-size, -size, -size),
                new Vector3(size, -size, -size),
                new Vector3(size, size, -size),
                new Vector3(-size, size, -size),
                new Vector3(-size, -size, size),
                new Vector3(size, -size, size),
                new Vector3(size, size, size),
                new Vector3(-size, size, size)
            };

            int[] triangles = new int[]
            {
                0, 2, 1, 0, 3, 2,
                4, 5, 6, 4, 6, 7,
                0, 1, 5, 0, 5, 4,
                1, 2, 6, 1, 6, 5,
                2, 3, 7, 2, 7, 6,
                3, 0, 4, 3, 4, 7
            };

            Vector3[] normals = new Vector3[8];
            for (int i = 0; i < 8; i++)
            {
                normals[i] = vertices[i].normalized;
            }

            Vector2[] uvs = new Vector2[8]
            {
                new Vector2(0, 0), new Vector2(1, 0), new Vector2(1, 1), new Vector2(0, 1),
                new Vector2(0, 0), new Vector2(1, 0), new Vector2(1, 1), new Vector2(0, 1)
            };

            m_mesh.vertices = vertices;
            m_mesh.normals = normals;
            m_mesh.uv = uvs;
            m_mesh.triangles = triangles;
            m_mesh.RecalculateBounds();

            ApplyMesh();
        }

        private void CreateMeshFromData(Vertex[] vertices, Triangle[] triangles)
        {
            m_mesh = new Mesh
            {
                name = "IceFragmentMesh",
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

            m_mesh.vertices = meshVertices;
            m_mesh.normals = meshNormals;
            m_mesh.uv = meshUVs;
            m_mesh.triangles = meshTriangles;
            m_mesh.RecalculateBounds();
            m_mesh.RecalculateTangents();

            ApplyMesh();
        }

        private void ApplyMesh()
        {
            MeshFilter meshFilter = GetComponent<MeshFilter>();
            MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
            MeshCollider meshCollider = GetComponent<MeshCollider>();

            meshFilter.sharedMesh = m_mesh;
            meshCollider.sharedMesh = m_mesh;

            if (m_fragmentMaterial != null)
            {
                meshRenderer.material = m_fragmentMaterial;
            }
            else
            {
                Shader iceShader = Shader.Find("Standard");
                Material material = new Material(iceShader);
                material.color = m_fragmentColor;
                material.SetFloat("_Glossiness", 0.25f);
                material.SetFloat("_Metallic", 0.0f);
                meshRenderer.material = material;
            }
        }

        private void SetupRigidBody()
        {
            if (rigidbody == null)
                rigidbody = GetComponent<Rigidbody>();

            rigidbody.mass = m_desc.mass > 0 ? m_desc.mass : 100.0f;
            rigidbody.linearDamping = 0.5f;
            rigidbody.angularDamping = m_rotationDamping;
            rigidbody.interpolation = RigidbodyInterpolation.Interpolate;
            rigidbody.collisionDetectionMode = CollisionDetectionMode.Discrete;
            rigidbody.useGravity = true;

            rigidbody.velocity = m_desc.velocity;
            rigidbody.angularVelocity = m_desc.angularVelocity;

            if (m_mesh != null)
            {
                Vector3 size = m_mesh.bounds.size;
                m_fragmentRadius = Mathf.Max(size.x, size.y, size.z) * 0.5f;
            }
            else
            {
                float volume = rigidbody.mass / m_iceDensity;
                m_fragmentRadius = Mathf.Pow(volume * 3.0f / (4.0f * Mathf.PI), 1.0f / 3.0f);
            }
        }

        private void CalculateBuoyancy()
        {
            float volume = rigidbody.mass / m_iceDensity;
            float displacedWaterMass = volume * m_waterDensity;
            m_buoyancy = displacedWaterMass * 9.81f;
        }

        private void Update()
        {
            m_age += Time.deltaTime;

            ApplyBuoyancy();

            if (m_age > m_lifetime * 0.8f && !m_isSinking)
            {
                m_isSinking = true;
            }

            if (m_isSinking)
            {
                Vector3 sinkForce = Vector3.down * m_sinkSpeed * rigidbody.mass;
                rigidbody.AddForce(sinkForce, ForceMode.Force);
            }

            if (m_age > m_lifetime)
            {
                Destroy(gameObject);
            }
        }

        private void FixedUpdate()
        {
            ApplyOceanForces();

            if (rigidbody == null)
                return;

            if (rigidbody.velocity.y < -10.0f)
            {
                Vector3 vel = rigidbody.velocity;
                vel.y = -10.0f;
                rigidbody.velocity = vel;
            }

            if (transform.position.y < -50.0f)
            {
                Destroy(gameObject);
            }
        }

        private void ApplyOceanForces()
        {
            if (IcePhysicsManager.Instance == null)
                return;

            float simTime = IcePhysicsManager.Instance.SimulationTime;
            Vector3 pos = transform.position;

            Vector3 currentVel = IcePhysicsInterop.IP_GetOceanCurrentAtPosition(ref pos, simTime);
            Vector3 waveHeight = IcePhysicsInterop.IP_GetWaveHeightAtPosition(ref pos, simTime);

            Vector3 relativeVel = currentVel - rigidbody.velocity;
            relativeVel.y = 0.0f;

            float dragCoeff = 0.8f;
            float crossSection = Mathf.PI * m_fragmentRadius * m_fragmentRadius;
            float waterDensity = m_waterDensity;

            Vector3 currentDragForce = 0.5f * waterDensity * crossSection * dragCoeff *
                relativeVel.magnitude * relativeVel;

            currentDragForce *= 0.1f;

            rigidbody.AddForce(currentDragForce, ForceMode.Force);

            float waterLevel = waveHeight.y;
            float buoyancyHeight = Mathf.Clamp01(waterLevel - transform.position.y);
            if (buoyancyHeight > 0.01f)
            {
                float torqueFactor = buoyancyHeight * 0.5f;
                Vector3 waveTorque = new Vector3(
                    (currentVel.z - rigidbody.velocity.z) * torqueFactor,
                    0.0f,
                    -(currentVel.x - rigidbody.velocity.x) * torqueFactor
                );
                rigidbody.AddTorque(waveTorque * rigidbody.mass, ForceMode.Force);
            }
        }

        private void ApplyBuoyancy()
        {
            if (IcePhysicsManager.Instance == null)
                return;

            float simTime = IcePhysicsManager.Instance.SimulationTime;
            Vector3 pos = transform.position;
            Vector3 waveHeight = IcePhysicsInterop.IP_GetWaveHeightAtPosition(ref pos, simTime);

            float waterLevel = waveHeight.y;
            float submergedHeight = Mathf.Clamp01(waterLevel - transform.position.y);

            if (submergedHeight > 0f)
            {
                float submergedVolume = m_mesh.bounds.size.x * m_mesh.bounds.size.z * submergedHeight;
                float buoyancyForce = submergedVolume * m_waterDensity * 9.81f;

                Vector3 buoyancy = Vector3.up * buoyancyForce;
                buoyancy *= 0.9f;

                rigidbody.AddForceAtPosition(buoyancy, transform.position + Vector3.up * submergedHeight * 0.5f, ForceMode.Force);

                Vector3 damping = -rigidbody.velocity * 0.3f * submergedHeight;
                rigidbody.AddForce(damping, ForceMode.Force);

                Vector3 angularDamping = -rigidbody.angularVelocity * 0.5f * submergedHeight;
                rigidbody.AddTorque(angularDamping, ForceMode.Force);
            }
        }

        private void OnCollisionEnter(Collision collision)
        {
            IcebreakerShip ship = collision.collider.GetComponent<IcebreakerShip>();
            if (ship != null)
            {
                float impactForce = collision.relativeVelocity.magnitude * rigidbody.mass;

                Vector3 reactionForce = collision.contacts[0].normal * impactForce * 0.3f;
                rigidbody.AddForce(reactionForce, ForceMode.Impulse);

                Vector3 torque = UnityEngine.Random.insideUnitSphere * impactForce * 0.1f;
                rigidbody.AddTorque(torque, ForceMode.Impulse);
            }
        }

        private void OnDrawGizmos()
        {
            if (m_mesh != null)
            {
                Gizmos.color = m_isSinking ? new Color(0.5f, 0.5f, 1.0f, 0.5f) : new Color(0.8f, 0.9f, 1.0f, 0.3f);
                Gizmos.DrawWireMesh(m_mesh, transform.position, transform.rotation);

                Gizmos.color = Color.green;
                Gizmos.DrawLine(transform.position, transform.position + Vector3.up * m_buoyancy * 0.001f);
            }
        }
    }
}
