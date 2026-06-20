#include "../include/RigidBody.h"
#include "../include/PhysicsWorld.h"

namespace IcePhysics
{
    RigidBody::RigidBody(const RigidBodyDesc& desc)
        : m_id(0)
        , m_position(desc.position)
        , m_rotation(Math::QuatNormalize(desc.rotation))
        , m_linearVelocity(desc.linearVelocity)
        , m_angularVelocity(desc.angularVelocity)
        , m_forceAccumulator(Math::Vec3Zero())
        , m_torqueAccumulator(Math::Vec3Zero())
        , m_prevPosition(desc.position)
        , m_prevRotation(desc.rotation)
        , m_mass(desc.mass)
        , m_invMass(desc.mass > 0.0f ? 1.0f / desc.mass : 0.0f)
        , m_inertiaTensor(desc.inertiaTensor)
        , m_invInertiaTensor(Math::Mat3Inverse(desc.inertiaTensor))
        , m_restitution(desc.restitution)
        , m_staticFriction(desc.staticFriction)
        , m_dynamicFriction(desc.dynamicFriction)
        , m_linearDamping(desc.linearDamping)
        , m_angularDamping(desc.angularDamping)
        , m_isKinematic(desc.isKinematic)
        , m_isSensor(desc.isSensor)
        , m_isAwake(true)
        , m_isDirty(true)
        , m_isCCDEnabled(false)
        , m_userData(desc.userData)
        , m_flags(desc.flags)
        , m_collisionGroup(desc.collisionGroup)
        , m_collisionMask(desc.collisionMask ? desc.collisionMask : 0xFFFFFFFF)
        , m_islandId(0)
        , m_sleepThreshold(desc.sleepThreshold > 0.0f ? desc.sleepThreshold : 0.08f)
        , m_sleepEnergy(0.0f)
        , m_sleepTimer(0)
        , m_ccdRadius(desc.ccdRadius > 0.0f ? desc.ccdRadius : 0.5f)
    {
        if (desc.flags & RB_FLAG_CCD_ENABLED)
        {
            m_isCCDEnabled = true;
        }
        if (desc.flags & RB_FLAG_ALWAYS_AWAKE)
        {
            m_sleepThreshold = 0.0f;
        }
    }

    RigidBody::~RigidBody()
    {
    }

    void RigidBody::SetMass(float mass)
    {
        m_mass = mass;
        m_invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
        m_isDirty = true;
    }

    void RigidBody::SetInertiaTensor(const Matrix3x3& tensor)
    {
        m_inertiaTensor = tensor;
        m_invInertiaTensor = Math::Mat3Inverse(tensor);
        m_isDirty = true;
    }

    Matrix3x3 RigidBody::GetWorldInertiaTensor() const
    {
        Matrix3x3 rotMat = Math::Mat3FromQuat(m_rotation);
        Matrix3x3 rotMatT = Math::Mat3Transpose(rotMat);

        Matrix3x3 temp = {};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    sum += rotMat.m[i * 3 + k] * m_inertiaTensor.m[k * 3 + j];
                }
                temp.m[i * 3 + j] = sum;
            }
        }

        Matrix3x3 result = {};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    sum += temp.m[i * 3 + k] * rotMatT.m[k * 3 + j];
                }
                result.m[i * 3 + j] = sum;
            }
        }

        return result;
    }

    Matrix3x3 RigidBody::GetWorldInverseInertiaTensor() const
    {
        Matrix3x3 rotMat = Math::Mat3FromQuat(m_rotation);
        Matrix3x3 rotMatT = Math::Mat3Transpose(rotMat);

        Matrix3x3 temp = {};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    sum += rotMat.m[i * 3 + k] * m_invInertiaTensor.m[k * 3 + j];
                }
                temp.m[i * 3 + j] = sum;
            }
        }

        Matrix3x3 result = {};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    sum += temp.m[i * 3 + k] * rotMatT.m[k * 3 + j];
                }
                result.m[i * 3 + j] = sum;
            }
        }

        return result;
    }

    Vector3 RigidBody::WorldToLocal(const Vector3& worldPoint) const
    {
        Vector3 local = worldPoint - m_position;
        Quaternion invRot = Math::QuatInverse(m_rotation);
        return invRot * local;
    }

    Vector3 RigidBody::LocalToWorld(const Vector3& localPoint) const
    {
        return m_position + m_rotation * localPoint;
    }

    Vector3 RigidBody::WorldToLocalVector(const Vector3& worldVector) const
    {
        Quaternion invRot = Math::QuatInverse(m_rotation);
        return invRot * worldVector;
    }

    Vector3 RigidBody::LocalToWorldVector(const Vector3& localVector) const
    {
        return m_rotation * localVector;
    }

    Vector3 RigidBody::GetPointVelocity(const Vector3& worldPoint) const
    {
        Vector3 r = worldPoint - m_position;
        return m_linearVelocity + Math::Cross(m_angularVelocity, r);
    }

    void RigidBody::ApplyForce(const Vector3& force, const Vector3& point)
    {
        if (m_isKinematic || m_mass <= 0.0f) return;

        m_forceAccumulator += force;
        Vector3 r = point - m_position;
        m_torqueAccumulator += Math::Cross(r, force);
        m_isAwake = true;
        m_sleepTimer = 0;
    }

    void RigidBody::ApplyTorque(const Vector3& torque)
    {
        if (m_isKinematic || m_mass <= 0.0f) return;

        m_torqueAccumulator += torque;
        m_isAwake = true;
        m_sleepTimer = 0;
    }

    void RigidBody::ApplyImpulse(const Vector3& impulse, const Vector3& point)
    {
        if (m_isKinematic || m_mass <= 0.0f) return;

        m_linearVelocity += impulse * m_invMass;
        Vector3 r = point - m_position;
        Vector3 angularImpulse = Math::Cross(r, impulse);
        m_angularVelocity += Math::Mat3MulVec3(GetWorldInverseInertiaTensor(), angularImpulse);
        m_isAwake = true;
        m_sleepTimer = 0;
    }

    void RigidBody::ClearForces()
    {
        m_forceAccumulator = Math::Vec3Zero();
        m_torqueAccumulator = Math::Vec3Zero();
    }

    void RigidBody::IntegrateLinear(float dt)
    {
        if (m_isKinematic || m_mass <= 0.0f) return;

        Vector3 acceleration = m_forceAccumulator * m_invMass;
        m_linearVelocity += acceleration * dt;
        m_linearVelocity *= (1.0f - m_linearDamping * dt);

        m_prevPosition = m_position;
        m_position += m_linearVelocity * dt;
        m_isDirty = true;
    }

    void RigidBody::IntegrateAngular(float dt)
    {
        if (m_isKinematic || m_mass <= 0.0f) return;

        Vector3 angularAcceleration = Math::Mat3MulVec3(GetWorldInverseInertiaTensor(), m_torqueAccumulator);
        m_angularVelocity += angularAcceleration * dt;
        m_angularVelocity *= (1.0f - m_angularDamping * dt);

        m_prevRotation = m_rotation;

        float angle = Math::Length(m_angularVelocity) * dt;
        if (angle > Math::EPSILON)
        {
            Vector3 axis = Math::Normalize(m_angularVelocity);
            Quaternion deltaRot = Math::QuatFromAngleAxis(angle, axis);
            m_rotation = Math::QuatNormalize(m_rotation * deltaRot);
        }
        m_isDirty = true;
    }

    void RigidBody::SetAwake(bool awake)
    {
        if (awake)
        {
            m_isAwake = true;
            m_sleepTimer = 0;
            m_sleepEnergy = 0.0f;
        }
        else
        {
            if (m_sleepThreshold <= 0.0f) return;
            m_isAwake = false;
            m_linearVelocity = Math::Vec3Zero();
            m_angularVelocity = Math::Vec3Zero();
            m_forceAccumulator = Math::Vec3Zero();
            m_torqueAccumulator = Math::Vec3Zero();
        }
    }

    bool RigidBody::CanCollideWith(const RigidBody* other) const
    {
        if (!other) return false;
        if (m_isSensor || other->m_isSensor) return true;
        if ((m_collisionGroup & other->m_collisionMask) == 0) return false;
        if ((other->m_collisionGroup & m_collisionMask) == 0) return false;
        return true;
    }

    void RigidBody::Update(float dt)
    {
        if (!m_isAwake) return;
        if (m_sleepThreshold <= 0.0f) return;

        float kineticEnergy = 0.5f * m_mass * Math::LengthSquared(m_linearVelocity);
        Vector3 angularMomentum = Math::Mat3MulVec3(GetWorldInertiaTensor(), m_angularVelocity);
        float rotationalEnergy = 0.5f * Math::Dot(m_angularVelocity, angularMomentum);
        float totalEnergy = kineticEnergy + rotationalEnergy;

        m_sleepEnergy = Math::Max(m_sleepEnergy * 0.95f, totalEnergy);

        float sleepThresholdSq = m_sleepThreshold * m_sleepThreshold * m_mass;
        if (m_sleepEnergy < sleepThresholdSq * 0.1f)
        {
            m_sleepTimer++;
            if (m_sleepTimer > 120)
            {
                m_isAwake = false;
                m_linearVelocity = Math::Vec3Zero();
                m_angularVelocity = Math::Vec3Zero();
            }
        }
        else
        {
            m_sleepTimer = 0;
        }
    }

    RigidBodyManager::RigidBodyManager()
        : m_nextBodyId(1)
    {
    }

    RigidBodyManager::~RigidBodyManager()
    {
        for (auto& pair : m_bodies)
        {
            delete pair.second;
        }
        m_bodies.clear();
    }

    uint32_t RigidBodyManager::CreateBody(const RigidBodyDesc& desc)
    {
        RigidBody* body = new RigidBody(desc);
        body->SetId(m_nextBodyId);
        m_bodies[m_nextBodyId] = body;
        return m_nextBodyId++;
    }

    void RigidBodyManager::DestroyBody(uint32_t bodyId)
    {
        auto it = m_bodies.find(bodyId);
        if (it != m_bodies.end())
        {
            delete it->second;
            m_bodies.erase(it);
        }
    }

    RigidBody* RigidBodyManager::GetBody(uint32_t bodyId)
    {
        auto it = m_bodies.find(bodyId);
        if (it != m_bodies.end())
        {
            return it->second;
        }
        return nullptr;
    }

    const RigidBody* RigidBodyManager::GetBody(uint32_t bodyId) const
    {
        auto it = m_bodies.find(bodyId);
        if (it != m_bodies.end())
        {
            return it->second;
        }
        return nullptr;
    }

    void RigidBodyManager::Update(float dt)
    {
        for (auto& pair : m_bodies)
        {
            pair.second->Update(dt);
        }
    }

    void RigidBodyManager::IntegrateAll(float dt)
    {
        for (auto& pair : m_bodies)
        {
            if (pair.second->IsAwake())
            {
                pair.second->IntegrateLinear(dt);
                pair.second->IntegrateAngular(dt);
            }
        }
    }

    void RigidBodyManager::ClearAllForces()
    {
        for (auto& pair : m_bodies)
        {
            pair.second->ClearForces();
        }
    }

    uint32_t RigidBodyManager::GetAwakeCount() const
    {
        uint32_t count = 0;
        for (const auto& pair : m_bodies)
        {
            if (pair.second->IsAwake())
            {
                count++;
            }
        }
        return count;
    }

    uint32_t RigidBodyManager::GetSleepingCount() const
    {
        uint32_t count = 0;
        for (const auto& pair : m_bodies)
        {
            if (!pair.second->IsAwake())
            {
                count++;
            }
        }
        return count;
    }

    uint32_t RigidBodyManager::GetCCDBodyCount() const
    {
        uint32_t count = 0;
        for (const auto& pair : m_bodies)
        {
            if (pair.second->IsCCDEnabled())
            {
                count++;
            }
        }
        return count;
    }

    void RigidBodyManager::IntegrateAwake(float dt)
    {
        for (auto& pair : m_bodies)
        {
            if (pair.second->IsAwake())
            {
                pair.second->IntegrateLinear(dt);
                pair.second->IntegrateAngular(dt);
            }
        }
    }

    void RigidBodyManager::WakeUpBody(uint32_t bodyId)
    {
        RigidBody* body = GetBody(bodyId);
        if (body)
        {
            body->SetAwake(true);
        }
    }

    void RigidBodyManager::WakeUpAll()
    {
        for (auto& pair : m_bodies)
        {
            pair.second->SetAwake(true);
        }
    }
}
