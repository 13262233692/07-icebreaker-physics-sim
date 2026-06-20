#pragma once

#include "IcePhysicsAPI.h"
#include "MathUtils.h"
#include <vector>
#include <unordered_map>

namespace IcePhysics
{
    class RigidBody
    {
    public:
        RigidBody(const RigidBodyDesc& desc);
        ~RigidBody();

        void IntegrateLinear(float dt);
        void IntegrateAngular(float dt);
        void ApplyForce(const Vector3& force, const Vector3& point);
        void ApplyTorque(const Vector3& torque);
        void ApplyImpulse(const Vector3& impulse, const Vector3& point);
        void ClearForces();

        Vector3 GetPosition() const { return m_position; }
        void SetPosition(const Vector3& pos) { m_position = pos; m_isDirty = true; }

        Quaternion GetRotation() const { return m_rotation; }
        void SetRotation(const Quaternion& rot) { m_rotation = Math::QuatNormalize(rot); m_isDirty = true; }

        Vector3 GetLinearVelocity() const { return m_linearVelocity; }
        void SetLinearVelocity(const Vector3& vel) { m_linearVelocity = vel; }

        Vector3 GetAngularVelocity() const { return m_angularVelocity; }
        void SetAngularVelocity(const Vector3& vel) { m_angularVelocity = vel; }

        float GetMass() const { return m_mass; }
        float GetInverseMass() const { return m_invMass; }
        void SetMass(float mass);

        Matrix3x3 GetInertiaTensor() const { return m_inertiaTensor; }
        Matrix3x3 GetInverseInertiaTensor() const { return m_invInertiaTensor; }
        void SetInertiaTensor(const Matrix3x3& tensor);

        Matrix3x3 GetWorldInertiaTensor() const;
        Matrix3x3 GetWorldInverseInertiaTensor() const;

        Vector3 WorldToLocal(const Vector3& worldPoint) const;
        Vector3 LocalToWorld(const Vector3& localPoint) const;
        Vector3 WorldToLocalVector(const Vector3& worldVector) const;
        Vector3 LocalToWorldVector(const Vector3& localVector) const;

        Vector3 GetPointVelocity(const Vector3& worldPoint) const;

        float GetRestitution() const { return m_restitution; }
        void SetRestitution(float restitution) { m_restitution = restitution; }

        float GetStaticFriction() const { return m_staticFriction; }
        void SetStaticFriction(float friction) { m_staticFriction = friction; }

        float GetDynamicFriction() const { return m_dynamicFriction; }
        void SetDynamicFriction(float friction) { m_dynamicFriction = friction; }

        float GetLinearDamping() const { return m_linearDamping; }
        void SetLinearDamping(float damping) { m_linearDamping = damping; }

        float GetAngularDamping() const { return m_angularDamping; }
        void SetAngularDamping(float damping) { m_angularDamping = damping; }

        bool IsKinematic() const { return m_isKinematic; }
        void SetKinematic(bool kinematic) { m_isKinematic = kinematic; }

        bool IsSensor() const { return m_isSensor; }
        void SetSensor(bool sensor) { m_isSensor = sensor; }

        bool IsAwake() const { return m_isAwake; }
        void SetAwake(bool awake) { m_isAwake = awake; }

        uint32_t GetUserData() const { return m_userData; }
        void SetUserData(uint32_t data) { m_userData = data; }

        uint32_t GetId() const { return m_id; }
        void SetId(uint32_t id) { m_id = id; }

        bool IsDirty() const { return m_isDirty; }
        void ClearDirty() { m_isDirty = false; }

        void Update(float dt);

    private:
        uint32_t m_id;
        Vector3 m_position;
        Quaternion m_rotation;
        Vector3 m_linearVelocity;
        Vector3 m_angularVelocity;
        Vector3 m_forceAccumulator;
        Vector3 m_torqueAccumulator;
        Vector3 m_prevPosition;
        Quaternion m_prevRotation;

        float m_mass;
        float m_invMass;
        Matrix3x3 m_inertiaTensor;
        Matrix3x3 m_invInertiaTensor;

        float m_restitution;
        float m_staticFriction;
        float m_dynamicFriction;
        float m_linearDamping;
        float m_angularDamping;

        bool m_isKinematic;
        bool m_isSensor;
        bool m_isAwake;
        bool m_isDirty;

        uint32_t m_userData;

        float m_sleepThreshold;
        uint32_t m_sleepTimer;
    };

    class RigidBodyManager
    {
    public:
        RigidBodyManager();
        ~RigidBodyManager();

        uint32_t CreateBody(const RigidBodyDesc& desc);
        void DestroyBody(uint32_t bodyId);
        RigidBody* GetBody(uint32_t bodyId);
        const RigidBody* GetBody(uint32_t bodyId) const;

        void Update(float dt);
        void IntegrateAll(float dt);
        void ClearAllForces();

        uint32_t GetActiveCount() const { return (uint32_t)m_bodies.size(); }
        uint32_t GetAwakeCount() const;

        const std::unordered_map<uint32_t, RigidBody*>& GetBodies() const { return m_bodies; }

    private:
        std::unordered_map<uint32_t, RigidBody*> m_bodies;
        uint32_t m_nextBodyId;
    };
}
