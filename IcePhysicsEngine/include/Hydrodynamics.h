#pragma once

#include "IcePhysicsAPI.h"
#include "MathUtils.h"

namespace IcePhysics
{
    class HydrodynamicSystem
    {
    public:
        HydrodynamicSystem();
        ~HydrodynamicSystem();

        void SetParams(const HydrodynamicParams* params);
        void GetParams(HydrodynamicParams* params) const;

        void SetEngineParams(const EngineThrustParams* params);
        void GetEngineParams(EngineThrustParams* params) const;

        void CalculateForces(const ShipState& state, Vector3& outForce, Vector3& outTorque);

        float CalculateTotalResistance(float speedKnots, float waterTemperature, float iceConcentration);

        float CalculateFrictionalResistance(float speedKnots, float waterTemperature);
        float CalculateWaveResistance(float speedKnots);
        float CalculateViscousResistance(float speedKnots);
        float CalculateAddedMassForce(const Vector3& acceleration);
        float CalculateIceResistance(float speedKnots, float iceConcentration);

        Vector3 CalculateThrustForce(float throttle, float speedKnots);
        Vector3 CalculateRudderForce(float rudderAngle, float speedKnots);
        Vector3 CalculateManeuveringForce(float rudderAngle, float speedKnots, float driftAngle);

        void CalculateMomentEquation(const Vector3& force, const Vector3& torque,
                                      const Vector3& position, const Quaternion& rotation,
                                      const Vector3& velocity, const Vector3& angularVelocity,
                                      Vector3& linearAcceleration, Vector3& angularAcceleration);

        void UpdateShipDynamics(ShipState& state, const Vector3& totalForce, const Vector3& totalTorque, float dt);

        float GetReynoldsNumber(float speedKnots, float waterTemperature) const;
        float GetWaterDensity(float waterTemperature) const;
        float GetWaterKinematicViscosity(float waterTemperature) const;

    private:
        HydrodynamicParams m_params;
        EngineThrustParams m_engineParams;

        float m_previousSpeed;
        Vector3 m_previousVelocity;
        float m_shipMass;

        Matrix3x3 m_addedMassMatrix;
        Matrix3x3 m_dampingMatrix;
        Matrix3x3 m_restoringMatrix;

        void ComputeAddedMassMatrix();
        void ComputeDampingMatrix();
        void ComputeRestoringMatrix();

        float CalculateFroudeNumber(float speedKnots) const;
        float CalculatePrismaticResistance(float froudeNumber) const;
        float CalculateWaveBreakingResistance(float froudeNumber) const;

        Vector3 CalculateHydrostaticForce(const Vector3& position, const Quaternion& rotation);
        Vector3 CalculateHydrostaticTorque(const Vector3& position, const Quaternion& rotation);

        float CalculateDisplacement() const;
        float CalculateMetacentricHeight() const;
    };

    class InertiaMomentumSolver
    {
    public:
        InertiaMomentumSolver();
        ~InertiaMomentumSolver();

        void SetShipMass(float mass) { m_mass = mass; }
        float GetShipMass() const { return m_mass; }

        void SetInertiaTensor(const Matrix3x3& tensor) { m_inertiaTensor = tensor; m_invInertiaTensor = Math::Mat3Inverse(tensor); }
        Matrix3x3 GetInertiaTensor() const { return m_inertiaTensor; }

        void SetCenterOfMass(const Vector3& com) { m_centerOfMass = com; }
        Vector3 GetCenterOfMass() const { return m_centerOfMass; }

        void IntegrateLinear(Vector3& position, Vector3& velocity, const Vector3& force, float dt);
        void IntegrateAngular(const Quaternion& rotation, Vector3& angularVelocity, const Vector3& torque, float dt);

        Vector3 GetLinearMomentum(const Vector3& velocity) const;
        Vector3 GetAngularMomentum(const Quaternion& rotation, const Vector3& angularVelocity) const;

        float GetKineticEnergy(const Vector3& velocity, const Quaternion& rotation, const Vector3& angularVelocity) const;
        float GetPotentialEnergy(const Vector3& position) const;

        void ApplyImpulse(Vector3& velocity, const Vector3& impulse);
        void ApplyAngularImpulse(const Quaternion& rotation, Vector3& angularVelocity, const Vector3& angularImpulse);

        void ConserveMomentum(Vector3& velocityA, Vector3& velocityB, float massA, float massB,
                              const Vector3& collisionNormal, float restitution);

    private:
        float m_mass;
        float m_invMass;
        Matrix3x3 m_inertiaTensor;
        Matrix3x3 m_invInertiaTensor;
        Vector3 m_centerOfMass;
        Vector3 m_gravity;
    };

    struct ShipDimensions
    {
        float lengthOverall;
        float lengthWaterline;
        float beam;
        float draft;
        float depth;
        float displacement;
        float blockCoefficient;
        float prismaticCoefficient;
        float midshipCoefficient;
        float waterplaneArea;
        float wettedSurfaceArea;
    };

    struct IceResistanceParams
    {
        float iceThickness;
        float iceStrength;
        float iceDensity;
        float frictionCoefficient;
        float bowAngle;
        float waterlineAngle;
    };

    class IceResistanceCalculator
    {
    public:
        IceResistanceCalculator();
        ~IceResistanceCalculator();

        void SetShipDimensions(const ShipDimensions& dims) { m_dimensions = dims; }
        void SetIceParams(const IceResistanceParams& params) { m_iceParams = params; }

        float CalculateIceBreakingResistance(float speedKnots);
        float CalculateIceBendingResistance(float speedKnots);
        float CalculateIceCrushingResistance(float speedKnots);
        float CalculateIceSubmergenceResistance(float speedKnots);

        float CalculateTotalIceResistance(float speedKnots, float iceConcentration);

        float CalculateMinimumSpeedForIceBreaking(float iceThickness);
        float CalculateMaximumIceThickness(float powerKW, float speedKnots);

    private:
        ShipDimensions m_dimensions;
        IceResistanceParams m_iceParams;

        float CalculateBendingMoment(float iceThickness, float contactWidth);
        float CalculateCrushingForce(float contactArea, float iceStrength);
        float CalculateContactArea(float iceThickness, float penetration);
    };
}
