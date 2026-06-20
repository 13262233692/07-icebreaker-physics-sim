#include "../include/Hydrodynamics.h"
#include <cmath>

namespace IcePhysics
{
    const float KNOTS_TO_MPS = 0.514444f;
    const float GRAVITY = 9.81f;
    const float SEA_WATER_DENSITY_15C = 1025.0f;
    const float FRESH_WATER_DENSITY_4C = 1000.0f;

    HydrodynamicSystem::HydrodynamicSystem()
        : m_previousSpeed(0.0f)
        , m_previousVelocity(Math::Vec3Zero())
        , m_shipMass(0.0f)
    {
        m_params.waterDensity = SEA_WATER_DENSITY_15C;
        m_params.dragCoefficient = 0.08f;
        m_params.liftCoefficient = 0.02f;
        m_params.addedMassCoefficient = 0.05f;
        m_params.viscousDragCoefficient = 1.1f;
        m_params.waveDragCoefficient = 0.5f;
        m_params.hullWettedArea = 1000.0f;
        m_params.hullLength = 100.0f;
        m_params.hullBeam = 20.0f;
        m_params.hullDraft = 8.0f;
        m_params.prismaticCoefficient = 0.65f;
        m_params.blockCoefficient = 0.75f;
        m_params.midshipCoefficient = 0.95f;

        m_engineParams.totalPower = 20000.0f;
        m_engineParams.propellerEfficiency = 0.6f;
        m_engineParams.hullEfficiency = 0.9f;
        m_engineParams.maxRudderAngle = 35.0f;
        m_engineParams.rudderArea = 50.0f;
        m_engineParams.rudderAspectRatio = 2.0f;
        m_engineParams.waterDensity = SEA_WATER_DENSITY_15C;

        m_addedMassMatrix = Math::Mat3Identity();
        m_dampingMatrix = Math::Mat3Identity();
        m_restoringMatrix = Math::Mat3Identity();

        ComputeAddedMassMatrix();
        ComputeDampingMatrix();
        ComputeRestoringMatrix();
    }

    HydrodynamicSystem::~HydrodynamicSystem()
    {
    }

    void HydrodynamicSystem::SetParams(const HydrodynamicParams* params)
    {
        if (params)
        {
            m_params = *params;
            ComputeAddedMassMatrix();
            ComputeDampingMatrix();
            ComputeRestoringMatrix();
        }
    }

    void HydrodynamicSystem::GetParams(HydrodynamicParams* params) const
    {
        if (params)
        {
            *params = m_params;
        }
    }

    void HydrodynamicSystem::SetEngineParams(const EngineThrustParams* params)
    {
        if (params)
        {
            m_engineParams = *params;
        }
    }

    void HydrodynamicSystem::GetEngineParams(EngineThrustParams* params) const
    {
        if (params)
        {
            *params = m_engineParams;
        }
    }

    void HydrodynamicSystem::CalculateForces(const ShipState& state, Vector3& outForce, Vector3& outTorque)
    {
        float speedMPS = state.speedKnots * KNOTS_TO_MPS;
        Vector3 forwardDir = state.rotation * Math::Vec3Forward();
        Vector3 rightDir = state.rotation * Math::Vec3Right();
        Vector3 upDir = state.rotation * Math::Vec3Up();

        Vector3 thrustForce = CalculateThrustForce(state.throttle, state.speedKnots);
        Vector3 rudderForce = CalculateRudderForce(state.rudderAngle, state.speedKnots);
        Vector3 maneuveringForce = CalculateManeuveringForce(state.rudderAngle, state.speedKnots, state.driftAngle);

        Vector3 acceleration = (state.linearVelocity - m_previousVelocity) / Math::Max(0.016f, 0.016f);
        Vector3 addedMassForceVec = -Math::Mat3MulVec3(m_addedMassMatrix, acceleration);

        float totalResistance = CalculateTotalResistance(state.speedKnots, 15.0f, 0.0f);
        Vector3 resistanceForce = forwardDir * (-totalResistance);

        Vector3 hydrostaticForce = CalculateHydrostaticForce(state.position, state.rotation);
        Vector3 hydrostaticTorque = CalculateHydrostaticTorque(state.position, state.rotation);

        outForce = thrustForce + rudderForce + maneuveringForce + addedMassForceVec + resistanceForce + hydrostaticForce;

        float rudderLever = m_params.hullLength * 0.45f;
        Vector3 rudderTorque = Math::Cross(upDir * rudderLever, rudderForce);
        Vector3 maneuveringTorque = Math::Cross(upDir * rudderLever * 0.5f, maneuveringForce);

        outTorque = rudderTorque + maneuveringTorque + hydrostaticTorque;

        m_previousVelocity = state.linearVelocity;
        m_previousSpeed = speedMPS;
    }

    float HydrodynamicSystem::CalculateTotalResistance(float speedKnots, float waterTemperature, float iceConcentration)
    {
        if (speedKnots <= 0.0f) return 0.0f;

        float frictionalResistance = CalculateFrictionalResistance(speedKnots, waterTemperature);
        float waveResistance = CalculateWaveResistance(speedKnots);
        float viscousResistance = CalculateViscousResistance(speedKnots);
        float iceResistance = CalculateIceResistance(speedKnots, iceConcentration);

        float totalResistance = frictionalResistance + waveResistance + viscousResistance + iceResistance;

        return Math::Max(0.0f, totalResistance);
    }

    float HydrodynamicSystem::CalculateFrictionalResistance(float speedKnots, float waterTemperature)
    {
        if (speedKnots <= 0.0f) return 0.0f;

        float speedMPS = speedKnots * KNOTS_TO_MPS;
        float reynoldsNumber = GetReynoldsNumber(speedKnots, waterTemperature);

        if (reynoldsNumber < 1000.0f) return 0.0f;

        float Cf = 0.075f / Math::Pow(Math::Log10(reynoldsNumber) - 2.0f, 2.0f);

        float formFactor = 1.0f + (1.5f * Math::Pow(m_params.blockCoefficient, 1.5f)) / 
                            Math::Pow(m_params.hullLength / m_params.hullBeam, 0.5f) -
                            (0.1f * m_params.hullDraft / m_params.hullBeam);

        float rho = GetWaterDensity(waterTemperature);
        float S = m_params.hullWettedArea;

        float resistance = 0.5f * rho * speedMPS * speedMPS * S * Cf * formFactor;

        return resistance;
    }

    float HydrodynamicSystem::CalculateWaveResistance(float speedKnots)
    {
        if (speedKnots <= 0.0f) return 0.0f;

        float froudeNumber = CalculateFroudeNumber(speedKnots);

        if (froudeNumber < 0.1f) return 0.0f;

        float prismaticResistance = CalculatePrismaticResistance(froudeNumber);
        float waveBreakingResistance = CalculateWaveBreakingResistance(froudeNumber);

        float speedMPS = speedKnots * KNOTS_TO_MPS;
        float displacement = CalculateDisplacement();

        float waveResistance = (prismaticResistance + waveBreakingResistance) * 
                               m_params.waterDensity * GRAVITY * displacement * 
                               Math::Pow(froudeNumber, 2.0f);

        return waveResistance;
    }

    float HydrodynamicSystem::CalculateViscousResistance(float speedKnots)
    {
        if (speedKnots <= 0.0f) return 0.0f;

        float speedMPS = speedKnots * KNOTS_TO_MPS;

        float roughnessDrag = 0.0004f * m_params.hullWettedArea * 
                              0.5f * m_params.waterDensity * speedMPS * speedMPS;

        float appendageDrag = 0.1f * CalculateFrictionalResistance(speedKnots, 15.0f);

        float pressureDrag = m_params.viscousDragCoefficient * m_params.hullBeam * m_params.hullDraft *
                             0.5f * m_params.waterDensity * speedMPS * speedMPS *
                             Math::Pow(1.0f - m_params.midshipCoefficient, 2.0f);

        return roughnessDrag + appendageDrag + pressureDrag;
    }

    float HydrodynamicSystem::CalculateAddedMassForce(const Vector3& acceleration)
    {
        Vector3 force = -Math::Mat3MulVec3(m_addedMassMatrix, acceleration);
        return Math::Length(force);
    }

    float HydrodynamicSystem::CalculateIceResistance(float speedKnots, float iceConcentration)
    {
        if (speedKnots <= 0.0f || iceConcentration <= 0.0f) return 0.0f;

        IceResistanceCalculator iceCalc;

        ShipDimensions dims;
        dims.lengthOverall = m_params.hullLength;
        dims.lengthWaterline = m_params.hullLength * 0.95f;
        dims.beam = m_params.hullBeam;
        dims.draft = m_params.hullDraft;
        dims.depth = m_params.hullDraft * 1.5f;
        dims.displacement = CalculateDisplacement();
        dims.blockCoefficient = m_params.blockCoefficient;
        dims.prismaticCoefficient = m_params.prismaticCoefficient;
        dims.midshipCoefficient = m_params.midshipCoefficient;
        dims.waterplaneArea = m_params.hullLength * m_params.hullBeam * 0.8f;
        dims.wettedSurfaceArea = m_params.hullWettedArea;

        IceResistanceParams iceParams;
        iceParams.iceThickness = 1.0f;
        iceParams.iceStrength = 500000.0f;
        iceParams.iceDensity = 900.0f;
        iceParams.frictionCoefficient = 0.1f;
        iceParams.bowAngle = 25.0f;
        iceParams.waterlineAngle = 15.0f;

        iceCalc.SetShipDimensions(dims);
        iceCalc.SetIceParams(iceParams);

        float totalIceResistance = iceCalc.CalculateTotalIceResistance(speedKnots, iceConcentration);

        return totalIceResistance;
    }

    Vector3 HydrodynamicSystem::CalculateThrustForce(float throttle, float speedKnots)
    {
        throttle = Math::Clamp(throttle, -1.0f, 1.0f);

        if (Math::Abs(throttle) < 0.01f) return Math::Vec3Zero();

        float effectivePower = m_engineParams.totalPower * Math::Abs(throttle) *
                               m_engineParams.propellerEfficiency * m_engineParams.hullEfficiency;

        float speedMPS = Math::Max(0.5f, speedKnots * KNOTS_TO_MPS);

        float thrust = effectivePower / speedMPS;

        if (throttle < 0.0f)
        {
            thrust *= 0.6f;
        }

        Vector3 forwardDir = Math::Vec3Forward();
        return forwardDir * thrust * Math::Sign(throttle);
    }

    Vector3 HydrodynamicSystem::CalculateRudderForce(float rudderAngle, float speedKnots)
    {
        if (Math::Abs(rudderAngle) < 0.1f || speedKnots < 0.5f) return Math::Vec3Zero();

        rudderAngle = Math::Clamp(rudderAngle, -m_engineParams.maxRudderAngle, m_engineParams.maxRudderAngle);

        float rudderAngleRad = rudderAngle * Math::DEG_TO_RAD;
        float speedMPS = speedKnots * KNOTS_TO_MPS;

        float aspectRatio = m_engineParams.rudderAspectRatio;
        float Cl = (2.0f * Math::PI * aspectRatio * rudderAngleRad) / 
                   (aspectRatio + 2.0f);

        float lift = 0.5f * m_engineParams.waterDensity * speedMPS * speedMPS *
                     m_engineParams.rudderArea * Cl;

        Vector3 rightDir = Math::Vec3Right();
        return rightDir * (-lift * Math::Sign(rudderAngle));
    }

    Vector3 HydrodynamicSystem::CalculateManeuveringForce(float rudderAngle, float speedKnots, float driftAngle)
    {
        if (speedKnots < 0.5f) return Math::Vec3Zero();

        float speedMPS = speedKnots * KNOTS_TO_MPS;
        float driftAngleRad = driftAngle * Math::DEG_TO_RAD;

        float hullLiftCoeff = Math::Sin(2.0f * driftAngleRad) * 0.8f;

        float hullLift = 0.5f * m_params.waterDensity * speedMPS * speedMPS *
                         m_params.hullLength * m_params.hullDraft * hullLiftCoeff;

        float dragCoeff = 0.5f * (1.0f - Math::Cos(2.0f * driftAngleRad));
        float hullDrag = 0.5f * m_params.waterDensity * speedMPS * speedMPS *
                         m_params.hullBeam * m_params.hullDraft * dragCoeff;

        Vector3 forwardDir = Math::Vec3Forward();
        Vector3 rightDir = Math::Vec3Right();

        Vector3 force = rightDir * hullLift - forwardDir * hullDrag;

        return force;
    }

    void HydrodynamicSystem::CalculateMomentEquation(const Vector3& force, const Vector3& torque,
                                                    const Vector3& position, const Quaternion& rotation,
                                                    const Vector3& velocity, const Vector3& angularVelocity,
                                                    Vector3& linearAcceleration, Vector3& angularAcceleration)
    {
        float totalMass = m_shipMass + m_addedMassMatrix.m[0];

        if (totalMass <= 0.0f)
        {
            linearAcceleration = Math::Vec3Zero();
        }
        else
        {
            linearAcceleration = force / totalMass;
        }

        Matrix3x3 totalInertia = m_addedMassMatrix;
        totalInertia.m[0] += m_shipMass * 0.01f;
        totalInertia.m[4] += m_shipMass * 0.01f;
        totalInertia.m[8] += m_shipMass * 0.01f;

        Matrix3x3 invInertia = Math::Mat3Inverse(totalInertia);

        Vector3 gyroscopicTorque = Math::Cross(
            Math::Mat3MulVec3(totalInertia, angularVelocity),
            angularVelocity
        );

        angularAcceleration = Math::Mat3MulVec3(invInertia, torque - gyroscopicTorque);
    }

    void HydrodynamicSystem::UpdateShipDynamics(ShipState& state, const Vector3& totalForce, const Vector3& totalTorque, float dt)
    {
        if (dt <= 0.0f) return;

        Vector3 linearAcceleration, angularAcceleration;
        CalculateMomentEquation(totalForce, totalTorque, state.position, state.rotation,
                                state.linearVelocity, state.angularVelocity,
                                linearAcceleration, angularAcceleration);

        state.linearVelocity += linearAcceleration * dt;
        state.angularVelocity += angularAcceleration * dt;

        state.position += state.linearVelocity * dt;

        float angle = Math::Length(state.angularVelocity) * dt;
        if (angle > Math::EPSILON)
        {
            Vector3 axis = Math::Normalize(state.angularVelocity);
            Quaternion deltaRot = Math::QuatFromAngleAxis(angle, axis);
            state.rotation = Math::QuatNormalize(state.rotation * deltaRot);
        }

        float speedMPS = Math::Length(state.linearVelocity);
        state.speedKnots = speedMPS / KNOTS_TO_MPS;

        Vector3 euler = Math::QuatToEuler(state.rotation);
        state.headingDegrees = Math::Clamp(euler.y, -180.0f, 180.0f);

        Vector3 forward = state.rotation * Math::Vec3Forward();
        Vector3 velDir = Math::Normalize(state.linearVelocity);
        float dot = Math::Dot(forward, velDir);
        float cross = Math::Cross(forward, velDir).y;
        state.driftAngle = Math::Atan2(cross, dot) * Math::RAD_TO_DEG;
    }

    float HydrodynamicSystem::GetReynoldsNumber(float speedKnots, float waterTemperature) const
    {
        float speedMPS = speedKnots * KNOTS_TO_MPS;
        float nu = GetWaterKinematicViscosity(waterTemperature);

        if (nu < Math::EPSILON) return 0.0f;

        return (speedMPS * m_params.hullLength) / nu;
    }

    float HydrodynamicSystem::GetWaterDensity(float waterTemperature) const
    {
        float t = Math::Clamp(waterTemperature, 0.0f, 40.0f);

        float density = 1000.0f - 
                        0.00019549f * Math::Pow(t - 4.0f, 2.0f) -
                        0.0000058f * Math::Pow(t - 4.0f, 3.0f);

        return density;
    }

    float HydrodynamicSystem::GetWaterKinematicViscosity(float waterTemperature) const
    {
        float t = Math::Clamp(waterTemperature, 0.0f, 40.0f);

        float nu = 1.787e-6f * Math::Exp(-0.035f * t);

        return Math::Max(nu, 1.0e-7f);
    }

    void HydrodynamicSystem::ComputeAddedMassMatrix()
    {
        float L = m_params.hullLength;
        float B = m_params.hullBeam;
        float T = m_params.hullDraft;
        float Cb = m_params.blockCoefficient;

        float displacement = L * B * T * Cb;
        m_shipMass = displacement * m_params.waterDensity;

        float kxx = 0.3f * B;
        float kyy = 0.25f * L;
        float kzz = 0.25f * L;

        float m11 = 0.05f * m_shipMass;
        float m22 = 0.4f * m_shipMass;
        float m33 = 0.9f * m_shipMass;
        float m44 = 0.15f * m_shipMass * kxx * kxx;
        float m55 = 0.1f * m_shipMass * kyy * kyy;
        float m66 = 0.05f * m_shipMass * kzz * kzz;

        m_addedMassMatrix = Math::Mat3Identity();
        m_addedMassMatrix.m[0] = m11;
        m_addedMassMatrix.m[4] = m22;
        m_addedMassMatrix.m[8] = m33;
    }

    void HydrodynamicSystem::ComputeDampingMatrix()
    {
        float L = m_params.hullLength;
        float B = m_params.hullBeam;
        float T = m_params.hullDraft;
        float speedMPS = Math::Max(1.0f, m_previousSpeed);

        float linearDamping = 0.5f * m_params.waterDensity * speedMPS * B * T;
        float angularDamping = 0.5f * m_params.waterDensity * speedMPS * L * L * B * 0.1f;

        m_dampingMatrix = Math::Mat3Identity();
        m_dampingMatrix.m[0] = linearDamping * 0.1f;
        m_dampingMatrix.m[4] = linearDamping * 0.5f;
        m_dampingMatrix.m[8] = linearDamping * 0.8f;
    }

    void HydrodynamicSystem::ComputeRestoringMatrix()
    {
        float GM = CalculateMetacentricHeight();
        float displacement = CalculateDisplacement();
        float B = m_params.hullBeam;
        float L = m_params.hullLength;

        float rollRestoring = m_params.waterDensity * GRAVITY * displacement * GM;
        float pitchRestoring = m_params.waterDensity * GRAVITY * displacement * GM * 1.5f;
        float heaveRestoring = m_params.waterDensity * GRAVITY * L * B;

        m_restoringMatrix = Math::Mat3Identity();
        m_restoringMatrix.m[0] = heaveRestoring;
        m_restoringMatrix.m[4] = rollRestoring;
        m_restoringMatrix.m[8] = pitchRestoring;
    }

    float HydrodynamicSystem::CalculateFroudeNumber(float speedKnots) const
    {
        float speedMPS = speedKnots * KNOTS_TO_MPS;
        float sqrtGL = Math::Sqrt(GRAVITY * m_params.hullLength);

        if (sqrtGL < Math::EPSILON) return 0.0f;

        return speedMPS / sqrtGL;
    }

    float HydrodynamicSystem::CalculatePrismaticResistance(float froudeNumber) const
    {
        if (froudeNumber < 0.1f) return 0.0f;

        float Cp = m_params.prismaticCoefficient;

        float Cw = 0.0f;

        if (froudeNumber < 0.3f)
        {
            Cw = 0.1f * Math::Pow(froudeNumber, 3.0f);
        }
        else if (froudeNumber < 0.45f)
        {
            float x = (froudeNumber - 0.3f) / 0.15f;
            float peak = 0.4f * Math::Exp(-Math::Pow(Cp - 0.65f, 2.0f) / 0.02f);
            Cw = peak * Math::Sin(Math::PI * x);
        }
        else if (froudeNumber < 0.6f)
        {
            float x = (froudeNumber - 0.45f) / 0.15f;
            Cw = 0.25f * (1.0f - x) + 0.05f * x;
        }
        else
        {
            Cw = 0.05f * Math::Exp(-(froudeNumber - 0.6f) * 2.0f);
        }

        return Cw * 0.5f;
    }

    float HydrodynamicSystem::CalculateWaveBreakingResistance(float froudeNumber) const
    {
        if (froudeNumber < 0.35f) return 0.0f;

        float B = m_params.hullBeam;
        float T = m_params.hullDraft;
        float L = m_params.hullLength;
        float Cb = m_params.blockCoefficient;

        float bowSharpness = (B / T) * (1.0f - Cb);

        float Cwb = 0.0f;

        if (froudeNumber > 0.4f)
        {
            Cwb = 0.001f * bowSharpness * Math::Pow(froudeNumber - 0.35f, 2.5f);
        }

        return Math::Min(Cwb, 0.1f);
    }

    Vector3 HydrodynamicSystem::CalculateHydrostaticForce(const Vector3& position, const Quaternion& rotation)
    {
        float displacement = CalculateDisplacement();
        float buoyancy = m_params.waterDensity * GRAVITY * displacement;

        float draft = m_params.hullDraft + position.y;
        if (draft < 0.0f) draft = 0.0f;

        float waterplaneArea = m_params.hullLength * m_params.hullBeam * 0.8f;
        float additionalBuoyancy = m_params.waterDensity * GRAVITY * waterplaneArea * Math::Max(0.0f, -position.y);

        Vector3 euler = Math::QuatToEuler(rotation);
        float heelAngleRad = euler.x * Math::DEG_TO_RAD;
        float trimAngleRad = euler.z * Math::DEG_TO_RAD;

        float heelCorrection = Math::Cos(heelAngleRad) * Math::Cos(trimAngleRad);

        Vector3 upDir = Math::Vec3Up();
        Vector3 force = upDir * (buoyancy + additionalBuoyancy) * heelCorrection;

        return force;
    }

    Vector3 HydrodynamicSystem::CalculateHydrostaticTorque(const Vector3& position, const Quaternion& rotation)
    {
        Vector3 euler = Math::QuatToEuler(rotation);
        float rollAngleRad = euler.x * Math::DEG_TO_RAD;
        float pitchAngleRad = euler.z * Math::DEG_TO_RAD;

        float GM = CalculateMetacentricHeight();
        float displacement = CalculateDisplacement();

        float rollRestoringTorque = -m_params.waterDensity * GRAVITY * displacement * GM * Math::Sin(rollAngleRad);
        float pitchRestoringTorque = -m_params.waterDensity * GRAVITY * displacement * GM * 1.5f * Math::Sin(pitchAngleRad);

        Vector3 torque;
        torque.x = rollRestoringTorque;
        torque.y = 0.0f;
        torque.z = pitchRestoringTorque;

        return torque;
    }

    float HydrodynamicSystem::CalculateDisplacement() const
    {
        return m_params.hullLength * m_params.hullBeam * m_params.hullDraft * m_params.blockCoefficient;
    }

    float HydrodynamicSystem::CalculateMetacentricHeight() const
    {
        float B = m_params.hullBeam;
        float T = m_params.hullDraft;
        float Cb = m_params.blockCoefficient;
        float L = m_params.hullLength;

        float I_T = Math::Pow(B, 3.0f) * L / 12.0f;

        float waterplaneArea = L * B * 0.8f;
        float BM_T = I_T / (L * B * T * Cb);

        float KB = T * (0.5f - 0.1f * Cb);

        float KG = T * 1.2f;

        float GM = KB + BM_T - KG;

        return Math::Max(GM, 0.1f);
    }

    InertiaMomentumSolver::InertiaMomentumSolver()
        : m_mass(0.0f)
        , m_invMass(0.0f)
        , m_inertiaTensor(Math::Mat3Identity())
        , m_invInertiaTensor(Math::Mat3Identity())
        , m_centerOfMass(Math::Vec3Zero())
        , m_gravity(Math::Vec3(0.0f, -GRAVITY, 0.0f))
    {
    }

    InertiaMomentumSolver::~InertiaMomentumSolver()
    {
    }

    void InertiaMomentumSolver::IntegrateLinear(Vector3& position, Vector3& velocity, const Vector3& force, float dt)
    {
        if (m_mass <= 0.0f || dt <= 0.0f) return;

        Vector3 totalForce = force + m_gravity * m_mass;

        Vector3 acceleration = totalForce * m_invMass;

        velocity += acceleration * dt;

        position += velocity * dt;
    }

    void InertiaMomentumSolver::IntegrateAngular(const Quaternion& rotation, Vector3& angularVelocity, const Vector3& torque, float dt)
    {
        if (m_mass <= 0.0f || dt <= 0.0f) return;

        Matrix3x3 rotMat = Math::Mat3FromQuat(rotation);
        Matrix3x3 rotMatT = Math::Mat3Transpose(rotMat);

        Matrix3x3 worldInertia = {};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    sum += rotMat.m[i * 3 + k] * m_inertiaTensor.m[k * 3 + j];
                }
                worldInertia.m[i * 3 + j] = sum;
            }
        }

        Matrix3x3 worldInvInertia = {};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    sum += rotMat.m[i * 3 + k] * m_invInertiaTensor.m[k * 3 + j];
                }
                worldInvInertia.m[i * 3 + j] = sum;
            }
        }

        Vector3 angularMomentum = Math::Mat3MulVec3(worldInertia, angularVelocity);
        Vector3 gyroscopicTorque = Math::Cross(angularVelocity, angularMomentum);

        Vector3 totalTorque = torque - gyroscopicTorque;

        Vector3 angularAcceleration = Math::Mat3MulVec3(worldInvInertia, totalTorque);

        angularVelocity += angularAcceleration * dt;
    }

    Vector3 InertiaMomentumSolver::GetLinearMomentum(const Vector3& velocity) const
    {
        return velocity * m_mass;
    }

    Vector3 InertiaMomentumSolver::GetAngularMomentum(const Quaternion& rotation, const Vector3& angularVelocity) const
    {
        Matrix3x3 rotMat = Math::Mat3FromQuat(rotation);
        Matrix3x3 rotMatT = Math::Mat3Transpose(rotMat);

        Matrix3x3 worldInertia = {};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    sum += rotMat.m[i * 3 + k] * m_inertiaTensor.m[k * 3 + j];
                }
                worldInertia.m[i * 3 + j] = sum;
            }
        }

        Matrix3x3 worldInertiaFinal = {};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    sum += worldInertia.m[i * 3 + k] * rotMatT.m[k * 3 + j];
                }
                worldInertiaFinal.m[i * 3 + j] = sum;
            }
        }

        return Math::Mat3MulVec3(worldInertiaFinal, angularVelocity);
    }

    float InertiaMomentumSolver::GetKineticEnergy(const Vector3& velocity, const Quaternion& rotation, const Vector3& angularVelocity) const
    {
        float translationalKE = 0.5f * m_mass * Math::LengthSquared(velocity);

        Vector3 angularMomentum = GetAngularMomentum(rotation, angularVelocity);
        float rotationalKE = 0.5f * Math::Dot(angularMomentum, angularVelocity);

        return translationalKE + rotationalKE;
    }

    float InertiaMomentumSolver::GetPotentialEnergy(const Vector3& position) const
    {
        return m_mass * GRAVITY * (position.y - m_centerOfMass.y);
    }

    void InertiaMomentumSolver::ApplyImpulse(Vector3& velocity, const Vector3& impulse)
    {
        if (m_mass <= 0.0f) return;

        velocity += impulse * m_invMass;
    }

    void InertiaMomentumSolver::ApplyAngularImpulse(const Quaternion& rotation, Vector3& angularVelocity, const Vector3& angularImpulse)
    {
        if (m_mass <= 0.0f) return;

        Matrix3x3 rotMat = Math::Mat3FromQuat(rotation);

        Matrix3x3 worldInvInertia = {};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    sum += rotMat.m[i * 3 + k] * m_invInertiaTensor.m[k * 3 + j];
                }
                worldInvInertia.m[i * 3 + j] = sum;
            }
        }

        angularVelocity += Math::Mat3MulVec3(worldInvInertia, angularImpulse);
    }

    void InertiaMomentumSolver::ConserveMomentum(Vector3& velocityA, Vector3& velocityB, float massA, float massB,
                                                 const Vector3& collisionNormal, float restitution)
    {
        if (massA <= 0.0f || massB <= 0.0f) return;

        Vector3 relVel = velocityA - velocityB;
        float velAlongNormal = Math::Dot(relVel, collisionNormal);

        if (velAlongNormal > 0.0f) return;

        float e = Math::Clamp(restitution, 0.0f, 1.0f);

        float j = -(1.0f + e) * velAlongNormal / (1.0f / massA + 1.0f / massB);

        Vector3 impulse = collisionNormal * j;

        velocityA += impulse / massA;
        velocityB -= impulse / massB;
    }

    IceResistanceCalculator::IceResistanceCalculator()
    {
        m_dimensions.lengthOverall = 100.0f;
        m_dimensions.lengthWaterline = 95.0f;
        m_dimensions.beam = 20.0f;
        m_dimensions.draft = 8.0f;
        m_dimensions.depth = 12.0f;
        m_dimensions.displacement = 10000.0f;
        m_dimensions.blockCoefficient = 0.75f;
        m_dimensions.prismaticCoefficient = 0.65f;
        m_dimensions.midshipCoefficient = 0.95f;
        m_dimensions.waterplaneArea = 1600.0f;
        m_dimensions.wettedSurfaceArea = 1000.0f;

        m_iceParams.iceThickness = 1.0f;
        m_iceParams.iceStrength = 500000.0f;
        m_iceParams.iceDensity = 900.0f;
        m_iceParams.frictionCoefficient = 0.1f;
        m_iceParams.bowAngle = 25.0f;
        m_iceParams.waterlineAngle = 15.0f;
    }

    IceResistanceCalculator::~IceResistanceCalculator()
    {
    }

    float IceResistanceCalculator::CalculateIceBreakingResistance(float speedKnots)
    {
        if (speedKnots <= 0.0f || m_iceParams.iceThickness <= 0.0f) return 0.0f;

        float speedMPS = speedKnots * KNOTS_TO_MPS;
        float h = m_iceParams.iceThickness;
        float B = m_dimensions.beam;
        float alpha = m_iceParams.bowAngle * Math::DEG_TO_RAD;
        float sigma = m_iceParams.iceStrength;

        float contactWidth = B / Math::Cos(alpha);

        float bendingMoment = CalculateBendingMoment(h, contactWidth);

        float requiredMoment = sigma * Math::Pow(h, 2.0f) * contactWidth / 6.0f;

        float breakingResistance = 0.0f;

        if (speedMPS > 0.5f)
        {
            float dynamicFactor = 1.0f + 0.5f * Math::Min(speedMPS / 5.0f, 1.0f);
            breakingResistance = dynamicFactor * requiredMoment / (2.0f * h);
        }
        else
        {
            breakingResistance = 0.8f * requiredMoment / (2.0f * h);
        }

        return breakingResistance;
    }

    float IceResistanceCalculator::CalculateIceBendingResistance(float speedKnots)
    {
        if (speedKnots <= 0.0f || m_iceParams.iceThickness <= 0.0f) return 0.0f;

        float speedMPS = speedKnots * KNOTS_TO_MPS;
        float h = m_iceParams.iceThickness;
        float B = m_dimensions.beam;
        float L = m_dimensions.lengthWaterline;
        float alpha = m_iceParams.bowAngle * Math::DEG_TO_RAD;
        float E = 5.0e9f;
        float nu = 0.3f;
        float rho_i = m_iceParams.iceDensity;
        float rho_w = SEA_WATER_DENSITY_15C;

        float D = E * Math::Pow(h, 3.0f) / (12.0f * (1.0f - nu * nu));
        float k = rho_w * GRAVITY;
        float lambda0 = Math::Pow(4.0f * D / k, 0.25f);

        float flexuralStrength = 0.5f * m_iceParams.iceStrength;

        float contactWidth = B / Math::Cos(alpha);

        float maxDeflection = flexuralStrength * Math::Pow(lambda0, 2.0f) / (E * h);

        float penetration = Math::Min(maxDeflection * 0.5f, h * 0.3f);

        float contactArea = CalculateContactArea(h, penetration);

        float bendingForce = 0.5f * k * contactArea * Math::Pow(penetration, 2.0f) / h;

        float speedFactor = 1.0f + 0.3f * Math::Sin(Math::Min(speedMPS / 3.0f, 1.0f) * Math::HALF_PI);

        float bendingResistance = bendingForce * speedFactor / Math::Cos(alpha);

        return bendingResistance;
    }

    float IceResistanceCalculator::CalculateIceCrushingResistance(float speedKnots)
    {
        if (speedKnots <= 0.0f || m_iceParams.iceThickness <= 0.0f) return 0.0f;

        float speedMPS = speedKnots * KNOTS_TO_MPS;
        float h = m_iceParams.iceThickness;
        float alpha = m_iceParams.bowAngle * Math::DEG_TO_RAD;
        float B = m_dimensions.beam;

        float penetration = Math::Min(h * 0.2f, 0.3f);
        float contactArea = CalculateContactArea(h, penetration);

        float crushingForce = CalculateCrushingForce(contactArea, m_iceParams.iceStrength);

        float strainRate = speedMPS / Math::Max(penetration, 0.01f);
        float strainRateFactor = 1.0f + 0.2f * Math::Log10(Math::Max(strainRate, 1.0f));

        float speedFactor = 1.0f + 0.4f * Math::Min(speedMPS / 10.0f, 1.0f);

        float crushingResistance = crushingForce * strainRateFactor * speedFactor / Math::Cos(alpha);

        return crushingResistance;
    }

    float IceResistanceCalculator::CalculateIceSubmergenceResistance(float speedKnots)
    {
        if (speedKnots <= 0.0f || m_iceParams.iceThickness <= 0.0f) return 0.0f;

        float speedMPS = speedKnots * KNOTS_TO_MPS;
        float h = m_iceParams.iceThickness;
        float B = m_dimensions.beam;
        float T = m_dimensions.draft;
        float alpha = m_iceParams.waterlineAngle * Math::DEG_TO_RAD;
        float mu = m_iceParams.frictionCoefficient;
        float rho_i = m_iceParams.iceDensity;
        float rho_w = SEA_WATER_DENSITY_15C;

        float iceDraft = h * (1.0f - rho_i / rho_w);

        float submergedWidth = Math::Min(B, 2.0f * iceDraft / Math::Tan(alpha));
        float submergedArea = submergedWidth * iceDraft;

        float buoyancyForce = rho_w * GRAVITY * submergedArea * h * 0.5f;

        float velocityHead = 0.5f * rho_w * speedMPS * speedMPS;
        float dynamicPressure = velocityHead * submergedArea;

        float normalForce = buoyancyForce + dynamicPressure * 0.3f;

        float frictionForce = mu * normalForce;

        float submergenceResistance = normalForce * Math::Sin(alpha) + frictionForce * Math::Cos(alpha);

        return submergenceResistance;
    }

    float IceResistanceCalculator::CalculateTotalIceResistance(float speedKnots, float iceConcentration)
    {
        if (speedKnots <= 0.0f || iceConcentration <= 0.0f) return 0.0f;

        iceConcentration = Math::Clamp(iceConcentration, 0.0f, 1.0f);

        float breakingResistance = CalculateIceBreakingResistance(speedKnots);
        float bendingResistance = CalculateIceBendingResistance(speedKnots);
        float crushingResistance = CalculateIceCrushingResistance(speedKnots);
        float submergenceResistance = CalculateIceSubmergenceResistance(speedKnots);

        float h = m_iceParams.iceThickness;
        float speedMPS = speedKnots * KNOTS_TO_MPS;

        float modeFactor = 1.0f;

        if (speedMPS < 1.0f && h < 0.5f)
        {
            modeFactor = 0.7f;
        }
        else if (speedMPS > 3.0f && h > 1.0f)
        {
            modeFactor = 1.3f;
        }

        float totalResistance = (breakingResistance + bendingResistance + 
                                 crushingResistance + submergenceResistance) * 
                                 iceConcentration * modeFactor;

        return Math::Max(0.0f, totalResistance);
    }

    float IceResistanceCalculator::CalculateMinimumSpeedForIceBreaking(float iceThickness)
    {
        if (iceThickness <= 0.0f) return 0.0f;

        float h = iceThickness;
        float sigma = m_iceParams.iceStrength;
        float B = m_dimensions.beam;
        float alpha = m_iceParams.bowAngle * Math::DEG_TO_RAD;
        float Cb = m_dimensions.blockCoefficient;

        float requiredEnergy = sigma * Math::Pow(h, 2.5f) * B / (Math::Sqrt(GRAVITY) * Math::Cos(alpha));

        float displacement = m_dimensions.displacement;
        float shipMass = displacement * SEA_WATER_DENSITY_15C;

        float minSpeedMPS = Math::Sqrt(2.0f * requiredEnergy / shipMass);

        float minSpeedKnots = minSpeedMPS / KNOTS_TO_MPS;

        return Math::Max(minSpeedKnots, 2.0f);
    }

    float IceResistanceCalculator::CalculateMaximumIceThickness(float powerKW, float speedKnots)
    {
        if (powerKW <= 0.0f || speedKnots <= 0.0f) return 0.0f;

        float speedMPS = speedKnots * KNOTS_TO_MPS;
        float powerW = powerKW * 1000.0f;

        float maxResistance = powerW / speedMPS;

        float h = 0.0f;
        float dh = 0.01f;
        const int maxIterations = 100;

        for (int i = 0; i < maxIterations; i++)
        {
            h += dh;

            IceResistanceParams tempParams = m_iceParams;
            tempParams.iceThickness = h;

            IceResistanceCalculator tempCalc;
            tempCalc.SetShipDimensions(m_dimensions);
            tempCalc.SetIceParams(tempParams);

            float resistance = tempCalc.CalculateTotalIceResistance(speedKnots, 1.0f);

            if (resistance > maxResistance)
            {
                h -= dh;
                break;
            }
        }

        return Math::Max(h, 0.0f);
    }

    float IceResistanceCalculator::CalculateBendingMoment(float iceThickness, float contactWidth)
    {
        float h = iceThickness;
        float L = contactWidth;
        float sigma = m_iceParams.iceStrength;

        float plasticSectionModulus = L * h * h / 4.0f;

        return sigma * plasticSectionModulus;
    }

    float IceResistanceCalculator::CalculateCrushingForce(float contactArea, float iceStrength)
    {
        float confinementFactor = 1.5f;
        return confinementFactor * iceStrength * contactArea;
    }

    float IceResistanceCalculator::CalculateContactArea(float iceThickness, float penetration)
    {
        float h = iceThickness;
        float B = m_dimensions.beam;
        float alpha = m_iceParams.bowAngle * Math::DEG_TO_RAD;

        float contactLength = penetration / Math::Sin(alpha);
        float contactWidth = B + 2.0f * penetration / Math::Tan(alpha);

        return contactLength * contactWidth;
    }
}
