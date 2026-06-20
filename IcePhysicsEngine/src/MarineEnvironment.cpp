#include "MarineEnvironment.h"
#include "RigidBody.h"
#include <cmath>
#include <algorithm>
namespace IcePhysics
{
    WindFieldSystem::WindFieldSystem()
        : m_timeAccumulator(0.0f)
    {
        m_params.windDirection = Vector3(1.0f, 0.0f, 0.0f);
        m_params.windSpeed = 5.0f;
        m_params.windGustFactor = 1.2f;
        m_params.turbulenceIntensity = 0.1f;
        m_params.airDensity = 1.225f;
        m_params.boundaryLayerHeight = 500.0f;
    }

    WindFieldSystem::~WindFieldSystem()
    {
    }

    void WindFieldSystem::SetParams(const WindFieldParams* params)
    {
        if (params)
        {
            m_params = *params;
            if (Math::Length(m_params.windDirection) < 0.001f)
            {
                m_params.windDirection = Vector3(1.0f, 0.0f, 0.0f);
            }
            else
            {
                m_params.windDirection = Math::Normalize(m_params.windDirection);
            }
            m_params.windSpeed = Math::Max(0.0f, m_params.windSpeed);
            m_params.airDensity = Math::Max(0.1f, m_params.airDensity);
        }
    }

    void WindFieldSystem::GetParams(WindFieldParams* params) const
    {
        if (params)
        {
            *params = m_params;
        }
    }

    Vector3 WindFieldSystem::GetWindVelocityAtPoint(const Vector3& position, float time) const
    {
        Vector3 baseWind = m_params.windDirection * m_params.windSpeed;

        float heightFactor = 1.0f;
        if (position.y < m_params.boundaryLayerHeight && position.y > 0.0f)
        {
            heightFactor = powf(position.y / m_params.boundaryLayerHeight, 0.143f);
        }
        else if (position.y <= 0.0f)
        {
            heightFactor = 0.0f;
        }

        float gustFactor = 1.0f + 0.5f * (m_params.windGustFactor - 1.0f) *
            (sinf(time * 0.5f) + sinf(time * 1.3f + 1.5f) + sinf(time * 2.7f + 0.8f)) / 3.0f;
        gustFactor = Math::Clamp(gustFactor, 1.0f / m_params.windGustFactor, m_params.windGustFactor);

        Vector3 turbulence = CalculateTurbulence(position, time);

        return baseWind * heightFactor * gustFactor + turbulence;
    }

    Vector3 WindFieldSystem::CalculateTurbulence(const Vector3& position, float time) const
    {
        if (m_params.turbulenceIntensity <= 0.0f)
        {
            return Math::Vec3Zero();
        }

        float intensity = m_params.turbulenceIntensity * m_params.windSpeed;

        float nx = sinf(position.x * 0.1f + time * 1.7f) * cosf(position.z * 0.15f + time * 2.3f);
        float ny = sinf(position.y * 0.2f + time * 3.1f) * cosf(position.x * 0.1f + time * 1.1f);
        float nz = cosf(position.z * 0.1f + time * 2.9f) * sinf(position.x * 0.05f + time * 0.7f);

        return Vector3(nx, ny, nz) * intensity;
    }

    float WindFieldSystem::CalculateWindPressure(float relativeSpeed) const
    {
        return 0.5f * m_params.airDensity * relativeSpeed * relativeSpeed;
    }

    float WindFieldSystem::CalculateDragCoefficient(float angleOfAttack, bool isLateral) const
    {
        float rad = angleOfAttack * Math::DEG_TO_RAD;
        float cosA = cosf(rad);
        float sinA = sinf(rad);

        if (isLateral)
        {
            return 0.8f + 0.4f * sinA * sinA;
        }
        else
        {
            return 0.5f * cosA * cosA + 0.9f * sinA * sinA;
        }
    }

    void WindFieldSystem::CalculateWindForce(
        const Vector3& shipPosition,
        const Quaternion& shipRotation,
        const Vector3& shipVelocity,
        float hullLength,
        float hullBeam,
        float hullDraft,
        float superstructureHeight,
        float superstructureFrontalArea,
        float superstructureLateralArea,
        float time,
        Vector3& outForce,
        Vector3& outTorque,
        float& outLateralDriftForce,
        float& outYawMoment,
        float& outDriftAngleDeg,
        float& outRelativeWindAngleDeg
    ) const
    {
        outForce = Math::Vec3Zero();
        outTorque = Math::Vec3Zero();
        outLateralDriftForce = 0.0f;
        outYawMoment = 0.0f;
        outDriftAngleDeg = 0.0f;
        outRelativeWindAngleDeg = 0.0f;

        Vector3 windVel = GetWindVelocityAtPoint(
            shipPosition + Vector3(0.0f, superstructureHeight * 0.5f, 0.0f),
            time
        );

        Vector3 relativeWind = windVel - shipVelocity;
        float relWindSpeed = Math::Length(relativeWind);

        if (relWindSpeed < 0.1f)
        {
            return;
        }

        Vector3 relWindDir = relativeWind / relWindSpeed;

        Vector3 shipForward = shipRotation * Vector3(0.0f, 0.0f, 1.0f);
        Vector3 shipRight = shipRotation * Vector3(1.0f, 0.0f, 0.0f);
        Vector3 shipUp = shipRotation * Vector3(0.0f, 1.0f, 0.0f);

        shipForward.y = 0.0f;
        shipForward = Math::Normalize(shipForward);
        shipRight.y = 0.0f;
        shipRight = Math::Normalize(shipRight);

        float windForwardDot = Math::Dot(relWindDir, shipForward);
        float windRightDot = Math::Dot(relWindDir, shipRight);
        outRelativeWindAngleDeg = atan2f(windRightDot, windForwardDot) * Math::RAD_TO_DEG;

        float pressure = CalculateWindPressure(relWindSpeed);

        float frontalArea = superstructureFrontalArea > 0.0f ? superstructureFrontalArea :
            hullBeam * (superstructureHeight + hullDraft * 0.3f);
        float lateralArea = superstructureLateralArea > 0.0f ? superstructureLateralArea :
            hullLength * (superstructureHeight * 0.6f + hullDraft * 0.2f);

        float CdFrontal = CalculateDragCoefficient(outRelativeWindAngleDeg, false);
        float CdLateral = CalculateDragCoefficient(outRelativeWindAngleDeg, true);

        float absCosAngle = fabsf(windForwardDot);
        float absSinAngle = fabsf(windRightDot);

        float forceFrontal = pressure * frontalArea * CdFrontal * absCosAngle * absCosAngle;
        float forceLateral = pressure * lateralArea * CdLateral * absSinAngle * absSinAngle;

        Vector3 forceDir = shipForward * windForwardDot + shipRight * windRightDot;
        forceDir.y = 0.0f;
        if (Math::Length(forceDir) > 0.001f)
        {
            forceDir = Math::Normalize(forceDir);
        }

        float totalForceMag = forceFrontal + forceLateral;
        outForce = forceDir * totalForceMag;

        float lateralComponent = Math::Dot(outForce, shipRight);
        outLateralDriftForce = lateralComponent;

        float yawArmLength = hullLength * 0.35f;
        float yawMomentFactor = windRightDot * windForwardDot;
        outYawMoment = outLateralDriftForce * yawArmLength * yawMomentFactor * 2.0f;

        float verticalCenter = hullDraft + superstructureHeight * 0.6f;
        outTorque = shipRight * outYawMoment + shipUp * (outForce.x * verticalCenter);

        if (relWindSpeed > 1.0f)
        {
            float driftFactor = Math::Clamp(relWindSpeed / 20.0f, 0.0f, 1.0f);
            outDriftAngleDeg = outRelativeWindAngleDeg * 0.15f * driftFactor;
        }
    }
    OceanWaveSystem::OceanWaveSystem()
    {
        m_params.currentDirection = Vector3(1.0f, 0.0f, 0.0f);
        m_params.currentSpeed = 0.5f;
        m_params.waveAmplitude = 1.0f;
        m_params.waveFrequency = 0.5f;
        m_params.waveDirectionX = 1.0f;
        m_params.waveDirectionZ = 0.0f;
        m_params.swellAmplitude = 0.5f;
        m_params.swellFrequency = 0.2f;
        m_params.swellDirectionX = 0.707f;
        m_params.swellDirectionZ = 0.707f;
        m_params.tideHeight = 1.5f;
        m_params.tidePeriod = 12.42f * 3600.0f;
    }

    OceanWaveSystem::~OceanWaveSystem()
    {
    }

    void OceanWaveSystem::SetParams(const OceanCurrentParams* params)
    {
        if (params)
        {
            m_params = *params;

            float len = sqrtf(m_params.waveDirectionX * m_params.waveDirectionX +
                              m_params.waveDirectionZ * m_params.waveDirectionZ);
            if (len > 0.001f)
            {
                m_params.waveDirectionX /= len;
                m_params.waveDirectionZ /= len;
            }
            else
            {
                m_params.waveDirectionX = 1.0f;
                m_params.waveDirectionZ = 0.0f;
            }

            len = sqrtf(m_params.swellDirectionX * m_params.swellDirectionX +
                        m_params.swellDirectionZ * m_params.swellDirectionZ);
            if (len > 0.001f)
            {
                m_params.swellDirectionX /= len;
                m_params.swellDirectionZ /= len;
            }
            else
            {
                m_params.swellDirectionX = 0.707f;
                m_params.swellDirectionZ = 0.707f;
            }

            if (Math::Length(m_params.currentDirection) < 0.001f)
            {
                m_params.currentDirection = Vector3(1.0f, 0.0f, 0.0f);
            }
            else
            {
                m_params.currentDirection = Math::Normalize(m_params.currentDirection);
            }
        }
    }

    void OceanWaveSystem::GetParams(OceanCurrentParams* params) const
    {
        if (params)
        {
            *params = m_params;
        }
    }

    float OceanWaveSystem::GerstnerWaveHeight(
        float x, float z, float time,
        float amplitude, float frequency,
        float dirX, float dirZ, float phase
    ) const
    {
        float dot = dirX * x + dirZ * z;
        return amplitude * sinf(frequency * dot + frequency * time + phase);
    }

    Vector3 OceanWaveSystem::GerstnerWaveVelocity(
        float x, float z, float time,
        float amplitude, float frequency,
        float dirX, float dirZ, float phase
    ) const
    {
        float dot = dirX * x + dirZ * z;
        float angle = frequency * dot + frequency * time + phase;
        float circularSpeed = amplitude * frequency;

        return Vector3(
            -dirX * circularSpeed * cosf(angle),
            circularSpeed * sinf(angle),
            -dirZ * circularSpeed * cosf(angle)
        );
    }

    float OceanWaveSystem::GetWaveHeight(const Vector3& position, float time) const
    {
        float wave1 = GerstnerWaveHeight(
            position.x, position.z, time,
            m_params.waveAmplitude, m_params.waveFrequency,
            m_params.waveDirectionX, m_params.waveDirectionZ, 0.0f
        );

        float wave2 = GerstnerWaveHeight(
            position.x, position.z, time,
            m_params.waveAmplitude * 0.5f, m_params.waveFrequency * 1.7f,
            -m_params.waveDirectionZ, m_params.waveDirectionX, 1.3f
        );

        float wave3 = GerstnerWaveHeight(
            position.x, position.z, time,
            m_params.waveAmplitude * 0.3f, m_params.waveFrequency * 2.3f,
            m_params.waveDirectionX * 0.5f + m_params.waveDirectionZ * 0.5f,
            m_params.waveDirectionZ * 0.5f - m_params.waveDirectionX * 0.5f, 2.7f
        );

        float swell = GerstnerWaveHeight(
            position.x, position.z, time,
            m_params.swellAmplitude, m_params.swellFrequency,
            m_params.swellDirectionX, m_params.swellDirectionZ, 0.5f
        );

        float tide = GetTideHeight(time);

        return wave1 + wave2 + wave3 + swell + tide;
    }

    Vector3 OceanWaveSystem::GetWaveNormal(const Vector3& position, float time) const
    {
        float eps = 0.1f;
        float h = GetWaveHeight(position, time);
        float hx = GetWaveHeight(Vector3(position.x + eps, position.y, position.z), time);
        float hz = GetWaveHeight(Vector3(position.x, position.y, position.z + eps), time);

        Vector3 tangentX(eps, hx - h, 0.0f);
        Vector3 tangentZ(0.0f, hz - h, eps);

        Vector3 normal = Math::Cross(tangentZ, tangentX);
        return Math::Normalize(normal);
    }

    Vector3 OceanWaveSystem::GetWaveVelocity(const Vector3& position, float time) const
    {
        Vector3 vel1 = GerstnerWaveVelocity(
            position.x, position.z, time,
            m_params.waveAmplitude, m_params.waveFrequency,
            m_params.waveDirectionX, m_params.waveDirectionZ, 0.0f
        );

        Vector3 vel2 = GerstnerWaveVelocity(
            position.x, position.z, time,
            m_params.swellAmplitude, m_params.swellFrequency,
            m_params.swellDirectionX, m_params.swellDirectionZ, 0.5f
        );

        return vel1 + vel2;
    }

    Vector3 OceanWaveSystem::GetOceanCurrent(const Vector3& position, float time) const
    {
        Vector3 baseCurrent = m_params.currentDirection * m_params.currentSpeed;

        float swirl = 0.05f * m_params.currentSpeed;
        float swirlX = swirl * sinf(position.x * 0.01f + time * 0.1f);
        float swirlZ = swirl * cosf(position.z * 0.01f + time * 0.1f);

        return Vector3(
            baseCurrent.x + swirlX,
            0.0f,
            baseCurrent.z + swirlZ
        );
    }

    float OceanWaveSystem::GetTideHeight(float time) const
    {
        float omega = 2.0f * Math::PI / m_params.tidePeriod;
        return m_params.tideHeight * sinf(omega * time);
    }

    void OceanWaveSystem::GetMarineState(MarineEnvironmentState* state, float time) const
    {
        if (!state) return;

        state->windVelocity = Vector3(0.0f, 0.0f, 0.0f);
        state->currentVelocity = GetOceanCurrent(Vector3(0.0f, 0.0f, 0.0f), time);
        state->waveHeight = 2.0f * (m_params.waveAmplitude + m_params.swellAmplitude);
        state->wavePeriod = m_params.waveFrequency > 0.0f ? 2.0f * Math::PI / m_params.waveFrequency : 0.0f;
        state->waveDirection = atan2f(m_params.waveDirectionZ, m_params.waveDirectionX) * Math::RAD_TO_DEG;

        float significantWaveHeight = state->waveHeight;
        if (significantWaveHeight < 0.1f) state->seaState = 0.0f;
        else if (significantWaveHeight < 0.5f) state->seaState = 1.0f;
        else if (significantWaveHeight < 1.25f) state->seaState = 2.0f;
        else if (significantWaveHeight < 2.5f) state->seaState = 3.0f;
        else if (significantWaveHeight < 4.0f) state->seaState = 4.0f;
        else if (significantWaveHeight < 6.0f) state->seaState = 5.0f;
        else if (significantWaveHeight < 9.0f) state->seaState = 6.0f;
        else state->seaState = 7.0f;

        state->visibility = 10000.0f;
    }

    MarineEnvironmentSystem::MarineEnvironmentSystem()
        : m_simulationTime(0.0f)
    {
    }

    MarineEnvironmentSystem::~MarineEnvironmentSystem()
    {
    }

    void MarineEnvironmentSystem::GetEnvironmentState(MarineEnvironmentState* state, float time) const
    {
        m_oceanWaves.GetMarineState(state, time);
    }

    void MarineEnvironmentSystem::ApplyOceanForcesToFragment(
        RigidBody* body,
        float fragmentRadius,
        float fragmentMass,
        float time
    )
    {
        if (!body || body->IsKinematic() || fragmentMass <= 0.0f)
        {
            return;
        }

        Vector3 position = body->GetPosition();
        Vector3 currentVel = m_oceanWaves.GetOceanCurrent(position, time);
        Vector3 waveVel = m_oceanWaves.GetWaveVelocity(position, time);

        Vector3 bodyVel = body->GetLinearVelocity();
        Vector3 relVel = currentVel + waveVel * 0.3f - bodyVel;

        float dragCoeff = 1.2f;
        float area = Math::PI * fragmentRadius * fragmentRadius;
        float waterDensity = 1025.0f;

        float relSpeed = Math::Length(relVel);
        if (relSpeed < 0.001f)
        {
            body->SetLinearVelocity(currentVel * 0.1f + body->GetLinearVelocity() * 0.9f);
            return;
        }

        Vector3 dragDir = relVel / relSpeed;
        Vector3 dragForce = dragDir * 0.5f * waterDensity * dragCoeff * area * relSpeed * relSpeed;

        float maxForce = fragmentMass * 2.0f;
        if (Math::Length(dragForce) > maxForce)
        {
            dragForce = Math::Normalize(dragForce) * maxForce;
        }

        body->ApplyForce(dragForce, position);

        Vector3 waveNormal = m_oceanWaves.GetWaveNormal(position, time);
        Vector3 torqueAxis = Math::Cross(Vector3(0.0f, 1.0f, 0.0f), waveNormal);
        float waveHeight = m_oceanWaves.GetWaveHeight(position, time);

        if (waveHeight > 0.1f)
        {
            Vector3 waveTorque = torqueAxis * fragmentMass * 0.5f * Math::Min(fabsf(waveHeight), 1.0f);
            body->ApplyTorque(waveTorque);
        }

        float buoyancyArea = Math::PI * fragmentRadius * fragmentRadius;
        float submerged = Math::Clamp(-position.y + waveHeight, 0.0f, fragmentRadius * 2.0f);
        if (submerged > 0.0f && position.y < 0.5f)
        {
            float buoyancyForce = waterDensity * 9.81f * buoyancyArea * submerged * 0.5f;
            buoyancyForce = Math::Min(buoyancyForce, fragmentMass * 10.0f);
            body->ApplyForce(Vector3(0.0f, buoyancyForce, 0.0f), position);
        }
    }

    void MarineEnvironmentSystem::Update(float dt)
    {
        m_simulationTime += dt;
    }
}
