#pragma once

#include "IcePhysicsAPI.h"
#include "MathUtils.h"

namespace IcePhysics
{
    class RigidBody;

    class WindFieldSystem
    {
    public:
        WindFieldSystem();
        ~WindFieldSystem();

        void SetParams(const WindFieldParams* params);
        void GetParams(WindFieldParams* params) const;

        Vector3 GetWindVelocityAtPoint(const Vector3& position, float time) const;

        float GetWindSpeed() const { return m_params.windSpeed; }
        Vector3 GetWindDirection() const { return m_params.windDirection; }

        void CalculateWindForce(
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
        ) const;

    private:
        WindFieldParams m_params;
        float m_timeAccumulator;

        float CalculateWindPressure(float relativeSpeed) const;
        float CalculateDragCoefficient(float angleOfAttack, bool isLateral) const;
        Vector3 CalculateTurbulence(const Vector3& position, float time) const;
    };

    class OceanWaveSystem
    {
    public:
        OceanWaveSystem();
        ~OceanWaveSystem();

        void SetParams(const OceanCurrentParams* params);
        void GetParams(OceanCurrentParams* params) const;

        float GetWaveHeight(const Vector3& position, float time) const;
        Vector3 GetWaveNormal(const Vector3& position, float time) const;
        Vector3 GetWaveVelocity(const Vector3& position, float time) const;

        Vector3 GetOceanCurrent(const Vector3& position, float time) const;

        float GetTideHeight(float time) const;

        void GetMarineState(MarineEnvironmentState* state, float time) const;

    private:
        OceanCurrentParams m_params;

        float GerstnerWaveHeight(
            float x, float z, float time,
            float amplitude, float frequency,
            float dirX, float dirZ, float phase
        ) const;

        Vector3 GerstnerWaveVelocity(
            float x, float z, float time,
            float amplitude, float frequency,
            float dirX, float dirZ, float phase
        ) const;
    };

    class MarineEnvironmentSystem
    {
    public:
        MarineEnvironmentSystem();
        ~MarineEnvironmentSystem();

        void SetWindParams(const WindFieldParams* params) { m_windField.SetParams(params); }
        void GetWindParams(WindFieldParams* params) const { m_windField.GetParams(params); }

        void SetOceanParams(const OceanCurrentParams* params) { m_oceanWaves.SetParams(params); }
        void GetOceanParams(OceanCurrentParams* params) const { m_oceanWaves.GetParams(params); }

        const WindFieldSystem& GetWindField() const { return m_windField; }
        const OceanWaveSystem& GetOceanWaves() const { return m_oceanWaves; }

        void GetEnvironmentState(MarineEnvironmentState* state, float time) const;

        void ApplyOceanForcesToFragment(
            RigidBody* body,
            float fragmentRadius,
            float fragmentMass,
            float time
        );

        void Update(float dt);

        float GetSimulationTime() const { return m_simulationTime; }

    private:
        WindFieldSystem m_windField;
        OceanWaveSystem m_oceanWaves;
        float m_simulationTime;
    };
}
