#include "plugin.h"
#include "common.h"
#include "CAutomobile.h"
#include "CVehicle.h"
#include "CCheat.h"
#include "CPad.h"
#include "CTimer.h"
#include "CVector.h"

#include <math.h>

using namespace plugin;

namespace BetterDrivingPhysicsSA {

    static const float BDP_PI = 3.14159265358979323846f;
    static const float BDP_DEG_TO_RAD = BDP_PI / 180.0f;

    static const int DRIVE_LAYOUT_UNKNOWN = 0;
    static const int DRIVE_LAYOUT_FWD = 1;
    static const int DRIVE_LAYOUT_RWD = 2;
    static const int DRIVE_LAYOUT_AWD = 3;

    struct RuntimeState {
        CVehicle* vehicle;
        float smoothedSteer;
    };

    static RuntimeState gState = { 0, 0.0f };

    static float ClampFloat(float value, float minValue, float maxValue) {
        if (value < minValue)
            return minValue;
        if (value > maxValue)
            return maxValue;
        return value;
    }

    static float SaturateFloat(float value) {
        return ClampFloat(value, 0.0f, 1.0f);
    }

    static float LerpFloat(float a, float b, float t) {
        return a + (b - a) * t;
    }

    static float SignFloat(float value) {
        if (value > 0.0f)
            return 1.0f;
        if (value < 0.0f)
            return -1.0f;
        return 0.0f;
    }

    static float DotProductSimple(const CVector& a, const CVector& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static CVector ScaledVector(const CVector& v, float s) {
        CVector out = v;
        out *= s;
        return out;
    }

    static int GetDriveLayout(CAutomobile* car) {
        unsigned char driveType;

        if (!car || !car->m_pHandlingData)
            return DRIVE_LAYOUT_UNKNOWN;

        driveType = car->m_pHandlingData->m_transmissionData.m_nDriveType;

        if (driveType == 'F')
            return DRIVE_LAYOUT_FWD;

        if (driveType == 'R')
            return DRIVE_LAYOUT_RWD;

        if (driveType == '4')
            return DRIVE_LAYOUT_AWD;

        return DRIVE_LAYOUT_UNKNOWN;
    }

    static bool IsPlayerDrivingVehicle(CVehicle* vehicle) {
        CPlayerPed* player;

        if (!vehicle)
            return false;

        player = FindPlayerPed(-1);
        if (!player)
            return false;

        return vehicle->m_pDriver == player;
    }

    static bool IsValidTargetVehicle(CVehicle* vehicle) {
        if (!vehicle)
            return false;

        if (vehicle->GetVehicleAppearance() != VEHICLE_APPEARANCE_AUTOMOBILE)
            return false;

        if (!IsPlayerDrivingVehicle(vehicle))
            return false;

        return true;
    }

    static void ResetState(CVehicle* vehicle) {
        if (gState.vehicle != vehicle) {
            gState.vehicle = vehicle;

            if (vehicle)
                gState.smoothedSteer = vehicle->m_fSteerAngle;
            else
                gState.smoothedSteer = 0.0f;
        }
    }

    static void ApplySteeringFilter(CAutomobile* car, float speed2D, float dt) {
        float steeringLockRad;
        float speedNorm;
        float lockScale;
        float allowedLock;
        float steerNow;
        float steerResponse;
        float blend;

        if (!car || !car->m_pHandlingData)
            return;

        steeringLockRad = car->m_pHandlingData->m_fSteeringLock * BDP_DEG_TO_RAD;
        speedNorm = SaturateFloat(speed2D / 0.90f);

        lockScale = LerpFloat(1.00f, 0.78f, speedNorm);
        allowedLock = steeringLockRad * lockScale;

        steerNow = ClampFloat(car->m_fSteerAngle, -allowedLock, allowedLock);

        blend = LerpFloat(0.40f, 0.20f, speedNorm);
        blend = SaturateFloat(blend * dt);

        gState.smoothedSteer = LerpFloat(gState.smoothedSteer, steerNow, blend);
        car->m_fSteerAngle = gState.smoothedSteer;
    }

    static void ApplySlipAssist(CAutomobile* car, float dt) {
        CPad* pad;
        CVector forward;
        CVector right;
        CVector sideCorrection;
        float localForward;
        float localSide;
        float absForward;
        float absSide;
        float speed2D;
        bool handbrake;
        bool braking;
        bool accelerating;
        bool reversing;
        float slipAngle;
        float startAssistAngle;
        float fullAssistAngle;
        float assist;
        float speedNorm;
        int layout;
        float yawAssist;
        float brakeAssist;
        float throttleRelax;
        float yawDamping;
        float slideSign;
        float yawSign;
        float maxYawRate;
        float sideTrim;
        float brakeStraighten;

        if (!car || !car->m_pHandlingData)
            return;

        if (car->m_nWheelsOnGround < 3)
            return;

        pad = CPad::GetPad(0);
        if (!pad)
            return;

        forward = car->GetForward();
        right = car->GetRight();

        localForward = DotProductSimple(car->m_vecMoveSpeed, forward);
        localSide = DotProductSimple(car->m_vecMoveSpeed, right);

        absForward = (float)fabs(localForward);
        absSide = (float)fabs(localSide);
        speed2D = car->m_vecMoveSpeed.Magnitude2D();

        if (speed2D < 0.08f)
            return;

        handbrake = (pad->GetHandBrake() != 0) || (car->bIsHandbrakeOn != 0);
        braking = (pad->GetBrake() > 0) || (car->m_fBreakPedal > 0.08f);
        accelerating = (pad->GetAccelerate() > 0) || (car->m_fGasPedal > 0.08f);
        reversing = (localForward < -0.02f);

        slipAngle = (float)atan2(absSide, absForward + 0.02f);

        startAssistAngle = 8.0f * BDP_DEG_TO_RAD;
        fullAssistAngle = 22.0f * BDP_DEG_TO_RAD;

        if (slipAngle <= startAssistAngle)
            return;

        assist = SaturateFloat((slipAngle - startAssistAngle) / (fullAssistAngle - startAssistAngle));
        speedNorm = SaturateFloat((speed2D - 0.10f) / 0.75f);
        assist *= speedNorm;

        if (assist <= 0.0f)
            return;

        layout = GetDriveLayout(car);

        yawAssist = 1.0f;
        brakeAssist = 1.0f;
        throttleRelax = 1.0f;

        if (layout == DRIVE_LAYOUT_FWD) {
            yawAssist = 1.08f;
            brakeAssist = 1.08f;
            throttleRelax = 0.95f;
        }
        else if (layout == DRIVE_LAYOUT_RWD) {
            yawAssist = 0.90f;
            brakeAssist = 0.96f;
            throttleRelax = 0.76f;
        }
        else if (layout == DRIVE_LAYOUT_AWD) {
            yawAssist = 1.00f;
            brakeAssist = 1.02f;
            throttleRelax = 0.86f;
        }

        if (handbrake)
            assist *= 0.25f;

        if (accelerating && !braking && !reversing)
            assist *= throttleRelax;

        yawDamping = (0.006f + 0.013f * assist) * yawAssist * dt;

        if (braking && !handbrake)
            yawDamping *= 1.20f * brakeAssist;

        slideSign = SignFloat(localSide);
        yawSign = SignFloat(car->m_vecTurnSpeed.z);

        if (slideSign != 0.0f && yawSign == slideSign) {
            car->m_vecTurnSpeed.z -= slideSign * yawDamping;
        }
        else {
            car->m_vecTurnSpeed.z *= (1.0f - 0.006f * assist * dt);
        }

        maxYawRate = LerpFloat(0.20f, 0.11f, speedNorm);
        car->m_vecTurnSpeed.z = ClampFloat(car->m_vecTurnSpeed.z, -maxYawRate, maxYawRate);

        if (!handbrake && slipAngle > (12.0f * BDP_DEG_TO_RAD)) {
            sideTrim = (0.0010f + 0.0030f * assist) * dt;

            if (accelerating && layout == DRIVE_LAYOUT_RWD)
                sideTrim *= 0.70f;

            if (braking)
                sideTrim *= 1.10f;

            sideCorrection = ScaledVector(right, localSide * sideTrim);
            car->m_vecMoveSpeed -= sideCorrection;
        }

        if (braking && !handbrake && speedNorm > 0.2f) {
            brakeStraighten = (0.0025f + 0.0060f * assist) * dt;
            car->m_vecTurnSpeed.z *= (1.0f - brakeStraighten);
        }
    }

    static void ProcessPlayerCar() {
        CVehicle* vehicle;
        CAutomobile* car;
        float dt;
        float speed2D;

        vehicle = FindPlayerVehicle(-1, false);

        if (!IsValidTargetVehicle(vehicle)) {
            ResetState(0);
            return;
        }

        if (CCheat::m_aCheatsActive[CHEAT_PERFECT_HANDLING]) {
            ResetState(vehicle);
            return;
        }

        car = (CAutomobile*)vehicle;

        ResetState(vehicle);

        dt = ClampFloat(CTimer::ms_fTimeStep, 0.25f, 3.0f);
        speed2D = car->m_vecMoveSpeed.Magnitude2D();

        ApplySteeringFilter(car, speed2D, dt);
        ApplySlipAssist(car, dt);
    }

    static void OnGameProcess() {
        ProcessPlayerCar();
    }

    class PluginMain {
    public:
        PluginMain() {
            Events::gameProcessEvent += OnGameProcess;
        }
    };

    static PluginMain gPluginMain;
}
