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

    static float AbsFloat(float value) {
        return (float)fabs(value);
    }

    static float DotSimple(const CVector& a, const CVector& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static float SmoothStep01(float t) {
        t = SaturateFloat(t);
        return t * t * (3.0f - 2.0f * t);
    }

    static float RemapTo01(float value, float fromA, float fromB) {
        if (fromB <= fromA)
            return 0.0f;
        return SaturateFloat((value - fromA) / (fromB - fromA));
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
        float targetSteer;
        float steerResponse;
        float blend;

        if (!car || !car->m_pHandlingData)
            return;

        steeringLockRad = car->m_pHandlingData->m_fSteeringLock * BDP_DEG_TO_RAD;
        speedNorm = SaturateFloat(speed2D / 0.95f);

        /* keep low-speed maneuverability, trim twitch at speed */
        lockScale = LerpFloat(1.00f, 0.80f, speedNorm);
        allowedLock = steeringLockRad * lockScale;

        targetSteer = ClampFloat(car->m_fSteerAngle, -allowedLock, allowedLock);

        /* very light smoothing only */
        steerResponse = LerpFloat(0.48f, 0.22f, speedNorm);
        blend = SaturateFloat(steerResponse * dt);

        gState.smoothedSteer = LerpFloat(gState.smoothedSteer, targetSteer, blend);
        car->m_fSteerAngle = gState.smoothedSteer;
    }

    static void ApplyBetterDriving(CAutomobile* car, float dt) {
        CPad* pad;
        CVector forward;
        CVector right;

        float localForward;
        float localSide;
        float absForward;
        float absSide;
        float speed2D;
        float speedNorm;

        bool handbrake;
        bool braking;
        bool accelerating;
        bool reversing;

        float slipAngle;
        float slipStart;
        float slipPeak;
        float slipCurve;

        int layout;

        float yawTrimStrength;
        float sideTrimStrength;
        float brakeStabilityStrength;
        float throttleYawStrength;

        float slideSign;
        float yawSign;
        float steerInputNorm;
        float steerDirection;
        float throttleAmount;
        float brakeAmount;
        float maxYawRate;

        if (!car || !car->m_pHandlingData)
            return;

        if (car->m_nWheelsOnGround < 3)
            return;

        pad = CPad::GetPad(0);
        if (!pad)
            return;

        forward = car->GetForward();
        right = car->GetRight();

        localForward = DotSimple(car->m_vecMoveSpeed, forward);
        localSide = DotSimple(car->m_vecMoveSpeed, right);

        absForward = AbsFloat(localForward);
        absSide = AbsFloat(localSide);
        speed2D = car->m_vecMoveSpeed.Magnitude2D();

        if (speed2D < 0.06f)
            return;

        speedNorm = SaturateFloat((speed2D - 0.08f) / 0.85f);

        handbrake = (pad->GetHandBrake() != 0) || (car->bIsHandbrakeOn != 0);
        braking = (pad->GetBrake() > 0) || (car->m_fBreakPedal > 0.08f);
        accelerating = (pad->GetAccelerate() > 0) || (car->m_fGasPedal > 0.08f);
        reversing = (localForward < -0.02f);

        throttleAmount = SaturateFloat(car->m_fGasPedal);
        brakeAmount = SaturateFloat(car->m_fBreakPedal);

        /* core slip-angle measurement */
        slipAngle = (float)atan2(absSide, absForward + 0.02f);

        /*
            slip curve:
            - below slipStart: leave car alone
            - near slipPeak: strongest correction
            this avoids the "soap" effect
        */
        slipStart = 5.5f * BDP_DEG_TO_RAD;
        slipPeak = 19.0f * BDP_DEG_TO_RAD;
        slipCurve = SmoothStep01(RemapTo01(slipAngle, slipStart, slipPeak));

        if (slipCurve <= 0.0f)
            return;

        /* fade out most of the intervention at very low speeds */
        slipCurve *= speedNorm;

        if (slipCurve <= 0.0f)
            return;

        layout = GetDriveLayout(car);

        yawTrimStrength = 1.0f;
        sideTrimStrength = 1.0f;
        brakeStabilityStrength = 1.0f;
        throttleYawStrength = 1.0f;

        if (layout == DRIVE_LAYOUT_FWD) {
            yawTrimStrength = 1.08f;
            sideTrimStrength = 1.08f;
            brakeStabilityStrength = 1.08f;
            throttleYawStrength = 0.55f;
        }
        else if (layout == DRIVE_LAYOUT_RWD) {
            yawTrimStrength = 0.88f;
            sideTrimStrength = 0.82f;
            brakeStabilityStrength = 0.95f;
            throttleYawStrength = 1.15f;
        }
        else if (layout == DRIVE_LAYOUT_AWD) {
            yawTrimStrength = 0.98f;
            sideTrimStrength = 0.95f;
            brakeStabilityStrength = 1.02f;
            throttleYawStrength = 0.85f;
        }

        if (handbrake)
            slipCurve *= 0.22f;

        slideSign = SignFloat(localSide);
        yawSign = SignFloat(car->m_vecTurnSpeed.z);

        /*
            1) reduce the "toy car" instant yaw feel:
               trim overspeeding yaw when it matches the slide direction
        */
        {
            float yawTrim = (0.0045f + 0.0105f * slipCurve) * yawTrimStrength * dt;

            if (braking && !handbrake)
                yawTrim *= (1.10f + 0.15f * brakeAmount);

            if (slideSign != 0.0f && yawSign == slideSign) {
                car->m_vecTurnSpeed.z -= slideSign * yawTrim;
            }
            else {
                car->m_vecTurnSpeed.z *= (1.0f - 0.0040f * slipCurve * dt);
            }
        }

        /*
            2) mild slip cleanup only when already over the threshold
               keep this small so it doesn't feel like ice/soap
        */
        if (!handbrake) {
            float sideTrim = (0.0009f + 0.0032f * slipCurve) * sideTrimStrength * dt;

            if (accelerating && layout == DRIVE_LAYOUT_RWD)
                sideTrim *= 0.72f;

            if (braking)
                sideTrim *= 1.12f;

            car->m_vecMoveSpeed -= right * (localSide * sideTrim);
        }

        /*
            3) braking stability:
               stop the rear from doing dumb snap wiggles under braking
        */
        if (braking && !handbrake && speedNorm > 0.15f) {
            float straighten = (0.0020f + 0.0065f * slipCurve) * brakeStabilityStrength * dt;
            car->m_vecTurnSpeed.z *= (1.0f - straighten);
        }

        /*
            4) throttle yaw for RWD/AWD:
               this is the part SA often lacks and why cars can feel fake-FWD
               only add a little, only when moving forward, only while steering
        */
        steerInputNorm = SaturateFloat(AbsFloat(car->m_fSteerAngle) / (18.0f * BDP_DEG_TO_RAD));
        steerDirection = SignFloat(car->m_fSteerAngle);

        if (!handbrake && accelerating && !braking && !reversing && steerDirection != 0.0f) {
            float throttleYaw = 0.0f;

            throttleYaw =
                (0.0016f + 0.0060f * slipCurve) *
                throttleYawStrength *
                throttleAmount *
                steerInputNorm *
                speedNorm *
                dt;

            /*
                For RWD:
                add a bit of yaw into the turn under throttle.
                For FWD:
                much weaker so it keeps pulling itself straight more.
            */
            car->m_vecTurnSpeed.z += steerDirection * throttleYaw;
        }

        /*
            5) clamp total yaw so it never turns into nonsense
        */
        maxYawRate = LerpFloat(0.22f, 0.12f, speedNorm);

        if (handbrake)
            maxYawRate *= 1.35f;

        car->m_vecTurnSpeed.z = ClampFloat(car->m_vecTurnSpeed.z, -maxYawRate, maxYawRate);
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
        ApplyBetterDriving(car, dt);
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
