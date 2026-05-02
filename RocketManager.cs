using System;
using System.Collections.Generic;
using System.Reflection;
using SFS.Parts;
using SFS.Parts.Modules;
using SFS.UI;
using SFS.World;
using SFS.WorldBase;
using UnityEngine;

namespace RevolutionlessAutopilot
{
    /// <summary>
    /// Centralises low-level rocket control operations.
    /// Holds rocket state only and does not own mission/ascent state.
    /// </summary>
    public class RocketManager
    {
        private Rocket rocket;
        private double cachedMaxAcceleration = 0;
        private double lastStageFireWorldTime = double.NegativeInfinity;
        private readonly HashSet<Stage> stagingAttempted = new HashSet<Stage>();

        public float PitchTolerance = 2.0f;
        public float AngularVelocityLimit = 4.0f;
        public float ThrottleSnapThreshold = 0.01f;
        public double StageCooldown = 1.0;

        public bool Debug = true;

        public RocketManager(Rocket rocket)
        {
            this.rocket = rocket;
        }

        public void SetRocket(Rocket rocket)
        {
            this.rocket = rocket;
        }

        public void ResetStagingHistory()
        {
            stagingAttempted.Clear();
            lastStageFireWorldTime = double.NegativeInfinity;
        }

        public void SetThrottle(float percent)
        {
            if (rocket == null) return;

            float effective = Mathf.Clamp01(percent);
            if (effective < ThrottleSnapThreshold) effective = 0f;

            float oldPct = rocket.throttle.throttlePercent.Value;
            bool oldOn = rocket.throttle.throttleOn.Value;

            rocket.throttle.throttlePercent.Value = effective;
            rocket.throttle.throttleOn.Value = effective > 0f;

            if (Debug && (Math.Abs(oldPct - rocket.throttle.throttlePercent.Value) > 0.0001f || oldOn != rocket.throttle.throttleOn.Value))
            {
                UnityEngine.Debug.Log($"[RocketManager] Throttle {effective:F3} ON={effective > 0f}");
            }
        }

        public void CutEngines()
        {
            SetThrottle(0f);
        }

        public bool CheckStaging(bool skipDuringLiftoff = false)
        {
            if (rocket == null || skipDuringLiftoff) return false;
            if (rocket.staging.stages.Count == 0) return false;
            if (!double.IsNegativeInfinity(lastStageFireWorldTime) &&
                WorldTime.main.worldTime - lastStageFireWorldTime < StageCooldown)
                return false;

            Stage stage = rocket.staging.stages[0];
            if (stagingAttempted.Contains(stage)) return false;
            if (HasThrust()) return false;

            if (!FireStage(stage))
                return false;

            stagingAttempted.Add(stage);
            lastStageFireWorldTime = WorldTime.main.worldTime;
            return true;
        }

        private bool FireStage(Stage stage)
        {
            try
            {
                if (StagingDrawer.main == null)
                {
                    if (Debug) UnityEngine.Debug.Log("[RocketManager] StagingDrawer.main is null");
                    return false;
                }

                var method = typeof(StagingDrawer).GetMethod("UseStage", BindingFlags.NonPublic | BindingFlags.Instance);
                if (method == null)
                {
                    if (Debug) UnityEngine.Debug.Log("[RocketManager] UseStage method not found");
                    return false;
                }

                bool hadControl = PlayerController.main.hasControl.Value;
                PlayerController.main.hasControl.Value = true;
                method.Invoke(StagingDrawer.main, new object[] { stage });
                PlayerController.main.hasControl.Value = hadControl;

                if (Debug) UnityEngine.Debug.Log($"[RocketManager] Stage {stage.stageId} fired");
                return true;
            }
            catch (Exception e)
            {
                if (Debug) UnityEngine.Debug.Log($"[RocketManager] FireStage exception: {e.Message}");
                return false;
            }
        }

        public void SetPitch(float targetDegrees)
        {
            if (rocket == null) return;

            float current = rocket.GetRotation();
            float error = NormalizeAngle(targetDegrees - current);
            float angVel = rocket.rb2d.angularVelocity;
            float turn;

            try
            {
                var method = typeof(Rocket).GetMethod("GetTorque", BindingFlags.NonPublic | BindingFlags.Instance);
                float torque = (float)method.Invoke(rocket, null);
                float mass = rocket.rb2d.mass;
                if (mass > 200f) torque /= Mathf.Pow(mass / 200f, 0.35f);

                float maxAngAccel = torque * Mathf.Rad2Deg / mass;
                float stopTime = Mathf.Abs(angVel / maxAngAccel);
                float timeToTarget = Mathf.Abs(angVel) > 0.001f ? Mathf.Abs(error / angVel) : float.PositiveInfinity;

                turn = (float.IsInfinity(timeToTarget) || stopTime > timeToTarget)
                    ? Mathf.Sign(angVel)
                    : -Mathf.Sign(error);
            }
            catch
            {
                turn = (-error * 0.8f - angVel * 0.5f) / 30f;
            }

            rocket.arrowkeys.turnAxis.Value = Mathf.Clamp(turn, -1f, 1f);

            if (Debug && UnityEngine.Time.frameCount % 60 == 0)
            {
                UnityEngine.Debug.Log($"[RocketManager] Pitch cur={current:F1} tgt={targetDegrees:F1} err={error:F1} av={angVel:F2} turn={turn:F2}");
            }
        }

        public bool IsPitchSettled(float targetDegrees)
        {
            if (rocket == null) return false;

            float err = Mathf.Abs(NormalizeAngle(targetDegrees - rocket.GetRotation()));
            return err <= PitchTolerance && Mathf.Abs(rocket.rb2d.angularVelocity) <= AngularVelocityLimit;
        }

        public void SetTurnAxis(float value)
        {
            if (rocket == null) return;
            rocket.arrowkeys.turnAxis.Value = Mathf.Clamp(value, -1f, 1f);
        }

        public void AlignPrograde()
        {
            SetPitch(GetProgradeAngle());
        }

        public void AlignRetrograde()
        {
            SetPitch(GetProgradeAngle() + 180f);
        }

        public void AlignSurfaceUp()
        {
            if (rocket == null) return;

            Double2 radial = rocket.location.position.Value.normalized;
            float angle = (float)(Math.Atan2(radial.y, radial.x) * Mathf.Rad2Deg);
            SetPitch(angle);
        }

        public void AlignHorizontal()
        {
            SetPitch(GetHorizontalAngle());
        }

        public void AlignToTarget(Double2 targetWorldPosition)
        {
            if (rocket == null) return;

            Double2 dir = (targetWorldPosition - rocket.location.position.Value).normalized;
            float angle = (float)(Math.Atan2(dir.y, dir.x) * Mathf.Rad2Deg);
            SetPitch(angle);
        }

        public void AlignBlended(float fromAngle, float toAngle, float t)
        {
            SetPitch(Mathf.LerpAngle(fromAngle, toAngle, Mathf.Clamp01(t)));
        }

        public float GetProgradeAngle()
        {
            if (rocket == null) return 0f;

            if (rocket.location.velocity.Value.magnitude < 0.1)
                return rocket.GetRotation();

            return (float)(Math.Atan2(rocket.location.velocity.Value.y, rocket.location.velocity.Value.x) * Mathf.Rad2Deg);
        }

        public float GetHorizontalAngle()
        {
            Double2 dir = GetHorizontalDirection();
            return (float)(Math.Atan2(dir.y, dir.x) * Mathf.Rad2Deg);
        }

        public double GetHorizontalSpeed()
        {
            if (rocket == null) return 0.0;
            return Double2.Dot(rocket.location.velocity.Value, GetHorizontalDirection());
        }

        public double GetVerticalSpeed()
        {
            if (rocket == null) return 0.0;

            Double2 radial = rocket.location.position.Value.normalized;
            return Double2.Dot(rocket.location.velocity.Value, radial);
        }

        public double GetAltitude()
        {
            if (rocket == null) return 0.0;
            return rocket.location.Value.Radius - rocket.location.planet.Value.Radius;
        }

        public double GetLocalGravity()
        {
            if (rocket == null) return 0.0;

            double radius = rocket.location.Value.Radius;
            return radius > 0 ? rocket.location.planet.Value.mass / (radius * radius) : 0.0;
        }

        public Double2 GetHorizontalDirection()
        {
            if (rocket == null) return Double2.right;

            Double2 radial = rocket.location.position.Value.normalized;
            if (radial.sqrMagnitude < 1E-10) return Double2.right;
            return new Double2(-radial.y, radial.x) * GetOrbitalDirectionSign();
        }

        public double GetOrbitalDirectionSign()
        {
            if (rocket == null) return 1.0;

            Double3 cross = Double3.Cross(rocket.location.position.Value, rocket.location.velocity.Value);
            return cross.z < 0.0 ? -1.0 : 1.0;
        }

        public double GetMaxThrust()
        {
            if (rocket == null) return 0.0;

            double thrust = 0.0;
            foreach (var engine in rocket.partHolder.GetModules<EngineModule>())
            {
                if (engine.engineOn.Value) thrust += engine.thrust.Value;
            }

            foreach (var booster in rocket.partHolder.GetModules<BoosterModule>())
            {
                if (booster.enabled) thrust += booster.thrustVector.Value.magnitude;
            }

            return thrust * 9.8;
        }

        public double GetMaxAcceleration()
        {
            if (rocket == null) return cachedMaxAcceleration;

            double thrust = GetMaxThrust();
            double mass = rocket.mass.GetMass();
            if (mass <= 0.00001)
                return cachedMaxAcceleration;

            double accel = thrust / mass;
            if (accel > 0.001) cachedMaxAcceleration = accel;
            return accel > 0.001 ? accel : cachedMaxAcceleration;
        }

        public bool HasThrust()
        {
            if (rocket == null) return false;

            foreach (var engine in rocket.partHolder.GetModules<EngineModule>())
            {
                if (engine.engineOn.Value && engine.thrust.Value > 0.001f) return true;
            }

            foreach (var booster in rocket.partHolder.GetModules<BoosterModule>())
            {
                if (booster.enabled && booster.thrustVector.Value.magnitude > 0.001f) return true;
            }

            return false;
        }

        public static float NormalizeAngle(float angle)
        {
            float wrapped = (angle + 180f) % 360f;
            if (wrapped < 0) wrapped += 360f;
            return wrapped - 180f;
        }
    }
}
