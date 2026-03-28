using System;
using System.Collections.Generic;
using System.Reflection;
using SFS.Parts;
using SFS.Parts.Modules;
using SFS.World;
using UnityEngine;

namespace RevolutionlessAutopilot
{
    /// <summary>
    /// Centralises all low-level rocket control operations.
    /// AscentAutopilot and LandingAutopilot delegate here instead of
    /// duplicating control code. Holds no mission state — only rocket state.
    /// </summary>
    public class RocketManager
    {
        private Rocket rocket;

        // (RU) Последнее ненулевое ускорение — используем когда двигатели не горят
        // (EN) Last known non-zero acceleration — used when engines are not burning
        private double cachedMaxAcceleration = 0;

        // (RU) Ступени, для которых уже была попытка запуска
        // (EN) Stages that have already been staged this session
        private HashSet<Stage> stagingAttempted = new HashSet<Stage>();

        // (RU) Регулируемые допуски | (EN) Tuneable tolerances
        // (RU) Эти значения соответствуют значениям, используемым в AscentAutopilot и LandingAutopilot.
        //      Установите их перед использованием, если другому автопилоту требуется более жесткое/мягкое управление.
        // (EN) These match the values used in AscentAutopilot and LandingAutopilot.
        //      Set them before use if a different autopilot needs tighter/looser control.
        public float PitchTolerance        = 2.0f;   // (RU) степени | (EN) degrees
        public float AngularVelocityLimit  = 4.0f;   // (RU) град/с | (EN) deg/s
        public float ThrottleSnapThreshold = 0.01f;  // (RU) ниже → перейти к 0 | (EN) below this → snap to 0

        public bool Debug = true;

        // (RU) Строительство | (EN) Construction
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
        }

        // (RU) Дроссель | (EN) Throttle
        /// <summary>Sets throttle [0–1]. Values below ThrottleSnapThreshold snap to 0.</summary>
        public void SetThrottle(float percent)
        {
            float effective = Mathf.Clamp01(percent);
            if (effective < ThrottleSnapThreshold) effective = 0f;

            float oldPct = rocket.throttle.throttlePercent.Value;
            bool  oldOn  = rocket.throttle.throttleOn.Value;

            rocket.throttle.throttlePercent.Value = effective;
            rocket.throttle.throttleOn.Value      = effective > 0f;

            if (Debug && (oldPct != rocket.throttle.throttlePercent.Value || oldOn != rocket.throttle.throttleOn.Value))
                UnityEngine.Debug.Log($"[RocketManager] Throttle {effective:F3}  ON={effective > 0f}");
        }

        /// <summary>Immediately cuts all engine throttle.</summary>
        public void CutEngines() => SetThrottle(0f);

        // (RU) Постановка | (EN) Staging
        /// <summary>
        /// Fires the next stage if no engine is currently producing thrust.
        /// Pass skipDuringLiftoff=true from the Liftoff state to avoid premature staging.
        /// </summary>
        public void CheckStaging(bool skipDuringLiftoff = false)
        {
            if (skipDuringLiftoff) return;
            if (rocket.staging.stages.Count == 0) return;

            Stage stage = rocket.staging.stages[0];
            if (stagingAttempted.Contains(stage)) return;

            if (HasThrust()) return;

            stagingAttempted.Add(stage);
            FireStage(stage);
        }

        private void FireStage(Stage stage)
        {
            try
            {
                if (StagingDrawer.main == null)
                {
                    if (Debug) UnityEngine.Debug.Log("[RocketManager] StagingDrawer.main is null");
                    return;
                }

                var method = typeof(StagingDrawer).GetMethod("UseStage",
                    BindingFlags.NonPublic | BindingFlags.Instance);
                if (method == null)
                {
                    if (Debug) UnityEngine.Debug.Log("[RocketManager] UseStage method not found");
                    return;
                }

                // (RU) Временно перехватите управление у игрока, чтобы проверка HasControl внутри UseStage прошла успешно | (EN) Temporarily claim player control so the HasControl check inside UseStage passes
                bool hadControl = PlayerController.main.hasControl.Value;
                PlayerController.main.hasControl.Value = true;
                method.Invoke(StagingDrawer.main, new object[] { stage });
                PlayerController.main.hasControl.Value = hadControl;

                if (Debug) UnityEngine.Debug.Log($"[RocketManager] Stage {stage.stageId} fired");
            }
            catch (Exception e)
            {
                if (Debug) UnityEngine.Debug.Log($"[RocketManager] FireStage exception: {e.Message}");
            }
        }

        // (RU) Настрой — контроль над подачей | (EN) Attitude — raw pitch control
        /// <summary>
        /// Drives the rocket toward targetDegrees using a bang-bang + PD fallback controller.
        /// Uses internal GetTorque to compute the optimal braking point.
        /// </summary>
        public void SetPitch(float targetDegrees)
        {
            float current = rocket.GetRotation();
            float error   = NormalizeAngle(targetDegrees - current);
            float angVel  = rocket.rb2d.angularVelocity;
            float turn;

            try
            {
                var method = typeof(Rocket).GetMethod("GetTorque",
                    BindingFlags.NonPublic | BindingFlags.Instance);
                float torque = (float)method.Invoke(rocket, null);
                float mass   = rocket.rb2d.mass;
                if (mass > 200f) torque /= Mathf.Pow(mass / 200f, 0.35f);

                float maxAngAccel  = torque * Mathf.Rad2Deg / mass;
                float stopTime     = Mathf.Abs(angVel / maxAngAccel);
                float timeToTarget = Mathf.Abs(angVel) > 0.001f
                    ? Mathf.Abs(error / angVel)
                    : float.PositiveInfinity;

                // (RU) Если не удаётся вовремя остановиться, стреляем в текущем направлении для торможения; в противном случае поворачиваем руль | (EN) If we can't stop in time, fire in the current direction to brake; otherwise steer
                turn = (float.IsInfinity(timeToTarget) || stopTime > timeToTarget)
                    ? Mathf.Sign(angVel)
                    : -Mathf.Sign(error);
            }
            catch
            {
                // (RU) В случае неудачи при использовании рефлексии, используйте резервный контроллер PD | (EN) Fallback PD controller if reflection fails
                turn = (-error * 0.8f - angVel * 0.5f) / 30f;
            }

            rocket.arrowkeys.turnAxis.Value = Mathf.Clamp(turn, -1f, 1f);

            if (Debug && UnityEngine.Time.frameCount % 60 == 0)
                UnityEngine.Debug.Log($"[RocketManager] Pitch cur={current:F1}° tgt={targetDegrees:F1}° err={error:F1}° av={angVel:F2} turn={turn:F2}");
        }

        /// <summary>Returns true when the rocket is pointing within tolerance of targetDegrees and nearly stopped rotating.</summary>
        public bool IsPitchSettled(float targetDegrees)
        {
            float err = Mathf.Abs(NormalizeAngle(targetDegrees - rocket.GetRotation()));
            return err <= PitchTolerance && Mathf.Abs(rocket.rb2d.angularVelocity) <= AngularVelocityLimit;
        }

        /// <summary>Directly sets the turn axis without any PID. Use for coasting (0) or manual slew.</summary>
        public void SetTurnAxis(float value)
        {
            rocket.arrowkeys.turnAxis.Value = Mathf.Clamp(value, -1f, 1f);
        }

        // (RU) Отношение — именованные ярлыки выравнивания | (EN) Attitude — named alignment shortcuts
        /// <summary>Points the rocket prograde (along velocity vector).</summary>
        public void AlignPrograde()
        {
            SetPitch(GetProgradeAngle());
        }

        /// <summary>Points the rocket retrograde (opposite velocity vector).</summary>
        public void AlignRetrograde()
        {
            SetPitch(GetProgradeAngle() + 180f);
        }

        /// <summary>Points the rocket straight up (radially away from planet center).</summary>
        public void AlignSurfaceUp()
        {
            Double2 radial = rocket.location.position.Value.normalized;
            float angle = (float)(Math.Atan2(radial.y, radial.x) * Mathf.Rad2Deg);
            SetPitch(angle);
        }

        /// <summary>Points the rocket to the local horizontal (orbital tangent, prograde direction).</summary>
        public void AlignHorizontal()
        {
            SetPitch(GetHorizontalAngle());
        }

        /// <summary>Points the rocket toward an arbitrary world-space position.</summary>
        public void AlignToTarget(Double2 targetWorldPosition)
        {
            Double2 dir = (targetWorldPosition - rocket.location.position.Value).normalized;
            float angle = (float)(Math.Atan2(dir.y, dir.x) * Mathf.Rad2Deg);
            SetPitch(angle);
        }

        /// <summary>
        /// Blends between two named attitudes by t in [0, 1].
        /// t=0 → from, t=1 → to. Useful for smooth attitude transitions.
        /// </summary>
        public void AlignBlended(float fromAngle, float toAngle, float t)
        {
            SetPitch(Mathf.LerpAngle(fromAngle, toAngle, Mathf.Clamp01(t)));
        }

        // (RU) Измерения скорости и положения | (EN) Velocity and position measurements
        /// <summary>Angle of the velocity vector in world space (degrees). Used as prograde reference.</summary>
        public float GetProgradeAngle()
        {
            if (rocket.location.velocity.Value.magnitude < 0.1)
                return rocket.GetRotation();
            return (float)(Math.Atan2(rocket.location.velocity.Value.y,
                                      rocket.location.velocity.Value.x) * Mathf.Rad2Deg);
        }

        /// <summary>Angle of the local horizontal direction (orbital tangent) in world space degrees.</summary>
        public float GetHorizontalAngle()
        {
            return (float)(GetHorizontalDirection().AngleRadians * Mathf.Rad2Deg);
        }

        /// <summary>Speed component along the local horizontal (positive = prograde).</summary>
        public double GetHorizontalSpeed()
        {
            return Double2.Dot(rocket.location.velocity.Value, GetHorizontalDirection());
        }

        /// <summary>Speed component along the local vertical (positive = ascending).</summary>
        public double GetVerticalSpeed()
        {
            Double2 radial = rocket.location.position.Value.normalized;
            return Double2.Dot(rocket.location.velocity.Value, radial);
        }

        /// <summary>Altitude above the planet surface in metres.</summary>
        public double GetAltitude()
        {
            return rocket.location.Value.Radius - rocket.location.planet.Value.Radius;
        }

        /// <summary>Local gravitational acceleration at the current radius (m/s²).</summary>
        public double GetLocalGravity()
        {
            double r = rocket.location.Value.Radius;
            return r > 0 ? rocket.location.planet.Value.mass / (r * r) : 0;
        }

        /// <summary>Unit vector in the local horizontal (orbital tangent) direction.</summary>
        public Double2 GetHorizontalDirection()
        {
            Double2 radial = rocket.location.position.Value.normalized;
            if (radial.sqrMagnitude < 1E-10) return Double2.right;
            return new Double2(-radial.y, radial.x) * GetOrbitalDirectionSign();
        }

        /// <summary>+1 if orbiting counterclockwise, -1 if clockwise.</summary>
        public double GetOrbitalDirectionSign()
        {
            Double3 cross = Double3.Cross(rocket.location.position.Value, rocket.location.velocity.Value);
            return cross.z < 0.0 ? -1.0 : 1.0;
        }

        // (RU) Производительность двигателя | (EN) Engine performance
        /// <summary>Total thrust from all active engines and boosters (N).</summary>
        public double GetMaxThrust()
        {
            double thrust = 0;
            foreach (var e in rocket.partHolder.GetModules<EngineModule>())
                if (e.engineOn.Value) thrust += e.thrust.Value;
            foreach (var b in rocket.partHolder.GetModules<BoosterModule>())
                if (b.enabled) thrust += b.thrustVector.Value.magnitude;
            return thrust * 9.8;
        }

        /// <summary>
        /// Maximum acceleration from all active engines (m/s²).
        /// Returns the last cached value when engines are off, so burn planning
        /// still works immediately after CutEngines().
        /// </summary>
        public double GetMaxAcceleration()
        {
            double thrust = 0;
            foreach (var e in rocket.partHolder.GetModules<EngineModule>())
                if (e.engineOn.Value) thrust += e.thrust.Value * 9.8;
            foreach (var b in rocket.partHolder.GetModules<BoosterModule>())
                if (b.enabled) thrust += b.thrustVector.Value.magnitude * 9.8;

            double mass = rocket.mass.GetMass();
            if (mass <= 0.00001)
                return cachedMaxAcceleration;

            double accel = thrust / mass;
            if (accel > 0.001) cachedMaxAcceleration = accel;

            // (RU) Возвращает кэшированное значение, если двигатели только что отключились (ускорение кратковременно показывает 0 после начала подготовки) | (EN) Return cached value if engines just cut (accel briefly reads 0 after staging)
            return accel > 0.001 ? accel : cachedMaxAcceleration;
        }

        /// <summary>True if any engine or booster is currently producing thrust.</summary>
        public bool HasThrust()
        {
            foreach (var e in rocket.partHolder.GetModules<EngineModule>())
                if (e.engineOn.Value && e.thrust.Value > 0.001f) return true;
            foreach (var b in rocket.partHolder.GetModules<BoosterModule>())
                if (b.enabled && b.thrustVector.Value.magnitude > 0.001f) return true;
            return false;
        }

        // (RU) Утилита | (EN) Utility
        /// <summary>Wraps an angle to [-180, +180] degrees.</summary>
        public static float NormalizeAngle(float angle)
        {
            float m = (angle + 180f) % 360f;
            if (m < 0) m += 360f;
            return m - 180f;
        }
    }
}