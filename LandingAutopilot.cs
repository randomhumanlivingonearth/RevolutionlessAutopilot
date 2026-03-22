using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using SFS;
using SFS.Parts;
using SFS.Parts.Modules;
using SFS.UI;
using SFS.World;
using SFS.WorldBase;
using UnityEngine;

namespace RevolutionlessAutopilot
{
    public enum LandingState
    {
        Idle,
        Deorbit,       // (RU) Ретроградный импульс для снижения перицентра | (EN) Retrograde burn to lower periapsis
        Reentry,       // (RU) Двигатели выключены, тормозим об атмосферу | (EN) Engines off, aerobraking through atmosphere
        AerobrakeCoast,// (RU) Ожидаем снижения скорости до точки зажигания | (EN) Waiting for speed to drop to ignition point
        Flip,          // (RU) Разворот на 180° — двигатели вниз | (EN) Flip 180° — engines pointing down
        SuicideBurn,   // (RU) Финальный тормозной импульс | (EN) Final braking burn
        TouchdownIdle  // (RU) Посадка выполнена | (EN) Landing complete
    }

    public class LandingAutopilot
    {
        private Rocket rocket;
        private LandingState state = LandingState.Idle;

        // (RU) Высота атмосферы планеты (0 если нет атмосферы) | (EN) Planet atmosphere height (0 if no atmosphere)
        private double atmosphereHeight = 0;

        // (RU) Локальное ускорение свободного падения, пересчитывается каждый кадр | (EN) Local gravitational acceleration, recalculated each frame
        private double localGravity = 0;

        // ──────────────────────────────────────────────
        // (RU) Параметры деорбитального манёвра | (EN) Deorbit parameters
        // ──────────────────────────────────────────────

        // (RU) До куда опускаем перицентр при деорбите (ниже 0 = под поверхность, надёжнее) | (EN) How low to push periapsis on deorbit (below 0 = underground, more reliable)
        private const double DEORBIT_PERIAPSIS_TARGET = -10000.0;

        // (RU) Допуск перицентра при деорбите — прекращаем жечь когда перицентр достиг цели | (EN) Periapsis tolerance for deorbit — stop burning when periapsis reaches target
        private const double DEORBIT_PERIAPSIS_TOLERANCE = 2000.0;

        // (RU) Минимальная высота для начала деорбитального манёвра (должны быть выше атмосферы) | (EN) Minimum altitude to begin deorbit burn (must be above atmosphere)
        private const double DEORBIT_MIN_ALTITUDE = 5000.0;

        // ──────────────────────────────────────────────
        // (RU) Параметры повторного входа | (EN) Reentry parameters
        // ──────────────────────────────────────────────

        // (RU) Высота переключения с Reentry на AerobrakeCoast (без атмосферы — сразу Flip) | (EN) Altitude to switch from Reentry to AerobrakeCoast (no atmosphere — go straight to Flip)
        private const double REENTRY_HANDOFF_ALTITUDE = 8000.0;

        // (RU) Максимальная скорость при входе в зону ожидания (иначе продолжаем тормозить атмосферой) | (EN) Max speed at aerobrake coast handoff (otherwise keep aerobraking)
        private const double REENTRY_MAX_COAST_SPEED = 500.0;

        // ──────────────────────────────────────────────
        // (RU) Параметры разворота | (EN) Flip parameters
        // ──────────────────────────────────────────────

        // (RU) Целевой угол тангажа при развороте — прямо вниз (двигатели к земле) | (EN) Target pitch angle during flip — straight down (engines toward ground)
        private const float FLIP_TARGET_PITCH = -90f;

        // (RU) Допуск угла при развороте перед зажиганием | (EN) Pitch tolerance before ignition
        private const float FLIP_PITCH_TOLERANCE = 5f;

        // (RU) Высота, ниже которой начинаем разворот | (EN) Altitude below which we begin the flip
        private const double FLIP_START_ALTITUDE = 6000.0;

        // ──────────────────────────────────────────────
        // (RU) Параметры финального торможения (suicide burn) | (EN) Suicide burn parameters
        // ──────────────────────────────────────────────

        // (RU) Запас высоты поверх расчётной точки зажигания (страховка от недожога) | (EN) Extra altitude margin above calculated ignition point (safety buffer)
        private const double SUICIDE_BURN_ALTITUDE_MARGIN = 150.0;

        // (RU) Скорость, при которой считаем посадку выполненной | (EN) Speed below which we consider landing complete
        private const double TOUCHDOWN_SPEED_THRESHOLD = 5.0;

        // (RU) Высота, при которой считаем посадку выполненной | (EN) Altitude below which we consider landing complete
        private const double TOUCHDOWN_ALTITUDE_THRESHOLD = 10.0;

        // (RU) Минимальная тяга в финальной фазе, чтобы не падать камнем | (EN) Minimum throttle in final phase to avoid free-fall
        private const float SUICIDE_BURN_MIN_THROTTLE = 0.05f;

        // (RU) Коэффициент дросселирования при подлёте к земле | (EN) Throttle reduction coefficient as ground approaches
        private const double SUICIDE_BURN_THROTTLE_GAIN = 0.5;

        // (RU) Высота перехода к режиму мягкой посадки | (EN) Altitude to transition to soft landing mode
        private const double SOFT_LANDING_ALTITUDE = 200.0;

        // (RU) Целевая вертикальная скорость в режиме мягкой посадки (м/с вниз) | (EN) Target vertical speed in soft landing mode (m/s downward)
        private const double SOFT_LANDING_TARGET_SPEED = 8.0;

        // ──────────────────────────────────────────────
        // (RU) Общие параметры управления | (EN) General control parameters
        // ──────────────────────────────────────────────

        private const float THROTTLE_SNAP_THRESHOLD = 0.01f;
        private const double GUIDED_BURN_SETTLE_TIME = 0.75;
        private const float GUIDED_BURN_PITCH_TOLERANCE = 3.0f;
        private const float GUIDED_BURN_ANGULAR_VELOCITY_TOLERANCE = 4.0f;

        // ──────────────────────────────────────────────
        // (RU) Внутреннее состояние | (EN) Internal state
        // ──────────────────────────────────────────────

        private double throttleUnlockWorldTime;
        private bool enginesInitialized = false;
        private Stage currentStage = null;
        private HashSet<Stage> stagingAttempted = new HashSet<Stage>();
        private bool flipComplete = false;
        private int coastLogCounter = 0;

        private bool debug = true;

        public bool IsActive { get; private set; }

        // ──────────────────────────────────────────────
        // (RU) Конструктор и инициализация | (EN) Constructor and initialization
        // ──────────────────────────────────────────────

        public LandingAutopilot(Rocket rocket)
        {
            this.rocket = rocket;
            RefreshPlanetData();

            if (debug) Debug.Log("[LandingAutopilot] LandingAutopilot initialized");
        }

        public void SetRocket(Rocket rocket)
        {
            this.rocket = rocket;
            RefreshPlanetData();
        }

        private void RefreshPlanetData()
        {
            if (rocket?.location?.planet?.Value == null) return;

            var planet = rocket.location.planet.Value;
            atmosphereHeight = planet.AtmosphereHeightPhysics;

            double r = rocket.location.Value.Radius;
            if (r > 0)
                localGravity = planet.mass / (r * r);

            if (debug)
                Debug.Log($"[LandingAutopilot] Planet data: atmosphere={atmosphereHeight:F0}m, gravity={localGravity:F3}m/s²");
        }

        public void Start()
        {
            if (rocket == null) return;
            RefreshPlanetData();
            IsActive = true;
            state = LandingState.Deorbit;
            enginesInitialized = false;
            currentStage = null;
            stagingAttempted.Clear();
            flipComplete = false;
            throttleUnlockWorldTime = 0.0;
            if (debug) Debug.Log("[LandingAutopilot] STARTED");
        }

        public void Stop()
        {
            IsActive = false;
            state = LandingState.Idle;
            if (rocket != null)
            {
                rocket.throttle.throttleOn.Value = false;
                rocket.throttle.throttlePercent.Value = 0f;
                rocket.arrowkeys.turnAxis.Value = 0f;
            }
            if (debug) Debug.Log("[LandingAutopilot] STOPPED");
        }

        // ──────────────────────────────────────────────
        // (RU) Основной цикл | (EN) Main loop
        // ──────────────────────────────────────────────

        public void Update() { }

        public void FixedUpdate()
        {
            if (!IsActive || rocket == null) return;

            RefreshPlanetData();

            double altitude = rocket.location.Value.Radius - rocket.location.planet.Value.Radius;
            double speed = rocket.location.velocity.Value.magnitude;
            double verticalSpeed = GetVerticalSpeed(); // (RU) отрицательное при снижении | (EN) negative when descending
            bool hasAtmosphere = atmosphereHeight > 100.0;
            bool aboveAtmosphere = altitude > atmosphereHeight;

            // (RU) Пересчитываем гравитацию по текущему радиусу | (EN) Recalculate gravity at current radius
            double r = rocket.location.Value.Radius;
            if (r > 0)
                localGravity = rocket.location.planet.Value.mass / (r * r);

            if (debug && UnityEngine.Time.frameCount % 60 == 0)
                Debug.Log($"[LandingAutopilot] State={state}, Alt={altitude:F0}m, Speed={speed:F1}m/s, VertSpeed={verticalSpeed:F1}m/s, g={localGravity:F3}m/s²");

            // (RU) Обновляем текущую ступень | (EN) Update current stage
            Stage newStage = (rocket.staging.stages.Count > 0) ? rocket.staging.stages[0] : null;
            if (newStage != currentStage)
            {
                if (debug) Debug.Log($"[LandingAutopilot] Stage changed: {currentStage?.stageId} -> {newStage?.stageId}");
                currentStage = newStage;
                enginesInitialized = false;
            }

            switch (state)
            {
                // ──────────────────────────────────────────────
                // (RU) ДЕОРБИТА: ретроградный импульс для снижения перицентра | (EN) DEORBIT: retrograde burn to lower periapsis
                // ──────────────────────────────────────────────
                case LandingState.Deorbit:
                {
                    double periapsis = GetPeriapsis();
                    double periAlt = periapsis - rocket.location.planet.Value.Radius;

                    // (RU) Если перицентр уже ниже цели — идём на вход | (EN) If periapsis is already below target — go to reentry
                    if (periAlt <= DEORBIT_PERIAPSIS_TARGET + DEORBIT_PERIAPSIS_TOLERANCE)
                    {
                        CutEngines();
                        if (debug) Debug.Log($"[LandingAutopilot] Deorbit complete, periAlt={periAlt:F0}m -> Reentry");
                        state = hasAtmosphere ? LandingState.Reentry : LandingState.Flip;
                        break;
                    }

                    // (RU) Указываем ретроградно | (EN) Point retrograde
                    float retrogradeAngle = GetRetrogradeAngle();
                    SetPitch(retrogradeAngle);
                    bool settled = IsPitchSettled(retrogradeAngle);

                    // (RU) Жжём только если сориентированы и выше атмосферы | (EN) Burn only if oriented and above atmosphere
                    if (settled && aboveAtmosphere && altitude > DEORBIT_MIN_ALTITUDE)
                    {
                        // (RU) Тяга пропорциональна расстоянию до цели по перицентру | (EN) Throttle proportional to distance to periapsis target
                        double periError = periAlt - DEORBIT_PERIAPSIS_TARGET;
                        float throttle = (float)Math.Min(1.0, periError / 20000.0);
                        throttle = Mathf.Max(0.1f, throttle);
                        SetThrottle(throttle);

                        if (!enginesInitialized && currentStage != null)
                        {
                            InitializeEngines();
                            enginesInitialized = true;
                        }
                    }
                    else
                    {
                        CutEngines();
                    }

                    if (debug && UnityEngine.Time.frameCount % 60 == 0)
                        Debug.Log($"[LandingAutopilot] Deorbit: periAlt={periAlt:F0}m, target={DEORBIT_PERIAPSIS_TARGET:F0}m, settled={settled}");
                    break;
                }

                // ──────────────────────────────────────────────
                // (RU) ВХОД В АТМОСФЕРУ: двигатели выключены, ориентация ретроградно | (EN) REENTRY: engines off, hold retrograde
                // ──────────────────────────────────────────────
                case LandingState.Reentry:
                {
                    CutEngines();

                    // (RU) Держим ретроград чтобы не кувыркаться | (EN) Hold retrograde to avoid tumbling
                    SetPitch(GetRetrogradeAngle());

                    coastLogCounter++;
                    if (debug && coastLogCounter % 60 == 0)
                        Debug.Log($"[LandingAutopilot] Reentry: alt={altitude:F0}m, speed={speed:F1}m/s");

                    // (RU) Переход к ожиданию когда достаточно низко и медленно | (EN) Transition to coast when low and slow enough
                    if (altitude < REENTRY_HANDOFF_ALTITUDE && speed < REENTRY_MAX_COAST_SPEED)
                    {
                        if (debug) Debug.Log($"[LandingAutopilot] Reentry -> AerobrakeCoast at alt={altitude:F0}m, speed={speed:F1}m/s");
                        state = LandingState.AerobrakeCoast;
                        coastLogCounter = 0;
                    }
                    break;
                }

                // ──────────────────────────────────────────────
                // (RU) ОЖИДАНИЕ: ждём точки зажигания | (EN) AEROBRAKE COAST: waiting for ignition point
                // ──────────────────────────────────────────────
                case LandingState.AerobrakeCoast:
                {
                    CutEngines();
                    SetPitch(GetRetrogradeAngle());

                    double burnAlt = CalculateSuicideBurnAltitude(speed, verticalSpeed);

                    coastLogCounter++;
                    if (debug && coastLogCounter % 30 == 0)
                        Debug.Log($"[LandingAutopilot] AerobrakeCoast: alt={altitude:F0}m, burnAlt={burnAlt:F0}m, speed={speed:F1}m/s");

                    // (RU) Пора разворачиваться | (EN) Time to flip
                    if (altitude < FLIP_START_ALTITUDE || altitude < burnAlt + SUICIDE_BURN_ALTITUDE_MARGIN * 3)
                    {
                        if (debug) Debug.Log($"[LandingAutopilot] AerobrakeCoast -> Flip");
                        state = LandingState.Flip;
                        flipComplete = false;
                    }
                    break;
                }

                // ──────────────────────────────────────────────
                // (RU) РАЗВОРОТ: поворачиваем двигатели к земле | (EN) FLIP: rotate engines toward ground
                // ──────────────────────────────────────────────
                case LandingState.Flip:
                {
                    CutEngines();
                    SetPitch(FLIP_TARGET_PITCH);

                    float pitchError = Mathf.Abs(NormalizeAngle(FLIP_TARGET_PITCH - rocket.GetRotation()));
                    flipComplete = pitchError <= FLIP_PITCH_TOLERANCE;

                    if (debug && UnityEngine.Time.frameCount % 30 == 0)
                        Debug.Log($"[LandingAutopilot] Flip: pitchError={pitchError:F1}°, complete={flipComplete}");

                    if (flipComplete)
                    {
                        if (debug) Debug.Log("[LandingAutopilot] Flip complete -> SuicideBurn");
                        state = LandingState.SuicideBurn;
                        throttleUnlockWorldTime = WorldTime.main.worldTime + GUIDED_BURN_SETTLE_TIME;

                        if (currentStage != null)
                        {
                            InitializeEngines();
                            enginesInitialized = true;
                        }
                    }
                    break;
                }

                // ──────────────────────────────────────────────
                // (RU) ФИНАЛЬНОЕ ТОРМОЖЕНИЕ | (EN) SUICIDE BURN
                // ──────────────────────────────────────────────
                case LandingState.SuicideBurn:
                {
                    // (RU) Всегда держим двигатели вниз | (EN) Always hold engines down
                    SetPitch(FLIP_TARGET_PITCH);

                    // (RU) Проверка посадки | (EN) Check for touchdown
                    if (altitude < TOUCHDOWN_ALTITUDE_THRESHOLD && Math.Abs(verticalSpeed) < TOUCHDOWN_SPEED_THRESHOLD)
                    {
                        CutEngines();
                        rocket.arrowkeys.turnAxis.Value = 0f;
                        state = LandingState.TouchdownIdle;
                        if (debug) Debug.Log($"[LandingAutopilot] Touchdown! speed={speed:F1}m/s, alt={altitude:F0}m");
                        MsgDrawer.main.Log("Landing complete.");
                        Stop();
                        break;
                    }

                    // (RU) Ждём пока не установится ориентация | (EN) Wait for orientation to settle
                    if (WorldTime.main.worldTime < throttleUnlockWorldTime)
                    {
                        SetThrottle(0f);
                        break;
                    }

                    float finalThrottle;

                    if (altitude < SOFT_LANDING_ALTITUDE)
                    {
                        // (RU) Режим мягкой посадки: ПИД по вертикальной скорости | (EN) Soft landing: PID on vertical speed
                        double targetVertSpeed = -SOFT_LANDING_TARGET_SPEED;
                        double speedError = verticalSpeed - targetVertSpeed; // (RU) положительно => падаем быстрее цели | (EN) positive => falling faster than target

                        // (RU) Гравитационная компенсация + пропорциональный член | (EN) Gravity compensation + proportional term
                        double gravComp = localGravity / Math.Max(CalculateMaxAcceleration(), 0.001);
                        double pTerm = speedError / 10.0;
                        finalThrottle = Mathf.Clamp((float)(gravComp + pTerm), SUICIDE_BURN_MIN_THROTTLE, 1f);

                        if (debug && UnityEngine.Time.frameCount % 30 == 0)
                            Debug.Log($"[LandingAutopilot] SoftLanding: alt={altitude:F0}m, vspeed={verticalSpeed:F1}m/s, throttle={finalThrottle:F2}");
                    }
                    else
                    {
                        // ── (RU) Расчёт точки зажигания | (EN) Ignition point calculation ──
                        double burnAlt = CalculateSuicideBurnAltitude(speed, verticalSpeed);
                        bool shouldIgnite = altitude <= burnAlt + SUICIDE_BURN_ALTITUDE_MARGIN;

                        if (shouldIgnite)
                        {
                            // (RU) Дроссель пропорционален отношению скорости к максимальному ускорению | (EN) Throttle proportional to speed vs max acceleration
                            double maxAccel = CalculateMaxAcceleration();
                            double neededAccel = (speed * speed) / (2.0 * Math.Max(altitude, 1.0));
                            finalThrottle = Mathf.Clamp((float)(neededAccel / Math.Max(maxAccel, 0.001)), SUICIDE_BURN_MIN_THROTTLE, 1f);
                        }
                        else
                        {
                            finalThrottle = 0f;
                        }

                        if (debug && UnityEngine.Time.frameCount % 30 == 0)
                            Debug.Log($"[LandingAutopilot] SuicideBurn: alt={altitude:F0}m, burnAlt={burnAlt:F0}m, speed={speed:F1}m/s, throttle={finalThrottle:F2}");
                    }

                    SetThrottle(finalThrottle);
                    break;
                }

                case LandingState.TouchdownIdle:
                    break;
            }
        }

        // (RU) Расчёт высоты начала торможения | (EN) Suicide burn altitude calculation
        //
        // (RU) Формула: h = v² / (2 * (a - g))
        // (EN) Formula: h = v² / (2 * (a - g))
        //
        // (RU) Где v — скорость, a — ускорение от двигателей, g — гравитация
        // (EN) Where v = speed, a = engine acceleration, g = local gravity
        private double CalculateSuicideBurnAltitude(double speed, double verticalSpeed)
        {
            double effectiveSpeed = Math.Abs(verticalSpeed) > 0.1 ? Math.Abs(verticalSpeed) : speed;
            double maxAccel = CalculateMaxAcceleration();
            double netDecel = maxAccel - localGravity;

            if (netDecel <= 0.01)
            {
                // (RU) Двигатель слишком слабый — возвращаем большое значение как предупреждение | (EN) Engine too weak — return large value as warning
                if (debug) Debug.Log("[LandingAutopilot] WARNING: net deceleration <= 0, TWR < 1");
                return double.PositiveInfinity;
            }

            return (effectiveSpeed * effectiveSpeed) / (2.0 * netDecel);
        }

        // (RU) Угол ретрограда | (EN) Retrograde angle
        private float GetRetrogradeAngle()
        {
            var vel = rocket.location.velocity.Value;
            if (vel.magnitude < 0.1f) return rocket.GetRotation();
            float progradeAngle = (float)Math.Atan2(vel.y, vel.x) * Mathf.Rad2Deg;
            return progradeAngle + 180f; // (RU) Обратное направление | (EN) Opposite direction
        }

        // (RU) Вертикальная скорость (отрицательная при снижении) | (EN) Vertical speed (negative when descending)
        private double GetVerticalSpeed()
        {
            Double2 radial = rocket.location.position.Value.normalized;
            return Double2.Dot(rocket.location.velocity.Value, radial);
        }

        // (RU) Вспомогательные методы — идентичны AscentAutopilot | (EN) Helper methods — identical to AscentAutopilot
        private void InitializeEngines()
        {
            if (currentStage == null) return;
            if (!rocket.staging.stages.Contains(currentStage)) return;

            if (debug) Debug.Log($"[LandingAutopilot] Initializing engines for stage {currentStage.stageId}");

            foreach (var part in currentStage.parts)
            {
                foreach (var engine in part.GetModules<EngineModule>())
                {
                    if (!engine.engineOn.Value)
                    {
                        engine.engineOn.Value = true;
                        if (debug) Debug.Log("[LandingAutopilot] Engine turned ON");
                    }
                    if (engine.hasGimbal && engine.gimbalOn != null && !engine.gimbalOn.Value)
                    {
                        engine.gimbalOn.Value = true;
                        if (debug) Debug.Log("[LandingAutopilot] Gimbal enabled");
                    }
                }
            }
        }

        private void SetThrottle(float percent)
        {
            float effectivePercent = Mathf.Clamp01(percent);
            if (effectivePercent < THROTTLE_SNAP_THRESHOLD)
                effectivePercent = 0f;

            float oldPercent = rocket.throttle.throttlePercent.Value;
            bool oldOn = rocket.throttle.throttleOn.Value;

            rocket.throttle.throttlePercent.Value = effectivePercent;
            rocket.throttle.throttleOn.Value = effectivePercent > 0f;

            if (debug && (oldPercent != rocket.throttle.throttlePercent.Value || oldOn != rocket.throttle.throttleOn.Value))
                Debug.Log($"[LandingAutopilot] Throttle set to {rocket.throttle.throttlePercent.Value:F3}, ON={rocket.throttle.throttleOn.Value}");
        }

        private void CutEngines() => SetThrottle(0f);

        private float NormalizeAngle(float angle)
        {
            float m = (angle + 180f) % 360f;
            if (m < 0) m += 360f;
            return m - 180f;
        }

        private bool IsPitchSettled(float targetPitchDegrees)
        {
            float pitchError = Mathf.Abs(NormalizeAngle(targetPitchDegrees - rocket.GetRotation()));
            return pitchError <= GUIDED_BURN_PITCH_TOLERANCE && Math.Abs(rocket.rb2d.angularVelocity) <= GUIDED_BURN_ANGULAR_VELOCITY_TOLERANCE;
        }

        private void SetPitch(float targetPitchDegrees)
        {
            float currentPitch = rocket.GetRotation();
            float error = NormalizeAngle(targetPitchDegrees - currentPitch);
            float angularVelocity = rocket.rb2d.angularVelocity;
            float turn;

            try
            {
                var method = typeof(Rocket).GetMethod("GetTorque", BindingFlags.NonPublic | BindingFlags.Instance);
                float torque = (float)method.Invoke(rocket, null);
                float mass = rocket.rb2d.mass;
                if (mass > 200f)
                    torque /= Mathf.Pow(mass / 200f, 0.35f);

                float maxAcceleration = torque * Mathf.Rad2Deg / mass;
                float stoppingTime = Mathf.Abs(angularVelocity / maxAcceleration);
                float timeToTarget = (Mathf.Abs(angularVelocity) > 0.001f) ? Mathf.Abs(error / angularVelocity) : float.PositiveInfinity;

                if (float.IsInfinity(timeToTarget) || stoppingTime > timeToTarget)
                    turn = Mathf.Sign(angularVelocity);
                else
                    turn = -Mathf.Sign(error);
            }
            catch
            {
                float pGain = 0.8f;
                float dGain = 0.5f;
                turn = (-error * pGain - angularVelocity * dGain) / 30f;
            }

            turn = Mathf.Clamp(turn, -1f, 1f);
            rocket.arrowkeys.turnAxis.Value = turn;
        }

        private double CalculateMaxThrust()
        {
            double thrust = 0;
            foreach (var engine in rocket.partHolder.GetModules<EngineModule>())
                if (engine.engineOn.Value)
                    thrust += engine.thrust.Value;
            foreach (var booster in rocket.partHolder.GetModules<BoosterModule>())
                if (booster.enabled)
                    thrust += booster.thrustVector.Value.magnitude;
            return thrust * 9.8;
        }

        private double CalculateMaxAcceleration()
        {
            double thrust = 0;
            foreach (var engine in rocket.partHolder.GetModules<EngineModule>())
                if (engine.engineOn.Value)
                    thrust += engine.thrust.Value * 9.8;
            foreach (var booster in rocket.partHolder.GetModules<BoosterModule>())
                if (booster.enabled)
                    thrust += booster.thrustVector.Value.magnitude * 9.8;
            double mass = rocket.mass.GetMass();
            if (mass <= 0.00001) return 0.0;
            return thrust / mass;
        }

        private double GetPeriapsis()
        {
            var orbit = GetCurrentOrbit();
            if (orbit == null) return 0;
            return orbit.periapsis;
        }

        private Orbit GetCurrentOrbit()
        {
            if (rocket?.mapPlayer?.Trajectory == null) return null;
            var paths = rocket.mapPlayer.Trajectory.paths;
            if (paths.Count == 0) return null;
            return paths[0] as Orbit;
        }

        private double GetHorizontalSpeed()
        {
            Double2 horizontalDirection = GetHorizontalDirection();
            return Double2.Dot(rocket.location.velocity.Value, horizontalDirection);
        }

        private float GetHorizontalAngle()
        {
            Double2 horizontalDirection = GetHorizontalDirection();
            return (float)(horizontalDirection.AngleRadians * Mathf.Rad2Deg);
        }

        private Double2 GetHorizontalDirection()
        {
            Double2 radialDirection = rocket.location.position.Value.normalized;
            if (radialDirection.sqrMagnitude < 1E-10)
                return Double2.right;
            return new Double2(-radialDirection.y, radialDirection.x) * GetOrbitalDirectionSign();
        }

        private double GetOrbitalDirectionSign()
        {
            Double3 cross = Double3.Cross(rocket.location.position.Value, rocket.location.velocity.Value);
            return (cross.z < 0.0) ? -1.0 : 1.0;
        }
    }
}