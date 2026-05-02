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
    public enum AscentState
    {
        Idle,
        AwaitingLaunch,
        Liftoff,
        PitchOver,
        Coast,
        Circularize,
        TransferBurn
    }

    public class AscentAutopilot
    {
        private Rocket rocket;
        private readonly RocketManager rocketManager;
        private AscentState state = AscentState.Idle;
        private float targetAltitude;          // (RU) метры над поверхностью | (EN) meters above surface
        private float requestedTargetAltitude; // (RU) окончательная пользовательская запрошенная высота орбиты | (EN) final user-requested orbit altitude
        private double targetRadius;            // (RU) целевая высота + радиус планеты | (EN) target altitude + planet radius
        private bool pendingTransferBurn;

        private const float MIN_TARGET_ALTITUDE = 1000f;
        private const float DIRECT_ASCENT_MAX_EXTRA_ALTITUDE = 80000f;
        private const float PARKING_ORBIT_MIN_ALTITUDE = 5000f;
        private const float PARKING_ORBIT_ATMOSPHERE_MARGIN = 5000f;
        private const double COAST_TO_CIRC_MIN_LEAD_TIME = 0.75;
        private const double COAST_TO_CIRC_BURN_BUFFER = 0.35;
        private const float THROTTLE_SNAP_THRESHOLD = 0.01f;

        // (RU) Параметры подъёма (жёстко заданы) | (EN) Lifting parameters (hard-coded)
        private const float PITCH_START_ALTITUDE = 100f;      // (RU) начинаем поворот на 100 м | (EN) we begin the 100 m turn
        private const float TURN_END_ANGLE = 0f;              // (RU) заканчиваем горизонтально (0°) | (EN) we finish horizontally (0°)
        private const float TURN_SHAPE_EXPONENT = 0.8f;        // (RU) степень для кривой поворота | (EN) degree for the turning curve
        private const float MIN_PITCH = 5f;                    // (RU) минимальный угол тангажа | (EN) minimum pitch angle
        private const double TURN_TARGET_ALTITUDE_FACTOR = 0.45;
        private const double TURN_ATMOSPHERE_ALTITUDE_FACTOR = 0.9;
        private const double TURN_NO_ATMOSPHERE_ALTITUDE_FACTOR = 0.35;
        private const double TURN_MIN_END_ALTITUDE = 2500.0;

        // (RU) Параметры управления орбитой | (EN) Orbit control parameters
        private const double APOAPSIS_TARGET_MARGIN = 100;     // (RU) отключаем двигатели при достижении цели с недолётом 100 м (для первой фазы) | (EN) We turn off the engines when we reach the target within 100 m (for the first phase)
        private const double PERIAPSIS_TOLERANCE = 250;       // (RU) допустимая погрешность для периапсиса (250 м) | (EN) permissible error for periapsis (250 m)
        private const double CORRECTION_GAIN = 40000;          // (RU) коэффициент преобразования ошибки периапсиса в градусы | (EN) conversion factor of periapsis error to degrees
        private const float PERI_MAX_PITCH_CORRECTION = 2.0f;  // (RU) чтобы не уводить слишком далеко от prograde | (EN) so as not to lead too far away from prograde
        private const float APO_MAX_PITCH_CORRECTION = 5.0f;   // (RU) небольшая коррекция, но без жёсткого насыщения | (EN) a slight correction, but without hard saturation
        private const double APOAPSIS_OVER_CORRECTION_GAIN = 20000; // (RU) для коррекции по апогею (больше => мягче) | (EN) for apogee correction (more => softer)
        private const double APO_THROTTLE_REDUCTION_START = 250; // (RU) начинаем снижать тягу при заметном превышении апогея | (EN) we begin to reduce thrust when the apogee is noticeably exceeded
        private const double APO_THROTTLE_REDUCTION_END = 1500;  // (RU) снижать плавнее, чтобы не сорвать набор delta-V | (EN) descend more smoothly to avoid disrupting delta-V gain
        private const float APO_MIN_THROTTLE_FACTOR = 0.25f;     // (RU) минимальная доля тяги, чтобы продолжить набор delta-V | (EN) minimum thrust fraction to continue delta-V gain
        private const double COAST_TO_CIRC_THRESHOLD = 2.5;    // (RU) начинаем циркуляризацию за 2.5 секунды до апогея | (EN) We begin circularization 2.5 seconds before the apogee.
        private const double COAST_TO_CIRC_DISTANCE = 200;     // (RU) или за 200 м по высоте | (EN) or 200 m in altitude
        private const double CIRCULARIZE_MAX_DURATION = 90.0; // (RU) страховка от бесконечного горения | (EN) insurance against endless burning
        private const float CIRCULARIZE_PITCH_UP_BIAS = 0.6f;        // (RU) небольшой bias “вверх” | (EN) slight upward bias
        private const float CIRCULARIZE_PITCH_UP_BIAS_IF_APO_HIGH = 1.2f; // (RU) усиление, если апогей уже выше цели | (EN) gain if the apogee is already above the target
        private const double CIRCULARIZE_VEL_TOLERANCE = 2.0;          // (RU) m/s, “почти круговая скорость” | (EN) m/s, “near circular speed”
        private const double PERI_THROTTLE_GAIN = 120000.0;            // (RU) meters -> throttle (больше => слабее) | (EN) meters -> throttle (more => weaker)
        private const float PERI_THROTTLE_MIN = 0.0f;                  // (RU) даем тяге падать почти до нуля для точной доводки | (EN) let the thrust drop to almost zero for fine tuning
        private const double CIRCULARIZE_NEAR_APO_DISTANCE = 5000.0;  // (RU) м: доводим перицентр только очень близко к апогею | (EN) m: we bring the periapsis only very close to the apogee
        private const double CIRCULARIZE_NEAR_APO_TIME = 8.0;
        private const double CIRCULARIZE_BURN_WINDOW_DISTANCE = 12000.0;
        private const double CIRCULARIZE_BURN_WINDOW_TIME = 14.0;
        private const double CIRCULARIZE_PERI_CORRECTION_DISTANCE = 250.0;
        private const double CIRCULARIZE_PERI_CORRECTION_LEAD_TIME = 0.35;
        private const double MAJOR_PERIAPSIS_RAISE_MIN_ERROR = 3000.0;
        private const double MAJOR_APOAPSIS_CORRECTION_GAIN = 8000.0;
        private const float MAJOR_APOAPSIS_MAX_PITCH_CORRECTION = 1.5f;
        private const double CIRCULARIZE_RETRY_COAST_TIME = 25.0;
        private const double GUIDED_BURN_SETTLE_TIME = 0.75;
        private const float GUIDED_BURN_PITCH_TOLERANCE = 2.0f;
        private const float GUIDED_BURN_ANGULAR_VELOCITY_TOLERANCE = 4.0f;
        private const double CLOSE_ENOUGH_APOAPSIS_TOLERANCE = 10000.0;
        private const double ACTIVE_THRUST_THRESHOLD = 0.001;
        private const double STAGING_NO_THRUST_CONFIRMATION_TIME = 0.5;
        private const double STAGING_PASSIVE_CONFIRMATION_TIME = 0.75;
        private const float LOW_TWR_LIMIT = 1.45f;
        private const float VERY_LOW_TWR_LIMIT = 1.15f;
        private const float LOW_TWR_MIN_PROGRADE_UP_BIAS = 2.5f;
        private const float LOW_TWR_MAX_PROGRADE_UP_BIAS = 8.0f;

        private double burnDuration;
        private double circularizeStartWorldTime;
        private double deltaVTarget;
        private double circularizeEntryWorldTime;
        private double throttleUnlockWorldTime;
        private bool circularizeWaitingForNextPass;
        // private bool enginesInitialized = false;
        // private Stage currentStage = null;
        private int lastStageAttemptId = int.MinValue;
        private int lastStageAttemptCount = 0;
        private double lastStageAttemptWorldTime = double.NegativeInfinity;
        private double noThrustSinceWorldTime = double.NegativeInfinity;
        private double actualTurnStartAltitude = 0;
        private double launchPadAltitude = 0;
        private int launchQueuedStageId = int.MinValue;
        private readonly HashSet<Part> launchStagePropulsionParts = new HashSet<Part>();
        private double atmosphereHeight = 0;                   // (RU) высота атмосферы текущей планеты | (EN) the altitude of the current planet's atmosphere

        private bool debug = true;
        private int coastLogCounter = 0;
        private double cachedMaxAcceleration = 0; // (RU) Последнее известное ненулевое ускорение — для использования когда двигатели не горят | (EN) Last known non-zero acceleration — used when engines are not burning

        public bool IsActive { get; private set; }

        public AscentAutopilot(Rocket rocket)
        {
            this.rocket = rocket;
            rocketManager = new RocketManager(rocket)
            {
                Debug = debug,
                PitchTolerance = GUIDED_BURN_PITCH_TOLERANCE,
                AngularVelocityLimit = GUIDED_BURN_ANGULAR_VELOCITY_TOLERANCE,
                ThrottleSnapThreshold = THROTTLE_SNAP_THRESHOLD,
                StageCooldown = 1.0
            };
            RefreshRuntimeSettings();

            if (rocket != null && rocket.location.planet.Value != null)
            {
                atmosphereHeight = rocket.location.planet.Value.AtmosphereHeightPhysics;
                if (debug)
                {
                    Debug.Log($"[Autopilot] Difficulty: {Base.worldBase.settings.difficulty.difficulty}, Atmosphere height: {atmosphereHeight}m");
                }
            }

            if (debug) Debug.Log("[Autopilot] AscentAutopilot initialized, ModVersion: 0.9.0");
        }

        public void SetRocket(Rocket rocket)
        {
            this.rocket = rocket;
            rocketManager.SetRocket(rocket);
            RefreshRuntimeSettings();
        }

        public void Start()
        {
            if (rocket == null) return;
            RefreshRuntimeSettings();
            InitializeMissionTargets();
            IsActive = true;
            state = AscentState.AwaitingLaunch;
            // enginesInitialized = false;
            // currentStage = null;
            lastStageAttemptId = int.MinValue;
            lastStageAttemptCount = 0;
            lastStageAttemptWorldTime = double.NegativeInfinity;
            noThrustSinceWorldTime = double.NegativeInfinity;
            actualTurnStartAltitude = 0;
            launchPadAltitude = rocket.location.Value.Radius - rocket.location.planet.Value.Radius;
            launchQueuedStageId = rocket.staging.stages.Count > 0 ? rocket.staging.stages[0].stageId : int.MinValue;
            CacheLaunchStagePropulsionParts();
            throttleUnlockWorldTime = 0.0;
            circularizeWaitingForNextPass = false;
            rocketManager.ResetStagingHistory();
            if (debug) Debug.Log("[Autopilot] STARTED (awaiting manual stage)");
            GUI.RefreshAscentToggleButtonLabel();
        }

        public void Stop()
        {
            IsActive = false;
            state = AscentState.Idle;
            if (rocket != null)
            {
                rocket.throttle.throttleOn.Value = false;
                rocket.throttle.throttlePercent.Value = 0f;
                rocket.arrowkeys.turnAxis.Value = 0f;
            }
            circularizeWaitingForNextPass = false;
            noThrustSinceWorldTime = double.NegativeInfinity;
            launchQueuedStageId = int.MinValue;
            launchStagePropulsionParts.Clear();
            if (debug) Debug.Log("[Autopilot] STOPPED");
            GUI.RefreshAscentToggleButtonLabel();
        }

        private void RefreshRuntimeSettings(bool logTargetChanges = false)
        {
            float previousRequestedTargetAltitude = requestedTargetAltitude;
            requestedTargetAltitude = Mathf.Max(MIN_TARGET_ALTITUDE, Settings.data.targetOrbitAltitude);

            if (rocket != null && rocket.location.planet.Value != null)
            {
                atmosphereHeight = rocket.location.planet.Value.AtmosphereHeightPhysics;
            }

            if (!IsActive)
            {
                targetAltitude = GetInitialInsertionTargetAltitude(requestedTargetAltitude);
                pendingTransferBurn = requestedTargetAltitude > targetAltitude + 0.1f;
            }

            if (logTargetChanges && debug && Math.Abs(previousRequestedTargetAltitude - requestedTargetAltitude) > 0.1f)
            {
                Debug.Log($"[Autopilot] Target orbit request updated to {requestedTargetAltitude:F0}m");
            }
        }

        public void Update() { }

        public void FixedUpdate()
        {
            if (!IsActive || rocket == null) return;
            RefreshRuntimeSettings(logTargetChanges: true);

            // (RU) Если ступеней не осталось, автопилот не может работать | (EN) If no stages remain, the autopilot cannot operate
            // Major Bug
            if (rocket.staging.stages.Count == 0)
            {
                bool hasThrust = HasAnyActiveThrust();
                if (!hasThrust)
                {
                    if (debug) Debug.Log("[Autopilot] No stages and no thrust, stopping autopilot");
                    Stop();
                    return;
                }
            }

            double altitude = rocket.location.Value.Radius - rocket.location.planet.Value.Radius;
            double velocity = rocket.location.velocity.Value.magnitude;
            double planetRadius = rocket.location.planet.Value.Radius;
            targetRadius = targetAltitude + planetRadius;

            double rawApoapsis = GetApoapsis();
            double rawPeriapsis = GetPeriapsis();
            double apoapsis = rawApoapsis;
            double periapsis = rawPeriapsis;

            double apoapsisAltitude = (rawApoapsis > planetRadius) ? rawApoapsis - planetRadius : 0;
            double periapsisAltitude = (rawPeriapsis > planetRadius) ? rawPeriapsis - planetRadius : 0;

            bool aboveAtmosphere = altitude > atmosphereHeight;
            SuppressPrematureUpperStagePropulsion();

            if (debug && UnityEngine.Time.frameCount % 60 == 0)
            {
                Debug.Log($"[Autopilot] State: {state}, Alt: {altitude:F0}m, RawApo: {rawApoapsis:F0}m, ApoAlt: {apoapsisAltitude:F0}m, PerAlt: {periapsisAltitude:F0}m, TargetRadius: {targetRadius:F0}m, Vel: {velocity:F1}m/s, TurnAxis: {rocket.arrowkeys.turnAxis.Value:F2}, aboveAtmo: {aboveAtmosphere}");
            }

            // Stage newStage = (rocket.staging.stages.Count > 0) ? rocket.staging.stages[0] : null;
            // if (newStage != currentStage)
            // {
            //     if (debug) Debug.Log($"[Autopilot] Stage changed from {currentStage?.stageId} to {newStage?.stageId}");
            //     currentStage = newStage;
            //     enginesInitialized = false;
            // }

            // if (!enginesInitialized && currentStage != null)
            // {
            //     InitializeEngines();
            //     enginesInitialized = true;
            // }

            CheckStaging(aboveAtmosphere);

            switch (state)
            {
                case AscentState.AwaitingLaunch:
                    SetThrottle(0f);
                    SetPitch(90f);
                    bool launchStageChanged = rocket.staging.stages.Count == 0 ||
                        rocket.staging.stages[0].stageId != launchQueuedStageId;
                    if (launchStageChanged)
                    {
                        if (debug) Debug.Log("[Autopilot] Manual stage detected, entering Liftoff");
                        SetThrottle(1f);
                        state = AscentState.Liftoff;
                    }
                    break;

                case AscentState.Liftoff:
                    SetThrottle(1f);
                    SetPitch(90f);
                    if (altitude > PITCH_START_ALTITUDE)
                    {
                        if (debug) Debug.Log($"[Autopilot] Liftoff -> PitchOver at alt={altitude:F0}m");
                        actualTurnStartAltitude = altitude;
                        state = AscentState.PitchOver;
                    }
                    break;

                case AscentState.PitchOver:
                    float currentTwr = GetCurrentTwrEstimate();
                    double turnEndAltitude = GetPitchProgramEndAltitude();
                    double turnProgress = (altitude - actualTurnStartAltitude) / (turnEndAltitude - actualTurnStartAltitude);
                    turnProgress = Math.Max(0, Math.Min(1, turnProgress));
                    float targetPitch;
                    if (turnProgress < 1)
                    {
                        targetPitch = 90f - (float)Math.Pow(turnProgress, TURN_SHAPE_EXPONENT) * (90f - TURN_END_ANGLE);
                        targetPitch = Math.Max(targetPitch, MIN_PITCH);
                    }
                    else
                    {
                        targetPitch = TURN_END_ANGLE;
                    }

                    if (rocket.location.velocity.Value.magnitude > 0.1)
                    {
                        float progradePitch = (float)Math.Atan2(rocket.location.velocity.Value.y, rocket.location.velocity.Value.x) * Mathf.Rad2Deg;
                        float minimumPitch = progradePitch;
                        if (currentTwr > 0.01f && currentTwr < LOW_TWR_LIMIT && apoapsis < targetRadius - APOAPSIS_TARGET_MARGIN)
                        {
                            float twrProgress = Mathf.InverseLerp(VERY_LOW_TWR_LIMIT, LOW_TWR_LIMIT, currentTwr);
                            float lowTwrBias = Mathf.Lerp(LOW_TWR_MAX_PROGRADE_UP_BIAS, LOW_TWR_MIN_PROGRADE_UP_BIAS, twrProgress);
                            minimumPitch += lowTwrBias;
                        }

                        if ((apoapsis >= planetRadius + atmosphereHeight || altitude >= atmosphereHeight * 0.4 || currentTwr < LOW_TWR_LIMIT) &&
                            NormalizeAngle(targetPitch - minimumPitch) < 0f)
                        {
                            targetPitch = minimumPitch;
                        }
                    }
                    SetPitch(targetPitch);

                    float throttle = CalculateThrottleDynamic(apoapsis, targetRadius);
                    if (currentTwr > 0.01f && currentTwr < LOW_TWR_LIMIT && apoapsis < targetRadius - 1000.0)
                    {
                        throttle = 1f;
                    }
                    SetThrottle(throttle);

                    if (debug && UnityEngine.Time.frameCount % 60 == 0)
                        Debug.Log($"[Autopilot] PitchOver: targetPitch={targetPitch:F1}, progress={turnProgress:F3}, turnEndAlt={turnEndAltitude:F0}m, throttle={throttle:F2}");

                    // (RU) Отключаем двигатели при достижении целевого апогея с недолётом 500 м | (EN) We shut down the engines when we reach the target apogee with a shortfall of 500 m.
                    if (apoapsis >= targetRadius - APOAPSIS_TARGET_MARGIN)
                    {
                        if (CanSafelyCoastToNextApoapsis(periapsis))
                            BeginCoast(rawApoapsis, apoapsisAltitude, "PitchOver -> Coast");
                        else
                            BeginCircularize("PitchOver -> Circularize", altitude, apoapsisAltitude);
                    }
                    break;

                case AscentState.Coast:
                    rocket.arrowkeys.turnAxis.Value = 0f;

                    coastLogCounter++;
                    if (debug && coastLogCounter % 60 == 0)
                    {
                        double tta = GetTimeToApoapsis();
                        Debug.Log($"[Autopilot] Coast: TTA={tta:F2}s, aboveAtmo={aboveAtmosphere}");
                    }

                    // (RU) Включаем ускорение времени только если мы выше атмосферы и можно ускорять | (EN) We turn on time acceleration only if we are above the atmosphere and can accelerate
                    if (aboveAtmosphere && CanUseTimewarp())
                    {
                        double timeToApoapsis = GetTimeToApoapsis();
                        double requiredDeltaVForBurn = EstimateCircularizationDeltaVAtApoapsis(out _, out _, out _);
                        double burnLeadTime = Math.Max(COAST_TO_CIRC_MIN_LEAD_TIME, EstimateCircularizationBurnDuration(requiredDeltaVForBurn) * 0.5 + COAST_TO_CIRC_BURN_BUFFER);
                        double timeToWarpStop = timeToApoapsis - burnLeadTime;
                        if (timeToWarpStop > 120)
                        {
                            WorldTime.main.SetState(WorldTime.MaxTimewarpSpeed, false, false);
                        }
                        else if (timeToWarpStop > 60)
                        {
                            WorldTime.main.SetState(50, false, false);
                        }
                        else if (timeToWarpStop > 30)
                        {
                            WorldTime.main.SetState(25, false, false);
                        }
                        else if (timeToWarpStop > 15)
                        {
                            WorldTime.main.SetState(10, false, false);
                        }
                        else if (timeToWarpStop > 8)
                        {
                            WorldTime.main.SetState(5, false, false);
                        }
                        else if (WorldTime.main.timewarpSpeed > 1)
                        {
                            WorldTime.main.StopTimewarp(false);
                        }
                    }
                    else
                    {
                        if (WorldTime.main.timewarpSpeed > 1)
                            WorldTime.main.StopTimewarp(false);
                    }

                    double timeToApo = GetTimeToApoapsis();
                    double distanceToApo = Math.Abs(apoapsisAltitude - altitude);
                    double requiredDeltaV = EstimateCircularizationDeltaVAtApoapsis(out _, out _, out _);
                    double estimatedBurnDuration = EstimateCircularizationBurnDuration(requiredDeltaV);
                    double coastLeadTime = Math.Max(COAST_TO_CIRC_MIN_LEAD_TIME, estimatedBurnDuration * 0.5 + COAST_TO_CIRC_BURN_BUFFER);
                    double timeToBurnStart = timeToApo - coastLeadTime;

                    // (RU) Начинаем циркуляризацию, когда осталось меньше заданного времени ИЛИ расстояние по высоте мало | (EN) We start circularization when less than the specified time remains OR the height distance is small
                    if (((timeToBurnStart <= 0.0) && WorldTime.main.timewarpSpeed <= 1) || distanceToApo < COAST_TO_CIRC_DISTANCE)
                    {
                        BeginCircularize("Coast -> Circularize", altitude, apoapsisAltitude);
                        break;
#if false
                        // (RU) Оцениваем, сколько нужно гореть. Внутри SFS значения тяги/ускорения могут кратковременно равняться нулю. | (EN) Estimate how long we need to burn. Depending on SFS internals, thrust/accel values can briefly be 0.
                        // (RU) Это может случиться сразу после отключения двигателей, поэтому используем запасной запас времени. | (EN) This can happen right after we cut engines, so keep a safe fallback duration.
                        double mass = rocket.mass.GetMass();
                        double acceleration = CalculateMaxAcceleration();
                        if (acceleration <= 0.00001 && mass > 0.0)
                        {
                            double maxThrust = CalculateMaxThrust();
                            acceleration = maxThrust / mass;
                        }
                        burnDuration = (acceleration > 0.00001) ? (deltaVTarget / acceleration) : estimatedBurnDuration;
                        if (burnDuration <= 0.0 && deltaVTarget > 0.0)
                        {
                            burnDuration = 10.0; // (RU) запасные секунды | (EN) fallback seconds
                            if (debug) Debug.Log($"[Autopilot] Acceleration estimate was 0; using fallback burnDuration={burnDuration:F1}s");
                        }
                        circularizeStartWorldTime = WorldTime.main.worldTime;
                        circularizeEntryWorldTime = circularizeStartWorldTime;
                        throttleUnlockWorldTime = circularizeStartWorldTime + GUIDED_BURN_SETTLE_TIME;
                        if (debug)
                            Debug.Log($"[Autopilot] Required ΔV: {requiredDeltaV:F1}m/s, using ΔVtarget={deltaVTarget:F1}m/s, burn duration: {burnDuration:F1}s, horizVel={horizontalSpeed:F1}m/s, targetVel={targetOrbitalSpeed:F1}m/s, maxAccel est={CalculateMaxAcceleration():F2}m/s²");

                        // (RU) Если по тем или иным причинам у нас уже достаточно скорости для круговой орбиты на целевом радиусе, не горим. | (EN) If we somehow already have enough speed for circular orbit at target radius, don't burn.
                        if (deltaVTarget <= 0.0001)
                        {
                            CutEngines();
                            if (pendingTransferBurn)
                                BeginTransferBurn();
                            else
                                Stop();
                        }
#endif
                    }
                    break;

                case AscentState.Circularize:
                    // (RU) Основное направление – prograde (по вектору скорости) с коррекциями | (EN) The main direction is prograde (along the velocity vector) with corrections
                    float targetPitchCirc = rocket.GetRotation();
                    bool orientationReady = false;
                    if (circularizeWaitingForNextPass)
                    {
                        rocket.arrowkeys.turnAxis.Value = 0f;
                        orientationReady = true;
                    }
                    else if (rocket.location.velocity.Value.magnitude > 0.1)
                    {
                        float progradeAngle = (float)Math.Atan2(rocket.location.velocity.Value.y, rocket.location.velocity.Value.x) * Mathf.Rad2Deg;
                        float horizontalAngle = GetHorizontalAngle();
                        targetPitchCirc = Mathf.LerpAngle(progradeAngle, horizontalAngle, 0.35f);

                        double periError = targetRadius - periapsis;
                        bool needMajorPeriRaiseForPitch = periError > Math.Max(MAJOR_PERIAPSIS_RAISE_MIN_ERROR, Math.Min(50000.0, targetAltitude * 0.05));
                        targetPitchCirc = Mathf.LerpAngle(progradeAngle, horizontalAngle, needMajorPeriRaiseForPitch ? 0.8f : 0.35f);
                        double apoError = targetRadius - apoapsis; // (RU) отрицательное, если апогей выше цели | (EN) negative if the apogee is above the target

                        // (RU) Коррекция по перицентру (симметрично: поднимаем нос если ниже цели, опускаем если выше) | (EN) Pericenter correction (symmetrically: raise the nose if it is below the target, lower it if it is above)
                        if (Math.Abs(periError) > PERIAPSIS_TOLERANCE)
                        {
                            float periCorr = Mathf.Clamp((float)(periError / CORRECTION_GAIN), -PERI_MAX_PITCH_CORRECTION, PERI_MAX_PITCH_CORRECTION);
                            targetPitchCirc += periCorr;
                            if (debug && UnityEngine.Time.frameCount % 60 == 0)
                                Debug.Log($"[Autopilot] Peri correction={periCorr:F2}° (periErr={periError:F0}m)");
                        }

                        // (RU) Коррекция по апогею: | (EN) Apogee correction:
                        // (RU) apoError = targetRadius - apoapsis; отрицательно => апогей ВЫШЕ цели. | (EN) apoError = targetRadius - apoapsis; negative => apoapsis is ABOVE target.
                        // (RU) Важно: знак в старой версии приводил к тому, что коррекция почти всегда "обнулялась" clamping'ом. | (EN) Important: in the old version, the sign resulted in the correction almost always being "zeroed out" by clamping.
                        double apoOvershoot = apoapsis - targetRadius; // (RU) положительно => апогей выше цели | (EN) positive => apogee above the target
                        if (Math.Abs(apoOvershoot) > APOAPSIS_TARGET_MARGIN)
                        {
                            double apoCorrectionGain = needMajorPeriRaiseForPitch ? MAJOR_APOAPSIS_CORRECTION_GAIN : APOAPSIS_OVER_CORRECTION_GAIN;
                            float apoCorrectionLimit = needMajorPeriRaiseForPitch ? MAJOR_APOAPSIS_MAX_PITCH_CORRECTION : APO_MAX_PITCH_CORRECTION;
                            float apoCorrMag = Mathf.Clamp((float)(Math.Abs(apoOvershoot) / apoCorrectionGain), 0f, apoCorrectionLimit);
                            // (RU) Если апогей УЖЕ выше цели, нам нужно "сдерживать" его, | (EN) If the apogee is already above the target, we need to restrain it,
                            // (RU) поэтому при overshoot направляем коррекцию в противоположную сторону. | (EN) therefore for an overshoot we steer the correction in the opposite direction.
                            if (apoOvershoot > 0)
                                targetPitchCirc -= apoCorrMag;
                            else
                                targetPitchCirc += apoCorrMag;

                            if (debug && apoCorrMag > 0.01f)
                                Debug.Log($"[Autopilot] Apo pitch correction mag={apoCorrMag:F2}°, overshoot={apoOvershoot:F0}m");
                        }

                        // (RU) Небольшой bias “вверх” чтобы снизить склонность к уходу в баллистическую траекторию. | (EN) Small upward bias to reduce tendency to fall into a ballistic trajectory.
                        // (RU) Если апогей уже превышает цель, делаем bias сильнее. | (EN) If the apogee already exceeds the target, make the bias stronger.
                        double apoOvershootForBias = apoapsis - targetRadius; // (RU) положительно => апогей выше цели | (EN) positive => apogee above the target
                        float pitchBias = CIRCULARIZE_PITCH_UP_BIAS;
                        if (needMajorPeriRaiseForPitch || apoOvershootForBias > APO_THROTTLE_REDUCTION_START)
                            pitchBias = 0f;

                        targetPitchCirc += pitchBias;

                        SetPitch(targetPitchCirc);
                        orientationReady = IsPitchSettled(targetPitchCirc);
                    }

                    // (RU) Управление тягой: | (EN) Throttle control:
                    // (RU) целимся не в "заранее вычисленный" delta-V, а в скорость на текущем апоапсисе, | (EN) we target not a precomputed delta-V but the speed at the current apoapsis,
                    // (RU) чтобы при прохождении апоапсиса скорость совпала со скоростью круговой орбиты. | (EN) so that when passing apoapsis the speed matches circular orbital speed.
                    bool hasActiveThrustNow = HasAnyActiveThrust();
                    double maxAccel = CalculateMaxAcceleration();
                    // (RU) Используем кэш только на коротком переходе между поджигом и появлением тяги, но не когда тяги реально уже нет. | (EN) Use cached accel only for short ignition transitions, not when thrust is genuinely gone.
                    if (maxAccel < 0.001 &&
                        hasActiveThrustNow &&
                        cachedMaxAcceleration > 0.001)
                    {
                        maxAccel = cachedMaxAcceleration;
                    }
                    double rawTimeToApoCirc = GetTimeToApoapsis();
                    double timeToApoCirc = rawTimeToApoCirc;
                    if (double.IsInfinity(timeToApoCirc) || double.IsNaN(timeToApoCirc)) timeToApoCirc = 2.0;
                    double currentRadiusCirc = rocket.location.Value.Radius;
                    double horizontalSpeedCirc = GetHorizontalSpeed();
                    double radialSpeedCirc = GetRadialSpeed();
                    // (RU) v_circ = sqrt(mu / r_apo). Целимся в круговую скорость на реальном апогее, а не на целевом радиусе. | (EN) Target circular speed at actual apoapsis, not vis-viva speed for the target SMA.
                    double mu = rocket.location.planet.Value.mass;
                    double rApo = Math.Max(apoapsis, rocket.location.planet.Value.Radius + 1.0);
                    double targetOrbitalSpeedCirc = Math.Sqrt(mu / rApo);
                    double dvErrorNow = targetOrbitalSpeedCirc - velocity; // (RU) нужно набрать, если положительное | (EN) positive => need to gain

                    // (RU) Дополнительно: если перицентр ниже цели, продолжаем гореть даже если dvErrorNow ~ 0, | (EN) Additionally: if periapsis is below the target, continue burning even if dvErrorNow ~ 0,
                    // (RU) потому что именно это “добивает” перицентр и повышает точность круговой орбиты. | (EN) because this is what "finishes" the periapsis and improves circularization accuracy.
                    double periErrForThrottle = targetRadius - periapsis; // (RU) >0 => перицентр ниже цели | (EN) >0 => periapsis below target
                    bool needMajorPeriRaise = periErrForThrottle > Math.Max(MAJOR_PERIAPSIS_RAISE_MIN_ERROR, Math.Min(50000.0, targetAltitude * 0.05));
                    bool needUsefulPeriRaise = periErrForThrottle > Math.Max(PERIAPSIS_TOLERANCE * 4.0, 1500.0);
                    double closeEnoughVelTolerance = targetAltitude <= 100000f ? 3.0 : 4.0;
                    double closeEnoughPeriTolerance = targetAltitude <= 100000f
                        ? 3500.0
                        : Math.Max(3000.0, Math.Min(8000.0, targetAltitude * 0.02));
                    double closeEnoughApoTolerance = targetAltitude <= 100000f
                        ? 3000.0
                        : Math.Max(2500.0, Math.Min(25000.0, targetAltitude * 0.03));
                    dvErrorNow = targetOrbitalSpeedCirc - horizontalSpeedCirc;
                    double burnLeadTimeCirc = Math.Max(CIRCULARIZE_BURN_WINDOW_TIME, EstimateCircularizationBurnDuration(Math.Max(dvErrorNow, 0.0)) * 0.5 + COAST_TO_CIRC_BURN_BUFFER);
                    bool safeToCoastToNextApoapsis = CanSafelyCoastToNextApoapsis(periapsis);
                    bool continueCurrentPassBurn = radialSpeedCirc < 0.0 &&
                        !safeToCoastToNextApoapsis &&
                        periErrForThrottle > closeEnoughPeriTolerance;
                    float periThrottle = 0f;
                    double altitudeToApoCirc = apoapsisAltitude - altitude;
                    double distanceToApoCirc = Math.Abs(altitudeToApoCirc);
                    bool nearApo = rawTimeToApoCirc <= CIRCULARIZE_NEAR_APO_TIME ||
                        (distanceToApoCirc <= CIRCULARIZE_NEAR_APO_DISTANCE &&
                         radialSpeedCirc >= 0.0 &&
                         rawTimeToApoCirc <= CIRCULARIZE_NEAR_APO_TIME * 2.0);
                    bool inBurnWindow = continueCurrentPassBurn ||
                        rawTimeToApoCirc <= burnLeadTimeCirc ||
                        (distanceToApoCirc <= CIRCULARIZE_BURN_WINDOW_DISTANCE &&
                         radialSpeedCirc >= 0.0 &&
                         rawTimeToApoCirc <= burnLeadTimeCirc * 2.0);
                    bool canPeriCorrect = rawTimeToApoCirc <= CIRCULARIZE_PERI_CORRECTION_LEAD_TIME ||
                        (altitudeToApoCirc <= CIRCULARIZE_PERI_CORRECTION_DISTANCE && radialSpeedCirc >= 0.0) ||
                        continueCurrentPassBurn;
                    bool allowPeriCorrection = inBurnWindow &&
                        (needMajorPeriRaise || canPeriCorrect || continueCurrentPassBurn || (needUsefulPeriRaise && nearApo));
                    if (allowPeriCorrection && periErrForThrottle > PERIAPSIS_TOLERANCE)
                    {
                        float periMinThrottle = continueCurrentPassBurn
                            ? (needMajorPeriRaise ? 0.2f : 0.08f)
                            : (needMajorPeriRaise ? 0.05f : ((needUsefulPeriRaise && nearApo) ? 0.02f : PERI_THROTTLE_MIN));
                        float periMaxThrottle = continueCurrentPassBurn
                            ? (needMajorPeriRaise ? 1f : 0.6f)
                            : (needMajorPeriRaise ? 0.35f : 0.15f);
                        periThrottle = Mathf.Clamp((float)(periErrForThrottle / PERI_THROTTLE_GAIN), periMinThrottle, periMaxThrottle);
                    }

                    float circThrottle = 0f;
                    dvErrorNow = targetOrbitalSpeedCirc - horizontalSpeedCirc;
                    if (maxAccel > 0.00001 && dvErrorNow > 0.0 && rawTimeToApoCirc > -1.0 && inBurnWindow)
                    {
                        // (RU) Распределяем нужный прирост скорости на ближайшее окно до апогея. | (EN) Distribute the required delta-V over the nearest window until apoapsis.
                        double desiredTime = continueCurrentPassBurn
                            ? Math.Max(5.0, Math.Min(20.0, EstimateCircularizationBurnDuration(Math.Max(dvErrorNow, 0.0))))
                            : Math.Max(Math.Abs(rawTimeToApoCirc), 0.5);
                        double desiredAccel = dvErrorNow / desiredTime;
                        circThrottle = Mathf.Clamp01((float)(desiredAccel / maxAccel));

                        // (RU) Если апогей уже "уехал" выше цели, чуть притормаживаем, но не ломаем burn полностью. | (EN) If apoapsis has already gone above the target, slightly throttle down but don't break the burn completely.
                        double apoOvershoot = apoapsis - targetRadius;
                        if (!continueCurrentPassBurn && apoOvershoot > APO_THROTTLE_REDUCTION_START)
                        {
                            double reductionRange = needMajorPeriRaise ? Math.Max(5000.0, targetAltitude * 0.05) : (APO_THROTTLE_REDUCTION_END - APO_THROTTLE_REDUCTION_START);
                            double factor = 1.0 - (apoOvershoot - APO_THROTTLE_REDUCTION_START) / reductionRange;
                            factor = Math.Max(0.0, Math.Min(1.0, factor));
                            float applied = (float)Math.Max(factor, needMajorPeriRaise ? 0.35f : APO_MIN_THROTTLE_FACTOR);
                            circThrottle *= applied;
                        }

                    }
                    bool noUsableThrustNow = !hasActiveThrustNow && maxAccel < 0.001;

                    if (noUsableThrustNow &&
                        rocket.staging.stages.Count == 0 &&
                        (periErrForThrottle > closeEnoughPeriTolerance || Math.Abs(dvErrorNow) > closeEnoughVelTolerance))
                    {
                        if (debug)
                            Debug.Log("[Autopilot] No usable thrust remains during circularize, stopping autopilot");
                        PrepareForCoastOrWarp();
                        Stop();
                        break;
                    }

                    bool shouldWaitForNextPass = aboveAtmosphere &&
                        !noUsableThrustNow &&
                        !inBurnWindow &&
                        periErrForThrottle > closeEnoughPeriTolerance &&
                        Math.Abs(dvErrorNow) > closeEnoughVelTolerance;
                    if (shouldWaitForNextPass)
                    {
                        circularizeWaitingForNextPass = true;
                        circularizeEntryWorldTime = WorldTime.main.worldTime;
                        throttleUnlockWorldTime = WorldTime.main.worldTime;
                        PrepareForCoastOrWarp();

                        UpdateTimewarpToApoapsisWindow(rawTimeToApoCirc, burnLeadTimeCirc, aboveAtmosphere);

                        if (debug && UnityEngine.Time.frameCount % 60 == 0)
                            Debug.Log($"[Autopilot] Waiting for circularize burn, timeToApo={rawTimeToApoCirc:F1}s, periErr={periErrForThrottle:F0}m, lead={burnLeadTimeCirc:F1}s");
                        break;
                    }

                    if (circularizeWaitingForNextPass)
                    {
                        circularizeWaitingForNextPass = false;
                        circularizeEntryWorldTime = WorldTime.main.worldTime;
                        throttleUnlockWorldTime = WorldTime.main.worldTime + GUIDED_BURN_SETTLE_TIME;
                        orientationReady = false;
                    }

                    double apoOvershootForFinal = apoapsis - targetRadius;
                    float finalThrottle = 0f;
                    if (inBurnWindow)
                    {
                        finalThrottle = Mathf.Max(circThrottle, periThrottle);
                        if (!needMajorPeriRaise && apoOvershootForFinal > APO_THROTTLE_REDUCTION_START && dvErrorNow <= 0.0)
                        {
                            finalThrottle = Mathf.Min(finalThrottle, periThrottle);
                        }
                        if (!needMajorPeriRaise && apoOvershootForFinal > APO_THROTTLE_REDUCTION_END)
                        {
                            finalThrottle = Mathf.Min(finalThrottle, 0.02f);
                        }

                        if (needMajorPeriRaise && periErrForThrottle > Math.Max(closeEnoughPeriTolerance, 2000.0))
                        {
                            finalThrottle = Mathf.Max(finalThrottle, 0.05f);
                        }
                    }
                    else if (!needMajorPeriRaise && dvErrorNow > CIRCULARIZE_VEL_TOLERANCE && apoOvershootForFinal <= APO_THROTTLE_REDUCTION_START)
                    {
                        finalThrottle = Mathf.Min(circThrottle, 0.05f);
                    }

                    if (WorldTime.main.worldTime < throttleUnlockWorldTime)
                    {
                        finalThrottle = 0f;
                    }
                    else if (!orientationReady)
                    {
                        finalThrottle = continueCurrentPassBurn
                            ? Mathf.Min(finalThrottle, 0.35f)
                            : 0f;
                    }

                    bool orbitWithinTolerance = Math.Abs(periapsis - targetRadius) < PERIAPSIS_TOLERANCE &&
                        Math.Abs(targetOrbitalSpeedCirc - horizontalSpeedCirc) <= CIRCULARIZE_VEL_TOLERANCE;
                    bool orbitCloseEnough = Math.Abs(periapsis - targetRadius) <= closeEnoughPeriTolerance &&
                        Math.Abs(dvErrorNow) <= closeEnoughVelTolerance &&
                        Math.Abs(apoapsis - targetRadius) <= closeEnoughApoTolerance;

                    SetThrottle(finalThrottle);

                    bool anotherCircularizePassNeeded = !orbitWithinTolerance &&
                        !orbitCloseEnough &&
                        finalThrottle <= 0.0f &&
                        !inBurnWindow &&
                        WorldTime.main.worldTime >= throttleUnlockWorldTime &&
                        orientationReady &&
                        aboveAtmosphere &&
                        safeToCoastToNextApoapsis;
                    if (anotherCircularizePassNeeded)
                    {
                        circularizeWaitingForNextPass = true;
                        circularizeEntryWorldTime = WorldTime.main.worldTime;
                        PrepareForCoastOrWarp();
                        UpdateTimewarpToApoapsisWindow(rawTimeToApoCirc, burnLeadTimeCirc, aboveAtmosphere);
                        if (debug && UnityEngine.Time.frameCount % 60 == 0)
                            Debug.Log($"[Autopilot] Waiting for next safe apoapsis pass, timeToApo={rawTimeToApoCirc:F1}s, periErr={periErrForThrottle:F0}m");
                    }
                    else if (WorldTime.main.timewarpSpeed > 1)
                    {
                        WorldTime.main.StopTimewarp(false);
                    }

                    if (debug && UnityEngine.Time.frameCount % 30 == 0)
                        Debug.Log($"[Autopilot] Circularize throttle={finalThrottle:F2} (circ={circThrottle:F2}, peri={periThrottle:F2}), dvErrorNow={dvErrorNow:F1}m/s, horizVel={horizontalSpeedCirc:F1}m/s, targetVel={targetOrbitalSpeedCirc:F1}m/s, timeToApo={timeToApoCirc:F2}s, apoOvershoot={(apoapsis - targetRadius):F0}m, periErr={periErrForThrottle:F0}m, distToApo={distanceToApoCirc:F0}m, radialVel={radialSpeedCirc:F1}m/s, nearApo={nearApo}, burnWindow={inBurnWindow}, lead={burnLeadTimeCirc:F1}s, safeCoast={safeToCoastToNextApoapsis}, settled={orientationReady}");

                    // (RU) Завершение, когда перицентр достиг цели (допуск 1 км) | (EN) Finish when periapsis reaches the target (tolerance ~1 km)
                    closeEnoughPeriTolerance = targetAltitude <= 100000f
                        ? 3500.0
                        : Math.Max(3000.0, Math.Min(8000.0, targetAltitude * 0.02));
                    closeEnoughApoTolerance = targetAltitude <= 100000f
                        ? 3000.0
                        : Math.Max(2500.0, Math.Min(25000.0, targetAltitude * 0.03));
                    orbitWithinTolerance = Math.Abs(periapsis - targetRadius) < PERIAPSIS_TOLERANCE && Math.Abs(targetOrbitalSpeedCirc - horizontalSpeedCirc) <= CIRCULARIZE_VEL_TOLERANCE;
                    orbitCloseEnough = Math.Abs(periapsis - targetRadius) <= closeEnoughPeriTolerance &&
                        Math.Abs(dvErrorNow) <= closeEnoughVelTolerance &&
                        Math.Abs(apoapsis - targetRadius) <= closeEnoughApoTolerance;
                    if (orbitWithinTolerance || orbitCloseEnough)
                    {
                        if (debug)
                            Debug.Log(orbitWithinTolerance ? "[Autopilot] Circularize complete" : "[Autopilot] Circularize close enough");
                        CutEngines();
                        if (pendingTransferBurn)
                            BeginTransferBurn();
                        else
                            Stop();
                    }
                    else
                    {
                        // (RU) Защита от зависания управления | (EN) Safeguard against control hangs
                        double circElapsed = WorldTime.main.worldTime - circularizeEntryWorldTime;
                        if (circElapsed > CIRCULARIZE_MAX_DURATION)
                        {
                            if (debug)
                                Debug.Log($"[Autopilot] Circularize timeout after {circElapsed:F1}s. periErr={(periapsis - targetRadius):F1}m dvErrNow={(targetOrbitalSpeedCirc - horizontalSpeedCirc):F1}m/s");
                            CutEngines();
                            Stop();
                        }
                    }
                    break;

                case AscentState.TransferBurn:
                    float targetPitchTransfer = GetHorizontalAngle();
                    if (rocket.location.velocity.Value.magnitude > 0.1)
                    {
                        float progradeAngle = (float)Math.Atan2(rocket.location.velocity.Value.y, rocket.location.velocity.Value.x) * Mathf.Rad2Deg;
                        targetPitchTransfer = Mathf.LerpAngle(progradeAngle, targetPitchTransfer, 0.25f);
                    }

                    SetPitch(targetPitchTransfer);
                    bool transferOrientationReady = IsPitchSettled(targetPitchTransfer);
                    double transferApoError = targetRadius - apoapsis;
                    float transferThrottle = 0f;
                    if (transferApoError > 0.0)
                    {
                        transferThrottle = CalculateThrottleDynamic(apoapsis, targetRadius);
                        if (transferApoError < 5000.0)
                            transferThrottle = Mathf.Min(transferThrottle, 0.25f);
                        if (transferApoError < 2000.0)
                            transferThrottle = Mathf.Min(transferThrottle, 0.1f);
                    }

                    if (WorldTime.main.worldTime < throttleUnlockWorldTime || !transferOrientationReady)
                    {
                        transferThrottle = 0f;
                    }

                    if (debug && UnityEngine.Time.frameCount % 30 == 0)
                        Debug.Log($"[Autopilot] Transfer throttle={transferThrottle:F2}, apoErr={transferApoError:F0}m, settled={transferOrientationReady}");

                    SetThrottle(transferThrottle);

                    if (apoapsis >= targetRadius - APOAPSIS_TARGET_MARGIN)
                    {
                        CutEngines();
                        state = AscentState.Coast;
                        coastLogCounter = 0;
                        if (debug) Debug.Log($"[Autopilot] Transfer burn -> Coast, apoapsis reached {apoapsisAltitude:F0}m for target {targetAltitude:F0}m");
                    }
                    break;
            }
        }

        // private void InitializeEngines()
        // {
        //     if (currentStage == null) return;

        //     if (!rocket.staging.stages.Contains(currentStage))
        //     {
        //         if (debug) Debug.Log("[Autopilot] Current stage no longer exists, aborting engine init");
        //         return;
        //     }

        //     if (debug) Debug.Log($"[Autopilot] Initializing engines for stage {currentStage.stageId}");

        //     foreach (var part in currentStage.parts)
        //     {
        //         foreach (var engine in part.GetModules<EngineModule>())
        //         {
        //             if (!engine.engineOn.Value)
        //             {
        //                 engine.engineOn.Value = true;
        //                 if (debug) Debug.Log($"[Autopilot] Engine turned ON");
        //             }
        //             if (engine.hasGimbal && engine.gimbalOn != null && !engine.gimbalOn.Value)
        //             {
        //                 engine.gimbalOn.Value = true;
        //                 if (debug) Debug.Log($"[Autopilot] Gimbal enabled");
        //             }
        //         }
        //     }
        // }

        private void SetThrottle(float percent)
        {
            float effectivePercent = Mathf.Clamp01(percent);
            if (effectivePercent < THROTTLE_SNAP_THRESHOLD)
            {
                effectivePercent = 0f;
            }

            if (effectivePercent > 0f)
            {
                SetEngineIgnition(true);
            }

            float oldPercent = rocket.throttle.throttlePercent.Value;
            bool oldOn = rocket.throttle.throttleOn.Value;

            rocket.throttle.throttlePercent.Value = effectivePercent;
            rocket.throttle.throttleOn.Value = effectivePercent > 0f;

            if (debug && (oldPercent != rocket.throttle.throttlePercent.Value || oldOn != rocket.throttle.throttleOn.Value))
            {
                Debug.Log($"[Autopilot] Throttle set to {rocket.throttle.throttlePercent.Value:F3}, ON={rocket.throttle.throttleOn.Value}");
            }
        }

        private void SetEngineIgnition(bool enabled)
        {
            foreach (var engine in rocket.partHolder.GetModules<EngineModule>())
            {
                if (engine.engineOn.Value != enabled)
                {
                    engine.engineOn.Value = enabled;
                }
            }
        }

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
                {
                    turn = Mathf.Sign(angularVelocity);
                }
                else
                {
                    turn = -Mathf.Sign(error);
                }
            }
            catch
            {
                float pGain = 0.8f;
                float dGain = 0.5f;
                turn = (-error * pGain - angularVelocity * dGain) / 30f;
            }

            turn = Mathf.Clamp(turn, -1f, 1f);
            rocket.arrowkeys.turnAxis.Value = turn;

            if (debug && UnityEngine.Time.frameCount % 60 == 0)
            {
                Debug.Log($"[Autopilot] Pitch: current={currentPitch:F1}, target={targetPitchDegrees:F1}, error={error:F1}, angVel={angularVelocity:F1}, turn={turn:F2}");
            }
        }

        private void CutEngines() => SetThrottle(0f);

        private void PrepareForCoastOrWarp()
        {
            SetThrottle(0f);
            SetEngineIgnition(false);
            rocket.arrowkeys.turnAxis.Value = 0f;
        }

        // (RU) Динамический расчёт тяги на основе расстояния до цели (для фазы подъёма) | (EN) Dynamic thrust calculation based on distance to target (for the ascent phase)
        private float CalculateThrottleDynamic(double currentValue, double targetRadius)
        {
            if (currentValue <= 0) return 1f;

            double diff = targetRadius - currentValue;
            if (diff <= 0) return 0f;

            if (diff > 20000) return 1f;
            if (diff > 10000) return 0.85f;
            if (diff > 5000) return 0.65f;
            if (diff > 2000) return 0.4f;
            return 0.25f;
        }

        private void InitializeMissionTargets()
        {
            requestedTargetAltitude = Mathf.Max(MIN_TARGET_ALTITUDE, Settings.data.targetOrbitAltitude);
            targetAltitude = GetInitialInsertionTargetAltitude(requestedTargetAltitude);
            pendingTransferBurn = requestedTargetAltitude > targetAltitude + 0.1f;

            MsgDrawer.main.Log(pendingTransferBurn
                ? $"Autopilot target: {requestedTargetAltitude / 1000f:0.#} km via {targetAltitude / 1000f:0.#} km parking orbit."
                : $"Autopilot target: {targetAltitude / 1000f:0.#} km.");

            if (!debug)
                return;

            if (pendingTransferBurn)
                Debug.Log($"[Autopilot] Mission target {requestedTargetAltitude:F0}m, inserting into {targetAltitude:F0}m parking orbit first");
            else
                Debug.Log($"[Autopilot] Mission target {targetAltitude:F0}m");
        }

        private void BeginCoast(double rawApoapsis, double apoapsisAltitude, string transitionReason)
        {
            if (debug)
                Debug.Log($"[Autopilot] {transitionReason}, apoapsis reached {rawApoapsis:F0}m (alt {apoapsisAltitude:F0}m)");

            PrepareForCoastOrWarp();
            state = AscentState.Coast;
            coastLogCounter = 0;
        }

        private void BeginCircularize(string transitionReason, double altitude, double apoapsisAltitude)
        {
            double timeToApo = GetTimeToApoapsis();
            double distanceToApo = Math.Abs(apoapsisAltitude - altitude);
            double requiredDeltaV = EstimateCircularizationDeltaVAtApoapsis(out double burnSpeed, out double targetCircularSpeed, out double burnRadius);
            double horizontalSpeed = burnSpeed;
            double targetOrbitalSpeed = targetCircularSpeed;
            double estimatedBurnDuration = EstimateCircularizationBurnDuration(requiredDeltaV);
            double coastLeadTime = Math.Max(COAST_TO_CIRC_MIN_LEAD_TIME, estimatedBurnDuration * 0.5 + COAST_TO_CIRC_BURN_BUFFER);

            WorldTime.main.StopTimewarp(false);
            if (debug)
                Debug.Log($"[Autopilot] {transitionReason}, T-{timeToApo:F1}s, dist={distanceToApo:F0}m, lead={coastLeadTime:F2}s");

            PrepareForCoastOrWarp();
            state = AscentState.Circularize;
            deltaVTarget = requiredDeltaV;
            circularizeWaitingForNextPass = false;

            double mass = rocket.mass.GetMass();
            double acceleration = CalculateMaxAcceleration();
            if (acceleration <= 0.00001 && mass > 0.0)
            {
                double maxThrust = CalculateMaxThrust();
                acceleration = maxThrust / mass;
            }

            burnDuration = (acceleration > 0.00001) ? (deltaVTarget / acceleration) : estimatedBurnDuration;
            if (burnDuration <= 0.0 && deltaVTarget > 0.0)
            {
                burnDuration = 10.0;
                if (debug) Debug.Log($"[Autopilot] Acceleration estimate was 0; using fallback burnDuration={burnDuration:F1}s");
            }

            circularizeStartWorldTime = WorldTime.main.worldTime;
            circularizeEntryWorldTime = circularizeStartWorldTime;
            throttleUnlockWorldTime = circularizeStartWorldTime + GUIDED_BURN_SETTLE_TIME;

            if (debug)
                Debug.Log($"[Autopilot] Required О”V: {requiredDeltaV:F1}m/s, using О”Vtarget={deltaVTarget:F1}m/s, burn duration: {burnDuration:F1}s, horizVel={horizontalSpeed:F1}m/s, targetVel={targetOrbitalSpeed:F1}m/s, maxAccel est={CalculateMaxAcceleration():F2}m/sВІ");
        }

        private float GetInitialInsertionTargetAltitude(float requestedAltitude)
        {
            float parkingOrbitAltitude = GetParkingOrbitAltitude();
            if (requestedAltitude > parkingOrbitAltitude + DIRECT_ASCENT_MAX_EXTRA_ALTITUDE)
                return parkingOrbitAltitude;
            return requestedAltitude;
        }

        private float GetParkingOrbitAltitude()
        {
            float atmosphereSafeAltitude = (float)Math.Max(atmosphereHeight + PARKING_ORBIT_ATMOSPHERE_MARGIN, TURN_MIN_END_ALTITUDE);
            return Math.Max(PARKING_ORBIT_MIN_ALTITUDE, atmosphereSafeAltitude);
        }

        private void BeginTransferBurn()
        {
            targetAltitude = requestedTargetAltitude;
            targetRadius = targetAltitude + rocket.location.planet.Value.Radius;
            pendingTransferBurn = false;
            state = AscentState.TransferBurn;
            throttleUnlockWorldTime = WorldTime.main.worldTime + GUIDED_BURN_SETTLE_TIME;
            circularizeEntryWorldTime = WorldTime.main.worldTime;
            circularizeWaitingForNextPass = false;
            PrepareForCoastOrWarp();

            if (debug)
                Debug.Log($"[Autopilot] Parking orbit complete, beginning transfer burn to {targetAltitude:F0}m");
        }

        private double GetPitchProgramEndAltitude()
        {
            double desiredTurnEndAltitude = (atmosphereHeight > 1.0)
                ? Math.Min(targetAltitude * TURN_TARGET_ALTITUDE_FACTOR, atmosphereHeight * TURN_ATMOSPHERE_ALTITUDE_FACTOR)
                : targetAltitude * TURN_NO_ATMOSPHERE_ALTITUDE_FACTOR;

            float currentTwr = GetCurrentTwrEstimate();
            if (currentTwr > 0.01f && currentTwr < LOW_TWR_LIMIT)
            {
                float twrProgress = Mathf.InverseLerp(VERY_LOW_TWR_LIMIT, LOW_TWR_LIMIT, currentTwr);
                double lowTwrMultiplier = Mathf.Lerp(1.45f, 1.10f, twrProgress);
                desiredTurnEndAltitude *= lowTwrMultiplier;
                if (atmosphereHeight > 1.0)
                {
                    desiredTurnEndAltitude = Math.Max(desiredTurnEndAltitude, atmosphereHeight * 0.9);
                }
            }

            desiredTurnEndAltitude = Math.Max(actualTurnStartAltitude + TURN_MIN_END_ALTITUDE, desiredTurnEndAltitude);
            double maxTurnEndAltitude = Math.Max(actualTurnStartAltitude + 1000.0, targetAltitude - 2000.0);
            return Math.Max(actualTurnStartAltitude + 1.0, Math.Min(desiredTurnEndAltitude, maxTurnEndAltitude));
        }

        private double EstimateCircularizationBurnDuration(double deltaV)
        {
            if (deltaV <= 0.0)
            {
                return 0.0;
            }

            double acceleration = CalculateMaxAcceleration();
            if (acceleration <= 0.00001)
            {
                double mass = rocket.mass.GetMass();
                if (mass > 0.0)
                {
                    double maxThrust = CalculateMaxThrust();
                    acceleration = maxThrust / mass;
                }
            }

            return (acceleration > 0.00001) ? (deltaV / acceleration) : 0.0;
        }

        private void UpdateTimewarpToApoapsisWindow(double timeToApoapsis, double leadTime, bool aboveAtmosphere)
        {
            bool controlsNeutral = Math.Abs(rocket.arrowkeys.turnAxis.Value) <= 0.01f &&
                rocket.throttle.throttlePercent.Value <= THROTTLE_SNAP_THRESHOLD;

            if (!aboveAtmosphere || !controlsNeutral)
            {
                if (WorldTime.main.timewarpSpeed > 1)
                    WorldTime.main.StopTimewarp(false);
                return;
            }

            if (!CanUseTimewarp())
                return;

            double timeToWarpStop = timeToApoapsis - leadTime;
            if (timeToWarpStop > 120)
            {
                WorldTime.main.SetState(WorldTime.MaxTimewarpSpeed, false, false);
            }
            else if (timeToWarpStop > 60)
            {
                WorldTime.main.SetState(50, false, false);
            }
            else if (timeToWarpStop > 30)
            {
                WorldTime.main.SetState(25, false, false);
            }
            else if (timeToWarpStop > 15)
            {
                WorldTime.main.SetState(10, false, false);
            }
            else if (timeToWarpStop > 8)
            {
                WorldTime.main.SetState(5, false, false);
            }
            else if (WorldTime.main.timewarpSpeed > 1)
            {
                WorldTime.main.StopTimewarp(false);
            }
        }

        private bool CanUseTimewarp()
        {
            bool isInWater;
            return WorldTime.CanTimewarp(false, false, out isInWater);
        }

        private double EstimateCircularizationDeltaVAtApoapsis(out double currentSpeedAtBurn, out double targetCircularSpeed, out double burnRadius)
        {
            double planetRadius = rocket.location.planet.Value.Radius;
            double mu = rocket.location.planet.Value.mass;

            burnRadius = Math.Max(rocket.location.Value.Radius, planetRadius + 1.0);
            currentSpeedAtBurn = Math.Max(0.0, GetHorizontalSpeed());

            Orbit orbit = GetCurrentOrbit();
            if (orbit != null && !double.IsInfinity(orbit.apoapsis) && !double.IsNaN(orbit.apoapsis) && orbit.apoapsis > planetRadius)
            {
                burnRadius = Math.Max(orbit.apoapsis, planetRadius + 1.0);
                Double2 burnVelocity = orbit.GetVelocityAtTrueAnomaly(Math.PI);
                if (!double.IsNaN(burnVelocity.x) && !double.IsNaN(burnVelocity.y))
                {
                    currentSpeedAtBurn = burnVelocity.magnitude;
                }
            }

            targetCircularSpeed = Math.Sqrt(mu / burnRadius);
            if (double.IsNaN(targetCircularSpeed) || double.IsInfinity(targetCircularSpeed))
            {
                targetCircularSpeed = 0.0;
            }

            return Math.Max(0.0, targetCircularSpeed - currentSpeedAtBurn);
        }

        private double GetHorizontalSpeed()
        {
            Double2 horizontalDirection = GetHorizontalDirection();
            return Double2.Dot(rocket.location.velocity.Value, horizontalDirection);
        }

        private double GetRadialSpeed()
        {
            Double2 radialDirection = rocket.location.position.Value.normalized;
            return Double2.Dot(rocket.location.velocity.Value, radialDirection);
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
            {
                return Double2.right;
            }

            return new Double2(-radialDirection.y, radialDirection.x) * GetOrbitalDirectionSign();
        }

        private bool HasAnyActiveThrust()
        {
            foreach (var engine in rocket.partHolder.GetModules<EngineModule>())
            {
                if (engine.engineOn.Value && engine.thrust.Value > ACTIVE_THRUST_THRESHOLD)
                    return true;
            }

            foreach (var booster in rocket.partHolder.GetModules<BoosterModule>())
            {
                if (booster.enabled && booster.thrustVector.Value.magnitude > ACTIVE_THRUST_THRESHOLD)
                    return true;
            }

            return false;
        }

        private bool StageHasActiveThrust(Stage stage)
        {
            foreach (var part in stage.parts)
            {
                foreach (var engine in part.GetModules<EngineModule>())
                {
                    if (engine.engineOn.Value && engine.thrust.Value > ACTIVE_THRUST_THRESHOLD)
                        return true;
                }

                foreach (var booster in part.GetModules<BoosterModule>())
                {
                    if (booster.enabled && booster.thrustVector.Value.magnitude > ACTIVE_THRUST_THRESHOLD)
                        return true;
                }
            }

            return false;
        }

        private bool HasSpentBoostersAttached()
        {
            foreach (var booster in rocket.partHolder.GetModules<BoosterModule>())
            {
                if (!booster.enabled || booster.thrustVector.Value.magnitude <= ACTIVE_THRUST_THRESHOLD)
                    return true;
            }

            return false;
        }

        private float GetCurrentTwrEstimate()
        {
            double accel = CalculateMaxAcceleration();
            if (accel < 0.001 && cachedMaxAcceleration > 0.001)
                accel = cachedMaxAcceleration;
            return (float)(accel / 9.8);
        }

        private double GetOrbitalDirectionSign()
        {
            Double3 cross = Double3.Cross(rocket.location.position.Value, rocket.location.velocity.Value);
            return (cross.z < 0.0) ? -1.0 : 1.0;
        }

        private double CalculateMaxThrust()
        {
            double thrust = 0;
            foreach (var engine in rocket.partHolder.GetModules<EngineModule>())
            {
                if (engine.engineOn.Value)
                    thrust += engine.thrust.Value;
            }
            foreach (var booster in rocket.partHolder.GetModules<BoosterModule>())
            {
                if (booster.enabled)
                    thrust += booster.thrustVector.Value.magnitude;
            }
            return thrust * 9.8;
        }

        private double CalculateMaxAcceleration()
        {
            double thrust = 0;
            foreach (var engine in rocket.partHolder.GetModules<EngineModule>())
            {
                if (engine.engineOn.Value)
                    thrust += engine.thrust.Value * 9.8;
            }
            foreach (var booster in rocket.partHolder.GetModules<BoosterModule>())
            {
                if (booster.enabled)
                    thrust += booster.thrustVector.Value.magnitude * 9.8;
            }
            double mass = rocket.mass.GetMass();
            if (mass <= 0.00001)
                return 0.0;
            double accel = thrust / mass;
            // (RU) Сохраняем последнее ненулевое значение — используется когда двигатели не горят (например, в начале циркуляризации) | (EN) Cache last non-zero value — used when engines are not burning (e.g. at start of circularization)
            if (accel > 0.001)
                cachedMaxAcceleration = accel;
            if (debug && UnityEngine.Time.frameCount % 60 == 0)
                Debug.Log($"[Autopilot] Total thrust: {thrust:N1}kN, mass: {mass:F1}t, accel: {accel:F1}m/s²");
            return accel;
        }

        private bool StageContainsPropulsion(Stage stage)
        {
            foreach (var part in stage.parts)
            {
                if (PartHasPropulsion(part))
                    return true;
            }

            return false;
        }

        private bool PartHasPropulsion(Part part)
        {
            if (part == null)
                return false;

            return part.GetModules<EngineModule>().Any() || part.GetModules<BoosterModule>().Any();
        }

        private bool PartHasActiveThrust(Part part)
        {
            if (part == null)
                return false;

            foreach (var engine in part.GetModules<EngineModule>())
            {
                if (engine.engineOn.Value && engine.thrust.Value > ACTIVE_THRUST_THRESHOLD)
                    return true;
            }

            foreach (var booster in part.GetModules<BoosterModule>())
            {
                if (booster.enabled && booster.thrustVector.Value.magnitude > ACTIVE_THRUST_THRESHOLD)
                    return true;
            }

            return false;
        }

        private void CacheLaunchStagePropulsionParts()
        {
            launchStagePropulsionParts.Clear();

            if (rocket == null || rocket.staging.stages.Count == 0)
                return;

            foreach (var part in rocket.staging.stages[0].parts)
            {
                if (PartHasPropulsion(part))
                    launchStagePropulsionParts.Add(part);
            }
        }

        private bool IsLaunchStageStillAttached()
        {
            foreach (var part in launchStagePropulsionParts)
            {
                if (part != null && part.Rocket == rocket)
                    return true;
            }

            return false;
        }

        private bool ShouldProtectLaunchStagePropulsion()
        {
            if (launchStagePropulsionParts.Count == 0 || rocket == null)
                return false;

            if (!IsLaunchStageStillAttached())
            {
                launchStagePropulsionParts.Clear();
                return false;
            }

            foreach (var part in launchStagePropulsionParts)
            {
                if (PartHasActiveThrust(part))
                    return true;
            }

            return false;
        }

        private void SuppressPrematureUpperStagePropulsion()
        {
            if (!ShouldProtectLaunchStagePropulsion())
                return;

            foreach (var part in rocket.partHolder.parts)
            {
                if (launchStagePropulsionParts.Contains(part))
                    continue;

                foreach (var engine in part.GetModules<EngineModule>())
                {
                    if (engine.engineOn.Value)
                    {
                        engine.engineOn.Value = false;
                        if (debug)
                            Debug.Log($"[Autopilot] Disabled non-launch-stage engine on part {part.name}");
                    }
                }

                foreach (var booster in part.GetModules<BoosterModule>())
                {
                    if (!booster.enabled && booster.boosterPrimed != null && booster.boosterPrimed.Value)
                    {
                        booster.boosterPrimed.Value = false;
                        if (debug)
                            Debug.Log($"[Autopilot] Unprimed non-launch-stage booster on part {part.name}");
                    }
                }
            }
        }

        private void FireStage(Stage stage, string reason)
        {
            if (stage.stageId == lastStageAttemptId)
                lastStageAttemptCount++;
            else
                lastStageAttemptCount = 1;

            lastStageAttemptId = stage.stageId;
            lastStageAttemptWorldTime = WorldTime.main.worldTime;

            if (debug) Debug.Log($"[Autopilot] {reason}, firing stage {stage.stageId} (attempt {lastStageAttemptCount})");

            try
            {
                if (StagingDrawer.main == null)
                {
                    if (debug) Debug.Log("[Autopilot] StagingDrawer.main is null");
                    return;
                }

                var method = typeof(StagingDrawer).GetMethod("UseStage",
                    BindingFlags.NonPublic | BindingFlags.Instance);

                if (method == null)
                {
                    if (debug) Debug.Log("[Autopilot] UseStage method not found");
                    return;
                }

                bool hadControl = PlayerController.main.hasControl.Value;
                PlayerController.main.hasControl.Value = true;

                method.Invoke(StagingDrawer.main, new object[] { stage });

                PlayerController.main.hasControl.Value = hadControl;

                if (debug) Debug.Log($"[Autopilot] Stage {stage.stageId} fired successfully");
            }
            catch (Exception e)
            {
                if (debug) Debug.Log($"[Autopilot] UseStage failed: {e.Message}");
            }
        }

        private void CheckStaging(bool aboveAtmosphere)
        {
            if (rocket.staging.stages.Count == 0) return;
            // (RU) Не проверяем стейджинг на старте — двигатели ещё не запущены | (EN) Don't check staging at liftoff — engines not yet started
            if (state == AscentState.AwaitingLaunch || state == AscentState.Liftoff) return;

            bool intentionalCoast = IsIntentionalZeroThrottleCoast();

            if (intentionalCoast)
            {
                noThrustSinceWorldTime = double.NegativeInfinity;
                return;
            }

            if (ShouldProtectLaunchStagePropulsion())
            {
                noThrustSinceWorldTime = double.NegativeInfinity;
                return;
            }

            bool hasThrust = rocketManager.HasThrust();
            if (hasThrust)
            {
                noThrustSinceWorldTime = double.NegativeInfinity;
                return;
            }

            double now = WorldTime.main.worldTime;
            if (double.IsNegativeInfinity(noThrustSinceWorldTime))
                noThrustSinceWorldTime = now;

            Stage stage = rocket.staging.stages[0];
            bool stageHasPropulsion = StageContainsPropulsion(stage);
            if (!stageHasPropulsion && !aboveAtmosphere)
                return;

            double confirmationTime = stageHasPropulsion
                ? STAGING_NO_THRUST_CONFIRMATION_TIME
                : STAGING_PASSIVE_CONFIRMATION_TIME;

            if (now - noThrustSinceWorldTime < confirmationTime)
                return;

            if (rocketManager.CheckStaging())
                noThrustSinceWorldTime = now;
            return;

#if false
            Stage stage = rocket.staging.stages[0];
            bool anyThrust = HasAnyActiveThrust();

            if (stage.stageId != lastStageAttemptId)
                lastStageAttemptCount = 0;

            bool stageHasPropulsion = StageContainsPropulsion(stage);
            bool intentionalCoast = (state == AscentState.Coast || circularizeWaitingForNextPass) &&
                rocket.throttle.throttlePercent.Value <= THROTTLE_SNAP_THRESHOLD;
            bool canRetryStage = stage.stageId != lastStageAttemptId ||
                (lastStageAttemptCount < 3 && WorldTime.main.worldTime - lastStageAttemptWorldTime >= 1.0);

            if (intentionalCoast)
                return;

            if (!anyThrust)
            {
                if (canRetryStage)
                    FireStage(stage, "No thrust detected");
                return;
            }

            if (!stageHasPropulsion && aboveAtmosphere && canRetryStage)
            {
                FireStage(stage, "Passive ascent stage ready above atmosphere");
                return;
            }
#endif
        }

        private bool IsIntentionalZeroThrottleCoast()
        {
            if (rocket.throttle.throttlePercent.Value > THROTTLE_SNAP_THRESHOLD)
                return false;

            if (state == AscentState.Coast || circularizeWaitingForNextPass)
                return true;

            if (state == AscentState.Circularize)
                return true;

            if (state == AscentState.TransferBurn && WorldTime.main.worldTime < throttleUnlockWorldTime)
                return true;

            return false;
        }

        private double GetSafeCoastPeriapsisRadius()
        {
            double planetRadius = rocket.location.planet.Value.Radius;
            return planetRadius + Math.Max(1000.0, atmosphereHeight + 1000.0);
        }

        private bool CanSafelyCoastToNextApoapsis(double periapsis)
        {
            return periapsis >= GetSafeCoastPeriapsisRadius();
        }

        private double GetApoapsis()
        {
            var orbit = GetCurrentOrbit();
            if (orbit == null) return 0;
            double ap = orbit.apoapsis;
            if (double.IsInfinity(ap) || ap > 1e9)
                return double.PositiveInfinity;
            return ap;
        }

        private double GetPeriapsis()
        {
            var orbit = GetCurrentOrbit();
            if (orbit == null) return 0;
            return orbit.periapsis;
        }

        private double GetTimeToApoapsis()
        {
            var orbit = GetCurrentOrbit();
            if (orbit == null) return double.PositiveInfinity;
            double now = WorldTime.main.worldTime;
            double nextApoapsisTime = orbit.GetNextTrueAnomalyPassTime(now, Math.PI);
            return nextApoapsisTime - now;
        }

        private Orbit GetCurrentOrbit()
        {
            var trajectory = rocket?.physics?.GetTrajectory();
            if (trajectory == null) return null;
            var paths = trajectory.paths;
            if (paths.Count == 0) return null;
            return paths[0] as Orbit;
        }
    }
}
