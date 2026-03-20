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
        Liftoff,
        PitchOver,
        Coast,
        Circularize,
        TransferBurn
    }

    public class AscentAutopilot
    {
        private Rocket rocket;
        private AscentState state = AscentState.Idle;
        private float targetAltitude;          // meters above surface
        private float requestedTargetAltitude; // final user-requested orbit altitude
        private double targetRadius;            // target altitude + planet radius
        private bool pendingTransferBurn;

        private const float MIN_TARGET_ALTITUDE = 1000f;
        private const float DIRECT_ASCENT_MAX_EXTRA_ALTITUDE = 80000f;
        private const float PARKING_ORBIT_MIN_ALTITUDE = 5000f;
        private const float PARKING_ORBIT_ATMOSPHERE_MARGIN = 5000f;
        private const double COAST_TO_CIRC_MIN_LEAD_TIME = 0.75;
        private const double COAST_TO_CIRC_BURN_BUFFER = 0.35;
        private const float THROTTLE_SNAP_THRESHOLD = 0.01f;

        // Параметры подъёма (жёстко заданы)
        private const float PITCH_START_ALTITUDE = 100f;      // начинаем поворот на 100 м
        private const float TURN_END_ANGLE = 0f;              // заканчиваем горизонтально (0°)
        private const float TURN_SHAPE_EXPONENT = 0.8f;        // степень для кривой поворота
        private const float MIN_PITCH = 5f;                    // минимальный угол тангажа
        private const double TURN_TARGET_ALTITUDE_FACTOR = 0.45;
        private const double TURN_ATMOSPHERE_ALTITUDE_FACTOR = 0.9;
        private const double TURN_NO_ATMOSPHERE_ALTITUDE_FACTOR = 0.35;
        private const double TURN_MIN_END_ALTITUDE = 2500.0;

        // Параметры управления орбитой
        private const double APOAPSIS_TARGET_MARGIN = 100;     // отключаем двигатели при достижении цели с недолётом 100 м (для первой фазы)
        private const double PERIAPSIS_TOLERANCE = 250;       // допустимая погрешность для периапсиса (250 м)
        private const double CORRECTION_GAIN = 40000;          // коэффициент преобразования ошибки периапсиса в градусы
        private const float PERI_MAX_PITCH_CORRECTION = 2.0f;  // чтобы не уводить слишком далеко от prograde
        private const float APO_MAX_PITCH_CORRECTION = 5.0f;   // небольшая коррекция, но без жёсткого насыщения
        private const double APOAPSIS_OVER_CORRECTION_GAIN = 20000; // для коррекции по апогею (больше => мягче)
        private const double APO_THROTTLE_REDUCTION_START = 250; // начинаем снижать тягу при заметном превышении апогея
        private const double APO_THROTTLE_REDUCTION_END = 1500;  // снижать плавнее, чтобы не сорвать набор delta-V
        private const float APO_MIN_THROTTLE_FACTOR = 0.25f;     // минимальная доля тяги, чтобы продолжить набор delta-V
        private const double COAST_TO_CIRC_THRESHOLD = 2.5;    // начинаем циркуляризацию за 2.5 секунды до апогея
        private const double COAST_TO_CIRC_DISTANCE = 200;     // или за 200 м по высоте
        private const double CIRCULARIZE_MAX_DURATION = 90.0; // страховка от бесконечного горения
        private const float CIRCULARIZE_PITCH_UP_BIAS = 0.6f;        // небольшой bias “вверх”
        private const float CIRCULARIZE_PITCH_UP_BIAS_IF_APO_HIGH = 1.2f; // усиление, если апогей уже выше цели
        private const double CIRCULARIZE_VEL_TOLERANCE = 2.0;          // m/s, “почти круговая скорость”
        private const double PERI_THROTTLE_GAIN = 120000.0;            // meters -> throttle (больше => слабее)
        private const float PERI_THROTTLE_MIN = 0.0f;                  // даем тяге падать почти до нуля для точной доводки
        private const double CIRCULARIZE_NEAR_APO_DISTANCE = 5000.0;  // м: доводим перицентр только очень близко к апогею
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

        private double burnDuration;
        private double circularizeStartWorldTime;
        private double deltaVTarget;
        private double circularizeEntryWorldTime;
        private double throttleUnlockWorldTime;
        private bool enginesInitialized = false;
        private Stage currentStage = null;
        private HashSet<Stage> stagingAttempted = new HashSet<Stage>();
        private double actualTurnStartAltitude = 0;
        private double atmosphereHeight = 0;                   // высота атмосферы текущей планеты

        private bool debug = true;
        private int coastLogCounter = 0;

        public bool IsActive { get; private set; }

        public AscentAutopilot(Rocket rocket)
        {
            this.rocket = rocket;
            RefreshRuntimeSettings();

            if (rocket != null && rocket.location.planet.Value != null)
            {
                atmosphereHeight = rocket.location.planet.Value.AtmosphereHeightPhysics;
                if (debug)
                {
                    Debug.Log($"[Autopilot] Difficulty: {Base.worldBase.settings.difficulty.difficulty}, Atmosphere height: {atmosphereHeight}m");
                }
            }

            if (debug) Debug.Log("[Autopilot] AscentAutopilot initialized");
        }

        public void SetRocket(Rocket rocket)
        {
            this.rocket = rocket;
            RefreshRuntimeSettings();
        }

        public void Start()
        {
            if (rocket == null) return;
            RefreshRuntimeSettings();
            InitializeMissionTargets();
            IsActive = true;
            state = AscentState.Liftoff;
            enginesInitialized = false;
            currentStage = null;
            stagingAttempted.Clear();
            actualTurnStartAltitude = 0;
            throttleUnlockWorldTime = 0.0;
            if (debug) Debug.Log("[Autopilot] STARTED");
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
            if (debug) Debug.Log("[Autopilot] STOPPED");
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

            // Если ступеней не осталось, автопилот не может работать
            if (rocket.staging.stages.Count == 0)
            {
                if (debug) Debug.Log("[Autopilot] No stages left, stopping autopilot");
                Stop();
                return;
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

            if (debug && UnityEngine.Time.frameCount % 60 == 0)
            {
                Debug.Log($"[Autopilot] State: {state}, Alt: {altitude:F0}m, RawApo: {rawApoapsis:F0}m, ApoAlt: {apoapsisAltitude:F0}m, PerAlt: {periapsisAltitude:F0}m, TargetRadius: {targetRadius:F0}m, Vel: {velocity:F1}m/s, TurnAxis: {rocket.arrowkeys.turnAxis.Value:F2}, aboveAtmo: {aboveAtmosphere}");
            }

            Stage newStage = (rocket.staging.stages.Count > 0) ? rocket.staging.stages[0] : null;
            if (newStage != currentStage)
            {
                if (debug) Debug.Log($"[Autopilot] Stage changed from {currentStage?.stageId} to {newStage?.stageId}");
                currentStage = newStage;
                enginesInitialized = false;
            }

            if (!enginesInitialized && currentStage != null)
            {
                InitializeEngines();
                enginesInitialized = true;
            }

            CheckStaging();

            switch (state)
            {
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
                    SetPitch(targetPitch);

                    float throttle = CalculateThrottleDynamic(apoapsis, targetRadius);
                    SetThrottle(throttle);

                    if (debug && UnityEngine.Time.frameCount % 60 == 0)
                        Debug.Log($"[Autopilot] PitchOver: targetPitch={targetPitch:F1}, progress={turnProgress:F3}, turnEndAlt={turnEndAltitude:F0}m, throttle={throttle:F2}");

                    // Отключаем двигатели при достижении целевого апогея с недолётом 500 м
                    if (aboveAtmosphere && apoapsis >= targetRadius - APOAPSIS_TARGET_MARGIN)
                    {
                        if (debug) Debug.Log($"[Autopilot] PitchOver -> Coast, apoapsis reached {rawApoapsis:F0}m (alt {apoapsisAltitude:F0}m)");
                        CutEngines();
                        state = AscentState.Coast;
                        coastLogCounter = 0;
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

                    // Включаем ускорение времени только если мы выше атмосферы и можно ускорять
                    if (aboveAtmosphere && WorldTime.CanTimewarp(false, false))
                    {
                        double timeToApoapsis = GetTimeToApoapsis();
                        double currentRadiusForBurn = rocket.location.Value.Radius;
                        double horizontalSpeedForBurn = GetHorizontalSpeed();
                        double targetOrbitalSpeedForBurn = CalculateTargetOrbitalSpeed(currentRadiusForBurn);
                        double requiredDeltaVForBurn = Math.Max(0.0, targetOrbitalSpeedForBurn - horizontalSpeedForBurn);
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
                    double currentRadius = rocket.location.Value.Radius;
                    double horizontalSpeed = GetHorizontalSpeed();
                    double targetOrbitalSpeed = CalculateTargetOrbitalSpeed(currentRadius);
                    double requiredDeltaV = Math.Max(0.0, targetOrbitalSpeed - horizontalSpeed);
                    double estimatedBurnDuration = EstimateCircularizationBurnDuration(requiredDeltaV);
                    double coastLeadTime = Math.Max(COAST_TO_CIRC_MIN_LEAD_TIME, estimatedBurnDuration * 0.5 + COAST_TO_CIRC_BURN_BUFFER);
                    double timeToBurnStart = timeToApo - coastLeadTime;

                    // Начинаем циркуляризацию, когда осталось меньше заданного времени ИЛИ расстояние по высоте мало
                    if (((timeToBurnStart <= 0.0) && WorldTime.main.timewarpSpeed <= 1) || distanceToApo < COAST_TO_CIRC_DISTANCE)
                    {
                        WorldTime.main.StopTimewarp(false);
                        if (debug) Debug.Log($"[Autopilot] Coast -> Circularize, T-{timeToApo:F1}s, burnIn={timeToBurnStart:F1}s, dist={distanceToApo:F0}m, lead={coastLeadTime:F2}s");
                        CutEngines();
                        state = AscentState.Circularize;
                        deltaVTarget = requiredDeltaV;
                        // Estimate how long we need to burn. Depending on SFS internals, thrust/accel values can briefly be 0
                        // right after we cut engines, so keep a safe fallback duration.
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
                            burnDuration = 10.0; // fallback seconds
                            if (debug) Debug.Log($"[Autopilot] Acceleration estimate was 0; using fallback burnDuration={burnDuration:F1}s");
                        }
                        circularizeStartWorldTime = WorldTime.main.worldTime;
                        circularizeEntryWorldTime = circularizeStartWorldTime;
                        throttleUnlockWorldTime = circularizeStartWorldTime + GUIDED_BURN_SETTLE_TIME;
                        if (debug)
                            Debug.Log($"[Autopilot] Required ΔV: {requiredDeltaV:F1}m/s, using ΔVtarget={deltaVTarget:F1}m/s, burn duration: {burnDuration:F1}s, horizVel={horizontalSpeed:F1}m/s, targetVel={targetOrbitalSpeed:F1}m/s, maxAccel est={CalculateMaxAcceleration():F2}m/s²");

                        // If we somehow already have enough speed for circular orbit at target radius, don't burn.
                        if (deltaVTarget <= 0.0001)
                        {
                            CutEngines();
                            if (pendingTransferBurn)
                                BeginTransferBurn();
                            else
                                Stop();
                        }
                    }
                    break;

                case AscentState.Circularize:
                    // Основное направление – prograde (по вектору скорости) с коррекциями
                    float targetPitchCirc = rocket.GetRotation();
                    bool orientationReady = false;
                    if (rocket.location.velocity.Value.magnitude > 0.1)
                    {
                        float progradeAngle = (float)Math.Atan2(rocket.location.velocity.Value.y, rocket.location.velocity.Value.x) * Mathf.Rad2Deg;
                        float horizontalAngle = GetHorizontalAngle();
                        targetPitchCirc = Mathf.LerpAngle(progradeAngle, horizontalAngle, 0.35f);

                        double periError = targetRadius - periapsis;
                        bool needMajorPeriRaiseForPitch = periError > Math.Max(MAJOR_PERIAPSIS_RAISE_MIN_ERROR, Math.Min(50000.0, targetAltitude * 0.05));
                        targetPitchCirc = Mathf.LerpAngle(progradeAngle, horizontalAngle, needMajorPeriRaiseForPitch ? 0.8f : 0.35f);
                        double apoError = targetRadius - apoapsis; // отрицательное, если апогей выше цели

                        // Коррекция по перицентру (симметрично: поднимаем нос если ниже цели, опускаем если выше)
                        if (Math.Abs(periError) > PERIAPSIS_TOLERANCE)
                        {
                            float periCorr = Mathf.Clamp((float)(periError / CORRECTION_GAIN), -PERI_MAX_PITCH_CORRECTION, PERI_MAX_PITCH_CORRECTION);
                            targetPitchCirc += periCorr;
                            if (debug && UnityEngine.Time.frameCount % 60 == 0)
                                Debug.Log($"[Autopilot] Peri correction={periCorr:F2}° (periErr={periError:F0}m)");
                        }

                        // Коррекция по апогею:
                        // apoError = targetRadius - apoapsis; отрицательно => апогей ВЫШЕ цели.
                        // Важно: знак в старой версии приводил к тому, что коррекция почти всегда "обнулялась" clamping'ом.
                        double apoOvershoot = apoapsis - targetRadius; // положительно => апогей выше цели
                        if (Math.Abs(apoOvershoot) > APOAPSIS_TARGET_MARGIN)
                        {
                            double apoCorrectionGain = needMajorPeriRaiseForPitch ? MAJOR_APOAPSIS_CORRECTION_GAIN : APOAPSIS_OVER_CORRECTION_GAIN;
                            float apoCorrectionLimit = needMajorPeriRaiseForPitch ? MAJOR_APOAPSIS_MAX_PITCH_CORRECTION : APO_MAX_PITCH_CORRECTION;
                            float apoCorrMag = Mathf.Clamp((float)(Math.Abs(apoOvershoot) / apoCorrectionGain), 0f, apoCorrectionLimit);
                            // Если апогей УЖЕ выше цели, нам нужно "сдерживать" его,
                            // поэтому при overshoot направляем коррекцию в противоположную сторону.
                            if (apoOvershoot > 0)
                                targetPitchCirc -= apoCorrMag;
                            else
                                targetPitchCirc += apoCorrMag;

                            if (debug && apoCorrMag > 0.01f)
                                Debug.Log($"[Autopilot] Apo pitch correction mag={apoCorrMag:F2}°, overshoot={apoOvershoot:F0}m");
                        }

                        // Небольшой bias “вверх” чтобы снизить склонность к уходу в баллистическую траекторию.
                        // Если апогей уже превышает цель, делаем bias сильнее.
                        double apoOvershootForBias = apoapsis - targetRadius; // положительно => апогей выше цели
                        float pitchBias = CIRCULARIZE_PITCH_UP_BIAS;
                        if (needMajorPeriRaiseForPitch || apoOvershootForBias > APO_THROTTLE_REDUCTION_START)
                            pitchBias = 0f;

                        targetPitchCirc += pitchBias;

                        SetPitch(targetPitchCirc);
                        orientationReady = IsPitchSettled(targetPitchCirc);
                    }

                    // Управление тягой:
                    // целимся не в "заранее вычисленный" delta-V, а в скорость на текущем апоапсисе,
                    // чтобы при прохождении апоапсиса скорость совпала со скоростью круговой орбиты.
                    double maxAccel = CalculateMaxAcceleration();
                    double rawTimeToApoCirc = GetTimeToApoapsis();
                    double timeToApoCirc = rawTimeToApoCirc;
                    if (double.IsInfinity(timeToApoCirc) || double.IsNaN(timeToApoCirc)) timeToApoCirc = 2.0;
                    timeToApoCirc = Math.Max(-5.0, Math.Min(20.0, timeToApoCirc));
                    double currentRadiusCirc = rocket.location.Value.Radius;
                    double horizontalSpeedCirc = GetHorizontalSpeed();
                    double targetOrbitalSpeedCirc = CalculateTargetOrbitalSpeed(currentRadiusCirc);

                    // v_circ = sqrt(mu / r). В SFS здесь используется planet.mass как mu.
                    double mu = rocket.location.planet.Value.mass;
                    double rApo = Math.Max(apoapsis, 1.0);
                    double vCirc = Math.Sqrt(mu / rApo);
                    double dvErrorNow = vCirc - velocity; // нужно набрать, если положительное

                    // Дополнительно: если перицентр ниже цели, продолжаем гореть даже если dvErrorNow ~ 0,
                    // потому что именно это “добивает” перицентр и повышает точность круговой орбиты.
                    double periErrForThrottle = targetRadius - periapsis; // >0 => перицентр ниже цели
                    bool needMajorPeriRaise = periErrForThrottle > Math.Max(MAJOR_PERIAPSIS_RAISE_MIN_ERROR, Math.Min(50000.0, targetAltitude * 0.05));
                    float periThrottle = 0f;
                    double altitudeToApoCirc = apoapsisAltitude - altitude;
                    double distanceToApoCirc = Math.Abs(altitudeToApoCirc);
                    bool nearApo = distanceToApoCirc <= CIRCULARIZE_NEAR_APO_DISTANCE || Math.Abs(rawTimeToApoCirc) <= CIRCULARIZE_NEAR_APO_TIME;
                    bool inBurnWindow = distanceToApoCirc <= CIRCULARIZE_BURN_WINDOW_DISTANCE || Math.Abs(rawTimeToApoCirc) <= CIRCULARIZE_BURN_WINDOW_TIME;
                    bool canPeriCorrect = altitudeToApoCirc <= CIRCULARIZE_PERI_CORRECTION_DISTANCE || rawTimeToApoCirc <= CIRCULARIZE_PERI_CORRECTION_LEAD_TIME;
                    bool allowPeriCorrection = inBurnWindow && (needMajorPeriRaise || canPeriCorrect);
                    if (allowPeriCorrection && periErrForThrottle > PERIAPSIS_TOLERANCE)
                    {
                        float periMinThrottle = needMajorPeriRaise ? 0.05f : PERI_THROTTLE_MIN;
                        float periMaxThrottle = needMajorPeriRaise ? 0.35f : 0.15f;
                        periThrottle = Mathf.Clamp((float)(periErrForThrottle / PERI_THROTTLE_GAIN), periMinThrottle, periMaxThrottle);
                    }

                    float circThrottle = 0f;
                    dvErrorNow = targetOrbitalSpeedCirc - horizontalSpeedCirc;
                    if (maxAccel > 0.00001 && dvErrorNow > 0.0 && rawTimeToApoCirc > -1.0 && inBurnWindow)
                    {
                        // Распределяем нужный прирост скорости на ближайшее окно до апогея.
                        double desiredTime = Math.Max(Math.Abs(rawTimeToApoCirc), 0.5);
                        double desiredAccel = dvErrorNow / desiredTime;
                        circThrottle = Mathf.Clamp01((float)(desiredAccel / maxAccel));

                        // Если апогей уже "уехал" выше цели, чуть притормаживаем, но не ломаем burn полностью.
                        double apoOvershoot = apoapsis - targetRadius;
                        if (apoOvershoot > APO_THROTTLE_REDUCTION_START)
                        {
                            double reductionRange = needMajorPeriRaise ? Math.Max(5000.0, targetAltitude * 0.05) : (APO_THROTTLE_REDUCTION_END - APO_THROTTLE_REDUCTION_START);
                            double factor = 1.0 - (apoOvershoot - APO_THROTTLE_REDUCTION_START) / reductionRange;
                            factor = Math.Max(0.0, Math.Min(1.0, factor));
                            float applied = (float)Math.Max(factor, needMajorPeriRaise ? 0.35f : APO_MIN_THROTTLE_FACTOR);
                            circThrottle *= applied;
                        }

                    }
                    double closeEnoughPeriTolerance = Math.Max(1500.0, Math.Min(5000.0, targetAltitude * 0.015));
                    double closeEnoughApoTolerance = Math.Max(2500.0, Math.Min(25000.0, targetAltitude * 0.03));
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

                    if (WorldTime.main.worldTime < throttleUnlockWorldTime || !orientationReady)
                    {
                        finalThrottle = 0f;
                    }

                    bool anotherCircularizePassNeeded = periErrForThrottle > closeEnoughPeriTolerance &&
                        finalThrottle <= 0.0f &&
                        rawTimeToApoCirc > CIRCULARIZE_RETRY_COAST_TIME;
                    if (anotherCircularizePassNeeded)
                    {
                        double circularizeLeadTime = Math.Max(CIRCULARIZE_NEAR_APO_TIME, EstimateCircularizationBurnDuration(Math.Max(dvErrorNow, 0.0)) * 0.5 + 1.0);
                        UpdateTimewarpToApoapsisWindow(rawTimeToApoCirc, circularizeLeadTime, aboveAtmosphere);
                        rocket.arrowkeys.turnAxis.Value = 0f;
                    }
                    else if (WorldTime.main.timewarpSpeed > 1)
                    {
                        WorldTime.main.StopTimewarp(false);
                    }

                    if (debug && UnityEngine.Time.frameCount % 30 == 0)
                        Debug.Log($"[Autopilot] Circularize throttle={finalThrottle:F2} (circ={circThrottle:F2}, peri={periThrottle:F2}), dvErrorNow={dvErrorNow:F1}m/s, horizVel={horizontalSpeedCirc:F1}m/s, targetVel={targetOrbitalSpeedCirc:F1}m/s, timeToApo={timeToApoCirc:F2}s, apoOvershoot={(apoapsis - targetRadius):F0}m, periErr={periErrForThrottle:F0}m, distToApo={distanceToApoCirc:F0}m, nearApo={nearApo}, burnWindow={inBurnWindow}, settled={orientationReady}");

                    SetThrottle(finalThrottle);

                    // Завершение, когда перицентр достиг цели (допуск 1 км)
                    closeEnoughPeriTolerance = Math.Max(3000.0, Math.Min(8000.0, targetAltitude * 0.02));
                    closeEnoughApoTolerance = Math.Max(2500.0, Math.Min(25000.0, targetAltitude * 0.03));
                    bool orbitWithinTolerance = Math.Abs(periapsis - targetRadius) < PERIAPSIS_TOLERANCE && Math.Abs(targetOrbitalSpeedCirc - horizontalSpeedCirc) <= CIRCULARIZE_VEL_TOLERANCE;
                    bool orbitCloseEnough = Math.Abs(periapsis - targetRadius) <= closeEnoughPeriTolerance &&
                        Math.Abs(dvErrorNow) <= CIRCULARIZE_VEL_TOLERANCE &&
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
                        // Защита от зависания управления
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

        private void InitializeEngines()
        {
            if (currentStage == null) return;

            if (!rocket.staging.stages.Contains(currentStage))
            {
                if (debug) Debug.Log("[Autopilot] Current stage no longer exists, aborting engine init");
                return;
            }

            if (debug) Debug.Log($"[Autopilot] Initializing engines for stage {currentStage.stageId}");

            foreach (var part in currentStage.parts)
            {
                foreach (var engine in part.GetModules<EngineModule>())
                {
                    if (!engine.engineOn.Value)
                    {
                        engine.engineOn.Value = true;
                        if (debug) Debug.Log($"[Autopilot] Engine turned ON");
                    }
                    if (engine.hasGimbal && engine.gimbalOn != null && !engine.gimbalOn.Value)
                    {
                        engine.gimbalOn.Value = true;
                        if (debug) Debug.Log($"[Autopilot] Gimbal enabled");
                    }
                }
            }
        }

        private void SetThrottle(float percent)
        {
            float effectivePercent = Mathf.Clamp01(percent);
            if (effectivePercent < THROTTLE_SNAP_THRESHOLD)
            {
                effectivePercent = 0f;
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

        // Динамический расчёт тяги на основе расстояния до цели (для фазы подъёма)
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
            CutEngines();

            if (debug)
                Debug.Log($"[Autopilot] Parking orbit complete, beginning transfer burn to {targetAltitude:F0}m");
        }

        private double GetPitchProgramEndAltitude()
        {
            double desiredTurnEndAltitude = (atmosphereHeight > 1.0)
                ? Math.Min(targetAltitude * TURN_TARGET_ALTITUDE_FACTOR, atmosphereHeight * TURN_ATMOSPHERE_ALTITUDE_FACTOR)
                : targetAltitude * TURN_NO_ATMOSPHERE_ALTITUDE_FACTOR;

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
            if (!aboveAtmosphere || !WorldTime.CanTimewarp(false, false))
            {
                if (WorldTime.main.timewarpSpeed > 1)
                    WorldTime.main.StopTimewarp(false);
                return;
            }

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

        private double CalculateTargetOrbitalSpeed(double radius)
        {
            double safeRadius = Math.Max(radius, rocket.location.planet.Value.Radius + 1.0);
            double safeTargetRadius = Math.Max(targetRadius, rocket.location.planet.Value.Radius + 1.0);
            double visVivaTerm = 2.0 / safeRadius - 1.0 / safeTargetRadius;
            if (visVivaTerm <= 0.0)
            {
                return Math.Sqrt(rocket.location.planet.Value.mass / safeTargetRadius);
            }

            return Math.Sqrt(rocket.location.planet.Value.mass * visVivaTerm);
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
            {
                return Double2.right;
            }

            return new Double2(-radialDirection.y, radialDirection.x) * GetOrbitalDirectionSign();
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
            if (debug && UnityEngine.Time.frameCount % 60 == 0)
                Debug.Log($"[Autopilot] Total thrust: {thrust:N1}kN, mass: {mass:F1}t, accel: {thrust / mass:F1}m/s²");
            return thrust / mass;
        }

        private void CheckStaging()
        {
            if (currentStage == null) return;

            if (stagingAttempted.Contains(currentStage))
                return;

            var engines = currentStage.parts
                .SelectMany(p => p.GetModules<EngineModule>())
                .Where(e => e != null)
                .ToList();

            var detachModules = currentStage.parts
                .SelectMany(p => p.GetModules<DetachModule>())
                .ToList();

            if (debug && UnityEngine.Time.frameCount % 60 == 0)
            {
                Debug.Log($"[Autopilot] Stage {currentStage.stageId}: engines count={engines.Count}, detach modules count={detachModules.Count}");
            }

            if (engines.Count == 0)
            {
                if (debug) Debug.Log($"[Autopilot] Stage {currentStage.stageId} has no engines, removing it.");
                stagingAttempted.Add(currentStage);
                if (rocket.staging.stages.Contains(currentStage))
                {
                    rocket.staging.RemoveStage(currentStage, false);
                }
                return;
            }

            bool hasFuel = false;
            foreach (var engine in engines)
            {
                if (engine.engineOn.Value && engine.source.CanFlow(new MsgNone()))
                {
                    hasFuel = true;
                    if (debug && UnityEngine.Time.frameCount % 60 == 0)
                        Debug.Log($"[Autopilot] Engine has fuel and is on");
                    break;
                }
            }

            if (!hasFuel)
            {
                if (debug) Debug.Log($"[Autopilot] Stage {currentStage.stageId} out of fuel, staging...");
                stagingAttempted.Add(currentStage);

                var parts = currentStage.parts.ToArray();
                var partData = parts.Select(p => new ValueTuple<Part, PolygonData>(p, null)).ToArray();
                Rocket.UseParts(true, partData);

                if (detachModules.Count > 0)
                {
                    var sharedData = new UsePartData.SharedData(true);
                    foreach (var detach in detachModules)
                    {
                        var useData = new UsePartData(sharedData, null);
                        detach.Detach(useData);
                    }
                }

                if (rocket.staging.stages.Contains(currentStage))
                {
                    rocket.staging.RemoveStage(currentStage, false);
                }
            }
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
            if (rocket?.mapPlayer?.Trajectory == null) return null;
            var paths = rocket.mapPlayer.Trajectory.paths;
            if (paths.Count == 0) return null;
            return paths[0] as Orbit;
        }
    }
}
