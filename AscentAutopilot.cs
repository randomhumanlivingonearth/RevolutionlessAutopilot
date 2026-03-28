using System;
using SFS;
using SFS.UI;
using SFS.World;
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
        // ── Dependencies ───────────────────────────────────────────────────────
        private Rocket        rocket;
        private RocketManager rm;
        private OrbitManager  om;

        // ── Mission state ──────────────────────────────────────────────────────
        private AscentState state = AscentState.Idle;
        private float  targetAltitude;           // metres — current insertion target
        private float  requestedTargetAltitude;  // final user-requested altitude
        private double targetRadius;             // targetAltitude + planet radius
        private bool   pendingTransferBurn;

        // ── Gravity turn ───────────────────────────────────────────────────────
        private double turnStartAltitude = 0;

        // ── Circularize timing ─────────────────────────────────────────────────
        private double circularizeEntryTime;
        private double burnDuration;
        private double throttleUnlockTime;

        // ── Coast logging ──────────────────────────────────────────────────────
        private int coastFrames = 0;

        private bool debug = true;

        // ══════════════════════════════════════════════════════════════════════
        // Constants
        // ══════════════════════════════════════════════════════════════════════

        private const float  MIN_TARGET_ALTITUDE              = 1000f;
        private const float  DIRECT_ASCENT_MAX_EXTRA_ALTITUDE = 80000f;
        private const float  PARKING_ORBIT_MIN_ALTITUDE       = 5000f;
        private const float  PARKING_ORBIT_ATMOSPHERE_MARGIN  = 5000f;

        // Gravity turn shape
        private const float  PITCH_START_ALTITUDE             = 1000f;
        private const float  TURN_END_ANGLE                   = 0f;
        private const float  TURN_SHAPE_EXPONENT              = 0.8f;
        private const float  MIN_PITCH                        = 5f;
        private const double TURN_TARGET_ALTITUDE_FACTOR      = 0.45;
        private const double TURN_ATMOSPHERE_ALTITUDE_FACTOR  = 0.9;
        private const double TURN_NO_ATMOSPHERE_FACTOR        = 0.35;
        private const double TURN_MIN_END_ALTITUDE            = 2500.0;

        // Apoapsis control
        private const double APOAPSIS_TARGET_MARGIN           = 100.0;

        // Circularize
        private const double CIRC_VEL_TOLERANCE               = 2.0;
        private const double CIRC_PERI_TOLERANCE              = 500.0;
        private const double CIRC_BURN_WINDOW_TIME            = 14.0;
        private const double CIRC_BURN_WINDOW_DIST            = 12000.0;
        private const double CIRC_NEAR_APO_TIME               = 8.0;
        private const double CIRC_NEAR_APO_DIST               = 5000.0;
        private const double CIRC_SETTLE_TIME                 = 0.75;
        private const double CIRC_MAX_DURATION_BASE           = 90.0;

        // Peri correction during circularise
        private const double PERI_CORR_GAIN                   = 40000.0;
        private const float  PERI_CORR_MAX                    = 2.0f;
        private const double PERI_THROTTLE_GAIN               = 120000.0;
        private const double MAJOR_PERI_ERROR                 = 3000.0;

        // Coast
        private const double COAST_TO_CIRC_LEAD               = 0.75;
        private const double COAST_TO_CIRC_BUFFER             = 0.35;
        private const double COAST_TO_CIRC_DIST               = 200.0;

        // ══════════════════════════════════════════════════════════════════════
        // Lifecycle
        // ══════════════════════════════════════════════════════════════════════

        public bool IsActive { get; private set; }

        public AscentAutopilot(Rocket rocket)
        {
            this.rocket = rocket;
            rm = new RocketManager(rocket) { Debug = false };
            om = new OrbitManager(rocket)  { Debug = false };
            RefreshSettings();
            if (debug) UnityEngine.Debug.Log("[Ascent] Initialized");
        }

        public void SetRocket(Rocket rocket)
        {
            this.rocket = rocket;
            rm.SetRocket(rocket);
            om.SetRocket(rocket);
            RefreshSettings();
        }

        public void Start()
        {
            if (rocket == null) return;
            RefreshSettings();
            InitMissionTargets();
            IsActive           = true;
            state              = AscentState.Liftoff;
            turnStartAltitude  = 0;
            throttleUnlockTime = 0;
            coastFrames        = 0;
            rm.ResetStagingHistory();
            if (debug) UnityEngine.Debug.Log("[Ascent] STARTED");
        }

        public void Stop()
        {
            IsActive = false;
            state    = AscentState.Idle;
            rm.CutEngines();
            rm.SetTurnAxis(0f);
            if (debug) UnityEngine.Debug.Log("[Ascent] STOPPED");
        }

        public void Update() { }

        // ══════════════════════════════════════════════════════════════════════
        // Main loop
        // ══════════════════════════════════════════════════════════════════════

        public void FixedUpdate()
        {
            if (!IsActive || rocket == null) return;
            RefreshSettings(log: true);

            // Safety — no stages, no thrust, not mid-burn → give up
            bool activeBurn = state == AscentState.PitchOver ||
                              state == AscentState.Circularize ||
                              state == AscentState.TransferBurn;
            if (rocket.staging.stages.Count == 0 && !rm.HasThrust() && !activeBurn)
            {
                if (debug) UnityEngine.Debug.Log("[Ascent] No stages and no thrust — stopping");
                Stop();
                return;
            }

            double altitude    = rm.GetAltitude();
            double atmosphereH = om.GetAtmosphereHeight();
            bool   aboveAtmo   = altitude > atmosphereH;
            double apoapsis    = om.GetApoapsis();
            double periapsis   = om.GetPeriapsis();
            double apoAlt      = om.GetApoapsisAltitude();
            double periAlt     = om.GetPeriapsisAltitude();
            double planetR     = rocket.location.planet.Value.Radius;
            targetRadius       = targetAltitude + planetR;

            if (debug && UnityEngine.Time.frameCount % 60 == 0)
                UnityEngine.Debug.Log($"[Ascent] {state}  alt={altitude:F0}m  apoAlt={apoAlt:F0}m  periAlt={periAlt:F0}m  aboveAtmo={aboveAtmo}");

            rm.CheckStaging(skipDuringLiftoff: state == AscentState.Liftoff);

            switch (state)
            {
                // ──────────────────────────────────────────────────────────────
                case AscentState.Liftoff:
                    rm.SetThrottle(1f);
                    rm.SetPitch(90f);
                    if (altitude > PITCH_START_ALTITUDE)
                    {
                        turnStartAltitude = altitude;
                        state = AscentState.PitchOver;
                        if (debug) UnityEngine.Debug.Log($"[Ascent] Liftoff → PitchOver at {altitude:F0}m");
                    }
                    break;

                // ──────────────────────────────────────────────────────────────
                case AscentState.PitchOver:
                {
                    double turnEndAlt = GetTurnEndAltitude(atmosphereH);
                    double progress   = Math.Max(0, Math.Min(1,
                        (altitude - turnStartAltitude) / (turnEndAlt - turnStartAltitude)));
                    float pitch = progress < 1.0
                        ? Math.Max(MIN_PITCH, 90f - (float)Math.Pow(progress, TURN_SHAPE_EXPONENT) * (90f - TURN_END_ANGLE))
                        : TURN_END_ANGLE;

                    rm.SetPitch(pitch);
                    rm.SetThrottle(altitude < turnStartAltitude + 5000.0 ? 1f : ThrottleForApoapsis(apoapsis));

                    // Rescue: inside atmosphere with unsafe periapsis → burn prograde
                    if (!aboveAtmo && !IsPeriSafe(periAlt, atmosphereH))
                    {
                        rm.AlignPrograde();
                        rm.SetThrottle(0.6f);
                        if (debug && UnityEngine.Time.frameCount % 30 == 0)
                            UnityEngine.Debug.Log($"[Ascent] PitchOver rescue burn: periAlt={periAlt:F0}m");
                        break;
                    }

                    if (debug && UnityEngine.Time.frameCount % 60 == 0)
                        UnityEngine.Debug.Log($"[Ascent] PitchOver: pitch={pitch:F1}° progress={progress:F3}");

                    double tta = om.GetTimeToApoapsis();
                    if (apoapsis >= targetRadius - APOAPSIS_TARGET_MARGIN && tta > 0.5)
                    {
                        rm.CutEngines();
                        state       = AscentState.Coast;
                        coastFrames = 0;
                        if (debug) UnityEngine.Debug.Log($"[Ascent] PitchOver → Coast  apoAlt={apoAlt:F0}m  TTA={tta:F1}s");
                    }
                    break;
                }

                // ──────────────────────────────────────────────────────────────
                case AscentState.Coast:
                {
                    rm.SetTurnAxis(0f);
                    coastFrames++;

                    // Rescue: inside atmosphere with unsafe periapsis
                    if (!aboveAtmo && !IsPeriSafe(periAlt, atmosphereH))
                    {
                        rm.AlignPrograde();
                        rm.SetThrottle(0.6f);
                        if (debug && coastFrames % 30 == 0)
                            UnityEngine.Debug.Log($"[Ascent] Coast rescue burn: periAlt={periAlt:F0}m");
                        break;
                    }

                    double timeToApo = om.GetTimeToApoapsis();
                    if (aboveAtmo && WorldTime.CanTimewarp(false, false))
                    {
                        double hSpeed   = rm.GetHorizontalSpeed();
                        double tgtSpeed = om.GetTransferSpeed(rocket.location.Value.Radius, targetRadius);
                        double dv       = Math.Max(0, tgtSpeed - hSpeed);
                        double burnTime = om.EstimateBurnDuration(dv, rm.GetMaxAcceleration());
                        double lead     = Math.Max(COAST_TO_CIRC_LEAD, burnTime * 0.5 + COAST_TO_CIRC_BUFFER);
                        SetTimewarp(timeToApo - lead);
                    }
                    else if (WorldTime.main.timewarpSpeed > 1)
                    {
                        WorldTime.main.StopTimewarp(false);
                    }

                    if (debug && coastFrames % 60 == 0)
                        UnityEngine.Debug.Log($"[Ascent] Coast: TTA={timeToApo:F1}s  aboveAtmo={aboveAtmo}");

                    double distToApo = Math.Abs(apoAlt - altitude);
                    double hNow      = rm.GetHorizontalSpeed();
                    double tNow      = om.GetTransferSpeed(rocket.location.Value.Radius, targetRadius);
                    double dvNow     = Math.Max(0, tNow - hNow);
                    double burnEst   = om.EstimateBurnDuration(dvNow, rm.GetMaxAcceleration());
                    double leadNow   = Math.Max(COAST_TO_CIRC_LEAD, burnEst * 0.5 + COAST_TO_CIRC_BUFFER);

                    bool timeReady = (timeToApo - leadNow) <= 0 && WorldTime.main.timewarpSpeed <= 1;
                    bool distReady = distToApo < COAST_TO_CIRC_DIST;
                    if (timeReady || distReady)
                    {
                        WorldTime.main.StopTimewarp(false);
                        rm.CutEngines();
                        burnDuration         = om.EstimateBurnDuration(dvNow, rm.GetMaxAcceleration());
                        circularizeEntryTime = WorldTime.main.worldTime;
                        throttleUnlockTime   = circularizeEntryTime + CIRC_SETTLE_TIME;
                        state = AscentState.Circularize;
                        if (debug) UnityEngine.Debug.Log($"[Ascent] Coast → Circularize  dv={dvNow:F1}m/s  burnEst={burnDuration:F1}s");
                    }
                    break;
                }

                // ──────────────────────────────────────────────────────────────
                case AscentState.Circularize:
                {
                    double timeNow   = WorldTime.main.worldTime;
                    double timeToApo = om.GetTimeToApoapsis();
                    double distToApo = Math.Abs(apoAlt - altitude);
                    bool   inWindow  = distToApo <= CIRC_BURN_WINDOW_DIST || Math.Abs(timeToApo) <= CIRC_BURN_WINDOW_TIME;
                    bool   settled   = timeNow >= throttleUnlockTime;

                    // Attitude — prograde + small peri correction
                    float  progradeAngle = rm.GetProgradeAngle();
                    float  pitch         = progradeAngle;
                    double periErr       = targetRadius - periapsis;
                    bool   bigPeriErr    = periErr > Math.Max(MAJOR_PERI_ERROR, targetAltitude * 0.05);
                    if (Math.Abs(periErr) > CIRC_PERI_TOLERANCE)
                        pitch += Mathf.Clamp((float)(periErr / PERI_CORR_GAIN), -PERI_CORR_MAX, PERI_CORR_MAX);

                    rm.SetPitch(pitch);
                    bool orientReady = rm.IsPitchSettled(pitch) && settled;

                    // Throttle — target circular speed at actual apoapsis
                    double mu      = rocket.location.planet.Value.mass;
                    double rApo    = Math.Max(apoapsis, planetR + 1.0);
                    double vCirc   = Math.Sqrt(mu / rApo);
                    double hSpeed  = rm.GetHorizontalSpeed();
                    double dvNeeded = vCirc - hSpeed;
                    double maxAccel = rm.GetMaxAcceleration();

                    float circThrottle = 0f;
                    if (maxAccel > 0.00001 && dvNeeded > 0 && timeToApo > -1.0 && inWindow)
                    {
                        double t = Math.Max(Math.Abs(timeToApo), 0.5);
                        circThrottle = Mathf.Clamp01((float)(dvNeeded / t / maxAccel));
                    }

                    float periThrottle = 0f;
                    bool nearApo = distToApo <= CIRC_NEAR_APO_DIST || Math.Abs(timeToApo) <= CIRC_NEAR_APO_TIME;
                    if (nearApo && periErr > CIRC_PERI_TOLERANCE)
                    {
                        float pMin = bigPeriErr ? 0.05f : 0f;
                        float pMax = bigPeriErr ? 0.35f : 0.15f;
                        periThrottle = Mathf.Clamp((float)(periErr / PERI_THROTTLE_GAIN), pMin, pMax);
                    }

                    float finalThrottle = inWindow && orientReady ? Mathf.Max(circThrottle, periThrottle) : 0f;
                    rm.SetThrottle(finalThrottle);

                    if (debug && UnityEngine.Time.frameCount % 30 == 0)
                        UnityEngine.Debug.Log($"[Ascent] Circ: throttle={finalThrottle:F2}  dv={dvNeeded:F1}m/s  hSpeed={hSpeed:F1}  vCirc={vCirc:F1}  TTA={timeToApo:F1}s  periErr={periErr:F0}m");

                    // Completion
                    double closePeri = Math.Max(3000.0, Math.Min(8000.0, targetAltitude * 0.02));
                    double closeApo  = Math.Max(2500.0, Math.Min(25000.0, targetAltitude * 0.03));
                    bool exact = Math.Abs(periapsis - targetRadius) < CIRC_PERI_TOLERANCE && Math.Abs(dvNeeded) <= CIRC_VEL_TOLERANCE;
                    bool close = Math.Abs(periapsis - targetRadius) <= closePeri && Math.Abs(dvNeeded) <= CIRC_VEL_TOLERANCE && Math.Abs(apoapsis - targetRadius) <= closeApo;
                    if (exact || close)
                    {
                        rm.CutEngines();
                        if (debug) UnityEngine.Debug.Log(exact ? "[Ascent] Circularize complete" : "[Ascent] Circularize close enough");
                        if (pendingTransferBurn) BeginTransferBurn();
                        else Stop();
                        break;
                    }

                    // Timeout
                    double elapsed = timeNow - circularizeEntryTime;
                    double timeout = Math.Max(CIRC_MAX_DURATION_BASE, burnDuration * 3.0 + 30.0);
                    if (elapsed > timeout)
                    {
                        if (debug) UnityEngine.Debug.Log($"[Ascent] Circularize timeout after {elapsed:F1}s");
                        rm.CutEngines();
                        Stop();
                    }
                    break;
                }

                // ──────────────────────────────────────────────────────────────
                case AscentState.TransferBurn:
                {
                    rm.AlignPrograde();
                    bool ready    = rm.IsPitchSettled(rm.GetProgradeAngle()) && WorldTime.main.worldTime >= throttleUnlockTime;
                    double apoErr = targetRadius - apoapsis;
                    float  thr    = 0f;
                    if (apoErr > 0 && ready)
                    {
                        thr = ThrottleForApoapsis(apoapsis);
                        if (apoErr < 5000) thr = Mathf.Min(thr, 0.25f);
                        if (apoErr < 2000) thr = Mathf.Min(thr, 0.10f);
                    }
                    rm.SetThrottle(thr);

                    if (debug && UnityEngine.Time.frameCount % 30 == 0)
                        UnityEngine.Debug.Log($"[Ascent] Transfer: throttle={thr:F2}  apoErr={apoErr:F0}m");

                    if (apoapsis >= targetRadius - APOAPSIS_TARGET_MARGIN)
                    {
                        rm.CutEngines();
                        state       = AscentState.Coast;
                        coastFrames = 0;
                        if (debug) UnityEngine.Debug.Log($"[Ascent] Transfer → Coast  apoAlt={apoAlt:F0}m");
                    }
                    break;
                }
            }
        }

        // ══════════════════════════════════════════════════════════════════════
        // Private helpers
        // ══════════════════════════════════════════════════════════════════════

        private float ThrottleForApoapsis(double apoapsis)
        {
            if (apoapsis <= 0) return 1f;
            double diff = targetRadius - apoapsis;
            if (diff <= 0)   return 0f;
            if (diff > 3000) return 1f;
            return Mathf.Lerp(0.4f, 1f, (float)(diff / 3000.0));
        }

        private bool IsPeriSafe(double periAlt, double atmosphereH)
        {
            double threshold = atmosphereH > 100 ? atmosphereH + 1000 : 2000;
            return periAlt > threshold;
        }

        private double GetTurnEndAltitude(double atmosphereH)
        {
            double desired = atmosphereH > 1.0
                ? Math.Min(targetAltitude * TURN_TARGET_ALTITUDE_FACTOR, atmosphereH * TURN_ATMOSPHERE_ALTITUDE_FACTOR)
                : targetAltitude * TURN_NO_ATMOSPHERE_FACTOR;
            desired = Math.Max(turnStartAltitude + TURN_MIN_END_ALTITUDE, desired);
            double max = Math.Max(turnStartAltitude + 1000.0, targetAltitude - 2000.0);
            return Math.Max(turnStartAltitude + 1.0, Math.Min(desired, max));
        }

        private void SetTimewarp(double secondsUntilStop)
        {
            if      (secondsUntilStop > 120) WorldTime.main.SetState(WorldTime.MaxTimewarpSpeed, false, false);
            else if (secondsUntilStop > 60)  WorldTime.main.SetState(50, false, false);
            else if (secondsUntilStop > 30)  WorldTime.main.SetState(25, false, false);
            else if (secondsUntilStop > 15)  WorldTime.main.SetState(10, false, false);
            else if (secondsUntilStop > 8)   WorldTime.main.SetState(5, false, false);
            else if (WorldTime.main.timewarpSpeed > 1) WorldTime.main.StopTimewarp(false);
        }

        private void BeginTransferBurn()
        {
            targetAltitude      = requestedTargetAltitude;
            targetRadius        = targetAltitude + rocket.location.planet.Value.Radius;
            pendingTransferBurn = false;
            state               = AscentState.TransferBurn;
            throttleUnlockTime  = WorldTime.main.worldTime + CIRC_SETTLE_TIME;
            rm.CutEngines();
            if (debug) UnityEngine.Debug.Log($"[Ascent] Transfer burn to {targetAltitude:F0}m");
        }

        private void RefreshSettings(bool log = false)
        {
            float prev = requestedTargetAltitude;
            requestedTargetAltitude = Mathf.Max(MIN_TARGET_ALTITUDE, Settings.data.targetOrbitAltitude);
            if (!IsActive)
            {
                targetAltitude      = GetInsertionAltitude(requestedTargetAltitude);
                pendingTransferBurn = requestedTargetAltitude > targetAltitude + 0.1f;
            }
            if (log && debug && Math.Abs(prev - requestedTargetAltitude) > 0.1f)
                UnityEngine.Debug.Log($"[Ascent] Target updated to {requestedTargetAltitude:F0}m");
        }

        private void InitMissionTargets()
        {
            requestedTargetAltitude = Mathf.Max(MIN_TARGET_ALTITUDE, Settings.data.targetOrbitAltitude);
            targetAltitude          = GetInsertionAltitude(requestedTargetAltitude);
            pendingTransferBurn     = requestedTargetAltitude > targetAltitude + 0.1f;
            MsgDrawer.main.Log(pendingTransferBurn
                ? $"Autopilot: {requestedTargetAltitude / 1000f:0.#} km via {targetAltitude / 1000f:0.#} km parking orbit"
                : $"Autopilot: {targetAltitude / 1000f:0.#} km orbit");
        }

        private float GetInsertionAltitude(float requested)
        {
            float parking = GetParkingOrbitAltitude();
            return requested > parking + DIRECT_ASCENT_MAX_EXTRA_ALTITUDE ? parking : requested;
        }

        private float GetParkingOrbitAltitude()
        {
            float safeAtmo = (float)Math.Max(om.GetAtmosphereHeight() + PARKING_ORBIT_ATMOSPHERE_MARGIN, TURN_MIN_END_ALTITUDE);
            return Math.Max(PARKING_ORBIT_MIN_ALTITUDE, safeAtmo);
        }

        /// <summary>Human-readable current phase, shown in the GUI status label.</summary>
        public string StateDescription
        {
            get
            {
                switch (state)
                {
                    case AscentState.Liftoff:      return "Liftoff";
                    case AscentState.PitchOver:    return "Gravity turn";
                    case AscentState.Coast:        return "Coasting to apoapsis";
                    case AscentState.Circularize:  return "Circularizing";
                    case AscentState.TransferBurn: return "Transfer burn";
                    default:                       return "Idle";
                }
            }
        }
    }
}