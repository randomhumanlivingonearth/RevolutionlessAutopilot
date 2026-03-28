using System;
using SFS;
using SFS.UI;
using SFS.World;
using UnityEngine;

namespace RevolutionlessAutopilot
{
    public enum LandingState
    {
        Idle,
        Deorbit,        // Retrograde burn to lower periapsis below surface
        Reentry,        // Engines off, hold retrograde through atmosphere
        AerobrakeCoast, // Wait for speed to drop to suicide burn ignition point
        Flip,           // Rotate to retrograde (airless) or straight-down (post-aerobrake)
        SuicideBurn,    // Kill velocity, blend to vertical descent, soft land
        TouchdownIdle   // Complete
    }

    public class LandingAutopilot
    {
        // ── Dependencies ───────────────────────────────────────────────────────
        private Rocket        rocket;
        private RocketManager rm;
        private OrbitManager  om;

        // ── State ──────────────────────────────────────────────────────────────
        private LandingState state = LandingState.Idle;
        private double throttleUnlockTime;
        private bool   flipComplete = false;
        private int    coastFrames  = 0;

        private bool debug = true;

        // ══════════════════════════════════════════════════════════════════════
        // Constants
        // ══════════════════════════════════════════════════════════════════════

        // Deorbit
        private const double DEORBIT_PERIAPSIS_TARGET    = -10000.0; // m (underground)
        private const double DEORBIT_PERIAPSIS_TOLERANCE = 2000.0;
        private const double DEORBIT_MIN_ALTITUDE        = 5000.0;

        // Reentry / aerobrake
        private const double REENTRY_HANDOFF_ALTITUDE    = 8000.0;
        private const double REENTRY_MAX_COAST_SPEED     = 500.0;
        private const double FLIP_START_ALTITUDE         = 6000.0;

        // Flip
        private const float  FLIP_TARGET_PITCH           = -90f;  // straight down
        private const float  FLIP_PITCH_TOLERANCE        = 5f;

        // Suicide burn
        private const double SUICIDE_BURN_MARGIN         = 150.0;  // m above computed ignition point
        private const float  SUICIDE_BURN_MIN_THROTTLE   = 0.05f;
        private const double SOFT_LANDING_ALTITUDE       = 200.0;
        private const double SOFT_LANDING_TARGET_SPEED   = 8.0;   // m/s downward
        private const double TOUCHDOWN_ALT               = 10.0;
        private const double TOUCHDOWN_SPEED             = 5.0;

        // Control
        private const double SETTLE_TIME                 = 0.75;

        // ══════════════════════════════════════════════════════════════════════
        // Lifecycle
        // ══════════════════════════════════════════════════════════════════════

        public bool IsActive { get; private set; }

        public LandingAutopilot(Rocket rocket)
        {
            this.rocket = rocket;
            rm = new RocketManager(rocket) { Debug = false };
            om = new OrbitManager(rocket)  { Debug = false };
            if (debug) UnityEngine.Debug.Log("[Landing] Initialized");
        }

        public void SetRocket(Rocket rocket)
        {
            this.rocket = rocket;
            rm.SetRocket(rocket);
            om.SetRocket(rocket);
        }

        public void Start()
        {
            if (rocket == null) return;
            IsActive           = true;
            state              = LandingState.Deorbit;
            flipComplete       = false;
            throttleUnlockTime = 0;
            coastFrames        = 0;
            rm.ResetStagingHistory();
            if (debug) UnityEngine.Debug.Log("[Landing] STARTED");
        }

        public void Stop()
        {
            IsActive = false;
            state    = LandingState.Idle;
            rm.CutEngines();
            rm.SetTurnAxis(0f);
            if (debug) UnityEngine.Debug.Log("[Landing] STOPPED");
        }

        public void Update() { }

        // ══════════════════════════════════════════════════════════════════════
        // Main loop
        // ══════════════════════════════════════════════════════════════════════

        public void FixedUpdate()
        {
            if (!IsActive || rocket == null) return;

            double altitude    = rm.GetAltitude();
            double speed       = rocket.location.velocity.Value.magnitude;
            double vertSpeed   = rm.GetVerticalSpeed();
            double hSpeed      = rm.GetHorizontalSpeed();
            double localG      = rm.GetLocalGravity();
            double atmosphereH = om.GetAtmosphereHeight();
            bool   hasAtmo     = om.HasAtmosphere();
            bool   aboveAtmo   = altitude > atmosphereH;

            rm.CheckStaging();

            if (debug && UnityEngine.Time.frameCount % 60 == 0)
                UnityEngine.Debug.Log($"[Landing] {state}  alt={altitude:F0}m  speed={speed:F1}m/s  vspeed={vertSpeed:F1}m/s  hspeed={hSpeed:F1}m/s");

            switch (state)
            {
                // ──────────────────────────────────────────────────────────────
                case LandingState.Deorbit:
                {
                    double periAlt = om.GetPeriapsisAltitude();

                    // Exit if periapsis low enough
                    if (periAlt <= DEORBIT_PERIAPSIS_TARGET + DEORBIT_PERIAPSIS_TOLERANCE ||
                        altitude < DEORBIT_MIN_ALTITUDE)
                    {
                        rm.CutEngines();
                        state = hasAtmo ? LandingState.Reentry : LandingState.Flip;
                        flipComplete = false;
                        if (debug) UnityEngine.Debug.Log($"[Landing] Deorbit → {state}  periAlt={periAlt:F0}m");
                        break;
                    }

                    // Burn retrograde to lower periapsis
                    float retro = rm.GetProgradeAngle() + 180f;
                    rm.SetPitch(retro);
                    bool settled = rm.IsPitchSettled(retro);

                    if (settled && aboveAtmo && altitude > DEORBIT_MIN_ALTITUDE)
                    {
                        double periErr  = periAlt - DEORBIT_PERIAPSIS_TARGET;
                        float  throttle = Mathf.Max(0.1f, (float)Math.Min(1.0, periErr / 20000.0));
                        rm.SetThrottle(throttle);
                    }
                    else
                    {
                        rm.CutEngines();
                    }

                    if (debug && UnityEngine.Time.frameCount % 60 == 0)
                        UnityEngine.Debug.Log($"[Landing] Deorbit: periAlt={periAlt:F0}m  settled={settled}");
                    break;
                }

                // ──────────────────────────────────────────────────────────────
                case LandingState.Reentry:
                {
                    rm.CutEngines();
                    rm.AlignRetrograde();

                    coastFrames++;
                    if (debug && coastFrames % 60 == 0)
                        UnityEngine.Debug.Log($"[Landing] Reentry: alt={altitude:F0}m  speed={speed:F1}m/s");

                    if (altitude < REENTRY_HANDOFF_ALTITUDE && speed < REENTRY_MAX_COAST_SPEED)
                    {
                        state       = LandingState.AerobrakeCoast;
                        coastFrames = 0;
                        if (debug) UnityEngine.Debug.Log("[Landing] Reentry → AerobrakeCoast");
                    }
                    break;
                }

                // ──────────────────────────────────────────────────────────────
                case LandingState.AerobrakeCoast:
                {
                    rm.CutEngines();
                    rm.AlignRetrograde();

                    double burnAlt = SuicideBurnAltitude(speed, localG, rm.GetMaxAcceleration());
                    coastFrames++;
                    if (debug && coastFrames % 30 == 0)
                        UnityEngine.Debug.Log($"[Landing] AerobrakeCoast: alt={altitude:F0}m  burnAlt={burnAlt:F0}m  speed={speed:F1}m/s");

                    if (altitude < FLIP_START_ALTITUDE || altitude < burnAlt + SUICIDE_BURN_MARGIN * 3)
                    {
                        state        = LandingState.Flip;
                        flipComplete = false;
                        if (debug) UnityEngine.Debug.Log("[Landing] AerobrakeCoast → Flip");
                    }
                    break;
                }

                // ──────────────────────────────────────────────────────────────
                case LandingState.Flip:
                {
                    rm.CutEngines();

                    // Airless body: target retrograde (still fast horizontally).
                    // Post-aerobrake: go straight down (horizontal speed already killed).
                    float flipTarget = hasAtmo ? FLIP_TARGET_PITCH : rm.GetProgradeAngle() + 180f;
                    rm.SetPitch(flipTarget);

                    float pitchErr = Mathf.Abs(RocketManager.NormalizeAngle(flipTarget - rocket.GetRotation()));
                    flipComplete   = pitchErr <= FLIP_PITCH_TOLERANCE;

                    if (debug && UnityEngine.Time.frameCount % 30 == 0)
                        UnityEngine.Debug.Log($"[Landing] Flip: target={flipTarget:F1}°  err={pitchErr:F1}°  done={flipComplete}");

                    if (flipComplete)
                    {
                        state              = LandingState.SuicideBurn;
                        throttleUnlockTime = WorldTime.main.worldTime + SETTLE_TIME;
                        if (debug) UnityEngine.Debug.Log("[Landing] Flip → SuicideBurn");
                    }
                    break;
                }

                // ──────────────────────────────────────────────────────────────
                case LandingState.SuicideBurn:
                {
                    // Touchdown check
                    if (altitude < TOUCHDOWN_ALT && Math.Abs(vertSpeed) < TOUCHDOWN_SPEED)
                    {
                        rm.CutEngines();
                        rm.SetTurnAxis(0f);
                        state = LandingState.TouchdownIdle;
                        MsgDrawer.main.Log("Landing complete.");
                        if (debug) UnityEngine.Debug.Log($"[Landing] Touchdown! speed={speed:F1}m/s");
                        Stop();
                        break;
                    }

                    // Wait for attitude to settle after flip
                    if (WorldTime.main.worldTime < throttleUnlockTime)
                    {
                        rm.CutEngines();
                        break;
                    }

                    // Attitude: blend from retrograde to straight-down as horizontal speed falls off
                    float burnPitch;
                    if (speed > 0.1)
                    {
                        // blendT: 0 = full retrograde, 1 = straight down
                        // starts blending when |vertSpeed| > 30% of total speed
                        float blendT = Mathf.Clamp01((float)(Math.Abs(vertSpeed) / speed) - 0.3f) / 0.7f;
                        float retro  = rm.GetProgradeAngle() + 180f;
                        burnPitch    = Mathf.LerpAngle(retro, FLIP_TARGET_PITCH, blendT);
                    }
                    else
                    {
                        burnPitch = FLIP_TARGET_PITCH;
                    }
                    rm.SetPitch(burnPitch);

                    float finalThrottle;

                    if (altitude < SOFT_LANDING_ALTITUDE && Math.Abs(hSpeed) < 5.0)
                    {
                        // Soft landing PID — target vertical speed only once horizontal speed is killed
                        double targetVS  = -SOFT_LANDING_TARGET_SPEED;
                        double spdErr    = vertSpeed - targetVS;  // positive = falling faster than target
                        double gravComp  = localG / Math.Max(rm.GetMaxAcceleration(), 0.001);
                        finalThrottle    = Mathf.Clamp((float)(gravComp + spdErr / 10.0), SUICIDE_BURN_MIN_THROTTLE, 1f);

                        if (debug && UnityEngine.Time.frameCount % 30 == 0)
                            UnityEngine.Debug.Log($"[Landing] SoftLanding: alt={altitude:F0}m  vspeed={vertSpeed:F1}m/s  throttle={finalThrottle:F2}");
                    }
                    else
                    {
                        // Main burn: full throttle until speed is low
                        double burnAlt   = SuicideBurnAltitude(speed, localG, rm.GetMaxAcceleration());
                        bool shouldBurn  = altitude <= burnAlt + SUICIDE_BURN_MARGIN;

                        if (shouldBurn)
                        {
                            double maxAccel  = rm.GetMaxAcceleration();
                            double gravComp  = localG / Math.Max(maxAccel, 0.001);
                            // Blend from full throttle → gravity compensation as speed drops below 30 m/s
                            float speedFrac  = Mathf.Clamp01((float)(speed / 30.0));
                            finalThrottle    = Mathf.Clamp((float)(speedFrac + (1.0 - speedFrac) * gravComp),
                                                            SUICIDE_BURN_MIN_THROTTLE, 1f);
                        }
                        else
                        {
                            finalThrottle = 0f;
                        }

                        if (debug && UnityEngine.Time.frameCount % 30 == 0)
                            UnityEngine.Debug.Log($"[Landing] SuicideBurn: alt={altitude:F0}m  burnAlt={burnAlt:F0}m  speed={speed:F1}m/s  hspeed={hSpeed:F1}m/s  pitch={burnPitch:F1}°  throttle={finalThrottle:F2}");
                    }

                    rm.SetThrottle(finalThrottle);
                    break;
                }

                case LandingState.TouchdownIdle:
                    break;
            }
        }

        // ══════════════════════════════════════════════════════════════════════
        // Helpers
        // ══════════════════════════════════════════════════════════════════════

        /// <summary>
        /// Altitude at which a full-throttle burn must start to reduce 'speed' to zero.
        /// h = v² / (2 * (a - g))
        /// Always uses total speed — on an airless body from orbit, horizontal speed
        /// dominates and must be killed before the vertical component matters.
        /// </summary>
        private double SuicideBurnAltitude(double speed, double gravity, double acceleration)
        {
            double netDecel = acceleration - gravity;
            if (netDecel <= 0.01)
            {
                if (debug) UnityEngine.Debug.Log("[Landing] WARNING: TWR < 1, can't decelerate");
                return double.PositiveInfinity;
            }
            return (speed * speed) / (2.0 * netDecel);
        }

        /// <summary>Human-readable current phase, shown in the GUI status label.</summary>
        public string StateDescription
        {
            get
            {
                switch (state)
                {
                    case LandingState.Deorbit:        return "Deorbit burn";
                    case LandingState.Reentry:        return "Re-entry";
                    case LandingState.AerobrakeCoast: return "Aerobrake coast";
                    case LandingState.Flip:           return "Flip manoeuvre";
                    case LandingState.SuicideBurn:    return "Suicide burn";
                    case LandingState.TouchdownIdle:  return "Landed";
                    default:                          return "Idle";
                }
            }
        }
    }
}