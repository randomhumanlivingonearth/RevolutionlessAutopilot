using SFS.UI;
using SFS.World;
using UnityEngine;

namespace RevolutionlessAutopilot
{
    public class AutopilotUpdater : MonoBehaviour
    {
        public static AutopilotUpdater Instance { get; private set; }

        private Rocket currentRocket;
        private AscentAutopilot  ascentAutopilot;
        private LandingAutopilot landingAutopilot;

        // ── State exposed to GUI ───────────────────────────────────────────────

        public bool IsAscentActive  => ascentAutopilot  != null && ascentAutopilot.IsActive;
        public bool IsLandingActive => landingAutopilot != null && landingAutopilot.IsActive;
        public bool HasRocket       => currentRocket    != null;

        public string AscentStatus  => ascentAutopilot  != null && ascentAutopilot.IsActive
                                        ? ascentAutopilot.StateDescription  : "Idle";
        public string LandingStatus => landingAutopilot != null && landingAutopilot.IsActive
                                        ? landingAutopilot.StateDescription : "Idle";

        // ── Lifecycle ──────────────────────────────────────────────────────────

        private void Awake()
        {
            if (Instance != null) { Destroy(gameObject); return; }
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }

        private void Update()
        {
            if (PlayerController.main?.player?.Value is Rocket rocket)
            {
                currentRocket = rocket;
            }
            else
            {
                currentRocket = null;
                if (ascentAutopilot  != null && ascentAutopilot.IsActive)  ascentAutopilot.Stop();
                if (landingAutopilot != null && landingAutopilot.IsActive) landingAutopilot.Stop();
                GUI.Refresh();
                return;
            }

            if (ascentAutopilot  != null && ascentAutopilot.IsActive)  ascentAutopilot.Update();
            if (landingAutopilot != null && landingAutopilot.IsActive) landingAutopilot.Update();

            // Refresh UI labels ~4× per second
            if (Time.frameCount % 15 == 0)
                GUI.Refresh();
        }

        private void FixedUpdate()
        {
            if (ascentAutopilot  != null && ascentAutopilot.IsActive)  ascentAutopilot.FixedUpdate();
            if (landingAutopilot != null && landingAutopilot.IsActive) landingAutopilot.FixedUpdate();
        }

        // ── Ascent ─────────────────────────────────────────────────────────────

        public void ToggleAscent()
        {
            if (currentRocket == null) { MsgDrawer.main.Log("No rocket controlled."); return; }

            if (landingAutopilot != null && landingAutopilot.IsActive)
            {
                landingAutopilot.Stop();
                MsgDrawer.main.Log("Landing autopilot stopped.");
            }

            if (ascentAutopilot == null)
                ascentAutopilot = new AscentAutopilot(currentRocket);
            else
                ascentAutopilot.SetRocket(currentRocket);

            if (ascentAutopilot.IsActive)
            {
                ascentAutopilot.Stop();
                MsgDrawer.main.Log("Ascent autopilot stopped.");
            }
            else
            {
                ascentAutopilot.Start();
                MsgDrawer.main.Log("Ascent autopilot started.");
            }

            GUI.Refresh();
        }

        // ── Landing ────────────────────────────────────────────────────────────

        public void ToggleLanding()
        {
            if (currentRocket == null) { MsgDrawer.main.Log("No rocket controlled."); return; }

            if (ascentAutopilot != null && ascentAutopilot.IsActive)
            {
                ascentAutopilot.Stop();
                MsgDrawer.main.Log("Ascent autopilot stopped.");
            }

            if (landingAutopilot == null)
                landingAutopilot = new LandingAutopilot(currentRocket);
            else
                landingAutopilot.SetRocket(currentRocket);

            if (landingAutopilot.IsActive)
            {
                landingAutopilot.Stop();
                MsgDrawer.main.Log("Landing autopilot stopped.");
            }
            else
            {
                landingAutopilot.Start();
                MsgDrawer.main.Log("Landing autopilot started.");
            }

            GUI.Refresh();
        }

        // ── Helpers ────────────────────────────────────────────────────────────

        public float GetRecommendedLowOrbitAltitudeMeters()
        {
            if (currentRocket?.location?.planet?.Value == null) return 40000f;
            return Mathf.Max(5000f, (float)currentRocket.location.planet.Value.AtmosphereHeightPhysics + 5000f);
        }
    }
}