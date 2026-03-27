// AutopilotUpdater.cs
using SFS.UI;
using SFS.World;
using UnityEngine;

namespace RevolutionlessAutopilot
{
    public class AutopilotUpdater : MonoBehaviour
    {
        public static AutopilotUpdater Instance { get; private set; }

        private Rocket currentRocket;
        private AscentAutopilot ascentAutopilot;
        private LandingAutopilot landingAutopilot;

        private void Awake()
        {
            if (Instance != null)
            {
                Destroy(gameObject);
                return;
            }
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

                // (RU) Останавливаем все автопилоты если ракета потеряна | (EN) Stop all autopilots if rocket is lost
                if (ascentAutopilot != null && ascentAutopilot.IsActive)
                    ascentAutopilot.Stop();
                if (landingAutopilot != null && landingAutopilot.IsActive)
                    landingAutopilot.Stop();

                return;
            }

            if (ascentAutopilot != null && ascentAutopilot.IsActive)
                ascentAutopilot.Update();

            if (landingAutopilot != null && landingAutopilot.IsActive)
                landingAutopilot.Update();
        }

        private void FixedUpdate()
        {
            if (ascentAutopilot != null && ascentAutopilot.IsActive)
                ascentAutopilot.FixedUpdate();

            if (landingAutopilot != null && landingAutopilot.IsActive)
                landingAutopilot.FixedUpdate();
        }

        // (RU) Управление автопилотом подъёма | (EN) Ascent autopilot control
        public void ToggleAscent()
        {
            if (currentRocket == null)
            {
                MsgDrawer.main.Log("No rocket controlled.");
                return;
            }

            // (RU) Останавливаем посадочный автопилот если он активен | (EN) Stop landing autopilot if it is active
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
        }

        // (RU) Управление посадочным автопилотом | (EN) Landing autopilot control
        public void ToggleLanding()
        {
            if (currentRocket == null)
            {
                MsgDrawer.main.Log("No rocket controlled.");
                return;
            }

            // (RU) Останавливаем автопилот подъёма если он активен | (EN) Stop ascent autopilot if it is active
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
        }

        // ──────────────────────────────────────────────
        // (RU) Вспомогательные методы | (EN) Helper methods
        // ──────────────────────────────────────────────

        public float GetRecommendedLowOrbitAltitudeMeters()
        {
            if (currentRocket?.location?.planet?.Value == null)
                return 40000f;

            return Mathf.Max(5000f, (float)currentRocket.location.planet.Value.AtmosphereHeightPhysics + 5000f);
        }
    }
}