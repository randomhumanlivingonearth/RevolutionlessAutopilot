// AutopilotUpdater.cs - без изменений
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
                if (ascentAutopilot != null && ascentAutopilot.IsActive)
                    ascentAutopilot.Stop();
                return;
            }

            if (ascentAutopilot != null && ascentAutopilot.IsActive)
            {
                ascentAutopilot.Update();
            }
        }

        private void FixedUpdate()
        {
            if (ascentAutopilot != null && ascentAutopilot.IsActive)
            {
                ascentAutopilot.FixedUpdate();
            }
        }

        public void ToggleAscent()
        {
            if (currentRocket == null)
            {
                MsgDrawer.main.Log("No rocket controlled.");
                return;
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

        public float GetRecommendedLowOrbitAltitudeMeters()
        {
            if (currentRocket?.location?.planet?.Value == null)
                return 40000f;

            return Mathf.Max(5000f, (float)currentRocket.location.planet.Value.AtmosphereHeightPhysics + 5000f);
        }
    }
}
