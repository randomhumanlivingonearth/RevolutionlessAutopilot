using System.Globalization;
using SFS.UI;
using SFS.UI.ModGUI;
using TMPro;
using UITools;
using UnityEngine;
using Type = SFS.UI.ModGUI.Type;

namespace RevolutionlessAutopilot
{
    public static class GUI
    {
        // ── Window handles ─────────────────────────────────────────────────────
        private static GameObject mainHolder;
        private static ClosableWindow mainWindow;
        private static readonly int mainWindowID = Builder.GetRandomID();
        private const string mainWindowPosKey = "RevolutionlessAutopilot.MainWindow";

        private static GameObject ascentHolder;
        private static ClosableWindow ascentWindow;
        private static readonly int ascentWindowID = Builder.GetRandomID();
        private const string ascentWindowPosKey = "RevolutionlessAutopilot.AscentWindow";

        private static GameObject landingHolder;
        private static ClosableWindow landingWindow;
        private static readonly int landingWindowID = Builder.GetRandomID();
        private const string landingWindowPosKey = "RevolutionlessAutopilot.LandingWindow";

        // ── Dynamic UI elements ────────────────────────────────────────────────
        // Labels whose text gets swapped by Refresh()
        // Button labels use TextMeshProUGUI (TMP) since ModGUI buttons don't expose a Label child
        private static TextMeshProUGUI ascentButtonLabel;   // "Ascent" / "Ascent ●"
        private static TextMeshProUGUI landingButtonLabel;  // "Landing" / "Landing ●"
        private static TextMeshProUGUI ascentActionLabel;   // "Start Ascent" / "Stop Ascent"
        private static TextMeshProUGUI landingActionLabel;  // "Start Landing" / "Stop Landing"
        // Status labels are created via Builder.CreateLabel so they remain Label
        private static Label ascentStatusLabel;   // current state description
        private static Label landingStatusLabel;

        // ── Dev mode ───────────────────────────────────────────────────────────
        private static bool devModeEnabled = false;
        private const string DEV_UNLOCK_COMMAND = "/DEV MODE123";
        private static SFS.UI.ModGUI.Button landingButton;
        private static Container consoleRow;

        // ── Orbit altitude input ───────────────────────────────────────────────
        private const float minTargetOrbitKm = 1f;
        private const float fallbackRecommendedOrbitMeters = 40000f;
        private static readonly float[] targetAdjustButtonsKm = { 1f, 10f, 100f, 1000f, 10000f };
        private static TextInput targetAltitudeInput;
        private static string pendingTargetOrbitText;

        private static bool ascentWindowVisible;
        private static bool landingWindowVisible;

        // ══════════════════════════════════════════════════════════════════════
        // Show / hide
        // ══════════════════════════════════════════════════════════════════════

        public static void ShowGUI()
        {
            // ── Main window ────────────────────────────────────────────────────
            mainHolder = Builder.CreateHolder(Builder.SceneToAttach.CurrentScene, "AutopilotMainHolder");
            mainWindow = UIToolsBuilder.CreateClosableWindow(
                mainHolder.transform, mainWindowID,
                300, 220,
                Settings.data.mainWindowPosition.x, Settings.data.mainWindowPosition.y,
                true, false, 0.95f, "Autopilot", false);
            mainWindow.RegisterPermanentSaving(mainWindowPosKey);
            mainWindow.CreateLayoutGroup(Type.Vertical, TextAnchor.MiddleCenter, 8f);

            // Ascent sub-window toggle — holds a label we can update later
            var ascentMainBtn = Builder.CreateButton(mainWindow, 280, 40, 0, 0, ToggleAscentWindow, "");
            ascentButtonLabel = GetButtonLabel(ascentMainBtn);

            // Landing sub-window toggle — hidden until dev mode
            landingButton = Builder.CreateButton(mainWindow, 280, 40, 0, 0, ToggleLandingWindow, "");
            landingButtonLabel = GetButtonLabel(landingButton);
            landingButton.gameObject.SetActive(devModeEnabled);

            // Developer console row
            consoleRow = Builder.CreateContainer(mainWindow);
            consoleRow.CreateLayoutGroup(Type.Horizontal, TextAnchor.MiddleLeft, 4f);
            Builder.CreateLabel(consoleRow, 20, 24, 0, 0, ">");
            var consoleInput = Builder.CreateTextInput(consoleRow, 250, 28, 0, 0, "");
            consoleInput.field.characterValidation = TMP_InputField.CharacterValidation.None;
            consoleInput.field.onEndEdit.AddListener(OnConsoleCommand);
            consoleRow.gameObject.SetActive(!devModeEnabled);

            // ── Ascent window ──────────────────────────────────────────────────
            ascentHolder = Builder.CreateHolder(Builder.SceneToAttach.CurrentScene, "AutopilotAscentHolder");
            ascentWindow = UIToolsBuilder.CreateClosableWindow(
                ascentHolder.transform, ascentWindowID,
                380, 270,
                Settings.data.ascentWindowPosition.x, Settings.data.ascentWindowPosition.y,
                true, false, 0.95f, "Ascent Autopilot", false);
            ascentWindow.RegisterPermanentSaving(ascentWindowPosKey);
            ascentWindow.CreateLayoutGroup(Type.Vertical, TextAnchor.MiddleCenter, 5f);
            ascentWindow.Active = false;
            BuildAscentWindow();

            // ── Landing window ─────────────────────────────────────────────────
            landingHolder = Builder.CreateHolder(Builder.SceneToAttach.CurrentScene, "AutopilotLandingHolder");
            landingWindow = UIToolsBuilder.CreateClosableWindow(
                landingHolder.transform, landingWindowID,
                380, 160,
                Settings.data.landingWindowPosition.x, Settings.data.landingWindowPosition.y,
                true, false, 0.95f, "Landing Autopilot", false);
            landingWindow.RegisterPermanentSaving(landingWindowPosKey);
            landingWindow.CreateLayoutGroup(Type.Vertical, TextAnchor.MiddleCenter, 5f);
            landingWindow.Active = false;
            BuildLandingWindow();

            // Save positions on drag
            mainWindow.gameObject.GetComponent<DraggableWindowModule>().OnDropAction  += SaveMainWindowPosition;
            ascentWindow.gameObject.GetComponent<DraggableWindowModule>().OnDropAction += SaveAscentWindowPosition;
            landingWindow.gameObject.GetComponent<DraggableWindowModule>().OnDropAction += SaveLandingWindowPosition;

            Refresh();
        }

        public static void HideGUI()
        {
            if (mainHolder   != null) Object.Destroy(mainHolder);
            if (ascentHolder != null) Object.Destroy(ascentHolder);
            if (landingHolder != null) Object.Destroy(landingHolder);
        }

        // ══════════════════════════════════════════════════════════════════════
        // Refresh — called by AutopilotUpdater ~4× per second and on any toggle
        // ══════════════════════════════════════════════════════════════════════

        public static void Refresh()
        {
            var ap = AutopilotUpdater.Instance;
            if (ap == null) return;

            bool ascentActive  = ap.IsAscentActive;
            bool landingActive = ap.IsLandingActive;

            // Main window buttons — show a dot when active
            SetLabelText(ascentButtonLabel,  ascentActive  ? "Ascent  ●" : "Ascent");
            SetLabelText(landingButtonLabel, landingActive ? "Landing ●" : "Landing");

            // Ascent window action button and status
            SetLabelText(ascentActionLabel,  ascentActive  ? "Stop Ascent"   : "Start Ascent");
            SetLabelText(landingActionLabel, landingActive ? "Stop Landing"  : "Start Landing");

            // State descriptions
            SetLabelText(ascentStatusLabel,  ascentActive  ? ap.AscentStatus  : "—");
            SetLabelText(landingStatusLabel, landingActive ? ap.LandingStatus : "—");
        }

        // ══════════════════════════════════════════════════════════════════════
        // Build sub-windows
        // ══════════════════════════════════════════════════════════════════════

        private static void BuildAscentWindow()
        {
            // Target altitude row
            var inputRow = Builder.CreateContainer(ascentWindow);
            inputRow.CreateLayoutGroup(Type.Horizontal, TextAnchor.MiddleLeft, 5f);
            Builder.CreateLabel(inputRow, 150, 30, 0, 0, "Target orbit (km)");
            pendingTargetOrbitText = FormatTargetOrbitKm(Settings.data.targetOrbitAltitude);
            targetAltitudeInput = Builder.CreateTextInput(inputRow, 170, 40, 0, 0, pendingTargetOrbitText);
            targetAltitudeInput.field.characterValidation = TMP_InputField.CharacterValidation.Decimal;
            targetAltitudeInput.field.onValueChanged.AddListener(OnTargetAltitudeValueChanged);
            targetAltitudeInput.field.onEndEdit.AddListener(OnTargetAltitudeChanged);

            // Adjustment buttons
            var adjustRow = Builder.CreateContainer(ascentWindow);
            adjustRow.CreateLayoutGroup(Type.Horizontal, TextAnchor.MiddleCenter, 4f);
            foreach (float stepKm in targetAdjustButtonsKm)
            {
                float captured = stepKm;
                Builder.CreateButton(adjustRow, 68, 32, 0, 0, () => AdjustTargetOrbitKm(captured), $"+{captured:0}");
            }

            Builder.CreateButton(ascentWindow, 220, 34, 0, 0, ResetTargetOrbitKm, "Reset (Atmo + 5 km)");

            // Status label
            ascentStatusLabel = Builder.CreateLabel(ascentWindow, 360, 24, 0, 0, "—");

            // Start / Stop button — label gets swapped by Refresh()
            var ascentActionBtn = Builder.CreateButton(ascentWindow, 220, 40, 0, 0, ToggleAscentAutopilot, "");
            ascentActionLabel = GetButtonLabel(ascentActionBtn);
        }

        private static void BuildLandingWindow()
        {
            Builder.CreateLabel(landingWindow, 360, 28, 0, 0, "Deorbit, flip & suicide burn.");

            // Status label
            landingStatusLabel = Builder.CreateLabel(landingWindow, 360, 24, 0, 0, "—");

            // Start / Stop button
            var landingActionBtn = Builder.CreateButton(landingWindow, 220, 40, 0, 0, ToggleLandingAutopilot, "");
            landingActionLabel = GetButtonLabel(landingActionBtn);
        }

        // ══════════════════════════════════════════════════════════════════════
        // Sub-window toggles
        // ══════════════════════════════════════════════════════════════════════

        private static void ToggleAscentWindow()
        {
            ascentWindowVisible = !ascentWindowVisible;
            ascentWindow.Active = ascentWindowVisible;
        }

        private static void ToggleLandingWindow()
        {
            landingWindowVisible = !landingWindowVisible;
            landingWindow.Active = landingWindowVisible;
        }

        // ══════════════════════════════════════════════════════════════════════
        // Developer console
        // ══════════════════════════════════════════════════════════════════════

        private static void OnConsoleCommand(string input)
        {
            if (string.IsNullOrWhiteSpace(input)) return;
            if (input.Trim() == DEV_UNLOCK_COMMAND)
            {
                devModeEnabled = true;
                if (landingButton != null) landingButton.gameObject.SetActive(true);
                if (consoleRow    != null) consoleRow.gameObject.SetActive(false);
                MsgDrawer.main.Log("[DEV] Landing autopilot unlocked.");
                Refresh();
            }
            else
            {
                MsgDrawer.main.Log($"Unknown command: {input.Trim()}");
            }
        }

        // ══════════════════════════════════════════════════════════════════════
        // Ascent altitude input
        // ══════════════════════════════════════════════════════════════════════

        private static void OnTargetAltitudeValueChanged(string value)
        {
            pendingTargetOrbitText = value;
            if (TryParseTargetOrbitKm(value, out float km))
            {
                Settings.data.targetOrbitAltitude = km * 1000f;
                Settings.Save();
            }
        }

        private static void OnTargetAltitudeChanged(string value)
        {
            if (!TryCommitTargetAltitude(false))
                targetAltitudeInput.Text = FormatTargetOrbitKm(Settings.data.targetOrbitAltitude);
        }

        private static void ToggleAscentAutopilot()
        {
            if (!TryCommitTargetAltitude(true)) return;
            AutopilotUpdater.Instance.ToggleAscent();
        }

        private static void ToggleLandingAutopilot()
        {
            AutopilotUpdater.Instance.ToggleLanding();
        }

        private static bool TryCommitTargetAltitude(bool showErrors)
        {
            if (targetAltitudeInput == null) return true;

            string raw = !string.IsNullOrWhiteSpace(pendingTargetOrbitText)
                ? pendingTargetOrbitText
                : (targetAltitudeInput.field != null ? targetAltitudeInput.field.text : targetAltitudeInput.Text);

            if (!TryParseTargetOrbitKm(raw, out float km))
            {
                if (showErrors) MsgDrawer.main.Log("Enter a valid target orbit altitude.");
                return false;
            }

            Settings.data.targetOrbitAltitude = km * 1000f;
            Settings.Save();
            pendingTargetOrbitText   = FormatTargetOrbitKm(Settings.data.targetOrbitAltitude);
            targetAltitudeInput.Text = pendingTargetOrbitText;
            return true;
        }

        private static void SetTargetOrbitKm(float km)
        {
            Settings.data.targetOrbitAltitude = Mathf.Max(minTargetOrbitKm, km) * 1000f;
            Settings.Save();
            pendingTargetOrbitText = FormatTargetOrbitKm(Settings.data.targetOrbitAltitude);
            if (targetAltitudeInput != null) targetAltitudeInput.Text = pendingTargetOrbitText;
        }

        private static void AdjustTargetOrbitKm(float deltaKm)
        {
            float current = Settings.data.targetOrbitAltitude / 1000f;
            if (TryParseTargetOrbitKm(pendingTargetOrbitText, out float pending)) current = pending;
            SetTargetOrbitKm(current + deltaKm);
        }

        private static void ResetTargetOrbitKm()
        {
            float recommended = fallbackRecommendedOrbitMeters;
            if (AutopilotUpdater.Instance != null)
                recommended = AutopilotUpdater.Instance.GetRecommendedLowOrbitAltitudeMeters();
            SetTargetOrbitKm(recommended / 1000f);
        }

        // ══════════════════════════════════════════════════════════════════════
        // Utilities
        // ══════════════════════════════════════════════════════════════════════

        /// <summary>
        /// Extracts the TextMeshProUGUI component from a ModGUI Button so we can
        /// update its text without recreating the button.
        /// </summary>
        private static TextMeshProUGUI GetButtonLabel(SFS.UI.ModGUI.Button btn)
        {
            if (btn == null) return null;
            return btn.gameObject.GetComponentInChildren<TextMeshProUGUI>();
        }

        /// <summary>Safely sets a TMP label's text. Does nothing if the label is null.</summary>
        private static void SetLabelText(TextMeshProUGUI label, string text)
        {
            if (label == null) return;
            label.text = text;  // TMP uses lowercase .text
        }

        /// <summary>Safely sets a ModGUI Label's text. Does nothing if the label is null.</summary>
        private static void SetLabelText(Label label, string text)
        {
            if (label == null) return;
            label.Text = text;
        }

        private static bool TryParseTargetOrbitKm(string value, out float km)
        {
            km = 0f;
            if (string.IsNullOrWhiteSpace(value)) return false;
            var styles = NumberStyles.Float | NumberStyles.AllowThousands;
            if (!float.TryParse(value, styles, CultureInfo.CurrentCulture,   out km) &&
                !float.TryParse(value, styles, CultureInfo.InvariantCulture, out km) &&
                !float.TryParse(value.Replace(',', '.'), styles, CultureInfo.InvariantCulture, out km))
                return false;
            km = Mathf.Max(minTargetOrbitKm, km);
            return true;
        }

        private static string FormatTargetOrbitKm(float altitudeMeters)
            => (altitudeMeters / 1000f).ToString("0.0", CultureInfo.InvariantCulture);

        private static void SaveMainWindowPosition()
        {
            Settings.data.mainWindowPosition = Vector2Int.RoundToInt(mainWindow.Position);
            Settings.Save();
        }

        private static void SaveAscentWindowPosition()
        {
            Settings.data.ascentWindowPosition = Vector2Int.RoundToInt(ascentWindow.Position);
            Settings.Save();
        }

        private static void SaveLandingWindowPosition()
        {
            Settings.data.landingWindowPosition = Vector2Int.RoundToInt(landingWindow.Position);
            Settings.Save();
        }
    }
}