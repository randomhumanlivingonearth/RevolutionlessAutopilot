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
        // (RU) Главное окно | (EN) Main window
        private static GameObject mainHolder;
        private static ClosableWindow mainWindow;
        private static readonly int mainWindowID = Builder.GetRandomID();
        private const string mainWindowTitle = "Autopilot";
        private const string mainWindowPosKey = "RevolutionlessAutopilot.MainWindow";

        // (RU) Окно подъёма | (EN) Ascent window
        private static GameObject ascentHolder;
        private static ClosableWindow ascentWindow;
        private static readonly int ascentWindowID = Builder.GetRandomID();
        private const string ascentWindowTitle = "Ascent Autopilot";
        private const string ascentWindowPosKey = "RevolutionlessAutopilot.AscentWindow";

        // (RU) Окно посадки | (EN) Landing window
        private static GameObject landingHolder;
        private static ClosableWindow landingWindow;
        private static readonly int landingWindowID = Builder.GetRandomID();
        private const string landingWindowTitle = "Landing Autopilot";
        private const string landingWindowPosKey = "RevolutionlessAutopilot.LandingWindow";

        // (RU) Состояние UI | (EN) UI state
        private const float minTargetOrbitKm = 1f;
        private const float fallbackRecommendedOrbitMeters = 40000f;
        private static readonly float[] targetAdjustButtonsKm = { 1f, 10f, 100f, 1000f, 10000f };

        private static bool ascentWindowVisible;
        private static bool landingWindowVisible;

        private static TextInput targetAltitudeInput;
        private static string pendingTargetOrbitText;

        // (RU) Показать / скрыть всё GUI | (EN) Show / hide all GUI
        public static void ShowGUI()
        {
            // (RU) Главное окно | (EN) Main window
            mainHolder = Builder.CreateHolder(Builder.SceneToAttach.CurrentScene, "AutopilotMainHolder");
            mainWindow = UIToolsBuilder.CreateClosableWindow(
                mainHolder.transform,
                mainWindowID,
                300, 240,
                Settings.data.mainWindowPosition.x,
                Settings.data.mainWindowPosition.y,
                true, false, 0.95f, mainWindowTitle, false
            );
            mainWindow.RegisterPermanentSaving(mainWindowPosKey);
            mainWindow.CreateLayoutGroup(Type.Vertical, TextAnchor.MiddleCenter, 10f);

            Builder.CreateButton(mainWindow, 280, 40, 0, 0, ToggleAscentWindow, "Ascent");
            // Builder.CreateButton(mainWindow, 280, 40, 0, 0, ToggleLandingWindow, "Landing");

            // (RU) Окно подъёма | (EN) Ascent window
            ascentHolder = Builder.CreateHolder(Builder.SceneToAttach.CurrentScene, "AutopilotAscentHolder");
            ascentWindow = UIToolsBuilder.CreateClosableWindow(
                ascentHolder.transform,
                ascentWindowID,
                380, 230,
                Settings.data.ascentWindowPosition.x,
                Settings.data.ascentWindowPosition.y,
                true, false, 0.95f, ascentWindowTitle, false
            );
            ascentWindow.RegisterPermanentSaving(ascentWindowPosKey);
            ascentWindow.CreateLayoutGroup(Type.Vertical, TextAnchor.MiddleCenter, 5f);
            ascentWindow.Active = false;

            BuildAscentWindow();

            // (RU) Окно посадки | (EN) Landing window
            landingHolder = Builder.CreateHolder(Builder.SceneToAttach.CurrentScene, "AutopilotLandingHolder");
            landingWindow = UIToolsBuilder.CreateClosableWindow(
                landingHolder.transform,
                landingWindowID,
                380, 140,
                Settings.data.landingWindowPosition.x,
                Settings.data.landingWindowPosition.y,
                true, false, 0.95f, landingWindowTitle, false
            );
            landingWindow.RegisterPermanentSaving(landingWindowPosKey);
            landingWindow.CreateLayoutGroup(Type.Vertical, TextAnchor.MiddleCenter, 5f);
            landingWindow.Active = false;

            // (RU) Временно скрываем окно посадки до тех пор, пока эта функция не будет реализована должным образом. | (EN) Temporarily hiding landing window until it's implemented properly
            // BuildLandingWindow();

            // // (RU) Сохраняем позиции при перетаскивании | (EN) Save positions on drag
            // mainWindow.gameObject.GetComponent<DraggableWindowModule>().OnDropAction += SaveMainWindowPosition;
            // ascentWindow.gameObject.GetComponent<DraggableWindowModule>().OnDropAction += SaveAscentWindowPosition;
            // landingWindow.gameObject.GetComponent<DraggableWindowModule>().OnDropAction += SaveLandingWindowPosition;
        }

        public static void HideGUI()
        {
            if (mainHolder != null)
                Object.Destroy(mainHolder);
            if (ascentHolder != null)
                Object.Destroy(ascentHolder);
            if (landingHolder != null)
                Object.Destroy(landingHolder);
        }

        // (RU) Переключение подокон | (EN) Sub-window toggles
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

        // (RU) Построение окна подъёма | (EN) Build ascent window
        private static void BuildAscentWindow()
        {
            var inputRow = Builder.CreateContainer(ascentWindow);
            inputRow.CreateLayoutGroup(Type.Horizontal, TextAnchor.MiddleLeft, 5f);
            Builder.CreateLabel(inputRow, 150, 30, 0, 0, "Target Orbit Alt (km)");
            pendingTargetOrbitText = FormatTargetOrbitKm(Settings.data.targetOrbitAltitude);
            targetAltitudeInput = Builder.CreateTextInput(inputRow, 170, 40, 0, 0, pendingTargetOrbitText);
            targetAltitudeInput.field.characterValidation = TMP_InputField.CharacterValidation.Decimal;
            targetAltitudeInput.field.onValueChanged.AddListener(OnTargetAltitudeValueChanged);
            targetAltitudeInput.field.onEndEdit.AddListener(OnTargetAltitudeChanged);

            var adjustRow = Builder.CreateContainer(ascentWindow);
            adjustRow.CreateLayoutGroup(Type.Horizontal, TextAnchor.MiddleCenter, 4f);
            foreach (float stepKm in targetAdjustButtonsKm)
            {
                float capturedStepKm = stepKm;
                Builder.CreateButton(adjustRow, 68, 32, 0, 0, () => AdjustTargetOrbitKm(capturedStepKm), $"+{capturedStepKm:0}");
            }

            Builder.CreateButton(ascentWindow, 220, 34, 0, 0, ResetTargetOrbitKm, "Reset (Atmo + 5 km)");
            Builder.CreateButton(ascentWindow, 220, 40, 0, 0, ToggleAscentAutopilot, "Start Ascent");
        }

        // (RU) Построение окна посадки | (EN) Build landing window
        private static void BuildLandingWindow()
        {
            Builder.CreateLabel(landingWindow, 360, 30, 0, 0, "Performs deorbit, flip & suicide burn.");
            Builder.CreateButton(landingWindow, 220, 40, 0, 0, ToggleLandingAutopilot, "Start Landing");
        }

        // (RU) Логика подъёма | (EN) Ascent logic
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
            if (!TryCommitTargetAltitude(true))
                return;

            AutopilotUpdater.Instance.ToggleAscent();
        }

        private static bool TryCommitTargetAltitude(bool showErrors)
        {
            if (targetAltitudeInput == null)
                return true;

            string rawValue = !string.IsNullOrWhiteSpace(pendingTargetOrbitText)
                ? pendingTargetOrbitText
                : (targetAltitudeInput.field != null ? targetAltitudeInput.field.text : targetAltitudeInput.Text);

            if (!TryParseTargetOrbitKm(rawValue, out float km))
            {
                if (showErrors)
                    MsgDrawer.main.Log("Enter a valid target orbit altitude.");
                return false;
            }

            Settings.data.targetOrbitAltitude = km * 1000f;
            Settings.Save();
            pendingTargetOrbitText = FormatTargetOrbitKm(Settings.data.targetOrbitAltitude);
            targetAltitudeInput.Text = pendingTargetOrbitText;
            return true;
        }

        private static void SetTargetOrbitKm(float km)
        {
            Settings.data.targetOrbitAltitude = Mathf.Max(minTargetOrbitKm, km) * 1000f;
            Settings.Save();
            pendingTargetOrbitText = FormatTargetOrbitKm(Settings.data.targetOrbitAltitude);
            if (targetAltitudeInput != null)
                targetAltitudeInput.Text = pendingTargetOrbitText;
        }

        private static void AdjustTargetOrbitKm(float deltaKm)
        {
            float currentKm = Settings.data.targetOrbitAltitude / 1000f;
            if (TryParseTargetOrbitKm(pendingTargetOrbitText, out float pendingKm))
                currentKm = pendingKm;

            SetTargetOrbitKm(currentKm + deltaKm);
        }

        private static void ResetTargetOrbitKm()
        {
            float recommendedMeters = fallbackRecommendedOrbitMeters;
            if (AutopilotUpdater.Instance != null)
                recommendedMeters = AutopilotUpdater.Instance.GetRecommendedLowOrbitAltitudeMeters();

            SetTargetOrbitKm(recommendedMeters / 1000f);
        }

        // (RU) Логика посадки | (EN) Landing logic
        private static void ToggleLandingAutopilot()
        {
            AutopilotUpdater.Instance.ToggleLanding();
        }

        // (RU) Вспомогательные методы парсинга | (EN) Parse helper methods
        private static bool TryParseTargetOrbitKm(string value, out float km)
        {
            km = 0f;
            if (string.IsNullOrWhiteSpace(value))
                return false;

            var styles = NumberStyles.Float | NumberStyles.AllowThousands;
            if (!float.TryParse(value, styles, CultureInfo.CurrentCulture, out km) &&
                !float.TryParse(value, styles, CultureInfo.InvariantCulture, out km) &&
                !float.TryParse(value.Replace(',', '.'), styles, CultureInfo.InvariantCulture, out km))
            {
                return false;
            }

            km = Mathf.Max(minTargetOrbitKm, km);
            return true;
        }

        private static string FormatTargetOrbitKm(float altitudeMeters)
        {
            return (altitudeMeters / 1000f).ToString("0.0", CultureInfo.InvariantCulture);
        }

        // (RU) Сохранение позиций окон | (EN) Save window positions
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