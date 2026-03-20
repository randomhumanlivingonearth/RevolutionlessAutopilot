using System.Collections.Generic;
using HarmonyLib;
using ModLoader;
using ModLoader.Helpers;
using SFS.IO;
using UITools;
using UnityEngine;

namespace RevolutionlessAutopilot
{
    public class Main : Mod
    {
        public override string ModNameID => "RevolutionlessAutopilot";
        public override string DisplayName => "Revolutionless Autopilot";
        public override string Author => "randomhumanlivingonearth";
        public override string MinimumGameVersionNecessary => "1.5.10.2";
        public override string ModVersion => "v0.1.0";
        public override string Description => "Adds an autopilot that will execute some basic maneuvers fully autonomously.";

        public override Dictionary<string, string> Dependencies { get; } = new Dictionary<string, string> { { "UITools", "1.0" } };

        public static FolderPath modFolder;
        public static AutopilotUpdater updater;

        private static Harmony patcher;

        public override void Early_Load()
        {
            modFolder = new FolderPath(ModFolder);
            patcher = new Harmony(ModNameID);
            patcher.PatchAll();
        }

        public override void Load()
        {
            Settings.Load();

            GameObject go = new GameObject("RevolutionlessAutopilot_Updater");
            Object.DontDestroyOnLoad(go);
            updater = go.AddComponent<AutopilotUpdater>();

            SceneHelper.OnWorldSceneLoaded += GUI.ShowGUI;
            SceneHelper.OnWorldSceneUnloaded += GUI.HideGUI;
        }
    }
}