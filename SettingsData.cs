// Settings.cs - упрощено, только целевая орбита
using System;
using SFS.IO;
using SFS.Parsers.Json;
using UnityEngine;

namespace RevolutionlessAutopilot
{
    [Serializable]
    public class SettingsData
    {
        public float targetOrbitAltitude = 100000f; // meters
        public Vector2Int mainWindowPosition = new Vector2Int(-400, -300);
        public Vector2Int ascentWindowPosition = new Vector2Int(-400, 100);
    }

    public static class Settings
    {
        private static readonly FilePath configPath = Main.modFolder.ExtendToFile("Config.txt");
        public static SettingsData data;

        public static void Load()
        {
            if (!JsonWrapper.TryLoadJson(configPath, out data) && configPath.FileExists())
            {
                Debug.Log("RevolutionlessAutopilot: Config file invalid, using defaults.");
            }
            data = data ?? new SettingsData();
            Save();
        }

        public static void Save()
        {
            configPath.WriteText(JsonWrapper.ToJson(data, true));
        }
    }
}