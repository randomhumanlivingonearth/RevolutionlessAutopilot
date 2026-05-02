// Settings.cs - (RU) упрощено, только целевая орбита | (EN) simplified, target orbit only
using System;
using SFS.IO;
using SFS.Parsers.Json;
using UnityEngine;

namespace RevolutionlessAutopilot
{
    [Serializable]
    public class SettingsData
    {
        public float targetOrbitAltitude = 100000f; // (RU) метры | (EN) meters
        public Vector2Int mainWindowPosition = new Vector2Int(-400, -300);
        public Vector2Int ascentWindowPosition = new Vector2Int(-400, 100);
        public Vector2Int landingWindowPosition = new Vector2Int(-400, -50);
    }

    public static class Settings
    {
        private static readonly FilePath configPath = Main.modFolder.ExtendToFile("Config.txt");
        public static SettingsData data;

        public static void Load()
        {
            data = null;
            if (configPath.FileExists())
            {
                try
                {
                    data = JsonWrapper.FromJson<SettingsData>(configPath.ReadText());
                    if (data == null)
                    {
                        Debug.Log("RevolutionlessAutopilot: Config file invalid, using defaults.");
                    }
                }
                catch (Exception exception)
                {
                    Debug.LogException(exception);
                    Debug.Log("RevolutionlessAutopilot: Config file invalid, using defaults.");
                }
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
