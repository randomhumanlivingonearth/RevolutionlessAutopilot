using System;
using SFS.World;
using SFS.WorldBase;
using UnityEngine;

namespace RevolutionlessAutopilot
{
    /// <summary>
    /// Provides orbital data queries for the current rocket and draws a
    /// target orbit ring on the map view using a LineRenderer.
    ///
    /// Usage:
    ///   var om = new OrbitManager(rocket);
    ///   om.CreateTargetOrbitLine(mapParentTransform);    // once at startup
    ///   om.UpdateTargetOrbitLine(targetAltitude, scale); // each map-view frame
    ///   om.DestroyTargetOrbitLine();                     // on teardown
    /// </summary>
    public class OrbitManager
    {
        private Rocket rocket;
        private LineRenderer orbitLine;

        // (RU) Количество отрезков, использованных для аппроксимации окружности.
        //      128 обеспечивает достаточно плавную картинку при любом уровне увеличения, оставаясь при этом недорогим.
        // (EN) Number of line segments used to approximate the circle.
        //      128 is smooth enough at any zoom level while staying cheap.
        private const int ORBIT_DRAW_RESOLUTION = 128;

        public bool Debug = true;

        // (RU) Строительство | (EN) Construction
        public OrbitManager(Rocket rocket)
        {
            this.rocket = rocket;
        }

        public void SetRocket(Rocket rocket)
        {
            this.rocket = rocket;
        }

        // (RU) Данные о планете | (EN) Planet data
        /// <summary>The planet the rocket is currently closest to.</summary>
        public Planet GetCurrentPlanet()
        {
            return rocket?.location?.planet?.Value;
        }

        /// <summary>Atmosphere height in metres. 0 if the planet has no atmosphere.</summary>
        public double GetAtmosphereHeight()
        {
            return GetCurrentPlanet()?.AtmosphereHeightPhysics ?? 0;
        }

        /// <summary>True if the planet has a meaningful atmosphere (>100 m).</summary>
        public bool HasAtmosphere()
        {
            return GetAtmosphereHeight() > 100.0;
        }

        // (RU) Текущие данные об орбите | (EN) Current orbit data
        /// <summary>Returns the current orbit from the trajectory, or null if unavailable.</summary>
        public Orbit GetCurrentOrbit()
        {
            if (rocket?.mapPlayer?.Trajectory == null) return null;
            var paths = rocket.mapPlayer.Trajectory.paths;
            if (paths.Count == 0) return null;
            return paths[0] as Orbit;
        }

        /// <summary>Apoapsis radius from planet center (m). Returns +Infinity for escape trajectories.</summary>
        public double GetApoapsis()
        {
            var orbit = GetCurrentOrbit();
            if (orbit == null) return 0;
            double ap = orbit.apoapsis;
            return (double.IsInfinity(ap) || ap > 1e9) ? double.PositiveInfinity : ap;
        }

        /// <summary>Periapsis radius from planet center (m).</summary>
        public double GetPeriapsis()
        {
            return GetCurrentOrbit()?.periapsis ?? 0;
        }

        /// <summary>Apoapsis altitude above the surface (m). 0 if orbit is unavailable.</summary>
        public double GetApoapsisAltitude()
        {
            var planet = GetCurrentPlanet();
            if (planet == null) return 0;
            double apo = GetApoapsis();
            if (double.IsPositiveInfinity(apo)) return double.PositiveInfinity;
            return Math.Max(0, apo - planet.Radius);
        }

        /// <summary>Periapsis altitude above the surface (m).</summary>
        public double GetPeriapsisAltitude()
        {
            var planet = GetCurrentPlanet();
            if (planet == null) return 0;
            return Math.Max(0, GetPeriapsis() - planet.Radius);
        }

        /// <summary>Seconds until the rocket reaches apoapsis. +Infinity if orbit unavailable.</summary>
        public double GetTimeToApoapsis()
        {
            var orbit = GetCurrentOrbit();
            if (orbit == null) return double.PositiveInfinity;
            double now = WorldTime.main.worldTime;
            return orbit.GetNextTrueAnomalyPassTime(now, Math.PI) - now;
        }

        /// <summary>Seconds until the rocket reaches periapsis. +Infinity if orbit unavailable.</summary>
        public double GetTimeToPeriapsis()
        {
            var orbit = GetCurrentOrbit();
            if (orbit == null) return double.PositiveInfinity;
            double now = WorldTime.main.worldTime;
            return orbit.GetNextTrueAnomalyPassTime(now, 0.0) - now;
        }

        // (RU) Расчеты орбитальной скорости | (EN) Orbital speed calculations
        /// <summary>
        /// Circular orbit speed at a given radius from the planet center (m/s).
        /// v = sqrt(μ / r)
        /// </summary>
        public double GetCircularOrbitSpeed(double radius)
        {
            var planet = GetCurrentPlanet();
            if (planet == null) return 0;
            double r = Math.Max(radius, planet.Radius + 1.0);
            return Math.Sqrt(planet.mass / r);
        }

        /// <summary>
        /// Speed needed to be on a Hohmann transfer that has its apoapsis at targetRadius,
        /// evaluated at the current orbital radius. Used for burn planning.
        /// v = sqrt(μ * (2/r - 1/a))
        /// </summary>
        public double GetTransferSpeed(double currentRadius, double targetRadius)
        {
            var planet = GetCurrentPlanet();
            if (planet == null) return 0;
            double r  = Math.Max(currentRadius, planet.Radius + 1.0);
            double a  = Math.Max(targetRadius,  planet.Radius + 1.0);
            double vv = 2.0 / r - 1.0 / a;
            return vv > 0 ? Math.Sqrt(planet.mass * vv) : GetCircularOrbitSpeed(a);
        }

        /// <summary>
        /// Delta-V required to circularize at the actual apoapsis given the current
        /// horizontal speed. This is the correct formula for a circularization burn.
        /// </summary>
        public double GetCircularizationDeltaV(double horizontalSpeed)
        {
            double apo = GetApoapsis();
            if (apo <= 0 || double.IsInfinity(apo)) return 0;
            return Math.Max(0, GetCircularOrbitSpeed(apo) - horizontalSpeed);
        }

        /// <summary>
        /// Estimates the burn duration in seconds to achieve a given delta-V
        /// at a given acceleration (m/s²). Returns 0 if acceleration is negligible.
        /// </summary>
        public double EstimateBurnDuration(double deltaV, double acceleration)
        {
            if (deltaV <= 0 || acceleration <= 0.00001) return 0;
            return deltaV / acceleration;
        }

        // (RU) Визуализация целевой орбиты | (EN) Target orbit visualisation
        /// <summary>
        /// Creates a LineRenderer GameObject as a child of parent and styles it as
        /// a target orbit ring. Call once when the autopilot UI appears.
        ///
        /// The parent Transform should be positioned at the planet's map-view origin
        /// so the circle is centred correctly. Typical SFS map code positions planet
        /// bodies at their world position scaled by the map camera's zoom factor —
        /// attach this to the same parent that planet markers use.
        /// </summary>
        public void CreateTargetOrbitLine(Transform parent)
        {
            if (orbitLine != null) return; // already created

            var go = new GameObject("TargetOrbitLine");
            go.transform.SetParent(parent, false);
            go.transform.localPosition = Vector3.zero;

            orbitLine = go.AddComponent<LineRenderer>();
            orbitLine.loop           = true;
            orbitLine.positionCount  = ORBIT_DRAW_RESOLUTION;
            orbitLine.useWorldSpace  = false;   // points are relative to parent (planet centre)
            orbitLine.startWidth     = 0.003f;
            orbitLine.endWidth       = 0.003f;
            orbitLine.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            orbitLine.receiveShadows = false;

            // (EN) Simple unlit colour — green tint to distinguish from existing orbit lines
            orbitLine.material             = new Material(Shader.Find("Sprites/Default"));
            orbitLine.startColor           = new Color(0.2f, 1.0f, 0.4f, 0.75f);
            orbitLine.endColor             = new Color(0.2f, 1.0f, 0.4f, 0.75f);
        }

        /// <summary>
        /// Redraws the circle to match targetAltitude at the current map zoom.
        /// Call every Update() while the map view is open.
        ///
        /// scaleMultiplier: the factor used to convert world-space metres to map
        /// screen units. Obtain this from the same source SFS uses when drawing
        /// its own orbit paths (typically MapCamera.main.scale or similar).
        /// </summary>
        public void UpdateTargetOrbitLine(double targetAltitude, double scaleMultiplier)
        {
            if (orbitLine == null) return;
            var planet = GetCurrentPlanet();
            if (planet == null) return;

            double r = (planet.Radius + targetAltitude) * scaleMultiplier;
            Vector3[] pts = new Vector3[ORBIT_DRAW_RESOLUTION];
            double step = 2.0 * Math.PI / ORBIT_DRAW_RESOLUTION;
            for (int i = 0; i < ORBIT_DRAW_RESOLUTION; i++)
            {
                double a = i * step;
                pts[i] = new Vector3((float)(r * Math.Cos(a)), (float)(r * Math.Sin(a)), 0f);
            }
            orbitLine.SetPositions(pts);
        }

        /// <summary>Shows or hides the orbit line without destroying it.</summary>
        public void SetOrbitLineVisible(bool visible)
        {
            if (orbitLine != null)
                orbitLine.enabled = visible;
        }

        /// <summary>Destroys the orbit line GameObject. Call on autopilot stop or UI teardown.</summary>
        public void DestroyTargetOrbitLine()
        {
            if (orbitLine == null) return;
            UnityEngine.Object.Destroy(orbitLine.gameObject);
            orbitLine = null;
        }
    }
}