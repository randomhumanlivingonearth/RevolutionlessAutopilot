# RevolutionlessAutopilot

An autopilot mod for Spaceflight Simulator (PC v1.5.10.2+) that autonomously executes flight maneuvers. Claimed to be the first autopilot mod for SFS.

## Requirements
- [UITools]([https://jmnet.one/sfs/forum/index.php?forums/mods.146/](https://jmnet.one/sfs/forum/index.php?threads/ui-tools.10596/)) — install to your Mods folder before using this mod

## Completed Autopilots

### Ascent Autopilot
Launches from the surface and inserts into a circular orbit at a user-selected altitude.
- Gravity turn with dynamic throttle control
- Automatic staging when fuel is depleted
- Time warp during coast phase
- Two-burn Hohmann transfer for high orbits (parks first, then transfers)

### Landing Autopilot
Returns from orbit and lands on the surface.
- Deorbit burn to lower periapsis
- Retrograde hold during atmospheric reentry
- Automatic flip maneuver
- Suicide burn with soft landing mode below 200m

## Planned Autopilots
- Rendezvous Autopilot
- Docking Autopilot

## Installation
1. Install UITools to your Mods folder
2. Copy `RevolutionlessAutopilot.dll` to your Mods folder
3. Launch the game — the Autopilot window will appear in the world scene

## Usage
Open the **Autopilot** window in-game and select the maneuver you want to execute.
For ascent, set your target orbit altitude in km before starting.

## ⚠️ Warning
Do not manually timewarp while the autopilot is active near a phase transition (e.g. just before circularization or suicide burn). The autopilot manages time warp automatically.

## Bugs & Suggestions
Report issues or suggest features on our Discord: https://discord.gg/m3tgpC3t
