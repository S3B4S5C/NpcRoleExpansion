# NpcRoleExpansion

NpcRoleExpansion is a lightweight library mod for Hytale that adds extra NPCRole JSON building blocks, mainly BodyMotions and MotionControllers, to make custom NPC behaviors easier to author through JSON.

The goal of this project is to provide reusable, well-documented behavior pieces that integrate naturally with the vanilla NPCRole system through `Type` ids, so role authors can build more advanced behaviors with less JSON boilerplate and less custom glue code.

This library is meant to grow over time as new useful NPC behavior patterns come up.  
If there is a vanilla behavior you feel is missing, or a JSON-friendly building block you would like to see added, feel free to suggest it.

## Included Features

### MotionController: `FlyFreeLook`

A flight motion controller that extends vanilla `Fly` and allows movement independently from look direction.

**Compatibility**
- Extends `MotionControllerFly`
- Uses the same JSON fields as vanilla `fly`

**Use cases**
- Strafing or orbiting while still watching a target
- Drones or flying enemies that should track a target without constantly turning into their movement direction

**Type id**
- `FlyFreeLook`

---

### BodyMotion: `MaintainDistanceFlyRing`

A flying-oriented body motion inspired by `MaintainDistance`, designed for NPCs using a fly-compatible motion controller.

**What it does**
- Keeps an NPC within a configurable XZ distance band around a target
- Supports configurable vertical offsets that can change over time
- Can optionally bias movement behind the target’s movement direction

**Why it is useful**
- Provides a maintain-distance style solution for flying NPCs
- Includes pathfinding through the `BodyMotionFindBase` flow, allowing pursuit while handling obstacles
- Reduces JSON complexity by avoiding the need to chain several separate motions just to handle chasing, spacing, and obstacle correction

**Requirements**
- Requires `Feature.AnyPosition` so an upstream sensor provides a position each tick
- Validates that the active motion controller is compatible with `MotionControllerFly`

**Type id**
- `MaintainDistanceFlyRing`

## Example

Below is a compact example showing a typical setup:

- `FlyFreeLook` as the motion controller
- `Watch` as the head motion
- `MaintainDistanceFlyRing` as the body motion

> This example assumes that a target sensor, or another upstream provider, is supplying a target position.

```json
{
  "MotionControllerList": [
    {
      "Type": "FlyFreeLook",
      "MaxHorizontalSpeed": 20,
      "MaxSinkSpeed": 10,
      "MaxClimbSpeed": 12,
      "MinAirSpeed": 0,
      "Acceleration": 7,
      "Deceleration": 15,
      "MaxRollAngle": 180,
      "MaxTurnSpeed": 360,
      "AutoLevel": true,
      "MinHeightOverGround": 0,
      "MaxHeightOverGround": 64,
      "DesiredAltitudeWeight": 0.35
    }
  ],
  "HeadMotion": {
    "Type": "Watch"
  },
  "BodyMotion": {
    "Type": "MaintainDistanceFlyRing",
    "DesiredDistanceRangeXZ": [2.5, 2.5],
    "YOffsets": [1.0, 3.0],
    "YOffsetChangeIntervalRange": [2.0, 4.0],
    "MoveThreshold": 0.35,
    "LockBehindTarget": false
  }
}
````

## Tuning Tips

* Want less twitchy orbit corrections? Increase `MoveThreshold`
* Want smoother braking near the ring point? Increase `MoveTowardsSlowdownThreshold`
* Want the NPC to prefer staying behind a moving target? Use:

  * `LockBehindTarget: true`
  * `BehindBlend` around `0.1` to `0.35`
  * `MinTargetSpeed` to prevent unstable flips when the target is barely moving

## `MaintainDistanceFlyRing` Quick Reference

### Required

* `DesiredDistanceRangeXZ: [min, max]`
* `YOffsets: [...]`
  If empty, it defaults to `[3.0]`

### Common tuning

* `TargetDistanceFactor` (default `0.5`)
* `MoveThreshold` (default `0.7`)
* `RelativeForwardsSpeed` (default `1.0`)
* `RelativeBackwardsSpeed` (default `0.6`)
* `MoveTowardsSlowdownThreshold` (default `12.0`)
* `HardStop` (default `false`)

### Behind-target anchoring

* `LockBehindTarget` (default `true`)
* `BehindBlend` (default `0.25`)
* `MinTargetSpeed` (default `0.15`)

## Feedback / Suggestions

NpcRoleExpansion is intended to become a toolbox of reusable NPCRole behaviors.

If there is a behavior you frequently rebuild in JSON, or something you feel vanilla NPCRoles are missing, feel free to suggest it.
