# Multi-Camera Vision Pose Estimation

How the robot uses three cameras to know where it is on the field, in auton and teleop.

## The big idea

Odometry (wheel encoders + gyro) tells us where the robot is, but it drifts: wheels slip,
robots get bumped. AprilTags on the field let us *correct* that drift.

We do **not** try to combine the three cameras ourselves. Each camera independently says
"I think the robot is here, at this time" and hands that guess to the drivetrain's pose
estimator (built into CTRE's Phoenix 6 `SwerveDrivetrain`, which our `Swerve` subsystem
extends). The pose estimator is a Kalman filter: it is built to blend many noisy measurements,
arriving at different times, with how much it trusts each one. That is exactly the
"fusion" job, and it is already written and tested.

```
 Camera A ─┐                                         ┌───────────────────────────┐
 Camera B ─┼─► PhotonVision (Mac Mini) ── NT ──►     │ SystemCore robot code     │
 Camera C ─┘   one MultiTag pipeline per camera      │                           │
                                                     │ PhotonCamera A ─► estimate├─┐
                                                     │ PhotonCamera B ─► estimate├─┼─► addVisionMeasurement()
                                                     │ PhotonCamera C ─► estimate├─┘          │
                                                     │                           │            ▼
              wheels + gyro (Phoenix odometry thread) ──────────────────────────► Swerve pose estimator
                                                     └───────────────────────────┘            │
                                                                                              ▼
                                                                                  robot pose (auton paths,
                                                                                  auto-aim, dashboard)
```

### Why not merge the three cameras into one answer first?

You *could* gather tag corners from all three cameras and solve for one pose. In
practice:

- The cameras don't take pictures at exactly the same instant, and the robot is moving.
- Any error in one camera's mounting position corrupts the merged answer, and you can't
  tell which camera was wrong.
- The pose estimator already gives a near-identical result by blending the three
  measurements statistically.

Independent cameras are simpler, and when one camera is badly calibrated or blocked, you
can see it and fix it (see [Debugging](#debugging-with-advantagescope)).

## What each piece does

### 1. PhotonVision on the Mac Mini

- One [AprilTag pipeline](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/about-apriltags.html)
  per camera, with [3D mode](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html)
  on and [MultiTag](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html)
  enabled (Output tab → "Do Multi-Target Estimation").
- Leave "Always Do Single-Target Estimation" **off**. PhotonVision still solves any tag
  that MultiTag didn't use (so a lone tag always gets its own solve). It only skips tags
  already covered by a MultiTag result.
- When a camera sees 2+ tags, PhotonVision solves one pose using *all* the tag corners in
  that image (much more accurate than one tag). This is the "MultiTag" result.
- When it sees only 1 tag, we fall back to a single-tag pose (see
  [section 4](#4-robot-code-photonlib)).
- PhotonVision publishes results to the **SystemCore's NetworkTables server**. Set the
  team number / NT server address in PhotonVision's
  [network settings](https://docs.photonvision.org/en/latest/docs/quick-start/networking.html)
  to point at the SystemCore.
- PhotonVision timestamps every frame and
  [syncs its clock with the robot](https://docs.photonvision.org/en/latest/docs/contributing/design-descriptions/time-sync.html),
  so the robot knows *when* each picture was taken, not just when it arrived.

**Each camera must have a [lens (intrinsic) calibration](https://docs.photonvision.org/en/latest/docs/calibration/calibration.html)**
done in PhotonVision at the exact resolution the pipeline uses. Skipping this is the #1
cause of bad vision poses.

### 2. The field map (apriltag-map-app)

Real fields are never exactly the official drawing. The apriltag-map-app measures where
the tags actually are on a given field and exports a field layout JSON.

The same layout must be used in **two places**:

1. **Uploaded to PhotonVision** (Settings → Device Control → Import Settings → "AprilTag
   Layout"; see [Updating the Field Layout](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#updating-the-field-layout)).
   MultiTag is solved on the Mac, so it uses PhotonVision's copy. PhotonLib can't upload
   it for you; do it by hand on every field.
2. **Deployed with the robot code** (`src/main/deploy/`) and loaded into the
   `PhotonPoseEstimator`.

All the image processing (finding tags, solving their 3D pose) happens on the Mac. When
only **one** tag is visible, PhotonVision sends the tag's position *relative to the
camera* and no field pose. PhotonLib on the robot finishes the job with a little math:
where the tag is on the field (from this copy of the map), the tag's position relative to
the camera, and the camera's position on the robot. If the two copies of the map differ,
MultiTag and single-tag estimates will disagree.

### 3. Camera positions on the robot (turntable calibration)

The robot code needs to know where each camera is relative to the robot center: its
x/y/z position and roll/pitch/yaw. This is the `robotToCamera` transform (axes are
explained in [Coordinate Systems](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html)).

We measure it with the turntable calibration in
[TurntableCalibration.java](src/main/java/first/robot/vision/TurntableCalibration.java):
put a tag in front of the robot, spin the robot in place, and the code fits the circle the
camera travels on. It finds the camera's distance from the spin axis, height, and tilt.

> The calibration is being extended to handle all three cameras. All three must be
> expressed in the **same** robot frame (same center, same "forward").

Repeat the calibration whenever a camera mount is bumped or moved.

### 4. Robot code (PhotonLib)

> **PhotonLib has no WPILib 2027 release yet.** The code below uses the current PhotonLib
> API and will need adjusting when the 2027 version comes out. In particular:
>
> - WPILib 2027 replaced `AprilTagFieldLayout` with `org.wpilib.fields.Field`
>   (`Field.loadFromFile(path)`), and the field JSON format changed (`"field-tags"` /
>   `"field-dimensions"` instead of `"tags"` / `"field"`). PhotonVision still imports the
>   old format. Source: [allwpilib#9166](https://github.com/wpilibsuite/allwpilib/pull/9166),
>   current [Field.java](https://github.com/wpilibsuite/allwpilib/blob/main/fields/src/main/java/org/wpilib/fields/Field.java)
>   and [2026-rebuilt-welded.json](https://github.com/wpilibsuite/allwpilib/blob/main/fields/src/main/native/resources/org/wpilib/fields/frc/2026-rebuilt-welded.json).
> - After alpha 8, WPILib moves the field origin to the **center of the field** with +X
>   toward the blue alliance wall (axes rotated 180° from 2026). Poses are not tagged with
>   their coordinate system, so the PhotonVision layout, the robot's layout and the
>   dashboards must all use the same convention, and the "off the field" check below
>   becomes `|x| > length/2`, `|y| > width/2`. Source:
>   [allwpilib#9590](https://github.com/wpilibsuite/allwpilib/pull/9590).

One `PhotonCamera` + `PhotonPoseEstimator` per physical camera. Every robot loop, for
each camera, read **all** new results (not just the latest), turn each into a pose, decide
how much to trust it, and give it to the drivetrain pose estimator. This follows the
pattern in PhotonVision's [PhotonPoseEstimator guide](https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html)
(install PhotonLib with the [vendordep instructions](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html)).

```java
// One of these per camera.
record VisionCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {}

AprilTagFieldLayout layout =
    new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/field-layout.json");

List<VisionCamera> cameras = List.of(
    new VisionCamera(new PhotonCamera("front"), new PhotonPoseEstimator(layout, FRONT_ROBOT_TO_CAM)),
    new VisionCamera(new PhotonCamera("left"),  new PhotonPoseEstimator(layout, LEFT_ROBOT_TO_CAM)),
    new VisionCamera(new PhotonCamera("right"), new PhotonPoseEstimator(layout, RIGHT_ROBOT_TO_CAM)));

void updateVision() {
  for (var cam : cameras) {
    // Record the gyro heading EVERY loop, even if no single-tag fallback happens this loop.
    // PhotonLib keeps the last 1 s of headings and looks up the one matching each frame's
    // timestamp (the picture was taken a bit in the past).
    cam.estimator().addHeadingData(Timer.getTimestamp(), swerve.getState().Pose.getRotation());

    for (var result : cam.camera().getAllUnreadResults()) {
      // Prefer MultiTag (solved on the Mac). Fall back to one tag + gyro heading.
      var estimate = cam.estimator().estimateCoprocMultiTagPose(result);
      // Single tag uses the estimator's heading, so only once MultiTag has fixed it.
      if (estimate.isEmpty() && headingFromMultiTag) {
        estimate = cam.estimator().estimatePnpDistanceTrigSolvePose(result);
      }
      estimate.ifPresent(est -> {
        if (!isReasonable(est, result)) return;
        if (est.targetsUsed.size() >= 2) headingFromMultiTag = true;
        swerve.addVisionMeasurement(
            est.estimatedPose.toPose2d(), toPhoenixTime(est.timestampSeconds), stdDevsFor(est, result));
      });
    }
  }
}

// PhotonVision timestamps use the WPILib clock (Timer.getTimestamp(); the old
// Timer.getFPGATimestamp() is gone in 2027). Phoenix's
// addVisionMeasurement wants the Phoenix clock (Utils.getCurrentTimeSeconds()).
// Utils.fpgaToCurrentTime() is gone in 2027, so convert by hand.
double toPhoenixTime(double wpilibSeconds) {
  return wpilibSeconds - Timer.getTimestamp() + Utils.getCurrentTimeSeconds();
}
```

> **Unlike the [PhotonVision docs example](https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html)
> (`estimateLowestAmbiguityPose`), our single-tag fallback takes heading from the gyro,
> which avoids picking the wrong one of a single tag's two mirror-image solutions but
> means MultiTag must first correct the heading (see [Auton vs. teleop](#auton-vs-teleop)).**

PhotonLib method names change between seasons; check the docs for the version you
install. Camera names (`"front"`, ...) must match the names set in PhotonVision. Reading
raw results is covered in [Getting Target Data](https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html)
and [Using Target Data](https://docs.photonvision.org/en/latest/docs/programming/photonlib/using-target-data.html).

Wheel odometry needs no code from us: Phoenix 6 updates it in its own high-rate thread.
Vision measurements can arrive slightly late or out of order; the estimator uses the
timestamp to apply them at the right moment in the past. That's why the timestamp
conversion above matters: a wrong clock makes vision fight odometry while driving.

## The two things that make or break it

Feeding every measurement in "raw" works on a good day. These two additions make it
reliable.

### Trust: standard deviations

The third argument to `addVisionMeasurement` says how wrong this measurement might be
(x meters, y meters, heading radians). Bigger number = trust less.

| Situation                           | Trust          | Example std devs (x, y, θ)  |
|-------------------------------------|----------------|-----------------------------|
| MultiTag, close                     | high           | 0.3 m, 0.3 m, 0.5 rad       |
| Single tag + gyro, close            | medium         | 0.9 m, 0.9 m, never heading |
| Anything far away                   | lower          | scale by distance²          |

A common recipe:

```java
Matrix<N3, N1> stdDevsFor(EstimatedRobotPose est, PhotonPipelineResult result) {
  int tagCount = est.targetsUsed.size();
  double avgDist = est.targetsUsed.stream()
      .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
      .average().orElse(99);

  double xy = (tagCount >= 2 ? 0.3 : 0.9) * (1 + avgDist * avgDist / 30);
  // Single-tag heading IS the gyro heading, so it adds nothing. Only MultiTag nudges heading.
  double theta = tagCount >= 2 ? 0.5 * (1 + avgDist * avgDist / 30) : 9999;
  return VecBuilder.fill(xy, xy, theta);
}
```

These numbers are a starting point. Tune them by watching logs (below).

### Rejection: throw out obviously bad poses

Reject a measurement if:

- The pose is **off the field** (x or y outside the field dimensions).
- The 3D pose is **floating or underground** (|z| > ~0.25 m) or badly tilted. The robot
  is on the floor; a pose that isn't means the solve went wrong.
- The tag is **too far** (e.g. > 4–5 m for single tag). Small pixel errors become big
  position errors at distance.

```java
boolean isReasonable(EstimatedRobotPose est, PhotonPipelineResult result) {
  Pose3d p = est.estimatedPose;
  if (p.getX() < 0 || p.getX() > layout.getFieldLength()) return false;
  if (p.getY() < 0 || p.getY() > layout.getFieldWidth()) return false;
  if (Math.abs(p.getZ()) > 0.25) return false;
  if (est.targetsUsed.size() == 1) {
    var t = est.targetsUsed.get(0);
    if (t.getBestCameraToTarget().getTranslation().getNorm() > 4.5) return false;
  }
  return true;
}
```

## Auton vs. teleop

Same code in both. We never call `swerve.resetPose(...)`: the cameras must find the
robot's real spot on the field while it sits there before the match. That needs three
things:

- **Trust vision a lot while disabled.** With normal std devs, the estimator moves only
  part of the way toward each reading, so a pose that starts at (0, 0) creeps across the
  field instead of jumping. While `DriverStation.isDisabled()`, use much smaller std devs
  (e.g. 0.05 m, 0.05 m, 0.05 rad for MultiTag) so it snaps within a second.
- **Heading comes from MultiTag.** The pose starts with an arbitrary heading, and the
  single-tag fallback uses that heading (the estimator's heading, which vision corrects,
  not the raw gyro). Until MultiTag has corrected the heading at least once, single-tag
  poses are wrong, so skip the single-tag fallback until then. Place the robot where a
  camera sees 2+ tags before the match.
- **No "close to the current pose" filter.** Phoenix's docs suggest only accepting vision
  within about a meter of the current pose. Don't: the first good reading is always far
  from the starting (0, 0) pose and would be thrown away.

## Debugging with AdvantageScope

`DataLogManager.logNetworkTables(true)` records everything on NetworkTables, which
includes PhotonVision's [raw results](https://docs.photonvision.org/en/latest/docs/additional-resources/nt-api.html)
for every camera (`/photonvision/<camera>/rawBytes`: tags seen, camera-to-tag poses,
MultiTag field-to-camera pose). Add `/photonvision` to the `MultiSubscriber` in
`Robot.java` (next to `/limelight`) so every frame is recorded, not just the latest.

What is **not** on NT unless we publish it is anything the robot code computes. Log,
for **each camera separately**:

- the estimated robot pose (`Pose3d`, after `robotToCamera` and the single-tag solve),
- the std devs it was given,
- whether it was accepted or rejected, and why.

Then in AdvantageScope's 3D field view, show the final fused pose plus the three per-camera
poses as ghost robots.

- All three ghosts agree and sit on the fused pose → everything is working.
- One camera's ghost is consistently offset or rotated → that camera's `robotToCamera`
  (turntable calibration) is wrong.
- All ghosts jump around near certain tags → that tag's position in the field map is off;
  re-run the apriltag-map-app.
- Ghosts are fine standing still but lag/lead while driving → timestamp / latency issue.

This is the main advantage of keeping cameras independent: you can see which one is
wrong.

## Setup checklist

1. Lens-calibrate each camera in PhotonVision at its pipeline resolution.
2. Map the field with apriltag-map-app; export the layout JSON.
3. Upload the layout to PhotonVision **and** put it in `src/main/deploy/`.
4. Create one AprilTag pipeline per camera: 3D mode, MultiTag on; name the cameras.
5. Point PhotonVision's NetworkTables at the SystemCore.
6. Turntable-calibrate the cameras; enter the three `robotToCamera` transforms in
   `Constants`.
7. Add the vision loop above; start with the example std devs and rejection rules.
8. Drive around, check the AdvantageScope ghosts, tune.

## PhotonVision docs

- [MultiTag localization](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html)
  (including uploading the field layout)
- [AprilTag pipelines](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/about-apriltags.html),
  [3D tracking](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html),
  [detector types](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/detector-types.html)
- [Coordinate systems](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html)
- [Camera calibration](https://docs.photonvision.org/en/latest/docs/calibration/calibration.html)
- [Networking](https://docs.photonvision.org/en/latest/docs/quick-start/networking.html),
  [camera matching](https://docs.photonvision.org/en/latest/docs/quick-start/camera-matching.html)
- [Time synchronization](https://docs.photonvision.org/en/latest/docs/contributing/design-descriptions/time-sync.html)
- [Installing PhotonLib](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html)
- [Getting target data](https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html),
  [using target data](https://docs.photonvision.org/en/latest/docs/programming/photonlib/using-target-data.html)
- [PhotonPoseEstimator](https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html)
  (all pose strategies, std dev example)
- [Advanced strategies](https://docs.photonvision.org/en/latest/docs/integration/advancedStrategies.html)
- [NetworkTables API](https://docs.photonvision.org/en/latest/docs/additional-resources/nt-api.html)
- [Simulation (Java)](https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html)
  (test the vision code without a robot)
