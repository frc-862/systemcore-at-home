package first.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableInt;
import org.wpilib.tunable.Tunables;

import com.limelightvision.FiducialTarget;
import com.limelightvision.Limelight;

import first.robot.Constants.VisionConstants;
import first.robot.vision.TurntableCalibration;

/**
 * Reads AprilTag results from the Systemcore camera, publishes them through the telemetry API, and
 * collects samples for {@link TurntableCalibration}.
 */
public class VisionSubsystem extends SubsystemBase {
  private final Limelight camera =
      new Limelight(VisionConstants.CAMERA_NAME).withTelemetry(true);

  // Editable from Elastic under /Tunables/Calibration.
  private final TunableInt tagId =
      Tunables.addInt("Calibration/tagId", VisionConstants.CALIBRATION_TAG_ID);
  private final TunableDouble tagCenterHeightMeters =
      Tunables.addDouble("Calibration/tagCenterHeightMeters", VisionConstants.TAG_CENTER_HEIGHT_METERS);
  private final TunableDouble maxAmbiguity =
      Tunables.addDouble("Calibration/maxAmbiguity", VisionConstants.MAX_AMBIGUITY);

  private final List<Pose3d> samples = new ArrayList<>();
  private boolean collecting = false;
  private int rejected = 0;

  public VisionSubsystem() {
    // Command buttons in Elastic under /Tunables/Calibration.
    Tunables.publish("Calibration/Collect", collectCalibrationSamples());
    Tunables.publish("Calibration/Fit", fitCalibration());
    Tunables.publish("Calibration/Clear", clearCalibrationSamples());
  }

  @Override
  public void periodic() {
    Telemetry.log("Vision/status", camera.getStatus().name());

    // Drain every frame since the last loop so none are lost.
    for (var results : camera.readResultsQueue()) {
      FiducialTarget[] tags = results.fiducialTargets == null ? new FiducialTarget[0] : results.fiducialTargets;

      Pose3d[] tagsInCamera = new Pose3d[tags.length];
      int[] ids = new int[tags.length];
      for (int i = 0; i < tags.length; i++) {
        tagsInCamera[i] = tags[i].getTargetPose_CameraSpace();
        ids[i] = tags[i].fiducialId;
      }
      Telemetry.log("Vision/frameIndex", results.frameIndex);
      Telemetry.log("Vision/latencyMs", results.targetingLatencyMillis + results.captureLatencyMillis);
      Telemetry.log("Vision/tagIds", ids);
      Telemetry.log("Vision/tagPosesCameraSpace", tagsInCamera, Pose3d.struct);

      for (var tag : tags) {
        if (tag.fiducialId != tagId.get()) {
          continue;
        }
        Pose3d cameraInTag = tag.getCameraPose_TargetSpace();
        Telemetry.log("Vision/calTag/cameraPoseTargetSpace", cameraInTag, Pose3d.struct);
        Telemetry.log("Vision/calTag/targetPoseCameraSpace", tag.getTargetPose_CameraSpace(), Pose3d.struct);
        Telemetry.log("Vision/calTag/ambiguity", tag.ambiguity);

        if (collecting) {
          if (tag.ambiguity <= maxAmbiguity.get()) {
            samples.add(cameraInTag);
            Telemetry.log("Calibration/sample", cameraInTag, Pose3d.struct);
          } else {
            rejected++;
          }
        }
      }
    }

    Telemetry.log("Calibration/collecting", collecting);
    Telemetry.log("Calibration/sampleCount", samples.size());
    Telemetry.log("Calibration/rejectedCount", rejected);
  }

  /** Collect calibration samples while this command runs (toggle in Elastic). Works while disabled. */
  public Command collectCalibrationSamples() {
    return startEnd(() -> collecting = true, () -> collecting = false)
        .ignoringDisable(true)
        .withName("CollectCalibrationSamples");
  }

  /** Throw away collected samples. */
  public Command clearCalibrationSamples() {
    return runOnce(() -> {
          samples.clear();
          rejected = 0;
          System.out.println("Calibration: samples cleared");
        })
        .ignoringDisable(true)
        .withName("ClearCalibrationSamples");
  }

  /** Fit the camera pose from the collected samples and publish it. */
  public Command fitCalibration() {
    return runOnce(this::fitAndPublish).ignoringDisable(true).withName("FitCalibration");
  }

  private void fitAndPublish() {
    try {
      var result = TurntableCalibration.fit(samples, tagCenterHeightMeters.get());
      System.out.println("Calibration result:\n" + result);
      Telemetry.log("Calibration/cameraToRobot", result.cameraToRobot(), Pose3d.struct);
      Telemetry.log("Calibration/cameraRadiusMeters", result.cameraRadiusMeters());
      Telemetry.log("Calibration/tagRadiusMeters", result.tagRadiusMeters());
      Telemetry.log("Calibration/positionRmsMeters", result.positionRmsMeters());
      Telemetry.log("Calibration/axisRmsDegrees", result.axisRmsDegrees());
      Telemetry.log("Calibration/arcDegrees", result.arcDegrees());
      Telemetry.log("Calibration/fitSamples", result.samples());
      Telemetry.log("Calibration/summary", result.toString());
    } catch (RuntimeException e) {
      System.out.println("Calibration failed: " + e.getMessage());
      Telemetry.log("Calibration/summary", "failed: " + e.getMessage());
    }
  }
}
