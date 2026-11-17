package first.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

class TurntableCalibrationTest {
  // Truth: camera 0.25 m out, 0.18 m up, rolled 2, pitched up 10, yawed 15 degrees.
  static final Pose3d CAMERA_IN_ROBOT =
      new Pose3d(0.25, 0, 0.18, Rotation3d.fromDegrees(2, -10, 15));
  // Tag 1.5 m away, 0.30 m high, facing back toward the robot (tag x points out of its face).
  static final double TAG_HEIGHT = 0.30;
  static final Pose3d TAG_IN_WORLD =
      new Pose3d(1.5, 0.2, TAG_HEIGHT, Rotation3d.fromDegrees(0, 0, 180));

  static List<Pose3d> simulate(double startDeg, double endDeg, int count, double posNoise,
      double rotNoiseDeg, long seed) {
    var rand = new Random(seed);
    var samples = new ArrayList<Pose3d>();
    var cameraTransform = new Transform3d(CAMERA_IN_ROBOT.getTranslation(), CAMERA_IN_ROBOT.getRotation());
    for (int i = 0; i < count; i++) {
      double theta = Math.toRadians(startDeg + (endDeg - startDeg) * i / (count - 1));
      Pose3d robot = new Pose3d(0, 0, 0, new Rotation3d(0, 0, theta));
      Pose3d cameraInWorld = robot.transformBy(cameraTransform);
      Pose3d cameraInTag = cameraInWorld.relativeTo(TAG_IN_WORLD);
      var noisy =
          new Transform3d(
              new Translation3d(
                  rand.nextGaussian() * posNoise, rand.nextGaussian() * posNoise,
                  rand.nextGaussian() * posNoise),
              Rotation3d.fromDegrees(
                  rand.nextGaussian() * rotNoiseDeg, rand.nextGaussian() * rotNoiseDeg,
                  rand.nextGaussian() * rotNoiseDeg));
      samples.add(cameraInTag.transformBy(noisy));
    }
    return samples;
  }

  static void assertMatches(TurntableCalibration.Result r, double posTol, double angTolDeg) {
    System.out.println(r);
    var t = r.cameraToRobot().getTranslation();
    var rot = r.cameraToRobot().getRotation();
    var truth = CAMERA_IN_ROBOT.getRotation();
    assertEquals(0.25, t.getX(), posTol);
    assertEquals(0.0, t.getY(), 1e-9);
    assertEquals(0.18, t.getZ(), posTol);
    assertEquals(Math.toDegrees(truth.getX()), Math.toDegrees(rot.getX()), angTolDeg);
    assertEquals(Math.toDegrees(truth.getY()), Math.toDegrees(rot.getY()), angTolDeg);
    assertEquals(Math.toDegrees(truth.getZ()), Math.toDegrees(rot.getZ()), angTolDeg);
  }

  @Test
  void exactDataRecoversTruth() {
    // Camera yawed 15 deg sees the tag roughly when robot heading is near -15 deg.
    var r = TurntableCalibration.fit(simulate(-45, 15, 30, 0, 0, 1), TAG_HEIGHT);
    assertMatches(r, 1e-6, 1e-6);
  }

  @Test
  void noisyDataOverSmallArc() {
    // 2 mm and 0.3 deg noise, 60 deg of rotation, 300 samples.
    var r = TurntableCalibration.fit(simulate(-45, 15, 300, 0.002, 0.3, 2), TAG_HEIGHT);
    assertMatches(r, 0.01, 1.0);
  }
}
