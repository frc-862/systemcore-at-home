package first.robot.vision;

import java.util.List;

import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Finds where a camera is mounted relative to the robot's center of rotation by spinning the
 * robot (or a lazy susan) in place in front of a fixed AprilTag.
 *
 * <h2>Model</h2>
 *
 * Frames are NWU: x forward, y left, z up. Work in the tag frame, which does not move.
 *
 * <p>For each sample i the camera reports its pose in the tag frame: position p_i and rotation
 * R_i (camera axes expressed in tag axes). The robot spins about a fixed axis through point o with
 * direction n (both in the tag frame). The camera is rigid on the robot, so:
 *
 * <ul>
 *   <li>The robot's up axis expressed in camera axes, z_c, is constant: R_i z_c = n. This gives
 *       n and z_c together as the null vector of [R_i, -I] stacked over all samples.
 *   <li>The vector from the axis to the camera, expressed in camera axes, w, is constant: p_i = o +
 *       R_i w. Linear least squares in (o, w). Sliding o along the axis is a free direction, so we
 *       also require w to be perpendicular to the axis (w . z_c = 0), which puts o at camera height.
 * </ul>
 *
 * So the camera sits on a cylinder of radius |w| around the axis, and the tag sits on another
 * cylinder around the same axis. Everything is linear, so it is well-conditioned even when the tag
 * is only visible over a small arc of rotation, as long as there is some rotation.
 *
 * <p>The robot's forward (x) axis cannot be observed from spinning, so it is defined to point from
 * the axis through the camera (camera y = 0). Heights cannot be observed either (only differences),
 * so the measured height of the tag center above the floor is an input.
 */
public final class TurntableCalibration {
  private TurntableCalibration() {}

  /**
   * Result of a fit.
   *
   * @param cameraToRobot camera pose in robot frame (robot origin on the spin axis at floor level)
   * @param cameraRadiusMeters distance from spin axis to camera
   * @param tagRadiusMeters horizontal distance from spin axis to tag center
   * @param positionRmsMeters RMS of p_i - (o + R_i w), how well the camera moved on a circle
   * @param axisRmsDegrees RMS angle between R_i z_c and n, how consistent the spin axis was
   * @param arcDegrees total rotation covered by the samples (bigger is better, aim for 45+)
   * @param samples number of samples used
   */
  public record Result(
      Pose3d cameraToRobot,
      double cameraRadiusMeters,
      double tagRadiusMeters,
      double positionRmsMeters,
      double axisRmsDegrees,
      double arcDegrees,
      int samples) {
    @Override
    public String toString() {
      var t = cameraToRobot.getTranslation();
      var r = cameraToRobot.getRotation();
      return String.format(
          "Camera in robot: x=%.4f y=%.4f z=%.4f m, roll=%.2f pitch=%.2f yaw=%.2f deg%n"
              + "  camera radius=%.4f m, tag radius=%.4f m%n"
              + "  fit: position RMS=%.2f mm, axis RMS=%.3f deg, arc=%.1f deg, samples=%d",
          t.getX(), t.getY(), t.getZ(),
          Math.toDegrees(r.getX()), Math.toDegrees(r.getY()), Math.toDegrees(r.getZ()),
          cameraRadiusMeters, tagRadiusMeters,
          positionRmsMeters * 1000, axisRmsDegrees, arcDegrees, samples);
    }
  }

  /**
   * Fit the camera mounting from samples.
   *
   * @param cameraInTag camera pose in tag space for each sample (Limelight cameraPoseTargetSpace)
   * @param tagCenterHeightMeters height of the tag center above the floor
   * @return the fit
   */
  public static Result fit(List<Pose3d> cameraInTag, double tagCenterHeightMeters) {
    int n = cameraInTag.size();
    if (n < 3) {
      throw new IllegalArgumentException("Need at least 3 samples, got " + n);
    }

    SimpleMatrix[] rot = new SimpleMatrix[n];
    SimpleMatrix[] pos = new SimpleMatrix[n];
    for (int i = 0; i < n; i++) {
      rot[i] = cameraInTag.get(i).getRotation().toMatrix().getStorage();
      pos[i] = vec(cameraInTag.get(i).getTranslation());
    }

    // 1. Spin axis: R_i z_c - n = 0 for all i. Null vector of A (3n x 6).
    SimpleMatrix a = new SimpleMatrix(3 * n, 6);
    for (int i = 0; i < n; i++) {
      a.insertIntoThis(3 * i, 0, rot[i]);
      a.insertIntoThis(3 * i, 3, SimpleMatrix.identity(3).negative());
    }
    SimpleMatrix nullVec = smallestRightSingularVector(a);
    SimpleMatrix zc = unit(nullVec.extractMatrix(0, 3, 0, 1));
    SimpleMatrix axis = unit(nullVec.extractMatrix(3, 6, 0, 1));
    if (axis.get(2) < 0) {
      // The null vector's sign is arbitrary; pick the axis pointing up in the tag frame (tag z is
      // up for an upright tag). z_c flips with it since R_i z_c = n.
      axis = axis.negative();
      zc = zc.negative();
    }

    // 2. Center and offset: [I R_i] [o; w] = p_i, plus w . z_c = 0.
    SimpleMatrix b = new SimpleMatrix(3 * n + 1, 6);
    SimpleMatrix y = new SimpleMatrix(3 * n + 1, 1);
    for (int i = 0; i < n; i++) {
      b.insertIntoThis(3 * i, 0, SimpleMatrix.identity(3));
      b.insertIntoThis(3 * i, 3, rot[i]);
      y.insertIntoThis(3 * i, 0, pos[i]);
    }
    double constraintWeight = Math.sqrt(n);
    b.insertIntoThis(3 * n, 3, transpose(zc).scale(constraintWeight));
    SimpleMatrix sol = b.solve(y); // least squares
    SimpleMatrix center = sol.extractMatrix(0, 3, 0, 1);
    SimpleMatrix w = sol.extractMatrix(3, 6, 0, 1);

    // 3. Robot axes in camera frame. x points from axis to camera, z up.
    SimpleMatrix xc = unit(w.minus(zc.scale(w.dot(zc))));
    SimpleMatrix yc = cross(zc, xc);
    SimpleMatrix robotAxesInCamera = new SimpleMatrix(3, 3);
    robotAxesInCamera.insertIntoThis(0, 0, xc);
    robotAxesInCamera.insertIntoThis(0, 1, yc);
    robotAxesInCamera.insertIntoThis(0, 2, zc);
    // Camera axes in robot frame is the transpose.
    SimpleMatrix cameraAxesInRobot = transpose(robotAxesInCamera);

    double cameraRadius = w.normF();
    double cameraHeight = tagCenterHeightMeters + center.dot(axis);
    SimpleMatrix tagFromCenter = center.negative();
    double tagRadius = tagFromCenter.minus(axis.scale(tagFromCenter.dot(axis))).normF();

    // 4. Residuals and coverage.
    double posSq = 0;
    double axisSq = 0;
    SimpleMatrix ref1 = unit(perp(rot[0].mult(xc), axis));
    SimpleMatrix ref2 = cross(axis, ref1);
    double minAngle = 0;
    double maxAngle = 0;
    double prev = 0;
    double unwrapped = 0;
    for (int i = 0; i < n; i++) {
      SimpleMatrix e = pos[i].minus(center).minus(rot[i].mult(w));
      posSq += e.dot(e);
      double cos = Math.max(-1, Math.min(1, rot[i].mult(zc).dot(axis)));
      double ang = Math.acos(cos);
      axisSq += ang * ang;

      SimpleMatrix heading = rot[i].mult(xc);
      double theta = Math.atan2(heading.dot(ref2), heading.dot(ref1));
      if (i > 0) {
        double d = Math.IEEEremainder(theta - prev, 2 * Math.PI);
        unwrapped += d;
      }
      prev = theta;
      minAngle = Math.min(minAngle, unwrapped);
      maxAngle = Math.max(maxAngle, unwrapped);
    }

    Pose3d cameraToRobot =
        new Pose3d(
            new Translation3d(cameraRadius, 0, cameraHeight),
            new Rotation3d(new org.wpilib.math.linalg.Matrix<>(cameraAxesInRobot)));

    return new Result(
        cameraToRobot,
        cameraRadius,
        tagRadius,
        Math.sqrt(posSq / n),
        Math.toDegrees(Math.sqrt(axisSq / n)),
        Math.toDegrees(maxAngle - minAngle),
        n);
  }

  private static SimpleMatrix vec(Translation3d t) {
    return new SimpleMatrix(3, 1, true, t.getX(), t.getY(), t.getZ());
  }

  private static SimpleMatrix transpose(SimpleMatrix m) {
    return m.transpose();
  }

  private static SimpleMatrix unit(SimpleMatrix v) {
    return v.divide(v.normF());
  }

  private static SimpleMatrix perp(SimpleMatrix v, SimpleMatrix axis) {
    return v.minus(axis.scale(v.dot(axis)));
  }

  private static SimpleMatrix cross(SimpleMatrix u, SimpleMatrix v) {
    return new SimpleMatrix(
        3,
        1,
        true,
        u.get(1) * v.get(2) - u.get(2) * v.get(1),
        u.get(2) * v.get(0) - u.get(0) * v.get(2),
        u.get(0) * v.get(1) - u.get(1) * v.get(0));
  }

  private static SimpleMatrix smallestRightSingularVector(SimpleMatrix m) {
    SimpleSVD<SimpleMatrix> svd = m.svd();
    SimpleMatrix s = svd.getW();
    int cols = m.getNumCols();
    int best = 0;
    for (int i = 1; i < cols; i++) {
      if (s.get(i, i) < s.get(best, best)) {
        best = i;
      }
    }
    return svd.getV().extractVector(false, best);
  }
}
