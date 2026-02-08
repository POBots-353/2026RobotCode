package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import java.util.Optional;

public class AllianceUtil {
  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  public static Rotation2d getZeroRotation() {
    if (isRedAlliance()) {
      return Rotation2d.fromDegrees(180.0);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }

  public static Pose2d getHubPose() {
    return isRedAlliance() ? FieldConstants.hubRedAlliance : FieldConstants.hubBlueAlliance;
  }

  public static Pose2d flipPose(Pose2d pose) {
    return isRedAlliance()
        ? new Pose2d(
            FieldConstants.fieldLength.in(Meters) - pose.getX(),
            FieldConstants.fieldWidth.in(Meters) - pose.getY(),
            pose.getRotation().rotateBy(Rotation2d.kPi))
        : pose;
  }
}
