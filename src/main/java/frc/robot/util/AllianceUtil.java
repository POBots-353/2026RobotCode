package frc.robot.util;

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

  private static int getShift(double time) {
    if (time <= 120 && time > 95) return 1;
    if (time <= 95 && time > 70) return 2;
    if (time <= 70 && time > 45) return 3;
    if (time <= 45 && time > 30) return 4;
    return -1;
  }

  private static boolean isOurShift(int shift, char inactiveFirst, boolean isRed) {
    boolean oddShift = (shift % 2 == 1);

    if (inactiveFirst == 'R') {
      return oddShift ? !isRed : isRed;
    } else {
      return oddShift ? isRed : !isRed;
    }
  }

  public static double timeUntilShift() {
    double time = DriverStation.getMatchTime();

    if (DriverStation.isAutonomous() || time > 130 || (time <= 130 && time > 120) || time <= 30) {
      return 0;
    }

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      return 0;
    }

    char inactiveFirst = gameData.charAt(0);
    boolean isRed = isRedAlliance();

    int currentShift = getShift(time);
    if (currentShift == -1) {
      return 0;
    }

    if (isOurShift(currentShift, inactiveFirst, isRed)) {
      return 0;
    }

    switch (currentShift) {
      case 1:
        return time - 95;
      case 2:
        return time - 70;
      case 3:
        return time - 45;
      case 4:
        return 0;
      default:
        return 0;
    }
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
            FieldConstants.fieldLength - pose.getX(),
            FieldConstants.fieldWidth - pose.getY(),
            pose.getRotation().rotateBy(Rotation2d.kPi))
        : pose;
  }
}
