package frc.robot.util;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SOTMConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class SOTMCalculator {
  //   public static InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
  // SOTMConstants.hoodAngleMap;
  //   public static InterpolatingDoubleTreeMap shooterSpeedMap = SOTMConstants.shooterSpeedMap;
  //   public static InterpolatingDoubleTreeMap timeOfFlightMap = SOTMConstants.timeOfFlightMap;

  // private static double distanceToHoodRegression(double dMeters) {
  //   return 18.6 * Math.pow(Math.E, 0.0759 * dMeters);
  // }

  // private static double distanceToShooterRegression(double dMeters) {
  //   return (16.6 + 9.8 * dMeters) - (0.627 * dMeters * dMeters);
  // }

  // private static double distanceToTimeRegression(double dMeters) {
  //   return 0.622 + (0.274 * dMeters) - (0.0193 * dMeters * dMeters);
  // }

  private static Translation2d robotToTurret2d = TurretConstants.robotToTurret.toTranslation2d();

  //   public static Time accelTime = Seconds.of(SimConstants.loopPeriodSecs);

  public record ShootingParameters(
      AngularVelocity shooterSpeed,
      Angle turretAngle,
      Angle hoodAngle,
      Translation2d lookAheadPosition) {}

  public static ShootingParameters getParameters(
      Swerve swerve,
      Turret turret,
      Pose2d target,
      double fieldAccelX,
      double fieldAccelY,
      ChassisSpeeds fieldChassisSpeeds,
      boolean isScoring) {
    Pose2d targetPose = target;

    InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
        isScoring ? SOTMConstants.hoodAngleMapScoring : SOTMConstants.hoodAngleMapFerrying;
    InterpolatingDoubleTreeMap shooterSpeedMap =
        isScoring ? SOTMConstants.shooterSpeedMapScoring : SOTMConstants.shooterSpeedMapFerrying;
    InterpolatingDoubleTreeMap timeOfFlightMap =
        isScoring ? SOTMConstants.timeOfFlightMapScoring : SOTMConstants.timeOfFlightMapFerrying;

    if (RobotBase.isSimulation()) {
      hoodAngleMap = SOTMConstants.hoodAngleMapSim;
      shooterSpeedMap = SOTMConstants.shooterSpeedMapSim;
      timeOfFlightMap = SOTMConstants.timeOfFlightMapSim;
    }

    Pose2d robotPose = swerve.getRobotPose();

    Translation2d turretPose =
        robotPose.getTranslation().plus(robotToTurret2d.rotateBy(robotPose.getRotation()));

    double robotAngle = robotPose.getRotation().getRadians();

    double turretVelocityX =
        fieldChassisSpeeds.vxMetersPerSecond
            + fieldChassisSpeeds.omegaRadiansPerSecond
                * (robotToTurret2d.getY() * Math.cos(robotAngle)
                    - robotToTurret2d.getX() * Math.sin(robotAngle));

    double turretVelocityY =
        fieldChassisSpeeds.vyMetersPerSecond
            + fieldChassisSpeeds.omegaRadiansPerSecond
                * (robotToTurret2d.getX() * Math.cos(robotAngle)
                    - robotToTurret2d.getY() * Math.sin(robotAngle));

    Translation2d lookAheadPosition = targetPose.getTranslation();

    double turretToTargetDistance = lookAheadPosition.getDistance(turretPose);

    double timeOfFlight = timeOfFlightMap.get(turretToTargetDistance);

    SmartDashboard.putNumber("SOTM/turretToLookAheadDistance", turretToTargetDistance);

    for (int i = 0; i < 20; i++) { // 20
      double offsetX =
          timeOfFlight
              * (turretVelocityX
              // + fieldAccelX * accelTime.in(Seconds)
              );
      double offsetY =
          timeOfFlight
              * (turretVelocityY
              // + fieldAccelY * accelTime.in(Seconds)
              );

      lookAheadPosition = targetPose.getTranslation().minus(new Translation2d(offsetX, offsetY));

      double newDistance = lookAheadPosition.getDistance(turretPose);

      timeOfFlight = timeOfFlightMap.get(newDistance);

      boolean hasConverged = Math.abs(newDistance - turretToTargetDistance) < 0.005;

      turretToTargetDistance = newDistance;

      if (hasConverged) {
        break;
      }
    }

    Angle turretAngle = turret.fieldAngleToFaceTarget(lookAheadPosition, swerve.getRobotPose());
    Rotation2d hoodAngle = hoodAngleMap.get(turretToTargetDistance);
    AngularVelocity shooterSpeed =
        RotationsPerSecond.of(shooterSpeedMap.get(turretToTargetDistance));

    return new ShootingParameters(
        shooterSpeed, turretAngle, hoodAngle.getMeasure(), lookAheadPosition);
  }
}
