package frc.robot.util;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class ShotCalculator {
  private static InterpolatingDoubleTreeMap hoodAngleMap = HoodConstants.hoodAngleMap;
  private static InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
  private static Transform2d robotToTurret =
      new Transform2d(TurretConstants.robotToTurret.toTranslation2d(), Rotation2d.kZero);
  private static Time feedTime = Seconds.of(0.1353);

  public record ShootingParameters(double shooterSpeed, Angle turretAngle, Angle hoodAngle) {}

  public static ShootingParameters getParameters(
      Swerve swerve, Turret turret, double fieldAccelX, double fieldAccelY) {
    Pose2d targetPose = AllianceUtil.getHubPose();
    Pose2d turretPose = swerve.getRobotPose().transformBy(robotToTurret);

    ChassisSpeeds robotVelocity = swerve.getChassisSpeeds();

    double robotAngle = swerve.getRobotPose().getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    Translation2d lookAheadPosition = targetPose.getTranslation();

    double turretToTargetDistance = lookAheadPosition.getDistance(turretPose.getTranslation());

    Angle turretAngle = turret.angleToFaceTarget(lookAheadPosition, swerve.getRobotPose());
    Angle hoodAngle = Rotations.of(hoodAngleMap.get(turretToTargetDistance));
    double shooterSpeed = shooterSpeedMap.get(turretToTargetDistance);

    double timeOfFlight = timeOfFlightMap.get(turretToTargetDistance);

    double distance = lookAheadPosition.getDistance(turretPose.getTranslation());

    for (int i = 0; i < 5; i++) {
      // + accelX * feedTime.in(Seconds) * 0.5
      double offsetX = timeOfFlight * (turretVelocityX + fieldAccelX * feedTime.in(Seconds) * 0.5);
      // + accelY * feedTime.in(Seconds) * 0.5
      double offsetY = timeOfFlight * (turretVelocityY + fieldAccelY * feedTime.in(Seconds) * 0.5);

      lookAheadPosition = targetPose.getTranslation().minus(new Translation2d(offsetX, offsetY));

      double newDistance = lookAheadPosition.getDistance(turretPose.getTranslation());

      timeOfFlight = timeOfFlightMap.get(newDistance);
      Angle newTurretAngle = turret.angleToFaceTarget(lookAheadPosition, swerve.getRobotPose());
      Angle newHoodAngle = Rotations.of(hoodAngleMap.get(newDistance));
      double newShooterSpeed = shooterSpeedMap.get(newDistance);

      boolean hasConverged = Math.abs(newDistance - distance) < 0.005;

      turretAngle = newTurretAngle;
      hoodAngle = newHoodAngle;
      shooterSpeed = newShooterSpeed;

      turretToTargetDistance = newDistance;

      if (hasConverged) {
        break;
      }
    }
    return new ShootingParameters(shooterSpeed, turretAngle, hoodAngle);
  }
}
