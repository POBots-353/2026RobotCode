package frc.robot.util;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class SOTMCalculator {

public static InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  public static InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {
    hoodAngleMap.put(0.8, Rotation2d.fromDegrees(16.0));
    hoodAngleMap.put(0.97, Rotation2d.fromDegrees(17.0));
    hoodAngleMap.put(1.1, Rotation2d.fromDegrees(18.0));
    hoodAngleMap.put(1.21, Rotation2d.fromDegrees(18.5));
    hoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    hoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    hoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    hoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    hoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    hoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0)); // Random values

    shooterSpeedMap.put(1.5, 1.0);
    shooterSpeedMap.put(2.0, 2.0);
    shooterSpeedMap.put(2.5, 3.0);
    shooterSpeedMap.put(3.0, 4.0);
    shooterSpeedMap.put(3.53, 5.0); // random values (Distance, RPM)

    timeOfFlightMap.put(1.5, 1.0);
    timeOfFlightMap.put(2.0, 1.0);
    timeOfFlightMap.put(2.5, 1.0);
    timeOfFlightMap.put(3.0, 1.0);
    timeOfFlightMap.put(3.53, 1.0); // random values (Distance, seconds)
  }

  private static Translation2d robotToTurret2d = TurretConstants.robotToTurret.toTranslation2d();
  public static Time feedTime = Seconds.of(0.1353);


  public record ShootingParameters(double shooterSpeed, Angle turretAngle, Angle hoodAngle) {}

  public static ShootingParameters getParameters(
      Swerve swerve, Turret turret, Pose2d target, double fieldAccelX, double fieldAccelY) {
    Pose2d targetPose = target;
    Translation2d turretPose = swerve.getRobotPose().getTranslation().plus(robotToTurret2d);

    ChassisSpeeds robotVelocity = swerve.getChassisSpeeds();

    double robotAngle = swerve.getRobotPose().getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret2d.getY() * Math.cos(robotAngle)
                    - robotToTurret2d.getX() * Math.sin(robotAngle));

    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret2d.getX() * Math.cos(robotAngle)
                    - robotToTurret2d.getY() * Math.sin(robotAngle));

    Translation2d lookAheadPosition = targetPose.getTranslation();

    double turretToTargetDistance = lookAheadPosition.getDistance(turretPose);

    Angle turretAngle = turret.angleToFaceTarget(lookAheadPosition, swerve.getRobotPose());
    Rotation2d hoodAngle = hoodAngleMap.get(turretToTargetDistance);
    double shooterSpeed = shooterSpeedMap.get(turretToTargetDistance);

    double timeOfFlight = timeOfFlightMap.get(turretToTargetDistance);

    double distance = lookAheadPosition.getDistance(turretPose);

    for (int i = 0; i < 5; i++) {
      // + accelX * feedTime.in(Seconds) * 0.5
      double offsetX = timeOfFlight * (turretVelocityX + fieldAccelX * feedTime.in(Seconds) * 0.5);
      // + accelY * feedTime.in(Seconds) * 0.5
      double offsetY = timeOfFlight * (turretVelocityY + fieldAccelY * feedTime.in(Seconds) * 0.5);

      lookAheadPosition = targetPose.getTranslation().minus(new Translation2d(offsetX, offsetY));

      double newDistance = lookAheadPosition.getDistance(turretPose);

      timeOfFlight = timeOfFlightMap.get(newDistance);
      Angle newTurretAngle = turret.angleToFaceTarget(lookAheadPosition, swerve.getRobotPose());
      Rotation2d newHoodAngle = hoodAngleMap.get(newDistance);
      double newShooterSpeed = shooterSpeedMap.get(newDistance);

      boolean hasConverged = Math.abs(newDistance - distance) < 0.00005;

      turretAngle = newTurretAngle;
      hoodAngle = newHoodAngle;
      shooterSpeed = newShooterSpeed;

      turretToTargetDistance = newDistance;

      if (hasConverged) {
        break;
      }
    }
    return new ShootingParameters(shooterSpeed, turretAngle, hoodAngle.getMeasure()); 
   }
}
