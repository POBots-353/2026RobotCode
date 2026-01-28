package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class SOTMCalculator {

  public static InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  public static InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {
    hoodAngleMap.put(2.9, Rotation2d.fromDegrees(21.448));
    hoodAngleMap.put(3.339, Rotation2d.fromDegrees(21.448));
    hoodAngleMap.put(2.614, Rotation2d.fromDegrees(21.448));
    hoodAngleMap.put(2.598, Rotation2d.fromDegrees(21.448));
    hoodAngleMap.put(3.520, Rotation2d.fromDegrees(21.448));
    hoodAngleMap.put(6.768, Rotation2d.fromDegrees(28.994));
    hoodAngleMap.put(2.865, Rotation2d.fromDegrees(21.488));
    hoodAngleMap.put(4.497, Rotation2d.fromDegrees(21.448));
    hoodAngleMap.put(5.907, Rotation2d.fromDegrees(23.473));
    hoodAngleMap.put(7.469, Rotation2d.fromDegrees(36.911)); // Random values distance, Degrees

    shooterSpeedMap.put(2.906, 7.112);
    shooterSpeedMap.put(3.339, 7.523);
    shooterSpeedMap.put(2.614, 6.825);
    shooterSpeedMap.put(2.598, 6.810);
    shooterSpeedMap.put(3.520, 7.689);
    shooterSpeedMap.put(6.768, 9.353);
    shooterSpeedMap.put(2.865, 7.080);
    shooterSpeedMap.put(4.497, 8.539);
    shooterSpeedMap.put(5.907, 9.353);
    shooterSpeedMap.put(7.469, 9.353);
    // random values (Distance, MPS)

    timeOfFlightMap.put(2.0, 1.09);
    timeOfFlightMap.put(4.135, 1.53);
    timeOfFlightMap.put(3.210, 1.35);
    timeOfFlightMap.put(4.535, 1.63);
    timeOfFlightMap.put(3.53, 1.45);
    timeOfFlightMap.put(8.057, 2.13);
    timeOfFlightMap.put(1.33, 0.91); // random values (Distance, seconds)
  }

  private static Translation2d robotToTurret2d = TurretConstants.robotToTurret.toTranslation2d();
  public static Time accelTime = Seconds.of(0.1353);

  public record ShootingParameters(double shooterSpeed, Angle turretAngle, Angle hoodAngle) {}

  public static ShootingParameters getParameters(
      Swerve swerve,
      Turret turret,
      Pose2d target,
      double fieldAccelX,
      double fieldAccelY,
      ChassisSpeeds fieldChassisSpeeds) {
    Pose2d targetPose = target;
    Translation2d turretPose = swerve.getRobotPose().getTranslation().plus(robotToTurret2d);

    double robotAngle = swerve.getRobotPose().getRotation().getRadians();
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

    Angle turretAngle = turret.angleToFaceTarget(lookAheadPosition, swerve.getRobotPose());
    Rotation2d hoodAngle = hoodAngleMap.get(turretToTargetDistance);
    double shooterSpeed = shooterSpeedMap.get(turretToTargetDistance);

    double timeOfFlight = timeOfFlightMap.get(turretToTargetDistance);

    double distance = lookAheadPosition.getDistance(turretPose);

    SmartDashboard.putNumber("SOTM/distance", distance);

    // for (int i = 0; i < 5; i++) {
    double offsetX = timeOfFlight * (turretVelocityX 
    // + fieldAccelX * accelTime.in(Seconds) TS IS BROKEN BRO
    );
    double offsetY = timeOfFlight * (turretVelocityY 
    // + fieldAccelY * accelTime.in(Seconds) ACCELERATION DOES NOT WORK
    );

    lookAheadPosition = targetPose.getTranslation().minus(new Translation2d(offsetX, offsetY));

    double newDistance = lookAheadPosition.getDistance(turretPose);

    timeOfFlight = timeOfFlightMap.get(newDistance);
    Angle newTurretAngle = turret.angleToFaceTarget(lookAheadPosition, swerve.getRobotPose());
    Rotation2d newHoodAngle = hoodAngleMap.get(newDistance);
    double newShooterSpeed = shooterSpeedMap.get(newDistance);

    // boolean hasConverged = Math.abs(newDistance - distance) < 0.00005;
    boolean hasConverged = Math.abs(newDistance - distance) < 5.0;

    turretAngle = newTurretAngle;
    hoodAngle = newHoodAngle;
    shooterSpeed = newShooterSpeed;

    turretToTargetDistance = newDistance;

    // if (hasConverged) {
    //   break;
    // }
    // }
    return new ShootingParameters(shooterSpeed, turretAngle, hoodAngle.getMeasure());
  }
}
