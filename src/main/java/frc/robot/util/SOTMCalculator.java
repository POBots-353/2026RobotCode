package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
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
    hoodAngleMap.put(1.681, Rotation2d.fromDegrees(21.448));
    hoodAngleMap.put(2.016, Rotation2d.fromDegrees(21.972));
    hoodAngleMap.put(2.338, Rotation2d.fromDegrees(22.648));
    hoodAngleMap.put(2.665, Rotation2d.fromDegrees(24.122));
    hoodAngleMap.put(3.034, Rotation2d.fromDegrees(25.658));
    hoodAngleMap.put(3.363, Rotation2d.fromDegrees(26.333));
    hoodAngleMap.put(3.711, Rotation2d.fromDegrees(27.562));
    hoodAngleMap.put(4.061, Rotation2d.fromDegrees(29.036));
    hoodAngleMap.put(4.324, Rotation2d.fromDegrees(29.835));
    hoodAngleMap.put(4.619, Rotation2d.fromDegrees(30.449));
    hoodAngleMap.put(5.010, Rotation2d.fromDegrees(32.046));
    hoodAngleMap.put(5.366, Rotation2d.fromDegrees(33.275));
    hoodAngleMap.put(5.610, Rotation2d.fromDegrees(34.319));
    hoodAngleMap.put(6.051, Rotation2d.fromDegrees(35.794));
    hoodAngleMap.put(6.369, Rotation2d.fromDegrees(36.408));
    hoodAngleMap.put(6.704, Rotation2d.fromDegrees(37.002));
    hoodAngleMap.put(7.022, Rotation2d.fromDegrees(36.531));

    shooterSpeedMap.put(1.681, 5.795);
    shooterSpeedMap.put(2.016, 5.940);
    shooterSpeedMap.put(2.338, 6.229);
    shooterSpeedMap.put(2.665, 6.422);
    shooterSpeedMap.put(3.034, 6.711);
    shooterSpeedMap.put(3.363, 6.903);
    shooterSpeedMap.put(3.711, 7.096);
    shooterSpeedMap.put(4.061, 7.337);
    shooterSpeedMap.put(4.324, 7.529);
    shooterSpeedMap.put(4.619, 7.674);
    shooterSpeedMap.put(5.010, 7.915);
    shooterSpeedMap.put(5.366, 8.156);
    shooterSpeedMap.put(5.610, 8.252);
    shooterSpeedMap.put(6.051, 8.445);
    shooterSpeedMap.put(6.369, 8.637);
    shooterSpeedMap.put(6.704, 8.830);
    shooterSpeedMap.put(7.022, 9.023);

    timeOfFlightMap.put(1.695, 0.893);
    timeOfFlightMap.put(2.016, 0.924);
    timeOfFlightMap.put(2.338, 0.985);
    timeOfFlightMap.put(2.665, 1.013);
    timeOfFlightMap.put(3.034, 1.059);
    timeOfFlightMap.put(3.363, 1.093);
    timeOfFlightMap.put(3.711, 1.118);
    timeOfFlightMap.put(4.061, 1.147);
    timeOfFlightMap.put(4.324, 1.175);
    timeOfFlightMap.put(4.619, 1.195);
    timeOfFlightMap.put(5.010, 1.216);
    timeOfFlightMap.put(5.366, 1.242);
    timeOfFlightMap.put(5.610, 1.241);
    timeOfFlightMap.put(6.051, 1.249);
    timeOfFlightMap.put(6.365, 1.273);
    timeOfFlightMap.put(6.704, 1.295);
    timeOfFlightMap.put(7.022, 1.341);
  }

  private static Translation2d robotToTurret2d = TurretConstants.robotToTurret.toTranslation2d();
  public static Time accelTime = Seconds.of(0.1353);

  public record ShootingParameters(LinearVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {}

  public static ShootingParameters getParameters(
      Swerve swerve,
      Turret turret,
      Pose2d target,
      double fieldAccelX,
      double fieldAccelY,
      ChassisSpeeds fieldChassisSpeeds) {
    Pose2d targetPose = target;

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

    SmartDashboard.putNumber("SOTM/distance", turretToTargetDistance);

    for (int i = 0; i < 20; i++) {
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

    Angle turretAngle = turret.angleToFaceTarget(lookAheadPosition, swerve.getRobotPose());
    Rotation2d hoodAngle = hoodAngleMap.get(turretToTargetDistance);
    LinearVelocity shooterSpeed = MetersPerSecond.of(shooterSpeedMap.get(turretToTargetDistance));

    return new ShootingParameters(shooterSpeed, turretAngle, hoodAngle.getMeasure());
  }
}
