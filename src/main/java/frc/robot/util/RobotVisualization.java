package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.util.HoodShotCalculator.ShotSolution;
import java.util.ArrayList;
import java.util.List;

public class RobotVisualization {
  private Turret turret;
  private Hood hood;
  private Swerve swerve;
  private Shooter shooter;

  private final List<FuelProjectile> activeShots = new ArrayList<>();

  public RobotVisualization(Turret turret, Hood hood, Swerve swerve, Shooter shooter) {
    this.turret = turret;
    this.hood = hood;
    this.swerve = swerve;
    this.shooter = shooter;
  }

  @Logged(name = "Turret")
  public Pose3d getTurretPose3d() {
    return new Pose3d(
        -0.1318, -0.15248, 0.376, new Rotation3d(0, 0, turret.getTurretAngle().in(Radians)));
  }

  @Logged(name = "Hood")
  public Pose3d getHoodPose3d() {
    return getTurretPose3d()
        .transformBy(
            new Transform3d(
                new Translation3d(0.121, -0.0025, 0.072),
                new Rotation3d(0.0, hood.getHoodAngle().in(Radians), 0.0)));
  }

  @Logged(name = "FuelPoses")
  public Pose3d[] getFuelPoses() {
    return activeShots.stream().map(p -> p.getPose()).toArray(s -> new Pose3d[s]);
  }

  public Command shootFuel() {
    return Commands.runOnce(
            () -> {
              Pose3d robotPose = new Pose3d(swerve.getRobotPose());

              ChassisSpeeds robotRelative = swerve.getChassisSpeeds();

              ChassisSpeeds fieldRelative =
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      robotRelative, robotPose.getRotation().toRotation2d());
              ShotSolution shotSolution = shooter.getShotSolution();

              Rotation2d turretYaw = Rotation2d.fromDegrees(turret.getTurretAngle().in(Degrees));
              Rotation2d hoodPitch = Rotation2d.fromDegrees(shotSolution.hoodAngle().in(Degrees));

              double shotYaw = turretYaw.plus(robotPose.toPose2d().getRotation()).getRadians();
              double shotPitch = Units.degreesToRadians(90) - hoodPitch.getRadians();

              double exitVelocity = shotSolution.exitVelocity().in(MetersPerSecond);

              double vXInit = fieldRelative.vxMetersPerSecond;
              double vYInit = fieldRelative.vyMetersPerSecond;

              double vXRot =
                  -fieldRelative.omegaRadiansPerSecond
                      * TurretConstants.robotToTurret.toTranslation2d().getY();
              double vYRot =
                  fieldRelative.omegaRadiansPerSecond
                      * TurretConstants.robotToTurret.toTranslation2d().getX();

              double vX = exitVelocity * Math.cos(shotPitch) * Math.cos(shotYaw) + vXInit + vXRot;
              double vY = exitVelocity * Math.cos(shotPitch) * Math.sin(shotYaw) + vYInit + vYRot;
              double vZ = exitVelocity * Math.sin(shotPitch);

              SmartDashboard.putNumber("Testing/vX", vX);
              SmartDashboard.putNumber("Testing/vY", vY);
              SmartDashboard.putNumber("Testing/vZ", vZ);
              SmartDashboard.putNumber("Testing/hoodAngle", shotSolution.hoodAngle().in(Degrees));
              SmartDashboard.putNumber(
                  "Testing/exitVelocity", shotSolution.exitVelocity().in(MetersPerSecond));

              Pose3d turretPose =
                  robotPose.transformBy(
                      new Transform3d(TurretConstants.robotToTurret, Rotation3d.kZero));

              activeShots.add(new FuelProjectile(turretPose, vX, vY, vZ));
            })
        .withName("ShootFuel");
  }

  public Command projectileUpdater() {
    return Commands.run(
            () -> {
              activeShots.forEach(FuelProjectile::update);
              activeShots.removeIf(p -> !p.isActive());
            })
        .ignoringDisable(true)
        .withName("ProjectileUpdater");
  }
}
