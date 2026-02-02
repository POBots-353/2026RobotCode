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
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.util.HoodShotCalculator.ShotSolution;
import frc.robot.util.SOTMCalculator.ShootingParameters;
import java.util.ArrayList;
import java.util.List;

public class RobotVisualization {
  private Turret turret;
  private Hood hood;
  private Swerve swerve;
  private Shooter shooter;

  private static int fuelStored = 8;

  private static List<FuelProjectile> activeShots = new ArrayList<>();

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
    return activeShots.stream().map(p -> p.getPose()).toArray(p -> new Pose3d[p]);
  }

  @Logged(name = "FuelStored")
  public int getFuelStored() {
    return fuelStored;
  }

  public void basicShootFuel(ShootingParameters shootingParameters) {

    Pose3d robotPose = new Pose3d(swerve.getRobotPose());

    ChassisSpeeds fieldSpeeds = swerve.getFieldSpeeds();

    Rotation2d turretYaw = Rotation2d.fromDegrees(shootingParameters.turretAngle().in(Degrees));
    Rotation2d hoodPitch = Rotation2d.fromDegrees(shootingParameters.hoodAngle().in(Degrees));

    double shotYaw = turretYaw.plus(robotPose.toPose2d().getRotation()).getRadians();
    double shotPitch = Units.degreesToRadians(90) - hoodPitch.getRadians();

    double exitVelocity = shootingParameters.shooterSpeed();

    double vXInit = fieldSpeeds.vxMetersPerSecond;
    double vYInit = fieldSpeeds.vyMetersPerSecond;

    double vXRot =
        -fieldSpeeds.omegaRadiansPerSecond * TurretConstants.robotToTurret.toTranslation2d().getY();
    double vYRot =
        fieldSpeeds.omegaRadiansPerSecond * TurretConstants.robotToTurret.toTranslation2d().getX();

    double vX = exitVelocity * Math.cos(shotPitch) * Math.cos(shotYaw) + vXInit + vXRot;
    double vY = exitVelocity * Math.cos(shotPitch) * Math.sin(shotYaw) + vYInit + vYRot;
    double vZ = exitVelocity * Math.sin(shotPitch);

    Pose3d turretPose =
        robotPose.transformBy(new Transform3d(TurretConstants.robotToTurret, Rotation3d.kZero));

    activeShots.add(new FuelProjectile(turretPose, vX, vY, vZ));
  }

  public Command shootFuelGatherData() {
    return Commands.runOnce(
            () -> {
              Pose3d robotPose = new Pose3d(swerve.getRobotPose());

              ChassisSpeeds fieldSpeeds = swerve.getFieldSpeeds();

              double shotAngle =
                  SmartDashboard.getNumber(
                      "GatherData/Manual Shot Angle", HoodConstants.minAngle.in(Degrees));
              double shotSpeed = SmartDashboard.getNumber("GatherData/Manual Shot Speed", 0);

              ShotSolution shotSolution =
                  new ShotSolution(Degrees.of(shotAngle), MetersPerSecond.of(shotSpeed));
              Pose3d turretPose =
                  robotPose.transformBy(
                      new Transform3d(TurretConstants.robotToTurret, Rotation3d.kZero));

              double turretDistance =
                  turretPose
                      .toPose2d()
                      .getTranslation()
                      .getDistance(AllianceUtil.getHubPose().getTranslation());

              Rotation2d turretYaw = Rotation2d.fromDegrees(turret.getTurretAngle().in(Degrees));
              Rotation2d hoodPitch = Rotation2d.fromDegrees(shotSolution.hoodAngle().in(Degrees));

              double shotYaw = turretYaw.plus(robotPose.toPose2d().getRotation()).getRadians();
              double shotPitch = Units.degreesToRadians(90) - hoodPitch.getRadians();

              double exitVelocity = shotSolution.exitVelocity().in(MetersPerSecond);

              double vXInit = fieldSpeeds.vxMetersPerSecond;
              double vYInit = fieldSpeeds.vyMetersPerSecond;

              double vXRot =
                  -fieldSpeeds.omegaRadiansPerSecond
                      * TurretConstants.robotToTurret.toTranslation2d().getY();
              double vYRot =
                  fieldSpeeds.omegaRadiansPerSecond
                      * TurretConstants.robotToTurret.toTranslation2d().getX();

              double vX = exitVelocity * Math.cos(shotPitch) * Math.cos(shotYaw) + vXInit + vXRot;
              double vY = exitVelocity * Math.cos(shotPitch) * Math.sin(shotYaw) + vYInit + vYRot;
              double vZ = exitVelocity * Math.sin(shotPitch);

              activeShots.add(new FuelProjectile(turretPose, vX, vY, vZ));

              SmartDashboard.putNumber("GatherData/Turret Distance", turretDistance);
              SmartDashboard.putNumber("GatherData/Shot Angle", shotAngle);
              SmartDashboard.putNumber("GatherData/Shot Speed", shotSpeed);

              if (activeShots.size() > 0) {
                double timeOfFlight =
                    activeShots.get(activeShots.size() - 1).getTOF(FieldConstants.mainHubHeight);
                SmartDashboard.putNumber("GatherData/TOF", timeOfFlight);
              }
            })
        .withName("ShootFuelGatherData");
  }

  public void shootFuel(ShootingParameters shootingParameters) {

    if (fuelStored < 1) return;

    fuelStored--;

    Pose3d robotPose = new Pose3d(swerve.getRobotPose());

    ChassisSpeeds fieldSpeeds = swerve.getFieldSpeeds();

    Pose3d turretPose =
        robotPose.transformBy(new Transform3d(TurretConstants.robotToTurret, Rotation3d.kZero));

    double shotYaw = shootingParameters.turretAngle().in(Radians);
    double shotPitch = Radians.of(Math.PI / 2).minus(shootingParameters.hoodAngle()).in(Radians);
    double exitVelocity = shootingParameters.shooterSpeed();

    double fieldYaw = robotPose.getRotation().toRotation2d().getRadians() + shotYaw;

    double vX =
        exitVelocity * Math.cos(shotPitch) * Math.cos(fieldYaw) + fieldSpeeds.vxMetersPerSecond;

    double vY =
        exitVelocity * Math.cos(shotPitch) * Math.sin(fieldYaw) + fieldSpeeds.vyMetersPerSecond;

    double vZ = exitVelocity * Math.sin(shotPitch);

    FuelSim.getInstance().spawnFuel(turretPose.getTranslation(), new Translation3d(vX, vY, vZ));
  }

  public static void projectileUpdater() {
    activeShots.forEach(FuelProjectile::update);
    activeShots.removeIf(p -> !p.isActive());
  }

  public boolean canSimIntake() {
    return fuelStored < SimConstants.maxCapacity;
  }

  public void simIntakeFuel() {
    fuelStored++;
  }

  public boolean isEmpty() {
    return fuelStored < 1;
  }

  public void addStartingFuel() {
    fuelStored = 8;
  }
}
