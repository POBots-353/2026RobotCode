package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
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

  @Logged(name = "Fuel Stored")
  public int getFuelStored() {
    return fuelStored;
  }

  public Command shootFuelGatherData() {
    return Commands.runOnce(
            () -> {
              Pose3d robotPose = new Pose3d(swerve.getRobotPose());

              double shotAngle =
                  SmartDashboard.getNumber(
                      "GatherData/Manual Shot Angle", HoodConstants.minAngle.in(Degrees));
              double shotSpeed = SmartDashboard.getNumber("GatherData/Manual Shot Speed", 0);

              Pose3d turretPose =
                  robotPose.transformBy(
                      new Transform3d(TurretConstants.robotToTurret, Rotation3d.kZero));

              double turretDistance =
                  turretPose
                      .toPose2d()
                      .getTranslation()
                      .getDistance(AllianceUtil.getHubPose().getTranslation());

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
    if (RobotBase.isReal()) return;
    if (fuelStored < 1) return;

    fuelStored--;

    // FuelSim.getInstance()
    //     .launchFuel(
    //         shootingParameters.shooterSpeed(),
    //         Radians.of(Math.PI / 2).minus(shootingParameters.hoodAngle()),
    //
    // swerve.getRobotPose().getRotation().getMeasure().plus(shootingParameters.turretAngle()),
    //         TurretConstants.robotToTurret);

    // once turret is tuned better
    FuelSim.getInstance()
        .launchFuel(
            shooter.getGoalSpeed(),
            Radians.of(Math.PI / 2).minus(hood.getHoodAngle()),
            swerve.getRobotPose().getRotation().getMeasure().plus(turret.getTurretAngle()),
            TurretConstants.robotToTurret);
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
