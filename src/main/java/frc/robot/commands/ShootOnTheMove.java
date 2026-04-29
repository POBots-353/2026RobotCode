// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SimConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.util.RobotVisualization;
import frc.robot.util.SOTMCalculator;
import frc.robot.util.SOTMCalculator.ShootingParameters;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOnTheMove extends Command {
  private final Shooter shooter;
  private final Hood hood;
  private final Turret turret;
  private final Swerve swerve;
  private final Spindexer spindexer;

  private Debouncer hoodSetPointDebouncer = new Debouncer(0.02);
  private Debouncer turretSetPointDebouncer = new Debouncer(0.02);
  private Debouncer shooterDebouncer = new Debouncer(0.02);

  private double turretTolerance = 10.0; // deg
  private double hoodTolerance = 3.0; // deg
  private AngularVelocity shooterScoringTolerance = RotationsPerSecond.of(10);
  private AngularVelocity shooterFerryingTolerance = RotationsPerSecond.of(10);

  private LinearFilter accelXFilter = LinearFilter.movingAverage(2);
  private LinearFilter accelYFilter = LinearFilter.movingAverage(2);

  private double shooterOffset = 0.53;

  private ChassisSpeeds previousFieldSpeeds = new ChassisSpeeds();

  private Supplier<Pose2d> targetPoseSupplier;

  private RobotVisualization robotVisualization;

  private double startTime;
  private boolean isVisualizationFirstShot = true;
  private BooleanSupplier scoringMode;

  public ShootOnTheMove(
      Swerve swerve,
      Turret turret,
      Hood hood,
      Shooter shooter,
      Spindexer spindexer,
      Supplier<Pose2d> targetPoseSupplier,
      RobotVisualization robotVisualization,
      BooleanSupplier scoringMode) {
    this.swerve = swerve;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    this.spindexer = spindexer;
    this.targetPoseSupplier = targetPoseSupplier;
    this.robotVisualization = robotVisualization;
    this.scoringMode = scoringMode;
    addRequirements(hood, turret, shooter, spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hoodSetPointDebouncer.calculate(false);
    turretSetPointDebouncer.calculate(false);

    accelXFilter.reset();
    accelYFilter.reset();

    previousFieldSpeeds = swerve.getFieldSpeeds();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds fieldSpeeds = swerve.getFieldSpeeds();

    ChassisSpeeds fieldAcceleration = fieldSpeeds.minus(previousFieldSpeeds).div(0.020);

    previousFieldSpeeds = fieldSpeeds;

    double fieldAccelX = accelXFilter.calculate(fieldAcceleration.vxMetersPerSecond);
    double fieldAccelY = accelYFilter.calculate(fieldAcceleration.vyMetersPerSecond);

    ShootingParameters shootingParameters =
        SOTMCalculator.getParameters(
            swerve,
            turret,
            targetPoseSupplier.get(),
            fieldAccelX,
            fieldAccelY,
            fieldSpeeds,
            scoringMode.getAsBoolean());

    // Angle compensatedAngle = shootingParameters.turretAngle();

    // if ((compensatedAngle.in(Degrees) < -20) && (compensatedAngle.in(Degrees) > -100)) {
    //   double distance = swerve.getTurretToHubMeters();
    //   double angleFix = MathUtil.clamp(25.0 - (5 * distance), 0.0, 25.0);
    //   compensatedAngle = compensatedAngle.minus(Degrees.of(angleFix));
    //   // compensatedAngle =
    //   //     compensatedAngle.minus(Degrees.of(SmartDashboard.getNumber("Dynamic Turret Angle",
    //   // 0)));
    // }

    // if (DriverStation.isAutonomous()) {
    //   compensatedAngle = compensatedAngle.minus(Degrees.of(10));
    // }

    // AngularVelocity compensatedVelocity = shootingParameters.shooterSpeed();

    // double constant2 = SmartDashboard.getNumber("Dynamic SHooter Extra SPeed", 0);

    // compensatedVelocity =
    //     RotationsPerSecond.of(
    //         compensatedVelocity.in(RotationsPerSecond)
    //             + shooterOffset / 100 * Math.abs(compensatedAngle.in(Degrees)));

    // compensatedVelocity = compensatedVelocity.times(0.965);

    turret.moveToAngle(shootingParameters.turretAngle());
    hood.moveToAngle(shootingParameters.hoodAngle());
    shooter.reachGoalVelocity(shootingParameters.shooterSpeed());
    swerve.setLookAheadPose(shootingParameters.lookAheadPosition());

    double turretErrorDeg =
        turret.getTurretAngle().in(Degrees) - shootingParameters.turretAngle().in(Degrees);
    double hoodErrorDeg =
        hood.getHoodAngle().in(Degrees) - shootingParameters.hoodAngle().in(Degrees);

    if ((turretSetPointDebouncer.calculate(Math.abs(turretErrorDeg) <= turretTolerance)
            && hoodSetPointDebouncer.calculate(Math.abs(hoodErrorDeg) <= hoodTolerance)
            && shooterDebouncer.calculate(
                shooter.shooterAtSetPoint(
                    shootingParameters.shooterSpeed(), shooterScoringTolerance)))
        || (!scoringMode.getAsBoolean()
            && shooter.shooterAtSetPoint(
                shootingParameters.shooterSpeed(), shooterFerryingTolerance))) {
      if (RobotBase.isSimulation()) { // if sim and ready to shoot
        if (isVisualizationFirstShot
            || ((Timer.getFPGATimestamp() - startTime) > 1 / SimConstants.fuelsPerSecond)) {
          robotVisualization.shootFuel(shootingParameters);

          startTime = Timer.getFPGATimestamp();
          isVisualizationFirstShot = false;
        }
      } else { // if real and ready to shoot
        spindexer.runBoth();
      }

    } else {
      spindexer.decide(startTime);
      spindexer.stopSpindexerMotor();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    spindexer.stopBoth();
    turret.stopTurret();
    hood.stopHood();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
