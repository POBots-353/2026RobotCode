// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShootingParameters;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOnTheMove extends Command {
  private final Shooter shooter;
  private final Hood hood;
  private final Turret turret;
  private final Swerve swerve;

  private Debouncer hoodSetPointDebouncer = new Debouncer(0.1);
  private Debouncer turretSetPointDebouncer = new Debouncer(0.1);
  private Debouncer shooterDebouncer = new Debouncer(0.1);

  private double turretTolerance = 3.53; // deg
  private double hoodTolerance = 3.53; // deg
  private boolean shooterAtSetPoint = true;

  private LinearFilter accelXFilter = LinearFilter.movingAverage(2);
  private LinearFilter accelYFilter = LinearFilter.movingAverage(2);

  private ChassisSpeeds previousSpeeds = new ChassisSpeeds();

  public ShootOnTheMove(Swerve swerve, Turret turret, Hood hood, Shooter shooter) {
    this.swerve = swerve;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    addRequirements(hood, turret, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hoodSetPointDebouncer.calculate(false);
    turretSetPointDebouncer.calculate(false);

    accelXFilter.reset();
    accelYFilter.reset();

    previousSpeeds = swerve.getChassisSpeeds();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ChassisSpeeds fieldSpeeds = swerve.getChassisSpeeds();

    ChassisSpeeds fieldAcceleration = fieldSpeeds.minus(previousSpeeds).div(0.020);

    double fieldAccelX = accelXFilter.calculate(fieldAcceleration.vxMetersPerSecond);
    double fieldAccelY = accelYFilter.calculate(fieldAcceleration.vyMetersPerSecond);

    ShootingParameters shootingParameters =
        ShotCalculator.getParameters(swerve, turret, fieldAccelX, fieldAccelY);

    turret.setTargetAngle(shootingParameters.turretAngle());

    hood.setTargetAngle(shootingParameters.hoodAngle());

    // shooter.setSpeed(shootingParametes.shooterSpeed());

    double turretErrorDeg =
        turret.getTurretAngle().in(Degrees) - shootingParameters.turretAngle().in(Degrees);
    double hoodErrorDeg =
        hood.getHoodAngle().in(Degrees) - shootingParameters.hoodAngle().in(Degrees);

    if (turretSetPointDebouncer.calculate(Math.abs(turretErrorDeg) <= turretTolerance)
        && hoodSetPointDebouncer.calculate(Math.abs(hoodErrorDeg) <= hoodTolerance)
        && shooterDebouncer.calculate(shooterAtSetPoint)) {
      // indexer.feed();
    } else {
      // indexer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooter.stop();
    // indexer.stop();
    turret.stopTurret();
    hood.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
