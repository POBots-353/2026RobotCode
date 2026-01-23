// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToFuel extends Command {
  private Rotation2d desiredRotation = Rotation2d.fromDegrees(5.1);

  private SwerveRequest.RobotCentric robotOriented = new SwerveRequest.RobotCentric();

  private SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));
  private SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));

  private PIDController turnToAlgaeController =
      new PIDController(SwerveConstants.headingP, 0, SwerveConstants.headingD);

  Swerve swerve;

  /** Creates a new MoveToFuel. */
  public MoveToFuel(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;

    addRequirements(swerve);
  }

  private void updateDesiredRotation() {
    desiredRotation = swerve.desiredFuelRotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateDesiredRotation();
    double turningSpeed =
        turnToAlgaeController.calculate(
            swerve.stateCache.Pose.getRotation().getRadians(), desiredRotation.getRadians());
    double strafeSpeed = 0.2;
    double forwardSpeed = 0.5;

    forwardSpeed = forwardRateLimiter.calculate(forwardSpeed);
    strafeSpeed = strafeRateLimiter.calculate(strafeSpeed);
    if (Math.hypot(forwardSpeed, strafeSpeed) <= Units.inchesToMeters(0.5)) {
      forwardSpeed = 0;
      strafeSpeed = 0;
    }

    swerve.setControl(
        robotOriented
            .withVelocityX(forwardSpeed)
            .withVelocityY(strafeSpeed)
            .withRotationalRate(turningSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    forwardRateLimiter.reset(0);
    strafeRateLimiter.reset(0);
    turnToAlgaeController.reset();
    swerve.setControl(robotOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
