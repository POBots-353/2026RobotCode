// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.util.HoodShotCalculator;
import frc.robot.util.HoodShotCalculator.ShotSolution;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PhysicsStationaryShoot extends Command {
  private final Shooter shooter;
  private final Hood hood;
  private final Supplier<Pose2d> robotPose;
  private final Supplier<Pose2d> targetPose;

  public PhysicsStationaryShoot(
      Shooter shooter, Hood hood, Supplier<Pose2d> robotPose, Supplier<Pose2d> targetPose) {
    this.shooter = shooter;
    this.hood = hood;
    this.robotPose = robotPose;
    this.targetPose = targetPose;
    addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Distance distance =
        Meters.of(robotPose.get().getTranslation().getDistance(targetPose.get().getTranslation()));
    ShotSolution solution =
        HoodShotCalculator.solveShot(distance, FieldConstants.hubHeight, shooter.getExitVelocity());
    shooter.logSolution(solution);
    hood.setTargetAngle(solution.hoodAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
