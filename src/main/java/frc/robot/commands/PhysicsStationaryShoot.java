// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.util.HoodShotCalculator;
import frc.robot.util.HoodShotCalculator.ShotSolution;
import frc.robot.util.RobotVisualization;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PhysicsStationaryShoot extends Command {
  private final Shooter shooter;
  private final Hood hood;
  private final Supplier<Pose2d> robotPose;
  private final Supplier<Pose2d> targetPose;
  private final Supplier<Distance> targetHeight;

  public PhysicsStationaryShoot(
      Shooter shooter,
      Hood hood,
      RobotVisualization robotVisualization,
      Supplier<Pose2d> robotPose,
      Supplier<Pose2d> targetPose,
      Supplier<Distance> targetHeight) {
    this.shooter = shooter;
    this.hood = hood;
    this.robotPose = robotPose;
    this.targetPose = targetPose;
    this.targetHeight = targetHeight;
    addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d turretPose =
        robotPose
            .get()
            .transformBy(
                new Transform2d(TurretConstants.robotToTurret.toTranslation2d(), Rotation2d.kZero));

    Distance distance =
        Meters.of(turretPose.getTranslation().getDistance(targetPose.get().getTranslation()));
    ShotSolution solution =
        HoodShotCalculator.solveShot(distance, targetHeight.get(), shooter.getExitVelocity());
    shooter.logSolution(solution);
    hood.setTargetAngle(solution.hoodAngle());

    SmartDashboard.putNumber("TESTING/Distance", distance.in(Meters));
    SmartDashboard.putNumber("TESTING/Angle", solution.hoodAngle().in(Degrees));
    SmartDashboard.putNumber("TESTING/Velocity", solution.exitVelocity().in(MetersPerSecond));
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
