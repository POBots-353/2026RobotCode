// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceUtil;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoClimb extends SequentialCommandGroup {
  /** Creates a new AutoClimb. */
  public AutoClimb(
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      Supplier<LinearVelocity> maxTranslationalSpeed,
      Swerve swerve,
      Climber climber,
      boolean leftAlign) {
    Supplier<Pose2d> targetClimbPosition =
        () -> {
          List<Pose2d> blueClimbingPoses =
              List.of(ClimbConstants.leftClimbPose, ClimbConstants.rightClimbPose);

          List<Pose2d> climbingPoses =
              blueClimbingPoses.stream().map(AllianceUtil::flipPose).toList();

          return leftAlign ? climbingPoses.get(0) : climbingPoses.get(1);
        };

    addCommands(
        new RotateToPose(
                forwardSupplier, strafeSupplier, targetClimbPosition, maxTranslationalSpeed, swerve)
            .withDeadline(climber.climberUpCommand()),
        new DriveToPose(swerve, false, targetClimbPosition),
        climber.climberDownCommand());
  }
}
