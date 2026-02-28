// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RotateToPose extends Command {
  /** Creates a new TeleopSwerve. */
  private final Swerve swerve;

  private SwerveRequest.FieldCentric fieldOriented =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withSteerRequestType(SteerRequestType.Position);

  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier strafeSupplier;

  private final Supplier<LinearVelocity> maxTranslationalSpeedSupplier;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          8.0,
          0.0,
          0.1,
          new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration),
          SimConstants.loopPeriodSecs);

  private static final double thetaTolerance = Units.degreesToRadians(1.0);
  private static final double thetaMaxVelocity =
      Units.rotationsToRadians(SwerveConstants.maxRotationalSpeed.in(RotationsPerSecond));
  private static final double thetaMaxAcceleration =
      Units.rotationsToRadians(
          SwerveConstants.maxAngularAcceleration.in(RotationsPerSecondPerSecond));

  private Supplier<Pose2d> targetPoseSupplier;

  private SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));
  private SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));

  public RotateToPose(
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      Supplier<Pose2d> targetPoseSupplier,
      Supplier<LinearVelocity> maxTranslationalSpeed,
      Swerve swerve) {
    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.swerve = swerve;
    this.maxTranslationalSpeedSupplier = maxTranslationalSpeed;
    this.targetPoseSupplier = targetPoseSupplier;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset(
        swerve.getState().Pose.getRotation().getRadians(),
        swerve.getState().Speeds.omegaRadiansPerSecond);

    thetaController.setTolerance(thetaTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double maxTranslationalSpeed = maxTranslationalSpeedSupplier.get().in(MetersPerSecond);
    double forwardSpeed = -forwardSupplier.getAsDouble() * maxTranslationalSpeed;
    double strafeSpeed = -strafeSupplier.getAsDouble() * maxTranslationalSpeed;

    forwardSpeed = forwardRateLimiter.calculate(forwardSpeed);
    strafeSpeed = strafeRateLimiter.calculate(strafeSpeed);

    if (Math.hypot(forwardSpeed, strafeSpeed) <= Units.inchesToMeters(0.5)) {
      forwardSpeed = 0;
      strafeSpeed = 0;

      forwardRateLimiter.reset(0);
      strafeRateLimiter.reset(0);
    }

    Pose2d currentPose = swerve.getState().Pose;
    Pose2d targetPose = targetPoseSupplier.get();

    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    double thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

    if (thetaErrorAbs < thetaController.getPositionTolerance()) {
      thetaVelocity = 0.0;
    }

    swerve.setControl(
        fieldOriented
            .withVelocityX(forwardSpeed)
            .withVelocityY(strafeSpeed)
            .withRotationalRate(thetaVelocity));
  }
  ;

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    forwardRateLimiter.reset(0);
    strafeRateLimiter.reset(0);
    swerve.setControl(fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return thetaController.atGoal();
    return false;
  }
}
