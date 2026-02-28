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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private SwerveRequest.FieldCentric fieldOriented =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withSteerRequestType(SteerRequestType.Position);

  private static final double drivekP = 2.0;
  private static final double drivekD = 0.15;
  private static final double thetakP = 8.0;
  private static final double thetakD = 0.10;

  private static final double driveMaxVelocity =
      SwerveConstants.maxTranslationalSpeed.in(MetersPerSecond);
  private static final double driveMaxAcceleration =
      SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond);
  private static final double thetaMaxVelocity =
      Units.rotationsToRadians(SwerveConstants.maxRotationalSpeed.in(RotationsPerSecond));
  private static final double thetaMaxAcceleration =
      Units.rotationsToRadians(
          SwerveConstants.maxAngularAcceleration.in(RotationsPerSecondPerSecond));

  private static final double driveTolerance = 0.02; // 0.02
  private static final double thetaTolerance = Units.degreesToRadians(1.0);

  private static final double approachVelocityScale = 0.1;
  private static final double approachDistanceThreshold = 1.0;

  private final Swerve swerve;

  private final Supplier<Pose2d> target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;
  private boolean rotateOnly;

  public DriveToPose(Swerve swerve, boolean rotateOnly, Supplier<Pose2d> target) {
    this.swerve = swerve;
    this.target = target;
    this.rotateOnly = rotateOnly;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("DriveToPose/target/XPosition", target.get().getX());
    SmartDashboard.putNumber("DriveToPose/target/YPosition", target.get().getY());
    SmartDashboard.putNumber(
        "DriveToPose/target/ThetaPosition", target.get().getRotation().getDegrees());

    Pose2d currentPose = swerve.getRobotPose();
    Translation2d linearFieldVelocity =
        new Translation2d(
            swerve.getState().Speeds.vxMetersPerSecond, swerve.getState().Speeds.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), swerve.getState().Speeds.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    driveController.setP(drivekP);
    driveController.setD(drivekD);
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    driveController.setTolerance(driveTolerance);

    thetaController.setP(thetakP);
    thetaController.setD(thetakD);
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
    thetaController.setTolerance(thetaTolerance);

    Pose2d currentPose = swerve.getRobotPose();
    Pose2d targetPose = target.get();

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    driveErrorAbs = currentDistance;

    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);

    double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);

    boolean inApproachZone = currentDistance < approachDistanceThreshold;
    if (inApproachZone) {
      // double scale = MathUtil.clamp(currentDistance / approachDistanceThreshold, 0.1, 1.0);
      // driveVelocityScalar *= scale * approachVelocityScale;
      driveVelocityScalar *= Math.max(currentDistance / approachDistanceThreshold, 0.5);
    }

    if (currentDistance < driveController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }

    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(
                    new Translation2d(driveController.getSetpoint().position, 0.0),
                    new Rotation2d()))
            .getTranslation();

    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

    if (thetaErrorAbs < thetaController.getPositionTolerance()) {
      thetaVelocity = 0.0;
    }

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
            .getTranslation();
    if (rotateOnly) {
      swerve.setControl(
          fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(thetaVelocity));
    } else {
      swerve.setControl(
          fieldOriented
              .withVelocityX(driveVelocity.getX())
              .withVelocityY(driveVelocity.getY())
              .withRotationalRate(thetaVelocity));
    }
    SmartDashboard.putBoolean("DriveToPose/Theta At Goal", thetaController.atGoal());
    SmartDashboard.putBoolean("DriveToPose/Translation At Goal", driveController.atGoal());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    running = false;
  }

  @Override
  public boolean isFinished() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }
}
