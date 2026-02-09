// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class GuidedTeleopSwerve extends Command {
  private final Swerve swerve;

  private final SwerveRequest.FieldCentric fieldOriented =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withSteerRequestType(SteerRequestType.Position);

  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier rotationSupplier;

  private final Supplier<LinearVelocity> maxTranslationalSpeedSupplier;
  private final double maxRotSpeedRads =
      Units.rotationsToRadians(SwerveConstants.maxRotationalSpeed.in(RotationsPerSecond));

  private final BooleanSupplier manualOverrideSupplier;

  private SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));
  private SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));
  private SlewRateLimiter rotationRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxAngularAcceleration.in(RotationsPerSecondPerSecond));

  private double flipFactor = 1;

  private final Trigger inTrenchZoneTrigger;

  private final Trigger inBumpZoneTrigger;

  private final PIDController trenchYController = new PIDController(6, 0, 0);
  private final PIDController rotationController = new PIDController(8, 0, 0);

  private DriveMode currentDriveMode = DriveMode.NormalDrive;

  public GuidedTeleopSwerve(
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      Supplier<LinearVelocity> maxTranslationalSpeedSupplier,
      BooleanSupplier manualOverrideSupplier,
      Swerve swerve) {
    this.swerve = swerve;
    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;
    this.maxTranslationalSpeedSupplier = maxTranslationalSpeedSupplier;
    this.manualOverrideSupplier = manualOverrideSupplier;

    inTrenchZoneTrigger = new Trigger(swerve::inTrenchZone).debounce(0.1);

    inBumpZoneTrigger = new Trigger(swerve::inBumpZone).debounce(0.1);

    inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TrenchDrive));
    inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BumpDrive));
    inTrenchZoneTrigger.or(inBumpZoneTrigger).onFalse(updateDriveMode(DriveMode.NormalDrive));

    addRequirements(swerve);
  }

  private Command updateDriveMode(DriveMode driveMode) {
    return Commands.runOnce(() -> currentDriveMode = driveMode);
  }

  private double getDotProduct() {
    Pose2d robotPose = swerve.getRobotPose();
    Pose2d targetPose =
        inTrenchZoneTrigger.getAsBoolean()
            ? swerve.getClosestTrenchPose()
            : swerve.getClosestBumpPose();

    Vector<N2> commandedVelocity = new Vector<>(Nat.N2());
    commandedVelocity.set(0, 0, getForwardSpeed());
    commandedVelocity.set(1, 0, getStrafeSpeed());

    Vector<N2> toTarget = new Vector<>(Nat.N2());
    toTarget.set(0, 0, (flipFactor) * (targetPose.getX() - robotPose.getX()));
    toTarget.set(1, 0, (flipFactor) * (targetPose.getY() - robotPose.getY()));

    double dotProduct = commandedVelocity.unit().dot(toTarget.unit());

    return dotProduct;
  }

  private double getMaxTranslationalSpeed() {
    return maxTranslationalSpeedSupplier.get().in(MetersPerSecond);
  }

  private double getForwardSpeed() {
    return forwardRateLimiter.calculate(
        -forwardSupplier.getAsDouble() * getMaxTranslationalSpeed());
  }

  private double getStrafeSpeed() {
    return strafeRateLimiter.calculate(-strafeSupplier.getAsDouble() * getMaxTranslationalSpeed());
  }

  private double getRotationSpeed() {
    double joystickRotation = -rotationSupplier.getAsDouble();
    joystickRotation =
        Math.copySign(
            joystickRotation * joystickRotation,
            joystickRotation); // square for more precise rotation control
    double rotationSpeed =
        joystickRotation * SwerveConstants.maxRotationalSpeed.in(RotationsPerSecond);

    if (Math.abs(rotationSpeed) <= (Units.degreesToRotations(1.5))) {
      rotationRateLimiter.reset(0);
      return 0;
    }

    return rotationRateLimiter.calculate(rotationSpeed);
  }

  private void driveNormal(double forward, double strafe, double rot) {
    swerve.setControl(
        fieldOriented
            .withVelocityX(forward)
            .withVelocityY(strafe)
            .withRotationalRate(Units.rotationsToRadians(rot)));
  }

  private void driveTrench(double forward) {
    double yVel =
        trenchYController.calculate(swerve.getRobotPose().getY(), swerve.getTrenchY().in(Meters));

    double rotStraightSpeed =
        rotationController.calculate(
            swerve.getRotation().getRadians(), swerve.getTrenchLockAngle().getRadians());

    rotStraightSpeed = MathUtil.clamp(rotStraightSpeed, -maxRotSpeedRads, maxRotSpeedRads);

    yVel = MathUtil.clamp(yVel, -getMaxTranslationalSpeed(), getMaxTranslationalSpeed());

    swerve.setControl(
        fieldOriented
            .withVelocityX(forward)
            .withVelocityY(flipFactor * yVel)
            .withRotationalRate(rotStraightSpeed));
  }

  private void driveBump(double forward, double strafe) {
    double rotDiagonalSpeed =
        rotationController.calculate(
            swerve.getRotation().getRadians(), swerve.getBumpLockAngle().getRadians());

    rotDiagonalSpeed = MathUtil.clamp(rotDiagonalSpeed, -maxRotSpeedRads, maxRotSpeedRads);

    swerve.setControl(
        fieldOriented
            .withVelocityX(forward)
            .withVelocityY(strafe)
            .withRotationalRate(rotDiagonalSpeed));
  }

  @Override
  public void initialize() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    flipFactor = AllianceUtil.isRedAlliance() ? -1 : 1;
  }

  @Override
  public void execute() {
    double forwardSpeed = getForwardSpeed();
    double strafeSpeed = getStrafeSpeed();
    double rotSpeed = getRotationSpeed();

    double dotProduct = currentDriveMode == DriveMode.NormalDrive ? 0 : getDotProduct();

    DriveMode effectiveDriveMode =
        (manualOverrideSupplier.getAsBoolean()
                || (dotProduct < 0.05)) // only lock if in trying to drive in that direction
            ? DriveMode.NormalDrive
            : currentDriveMode;

    switch (effectiveDriveMode) {
      case NormalDrive -> driveNormal(forwardSpeed, strafeSpeed, rotSpeed);
      case TrenchDrive -> driveTrench(forwardSpeed);
      case BumpDrive -> driveBump(forwardSpeed, strafeSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    forwardRateLimiter.reset(0);
    strafeRateLimiter.reset(0);
    rotationRateLimiter.reset(0);
    swerve.setControl(fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private enum DriveMode {
    NormalDrive,
    TrenchDrive,
    BumpDrive
  }
}
