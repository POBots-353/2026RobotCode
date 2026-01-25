// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;
import java.util.function.Supplier;

public class Hood extends SubsystemBase {
  private TalonFX hoodMotor;
  private StatusSignal<Angle> hoodPosition;
  private Turret turret;

  private final InterpolatingDoubleTreeMap hoodAngleMap = HoodConstants.hoodAngleMap;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  public Hood(Turret turret) {
    hoodMotor = new TalonFX(HoodConstants.hoodMotorID);
    hoodMotor.getConfigurator().apply(HoodConstants.hoodConfigs);

    this.turret = turret;

    zeroHood();

    hoodPosition = hoodMotor.getPosition();
  }

  private void zeroHood() {
    hoodMotor.setPosition(0);
  }

  @Logged(name = "Hood Angle")
  public Angle getHoodAngle() {
    return hoodPosition.getValue();
  }

  @Logged(name = "3D Pose Hood")
  public Pose3d getHoodPose3d() {
    return turret
        .getTurretPose3d()
        .transformBy(
            new Transform3d(
                new Translation3d(0.121, 0.0, 0.072),
                new Rotation3d(0.0,Timer.getFPGATimestamp() % 360, 0.0)));
  }

  public boolean atAngle(Angle target, Angle tolerance) {
    return Math.abs(getHoodAngle().minus(target).in(Degrees)) < tolerance.in(Degrees);
  }

  public double getInterpolatedHoodAngle(double distanceMeters) {
    return hoodAngleMap.get(distanceMeters);
  }

  public double getInterpolatedHoodAngle(Pose2d poseA, Pose2d poseB) {
    double distance = poseA.getTranslation().getDistance(poseB.getTranslation());
    return hoodAngleMap.get(distance);
  }

  public Angle getPhysicsHoodAngle(
      Distance distance,
      Distance targetHeight,
      Distance shooterHeight,
      LinearVelocity exitVelocity) {
    double distanceMeters = distance.in(Meters);
    double targetHeightMeters = targetHeight.in(Meters);
    double shooterHeightMeters = shooterHeight.in(Meters);
    double exitVelocityMetersPerSec = exitVelocity.in(MetersPerSecond);

    double vSq = exitVelocityMetersPerSec * exitVelocityMetersPerSec;

    double dSq = distanceMeters * distanceMeters;

    double dH = targetHeightMeters - shooterHeightMeters;

    double g = 9.80665;
    // long physics equation i derived :D -> dont ask idk if it works
    double numerator =
        (vSq * distanceMeters)
            + Math.sqrt((vSq * vSq * dSq) - (g * dSq * ((2 * vSq * dH) + (g * dSq))));

    double denominator = g * dSq;

    return Radians.of(Math.atan2(numerator, denominator));
  }

  public void stopHood() {
    hoodMotor.stopMotor();
  }

  public void setTargetAngle(Angle targetAngle) {
    hoodMotor.setControl(motionMagicRequest.withPosition(targetAngle.in(Rotations)));
  }

  public Command stop() {
    return runOnce(() -> hoodMotor.stopMotor()).withName("Stop Hood");
  }

  public Command moveToAngle(double targetPoseRot) {
    return run(() -> {
          hoodMotor.setControl(motionMagicRequest.withPosition(targetPoseRot));
        })
        .withName("Move Turret to Angle");
  }

  public Command aimForTarget(
      Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> robotPoseSupplier) {
    return run(() -> {
          Pose2d turretPose =
              robotPoseSupplier
                  .get()
                  .transformBy(
                      new Transform2d(
                          TurretConstants.robotToTurret.toTranslation2d(), Rotation2d.kZero));
          double distance =
              turretPose.getTranslation().getDistance(targetPoseSupplier.get().getTranslation());

          hoodMotor.setControl(motionMagicRequest.withPosition(getInterpolatedHoodAngle(distance)));
        })
        .withName("Aim Hood At Target");
  }

  public Command aimWithPhysics(
      Supplier<Pose2d> targetPoseSupplier,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<LinearVelocity> exitVelocitySupplier) {
    return run(() -> {
          Pose2d turretPose =
              robotPoseSupplier
                  .get()
                  .transformBy(
                      new Transform2d(
                          TurretConstants.robotToTurret.toTranslation2d(), Rotation2d.kZero));
          Distance distance =
              Meters.of(
                  turretPose
                      .getTranslation()
                      .getDistance(targetPoseSupplier.get().getTranslation()));

          Angle angle =
              getPhysicsHoodAngle(
                  distance,
                  FieldConstants.hubHeight,
                  TurretConstants.robotToTurret.getMeasureZ(),
                  exitVelocitySupplier.get());

          hoodMotor.setControl(motionMagicRequest.withPosition(angle));
        })
        .withName("Aim Hood (Physics)");
  }

  @Override
  public void periodic() {
    hoodPosition.refresh();
    SmartDashboard.putNumber("Hood/Hood Angle", getHoodAngle().in(Degrees));
  }
}
