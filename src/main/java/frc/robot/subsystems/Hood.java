// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Hood extends SubsystemBase {

  private TalonFX hoodMotor;
  StatusSignal<Angle> hoodPosition;

  private final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  public Hood() {
    hoodMotor = new TalonFX(HoodConstants.hoodMotorID);
    hoodMotor.getConfigurator().apply(HoodConstants.hoodConfigs);

    zeroHood();

    hoodPosition = hoodMotor.getPosition();

    hoodAngleMap.put(1.5, 1.0);
    hoodAngleMap.put(2.0, 2.0);
    hoodAngleMap.put(2.5, 3.0);
    hoodAngleMap.put(3.0, 4.0);
    hoodAngleMap.put(3.53, 5.0); // random values (Distance, ROTATIONS)
  }

  private void zeroHood() {
    hoodMotor.setPosition(0);
  }

  public double getHoodAngle() {
    return hoodPosition.getValue().in(Rotations);
  }

  public boolean atAngle(double targetDeg, double toleranceDeg) {
    return Math.abs(getHoodAngle() * 360 - targetDeg) < toleranceDeg;
  }

  public double getInterpolatedHoodAngle(double distanceMeters) {
    return hoodAngleMap.get(distanceMeters);
  }

  public double getInterpolatedHoodAngle(Pose2d poseA, Pose2d poseB) {
    double distance = poseA.getTranslation().getDistance(poseB.getTranslation());
    return hoodAngleMap.get(distance);
  }

  public double getPhysicsHoodAngle(
        double distanceMeters,
        double targetHeightMeters,
        double shooterHeightMeters,
        double exitVelocityMetersPerSec
) {

  double vSq = exitVelocityMetersPerSec * exitVelocityMetersPerSec;

  double dSq = distanceMeters * distanceMeters;

  double dH = targetHeightMeters - shooterHeightMeters;

  double g = 9.80665;
// long ass physics equation i derived :D -> dont ask idk if it works
  double numerator = (vSq * distanceMeters) + Math.sqrt((vSq * vSq * dSq) - (g * dSq*((2 * vSq * dH) + (g * dSq))));

  double denominator = g * dSq;

  double angleAboveHorizontalDeg = Math.atan(numerator/denominator); 

  return angleAboveHorizontalDeg;
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
                          TurretConstants.turretOnRobot.toTranslation2d(), Rotation2d.kZero));
          double distance =
              turretPose.getTranslation().getDistance(targetPoseSupplier.get().getTranslation());

          hoodMotor.setControl(motionMagicRequest.withPosition(getInterpolatedHoodAngle(distance)));
        })
        .withName("Aim Hood At Target");
  }

  public Command aimWithPhysics(
        Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> robotPoseSupplier,
        DoubleSupplier exitVelocitySupplier
) {
    return run(() -> {

      Pose2d turretPose =
              robotPoseSupplier
                  .get()
                  .transformBy(
                      new Transform2d(
                          TurretConstants.turretOnRobot.toTranslation2d(), Rotation2d.kZero));
          double distance =
              turretPose.getTranslation().getDistance(targetPoseSupplier.get().getTranslation());

        double angleDeg = getPhysicsHoodAngle(
                distance,
                FieldConstants.hubHeightMeters,
                TurretConstants.turretOnRobot.getZ(),
                exitVelocitySupplier.getAsDouble()
        );

        double targetRotations = angleDeg / 360.0;
        hoodMotor.setControl(motionMagicRequest.withPosition(targetRotations));

    }).withName("Aim Hood (Physics)");
}

  @Override
  public void periodic() {
    hoodPosition.refresh();
    SmartDashboard.putNumber("Hood/Hood Angle", getHoodAngle());
  }
}
