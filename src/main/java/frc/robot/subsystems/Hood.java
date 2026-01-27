// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.SOTMCalculator;

import java.util.function.Supplier;

public class Hood extends SubsystemBase {
  private TalonFX hoodMotor;
  private StatusSignal<Angle> hoodPosition;

  private SingleJointedArmSim hoodSim;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private Angle targetAngle = HoodConstants.minAngle;

  public Hood() {
    hoodMotor = new TalonFX(HoodConstants.hoodMotorID);
    hoodMotor.getConfigurator().apply(HoodConstants.hoodConfigs);

    zeroHood();

    hoodPosition = hoodMotor.getPosition();

    hoodSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            HoodConstants.hoodGearRatio,
            0.00398184688,
            0.1408176,
            HoodConstants.minAngle.in(Radians),
            HoodConstants.maxAngle.in(Radians),
            true,
            HoodConstants.minAngle.in(Radians));
  }

  private void zeroHood() {
    hoodMotor.setPosition(0);
  }

  @Logged(name = "Hood Angle")
  public Angle getHoodAngle() {
    return hoodPosition.getValue();
  }

  public boolean atAngle(Angle target, Angle tolerance) {
    return Math.abs(getHoodAngle().minus(target).in(Degrees)) < tolerance.in(Degrees);
  }

  public Angle getInterpolatedHoodAngle(double distanceMeters) {
    return SOTMCalculator.hoodAngleMap.get(distanceMeters).getMeasure();
  }

  public Angle getInterpolatedHoodAngle(Pose2d poseA, Pose2d poseB) {
    double distance = poseA.getTranslation().getDistance(poseB.getTranslation());
    return SOTMCalculator.hoodAngleMap.get(distance).getMeasure();  
  }

  public void stopHood() {
    hoodMotor.stopMotor();
  }

  public void moveHood(double speed) {
    hoodMotor.set(speed);
  }

  public void setTargetAngle(Angle targetAngle) {
    this.targetAngle = targetAngle;
    hoodMotor.setControl(motionMagicRequest.withPosition(targetAngle.in(Rotations)));
  }

  public Angle getTargetAngle() {
    return targetAngle;
  }

  public Command stop() {
    return runOnce(() -> hoodMotor.stopMotor()).withName("Stop Hood");
  }

  public Command moveToAngle(Angle targetPose) {
    return run(() -> {
          hoodMotor.setControl(motionMagicRequest.withPosition(targetPose.in(Rotations)));
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

  @Override
  public void periodic() {
    hoodPosition.refresh();
    SmartDashboard.putNumber("Hood/Hood Angle", getHoodAngle().in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimState hoodSimState = hoodMotor.getSimState();
    hoodSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    hoodSim.setInputVoltage(hoodSimState.getMotorVoltage());

    hoodSim.update(0.020);

    hoodSimState.setRawRotorPosition(
        Units.radiansToRotations(hoodSim.getAngleRads()) * HoodConstants.hoodGearRatio);

    hoodSimState.setRotorVelocity(
        Units.radiansToRotations(hoodSim.getVelocityRadPerSec()) * HoodConstants.hoodGearRatio);
  }
}
