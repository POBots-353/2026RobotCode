// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.SOTMConstants;
import frc.robot.Constants.TurretConstants;
import java.util.function.Supplier;

public class Hood extends SubsystemBase {
  private TalonFX hoodMotor;
  private StatusSignal<Angle> hoodPosition;
  private final StatusSignal<Current> hoodCurrent;
  private final StatusSignal<AngularVelocity> hoodVelocity;

  private SingleJointedArmSim hoodSim;

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);

  private Angle targetAngle = HoodConstants.minAngle;

  public Hood() {
    hoodMotor = new TalonFX(HoodConstants.hoodMotorID);
    hoodMotor.getConfigurator().apply(HoodConstants.hoodConfigs);

    zeroHood();

    hoodPosition = hoodMotor.getPosition();
    hoodCurrent = hoodMotor.getStatorCurrent();
    hoodVelocity = hoodMotor.getVelocity();

    hoodSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            HoodConstants.hoodGearRatio,
            0.00398184688,
            0.1408176,
            0,
            HoodConstants.maxAngle.minus(HoodConstants.minAngle).in(Radians),
            true,
            0);
  }

  public void zeroHood() {
    hoodMotor.setPosition(HoodConstants.minAngle);
    targetAngle = HoodConstants.minAngle;
  }

  // public Command zeroHoodCommand() {
  //   return runOnce(
  //           () -> {
  //             hoodMotor.stopMotor();
  //             hoodMotor.setPosition(HoodConstants.maxAngle);
  //           })
  //       .andThen(
  //           run(() -> {
  //                 hoodMotor.set(HoodConstants.hoodZeroSpeed);
  //               })
  //               .withTimeout(1.0))
  //       .andThen(
  //           () -> {
  //             hoodMotor.stopMotor();
  //             hoodMotor.setPosition(HoodConstants.minAngle.minus(Degrees.of(0.1)));
  //           })
  //       .withName("Zero Hood Command");
  // }

  public Command zeroHoodCommand() {
    return Commands.startRun(
            // run once
            () -> {
              hoodMotor.stopMotor();
              hoodMotor.setPosition(HoodConstants.maxAngle);
            },
            // run
            () -> {
              hoodMotor.set(HoodConstants.hoodZeroSpeed);
            })
        .until(
            () -> hoodMotor.getStatorCurrent().getValueAsDouble() > HoodConstants.hoodStallCurrent)
        .finallyDo(
            () -> {
              hoodMotor.stopMotor();
              hoodMotor.setPosition(HoodConstants.minAngle);
              hoodMotor.setControl(motionMagicRequest.withPosition(Degrees.of(25)));
            });
  }

  // public Command zeroHoodCommand() {
  //   return runOnce(() -> hoodMotor.setPosition(HoodConstants.maxAngle))
  //       .andThen(
  //           run(() -> {
  //                 hoodMotor.set(HoodConstants.hoodZeroSpeed);

  //                 hoodCurrent.refresh();
  //                 hoodVelocity.refresh();
  //               })
  //               .withTimeout(2)
  //               .until(
  //                   () ->
  //                       Math.abs(hoodCurrent.getValue().in(Amps)) >
  // HoodConstants.hoodStallCurrent)
  //               .andThen(
  //                   () -> {
  //                     hoodMotor.set(0);
  //                     zeroHood();
  //                   }))
  //       .withName("Zero Hood Command");
  // }

  @Logged(name = "Hood Angle")
  public Angle getHoodAngle() {
    return hoodPosition.getValue();
  }

  public boolean atAngle(Angle target, Angle tolerance) {
    return Math.abs(getHoodAngle().minus(target).in(Degrees)) < tolerance.in(Degrees);
  }

  public Angle getInterpolatedHoodAngle(double distanceMeters) {
    return SOTMConstants.hoodAngleMapScoring.get(distanceMeters).getMeasure();
  }

  public Angle getInterpolatedHoodAngle(Pose2d poseA, Pose2d poseB) {
    double distance = poseA.getTranslation().getDistance(poseB.getTranslation());
    return SOTMConstants.hoodAngleMapScoring.get(distance).getMeasure();
  }

  public void stopHood() {
    hoodMotor.stopMotor();
  }

  public void moveHood(double speed) {
    hoodMotor.set(speed);
  }

  public void moveToAngle(Angle targetHoodAngle) {
    targetAngle = targetHoodAngle;
    hoodMotor.setControl(motionMagicRequest.withPosition(targetHoodAngle));
  }

  public Angle getTargetAngle() {
    return targetAngle;
  }

  public Command stop() {
    return runOnce(() -> hoodMotor.stopMotor()).withName("Stop Hood");
  }

  public Command moveHoodCommand(boolean upMovement) {
    int flipFactor = upMovement ? 1 : -1;
    return run(() -> moveHood(flipFactor * HoodConstants.slowHoodSpeed));
  }

  public Command moveToAngleCommand(Angle targetPose) {
    return run(() -> {
          hoodMotor.setControl(motionMagicRequest.withPosition(targetPose));
        })
        .withName("Move Hood to Angle");
  }

  public Command maxPosition() {
    return runOnce(() -> hoodMotor.setPosition(HoodConstants.maxAngle));
  }

  public Command holdPosition() {
    return startRun(
            () -> {
              motionMagicRequest.Position = hoodPosition.getValueAsDouble();
              hoodMotor.setControl(motionMagicRequest);
            },
            () -> {
              hoodMotor.setControl(motionMagicRequest);
            })
        .withName("Hood Hold");
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
    hoodCurrent.refresh();

    SmartDashboard.putNumber("Hood/Current", hoodCurrent.getValue().in(Amps));

    // hoodMotor.setControl(motionMagicRequest.withPosition(targetAngle)); // remove this

    if (SmartDashboard.getBoolean("Data Point Mode", false)) {
      hoodMotor.setControl(
          motionMagicRequest.withPosition(
              Degrees.of(
                  SmartDashboard.getNumber(
                      "Dynamic Hood Angle", HoodConstants.minAngle.in(Degrees))))); // add this
    }

    SmartDashboard.putNumber("Hood/targetAngle", targetAngle.in(Degrees));

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
