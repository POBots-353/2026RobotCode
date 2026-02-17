// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /*Objects */
  private final TalonFX armMainMotor;
  private final TalonFX armFollowerMotor;
  private final TalonFX intakeMotor;
  private final DigitalInput armMagnetSensor;

  private boolean wasDeployedLastLoop;

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);

  private StatusSignal<Angle> armMainPosition;

  private StatusSignal<Angle> armFollowerPosition;

  private final StatusSignal<Current> armCurrent;
  private final StatusSignal<AngularVelocity> armVelocity;

  /*MotionMagic*/
  public Intake() {
    intakeMotor = new TalonFX(IntakeConstants.intakeID);
    armMainMotor = new TalonFX(IntakeConstants.armMainID);
    armFollowerMotor = new TalonFX(IntakeConstants.armFollowerID);

    armMagnetSensor = new DigitalInput(IntakeConstants.armMagnetID);

    armMainMotor.getConfigurator().apply(IntakeConstants.armConfigs);
    armFollowerMotor.getConfigurator().apply(IntakeConstants.armConfigs);

    armCurrent = armMainMotor.getStatorCurrent();
    armVelocity = armMainMotor.getVelocity();

    armMainPosition = armMainMotor.getPosition();
    armFollowerPosition = armFollowerMotor.getPosition();
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public Command stop() {
    return runOnce(this::stopIntake).withName("Stop Intake");
  }

  public Command intakeToPosition(boolean downPosition) {
    return run(() -> {
          if (downPosition) {
            armMainMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.downPosition));
            armFollowerMotor.setControl(
                motionMagicRequest.withPosition(IntakeConstants.downPosition));

          } else {
            armMainMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.upPosition));
            armFollowerMotor.setControl(
                motionMagicRequest.withPosition(IntakeConstants.downPosition));
          }
        })
        .withName("Intake To Position");
  }

  public Command intakeSequence(boolean intakeDown) {
    return run(() -> {
          if (intakeDown) {
            armMainMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.downPosition));
            armFollowerMotor.setControl(
                motionMagicRequest.withPosition(IntakeConstants.downPosition));
            setIntakeSpeed(IntakeConstants.intakeSpeed);
          } else {
            armMainMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.upPosition));
            armFollowerMotor.setControl(
                motionMagicRequest.withPosition(IntakeConstants.upPosition));
            stopIntake();
          }
        })
        .withName("Intake working");
  }

  public boolean isIntakeDeployed() {
    // return !armMagnetSensor.get();
    return true; // FOR TESTING IN SIM
  }

  public Angle getArmAngle() {
    return armMainPosition.getValue();
  }

  public void setZero() {
    armMainMotor.setPosition(0);
    armFollowerMotor.setPosition(0);
  }

  public Command zeroArmCommand() {
    return Commands.run(
            () -> {
              armMainMotor.set(IntakeConstants.armZeroSpeed);
              armFollowerMotor.set(IntakeConstants.armZeroSpeed);

              armCurrent.refresh();
              armVelocity.refresh();
            })
        .until(
            () ->
                armCurrent.getValue().in(Amps) > IntakeConstants.armStallCurrent
                    && Math.abs(armVelocity.getValue().in(RotationsPerSecond))
                        < IntakeConstants.armStallVelocity)
        .andThen(
            () -> {
              armMainMotor.stopMotor();
              armFollowerMotor.stopMotor();
              setZero();
            })
        .withName("Zero Arm Command");
  }

  @Override
  public void periodic() {
    if (!isIntakeDeployed() && wasDeployedLastLoop) {
      armMainMotor.setPosition(IntakeConstants.minPosition);
      armFollowerMotor.setPosition(IntakeConstants.minPosition);
    }
    wasDeployedLastLoop = isIntakeDeployed();

    armMainPosition.refresh();
    armFollowerPosition.refresh();

    SmartDashboard.putNumber("Intake speed", intakeMotor.get());
    SmartDashboard.putNumber("Intake Arm Position", armMainPosition.getValue().in(Rotations));
    SmartDashboard.putBoolean("Intake Arm Deployed", isIntakeDeployed());
  }
}
