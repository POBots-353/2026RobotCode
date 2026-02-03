// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /*Objects */
  private final TalonFX armMainMotor;
  private final Follower armFollowerMotor;
  private final TalonFX intakeMotor;
  private CANcoder armEncoder;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private StatusSignal<Angle> armMainPosition;

  // private StatusSignal<Angle> armFollowerPosition;

  /*MotionMagic*/
  public Intake() {
    intakeMotor = new TalonFX(IntakeConstants.intakeID);
    armMainMotor = new TalonFX(IntakeConstants.armMainID);
    armFollowerMotor = new Follower(IntakeConstants.armMainID, MotorAlignmentValue.Opposed);
    // armFollowerMotor = new TalonFX(IntakeConstants.armFollowerID);

    armEncoder = new CANcoder(IntakeConstants.armEncoderID);

    armMainMotor.getConfigurator().apply(IntakeConstants.armConfigs);
    // armFollowerMotor.getConfigurator().apply(IntakeConstants.armConfigs);

    armMainPosition = armMainMotor.getPosition();
    // armFollowerPosition = armFollowerMotor.getPosition();

    armEncoder
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withMagnetOffset(IntakeConstants.armMagnetOffset)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

    armMainMotor.setPosition(getAbsoluteArmAngle().in(Rotations));
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
          } else {
            armMainMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.upPosition));
          }
        })
        .withName("Intake working");
  }

  public Command intakeSequence(boolean intakeDown) {
    return run(() -> {
          if (intakeDown) {
            armMainMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.downPosition));
            setIntakeSpeed(IntakeConstants.intakeSpeed);
          } else {
            armMainMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.upPosition));
            stopIntake();
          }
        })
        .withName("Intake working");
  }

  public boolean isIntakeDeployed() {
    // return getAbsoluteArmAngle().in(Rotations)
    //         > IntakeConstants.armDownPositionTolerance.in(Rotations)
    //     ? true
    //     : false;
    return true; // FOR TESTING IN SIM
  }

  public Angle getAbsoluteArmAngle() {
    return armEncoder.getAbsolutePosition().getValue();
  }

  public Angle getArmAngle() {
    return armMainPosition.getValue();
  }

  @Override
  public void periodic() {
    armMainPosition.refresh();
    SmartDashboard.putNumber("Intake speed", intakeMotor.get());
    SmartDashboard.putNumber("Intake Arm Position", armMainPosition.getValue().in(Rotations));
  }
}
