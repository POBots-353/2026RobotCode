// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.intakeConstants;

public class Intake extends SubsystemBase {
  /*Objects */
  private final TalonFX armMotor;
  private final TalonFX intakeMotor;
  private final MotionMagicVoltage MM;
  private final TalonFXConfiguration config;

  /*MotionMagic*/
  public Intake() {
    intakeMotor = new TalonFX(intakeConstants.wheelID);
    armMotor = new TalonFX(intakeConstants.armID);
    MM = new MotionMagicVoltage(Constants.intakeConstants.voltagePreset);
    config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = Constants.intakeConstants.gearRatio;
    /*Random Values Currently */
    config.Slot0.kP = 1.5;
    config.Slot0.kD = .5;
    config.Slot0.kI = .5;

    config.MotionMagic.MotionMagicCruiseVelocity = 5;
    config.MotionMagic.MotionMagicAcceleration = 1;
    config.MotionMagic.MotionMagicJerk = 10;

    armMotor.getConfigurator().apply(config);
  }

  public Command intakeSequence(boolean intakeDown) {
    return run(() -> {
          if (intakeDown) {
            armMotor.setControl(MM.withPosition(intakeConstants.downPosition));
            intakeMotor.set(intakeConstants.speedUp);
          } else {
            armMotor.setControl(MM.withPosition(intakeConstants.upPosition));
            intakeMotor.set(0);
          }
        })
        .withName("Intake working");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake speed", intakeMotor.get());
    SmartDashboard.putNumber("Intake Arm Position", armMotor.getPosition().getValueAsDouble());
  }
}
