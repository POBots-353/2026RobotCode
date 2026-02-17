// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
  private TalonFX spindexerMotor;
  private TalonFX kickerMotor;
  private Debouncer currentEmptyDebouncer = new Debouncer(0.4);

  /** Creates a new Spindexer. */
  public Spindexer() {
    spindexerMotor = new TalonFX(SpindexerConstants.spindexerMotorID);
    kickerMotor = new TalonFX(SpindexerConstants.kickerMotorID);

    spindexerMotor.getConfigurator().apply(SpindexerConstants.spindexerConfigs);
    kickerMotor.getConfigurator().apply(SpindexerConstants.spindexerConfigs);
  }

  public void stopSpindexerMotor() {
    spindexerMotor.stopMotor();
  }

  public void stopKickerMotor() {
    kickerMotor.stopMotor();
  }

  public double getSpindexerSpeed() {
    return spindexerMotor.get();
  }

  public double getKickerSpeed() {
    return kickerMotor.get();
  }

  public double getCurrent() {
    return spindexerMotor.getStatorCurrent().getValueAsDouble();
  }

  public boolean currentSaysEmpty() {
    return currentEmptyDebouncer.calculate(getCurrent() < 9.0); // random number need to test
  }

  public void runBoth() {
    spindexerMotor.set(SpindexerConstants.spindexerMotorSpeed);
    kickerMotor.set(SpindexerConstants.kickerMotorSpeed);
  }

  public void stopBoth() {
    spindexerMotor.set(0);
    kickerMotor.set(0);
  }

  public Command runSpindexer() {
    return run(() -> spindexerMotor.set(SpindexerConstants.spindexerMotorSpeed));
  }

  public Command runKicker() {
    return run(() -> kickerMotor.set(SpindexerConstants.kickerMotorSpeed));
  }

  public Command runBothCommand() {
    return runKicker().alongWith(runSpindexer());
  }

  public Command stopSpindexer() {
    return runOnce(this::stopSpindexerMotor);
  }

  public Command stopKicker() {
    return runOnce(this::stopKickerMotor);
  }

  public Command stopBothCommand() {
    return stopSpindexer().alongWith(stopKicker());
  }

  public Command runUntilEmptyCommand() {
    return (runSpindexer()).until(() -> currentSaysEmpty());
  }

  public boolean isEmpty() {
    return (spindexerMotor.get() > 0.1) && currentSaysEmpty();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Spindexer Current", getCurrent());
    SmartDashboard.putBoolean("Spindexer Empty", isEmpty());
  }
}
