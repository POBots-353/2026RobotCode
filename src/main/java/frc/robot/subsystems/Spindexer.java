// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
  private TalonFX spindexerMotor;
  private TalonFX kickerMotor;
  private Debouncer currentEmptyDebouncer = new Debouncer(1.0);

  private StatusSignal<Current> kickerCurrent;
  private StatusSignal<Current> spindexerCurrent;

  /** Creates a new Spindexer. */
  public Spindexer() {
    spindexerMotor = new TalonFX(SpindexerConstants.spindexerMotorID);
    kickerMotor = new TalonFX(SpindexerConstants.kickerMotorID);

    spindexerMotor.getConfigurator().apply(SpindexerConstants.spindexerConfigs);
    kickerMotor.getConfigurator().apply(SpindexerConstants.spindexerConfigs);

    kickerCurrent = kickerMotor.getStatorCurrent();
    spindexerCurrent = spindexerMotor.getStatorCurrent();
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

  public boolean currentSaysEmpty() {
    return currentEmptyDebouncer.calculate(
        spindexerCurrent.getValue().in(Amps) < 9.0); // random number need to test
  }

  public void runBoth() {
    spindexerMotor.set(SpindexerConstants.spindexerMotorSpeed);
    kickerMotor.set(SpindexerConstants.kickerMotorSpeed);
  }

  public void stopBoth() {
    spindexerMotor.stopMotor();
    kickerMotor.stopMotor();
  }

  public Command idleReverse() {
    return run(
        () -> {
          spindexerMotor.set(SpindexerConstants.spindexerIdleSpeed);
          kickerMotor.set(SpindexerConstants.kickerIdleSpeed);
        });
  }

  public Command reverseBoth() {
    return run(
        () -> {
          spindexerMotor.set(SpindexerConstants.reverseSpindexerSpeed);
          kickerMotor.set(SpindexerConstants.reverseKickerSpeed);
        });
  }

  public Command runSpindexer() {
    return run(() -> spindexerMotor.set(SpindexerConstants.spindexerMotorSpeed));
  }

  public Command runKicker() {
    return run(() -> kickerMotor.set(SpindexerConstants.kickerMotorSpeed));
  }

  public Command stopSpindexerCommand() {
    return runOnce(this::stopSpindexerMotor);
  }

  public Command stopKickerCommand() {
    return runOnce(this::stopKickerMotor);
  }

  public Command runUntilEmptyCommand() {
    return (run(() -> runBoth())).until(() -> currentSaysEmpty());
  }

  public boolean isEmpty() {
    return (spindexerMotor.get() > 0.1) && currentSaysEmpty();
  }

  @Override
  public void periodic() {
    spindexerCurrent.refresh();
    kickerCurrent.refresh();

    SmartDashboard.putNumber("Spindexer/Spindexer Current", spindexerCurrent.getValue().in(Amps));
    SmartDashboard.putNumber("Spindexer/Kicker Current", kickerCurrent.getValue().in(Amps));
    SmartDashboard.putBoolean("Spindexer/Spindexer Empty", isEmpty());
  }
}
