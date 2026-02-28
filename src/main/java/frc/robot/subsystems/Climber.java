// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
  private TalonFX climberMotor;

  private StatusSignal<Angle> climberPosition;

  /** Creates a new Spindexer. */
  private ElevatorSim climberSim;

  public Climber() {
    climberMotor = new TalonFX(ClimbConstants.climberMotorID);

    climberMotor.getConfigurator().apply(ClimbConstants.climberConfigs);

    zeroClimber();

    climberPosition = climberMotor.getPosition();

    if (RobotBase.isSimulation()) {
      climberSim =
          new ElevatorSim(
              DCMotor.getKrakenX60(1),
              ClimbConstants.climberGearBox,
              Units.lbsToKilograms(5),
              Units.inchesToMeters(ClimbConstants.drumRadiusInches),
              ClimbConstants.minHeight.in(Meters),
              ClimbConstants.maxHeight.in(Meters),
              true,
              0);
    }
  }

  public void zeroClimber() {
    climberMotor.setPosition(0);
  }

  public void stopClimber() {
    climberMotor.stopMotor();
  }

  public Command stopClimberCommand() {
    return runOnce(() -> stopClimber());
  }

  public Angle getClimberPosition() {
    return climberPosition.getValue();
  }

  public Distance getClimberHeight() {
    return Meters.of(getClimberPosition().in(Rotations));
  }

  public boolean atTop() {
    return getClimberHeight().gte(ClimbConstants.maxHeight.minus(Inches.of(0.1)));
  }

  public boolean atBottom() {
    return getClimberHeight().lte(ClimbConstants.minHeight.plus(Inches.of(0.1)));
  }

  public void setSpeed(double speed) {
    climberMotor.set(speed);
  }

  public Command moveForward() {
    return run(() -> climberMotor.set(ClimbConstants.climbSpeed));
  }

  public Command moveReverse() {
    return run(() -> climberMotor.set(-1 * ClimbConstants.climbSpeed));
  }

  public Command climberUpCommand() {
    return run(() -> climberMotor.set(ClimbConstants.climbSpeed))
        .until(this::atTop)
        .finallyDo(() -> stopClimber());
  }

  public Command climberDownCommand() {
    return run(() -> climberMotor.set(-1 * ClimbConstants.climbSpeed))
        .until(this::atBottom)
        .finallyDo(() -> stopClimber());
  }

  @Override
  public void periodic() {
    climberPosition.refresh();
    SmartDashboard.putNumber("Climber/ClimberPosition", getClimberPosition().in(Rotations));
    SmartDashboard.putNumber("Climber/ClimberHeight Inches", getClimberHeight().in(Inches));
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimState climberSimState = climberMotor.getSimState();

    climberSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    climberSim.setInputVoltage(climberSimState.getMotorVoltage());

    climberSim.update(0.020);

    climberSimState.setRawRotorPosition(
        climberSim.getPositionMeters() * ClimbConstants.climberSensorToMechanismRatio);

    climberSimState.setRotorVelocity(
        climberSim.getVelocityMetersPerSecond() * ClimbConstants.climberSensorToMechanismRatio);
  }
}
