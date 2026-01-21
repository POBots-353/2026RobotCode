// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
  private TalonFX SpindexerMotor;
  private LaserCan SpindexerLaser;
  private Debouncer SpindexDebouncer;


  /** Creates a new Spindexer. */
  public Spindexer() {
    SpindexerMotor = new TalonFX(SpindexerConstants.SpindexerMotorID);
    SpindexerLaser = new LaserCan(SpindexerConstants.SpindexerLaserID);
    SpindexDebouncer = new Debouncer(1.5);

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    spindexerConfig.Slot0.kP = 0.0;
    spindexerConfig.Slot0.kI = 2.5;
    spindexerConfig.Slot0.kD = 5.3;

    SpindexerMotor.getConfigurator().apply(new TalonFXConfiguration());
  }

  public void setSpeed() {
    SpindexerMotor.set(SpindexerConstants.SpindexerMotorSpeed);
  }

  public void stopMotor() {
    SpindexerMotor.stopMotor();
  }

  public double getSpeed() {
    return SpindexerMotor.get();
  }

  public Command runSpindexer() {
    return run(this::setSpeed);
  }

  public Command stopSpindexer() {
    return run(this::stopMotor);
  }

  public Command upSpeed(double speed) {
    return run(
        () -> {
          SpindexerMotor.set(speed);
        });
  }

  public Command downSpeed(double speed) {
    return run(
        () -> {
          SpindexerMotor.set(-speed);
        });
  }

  //new command runUntilEMpty
  // empty means laser can cant see anything anymore
  // look up debouncer

  public Command runUntilEmptyCommand(){
    return (runSpindexer()) .until(()-> SpindexDebouncer.calculate(!beamBroken()));
  }
  
  
  public boolean beamBroken(){
    LaserCan.Measurement measurement = SpindexerLaser.getMeasurement();

    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){

      if (measurement.distance_mm <= SpindexerConstants.SpindexerDistance){
        return true;

      }else {
        return false;
      }
    }else{
      return false;
    }
  } 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
