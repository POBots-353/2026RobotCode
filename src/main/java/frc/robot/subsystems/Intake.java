// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// robot=be good
// ethan is the worst programmer
// ^statement = false
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /*Objects */
  private final TalonFX armMotor;
  private final TalonFX rollerMotor;
  // private final DigitalInput armMagnetSensor;

  private boolean wasDeployedLastLoop;

  private final double shakePeriod = 1.0;

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);

  private StatusSignal<Angle> armPosition;

  private final StatusSignal<Current> armCurrent;
  private final StatusSignal<AngularVelocity> armVelocity;

  /*MotionMagic*/
  public Intake() {
    rollerMotor = new TalonFX(IntakeConstants.intakeID);
    armMotor = new TalonFX(IntakeConstants.armMainID);

    // armMagnetSensor = new DigitalInput(IntakeConstants.armMagnetID);
    rollerMotor.getConfigurator().apply(IntakeConstants.intakeConfigs);
    armMotor.getConfigurator().apply(IntakeConstants.armMainConfigs);

    armCurrent = armMotor.getStatorCurrent();
    armVelocity = armMotor.getVelocity();

    armMotor.setPosition(IntakeConstants.minPosition);

    armPosition = armMotor.getPosition();
  }

  public void setIntakeSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public Command startIntake() {
    return run(() -> rollerMotor.set(IntakeConstants.intakeSpeed)).withName("Start Intake");
  }

  public Command reverseIntakeRollers() {
    return run(() -> rollerMotor.set(-1 * IntakeConstants.intakeSpeed)).withName("Reverse Intake");
  }

  public Command stopIntake() {
    return runOnce(() -> rollerMotor.stopMotor()).withName("Stop Intake");
  }

  public Command stopArm() {
    return runOnce(
            () -> {
              armMotor.stopMotor();
            })
        .withName("Stop Intake");
  }

  public Command shakeIntakeCommand() {
    return run(
        () -> {
          if (Timer.getFPGATimestamp() % shakePeriod < shakePeriod / 2) {
            moveMid();
          } else {
            moveDown();
          }
        });
  }

  public void moveDown() {
    armMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.downPosition));
  }

  public void moveMid() {
    armMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.shakePosition));
  }

  public void moveDownManual() {
    armMotor.set(0.25);
  }

  public void moveUpManual() {
    armMotor.set(-0.25);
  }

  public void moveUp() {
    armMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.upPosition));
  }

  public Command intakeToPosition(boolean downPosition) {
    return run(() -> {
          if (downPosition) {
            armMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.downPosition));

          } else {
            armMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.upPosition));
          }
        })
        .withName("Intake To Position");
  }

  public Command intakeSequence(boolean intakeDown) {
    return run(() -> {
          if (intakeDown) {
            armMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.downPosition));
            setIntakeSpeed(IntakeConstants.intakeSpeed);
          } else {
            armMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.upPosition));
            setIntakeSpeed(0);
          }
        })
        .withName("Intake working");
  }

  public Command toggleRollers() {
    return Commands.either(
        // if rolelrs are already on stop them
        stopIntake(),

        // if rollers off then move it down and then set them to the speed
        run(
            () -> {
              moveDown();
              setIntakeSpeed(IntakeConstants.intakeSpeed);
            }),
        () -> rollersRunning());
  }

  public boolean isIntakeDeployed() {
    if (RobotBase.isSimulation()) return true;
    return armPosition.getValue().gte(IntakeConstants.armDownPositionTolerance);
    // return true; // FOR TESTING IN SIM
  }

  public boolean rollersRunning() {
    return Math.abs(rollerMotor.get()) > 0.05;
  }

  public Angle getMainArmAngle() {
    return armPosition.getValue();
  }

  public void setZero() {
    armMotor.setPosition(0);
  }

  public void setArmMaxPosition() {
    armMotor.setPosition(IntakeConstants.maxPosition);
    armPosition.refresh();
  }

  public Command zeroArmCommand() {
    return Commands.startRun(
            // run once
            () -> {
              armMotor.stopMotor();

              armMotor.setPosition(0);
            },
            // run
            () -> {
              armMotor.set(IntakeConstants.armZeroSpeed);
            })
        .until(
            () -> armMotor.getStatorCurrent().getValueAsDouble() > IntakeConstants.armStallCurrent)
        .finallyDo(
            () -> {
              armMotor.stopMotor();

              armMotor.setPosition(IntakeConstants.maxPosition);

              // armMainMoo.setControl(motionMagicRequest.withPosition(Degrees.of(25)));
            });
  }

  // public Command zeroArmCommand() {
  //   return Commands.run(
  //           () -> {
  //             armMainMotor.set(IntakeConstants.armZeroSpeed);
  //             // armFollowerMotor.set(IntakeConstants.armZeroSpeed);

  //             armCurrent.refresh();
  //             armVelocity.refresh();
  //           })
  //       .until(
  //           () ->
  //               armCurrent.getValue().in(Amps) > IntakeConstants.armStallCurrent
  //                   && Math.abs(armVelocity.getValue().in(RotationsPerSecond))
  //                       < IntakeConstants.armStallVelocity)
  //       .andThen(
  //           () -> {
  //             armMainMotor.stopMotor();
  //             // armFollowerMotor.stopMotor();
  //             setZero();
  //           })
  //       .withName("Zero Arm Command");
  // }

  @Override
  public void periodic() {
    // if (!isIntakeDeployed() && wasDeployedLastLoop) {
    //   armMainMotor.setPosition(IntakeConstants.minPosition);
    //   armFollowerMotor.setPosition(IntakeConstants.minPosition);
    // }
    // wasDeployedLastLoop = isIntakeDeployed();

    armPosition.refresh();

    SmartDashboard.putNumber("Intake speed", rollerMotor.get());
    SmartDashboard.putNumber("Intake Arm Position", armPosition.getValue().in(Degrees));
    SmartDashboard.putBoolean("Intake Arm Deployed", isIntakeDeployed());
  }
}
