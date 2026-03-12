// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private final TalonFX armMainMotor;
  private final TalonFX armFollowerMotor;
  private final TalonFX intakeMotor;
  // private final DigitalInput armMagnetSensor;

  private boolean wasDeployedLastLoop;

  private final double shakePeriod = 1.0;

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

    // armMagnetSensor = new DigitalInput(IntakeConstants.armMagnetID);
    intakeMotor.getConfigurator().apply(IntakeConstants.intakeConfigs);
    armMainMotor.getConfigurator().apply(IntakeConstants.armMainConfigs);
    armFollowerMotor.getConfigurator().apply(IntakeConstants.armFollowerConfigs);

    armCurrent = armMainMotor.getStatorCurrent();
    armVelocity = armMainMotor.getVelocity();

    armMainMotor.setPosition(IntakeConstants.minPosition);
    armFollowerMotor.setPosition(IntakeConstants.minPosition);

    armMainPosition = armMainMotor.getPosition();
    armFollowerPosition = armFollowerMotor.getPosition();
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public Command startIntake() {
    return run(() -> intakeMotor.set(IntakeConstants.intakeSpeed)).withName("Start Intake");
  }

  public Command reverseIntakeRollers() {
    return run(() -> intakeMotor.set(-1 * IntakeConstants.intakeSpeed)).withName("Reverse Intake");
  }

  public Command stopIntake() {
    return runOnce(() -> intakeMotor.stopMotor()).withName("Stop Intake");
  }

  public Command stopArm() {
    return runOnce(
            () -> {
              armMainMotor.stopMotor();
              armFollowerMotor.stopMotor();
            })
        .withName("Stop Intake");
  }

  public Command shakeIntakeCommand() {
    return run(
        () -> {
          if (Timer.getFPGATimestamp() % shakePeriod < shakePeriod / 2) {
            moveDown();
          } else {
            moveUp();
          }
        });
  }

  public void moveDown() {
    armMainMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.downPosition));
    armFollowerMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.downPosition));
  }

  public void moveDownManual() {
    armMainMotor.set(0.25);
    armFollowerMotor.set(0.25);
  }

  public void moveUpManual() {
    armMainMotor.set(-0.25);
    armFollowerMotor.set(-0.25);
  }

  public void moveUp() {
    armMainMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.upPosition));
    armFollowerMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.upPosition));
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
                motionMagicRequest.withPosition(IntakeConstants.upPosition));
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
        () -> Math.abs(intakeMotor.get()) > 0.05);
  }

  public boolean isIntakeDeployed() {
    if (RobotBase.isSimulation()) return true;
    return armMainPosition.getValue().gte(IntakeConstants.armDownPositionTolerance);
    // return true; // FOR TESTING IN SIM
  }

  public boolean rollersRunning() {
    return Math.abs(intakeMotor.get()) > 0.05;
  }

  public Angle getMainArmAngle() {
    return armMainPosition.getValue();
  }

  public void setZero() {
    armMainMotor.setPosition(0);
    armFollowerMotor.setPosition(0);
  }

  public void setArmMaxPosition() {
    armMainMotor.setPosition(IntakeConstants.maxPosition);
    armFollowerMotor.setPosition(IntakeConstants.maxPosition);
    armMainPosition.refresh();
    armFollowerPosition.refresh();
  }

  public Command zeroArmCommand() {
    return Commands.startRun(
            // run once
            () -> {
              armMainMotor.stopMotor();
              armFollowerMotor.stopMotor();

              armMainMotor.setPosition(0);
            },
            // run
            () -> {
              armMainMotor.set(IntakeConstants.armZeroSpeed);
              armFollowerMotor.set(IntakeConstants.armZeroSpeed);
            })
        .until(
            () ->
                armMainMotor.getStatorCurrent().getValueAsDouble()
                    > IntakeConstants.armStallCurrent)
        .finallyDo(
            () -> {
              armMainMotor.stopMotor();
              armFollowerMotor.stopMotor();

              armMainMotor.setPosition(IntakeConstants.maxPosition);
              armFollowerMotor.setPosition(IntakeConstants.maxPosition);

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

    armMainPosition.refresh();
    armFollowerPosition.refresh();

    SmartDashboard.putNumber("Intake speed", intakeMotor.get());
    SmartDashboard.putNumber("Intake Arm Position", armMainPosition.getValue().in(Degrees));
    SmartDashboard.putBoolean("Intake Arm Deployed", isIntakeDeployed());
  }
}
