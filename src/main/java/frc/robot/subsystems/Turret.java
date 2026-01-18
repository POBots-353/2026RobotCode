// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.AllianceUtil;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
  /** Creates a new twoEncoderTurret. */

  // private final Field2d m_field = new Field2d();
  private final Pose2d towerPose = AllianceUtil.getHubPose();

  private final double motorGearTeeth = 30.0;
  private final double secondaryGearTeeth = 32.0;
  private final double turretGearTeeth = 210.0;

  private final double kEncoderGearing = motorGearTeeth / secondaryGearTeeth; // 30/32 = 0.9375

  private final double kTurretReduction = turretGearTeeth / motorGearTeeth; // 210/30 = 7

  private CANcoder encoderA;
  private CANcoder encoderB;

  private TalonFX turretMotor;

  private StatusSignal<Angle> turretPosition;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  public Turret() {
    encoderA = new CANcoder(TurretConstants.encoderAID);
    encoderB = new CANcoder(TurretConstants.encoderBID);
    turretMotor = new TalonFX(TurretConstants.turretMotorID);

    turretMotor.getConfigurator().apply(TurretConstants.turretConfigs);

    encoderA
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

    encoderB
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

    turretMotor.setPosition(getAbsoluteTurretPosition().in(Rotations));
    turretPosition = turretMotor.getPosition();
  }

  // public void faceTowards(Pose2d target, Pose2d robotPose) {
  //   Pose2d turretPose =
  //       robotPose.transformBy(
  //           new Transform2d(TurretConstants.turretOnRobot.toTranslation2d(), Rotation2d.kZero));

  //   Rotation2d turretAngle =
  // target.getTranslation().minus(turretPose.getTranslation()).getAngle();
  //   Rotation2d angleToFace = turretAngle.minus(robotPose.getRotation());
  //   Angle finalAngle = Degrees.of(angleToFace.getDegrees());
  //   finalAngle = optimizeAngle(finalAngle);

  //   turretMotor.setControl(motionMagicRequest.withPosition(finalAngle.in(Rotations)));

  // }

  public Command faceTarget(Supplier<Pose2d> targetSupplier, Supplier<Pose2d> robotPoseSupplier) {

    return run(() -> {
          Pose2d target = targetSupplier.get();
          Pose2d robotPose = robotPoseSupplier.get();

          Pose2d turretPose =
              robotPose.transformBy(
                  new Transform2d(
                      TurretConstants.turretOnRobot.toTranslation2d(), Rotation2d.kZero));

          Rotation2d turretAngle =
              target.getTranslation().minus(turretPose.getTranslation()).getAngle();
          Rotation2d angleToFace = turretAngle.minus(robotPose.getRotation());
          Angle finalAngle = Degrees.of(angleToFace.getDegrees());
          finalAngle = optimizeAngle(finalAngle);

          turretMotor.setControl(motionMagicRequest.withPosition(finalAngle.in(Rotations)));
        })
        .withName("Turret Face Target");
  }

  private Angle getAbsoluteTurretPosition() {

    double eA = encoderA.getAbsolutePosition().getValue().in(Rotations);
    double eB = encoderB.getAbsolutePosition().getValue().in(Rotations);

    double predicted32 = eA * kEncoderGearing;

    double diff = eB - predicted32;

    diff = MathUtil.inputModulus(diff, -0.5, 0.5);

    long fullRotations = Math.round(diff / (kEncoderGearing - 1));

    double motorRotations = eA + fullRotations;

    double turretRotations = motorRotations / kTurretReduction;

    return Degrees.of(turretRotations * 360);
  }

  public void setTargetAngleDeg(Angle desiredTurretAngle) {

    turretMotor.setControl(motionMagicRequest.withPosition(optimizeAngle(desiredTurretAngle)));
  }

  private Angle optimizeAngle(Angle desiredAngle) {
    // double current = getAbsoluteTurretPosition().in(Degrees);
    double current = turretPosition.getValue().in(Degrees);

    double desiredDeg = desiredAngle.in(Degrees);

    double delta = desiredDeg - current;
    while (delta > 180) delta -= 360;
    while (delta < -180) delta += 360;

    double candidate = current + delta;

    if (candidate > TurretConstants.MAX_DEGREES) {
      candidate -= 360;
    } else if (candidate < TurretConstants.MIN_DEGREES) {
      candidate += 360;
    }

    candidate = MathUtil.clamp(candidate, TurretConstants.MIN_DEGREES, TurretConstants.MAX_DEGREES);

    return Degrees.of(candidate);
  }

  public Command setZero() {
    return runOnce(() -> turretMotor.setPosition(0)).withName("Set Turret Zero");
  }

  public Command stop() {
    return runOnce(() -> turretMotor.stopMotor()).withName("Stop Turret");
  }

  public boolean hasDriftedTooMuch(double toleranceDeg) {
    double motorAngleRad = Units.degreesToRadians(turretPosition.getValueAsDouble() * 360);
    double absAngleRad =
        Units.degreesToRadians(encoderA.getAbsolutePosition().getValueAsDouble() * 360);

    double errorRad = MathUtil.angleModulus(motorAngleRad - absAngleRad);
    return Math.abs(Units.radiansToDegrees(errorRad)) > toleranceDeg;
  }

  @Override
  public void periodic() {
    turretPosition.refresh();
    SmartDashboard.putNumber("TwoEncoder Angle", getAbsoluteTurretPosition().in(Degrees));
    SmartDashboard.putNumber("Turret Angle", turretPosition.getValueAsDouble());
    SmartDashboard.putBoolean("Drifted too much", hasDriftedTooMuch(5));
  }
}
