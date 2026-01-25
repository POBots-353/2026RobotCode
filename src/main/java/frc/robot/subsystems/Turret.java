// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {

  private final double gearOneTeeth = TurretConstants.gearOneTeeth;
  private final double gearTwoTeeth = TurretConstants.gearTwoTeeth;
  private final double turretTeeth = TurretConstants.turretTeeth;

  private final double encoderARatio = 10 / gearOneTeeth;
  private final double encoderBRatio = 10 / gearTwoTeeth;

  private final double encoderGearing = encoderBRatio / encoderARatio;

  private final double turretReduction = (turretTeeth / 10) / (10 / gearOneTeeth);

  private CANcoder encoderA;
  private CANcoder encoderB;

  private TalonFX turretMotor;

  private StatusSignal<Angle> turretPosition;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private final SingleJointedArmSim turretSim;

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

    turretSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            TurretConstants.totalGearRatio,
            0.001,
            0.1,
            TurretConstants.MIN_ANGLE.in(Radians),
            TurretConstants.MAX_ANGLE.in(Radians),
            false,
            0,
            0.0,
            0.0);
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
                      TurretConstants.robotToTurret.toTranslation2d(), Rotation2d.kZero));

          Rotation2d turretAngle =
              target.getTranslation().minus(turretPose.getTranslation()).getAngle();
          Rotation2d angleToFace = turretAngle.minus(robotPose.getRotation());
          Angle finalAngle = Degrees.of(angleToFace.getDegrees());
          finalAngle = optimizeAngle(finalAngle);

          turretMotor.setControl(motionMagicRequest.withPosition(finalAngle.in(Rotations)));
        })
        .withName("Turret Face Target");
  }

  public Angle angleToFaceTarget(Translation2d targetPose, Pose2d robotPose) {

    Pose2d turretPose =
        robotPose.transformBy(
            new Transform2d(TurretConstants.robotToTurret.toTranslation2d(), Rotation2d.kZero));

    Rotation2d turretAngle = targetPose.minus(turretPose.getTranslation()).getAngle();
    Rotation2d angleToFace = turretAngle.minus(robotPose.getRotation());
    Angle finalAngle = Degrees.of(angleToFace.getDegrees());
    finalAngle = optimizeAngle(finalAngle);

    return finalAngle;
  }

  @Logged(name = "Turret Angle")
  public Angle getTurretAngle() {
    return turretPosition.getValue();
  }

  @Logged(name = "Absolute Position")
  public Angle getAbsoluteTurretPosition() {
    double eA = encoderA.getAbsolutePosition().getValue().in(Rotations);
    double eB = encoderB.getAbsolutePosition().getValue().in(Rotations);

    double predictedB = eA * (encoderGearing);

    double diff = eB - predictedB;

    diff = MathUtil.inputModulus(diff, -0.5, 0.5);

    long fullRotations = Math.round(diff / (encoderGearing - 1));

    double motorRotations = (eA / encoderARatio) + fullRotations;

    double turretRotations = motorRotations / turretReduction;

    return Degrees.of(turretRotations * 360);
  }

  public void setTargetAngle(Angle desiredTurretAngle) {
    turretMotor.setControl(motionMagicRequest.withPosition(optimizeAngle(desiredTurretAngle)));
  }

  private Angle optimizeAngle(Angle desiredAngle) {
    double current = turretPosition.getValue().in(Degrees);

    double desiredDeg = desiredAngle.in(Degrees);

    double delta = desiredDeg - current;
    while (delta > 180) delta -= 360;
    while (delta < -180) delta += 360;

    double candidate = current + delta;

    if (candidate > TurretConstants.MAX_ANGLE.in(Degrees)) {
      candidate -= 360;
    } else if (candidate < TurretConstants.MIN_ANGLE.in(Degrees)) {
      candidate += 360;
    }

    candidate =
        MathUtil.clamp(
            candidate,
            TurretConstants.MIN_ANGLE.in(Degrees),
            TurretConstants.MAX_ANGLE.in(Degrees));

    return Degrees.of(candidate);
  }

  public void stopTurret() {
    turretMotor.stopMotor();
  }

  public Command setZero() {
    return runOnce(() -> turretMotor.setPosition(0)).withName("Set Turret Zero");
  }

  public Command stop() {
    return runOnce(() -> turretMotor.stopMotor()).withName("Stop Turret");
  }

  public void moveTurret(Double speed) {
    turretMotor.set(speed);
  }

  public Command spinOne() {
    return run(() -> turretMotor.setControl(motionMagicRequest.withPosition(1)));
  }

  public Command spinZero() {
    return run(() -> turretMotor.setControl(motionMagicRequest.withPosition(0)));
  }

  public boolean hasDriftedTooMuch(Angle tolerance) {
    Angle motorAngle = turretPosition.getValue();
    Angle absAngle = encoderA.getAbsolutePosition().getValue();

    double errorRad = MathUtil.angleModulus(motorAngle.in(Radians) - absAngle.in(Radians));
    return Math.abs(Units.radiansToDegrees(errorRad)) > tolerance.in(Degrees);
  }

  @Logged(name = "3D Pose Turret")
  public Pose3d getTurretPose3d() {
    return new Pose3d(
        -.1523 + 0.0205,
        -.15248,
        .376,
        new Rotation3d(0, 0, turretPosition.getValue().in(Radians)));
  }

  @Logged(name = "Zeroed Poses Turret")
  public Pose3d[] zeroedComponentPoses() {
    return new Pose3d[] {new Pose3d(), new Pose3d()};
  }

  @Override
  public void periodic() {
    turretPosition.refresh();
    SmartDashboard.putNumber("TwoEncoder Angle", getAbsoluteTurretPosition().in(Degrees));
    SmartDashboard.putNumber("Turret Angle", turretPosition.getValue().in(Degrees));
    SmartDashboard.putBoolean("Drifted too much", hasDriftedTooMuch(Degrees.of(5)));
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimState turretSimState = turretMotor.getSimState();
    turretSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    turretSim.setInputVoltage(turretSimState.getMotorVoltage());

    turretSim.update(0.020);

    turretSimState.setRawRotorPosition(
        (turretSim.getAngleRads() / (2 * Math.PI)) * TurretConstants.totalGearRatio);

    turretSimState.setRotorVelocity(
        (turretSim.getVelocityRadPerSec() / (2 * Math.PI)) * TurretConstants.totalGearRatio);
  }
}
