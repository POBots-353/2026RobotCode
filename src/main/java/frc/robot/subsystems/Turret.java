// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
  private final double gearATeeth = TurretConstants.gearATeeth;
  private final double gearBTeeth = TurretConstants.gearBTeeth;
  private final double turretTeeth = TurretConstants.turretTeeth;

  private final double encoderARatio = 10 / gearATeeth;
  private final double encoderBRatio = 10 / gearBTeeth;

  private final double encoderGearing = encoderBRatio / encoderARatio; // 48/50

  private final double turretReduction = (turretTeeth / 10) / (10 / gearATeeth);

  private CANcoder encoderA;
  private CANcoder encoderB;

  private TalonFX turretMotor;

  private StatusSignal<Angle> turretPosition;

  private Angle desiredAngle = Degrees.of(0);

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);

  private final DCMotorSim turretSim;

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
                        .withMagnetOffset(TurretConstants.encAMagnetOffset)
                        .withAbsoluteSensorDiscontinuityPoint(1.0)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

    encoderB
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withMagnetOffset(TurretConstants.encBMagnetOffset)
                        .withAbsoluteSensorDiscontinuityPoint(1.0)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

    turretMotor.setPosition(getAbsoluteTurretPosition());
    turretPosition = turretMotor.getPosition();

    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.196, TurretConstants.totalGearRatio),
            DCMotor.getKrakenX60(1));
  }

  public Command faceTarget(Supplier<Pose2d> targetSupplier, Supplier<Pose2d> robotPoseSupplier) {
    return run(() -> {
          Pose2d targetPose = targetSupplier.get();
          Pose2d robotPose = robotPoseSupplier.get();

          Angle angleToFace = fieldAngleToFaceTarget(targetPose.getTranslation(), robotPose);

          turretMotor.setControl(motionMagicRequest.withPosition(angleToFace.in(Rotations)));
        })
        .withName("Turret Face Target");
  }

  public Angle fieldAngleToFaceTarget(Translation2d targetPose, Pose2d robotPose) {
    Pose2d turretPose = robotPose.transformBy(TurretConstants.robotToTurretTransform);

    Rotation2d turretAngle = targetPose.minus(turretPose.getTranslation()).getAngle();
    Rotation2d fieldAngle = turretAngle.minus(robotPose.getRotation());

    SmartDashboard.putNumber("Turret/READ HERE", turretAngle.getDegrees());

    Angle angleToFace = optimizeAngle(Degrees.of(fieldAngle.getDegrees()));

    return angleToFace;
  }

  @Logged(name = "Turret Angle")
  public Angle getTurretAngle() {
    return turretPosition.getValue();
  }

  @Logged(name = "Absolute Position")
  public Angle getAbsoluteTurretPosition() {
    double eA = encoderA.getAbsolutePosition().getValue().in(Rotations);
    double eB = encoderB.getAbsolutePosition().getValue().in(Rotations);
    SmartDashboard.putNumber("Turret/Encoder A", eA);
    SmartDashboard.putNumber("Turret/Encoder B", eB);

    double predictedB = eA * (encoderGearing);
    SmartDashboard.putNumber("Turret/predictedB", predictedB);

    double diff = eB - predictedB;
    SmartDashboard.putNumber("Turret/diff", diff);

    diff = MathUtil.inputModulus(diff, -0.5, 0.5);

    long aRotations = (-1) * Math.round(diff / (1 - encoderGearing));
    SmartDashboard.putNumber("Turret/aRotations", aRotations);

    double motorRotations = (eA / encoderARatio) + (aRotations / encoderARatio);
    SmartDashboard.putNumber("Turret/motorRotations", motorRotations);

    double turretRotations = motorRotations / turretReduction;

    return Rotations.of(-turretRotations);
  }

  public void moveToAngle(Angle desiredTurretAngle) {
    desiredAngle = optimizeAngle(desiredTurretAngle);
    turretMotor.setControl(motionMagicRequest.withPosition(desiredAngle));
  }

  public Command moveToAngleCommand(Angle desiredTurretAngle) {
    return run(() -> {
          desiredAngle = optimizeAngle(desiredTurretAngle);
          turretMotor.setControl(motionMagicRequest.withPosition(desiredAngle));
        })
        .withName("Turret To Position ");
  }

  private Angle optimizeAngle(Angle desiredAngle) {
    double currentDeg = turretPosition.getValue().in(Degrees);

    double desiredDeg = desiredAngle.in(Degrees);

    double delta = desiredDeg - currentDeg;
    while (delta > 180) delta -= 360;
    while (delta < -180) delta += 360;

    double candidate = currentDeg + delta;

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

  public void runTurret(double speed) {
    turretMotor.set(speed);
  }

  public Command resetTurretPosition() {
    return runOnce(() -> turretMotor.setPosition(getAbsoluteTurretPosition()))
        .withName("Reset Turret Position");
  }

  public Command stop() {
    return runOnce(() -> turretMotor.stopMotor()).withName("Stop Turret");
  }

  public double getTurretVelocity() {
    return turretMotor.getVelocity().getValue().in(DegreesPerSecond);
  }

  public boolean hasDriftedTooMuch(Angle tolerance) {
    Angle motorAngle = turretPosition.getValue();
    Angle absAngle = encoderA.getAbsolutePosition().getValue();

    double errorRad = MathUtil.angleModulus(motorAngle.in(Radians) - absAngle.in(Radians));
    return Math.abs(Units.radiansToDegrees(errorRad)) > tolerance.in(Degrees);
  }

  @Logged(name = "Zeroed Poses Turret")
  public Pose3d[] zeroedComponentPoses() {
    return new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()};
  }

  @Override
  public void periodic() {
    turretPosition.refresh();

    // turretMotor.setControl(motionMagicRequest.withPosition(desiredAngle));

    SmartDashboard.putNumber("Turret/TwoEncoder Angle", getAbsoluteTurretPosition().in(Degrees));
    SmartDashboard.putNumber(
        "Turret/Turret Angle", Units.rotationsToDegrees(turretPosition.getValueAsDouble()));

    SmartDashboard.putNumber("Turret/desired turret angle", desiredAngle.in(Degrees));

    SmartDashboard.putNumber(
        "Turret/Angle difference", desiredAngle.minus(turretPosition.getValue()).in(Degrees));

    SmartDashboard.putBoolean("Turret/Drifted too much", hasDriftedTooMuch(Degrees.of(5)));
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimState turretSimState = turretMotor.getSimState();
    turretSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    turretSim.setInputVoltage(turretSimState.getMotorVoltage());

    turretSim.update(0.020);

    turretSimState.setRawRotorPosition(
        turretSim.getAngularPosition().times(TurretConstants.totalGearRatio));

    turretSimState.setRotorVelocity(
        turretSim.getAngularVelocity().times(TurretConstants.totalGearRatio));
  }
}
