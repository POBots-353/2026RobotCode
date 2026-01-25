// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class SwerveConstants {
    public static final int pigeonID = 20;

    public static final LinearVelocity maxTranslationalSpeed = FeetPerSecond.of(15);
    public static final LinearVelocity slowModeMaxTranslationalSpeed = FeetPerSecond.of(5);
    public static final AngularVelocity maxRotationalSpeed = RotationsPerSecond.of(1.5);

    public static final Time translationZeroToFull = Seconds.of(0.6);
    public static final Time rotationZeroToFull = Seconds.of(0.25);

    public static final LinearAcceleration maxTransationalAcceleration =
        maxTranslationalSpeed.div(translationZeroToFull);
    public static final AngularAcceleration maxAngularAcceleration =
        maxRotationalSpeed.div(rotationZeroToFull);


    public static final double headingP = 0.0;
    public static final double headingD = 0.0;

    public static final double steerKP = 100.0;
    public static final double steerKI = 0.0;
    public static final double steerKD = 0.5;
    public static final double steerKS = 0.1;
    public static final double steerKV = 1.91;
    public static final double steerKA = 0.0;
  }

  public static class intakeConstants {
    public static final int armID = 15;
    public static final int wheelID = 16;
    public static final int beamID = 17;
    public static final int joystickID = 18;

    public static final double voltagePreset = 0;
    /*Gear value TBD || All values random */
    public static final double gearRatio = 5.0;
    public static final double roationUp = 10;
    public static final double rotationDown = 0;

    public static final double speedUp = roationUp * gearRatio;
    public static final double speedDown = rotationDown * gearRatio; /*gonna be 0 anyway */
    public static final double slowUp = (rotationDown * gearRatio) / 2;

    public static final double upPosition = 0;
    public static final double downPosition = 5;
  }

  public static class VisionConstants {
    public static final String arducamLeftName = "Arducam_Left";

    public static final Transform3d arducamLeftTransform =
        new Transform3d(
            Units.inchesToMeters(-10.02),
            Units.inchesToMeters(10.02),
            Units.inchesToMeters(5),
            new Rotation3d(
                0, Units.degreesToRadians(-25), Units.degreesToRadians(180 - 45))); // Pitch: 65

    public static final String arducamRightName = "Arducam_Right";

    public static final Transform3d arducamRightTransform =
        new Transform3d(
            Units.inchesToMeters(-10.02),
            Units.inchesToMeters(-10.02),
            Units.inchesToMeters(5),
            new Rotation3d(
                0, Units.degreesToRadians(-25), Units.degreesToRadians(180 + 45))); // Pitch: 65

    public static final String arducamFrontName = "Arducam_Front";

    public static final Transform3d arducamFrontTransform =
        new Transform3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(-1),
            Units.inchesToMeters(10.07),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(0))); // Pitch: 65

    public static final String arducamFuelName = "Arducam_Fuel";

    // TODO: Update this transform
    public static final Transform3d arducamFuelTransform =
        new Transform3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0)));
  }

  public static class FieldConstants {
    public static final String aprilTagJson = "2026-rebuilt-welded";
    public static final Path aprilTagJsonPath =
        Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", aprilTagJson + ".json");

    public static AprilTagFieldLayout aprilTagLayout;

    static {
      try {
        aprilTagLayout = new AprilTagFieldLayout(aprilTagJsonPath);
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }

    public static final double fieldLength = aprilTagLayout.getFieldLength();
    public static final double fieldWidth = aprilTagLayout.getFieldWidth();

    public static final Pose2d hubBlueAlliance = new Pose2d(4.625594, 4.03479, Rotation2d.kZero);
    public static final Pose2d hubRedAlliance = new Pose2d(11.915394, 4.03479, Rotation2d.kZero);

    public static final Pose2d allianceLMid = new Pose2d(6, 7.43, Rotation2d.kZero);
    public static final Pose2d allianceLSide = new Pose2d(3.2353, 7.43, Rotation2d.kZero);
    public static final Pose2d allianceRMid = new Pose2d(6, 0.65, Rotation2d.kZero);
    public static final Pose2d allianceRSide = new Pose2d(3.2353, 0.65, Rotation2d.kZero);
    public static final Pose2d midRBumperPose =
        new Pose2d(5.600369930267334, 2.43641996383667, Rotation2d.kZero);
    public static final Pose2d midLBumperPose =
        new Pose2d(5.6198601722717285, 5.5177741050720215, Rotation2d.kZero);

    // top of the plastic ring on the hub is 72 inches
    public static final Distance hubHeight = Inches.of(72 - 8);

    public static final List<Pose2d> blueFerryPoints =
        List.of(
            new Pose2d(2.0, 7.347899913787842, Rotation2d.kZero),
            new Pose2d(2.0, 5.476860046386719, Rotation2d.kZero),
            new Pose2d(2.0, 3.118569850921631, Rotation2d.kZero),
            new Pose2d(2.0, 1.4229397773742676, Rotation2d.kZero));
  }

  public static class TurretConstants {

    public static final double gearOneTeeth = 48;
    public static final double gearTwoTeeth = 50;
    public static final double turretTeeth = 85;

    public static final Angle tolerance = Degrees.of(20);

    public static final double totalGearRatio = (gearOneTeeth / 10) * (turretTeeth / 10);

    public static final Angle MIN_ANGLE = Degrees.of(-360.0);
    public static final Angle MAX_ANGLE = Degrees.of(360.0);

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(40))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(80));

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.3)
            .withKV(0.67)
            .withKA(0.2)
            .withKG(0.00)
            .withKP(35.3)
            .withKI(0.00)
            .withKD(0.00)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(totalGearRatio);

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(
                InvertedValue.CounterClockwise_Positive) // needs to spin left when wires up
            .withNeutralMode(NeutralModeValue.Coast);

    public static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(MAX_ANGLE)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(MIN_ANGLE)
            .withReverseSoftLimitEnable(true);

    public static final CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(45).withSupplyCurrentLimitEnable(true);

    public static final TalonFXConfiguration turretConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

    public static final Translation3d robotToTurret =
        new Translation3d(-0.26, .26, Units.inchesToMeters(10.826));

    public static final int turretMotorID = 19;
    public static final int encoderAID = 20;
    public static final int encoderBID = 21;
  }

  public static class AutoConstants {
    public static final PIDConstants translationPID = new PIDConstants(2, 0.0, 0.1); // 5 2.2
    public static final PIDConstants rotationPID = new PIDConstants(1.4, 0.0, 0.1); // 1  2.8
    public static final PathConstraints pathConstraints =
        new PathConstraints(
            SwerveConstants.maxTranslationalSpeed,
            SwerveConstants.maxTransationalAcceleration,
            SwerveConstants.maxRotationalSpeed,
            SwerveConstants.maxAngularAcceleration);
  }

  public static class HoodConstants {
    public static final int hoodMotorID = 22;

    private static final double hoodGearRatio = 100.0;

    private static final Angle minAngle = Degrees.of(15.0);
    private static final Angle maxAngle = Degrees.of(60.0);

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(40))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(80));

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.00)
            .withKG(0.00)
            .withKP(0.0)
            .withKI(0.00)
            .withKD(0.00)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(hoodGearRatio);

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(
                InvertedValue.CounterClockwise_Positive) // needs to spin left when wires up
            .withNeutralMode(NeutralModeValue.Coast);

    public static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(maxAngle)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(minAngle)
            .withReverseSoftLimitEnable(true);

    public static final CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(45).withSupplyCurrentLimitEnable(true);

    public static final TalonFXConfiguration hoodConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

    public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
      hoodAngleMap.put(1.5, 1.0);
      hoodAngleMap.put(2.0, 2.0);
      hoodAngleMap.put(2.5, 3.0);
      hoodAngleMap.put(3.0, 4.0);
      hoodAngleMap.put(3.53, 5.0); // random values (Distance, ROTATIONS)
    }
  }

  public static class SpindexerConstants {

    public static final int SpindexerMotorID = 23;
    public static final int SpindexerLaserID = 24;
    public static final double SpindexerMotorSpeed = 0.5;
    public static final double SpindexerDistance = 100;
  }
}
