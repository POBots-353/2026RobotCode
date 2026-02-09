// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
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

  public static class SimConstants {
    public static final int maxCapacity = 30;
    public static final double fuelsPerSecond = 6.7;
    public static final double loopPeriodSecs = 0.020;
    public static final Distance closestPossibleShotDistance = Meters.of(1.5);
  }

  public static class SOTMConstants {
    public static InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public static InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
      hoodAngleMap.put(1.681, Rotation2d.fromDegrees(21.448));
      hoodAngleMap.put(2.016, Rotation2d.fromDegrees(21.972));
      hoodAngleMap.put(2.338, Rotation2d.fromDegrees(22.648));
      hoodAngleMap.put(2.665, Rotation2d.fromDegrees(24.122));
      hoodAngleMap.put(3.034, Rotation2d.fromDegrees(25.658));
      hoodAngleMap.put(3.363, Rotation2d.fromDegrees(26.333));
      hoodAngleMap.put(3.711, Rotation2d.fromDegrees(27.562));
      hoodAngleMap.put(4.061, Rotation2d.fromDegrees(29.036));
      hoodAngleMap.put(4.324, Rotation2d.fromDegrees(29.835));
      hoodAngleMap.put(4.619, Rotation2d.fromDegrees(30.449));
      hoodAngleMap.put(5.010, Rotation2d.fromDegrees(32.046));
      hoodAngleMap.put(5.366, Rotation2d.fromDegrees(33.275));
      hoodAngleMap.put(5.610, Rotation2d.fromDegrees(34.319));
      hoodAngleMap.put(6.051, Rotation2d.fromDegrees(35.794));
      hoodAngleMap.put(6.369, Rotation2d.fromDegrees(36.408));
      hoodAngleMap.put(6.704, Rotation2d.fromDegrees(37.002));
      hoodAngleMap.put(7.022, Rotation2d.fromDegrees(36.531));

      shooterSpeedMap.put(1.681, 5.795);
      shooterSpeedMap.put(2.016, 5.940);
      shooterSpeedMap.put(2.338, 6.229);
      shooterSpeedMap.put(2.665, 6.422);
      shooterSpeedMap.put(3.034, 6.711);
      shooterSpeedMap.put(3.363, 6.903);
      shooterSpeedMap.put(3.711, 7.096);
      shooterSpeedMap.put(4.061, 7.337);
      shooterSpeedMap.put(4.324, 7.529);
      shooterSpeedMap.put(4.619, 7.674);
      shooterSpeedMap.put(5.010, 7.915);
      shooterSpeedMap.put(5.366, 8.156);
      shooterSpeedMap.put(5.610, 8.252);
      shooterSpeedMap.put(6.051, 8.445);
      shooterSpeedMap.put(6.369, 8.637);
      shooterSpeedMap.put(6.704, 8.830);
      shooterSpeedMap.put(7.022, 9.023);

      timeOfFlightMap.put(1.695, 0.893);
      timeOfFlightMap.put(2.016, 0.924);
      timeOfFlightMap.put(2.338, 0.985);
      timeOfFlightMap.put(2.665, 1.013);
      timeOfFlightMap.put(3.034, 1.059);
      timeOfFlightMap.put(3.363, 1.093);
      timeOfFlightMap.put(3.711, 1.118);
      timeOfFlightMap.put(4.061, 1.147);
      timeOfFlightMap.put(4.324, 1.175);
      timeOfFlightMap.put(4.619, 1.195);
      timeOfFlightMap.put(5.010, 1.216);
      timeOfFlightMap.put(5.366, 1.242);
      timeOfFlightMap.put(5.610, 1.241);
      timeOfFlightMap.put(6.051, 1.249);
      timeOfFlightMap.put(6.365, 1.273);
      timeOfFlightMap.put(6.704, 1.295);
      timeOfFlightMap.put(7.022, 1.341);
    }
  }

  public static class SwerveConstants {
    public static final Distance bumperWidth = Inches.of(34);
    public static final Distance bumperLength = Inches.of(34);
    public static final Distance bumperHeight = Inches.of(7.5); // floor to top of bumperx

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

  public static class IntakeConstants {
    public static final int armMainID = 15;
    public static final int armFollowerID = 16;
    public static final int intakeID = 17;
    public static final int armEncoderID = 18;

    public static final double armGearRatio = 3.0;

    public static final Angle maxPosition = Rotations.of(1.0);
    public static final Angle minPosition = Rotations.of(0.0);

    public static final Angle downPosition = Rotations.of(1.0);
    public static final Angle upPosition = Rotations.of(0.0);

    public static final Angle armDownPositionTolerance = maxPosition.plus(minPosition).div(2);

    public static final Angle armMagnetOffset = Rotations.of(0);

    public static final double intakeSpeed = .353;

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(0))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(0));

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKG(0.00)
            .withKP(0.0)
            .withKI(0.00)
            .withKD(0.00)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(armGearRatio);

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(maxPosition)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(minPosition)
            .withReverseSoftLimitEnable(true);

    public static final CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(45).withSupplyCurrentLimitEnable(true);

    public static final TalonFXConfiguration armConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);
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

    public static final Distance fieldLength = Inches.of(650.12);
    public static final Distance fieldWidth = Inches.of(316.64);

    public static final Distance allianceZoneLength = Inches.of(156.06);

    public static final Pose2d hubBlueAlliance =
        new Pose2d(Units.inchesToMeters(181.56), fieldWidth.div(2).in(Meters), Rotation2d.kZero);

    public static final Pose2d hubRedAlliance =
        new Pose2d(
            fieldLength.minus(Inches.of(181.56)).in(Meters),
            fieldWidth.div(2).in(Meters),
            Rotation2d.kZero);

    public static final Pose2d allianceLMid = new Pose2d(6, 7.43, Rotation2d.kZero);
    public static final Pose2d allianceLSide = new Pose2d(3.2353, 7.43, Rotation2d.kZero);
    public static final Pose2d allianceRMid = new Pose2d(6, 0.65, Rotation2d.kZero);
    public static final Pose2d allianceRSide = new Pose2d(3.2353, 0.65, Rotation2d.kZero);
    public static final Pose2d midRBumperPose =
        new Pose2d(5.600369930267334, 2.43641996383667, Rotation2d.kZero);
    public static final Pose2d midLBumperPose =
        new Pose2d(5.6198601722717285, 5.5177741050720215, Rotation2d.kZero);

    // top of the plastic ring on the hub is 72 inches
    public static final Distance mainHubHeight = Inches.of(56.440945);

    public static final Distance funnelRadius = Inches.of(24);
    public static final Distance funnelHeight = Inches.of(72).minus(mainHubHeight);

    public static final List<Pose2d> blueFerryPoints =
        List.of(
            new Pose2d(2.0, 7.347899913787842, Rotation2d.kZero),
            new Pose2d(2.0, 5.476860046386719, Rotation2d.kZero),
            new Pose2d(2.0, 3.118569850921631, Rotation2d.kZero),
            new Pose2d(2.0, 1.4229397773742676, Rotation2d.kZero));

    public static final Distance TRENCH_BUMP_X = Inches.of(181.56);
    private static final Distance TRENCH_WIDTH = Inches.of(49.86);
    private static final Distance BUMP_INSET = TRENCH_WIDTH.plus(Inches.of(12));
    private static final Distance BUMP_LENGTH = Inches.of(73);
    public static final Distance BUMP_CENTER_Y = TRENCH_WIDTH.plus(BUMP_LENGTH.div(2));

    private static final Distance TRENCH_ZONE_EXTENSION = Inches.of(70);
    private static final Distance BUMP_ZONE_EXTENSION = Inches.of(60);
    private static final Distance TRENCH_BUMP_ZONE_TRANSITION =
        TRENCH_WIDTH.plus(BUMP_INSET).div(2);

    public static final Translation2d[][] TRENCH_ZONES = {
      new Translation2d[] {
        new Translation2d(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION), Inches.zero()),
        new Translation2d(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION), TRENCH_BUMP_ZONE_TRANSITION)
      },
      new Translation2d[] {
        new Translation2d(
            TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION),
            fieldWidth.minus(TRENCH_BUMP_ZONE_TRANSITION)),
        new Translation2d(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION), fieldWidth)
      },
      new Translation2d[] {
        new Translation2d(
            fieldLength.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)), Inches.zero()),
        new Translation2d(
            fieldLength.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)),
            TRENCH_BUMP_ZONE_TRANSITION)
      },
      new Translation2d[] {
        new Translation2d(
            fieldLength.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)),
            fieldWidth.minus(TRENCH_BUMP_ZONE_TRANSITION)),
        new Translation2d(fieldLength.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)), fieldWidth)
      }
    };

    public static final Translation2d[][] BUMP_ZONES = {
      new Translation2d[] {
        new Translation2d(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION), TRENCH_BUMP_ZONE_TRANSITION),
        new Translation2d(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION), BUMP_INSET.plus(BUMP_LENGTH))
      },
      new Translation2d[] {
        new Translation2d(
            TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION),
            fieldWidth.minus(BUMP_INSET.plus(BUMP_LENGTH))),
        new Translation2d(
            TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION), fieldWidth.minus(TRENCH_BUMP_ZONE_TRANSITION))
      },
      new Translation2d[] {
        new Translation2d(
            fieldLength.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)),
            fieldWidth.minus(BUMP_INSET.plus(BUMP_LENGTH))),
        new Translation2d(
            fieldLength.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)),
            fieldWidth.minus(TRENCH_BUMP_ZONE_TRANSITION))
      },
      new Translation2d[] {
        new Translation2d(
            fieldLength.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)),
            TRENCH_BUMP_ZONE_TRANSITION),
        new Translation2d(
            fieldLength.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)),
            BUMP_INSET.plus(BUMP_LENGTH))
      }
    };

    public static final Distance TRENCH_CENTER = TRENCH_WIDTH.div(2);
  }

  public static class TurretConstants {
    public static final double gearOneTeeth = 48;
    public static final double gearTwoTeeth = 50;
    public static final double turretTeeth = 85;

    public static final Angle tolerance = Degrees.of(20);

    public static final double totalGearRatio = (gearOneTeeth / 10) * (turretTeeth / 10);

    public static final Angle MIN_ANGLE = Degrees.of(-360.0);
    public static final Angle MAX_ANGLE = Degrees.of(360.0);

    public static final Angle encAMagnetOffset = Degrees.of(0);
    public static final Angle encBMagnetOffset = Degrees.of(0);

    public static final AngularVelocity maxTurretVelocity = DegreesPerSecond.of(3 * 360);
    public static final AngularAcceleration maxTurretAcceleration =
        DegreesPerSecondPerSecond.of(12 * 360);

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxTurretVelocity)
            .withMotionMagicAcceleration(maxTurretAcceleration);

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKG(0.00)
            .withKP(1000.3)
            .withKI(0)
            .withKD(13.53)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(totalGearRatio);

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(
                InvertedValue.CounterClockwise_Positive) // needs to spin left when wires up
            .withNeutralMode(NeutralModeValue.Brake);

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
        new Translation3d(-0.153, -0.15248, 0.376 + .1524);

    public static final Transform2d robotToTurretTransform =
        new Transform2d(TurretConstants.robotToTurret.toTranslation2d(), Rotation2d.kZero);

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

    public static final Angle minAngle = Degrees.of(21.448);
    public static final Angle maxAngle = Degrees.of(59.231);

    public static final double hoodGearRatio =
        ((48 / 12) * (30 / 15) * (15 / 10)) / ((maxAngle.minus(minAngle)).in(Degrees) / 360);

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(40))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(80));

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.01)
            .withKV(0.01)
            .withKA(0.01)
            .withKG(0.2)
            .withKP(50.0)
            .withKI(0.00)
            .withKD(0.00)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(hoodGearRatio);

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(
                InvertedValue.CounterClockwise_Positive) // needs to spin left when wires up
            .withNeutralMode(NeutralModeValue.Brake);

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
  }

  public static class SpindexerConstants {
    public static final int SpindexerMotorID = 23;
    public static final int SpindexerLaserID = 24;
    public static final double SpindexerMotorSpeed = 0.5;
    public static final double SpindexerDistance = 100;
  }
}
