package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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
import com.ctre.phoenix6.signals.GravityTypeValue;
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
    public static InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapScoring =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public static InterpolatingDoubleTreeMap shooterSpeedMapScoring =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap timeOfFlightMapScoring =
        new InterpolatingDoubleTreeMap();

    public static InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapFerrying =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public static InterpolatingDoubleTreeMap shooterSpeedMapFerrying =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap timeOfFlightMapFerrying =
        new InterpolatingDoubleTreeMap();

    public static InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapSim =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public static InterpolatingDoubleTreeMap shooterSpeedMapSim = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap timeOfFlightMapSim = new InterpolatingDoubleTreeMap();

    static {
      hoodAngleMapSim.put(1.681, Rotation2d.fromDegrees(21.448));
      hoodAngleMapSim.put(2.016, Rotation2d.fromDegrees(21.972));
      hoodAngleMapSim.put(2.338, Rotation2d.fromDegrees(22.648));
      hoodAngleMapSim.put(2.665, Rotation2d.fromDegrees(24.122));
      hoodAngleMapSim.put(3.034, Rotation2d.fromDegrees(25.658));
      hoodAngleMapSim.put(3.363, Rotation2d.fromDegrees(26.333));
      hoodAngleMapSim.put(3.711, Rotation2d.fromDegrees(27.562));
      hoodAngleMapSim.put(4.061, Rotation2d.fromDegrees(29.036));
      hoodAngleMapSim.put(4.324, Rotation2d.fromDegrees(29.835));
      hoodAngleMapSim.put(4.619, Rotation2d.fromDegrees(30.449));
      hoodAngleMapSim.put(5.010, Rotation2d.fromDegrees(32.046));
      hoodAngleMapSim.put(5.366, Rotation2d.fromDegrees(33.275));
      hoodAngleMapSim.put(5.610, Rotation2d.fromDegrees(34.319));
      hoodAngleMapSim.put(6.051, Rotation2d.fromDegrees(35.794));
      hoodAngleMapSim.put(6.369, Rotation2d.fromDegrees(36.408));
      hoodAngleMapSim.put(6.704, Rotation2d.fromDegrees(37.002));
      hoodAngleMapSim.put(7.022, Rotation2d.fromDegrees(36.531));

      shooterSpeedMapSim.put(1.681, 18.155);
      shooterSpeedMapSim.put(2.016, 18.611);
      shooterSpeedMapSim.put(2.338, 19.516);
      shooterSpeedMapSim.put(2.665, 20.121);
      shooterSpeedMapSim.put(3.034, 21.027);
      shooterSpeedMapSim.put(3.363, 21.629);
      shooterSpeedMapSim.put(3.711, 22.233);
      shooterSpeedMapSim.put(4.061, 22.981);
      shooterSpeedMapSim.put(4.324, 23.583);
      shooterSpeedMapSim.put(4.619, 24.037);
      shooterSpeedMapSim.put(5.010, 24.793);
      shooterSpeedMapSim.put(5.366, 25.549);
      shooterSpeedMapSim.put(5.610, 25.850);
      shooterSpeedMapSim.put(6.051, 26.455);
      shooterSpeedMapSim.put(6.369, 27.057);
      shooterSpeedMapSim.put(6.704, 27.661);
      shooterSpeedMapSim.put(7.022, 28.265);

      timeOfFlightMapSim.put(1.695, 0.893);
      timeOfFlightMapSim.put(2.016, 0.924);
      timeOfFlightMapSim.put(2.338, 0.985);
      timeOfFlightMapSim.put(2.665, 1.013);
      timeOfFlightMapSim.put(3.034, 1.059);
      timeOfFlightMapSim.put(3.363, 1.093);
      timeOfFlightMapSim.put(3.711, 1.118);
      timeOfFlightMapSim.put(4.061, 1.147);
      timeOfFlightMapSim.put(4.324, 1.175);
      timeOfFlightMapSim.put(4.619, 1.195);
      timeOfFlightMapSim.put(5.010, 1.216);
      timeOfFlightMapSim.put(5.366, 1.242);
      timeOfFlightMapSim.put(5.610, 1.241);
      timeOfFlightMapSim.put(6.051, 1.249);
      timeOfFlightMapSim.put(6.365, 1.273);
      timeOfFlightMapSim.put(6.704, 1.295);
      timeOfFlightMapSim.put(7.022, 1.341);
    }

    static {
        hoodAngleMapScoring.put(1.777, Rotation2d.fromDegrees(21.448));
        hoodAngleMapScoring.put(2.44, Rotation2d.fromDegrees(22.25));
        hoodAngleMapScoring.put(3.28, Rotation2d.fromDegrees(23.62));
        hoodAngleMapScoring.put(3.805, Rotation2d.fromDegrees(25.18));
        hoodAngleMapScoring.put(3.877, Rotation2d.fromDegrees(24.59));
        hoodAngleMapScoring.put(4.34, Rotation2d.fromDegrees(26.64));
        hoodAngleMapScoring.put(5.272, Rotation2d.fromDegrees(26.64));
        hoodAngleMapScoring.put(5.478, Rotation2d.fromDegrees(27.23));
        hoodAngleMapScoring.put(6.285, Rotation2d.fromDegrees(30.65));
        hoodAngleMapScoring.put(2.247, Rotation2d.fromDegrees(21.45));
        hoodAngleMapScoring.put(2.5405, Rotation2d.fromDegrees(22.54));
        hoodAngleMapScoring.put(2.33, Rotation2d.fromDegrees(22.25));
        hoodAngleMapScoring.put(4.842, Rotation2d.fromDegrees(27.52));
        hoodAngleMapScoring.put(3.315, Rotation2d.fromDegrees(24.10));
        hoodAngleMapScoring.put(5.207, Rotation2d.fromDegrees(27.23));

        shooterSpeedMapScoring.put(1.777, 32.51);
        shooterSpeedMapScoring.put(2.44, 35.61);
        shooterSpeedMapScoring.put(3.28, 41.11);
        shooterSpeedMapScoring.put(3.805, 41.11);
        shooterSpeedMapScoring.put(4.34, 47.80);
        shooterSpeedMapScoring.put(5.272, 51.14);
        shooterSpeedMapScoring.put(5.478, 51.14);
        shooterSpeedMapScoring.put(6.285, 53.05);
        shooterSpeedMapScoring.put(2.247, 35.14);
        shooterSpeedMapScoring.put(2.5405, 38.00);
        shooterSpeedMapScoring.put(2.33, 36.09);
        shooterSpeedMapScoring.put(3.877, 47.32);
        shooterSpeedMapScoring.put(4.842, 49.47);
        shooterSpeedMapScoring.put(3.315, 43.74);
        shooterSpeedMapScoring.put(5.312075, 51.14);

        timeOfFlightMapScoring.put(1.777, 1.1);
        timeOfFlightMapScoring.put(2.44, 1.12);
        timeOfFlightMapScoring.put(3.28, 1.3);
        timeOfFlightMapScoring.put(3.805, 1.33);
        timeOfFlightMapScoring.put(4.34, 1.44);
        timeOfFlightMapScoring.put(5.272, 1.60);
        timeOfFlightMapScoring.put(5.478, 1.57);
        timeOfFlightMapScoring.put(6.285, 1.50);
        timeOfFlightMapScoring.put(2.247, 1.20);
        timeOfFlightMapScoring.put(2.5405, 1.20);
        timeOfFlightMapScoring.put(2.33, 1.10);
        timeOfFlightMapScoring.put(3.87, 1.40);
        timeOfFlightMapScoring.put(4.842, 1.50);
        timeOfFlightMapScoring.put(3.315, 1.30);
        timeOfFlightMapScoring.put(5.312, 1.59);

      hoodAngleMapScoring.put(1.667, Rotation2d.fromDegrees(21.45));
      hoodAngleMapScoring.put(2.14, Rotation2d.fromDegrees(21.45));
      hoodAngleMapScoring.put(2.412, Rotation2d.fromDegrees(21.45));
      hoodAngleMapScoring.put(2.735, Rotation2d.fromDegrees(22.85));
      hoodAngleMapScoring.put(3.046, Rotation2d.fromDegrees(23.35));
    //   hoodAngleMapScoring.put(3.467, Rotation2d.fromDegrees(23.35));
    //   hoodAngleMapScoring.put(3.725, Rotation2d.fromDegrees(24.24));
    //   hoodAngleMapScoring.put(3.963, Rotation2d.fromDegrees(25.13));
    //   hoodAngleMapScoring.put(4.187, Rotation2d.fromDegrees(25.43));
    //   hoodAngleMapScoring.put(4.5086, Rotation2d.fromDegrees(25.43));
    //   hoodAngleMapScoring.put(4.780, Rotation2d.fromDegrees(25.43));
    //   hoodAngleMapScoring.put(5.172, Rotation2d.fromDegrees(25.43));
    //   hoodAngleMapScoring.put(5.501, Rotation2d.fromDegrees(25.82));
    //   hoodAngleMapScoring.put(5.848, Rotation2d.fromDegrees(26.12));

      shooterSpeedMapScoring.put(1.667, 34.18);
      shooterSpeedMapScoring.put(2.14, 36.14);
      shooterSpeedMapScoring.put(2.412, 40.53);
      shooterSpeedMapScoring.put(2.735, 41.02);
      shooterSpeedMapScoring.put(3.046, 41.99);
    //   shooterSpeedMapScoring.put(3.467, 43.21);
    //   shooterSpeedMapScoring.put(3.725, 43.95);
    //   shooterSpeedMapScoring.put(3.963, 44.92);
    //   shooterSpeedMapScoring.put(4.187, 45.51);
    //   shooterSpeedMapScoring.put(4.5086, 45.65);
    //   shooterSpeedMapScoring.put(4.780, 45.65);
    //   shooterSpeedMapScoring.put(5.172, 46.87);
    //   shooterSpeedMapScoring.put(5.501, 47.36);
    //   shooterSpeedMapScoring.put(5.848, 48.83);

      timeOfFlightMapScoring.put(1.667, 0.95);
      timeOfFlightMapScoring.put(2.14, 1.05);
      timeOfFlightMapScoring.put(2.412, 1.25);
      timeOfFlightMapScoring.put(2.735, 1.25);
      timeOfFlightMapScoring.put(3.046, 1.25);
      timeOfFlightMapScoring.put(3.467, 1.40);
      timeOfFlightMapScoring.put(3.725, 1.39);
      timeOfFlightMapScoring.put(3.963, 1.41);
    //   timeOfFlightMapScoring.put(4.187, 1.50);
    //   timeOfFlightMapScoring.put(4.5086, 1.35);
    //   timeOfFlightMapScoring.put(4.780, 1.39);
    //   timeOfFlightMapScoring.put(5.172, 1.40);
    //   timeOfFlightMapScoring.put(5.501, 1.37);
    //   timeOfFlightMapScoring.put(5.848, 1.48);

      // ferrying maps
      hoodAngleMapFerrying.put(6.572, Rotation2d.fromDegrees(29.28));
      hoodAngleMapFerrying.put(7.229, Rotation2d.fromDegrees(39.24));
      hoodAngleMapFerrying.put(7.79, Rotation2d.fromDegrees(42.66));
      hoodAngleMapFerrying.put(8.344, Rotation2d.fromDegrees(44.52));
      hoodAngleMapFerrying.put(9.014, Rotation2d.fromDegrees(45.49));
      hoodAngleMapFerrying.put(9.594, Rotation2d.fromDegrees(46.27));
      hoodAngleMapFerrying.put(10.13, Rotation2d.fromDegrees(46.27));
      hoodAngleMapFerrying.put(10.61, Rotation2d.fromDegrees(46.97));
      hoodAngleMapFerrying.put(11.04, Rotation2d.fromDegrees(49.01));
      hoodAngleMapFerrying.put(14.24, Rotation2d.fromDegrees(52.43));
      hoodAngleMapFerrying.put(15.66, Rotation2d.fromDegrees(50.38));

      shooterSpeedMapFerrying.put(6.572, 48.75);
      shooterSpeedMapFerrying.put(7.229, 46.37);
      shooterSpeedMapFerrying.put(7.79, 48.52);
      shooterSpeedMapFerrying.put(8.344, 50.19);
      shooterSpeedMapFerrying.put(9.014, 50.67);
      shooterSpeedMapFerrying.put(9.594, 52.82);
      shooterSpeedMapFerrying.put(10.13, 56.40);
      shooterSpeedMapFerrying.put(10.61, 58.31);
      shooterSpeedMapFerrying.put(11.04, 62.13);
      shooterSpeedMapFerrying.put(14.24, 71.69);
      shooterSpeedMapFerrying.put(15.66, 86.02);

      timeOfFlightMapFerrying.put(6.572, 1.66);
      timeOfFlightMapFerrying.put(7.229, 1.59);
      timeOfFlightMapFerrying.put(7.79, 1.40);
      timeOfFlightMapFerrying.put(8.344, 1.46);
      timeOfFlightMapFerrying.put(9.014, 1.55);
      timeOfFlightMapFerrying.put(9.594, 1.52);
      timeOfFlightMapFerrying.put(10.13, 1.50);
      timeOfFlightMapFerrying.put(10.61, 1.57);
      timeOfFlightMapFerrying.put(11.04, 1.42);
      timeOfFlightMapFerrying.put(14.24, 1.66);
      timeOfFlightMapFerrying.put(15.66, 1.71);
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

  public static class ClimbConstants {
    public static final Pose2d rightClimbPose =
        new Pose2d(0.9172529578208923, 2.905292844772339, Rotation2d.kCW_90deg);
    public static final Pose2d leftClimbPose =
        new Pose2d(1.114691138267517, 4.505441703796387, Rotation2d.kCCW_90deg);

    public static final int climberMotorID = 25;
    public static final double climbSpeed = 0.80;

    public static final Distance maxHeight = Inches.of(9);
    public static final Distance minHeight = Inches.of(0);

    public static final double drumRadiusInches = 0.5;

    public static final double climberGearBox = (25 / 1);

    public static final double climberSensorToMechanismRatio =
        climberGearBox / (2 * Math.PI * Units.inchesToMeters(drumRadiusInches));

    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(climberSensorToMechanismRatio);

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(maxHeight.in(Meters))
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(minHeight.in(Meters))
            .withReverseSoftLimitEnable(true);
    public static final CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs().withStatorCurrentLimit(45).withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration climberConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);
  }

  public static class IntakeConstants {
    public static final int armMainID = 14;
    public static final int armFollowerID = 15;
    public static final int intakeID = 16;
    // public static final int armMagnetID = 17;

    public static final double armGearRatio = 60;
    // 111.182298 - 15.025 = 96;
    public static final Angle minPosition = Degrees.of(0.0);
    public static final Angle maxPosition = Degrees.of(97.57298);

    public static final Angle downPosition = maxPosition;
    public static final Angle upPosition = minPosition;

    public static final Angle armDownPositionTolerance = maxPosition.plus(minPosition).div(2);

    public static final Angle armMagnetOffset = Rotations.of(0);

    public static final double intakeSpeed = 0.900;

    public static final double armStallCurrent = 6.7; // amps
    public static final double armStallVelocity = 0.1353; // rps
    public static final double armZeroSpeed = 0.15;

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(22))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(44));

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.11)
            .withKV(0.021)
            .withKA(0.0105)
            .withKG(0.0353)
            .withKP(40.0)
            .withKI(0.00)
            .withKD(0.1)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(armGearRatio);

    public static final MotorOutputConfigs mainMotorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    public static final MotorOutputConfigs followerMotorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final MotorOutputConfigs intakeMotorOutputConfigs =
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
        new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true);

    public static final TalonFXConfiguration armMainConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(mainMotorOutputConfigs)
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

    public static final TalonFXConfiguration armFollowerConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(followerMotorOutputConfigs)
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

    public static final TalonFXConfiguration intakeConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withMotorOutput(intakeMotorOutputConfigs);
  }

  public static class VisionConstants {
    public static final String arducamLeftName = "Arducam_Left";

    public static final Transform3d arducamLeftTransform =
        new Transform3d(
            Units.inchesToMeters(-5.970),
            Units.inchesToMeters(12.930),
            Units.inchesToMeters(15.0),
            new Rotation3d(
                0, Units.degreesToRadians(-25), Units.degreesToRadians(90))); // Pitch: 65

    public static final String arducamRightName = "Arducam_Right";

    public static final Transform3d arducamRightTransform =
        new Transform3d(
            Units.inchesToMeters(3.375),
            Units.inchesToMeters(-12.8),
            Units.inchesToMeters(9.61),
            new Rotation3d(
                0, Units.degreesToRadians(-25), Units.degreesToRadians(-90))); // Pitch: 65

    public static final String arducamBackLeftName = "Arducam_BackLeft";

    public static final Transform3d arducamBackLeftTransform =
        new Transform3d(
            Units.inchesToMeters(-9.6),
            Units.inchesToMeters(10.6),
            Units.inchesToMeters(8.319),
            new Rotation3d(
                0, Units.degreesToRadians(-25), Units.degreesToRadians(180 - 45))); // Pitch: 65

    public static final String arducamBackRightName = "Arducam_BackRight";

    public static final Transform3d arducamBackRightTransform =
        new Transform3d(
            Units.inchesToMeters(-12.147),
            Units.inchesToMeters(-13.0),
            Units.inchesToMeters(16.88),
            new Rotation3d(
                0, Units.degreesToRadians(-25), Units.degreesToRadians(180 + 45))); // Pitch: 65

    // public static final String arducamFrontName = "Arducam_Front";

    // public static final Transform3d arducamFrontTransform =
    //     new Transform3d(
    //         Units.inchesToMeters(0),
    //         Units.inchesToMeters(-1),
    //         Units.inchesToMeters(10.07),
    //         new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(0))); // Pitch:
    // 65

    public static final String arducamFuelName = "Arducam_Fuel";

    // TODO: Update this transform
    public static final Transform3d arducamFuelTransform =
        new Transform3d(
            Units.inchesToMeters(24.468),
            Units.inchesToMeters(0),
            Units.inchesToMeters(10.591),
            new Rotation3d(0, Units.degreesToRadians(-13), Units.degreesToRadians(0)));

    public static InterpolatingDoubleTreeMap fuelAreaToDistanceMap =
        new InterpolatingDoubleTreeMap();

    static {
      fuelAreaToDistanceMap.put(80640.0, 15.0); // PIXEL area - INCHES horizontal distnace
      fuelAreaToDistanceMap.put(56882.0, 18.0);
      fuelAreaToDistanceMap.put(43264.0, 20.0);
      fuelAreaToDistanceMap.put(33488.0, 23.0);
      fuelAreaToDistanceMap.put(26080.0, 26.0);
      fuelAreaToDistanceMap.put(21025.0, 29.0);
      fuelAreaToDistanceMap.put(14884.0, 33.0);
      fuelAreaToDistanceMap.put(13110.0, 37.0);
      fuelAreaToDistanceMap.put(10816.0, 40.0);
      fuelAreaToDistanceMap.put(9312.0, 43.0);
      fuelAreaToDistanceMap.put(8190.0, 46.0);
      fuelAreaToDistanceMap.put(7225.0, 49.0);
      fuelAreaToDistanceMap.put(6400.0, 52.0);
      fuelAreaToDistanceMap.put(5550.0, 56.0);
    }
  }

  public static class FieldConstants {
    public static final String aprilTagJson = "2026-rebuilt-welded";
    public static final String regalEagleAprilTagJson = "2026RegalEagleTags";

    public static final Path aprilTagJsonPath =
        Path.of(
            Filesystem.getDeployDirectory().getPath(),
            "apriltags",
            aprilTagJson + ".json");

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
    // public static final List<Pose2d> cornerFerryPoint =
    //     List.of(new Pose2d(0.0, FieldConstants.fieldWidth.in(Meters), Rotation2d.kZero));

    public static final Distance TRENCH_LENGTH = Inches.of(47);
    public static final Distance TRENCH_BUMP_X = Inches.of(181.56);
    private static final Distance TRENCH_WIDTH = Inches.of(49.86);
    private static final Distance BUMP_INSET = TRENCH_WIDTH.plus(Inches.of(12));
    private static final Distance BUMP_LENGTH = Inches.of(73);
    public static final Distance BUMP_CENTER_Y = TRENCH_WIDTH.plus(BUMP_LENGTH.div(2));

    public static final Distance TRENCH_ZONE_EXTENSION = Inches.of(70);
    public static final Distance BUMP_ZONE_EXTENSION = Inches.of(60);
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
    public static final double gearATeeth = 48;
    public static final double gearBTeeth = 50;
    public static final double turretTeeth = 85;

    public static final Angle tolerance = Degrees.of(20);

    public static final double totalGearRatio = (gearATeeth / 10) * (turretTeeth / 10);

    public static final Angle MIN_ANGLE = Degrees.of(-270.0);
    public static final Angle MAX_ANGLE = Degrees.of(90.0);

    public static final Angle encAMagnetOffset = Rotations.of(-0.517333984375);
    public static final Angle encBMagnetOffset = Rotations.of(-0.802734375);

    public static final AngularVelocity maxTurretVelocity = DegreesPerSecond.of(2.75 * 360); // 3
    public static final AngularAcceleration maxTurretAcceleration =
        DegreesPerSecondPerSecond.of(10 * 360); // 12

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxTurretVelocity)
            .withMotionMagicAcceleration(maxTurretAcceleration);

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.09)
            .withKV(0.020)
            .withKA(0.007)
            .withKP(140.3) // 1000.3
            .withKI(0.00)
            .withKD(0.45) // 13.53
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

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

    public static final int turretMotorID = 18;
    public static final int encoderAID = 19;
    public static final int encoderBID = 20;
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
    public static final int hoodMotorID = 21; // 21

    public static final double hoodStallCurrent = 10; // amps
    public static final double hoodStallVelocity = 0.1353; // rps
    public static final double hoodZeroSpeed = -0.15;

    public static final double slowHoodSpeed = 0.20;

    public static final Angle minAngle = Degrees.of(21.448);
    public static final Angle maxAngle = Degrees.of(59.231);

    public static final double hoodGearRatio =
        ((48 / 12) * (30 / 15) * (17.5 / 10) / ((maxAngle.minus(minAngle)).in(Degrees) / 360));

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(50))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100));

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.267)
            .withKV(0.05)
            .withKA(0.01)
            .withKG(0.030)
            .withKP(200.0)
            .withKI(0.01)
            .withKD(0.5)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

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
    public static final int spindexerMotorID = 22;
    public static final int kickerMotorID = 23;
    public static final double spindexerMotorSpeed = 0.90;
    public static final double kickerMotorSpeed = 0.9;

    public static final int kickerLaserID = 26;

    public static final double spindexerIdleSpeed = -0.15;
    public static final double kickerIdleSpeed = -0.15;

    public static final double reverseKickerSpeed = -0.30;
    public static final double reverseSpindexerSpeed = -0.30;

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration spindexerConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withMotorOutput(motorOutputConfigs);
  }

  public static class ShooterConstants {
    public static final int shooterMotorID = 24;

    public static final double shooterGearRatio = 1;
    public static final Distance flyWheelRadius = Inches.of(2.0);

    public static final AngularVelocity shooterSpeedTolerance = RotationsPerSecond.of(10);

    public static final AngularAcceleration motionMagicAcceleration =
        (RadiansPerSecond.of(15 / (flyWheelRadius).in(Meters)).div(Seconds.of(.5)));
    public static final double motionMagicJerk =
        motionMagicAcceleration.in(RotationsPerSecondPerSecond) * 10;

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(motionMagicAcceleration.in(RotationsPerSecondPerSecond))
            .withMotionMagicJerk(motionMagicJerk);

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.164)
            .withKV(0.124)
            .withKA(0.018)
            .withKP(0.147) // 0.15
            .withKI(0.0)
            .withKD(0.01) // 0.02
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(shooterGearRatio);

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(
                InvertedValue.CounterClockwise_Positive) // needs to spin left when wires up
            .withNeutralMode(NeutralModeValue.Coast);
    public static final CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(48).withSupplyCurrentLimitEnable(true);

    public static final TalonFXConfiguration shooterConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(motorOutputConfigs);
  }
}
