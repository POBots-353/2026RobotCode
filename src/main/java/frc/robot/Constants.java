// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

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
  }

  public static class SwerveConstants {
    public static final LinearVelocity maxTranslationalSpeed = FeetPerSecond.of(15);
    public static final LinearVelocity slowModeMaxTranslationalSpeed = FeetPerSecond.of(5);
    public static final AngularVelocity maxRotationalSpeed = RotationsPerSecond.of(1.5);

    public static final Time translationZeroToFull = Seconds.of(0.6);
    public static final Time rotationZeroToFull = Seconds.of(0.25);

    public static final LinearAcceleration maxTransationalAcceleration =
        maxTranslationalSpeed.div(translationZeroToFull);
    public static final AngularAcceleration maxAngularAcceleration =
        maxRotationalSpeed.div(rotationZeroToFull);
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

    public static final Pose2d hubBlueAlliance = new Pose2d(4.625594, 4.03479, Rotation2d.kZero);
    public static final Pose2d hubRedAlliance = new Pose2d(11.915394, 4.03479, Rotation2d.kZero);

  }
}
