// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class FuelProjectile {

  private final Pose3d startPose;
  private final double vX;
  private final double vY;
  private final double vZ;
  private final double startTime;

  private static final double g = -9.80665;

  private Pose3d currentPose = new Pose3d();
  private boolean active = true;

  public FuelProjectile(Pose3d startPose, double vX, double vY, double vZ) {
    this.startPose = startPose;
    this.vX = vX;
    this.vY = vY;
    this.vZ = vZ;
    this.startTime = Timer.getFPGATimestamp();
  }

  public void update() {
    if (!active) return;

    double t = Timer.getFPGATimestamp() - startTime;

    double x = vX * t;
    double y = vY * t;
    double z = vZ * t + 0.5 * g * t * t;

    currentPose =
        new Pose3d(
            startPose.getX() + x, startPose.getY() + y, startPose.getZ() + z, Rotation3d.kZero);

    if (currentPose.getZ() <= 0.0 || t > 3.0) {
      active = false;
    }
  }

  public Pose3d getPose() {
    return currentPose;
  }

  public boolean isActive() {
    return active;
  }
}
