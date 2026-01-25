package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class RobotVisualization {
  private Turret turret;
  private Hood hood;

  public RobotVisualization(Turret turret, Hood hood) {
    this.turret = turret;
    this.hood = hood;
  }

  @Logged(name = "Turret")
  public Pose3d getTurretPose3d() {
    return new Pose3d(
        -0.1318, -0.15248, 0.376, new Rotation3d(0, 0, turret.getTurretAngle().in(Radians)));
  }

  @Logged(name = "Hood")
  public Pose3d getHoodPose3d() {
    return getTurretPose3d()
        .transformBy(
            new Transform3d(
                new Translation3d(0.121, -0.0025, 0.072),
                new Rotation3d(0.0, hood.getHoodAngle().in(Radians), 0.0)));
  }
}
