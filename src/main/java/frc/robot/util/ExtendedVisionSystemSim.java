package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.simulation.VisionSystemSim;

public class ExtendedVisionSystemSim extends VisionSystemSim {

  public ExtendedVisionSystemSim(String name) {
    super(name);
  }

  public void addFieldObject(String name, Pose2d pose) {
    Field2d field = getDebugField();

    field.getObject(name).setPose(pose);
  }
}
