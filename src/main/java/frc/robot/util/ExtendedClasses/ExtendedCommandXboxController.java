package frc.robot.util.ExtendedClasses;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ExtendedCommandXboxController extends CommandXboxController {
  public ExtendedCommandXboxController(int port) {
    super(port);
  }

  public Command rumbleFor(RumbleType type, double value, double seconds) {
    return Commands.sequence(
            Commands.runOnce(() -> getHID().setRumble(type, value)),
            Commands.waitSeconds(seconds),
            Commands.runOnce(() -> getHID().setRumble(type, 0)))
        .ignoringDisable(false);
  }
}
