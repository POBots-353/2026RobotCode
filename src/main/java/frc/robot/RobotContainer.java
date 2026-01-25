// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.MoveToFuel;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.commands.TeleopSwerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.util.AllianceUtil;
import frc.robot.util.SwerveTelemetry;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private SendableChooser<Command> autoChooser;

  private int ferryPoseIndex = 0;
  private final Supplier<Pose2d> ferryPoseSupplier =
      () -> FieldConstants.blueFerryPoints.get(ferryPoseIndex);

  @Logged(name = "Swerve")
  private final Swerve swerve = TunerConstants.createDrivetrain();

  @Logged(name = "Turret")
  private final Turret turret = new Turret();

  @Logged(name = "Hood")
  private final Hood hood = new Hood();

  private final Shooter shooter = new Shooter();

  private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Drive Over Bump To Middle", swerve.driveOverBump("To Middle"));
    NamedCommands.registerCommand(
        "Drive Over Bump To Alliance", swerve.driveOverBump("To Alliance"));
    NamedCommands.registerCommand("Move To Fuel", new MoveToFuel(swerve).withTimeout(2));
    NamedCommands.registerCommand(
        "Shoot On The Move", new ShootOnTheMove(swerve, turret, hood, shooter, ferryPoseSupplier));

    NamedCommands.registerCommand(
        "Pathfind to Mid-Left Bumper", swerve.pathFindToPose(FieldConstants.midLBumperPose));
    NamedCommands.registerCommand(
        "Pathfind to Mid-Right Bumper", swerve.pathFindToPose(FieldConstants.midRBumperPose));

    NamedCommands.registerCommand("Shoot", Commands.run(() -> shooter.setSpeed(1)).withTimeout(1));

    // Configure the trigger bindings
    configureDriverBindings();
    configureOperatorBindings();
    swerve.configureAutoBuilder();

    configureAutoChooser();

    swerve.updateFerryPoseDashboard(ferryPoseIndex);

    swerve.registerTelemetry(swerveTelemetry::telemeterize);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDriverBindings() {
    Trigger slowMode = driverController.leftTrigger();
    swerve.setDefaultCommand(
        new TeleopSwerve(
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            () -> {
              if (slowMode.getAsBoolean()) {
                return SwerveConstants.slowModeMaxTranslationalSpeed;
              }
              return SwerveConstants.maxTranslationalSpeed;
            },
            swerve));

    driverController.a().whileTrue(swerve.pathFindThroughTrench());

    turret.setDefaultCommand(turret.faceTarget(AllianceUtil::getHubPose, swerve::getRobotPose));
  }

  private void configureOperatorBindings() {
    Trigger ferryMode = operatorController.leftTrigger();
    turret.setDefaultCommand(turret.faceTarget(AllianceUtil::getHubPose, swerve::getRobotPose));

    hood.setDefaultCommand(hood.aimForTarget(AllianceUtil::getHubPose, swerve::getRobotPose));

    ferryMode.whileTrue(new ShootOnTheMove(swerve, turret, hood, shooter, ferryPoseSupplier));

    operatorController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  ferryPoseIndex = (ferryPoseIndex + 1) % FieldConstants.blueFerryPoints.size();
                  swerve.updateFerryPoseDashboard(ferryPoseIndex);
                }));
    //       operatorController.a().onTrue(
    //     Commands.runOnce(() -> {
    //       ferryPoseIndex = 0;
    //       updateFerryPoseDashboard();
    //     }));

    // operatorController.b().onTrue(
    //     Commands.runOnce(() -> {
    //       ferryPoseIndex = 1;
    //       updateFerryPoseDashboard();
    //     }));

    // operatorController.x().onTrue(
    //     Commands.runOnce(() -> {
    //       ferryPoseIndex = 2;
    //       updateFerryPoseDashboard();
    //     }));
  }

  private void configureAutoChooser() {
    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption(
        "[SysID] Quasistatic Steer Forward", swerve.sysIdQuasistaticSteer(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Quasistatic Steer Reverse", swerve.sysIdQuasistaticSteer(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Dynamic Steer Forward", swerve.sysIdDynamicSteer(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Dynamic Steer Reverse", swerve.sysIdDynamicSteer(Direction.kReverse));

    autoChooser.addOption(
        "[SysID] Quasistatic Translation Forward",
        swerve.sysIdQuasistaticTranslation(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Quasistatic Translation Reverse",
        swerve.sysIdQuasistaticTranslation(Direction.kReverse));
    autoChooser.addOption(
        "[SysID] Dynamic Translation Forward", swerve.sysIdDynamicTranslation(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Dynamic Translation Reverse", swerve.sysIdDynamicTranslation(Direction.kReverse));

    autoChooser.addOption(
        "[SysID] Quasistatic Rotation Forward",
        swerve.sysIdQuasistaticRotation(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Quasistatic Rotation Reverse",
        swerve.sysIdQuasistaticRotation(Direction.kReverse));
    autoChooser.addOption(
        "[SysID] Dynamic Rotation Forward", swerve.sysIdDynamicRotation(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Dynamic Rotation Reverse", swerve.sysIdDynamicRotation(Direction.kReverse));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
