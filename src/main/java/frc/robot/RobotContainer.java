// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SOTMConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.GuidedTeleopSwerve;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.util.AllianceUtil;
import frc.robot.util.ExtendedClasses.ExtendedCommandXboxController;
// import frc.robot.util.FuelSim;
import frc.robot.util.FuelSim;
import frc.robot.util.HubTracker;
import frc.robot.util.RobotVisualization;
import frc.robot.util.SwerveTelemetry;
import java.util.Set;
import java.util.function.Supplier;

public class RobotContainer {
  private final ExtendedCommandXboxController driverController =
      new ExtendedCommandXboxController(OperatorConstants.kDriverControllerPort);
  private final ExtendedCommandXboxController operatorController =
      new ExtendedCommandXboxController(OperatorConstants.kOperatorControllerPort);

  private SendableChooser<Command> autoChooser;

  private int logNumber = 0;

  private int ferryPoseIndex = 0;
  private final Supplier<Pose2d> ferryPoseSupplier =
      () ->
          FieldConstants.blueFerryPoints.stream()
              .map(AllianceUtil::flipPose)
              .toList()
              .get(ferryPoseIndex);

  private final Supplier<Pose2d> leftFerryPose =
      () -> AllianceUtil.flipPose(FieldConstants.blueFerryPoints.get(0));

  private final Supplier<Pose2d> rightFerryPose =
      () ->
          AllianceUtil.flipPose(
              FieldConstants.blueFerryPoints.get(FieldConstants.blueFerryPoints.size() - 1));

  private Pose2d goalShotTarget;
  private final Supplier<Pose2d> goalShotTargetSupplier = () -> goalShotTarget;

  private boolean canPreShoot = false;

  @Logged(name = "Swerve")
  private final Swerve swerve = TunerConstants.createDrivetrain();

  @Logged(name = "Turret")
  private final Turret turret = new Turret();

  @Logged(name = "Hood")
  private final Hood hood = new Hood();

  @Logged(name = "Shooter")
  private final Shooter shooter = new Shooter();

  @Logged(name = "Intake")
  private final Intake intake = new Intake();

  @Logged(name = "Spindexer")
  private final Spindexer spindexer = new Spindexer();

  @Logged(name = "Climber")
  private final Climber climber = new Climber();

  @Logged(name = "3D Visualization")
  private final RobotVisualization robotVisualization =
      new RobotVisualization(turret, hood, swerve, shooter, climber);

  @Logged(name = "Fuel Sim")
  private final FuelSim fuelInstance = FuelSim.getInstance();

  private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry();

  private final PowerDistribution powerDistribution = new PowerDistribution();

  private Debouncer kickerLaserDebouncer = new Debouncer(2.0);

  Trigger kickerLaserTrigger = new Trigger(() -> spindexer.kickerLaserBroken());

  Trigger onLeftSideTrigger = new Trigger(() -> swerve.onLeftSide());
  Trigger inAllianceZoneTrigger = new Trigger(() -> swerve.inAllianceZone());
  Trigger activeHubTrigger =
      new Trigger(HubTracker::isActive).or(() -> HubTracker.getMatchTime() < 0);

  Trigger shiftEndingTrigger =
      new Trigger(
          () -> HubTracker.timeRemainingInCurrentShift().orElse(Seconds.of(4)).in(Seconds) < 3.53);

  Trigger automatedShootingTrigger =
      new Trigger(() -> SmartDashboard.getBoolean("Automated Shooting Toggle", false));

  Trigger tooCloseToHubTrigger = new Trigger(() -> swerve.tooCloseToHub());

  Trigger preShiftShoot =
      new Trigger(
              () -> {
                double timeUntilActive =
                    HubTracker.timeUntilActive().orElse(Seconds.of(0)).in(Seconds);

                double distanceToHub =
                    swerve
                        .getRobotPose()
                        .getTranslation()
                        .getDistance(AllianceUtil.getHubPose().getTranslation());

                double timeOfFlight = SOTMConstants.timeOfFlightMapScoring.get(distanceToHub);

                return (timeUntilActive) <= timeOfFlight;
              })
          .and(activeHubTrigger.negate()) // only need to preshoot if not already active
          .and(inAllianceZoneTrigger) // only preshoot if in alliance zone
          .and(tooCloseToHubTrigger.negate()); // only shoot if we are far enough away

  public RobotContainer() {
    // NamedCommands.registerCommand("Drive Over Bump To Middle", swerve.driveOverBump(true));
    // NamedCommands.registerCommand("Drive Over Bump To Alliance", swerve.driveOverBump(false));
    // NamedCommands.registerCommand("Move To Fuel", new MoveToFuel(swerve).withTimeout(2));
    NamedCommands.registerCommand("Stop Swerve", swerve.stopSwerve());

    NamedCommands.registerCommand(
        "Shoot On The Move",
        new ShootOnTheMove(
                swerve,
                turret,
                hood,
                shooter,
                spindexer,
                AllianceUtil::getHubPose,
                robotVisualization,
                () -> true)
            .asProxy());

    NamedCommands.registerCommand(
        "Auto Shoot On The Move",
        new ShootOnTheMove(
                swerve,
                turret,
                hood,
                shooter,
                spindexer,
                AllianceUtil::getHubPose,
                robotVisualization,
                () -> true)
            .until(() -> spindexer.getBalls() > 7)
            .asProxy());

    // NamedCommands.registerCommand(
    //     "Pathfind to Mid-Left Bumper", swerve.pathFindToPose(FieldConstants.midLBumperPose));
    // NamedCommands.registerCommand(
    //     "Pathfind to Mid-Right Bumper", swerve.pathFindToPose(FieldConstants.midRBumperPose));

    NamedCommands.registerCommand(
        "FerryOTM",
        new ShootOnTheMove(
                swerve,
                turret,
                hood,
                shooter,
                spindexer,
                ferryPoseSupplier,
                robotVisualization,
                () -> false)
            .asProxy());

    NamedCommands.registerCommand(
        "IntakeUntilFull", Commands.waitUntil(() -> !robotVisualization.canSimIntake()).asProxy());

    NamedCommands.registerCommand(
        "Zero Intake", intake.runOnce(() -> intake.setZero()).asProxy().ignoringDisable(true));
    NamedCommands.registerCommand(
        "Zero Hood", hood.runOnce(() -> hood.zeroHood()).asProxy().ignoringDisable(true));

    NamedCommands.registerCommand("Intake In", intake.intakeToPosition(false).asProxy());
    NamedCommands.registerCommand("Intake Out", intake.intakeToPosition(true).asProxy());

    NamedCommands.registerCommand("Intake Sequence In", intake.intakeSequence(false).asProxy());
    NamedCommands.registerCommand("Intake Sequence Out", intake.intakeSequence(true).asProxy());

    NamedCommands.registerCommand("Shake Intake", intake.shakeIntakeCommand().asProxy());

    NamedCommands.registerCommand("Start Intake", intake.startIntake().asProxy());
    NamedCommands.registerCommand("Stop Intake", intake.stopIntake().asProxy());

    NamedCommands.registerCommand(
        "SOTM Until Empty",
        new ShootOnTheMove(
                swerve,
                turret,
                hood,
                shooter,
                spindexer,
                AllianceUtil::getHubPose,
                robotVisualization,
                () -> true)
            .until(() -> robotVisualization.isEmpty())
            .asProxy());

    NamedCommands.registerCommand(
        "Shoot",
        Commands.run(() -> shooter.reachGoalVelocity(RotationsPerSecond.of(1)))
            .asProxy()
            .withTimeout(1));

    configureDriverBindings();
    configureOperatorBindings();

    swerve.configureAutoBuilder();

    configureAutoChooser();

    swerve.updateFerryPoseDashboard(ferryPoseIndex);

    swerve.registerTelemetry(swerveTelemetry::telemeterize);

    if (RobotBase.isSimulation()) {
      configureFuelSim();
    }

    SmartDashboard.putData("Power Distribution", powerDistribution);

    SmartDashboard.putData(
        "Set Arm Down", intake.runOnce(() -> intake.setArmMaxPosition()).ignoringDisable(true));
    SmartDashboard.putData(
        "Set Arm Zero", intake.runOnce(() -> intake.setZero()).ignoringDisable(true));

    SmartDashboard.putData(
        "Set Hood Zero", hood.runOnce(() -> hood.zeroHood()).ignoringDisable(true));

    SmartDashboard.putData(
        "Reset Auto Ball Count",
        Commands.runOnce(() -> spindexer.zeroBalls()).ignoringDisable(true));

    SmartDashboard.putNumber("Dynamic Shooter Speed", 0);
    SmartDashboard.putNumber("Dynamic Hood Angle", HoodConstants.minAngle.in(Degrees));
    // SmartDashboard.putNumber("Dynamic Turret Angle", 0);
    SmartDashboard.putNumber("Dynamic SHooter Extra SPeed", 0);

    SmartDashboard.putNumber("Dynamic Shooter KP", 0);
    SmartDashboard.putNumber("Dynamic Shooter KV", 0);

    goalShotTarget = AllianceUtil.getHubPose();

    kickerLaserTrigger.onTrue(Commands.runOnce(() -> spindexer.addBall()));

    inAllianceZoneTrigger.onTrue(
        Commands.runOnce(() -> goalShotTarget = AllianceUtil.getHubPose()));

    onLeftSideTrigger
        .and(inAllianceZoneTrigger.negate())
        .onTrue(Commands.runOnce(() -> goalShotTarget = leftFerryPose.get()));

    onLeftSideTrigger
        .negate()
        .and(inAllianceZoneTrigger.negate())
        .onTrue(Commands.runOnce(() -> goalShotTarget = rightFerryPose.get()));

    preShiftShoot.onTrue(driverController.rumbleFor(RumbleType.kBothRumble, 1.0, 1));

    preShiftShoot.onTrue(Commands.runOnce(() -> canPreShoot = true));
    activeHubTrigger.onFalse(Commands.runOnce(() -> canPreShoot = false));
    shiftEndingTrigger.onTrue(
        driverController
            .rumbleFor(RumbleType.kBothRumble, 1.0, 3)
            .onlyIf(() -> DriverStation.isTeleop()));
    shiftEndingTrigger.onTrue(
        operatorController
            .rumbleFor(RumbleType.kBothRumble, 1.0, 3)
            .onlyIf(() -> DriverStation.isTeleop()));
  }

  private void configureFuelSim() {
    fuelInstance.spawnStartingFuel();
    fuelInstance.registerRobot(
        SwerveConstants.bumperWidth.in(Meters),
        SwerveConstants.bumperLength.in(Meters),
        SwerveConstants.bumperHeight.in(Meters),
        swerve::getRobotPose,
        swerve::getFieldSpeeds);

    fuelInstance.registerIntake(
        SwerveConstants.bumperLength.div(2).in(Meters),
        SwerveConstants.bumperLength.div(2).plus(Inches.of(8)).in(Meters),
        -SwerveConstants.bumperWidth.div(2).in(Meters),
        SwerveConstants.bumperWidth.div(2).in(Meters),
        () -> intake.isIntakeDeployed() && robotVisualization.canSimIntake(),
        robotVisualization::simIntakeFuel);

    fuelInstance.start();
    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  FuelSim.getInstance().clearFuel();
                  FuelSim.getInstance().spawnStartingFuel();
                  robotVisualization.addStartingFuel();
                  FuelSim.Hub.BLUE_HUB.resetScore();
                  FuelSim.Hub.RED_HUB.resetScore();
                })
            .withName("Reset Fuel")
            .ignoringDisable(true));

    SmartDashboard.putBoolean("Air Resistance Toggle", false);
    SmartDashboard.putBoolean("Only Count while Active", false);
    SmartDashboard.putBoolean("Automated Shooting Toggle", false);
    SmartDashboard.getBoolean("Data Point Mode", false);
  }

  private void configureDriverBindings() {
    Trigger slowMode = driverController.leftTrigger();
    Trigger driveOverrideButton = driverController.rightTrigger();
    Trigger resetHeadingButton = driverController.start().and(driverController.back());
    Trigger leftAutoClimbButton = driverController.leftBumper();
    Trigger rightAutoClimbButton = driverController.rightBumper();
    Trigger logButton = driverController.a();

    logButton.onTrue(
        Commands.runOnce(
            () -> {
              logNumber++;
              double speed = SmartDashboard.getNumber("Dynamic Shooter Speed", 0);
              double angle =
                  SmartDashboard.getNumber(
                      "Dynamic Hood Angle", HoodConstants.minAngle.in(Degrees));

              SmartDashboard.putString("DataPoints/" + logNumber, speed + " , " + angle);
            }));

    resetHeadingButton.onTrue(swerve.runOnce(swerve::seedFieldCentric).ignoringDisable(true));

    swerve.setDefaultCommand(
        new GuidedTeleopSwerve(
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            () -> {
              if (slowMode.getAsBoolean()) {
                return SwerveConstants.slowModeMaxTranslationalSpeed;
              }
              return SwerveConstants.maxTranslationalSpeed;
            },
            () -> driveOverrideButton.getAsBoolean(),
            swerve));

    // driverController.a().whileTrue(shooter.newConfigs());

    // driverController
    //     .b()
    //     .whileTrue(shooter.reachDynamicVelocity())
    //     .onFalse(shooter.stopShooterCommand());

    // swerve.setDefaultCommand(
    //     new TeleopSwerve(
    //         driverController::getLeftY,
    //         driverController::getLeftX,
    //         driverController::getRightX,
    //         () -> {
    //           if (slowMode.getAsBoolean()) {
    //             return SwerveConstants.slowModeMaxTranslationalSpeed;
    //           }
    //           return SwerveConstants.maxTranslationalSpeed;
    //         },
    //         swerve));

    // leftAutoClimbButton.whileTrue(
    //     new DriveToPose(swerve, false, () ->
    // AllianceUtil.flipPose(ClimbConstants.leftClimbPose)));

    // leftAutoClimbButton.whileTrue(
    //     new AutoClimb(
    //         driverController::getLeftY,
    //         driverController::getLeftX,
    //         () -> {
    //           if (slowMode.getAsBoolean()) {
    //             return SwerveConstants.slowModeMaxTranslationalSpeed;
    //           }
    //           return SwerveConstants.maxTranslationalSpeed;
    //         },
    //         swerve,
    //         climber,
    //         true));
    // rightAutoClimbButton.whileTrue(
    //     new AutoClimb(
    //         driverController::getLeftY,
    //         driverController::getLeftX,
    //         () -> {
    //           if (slowMode.getAsBoolean()) {
    //             return SwerveConstants.slowModeMaxTranslationalSpeed;
    //           }
    //           return SwerveConstants.maxTranslationalSpeed;
    //         },
    //         swerve,
    //         climber,
    //         false));
  }

  private void configureOperatorBindings() {
    Trigger shootButton = operatorController.rightTrigger();
    Trigger intakeSequenceButton = operatorController.leftTrigger();
    Trigger intakeRollerButton = operatorController.leftBumper();
    Trigger shakeIntakeButton = operatorController.rightBumper();
    // Trigger zeroButton = operat1 xcssxszorController.back().and(operatorController.start());
    Trigger manualSpindexerButton =
        operatorController.start().and(operatorController.back().negate());
    Trigger reverseIntakeButton =
        operatorController.back().and(operatorController.start().negate());
    Trigger climberUpButton = operatorController.a();
    Trigger climberDownButton = operatorController.x();
    Trigger manualTurretForward = operatorController.povUp();
    Trigger manualTurretLeft = operatorController.povLeft();
    Trigger manualTurretRight = operatorController.povRight();
    Trigger manualTurretBackward = operatorController.povDown();
    Trigger manualHoodUpButton = operatorController.y();
    Trigger manualHoodDownButton = operatorController.b();
    Trigger zeroHoodButton = operatorController.leftStick();
    Trigger zeroArmButton = operatorController.rightStick();
    // Trigger manualShootButton = operatorController.leftStick();

    // (activeHubTrigger.or(() -> canPreShoot))
    //     .and(automatedShootingTrigger)
    //     .and(inAllianceZoneTrigger)
    //     .and(shootButton.negate())
    //     .and(tooCloseToHubTrigger.negate())
    //     .whileTrue(
    //         new ShootOnTheMove(
    //             swerve,
    //             turret,
    //             hood,
    //             shooter,
    //             spindexer,
    //             goalShotTargetSupplier,
    //             robotVisualization));

    // if (!Constants.tuningMode) {
    //   hood.setDefaultCommand(hood.moveToAngleCommand(HoodConstants.minAngle));
    //   shooter.setDefaultCommand(shooter.stopShooterCommand());
    // }
    // idk
    hood.setDefaultCommand(
        Commands.either(
            Commands.none(),
            hood.moveToAngleCommand(HoodConstants.minAngle),
            () -> SmartDashboard.getBoolean("Data Point Mode", false)));

    shooter.setDefaultCommand(
        Commands.either(
            Commands.none(),
            shooter.stopShooterCommand(),
            () -> SmartDashboard.getBoolean("Data Point Mode", false)));

    turret.setDefaultCommand(turret.faceTarget(goalShotTargetSupplier, swerve::getRobotPose));

    spindexer.setDefaultCommand(spindexer.idleReverse());

    // intake.setDefaultCommand(intake.intakeSequence(false));
    // hood.setDefaultCommand(hood.aimForTarget(AllianceUtil::getHubPose, swerve::getRobotPose));

    shootButton.whileTrue(
        new ShootOnTheMove(
            swerve,
            turret,
            hood,
            shooter,
            spindexer,
            goalShotTargetSupplier,
            robotVisualization,
            inAllianceZoneTrigger));

    // shootButton.whileTrue(
    //     Commands.parallel(
    //         new ShootOnTheMove(
    //             swerve,
    //             turret,
    //             hood,
    //             shooter,
    //             spindexer,
    //             goalShotTargetSupplier,
    //             robotVisualization,
    //             inAllianceZoneTrigger),
    //         Commands.sequence(
    //             Commands.runOnce(() -> kickerLaserDebouncer.calculate(false)),
    //             Commands.waitUntil(
    //                 () -> kickerLaserDebouncer.calculate(!kickerLaserTrigger.getAsBoolean())),
    //             intake.shakeIntakeCommand())));

    manualTurretForward.whileTrue(turret.moveToAngleCommand(Degrees.of(0)));
    manualTurretRight.whileTrue(turret.moveToAngleCommand(Degrees.of(90)));
    manualTurretBackward.whileTrue(turret.moveToAngleCommand(Degrees.of(180)));
    manualTurretLeft.whileTrue(turret.moveToAngleCommand(Degrees.of(-90)));

    intakeSequenceButton.whileTrue(
        Commands.either(
            intake.intakeSequence(false), intake.intakeSequence(true), intake::isIntakeDeployed));

    intakeRollerButton.whileTrue(intake.toggleRollers());

    shakeIntakeButton.onTrue(intake.shakeIntakeCommand()).onFalse(intake.intakeSequence(true));

    // manualSpindexerButton
    //     .whileTrue((Constants.tuningMode) ? spindexer.manualBoth() : spindexer.manualReverse())
    //     .onFalse(spindexer.runOnce(() -> spindexer.stopBoth()));
    // idk
    manualSpindexerButton.whileTrue(
        Commands.defer(
            () ->
                SmartDashboard.getBoolean("Data Point Mode", false)
                    ? spindexer.manualBoth()
                    : spindexer.manualReverse(),
            Set.of(spindexer)));

    reverseIntakeButton.whileTrue(intake.reverseIntakeRollers()).onFalse(intake.stopIntake());

    // climberUpButton.whileTrue(climber.climberUpCommand());
    // climberDownButton.whileTrue(climber.climberDownCommand());

    manualHoodUpButton.whileTrue(hood.moveHoodCommand(true)).onFalse(hood.holdPosition());
    manualHoodDownButton.whileTrue(hood.moveHoodCommand(false)).onFalse(hood.holdPosition());

    // manualShootButton.whileTrue(
    //     shooter.manualInterpolatedShoot(() -> swerve.getTurretToHubMeters()));

    // zeroButton.onTrue(
    //     turret
    //         .resetTurretPosition()
    //         .alongWith(intake.zeroArmCommand())
    //         .alongWith(hood.zeroHoodCommand()));
    zeroHoodButton
        .whileTrue(Commands.sequence(hood.maxPosition(), hood.moveHoodCommand(false)))
        .onFalse(
            Commands.sequence(
                hood.stop(), Commands.waitSeconds(1), hood.runOnce(() -> hood.zeroHood())));

    zeroArmButton
        .whileTrue(
            Commands.sequence(
                intake.runOnce(() -> intake.setZero()), intake.run(() -> intake.moveDownManual())))
        .onFalse(
            Commands.sequence(
                intake.stopArm(),
                Commands.waitSeconds(1),
                intake.runOnce(() -> intake.setArmMaxPosition())));
  }

  private void configureAutoChooser() {
    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption(
        "[SysID] Quasistatic Steer Forward", swerve.sysIdQuasistaticSteer(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Quasistatic Steer Reverse", swerve.sysIdQuasistaticSteer(Direction.kReverse));
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
