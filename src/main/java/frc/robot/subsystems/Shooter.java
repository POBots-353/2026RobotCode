package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SOTMConstants;
import frc.robot.Constants.ShooterConstants;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMotorID);

  private final MotionMagicVelocityVoltage velocityMMRequest = new MotionMagicVelocityVoltage(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);


  public Shooter() {
    shooterMotor.getConfigurator().apply(ShooterConstants.shooterConfigs);
  }

  public Command newConfigs() {
    return new DeferredCommand(
        () -> {
          double kP = SmartDashboard.getNumber("Dynamic Shooter KP", 0);
          double kV = SmartDashboard.getNumber("Dynamic Shooter KV", 0);
          return run(
              () -> {
                shooterMotor
                    .getConfigurator()
                    .apply(
                        ShooterConstants.shooterConfigs.withSlot0(
                            new Slot0Configs().withKS(0.01).withKV(kV).withKP(kP).withKI(0.0)));
              });
        },
        Set.of(this));
  }

  public Command reachDynamicVelocity() {
    return new DeferredCommand(
        () -> {
          return run(
              () ->
                  shooterMotor.setControl(
                      velocityRequest.withVelocity(
                          SmartDashboard.getNumber("Dynamic Shooter Speed", 0))));
        },
        Set.of(this));
  }

  public Command reachGoalVelocityCommand(AngularVelocity goalVelocity) {
    return run(() -> shooterMotor.setControl(velocityRequest.withVelocity(goalVelocity)));
  }

  public void reachGoalVelocity(AngularVelocity goalVelocity) {
    shooterMotor.setControl(velocityRequest.withVelocity(goalVelocity));
  }

  public void stopShooter() {
    shooterMotor.setControl(velocityRequest.withVelocity(RotationsPerSecond.of(0)));
  }

  public Command stopShooterCommand() {
    // shooterMotor.stopMotor();
    // goalSpeed = MetersPerSecond.of(0);
    return run(
        () ->
            shooterMotor.setControl(
                velocityRequest.withVelocity(linearToAngularVelocity(MetersPerSecond.of(0)))));
  }

  public AngularVelocity linearToAngularVelocity(LinearVelocity vel) {
    return RadiansPerSecond.of(
        vel.in(MetersPerSecond) / ShooterConstants.flyWheelRadius.in(Meters));
  }

  public LinearVelocity angularToLinearVelocity(AngularVelocity vel) {
    return MetersPerSecond.of(
        vel.in(RadiansPerSecond) * ShooterConstants.flyWheelRadius.in(Meters));
  }

  public AngularVelocity getCurrentVelocity() {
    return shooterMotor.getVelocity().getValue();
  }

  public boolean shooterAtSetPoint(AngularVelocity goalSpeed, AngularVelocity tolerance) {
    if (RobotBase.isSimulation()) return true;

    AngularVelocity currentSpeed = getCurrentVelocity();

    return Math.abs(currentSpeed.in(RotationsPerSecond) - goalSpeed.in(RotationsPerSecond))
        < tolerance.in(RotationsPerSecond);
  }

  public Command runMotor(double speed) {
    return run(() -> shooterMotor.set(speed));
  }

  public void stopMotor() {
    shooterMotor.set(0.0);
  }

  public Command manualInterpolatedShoot(DoubleSupplier turretToHubMeters) {
    return run(
        () ->
            shooterMotor.setControl(
                velocityRequest.withVelocity(
                    linearToAngularVelocity(
                        MetersPerSecond.of(
                            SOTMConstants.shooterSpeedMapScoring.get(
                                turretToHubMeters.getAsDouble()))))));
  }

  @Override
  public void periodic() {
    // shooterMotor.setControl(velocityRequest.withVelocity(linearToAngularVelocity(goalSpeed)));

    if (SmartDashboard.getBoolean("Data Point Mode", false)) {
      shooterMotor.setControl(
          velocityRequest.withVelocity(SmartDashboard.getNumber("Dynamic Shooter Speed", 0)));
    }

    SmartDashboard.putNumber(
        "Shooter/Current Angular Velocity", getCurrentVelocity().in(RotationsPerSecond));
    // SmartDashboard.putNumber(
    //     "Shooter/Current Linear Velocity",
    //     angularToLinearVelocity(getCurrentVelocity()).in(MetersPerSecond));
  }
}
