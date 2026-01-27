package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HoodShotCalculator.ShotSolution;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX shooterMotor = new TalonFX(75);

  private double speed;

  private final MotionMagicVelocityVoltage shooterVelocity = new MotionMagicVelocityVoltage(0);

  private LinearVelocity goalSpeed = MetersPerSecond.of(0);
  private LinearVelocity currentSpeed = MetersPerSecond.of(0);
  private ShotSolution currentShotSolution;

  public Shooter() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0 = new Slot0Configs();
    config.Slot0.kP = 0.12;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.12;
    config.Slot0 = slot0;
    MotionMagicConfigs mm = new MotionMagicConfigs();
    mm.MotionMagicAcceleration = 80;
    mm.MotionMagicJerk = 300;
    config.MotionMagic = mm;

    shooterMotor.getConfigurator().apply(config);
  }

  public void setSpeed(LinearVelocity speed) {
    // this should be rotations per second but ill fix later
    shooterMotor.setControl(shooterVelocity.withVelocity(speed.in(MetersPerSecond)));
  }

  public LinearVelocity getGoalSpeed() {
    return goalSpeed;
  }

  public void stop() {
    shooterMotor.stopMotor();
  }

  public void setGoalSpeed(LinearVelocity speed) {
    goalSpeed = speed;
  }

  public void logSolution(ShotSolution solution) {
    currentShotSolution = solution;
    setGoalSpeed(solution.exitVelocity());
  }

  public ShotSolution getShotSolution() {
    return currentShotSolution;
  }

  public LinearVelocity getExitVelocity() {

    return MetersPerSecond.of(9.353);
  }

  @Override
  public void periodic() {

    if (currentSpeed != goalSpeed) {
      currentSpeed = goalSpeed;
      setSpeed(currentSpeed);
    }

    SmartDashboard.putNumber("Shooter/Current speed", shooterMotor.get());
    SmartDashboard.putNumber("Shooter/Goal speed", currentSpeed.in(MetersPerSecond));
  }
}
