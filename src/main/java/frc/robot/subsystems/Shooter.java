package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX shooterMotor = new TalonFX(75);

  private double speed;

  private final MotionMagicVelocityVoltage shooterVelocity = new MotionMagicVelocityVoltage(0);

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

  public void setSpeed(double speed) {
    shooterMotor.setControl(shooterVelocity.withVelocity(speed));
  }

  public void stop() {
    shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter speed", shooterMotor.get());
  }
}
