package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.TiltSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TiltSubsystem extends SubsystemBase {
  /** Creates a new TiltSubsystem. */
  //CANSparkMax tiltRightMotor = null;
  CANSparkMax tiltLeftMotor = null;

  public TiltSubsystem() {
  //tiltRightMotor = new CANSparkMax(Constants.TILT_RIGHT_MOTOR, MotorType.kBrushless);
  tiltLeftMotor = new CANSparkMax(Constants.TILT_LEFT_MOTOR, MotorType.kBrushless);

  //tiltRightMotor.follow(tiltLeftMotor);

    // tiltRightMotor.setSmartCurrentLimit(Constants.ELEVATOR_CURRENT_LIMIT);
    tiltLeftMotor.setSmartCurrentLimit(Constants.ELEVATOR_CURRENT_LIMIT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getHeight() {
    SmartDashboard.putNumber("Position is", tiltLeftMotor.getEncoder().getPosition());
    return tiltLeftMotor.getEncoder().getPosition();
  }

  public void set(double val) {
    tiltLeftMotor.set(val);
  }
}
