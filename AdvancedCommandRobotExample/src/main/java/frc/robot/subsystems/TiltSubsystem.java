package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TiltSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TiltSubsystem extends PIDSubsystem {
  CANSparkMax tiltRightMotor = new CANSparkMax(Constants.TILT_RIGHT_MOTOR, MotorType.kBrushless);
  CANSparkMax tiltLeftMotor = new CANSparkMax(Constants.TILT_LEFT_MOTOR, MotorType.kBrushless);

  public TiltSubsystem() {
    super(
        // The controller that the command will use
        new PIDController(Constants.Pivot_kP, Constants.Pivot_kI, Constants.Pivot_kD));
    getController().setTolerance(0.1);
    tiltLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
    tiltRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
  }

  public double getRightHeight() {
    // SmartDashboard.putNumber("Position is",
    // tiltRightMotor.getEncoder().getPosition());
    return Math.abs(tiltRightMotor.getEncoder().getPosition());
  }

  public double getLeftHeight() {
    // SmartDashboard.putNumber("Position is",
    // tiltLeftMotor.getEncoder().getPosition());
    return Math.abs(tiltLeftMotor.getEncoder().getPosition());
  }

  public double getSpeed() {
    return tiltLeftMotor.get();
  }

  public void resetTiltEncoders() {
    tiltLeftMotor.getEncoder().setPosition(0);
    tiltRightMotor.getEncoder().setPosition(0);
  }

  public void set(double val) {
    tiltLeftMotor.set(val);
    tiltRightMotor.set(-1 * val);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    if ((getRightHeight() > 21) && (setpoint > 21)) {
      tiltLeftMotor.set(0);
      tiltRightMotor.set(0);
    } else {
      tiltLeftMotor.set(-1 * (output + getController().calculate(getMeasurement(), setpoint)));
      tiltRightMotor.set(output + getController().calculate(getMeasurement(), setpoint));
    }
  }

  @Override
  protected double getMeasurement() {
    return RobotContainer.tiltSubsystem.getRightHeight();
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }
}
