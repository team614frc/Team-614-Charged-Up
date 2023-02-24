package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

  CANSparkMax elevatorRightMotor = null;
  CANSparkMax elevatorLeftMotor = null;

  public ElevatorSubsystem() {
    elevatorLeftMotor = new CANSparkMax(Constants.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
    elevatorLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
  }

  public void periodic() {
    // Called once per scheduler run
  }

  public double getHeight() {
    SmartDashboard.putNumber("Position is", elevatorLeftMotor.getEncoder().getPosition());
    return elevatorLeftMotor.getEncoder().getPosition();
  }

  public double getInches() {
    double motorRevolutions = elevatorLeftMotor.getEncoder().getPosition() / 42;
    double gearBoxOutputRevolutions = motorRevolutions * Constants.kGearRatio;
    double inchesTraveled = gearBoxOutputRevolutions * Constants.M_PI * 6;
    return inchesTraveled;
  }

  public void set(double val) {
    elevatorLeftMotor.set(val);
  }
}