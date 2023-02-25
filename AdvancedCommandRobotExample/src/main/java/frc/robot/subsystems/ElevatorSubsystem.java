package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

  CANSparkMax elevatorRightMotor = null;
  CANSparkMax elevatorLeftMotor = null;

  public ElevatorSubsystem() {
    elevatorRightMotor = new CANSparkMax(Constants.ELEVATOR_RIGHT_MOTOR,
      MotorType.kBrushless);
    elevatorLeftMotor = new CANSparkMax(Constants.ELEVATOR_LEFT_MOTOR, 
      MotorType.kBrushless);

    elevatorLeftMotor.restoreFactoryDefaults();
    elevatorRightMotor.restoreFactoryDefaults();
    // test out
    elevatorRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
    elevatorLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
  }

  public void periodic() {
    // Called once per scheduler run
  SmartDashboard.putNumber("Current left motor tick position:", elevatorLeftMotor.getEncoder().getPosition());
  SmartDashboard.putNumber("Current right motor tick position:", elevatorRightMotor.getEncoder().getPosition());
  SmartDashboard.putNumber("Current elevator speed:", elevatorLeftMotor.get());
  }

  public double getHeight() {
    SmartDashboard.putNumber("Position is", elevatorLeftMotor.getEncoder().getPosition());
    return Math.abs(elevatorLeftMotor.getEncoder().getPosition());
  }

  public double getRightHeight() {
    SmartDashboard.putNumber("Position is", elevatorRightMotor.getEncoder().getPosition());
    return Math.abs(elevatorRightMotor.getEncoder().getPosition());
  }

  public double getLeftHeight() {
    SmartDashboard.putNumber("Position is", elevatorLeftMotor.getEncoder().getPosition());
    return elevatorLeftMotor.getEncoder().getPosition();
  }
  public void resetElevatorEncoders() {
    elevatorLeftMotor.getEncoder().setPosition(0);
    elevatorRightMotor.getEncoder().setPosition(0);
  }
  
  public double getInches(){
    double motorRevolutions = elevatorLeftMotor.getEncoder().getPosition()/Constants.TICKS_PER_REV;
    double gearBoxOutputRevolutions = motorRevolutions * Constants.GEAR_BOX_RATIO;
    double inchesTraveled = gearBoxOutputRevolutions * Constants.M_PI * Constants.WHEEL_DIAMETER;
    return inchesTraveled;
  }
  public void set(double val){
    elevatorLeftMotor.set(val);
    elevatorRightMotor.set(-1*val);
  }
}