package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevationSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevationSubsystem extends SubsystemBase {
    /*Creates a new Elevation subsystem */ 
  CANSparkMax elevatorRightMotor = null;
  CANSparkMax elevatorLeftMotor = null;

  public ElevationSubsystem() {
    elevatorRightMotor = new CANSparkMax(Constants.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);
    elevatorLeftMotor = new CANSparkMax(Constants.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);

    elevatorRightMotor.follow(elevatorLeftMotor, false); //important make sure to test out

    elevatorRightMotor.setSmartCurrentLimit(40);
    elevatorLeftMotor.setSmartCurrentLimit(40);
  }
  public void periodic() {
    //Called once per scheduler run
  }
  public void set(double x){
    elevatorLeftMotor.set(x);
  }
}