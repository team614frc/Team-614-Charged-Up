package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
    /*Creates a new Elevation subsystem */ 
  Encoder encoder;
  CANSparkMax elevatorRightMotor = null;
  CANSparkMax elevatorLeftMotor = null;

  public ElevatorSubsystem(Encoder encoder) {
    elevatorRightMotor = new CANSparkMax(Constants.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);
    elevatorLeftMotor = new CANSparkMax(Constants.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
    this.encoder = encoder;

    elevatorRightMotor.follow(elevatorLeftMotor, false); //important make sure to test out

    elevatorRightMotor.setSmartCurrentLimit(40);
    elevatorLeftMotor.setSmartCurrentLimit(40);
  }
  public void periodic() {
    //Called once per scheduler run
  }
  public double getHeight(){
    return encoder.getDistance();
  }
  public void set(double val){
    elevatorLeftMotor.set(val);
  }
}