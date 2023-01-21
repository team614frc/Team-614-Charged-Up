package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {
    /*Creates a new intake subsystem */ 
  CANSparkMax intakeRightMotor = null;
  CANSparkMax intakeLeftMotor = null;

  public IntakeSubsystem(){
    intakeRightMotor = new CANSparkMax(Constants.INTAKE_RIGHT_MOTOR, MotorType.kBrushless);
    intakeLeftMotor = new CANSparkMax(Constants.INTAKE_LEFT_MOTOR, MotorType.kBrushless);

    /*We might not need the two motors for the gripper to follow each others movements in the future, as there is a possibility
    that we might need them to not match, in order to spin the game pieces onto their designated
    scoring locations*/
    
    intakeRightMotor.follow(intakeLeftMotor,true);

    intakeRightMotor.setSmartCurrentLimit(40);
    intakeLeftMotor.setSmartCurrentLimit(40);
}

@Override
public void periodic() {
  // This method will be called once per scheduler run

}

public void set (double val)
{
  intakeRightMotor.set(val);
}
}
