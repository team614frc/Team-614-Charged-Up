package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Manipulator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Manipulator extends SubsystemBase {
    /*Creates a new intake subsystem */ 
  PowerDistribution pdh;
  double spikeThreshold = Constants.MANIPULATOR_THRESHOLD;
  CANSparkMax intakeMotor = null;
  Encoder encoder;

  public Manipulator(){
    intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    encoder = new Encoder(0, 1);
    pdh = new PowerDistribution();
    /*We might not need the two motors for the gripper to follow each others movements in the future, as there is a possibility
    that we might need them to not match, in order to spin the game pieces onto their designated
    scoring locations*/
    
    intakeMotor.setSmartCurrentLimit(40);
}

@Override
public void periodic() {
  // This method will be called once per scheduler run
  //Periodically gets current motor is giving off, when the values exceeds the threshold specified, motors stop
if (pdh.getCurrent(Constants.INTAKE_MOTOR) > spikeThreshold) {
  intakeMotor.set(Constants.STOP_MOTOR);
}
}
//Returns rate of motor
public double getSpeed()
{
  return intakeMotor.get();
}
//Speed of motor
public void set (double val)
{
  intakeMotor.set(val);
  System.out.println(intakeMotor.getOutputCurrent());
}
}
