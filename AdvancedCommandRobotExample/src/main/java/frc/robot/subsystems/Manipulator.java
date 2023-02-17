package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Manipulator;
import com.revrobotics.CANSparkMax;

public class Manipulator extends SubsystemBase {
  /* Creates a new intake subsystem */
  PowerDistribution pdh;
  double spikeThreshold = Constants.MANIPULATOR_THRESHOLD;
  CANSparkMax intakeMotor = null;


  public Manipulator(){
    //intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    pdh = new PowerDistribution();
    /*We might not need the two motors for the gripper to follow each others movements in the future, as there is a possibility
    that we might need them to not match, in order to spin the game pieces onto their designated
    scoring locations*/
    
    intakeMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
  }

@Override
public void periodic() {
  // This method will be called once per scheduler run
  //Periodically gets current motor is giving off, when the values exceeds the threshold specified, motors stop
if (pdh.getCurrent(Constants.INTAKE_MOTOR) > spikeThreshold) {
  intakeMotor.set(Constants.MOTOR_REST_BACK);
  SmartDashboard.putBoolean("Picked Up game piece:", true);
}
else {
  SmartDashboard.putBoolean("Picked Up game piece:", false);
}
SmartDashboard.putNumber("Current Outputted by PDH:", pdh.getCurrent(Constants.INTAKE_MOTOR));
}

  // Returns rate of motor
  public double getSpeed() {
    return intakeMotor.get();
  }

  // Speed of motor
  public void set(double val) {
    intakeMotor.set(val);
    SmartDashboard.putNumber("Motor Speed", intakeMotor.get());

  }
}
