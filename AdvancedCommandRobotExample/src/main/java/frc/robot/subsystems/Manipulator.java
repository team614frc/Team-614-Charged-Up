package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Manipulator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Manipulator extends SubsystemBase {
  /* Creates a new intake subsystem */
  //public PowerDistribution pdh;
  //CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
  CANSparkMax intakeMotor;

  public Manipulator(){
    intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    //pdh = new PowerDistribution();
    
    intakeMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
  }

@Override
public void periodic() {
  //This method will be called once per scheduler run
  //Periodically gets current motor is giving off, when the values exceeds the threshold specified, motors stop
// if ((pdh.getCurrent(Constants.INTAKE_MOTOR) >= Constants.MANIPULATOR_THRESHOLD) && Timer.getFPGATimestamp() > 1) {
//   intakeMotor.set(Constants.MOTOR_ZERO_SPEED);
//   SmartDashboard.putBoolean("Picked Up game piece:", true);
// }
// else {
//   SmartDashboard.putBoolean("Picked Up game piece:", false);
// }
// SmartDashboard.putNumber("Current Outputted by PDH:", pdh.getCurrent(Constants.INTAKE_MOTOR));
}

//   // Returns rate of motor
  public double getSpeed() {
    return intakeMotor.get();
  }

//   // Speed of motor
  public void set(double val) {
    intakeMotor.set(val);
    SmartDashboard.putNumber("Motor Speed", intakeMotor.get());

  }
}