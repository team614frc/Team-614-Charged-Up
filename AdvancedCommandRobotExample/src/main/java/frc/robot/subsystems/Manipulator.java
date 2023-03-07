package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Manipulator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Manipulator extends SubsystemBase {
  /* Creates a new intake subsystem */
  public PowerDistribution pdh;
  CANSparkMax intakeMotor;

  public Manipulator() {
    intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    pdh = new PowerDistribution();

    intakeMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // // Returns rate of motor
  public double getSpeed() {
    return intakeMotor.get();
  }

  // // Speed of motor
  public void set(double val) {
    intakeMotor.set(val);
    SmartDashboard.putNumber("Motor Speed", intakeMotor.get());

  }
}