package frc.robot.subsystems;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    REVPhysicsSim.getInstance().addSparkMax(intakeRightMotor, DCMotor.getNEO(2));
    initSmartdashboard();
}

  public void set (double val)
  {
    intakeRightMotor.set(val);
  }

private void initSmartdashboard() {
  Shuffleboard.getTab("Intake").add(this);
  Shuffleboard.getTab("Intake").add("Intake Right Motor Output", intakeRightMotor.get());
}


@Override
public void periodic() {
  // This method will be called once per scheduler run

}

@Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
}


}
