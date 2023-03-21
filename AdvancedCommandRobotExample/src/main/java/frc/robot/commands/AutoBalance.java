package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoBalance extends CommandBase {
  private double error;
  private double currentAngle;
  private double lastAngle = 0;
  private double drivePower;
  private long balanceTimeMili = 0;
  private double ForwardMult = 1.5; // must have its own max speed
  private double maxSpeed = 0.5;
  private double diferenceInAngle;
  double stopAngle = 10.0;

  public AutoBalance() {
    addRequirements(RobotContainer.driveTrainSubsystem);
  }

  public long getMiliSeconds() {
    return System.currentTimeMillis();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balanceTimeMili = getMiliSeconds();
    lastAngle = -RobotContainer.driveTrainSubsystem.getPitch();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("AutoBalanceStopAngle", stopAngle);

    // sets angle to roll: angle the balence beam can rotate.
    this.currentAngle = Math.abs(RobotContainer.driveTrainSubsystem.getPitch());

    RobotContainer.driveTrainSubsystem.arcadeDrive(0.3, 0);

    // System.out.println("drivePower*FM: "+(drivePower*ForwardMult)+"angle:
    // "+currentAngle+" ForwardMult:"+ForwardMult+" difInAngle: "+diferenceInAngle+"
    // maxSpeed: "+maxSpeed);
    SmartDashboard.putNumber("drivePower*FM", (drivePower * ForwardMult));
    SmartDashboard.putNumber("pitch balance angle", currentAngle);
    SmartDashboard.putNumber("ForwardMult", ForwardMult);
    SmartDashboard.putNumber("difInAngle", diferenceInAngle);
    SmartDashboard.putNumber("AutoBalance maxSpeed", maxSpeed);
    SmartDashboard.putBoolean("balancing", true);

    this.lastAngle = currentAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    SmartDashboard.putBoolean("balancing", false);
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("balanceTimeMili Milisecs", (getMiliSeconds() - balanceTimeMili));

    // not balenced, reset timer
    // if (){
    // balanceTimeMili = getMiliSeconds();
    // }

    // if balenced for 2 secs, lock motors and finish
    // return ((getMiliSeconds()-balanceTimeMili) > 0.5);
    return (currentAngle < 10);// 12
    // return ( m_driveTrain.getNavxDisplacement() >= Units.inchesToMeters(9.25));
  }
}