package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    RobotContainer.elevatorSubsystem.resetElevatorEncoders();
    RobotContainer.tiltSubsystem.resetTiltEncoders();
    RobotContainer.driveTrainSubsystem.resetEncoderValues();
  }

  @Override
  public void robotPeriodic() {
    // Runs scheduled commands
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    
    RobotContainer.driveTrainSubsystem.navx.reset();
    RobotContainer.tiltSubsystem.resetTiltEncoders();
    RobotContainer.driveTrainSubsystem.resetEncoderValues();
    RobotContainer.elevatorSubsystem.resetElevatorEncoders();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // Runs an auto command in auto mode if there is one selected
    if (m_autonomousCommand != null)
      m_autonomousCommand.schedule();
  }

  public void autonomousPeriodic() {
  }

  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    // Cancles any scheduled auto commands in teleop mode
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    // Cancles all scheduled auto commands in test mode
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Current ticks of elevator", RobotContainer.elevatorSubsystem.getHeight());
  }

  @Override
  public void testExit() {
  }
}
