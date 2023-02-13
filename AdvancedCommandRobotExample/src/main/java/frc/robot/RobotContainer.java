package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.TiltSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomous.TimedBasedAuto.TestAuto;
import frc.robot.commands.PIDCommand.ElevatorPIDCommand;
import frc.robot.commands.PIDCommand.ManipulatorPIDCommand;
import frc.robot.commands.PIDCommand.TiltPID;

public class RobotContainer {
  // Subsystem Initalization
  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  public static Timer autoTimer;
  public static CommandXboxController m_CommandXboxController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  public static Manipulator manipulator = new Manipulator();
  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static TiltSubsystem tiltSubsystem = new TiltSubsystem();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final Command TestAuto = new TestAuto();

  public RobotContainer() {
    configureBindings();
    // Sets the default command for drivetrain subsystem
    driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());

    // Add commands to the auto chooser
    // m_chooser.addOption("Score_Mobility_Charge_Station_Engaged",
    // scoreMobilityChargeStationEngaged);
    m_chooser.setDefaultOption("Test Auto", TestAuto);

    // Puts the auto chooser on the dashboard
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putNumber("Drivetrain Encoder Position", driveTrainSubsystem.getPosition());
  }

  private void configureBindings() {
    // Configures the bindings for the xbox controller
    // Should probably be changed to .onTrue() instead of .whileTrue()
    m_CommandXboxController.button(Constants.LEFT_BUMPER).whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT));
    m_CommandXboxController.button(Constants.LEFT_STICK_PRESS).whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT2));
    m_CommandXboxController.button(Constants.Y_BUTTON).whileTrue(new ElevatorPIDCommand(Constants.ELEVATOR_SETPOINT));
    m_CommandXboxController.button(Constants.X_BUTTON).whileTrue(new ElevatorPIDCommand(Constants.ELEVATOR_SETPOINT2));
    m_CommandXboxController.button(Constants.RIGHT_BUMPER).whileTrue(new TiltPID(Constants.TILT_UP_SETPOINT));
    m_CommandXboxController.button(Constants.RIGHT_STICK_PRESS).whileTrue(new TiltPID(Constants.TILT_DOWN_SETPOINT));
  }

  public Command getAutonomousCommand() {
    // Returns the selected auto command
    return m_chooser.getSelected();
  }
}
