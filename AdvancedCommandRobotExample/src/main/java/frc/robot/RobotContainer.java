package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public static CommandXboxController m_CommandXboxController = new CommandXboxController(
      Constants.DRIVER_CONTROLLER_PORT);
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
    m_chooser.addOption("Straight Path",
        loadPathPlannerTrajectoryToRamseteCommand("pathplanner/generatedJSON/New Path.wpilib.json", true));

    // Puts the auto chooser on the dashboard
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putNumber("Drivetrain Average Encoder Position", driveTrainSubsystem.getAverageEncoderPosition());
  }

  public Command loadPathPlannerTrajectoryToRamseteCommand(String filename, boolean resetOdomtry) {
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Unable to open trajectory " + filename, exception.getStackTrace());
      System.out.println("Unable to read from file " + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, driveTrainSubsystem::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics, driveTrainSubsystem::getWheelSpeeds, new PIDController(Constants.V_kP, 0, 0),
        new PIDController(Constants.V_kP, 0, 0), driveTrainSubsystem::DifferentialDriveVolts, driveTrainSubsystem);

    if (resetOdomtry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> driveTrainSubsystem.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    } else {
      return ramseteCommand;
    }
  }

  private void configureBindings() {
    // Configures the bindings for the xbox controller
    // Should probably be changed to .onTrue() instead of .whileTrue()
    m_CommandXboxController.button(Constants.LEFT_BUMPER)
        .whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT));
    m_CommandXboxController.button(Constants.LEFT_STICK_PRESS)
        .whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT2));
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
