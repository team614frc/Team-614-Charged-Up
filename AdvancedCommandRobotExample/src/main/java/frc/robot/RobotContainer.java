// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

//import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.TiltSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomous.DrivePositionPIDCommand;
import frc.robot.commands.Autonomous.TimedBasedAuto.ChargeStationNotEngaged;
import frc.robot.commands.Autonomous.TimedBasedAuto.ScoreMobilityChargeStationEngaged;
import frc.robot.commands.Autonomous.TimedBasedAuto.ScoreMobilityChargeStationNotEngaged;
import frc.robot.commands.Autonomous.TimedBasedAuto.test;
import frc.robot.commands.PIDCommand.ElevatorPIDCommand;
import frc.robot.commands.PIDCommand.ManipulatorPIDCommand;
import frc.robot.commands.PIDCommand.TiltPID;

public class RobotContainer {
  // Encoder
  // public static Encoder elevatorEncoder = new Encoder(0, 1);
  // Subsystem Initalization
  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  public static Timer autoTimer;
  public static CommandXboxController m_CommandXboxController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  public static Manipulator manipulator = new Manipulator();
  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static TiltSubsystem tiltSubsystem = new TiltSubsystem();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final Command chargedStationNotEngaged = new ChargeStationNotEngaged();
  private final Command scoreMobilityChargeStationEngaged = new ScoreMobilityChargeStationEngaged();
  private final Command scoreMobilityChargeStationNotEngaged = new ScoreMobilityChargeStationNotEngaged();
  private final Command test = new test();

  // Timed Auto Initaliztion

  public RobotContainer() {
    configureBindings();
    // Calling Arcade Drive Comment
    driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());

    // Add commands to the autonomous command chooser
    m_chooser.addOption("Charge_Station_Not_Engaged", chargedStationNotEngaged);
    m_chooser.addOption("Score_Mobility_Charge_Station_Engaged", scoreMobilityChargeStationEngaged);
    m_chooser.addOption("Score_Mobility_Charge_Station_Not_Engaged", scoreMobilityChargeStationNotEngaged);
    m_chooser.setDefaultOption("test", test);

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);

    SmartDashboard.putNumber("Drivetrain Encoder Position", driveTrainSubsystem.getPosition());
  }


  private void configureBindings() {
    m_CommandXboxController.button(Constants.LEFT_BUMPER).whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT));
    m_CommandXboxController.button(Constants.LEFT_STICK_PRESS).whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT2));
    m_CommandXboxController.button(Constants.Y_BUTTON).whileTrue(new ElevatorPIDCommand(Constants.ELEVATOR_SETPOINT));
    m_CommandXboxController.button(Constants.X_BUTTON).whileTrue(new ElevatorPIDCommand(Constants.ELEVATOR_SETPOINT2));
    m_CommandXboxController.button(Constants.RIGHT_BUMPER).whileTrue(new TiltPID(Constants.TILT_UP_SETPOINT));
    m_CommandXboxController.button(Constants.RIGHT_STICK_PRESS).whileTrue(new TiltPID(Constants.TILT_DOWN_SETPOINT));
  }

  public Command getAutonomousCommand() {
// This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));

// This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
// Or the path can be sampled at a given point in time for custom path following

// Sample the state of the path at 1.2 seconds
PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);

// Print the velocity at the sampled time
System.out.println(exampleState.velocityMetersPerSecond);

return m_chooser.getSelected();
  }
}
