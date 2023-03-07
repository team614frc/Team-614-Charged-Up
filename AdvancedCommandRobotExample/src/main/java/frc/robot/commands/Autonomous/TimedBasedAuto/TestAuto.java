package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.commands.PIDCommand.TiltPIDCommand;
import frc.robot.commands.SimpleCommands.Intake;

public class TestAuto extends SequentialCommandGroup {

  public TestAuto() {
    // Auto for testing purposes
    // To change
    addCommands(
      new TiltPIDCommand(Constants.TILT_MID_SCORE_SETPOINT).withTimeout(0.5),
      new Intake(Constants.MANIPULATOR_SPEED_BLEH).withTimeout(0.5),
      new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT).withTimeout(0.5),
      // new RobotContainer().loadPathplannerTrajectoryToRamseteCommand("pathplanner/generatedJSON/inch.wpilib.json", true).withTimeout(3));
      new PathPlannerCommand("pathplanner/generatedJSON/idk.wpilib.json", true).withTimeout(15));
    }
}
