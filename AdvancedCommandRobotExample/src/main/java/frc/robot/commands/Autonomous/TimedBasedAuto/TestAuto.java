package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.commands.PIDCommand.TiltPIDCommand;
import frc.robot.commands.SimpleCommands.Extend;
import frc.robot.commands.SimpleCommands.Intake;
import frc.robot.commands.SimpleCommands.Retract;

public class TestAuto extends SequentialCommandGroup {

  public TestAuto() {

    addCommands(
        new TiltPIDCommand(Constants.TILT_HIGH_CUBE_SETPOINT).withTimeout(0.5),
        new Extend().withTimeout(0.5),
        new Intake(Constants.MANIPULATOR_SPEED_BLEH).withTimeout(0.5),
        new Retract().withTimeout(0.5),
        new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT).withTimeout(0.5),
        new PathPlannerCommand("pathplanner/generatedJSON/idk.wpilib.json", true).withTimeout(15));
  }
}
