package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadPathCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCube;

public class Auto1 extends SequentialCommandGroup {
  public Auto1() {
    addCommands(
      new ScoreMidCube(),
      new PathPlannerLoadPathCommand("pathplanner/generatedJSON/BBCMMCS.wpilib.json", true)
    );
  }
}
