package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadPathCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCube;

public class Auto4 extends SequentialCommandGroup {
  public Auto4() {
    addCommands(
      new ScoreMidCube(),
      new PathPlannerLoadPathCommand("pathplanner/generatedJSON/RBCMMCS.wpilib.json", true)
    );
  }
}
