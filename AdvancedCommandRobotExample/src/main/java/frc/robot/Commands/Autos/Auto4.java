package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PathPlannerCommands.PathPlannerLoadPathCommand;
import frc.robot.Commands.SequentialParallelCommands.ScoreMidCube;

public class Auto4 extends SequentialCommandGroup {
  public Auto4() {
    addCommands(
        new ScoreMidCube(),
        new PathPlannerLoadPathCommand("pathplanner/generatedJSON/RBCMMCS.wpilib.json", true));
  }
}
