package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PathPlannerCommands.PathPlannerLoadPathCommand;
import frc.robot.Commands.SequentialParallelCommands.ScoreMidCube;

public class Auto3 extends SequentialCommandGroup {
  public Auto3() {
    addCommands(
        new ScoreMidCube(),
        new PathPlannerLoadPathCommand("pathplanner/generatedJSON/BTCMM.wpilib.json", true));
  }
}
