package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommands.PathPlannerLoadPathCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCubeAuto;

public class Auto3 extends SequentialCommandGroup {
  public Auto3() {
    addCommands(
        new ScoreMidCubeAuto(),
        new PathPlannerLoadPathCommand("pathplanner/generatedJSON/BTCMM.wpilib.json", true));
  }
}
