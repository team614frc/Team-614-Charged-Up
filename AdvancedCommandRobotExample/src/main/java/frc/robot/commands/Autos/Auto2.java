package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommands.PathPlannerLoadPathCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCubeAuto;

public class Auto2 extends SequentialCommandGroup {
  public Auto2() {
    addCommands(
        new ScoreMidCubeAuto(),
        new PathPlannerLoadPathCommand("pathplanner/generatedJSON/BTCMCS.wpilib.json", true));
  }
}
