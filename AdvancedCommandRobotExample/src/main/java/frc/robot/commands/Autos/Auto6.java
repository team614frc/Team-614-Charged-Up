package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PathPlannerCommands.PathPlannerLoadPathCommand;
import frc.robot.Commands.SequentialParallelCommands.ScoreMidCubeAuto;

public class Auto6 extends SequentialCommandGroup {
  public Auto6() {
    addCommands(
        new ScoreMidCubeAuto(),
        new PathPlannerLoadPathCommand("pathplanner/generatedJSON/RTCMM.wpilib.json", true));
  }
}
