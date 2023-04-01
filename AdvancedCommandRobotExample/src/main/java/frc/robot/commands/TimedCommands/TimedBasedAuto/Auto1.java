package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.PathPlannerLoadPathCommand;
import frc.robot.commands.PIDCommand.TiltPIDCommand;
import frc.robot.commands.SimpleCommands.Extend;
import frc.robot.commands.SimpleCommands.Intake;
import frc.robot.commands.SimpleCommands.Retract;

public class Auto1 extends SequentialCommandGroup {
  public Auto1() {
    addCommands(
        new TiltPIDCommand(Constants.TILT_HIGH_CUBE_SETPOINT).withTimeout(0.5),
        new Extend().withTimeout(0.5),
        new Intake(Constants.MANIPULATOR_SPEED_BLEH).withTimeout(0.5),
        new Retract().withTimeout(0.5),
        new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT).withTimeout(0.5),
        new ParallelCommandGroup(
            new PathPlannerLoadPathCommand("pathplanner/generatedJSON/Path1.wpilib.json", true).withTimeout(15),
            new TiltPIDCommand(Constants.TILT_LOW_SETPOINT),
            new Intake(Constants.MANIPULATOR_SPEED_INTAKE)),
        new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT),
        new PathPlannerLoadPathCommand("pathplanner/generatedJSON/Patht2.wpilib.json", true).withTimeout(15),
        new TiltPIDCommand(18.0),
        new Intake(Constants.MANIPULATOR_SPEED_PCHOO));
  }
}
