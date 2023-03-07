// package frc.robot.commands.Autonomous.TimedBasedAuto;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.commands.LoadPathplannerTrajectoryCommand;
// import frc.robot.commands.PIDCommand.TiltPIDCommand;
// import frc.robot.commands.SimpleCommands.Extend;
// import frc.robot.commands.SimpleCommands.Intake;
// import frc.robot.commands.SimpleCommands.Retract;

// public class ScoreChargeStationAuto extends SequentialCommandGroup {
//   public ScoreChargeStationAuto() {
//     addCommands(
//         new TiltPIDCommand(Constants.TILT_LOAD_STATION_SETPOINT).withTimeout(0.5),
//         new Extend().withTimeout(0.5),
//         new Intake(Constants.MANIPULATOR_SPEED_BLEH).withTimeout(0.5),
//         new Retract().withTimeout(0.5),
//         new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT).withTimeout(0.5),
//         new LoadPathplannerTrajectoryCommand("pathplanner/generatedJSON/inch.wpilib.json", true));
//   }
// }
