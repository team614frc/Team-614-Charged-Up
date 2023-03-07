// package frc.robot.commands.PIDCommand;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;

// public class ManipulatorPIDCommand extends PIDCommand {

// public ManipulatorPIDCommand(double manipulatorSetpoint) {
// super(
// // The controller that the command will use
// new PIDController(Constants.P_kP, Constants.P_kI, Constants.P_kD),
// // Returns current intake speed
// RobotContainer.manipulator::getSpeed,
// // Could be used to hard code setpoint, but code requires two button presses
// // that dictate setpoint
// manipulatorSetpoint,
// // This uses the output
// RobotContainer.manipulator::set);
// // Use addRequirements() here to declare subsystem dependencies.
// addRequirements(RobotContainer.manipulator);
// }

// public void end(boolean interrupted) {
// RobotContainer.manipulator.set(Constants.MOTOR_REST_BACK);
// }

// @Override
// public boolean isFinished() {
// return false;
// }
// }
