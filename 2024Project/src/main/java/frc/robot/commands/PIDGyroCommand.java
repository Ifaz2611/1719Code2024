// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// WPILIB
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

// ROBOT
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDGyroCommand extends PIDCommand {
  /** Creates a new PIDGyroCommand. */
  public PIDGyroCommand(double direction, SwerveSubsystem mSwerveSubsystem) {
    super(
      // The controller that the command will use
      new PIDController(Constants.PTurnToAngle, Constants.ITurnToAngle, Constants.DTurnToAngle),
      // This should return the measurement (current angle from the gyro)
      () -> mSwerveSubsystem.getGYROAngle(),
      // This should return the setpoint (wants to end at given angle)
      direction,
      // This uses the output
      output -> {
        // Sends the command to turn to desired angle
        mSwerveSubsystem.SWERVE_DRIVE_COORDINATOR.drifTranslate(0, 0, output);
      }
    );
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(Constants.LimeLightDegreesTolerance, Constants.LimeLightVelocityTolerance);
    getController().enableContinuousInput(0, 360);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mSwerveSubsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.m_controller.atSetpoint();
  }
}
