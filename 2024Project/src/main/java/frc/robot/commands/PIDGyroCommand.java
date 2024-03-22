// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
        // This should return the measurement
        () -> mSwerveSubsystem.getGYROAngle(),
        // This should return the setpoint (can also be a constant)
        direction,
        // This uses the output
        output -> {
          // Use the output here
          mSwerveSubsystem.SWERVE_DRIVE_COORDINATOR.drifTranslate(0, 0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(Constants.LimeLightDegreesTolerance, Constants.LimeLightVelocityTolerance);
    addRequirements(mSwerveSubsystem);
    getController().enableContinuousInput(0, 360);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.m_controller.atSetpoint();
  }
}
