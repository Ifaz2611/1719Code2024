// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.SwerveSubsystem.SwerveDriveCoordinator; // unused import

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDCommandTurnToAngle extends PIDCommand {
  /** Creates a new PIDCommandTurnToAngle. */
  public PIDCommandTurnToAngle(LimelightSubsystem mLimelightSubsystem, SwerveSubsystem mSwerveSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(Constants.PTurnToAngle, Constants.ITurnToAngle, Constants.DTurnToAngle),
        // This should return the measurement
         () -> mLimelightSubsystem.getAngleToTag(),
        // This should return the setpoint (can also be a constant)
        ()-> 0,
        // This uses the output
        output -> {
          // Use the output here
          mSwerveSubsystem.SWERVE_DRIVE_COORDINATOR.setSwerveDrive(0, 0, output);
          
        });

        getController().setTolerance(Constants.LimeLightDegreesTolerance, Constants.LimeLightVelocityTolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.m_controller.atSetpoint();
  }
}
