// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// JAVA
import java.util.function.DoubleSupplier;

// WPILIB
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

// ROBOT
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDCompositionDriveCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private double direction;
  private double gyroAngle;
  private double distance;
  // Movement PID Controller
  PIDController MoveP = new PIDController(
    Constants.AUTO_MOVE_P, 
    Constants.AUTO_MOVE_I, 
    Constants.AUTO_MOVE_D
  );      
  // Turning PID Controller
  PIDController TurnP = new PIDController(
    Constants.PTurnToAngle, 
    Constants.ITurnToAngle, 
    Constants.DTurnToAngle
  );

  /** Creates a new PIDCompositionDriveSubsystem. */
  public PIDCompositionDriveCommand(SwerveSubsystem swerveSubsystem, double direction, double distance, double gyroAngle) {
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    this.direction = direction;
    // Convert distance using LINEAR_MOVE_RATIO
    this.distance = (distance/Constants.LINEAR_MOVE_RATIO);
    this.gyroAngle = gyroAngle;
    swerveSubsystem.resetDistanceMotors();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set Tolerances
    MoveP.setTolerance(Constants.LimeLightPositionTolerance, Constants.LimeLightVelocityTolerance);
    TurnP.setTolerance(Constants.LimeLightDegreesTolerance, Constants.LimeLightVelocityTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //swerveSubsystem.resetDistanceMotors(); //Add this instead of adding this before each move command
    // Calculate power and angle
    double translatePower = MoveP.calculate(swerveSubsystem.returnAverageDistance(), this.distance);
    double phi = TurnP.calculate(swerveSubsystem.getGYROAngle(), this.gyroAngle);
    // Executes driving and turning method given parameters
    this.swerveSubsystem.SWERVE_DRIVE_COORDINATOR.drifTranslate(
      this.direction, 
      translatePower * Constants.TELEOPSPEEDMODIFIER,
      phi * Constants.TELEOPTWISTMODIFIER
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MoveP.atSetpoint() && TurnP.atSetpoint();
  }
}
