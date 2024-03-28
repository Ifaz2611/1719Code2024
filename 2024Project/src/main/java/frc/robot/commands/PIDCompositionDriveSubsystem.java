// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDCompositionDriveSubsystem extends Command {
  private SwerveSubsystem swerveSubsystem;
  private double direction;
  private double gyroAngle;
  private double distance;
      PIDController MoveP = new PIDController(Constants.AUTO_MOVE_P, Constants.AUTO_MOVE_I, Constants.AUTO_MOVE_D);
      
    PIDController TurnP = new PIDController(Constants.PTurnToAngle, Constants.ITurnToAngle, Constants.DTurnToAngle);

  /** Creates a new PIDCompositionDriveSubsystem. */
  public PIDCompositionDriveSubsystem(SwerveSubsystem swerveSubsystem, double direction, double distance,
      double gyroAngle) {
this.swerveSubsystem = swerveSubsystem;
swerveSubsystem.resetDistanceMotors();
        addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.

    this.direction=direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
MoveP.setTolerance(Constants.LimeLightPositionTolerance, Constants.LimeLightVelocityTolerance);

TurnP.setTolerance(Constants.LimeLightDegreesTolerance, Constants.LimeLightVelocityTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

double translatePower = MoveP.calculate(swerveSubsystem.returnAverageDistance(), distance );
double Phi = TurnP.calculate(swerveSubsystem.getGYROAngle(), gyroAngle);
    this.swerveSubsystem.SWERVE_DRIVE_COORDINATOR.drifTranslate(direction, translatePower*Constants.TELEOPSPEEDMODIFIER,
         Phi*Constants.TELEOPTWISTMODIFIER);
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
