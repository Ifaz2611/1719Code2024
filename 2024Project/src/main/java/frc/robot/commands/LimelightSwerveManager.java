// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDirectionPIDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightSwerveManager extends Command {

  private LimelightSubsystem m_limelightSubsystem;
  private SwerveSubsystem m_swerveSubsystem;
  public double moveOutput;
  public double angleOutput;
  public double angle;

  LimeLightMovePIDCommand m_limelightMoveCommand;
  PIDCommandTurnToAngle m_swerveDirectionCommand;

  /** Creates a new LimelightSwerveManager. */
  public LimelightSwerveManager(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(limelightSubsystem, swerveSubsystem);
    this.m_limelightSubsystem = limelightSubsystem;
    this.m_swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_limelightMoveCommand = new LimeLightMovePIDCommand(m_limelightSubsystem, m_swerveSubsystem, this);
    this.m_swerveDirectionCommand = new PIDCommandTurnToAngle(m_limelightSubsystem, m_swerveSubsystem, this);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.SWERVE_DRIVE_COORDINATOR.drifTranslate(angle, moveOutput, -angleOutput);
    System.out.println("LimelightSwerveExecute");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limelightMoveCommand.getController().atSetpoint() && m_swerveDirectionCommand.getController().atSetpoint();
  }

  public void setMovePID(double output) {
    this.moveOutput = output;
  }

  public void setAnglePID(double output, double angle) {
    this.angleOutput = output;
    this.angle = angle;
  }
}
