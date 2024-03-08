// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterAnglePIDSubsystem;

public class ShootAngleControlCommand extends Command {
  private ShooterAnglePIDSubsystem mAnglePIDSubsystem;
  // private DoubleSupplier getShootAngle;
  private LimelightSubsystem limeLight;
// hahfdshsfkh
  /** Creates a new ShootAngleControlCommand. */
  public ShootAngleControlCommand(ShooterAnglePIDSubsystem mAnglePIDSubsystem, LimelightSubsystem limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mAnglePIDSubsystem);

    this.mAnglePIDSubsystem = mAnglePIDSubsystem;
    this.limeLight = limeLight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   mAnglePIDSubsystem.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // returns angle as double
    mAnglePIDSubsystem.setSetpoint(
     0// limeLight.getShootingAngle()
      );
  }

  // Called once the command ends or is interrupted.
  @Override 
  public void end(boolean interrupted ) {

   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
