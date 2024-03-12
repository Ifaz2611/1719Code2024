// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAnglePIDSubsystem;

public class ArmDirectControl extends Command {

  ShooterAnglePIDSubsystem mAnglePIDSubsystem;
  DoubleSupplier getX;
  double getY;

  /** Creates a new ArmDirectControl. */
  public ArmDirectControl(ShooterAnglePIDSubsystem mAnglePIDSubsystem, DoubleSupplier getX, DoubleSupplier getY) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.getX = getX; // idk if this is useful
    this.getY = getY.getAsDouble(); // gets a double of Y as it would have to do it anyways later on
    this.mAnglePIDSubsystem = mAnglePIDSubsystem; 
    addRequirements(mAnglePIDSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // sets Y as the set point. maybe this will work i hope 
    this.mAnglePIDSubsystem.setSetpoint(getY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        this.mAnglePIDSubsystem.setSetpoint(47); // make this a constant pls

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
