// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * SetIntakeOutake: This command sets both the intake and the shooter at the same time. 
 * Also added a java doc at the top, because I want us to standardize commenting more and this will help :)
 * 
 * Written by Harrison
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeviceSubsystem;

public class SetIntakeOutake extends Command {
  DeviceSubsystem mDeviceSubsystem;
  double intakeSpeed;
  double shooterSpeed;
  /** Creates a new SetIntakeOutake. */
  public SetIntakeOutake(DeviceSubsystem mDeviceSubsystem, double intakeSpeed, double shooterSpeed) {
    
    this.mDeviceSubsystem = mDeviceSubsystem;

    // These should make it so we never input anything past -1 to 1 for these
    // It probably would be better to put it into device subsystem but i wanna make sure people are cool with this
    this.intakeSpeed = (intakeSpeed > 1) ? 1.0 : (intakeSpeed < -1) ? -1.0 : intakeSpeed;
    this.shooterSpeed = (shooterSpeed > 1) ? 1.0 : (shooterSpeed < -1 ? -1.0 : shooterSpeed);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDeviceSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // turns the motors in the directions specified thats it.
    mDeviceSubsystem.turnIntakeMotors(intakeSpeed);
    mDeviceSubsystem.turnShooterMotors(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
