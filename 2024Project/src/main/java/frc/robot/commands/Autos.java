// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DeviceSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  SwerveSubsystem driveSubsystem;
  public static Command exampleAuto(SwerveSubsystem driveSubsystem, DeviceSubsystem deviceSubsystem ) {

   // return new InstantCommand(() -> {subsystem.SWERVE_DRIVE_COORDINATOR.setSwerveDrive(90,0,0 );});
    return Commands.sequence( Intake(deviceSubsystem) );
  }
  //instantCommands

  public static Command Intake(DeviceSubsystem device){

    return new InstantCommand(()-> {device.turnIntakeMotors(0);});
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
