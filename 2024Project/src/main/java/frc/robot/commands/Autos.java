// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DeviceSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  SwerveSubsystem driveSubsystem;
  // DEFAULT AUTO
  public static Command defaultAuto(SwerveSubsystem driveSubsystem, DeviceSubsystem deviceSubsystem ,LimelightSubsystem mLimelightSubsystem) {
    // return new InstantCommand(() -> {subsystem.SWERVE_DRIVE_COORDINATOR.setSwerveDrive(90,0,0 );});
    System.out.println("Default Auto");


    return Commands.sequence(new PIDCommandTurnToAngle(mLimelightSubsystem, driveSubsystem), new ShootSequence(deviceSubsystem));
  }

  // AUTO 1
  public static Command Auto1(SwerveSubsystem driveSubsystem, DeviceSubsystem deviceSubsystem, LimelightSubsystem mLimelightSubsystem) {
    // return new InstantCommand(() -> {subsystem.SWERVE_DRIVE_COORDINATOR.setSwerveDrive(90,0,0 );});
    System.out.println("Auto 1");
    return Commands.sequence(new PIDCommandTurnToAngle(mLimelightSubsystem, driveSubsystem), new ShootSequence(deviceSubsystem));
  }

  // AUTO 2
  public static Command Auto2(SwerveSubsystem driveSubsystem, DeviceSubsystem deviceSubsystem ) {
    // return new InstantCommand(() -> {subsystem.SWERVE_DRIVE_COORDINATOR.setSwerveDrive(90,0,0 );});
    System.out.println("Auto 2");
    return Commands.sequence( Autos.Intake(deviceSubsystem) );
  }

  // AUTO 3
  public static Command Auto3(SwerveSubsystem driveSubsystem, DeviceSubsystem deviceSubsystem ) {
    // return new InstantCommand(() -> {subsystem.SWERVE_DRIVE_COORDINATOR.setSwerveDrive(90,0,0 );});
    System.out.println("Auto 3");
    return Commands.sequence( Autos.Intake(deviceSubsystem) );
  }
  //instantCommands

  public static Command Intake(DeviceSubsystem device){

    return new InstantCommand(()-> {device.turnIntakeMotors(0);});
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
// 