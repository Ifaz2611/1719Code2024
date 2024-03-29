// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// WPILIB
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// ROBOT
import frc.robot.subsystems.DeviceSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSequence extends SequentialCommandGroup {
  DeviceSubsystem mDeviceSubsystem;
  
  // Turn shooter motors a given speed (forward or backward)
  public InstantCommand ShooterMotors (int speed) {
    return new InstantCommand(()->mDeviceSubsystem.turnShooterMotors(speed));
  }
  // Wait x seconds
  public WaitCommand waitwait (double time) {
    return new WaitCommand(time);
  }
  // Turn intake motors a given speed (forward or backward)
  public InstantCommand IntakeMotors (int speed) {
    return new InstantCommand(()->mDeviceSubsystem.turnIntakeMotors(-speed));
  }
  
  /** Creates a new ShootSequence. */
  public ShootSequence(DeviceSubsystem mDeviceSubsystem) {
    this.mDeviceSubsystem = mDeviceSubsystem;
    // Add your commands in the addCommands() call, e.g.
    /* 
       1 - Turns shooter motors on
       2 - Wait 1 second to speed up
       3 - Turn on intake motors
       4 - Wait for 0.5 seconds
       5 - Turn off the intake motors
       6 - Turn off the shooter motors 
    */
    addCommands(ShooterMotors(1), waitwait(1), IntakeMotors(1), waitwait(.5), IntakeMotors(0), ShooterMotors(0));
  }
}
