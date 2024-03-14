// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DeviceSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSequence extends SequentialCommandGroup {
  /** Creates a new ShootSequence. */
  DeviceSubsystem mDeviceSubsystem;

  public InstantCommand ShooterMotors (int speed) {
    return new InstantCommand(()->mDeviceSubsystem.turnShooterMotors(speed));
  }
  public WaitCommand waitwait (double time) {
    return new WaitCommand(time);
  }
  public InstantCommand IntakeMotors (int speed) {
    return new InstantCommand(()->mDeviceSubsystem.turnIntakeMotors(-speed));
  }

  public ShootSequence(DeviceSubsystem mDeviceSubsystem) {
    this.mDeviceSubsystem = mDeviceSubsystem;
    // Add your commands in the addCommands() call, e.g.
     //addCommands(new FooCommand(), new BarCommand());
    addCommands(ShooterMotors(1),waitwait(2), IntakeMotors(1), waitwait(1.5), IntakeMotors(0), ShooterMotors(0));
  }
}
