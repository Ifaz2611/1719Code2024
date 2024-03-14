// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DeviceSubsystem;
import frc.robot.subsystems.ShooterAnglePIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html



public class IntakeSequence extends SequentialCommandGroup {
  /** Creates a new ShootSequence. */
  DeviceSubsystem mDeviceSubsystem;

  public double ANGLEAIM = Constants.DEFAULT_SHOOTER_ANGLE;

  public WaitCommand waitwait (double time) {
    return new WaitCommand(time);
  }
  public InstantCommand IntakeMotors (int direction) {
    return new InstantCommand(()->mDeviceSubsystem.turnIntakeMotors(-direction));
  }
  public InstantCommand setIntakeState(ShooterAnglePIDSubsystem m_angler, boolean state) {
    return new InstantCommand(()-> m_angler.setIntakeState(state));
  }
  public InstantCommand setIntakeSetpoint(ShooterAnglePIDSubsystem m_angler) {
    return new InstantCommand(()->m_angler.setSetpoint(ANGLEAIM));
  }

  public IntakeSequence(DeviceSubsystem mDeviceSubsystem, ShooterAnglePIDSubsystem m_angler, int stateNum) {
    this.mDeviceSubsystem = mDeviceSubsystem;
    // Add your commands in the addCommands() call, e.g.
     //addCommands(new FooCommand(), new BarCommand());
     if (stateNum == 0) {
       addCommands(setIntakeState(m_angler, true), setIntakeSetpoint(m_angler), IntakeMotors(1));
    //} else if (stateNum == 2) {
      //addCommands(setIntakeState(m_angler, true), setIntakeSetpoint(m_angler), IntakeMotors(1), waitwait(2), IntakeMotors(0), setIntakeState(m_angler, false));
    } 
    else if (stateNum == 1) {
       addCommands(IntakeMotors(0), setIntakeState(m_angler, false));
    }
    else if (stateNum == 2) {
      addCommands(setIntakeState(m_angler, true), setIntakeSetpoint(m_angler));
    }
    if (stateNum == -1) {
       addCommands(setIntakeState(m_angler, true), setIntakeSetpoint(m_angler), IntakeMotors(-1));
    } 
  }
}
