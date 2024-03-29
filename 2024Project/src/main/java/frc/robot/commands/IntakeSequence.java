// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// WPILIB
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// ROBOT
import frc.robot.Constants;
import frc.robot.subsystems.DeviceSubsystem;
import frc.robot.subsystems.ShooterAnglePIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class IntakeSequence extends SequentialCommandGroup {  
  DeviceSubsystem mDeviceSubsystem;
  private double ANGLEAIM;

  // Waits for x seconds
  public WaitCommand waitwait(double time) {
    return new WaitCommand(time);
  }

  // Spins intake motors with given direction
  public InstantCommand IntakeMotors(String direction) {
    // "inShooter" --> -1
    // "outShooter" --> 1
    // otherwise stop --> 0
    return new InstantCommand(()->mDeviceSubsystem.turnIntakeMotors((direction == "inShooter") ? -1 : (direction == "outShooter") ? 1 : (direction == "off") ? 0 : 0));
  }

  // Sets the state of intake to true or false (true is intaking)
  public InstantCommand setIntakeState(ShooterAnglePIDSubsystem m_angler, boolean state) {
    return new InstantCommand(()-> m_angler.setIntakeState(state));
  }

  // Sets the setpoint for the arm angle
  public InstantCommand setIntakeSetpoint(ShooterAnglePIDSubsystem m_angler) {
    return new InstantCommand(()->m_angler.setSetpoint(ANGLEAIM));
  }

  /* Creates a new IntakeSequence. */
  public IntakeSequence(DeviceSubsystem mDeviceSubsystem, ShooterAnglePIDSubsystem m_angler, String action, double ANGLEAIM) {
    this.mDeviceSubsystem = mDeviceSubsystem;
    this.ANGLEAIM = ANGLEAIM;
    
    // Clamp ANGLEAIM between min and max values
    if (ANGLEAIM > Constants.MAX_SHOOTER_ANGLE) {
      ANGLEAIM = Constants.MAX_SHOOTER_ANGLE;
    }
    if (ANGLEAIM < Constants.MIN_SHOOTER_ANGLE) {
      ANGLEAIM = Constants.MIN_SHOOTER_ANGLE;
    }

    // Turn On Intake
    if (action == "intakeOn") {
       addCommands(setIntakeState(m_angler, true), setIntakeSetpoint(m_angler), IntakeMotors("inShooter")); 
    } 
    // Turn Off Intake
    else if (action == "intakeOff") {
       addCommands(IntakeMotors("off"), setIntakeState(m_angler, false));
    }
    // Move Shooter to an Angle
    else if (action == "setAngle") {
      addCommands(setIntakeState(m_angler, true), setIntakeSetpoint(m_angler));
    }
    // Turn On Outtake and Moves Arm to Floor
    else if (action == "outtakeFloor") {
       addCommands(setIntakeState(m_angler, true), setIntakeSetpoint(m_angler), IntakeMotors("outShooter"), waitwait(.05), IntakeMotors("off"), setIntakeState(m_angler, false));
    } 
    // Amp Shooting
    else if (action == "ampShoot") {
       addCommands(setIntakeState(m_angler, true), setIntakeSetpoint(m_angler), IntakeMotors("outShooter"), waitwait(.5), IntakeMotors("off"), setIntakeState(m_angler, false));
    } 
    // Outtake and Don't Move Arm
    else if (action == "outtake") {
       addCommands(setIntakeSetpoint(m_angler), IntakeMotors("outShooter"), waitwait(.05), IntakeMotors("off"), setIntakeState(m_angler, false));
    } 
  }

// Called once the command ends or is interrupted.
  // @Override 
  // public void end(boolean interrupted ) {
  //   mDeviceSubsystem.turnOffIntakeMotors();
  //  }
}
