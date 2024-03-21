/* 
 * Move and intake race command. This command will move until intook or moving finished. This is done to avoid crashing into a wall and moving forever!
 * 
 * UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED UNTESTED 
 * 
 * written by harrison (i have no idea if this will work LOL)
*/

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.DeviceSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveyIntakeyRaceCommand extends ParallelRaceGroup {
  /** Creates a new MoveyIntakeyRaceCommand. */
  public MoveyIntakeyRaceCommand(AutoMovePIDCommand mAutoMovePIDCommand, DeviceSubsystem m_DeviceSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(mAutoMovePIDCommand, 
    new InstantCommand(() -> {

      while (!m_DeviceSubsystem.checkRing()) {
        m_DeviceSubsystem.turnIntakeMotors(1);
      } 

      m_DeviceSubsystem.turnOffIntakeMotors();
    }));
  }
}
