// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.subsystems.SwerveSubsystem.SwerveDriveWheel;
import frc.robot.subsystems.SwerveSubsystem.degreeSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveDirectionPIDCommand extends PIDCommand {
  /** Creates a new SwervePIDCommand. 

  * @param directionSensor  
  @param m_DriveWheel  
 */
  degreeSupplier directionSensor;
  SwerveDriveWheel m_DriveWheel;
  public SwerveDirectionPIDCommand(DoubleSupplier directionSensor, SwerveDriveWheel m_DriveWheel) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DirectionP, Constants.DirectionI, Constants.DirectionD),
        // This should return the measurement
        directionSensor,
        // This should return the setpoint (can also be a constant)
        ()-> m_DriveWheel.getSetpoint(),
        // This uses the output
        output -> {
       m_DriveWheel.directionMotors(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

 
}
