// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.SwerveDriveWheel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;


import frc.robot.subsystems.SwerveSubsystem;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SwerveDirectionPIDSubsystem extends PIDSubsystem {
  /** Creates a new SwerveDirectionPIDSubsystem. */
SwerveDriveWheel m_DriveWheel;
DoubleSupplier directionSensor;
public int directionMotorpin;
public double directionInvert;
public CANSparkMax speedMotor;

public DoubleSupplier directionSetpoint;
public double setpoint;
public DoubleSupplier getSetpoint;
public CANSparkMax directionMotor;

  public SwerveDirectionPIDSubsystem(DoubleSupplier directionSensor, SwerveDriveWheel m_DriveWheel) {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.DirectionP, Constants.DirectionI, Constants.DirectionD));
directionMotor = new CANSparkMax(m_DriveWheel.directionMotorpin, MotorType.kBrushless);
this.directionSensor = directionSensor;

  }

  @Override
  public void useOutput(double output, double setpoint) {
    directionMotor.set(output*directionInvert);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return directionSensor.getAsDouble();
  }
}
