// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// JAVA
import java.util.function.DoubleSupplier;

// REV
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

// Phoenix
import com.ctre.phoenix6.hardware.CANcoder;

// WPILIB
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

// ROBOT
import frc.robot.Constants;

public class SwerveDirectionPIDSubsystem extends PIDSubsystem {
  SwerveDriveWheel m_DriveWheel;
  CANcoder directionSensor;
  public int directionMotorpin;
  public CANSparkMax speedMotor;
  public RelativeEncoder directionEncoder;
  public DoubleSupplier directionSetpoint;
  public CANSparkMax directionMotor;
  
  /* Creates a new SwerveDirectionPIDSubsystem. */
  public SwerveDirectionPIDSubsystem(int directionSensorPin, int directionPin) {
    super(
      // The PIDController used by the subsystem
      new PIDController(Constants.DirectionP, Constants.DirectionI, Constants.DirectionD)
    );
    this.directionMotor = new CANSparkMax(directionPin, MotorType.kBrushless);
    this.directionSensor = new CANcoder(directionSensorPin);
    getController().enableContinuousInput(0, 360);
  }

  // Set direction motor to the output
  @Override
  public void useOutput(double output, double setpoint) {
    directionMotor.set(output);
  }

  // Return the direction sensors measurement
  @Override
  public double getMeasurement() {
    return directionSensor.getAbsolutePosition().getValueAsDouble() * 360;    
  }

  // Set direction given a setpoint
  public void setDirection(double setpoint) {
    setSetpoint(setpoint);
  } 
}
