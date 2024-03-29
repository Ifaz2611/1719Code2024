/*
 * Shooter Angle PID Subsytem: this command allows us to change the position of the arm using a setpoint 
 * 
 * It also holds the manual control boolean, which allows us to manually control it in other parts
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// REV
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

// WPILIB
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

// ROBOT
import frc.robot.Constants;

public class ShooterAnglePIDSubsystem extends PIDSubsystem {
  public DutyCycleEncoder ShootAngleEncoder;
  public CANSparkMax ShootAngleMotor;  
  // Intake state (true is currently intaking)
  public boolean isIntaking = false;  
  // Manual control (true is using manual control)
  public boolean manualControl = false;
  
  /* Creates a new ShooterAnglePIDSubsystem. */
  public ShooterAnglePIDSubsystem() {
    super(
      // The PIDController used by the subsystem
      new PIDController(Constants.ShootAngleP, Constants.ShootAngleI, Constants.ShootAngleD)
    );
    // Initialize the through bore encoder
    this.ShootAngleEncoder = new DutyCycleEncoder(Constants.ShootAngleEncoder_PIN);
    // Initialize the motor to control the arm
    this.ShootAngleMotor = new CANSparkMax(Constants.ShootAngleMotorPin,MotorType.kBrushless);
  }

  // Sets intake state (true is currently intaking)
  public void setIntakeState(boolean state) {
    this.isIntaking = state;
  }

  // Returns the intake state
  public boolean getIntakeState() {
    return this.isIntaking;
  }

  // Sets manual control (true is using manual control)
  public void setManualControl(boolean state) {
    manualControl = state;
  }

  // Returns the shoot angle
  public double shootAngle() {
    return getMeasurement();   
  }

  // Sets the motor based on the output
  @Override
  public void useOutput(double output, double setpoint) {
    ShootAngleMotor.set(output);
  }

  // Returns the arm position in degrees
  @Override
  public double getMeasurement() {
    double AngleDegrees = ShootAngleEncoder.getAbsolutePosition()*360.0 - Constants.SHOOTER_ANGLE_ZEROPOINT_OFFSET - Constants.SHOOTER_ANGLE_CORRECTION;
    return AngleDegrees;
  }
}