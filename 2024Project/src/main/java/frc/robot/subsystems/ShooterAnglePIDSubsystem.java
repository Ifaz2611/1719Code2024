// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix6.hardware.CANcoder;
// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
//import com.revrobotics.;
import frc.robot.Constants;


public class ShooterAnglePIDSubsystem extends PIDSubsystem {
  /** Creates a new ShooterAnglePIDSubsystem. */

  // public DigitalInput armLowerLimit = new DigitalInput(Constants.BOTTOM_LIMIT_SWITCH_PIN);
  public DutyCycleEncoder ShootAngleEncoder;
  public CANSparkMax ShootAngleMotor;
  public boolean isIntaking = false;
  public boolean manualControl = false;
  
  //change to actual encoder
  public ShooterAnglePIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.ShootAngleP, Constants.ShootAngleI, Constants.ShootAngleD));
         {
          
        };
           this.ShootAngleEncoder = new DutyCycleEncoder(Constants.ShootAngleEncoder_PIN);
           this.ShootAngleMotor = new CANSparkMax(Constants.ShootAngleMotorPin,MotorType.kBrushless);
          //  System.out.println("initshootangle");  
  }
  public void setIntakeState(boolean state) {
    this.isIntaking = state;
  }

  public boolean getIntakeState() {
    return this.isIntaking;
  }

    // sets manual control
    public void setManualControl(boolean state) {
      manualControl = state;
    }

  public double shootAngle() {
      return getMeasurement();   
          }

  @Override
  public void useOutput(double output, double setpoint) {

    // if (armLowerLimit.get() && output > 0){
    //   output = 0;
    // } 

ShootAngleMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    double AngleDegrees = ShootAngleEncoder.getAbsolutePosition()*360.0 - Constants.SHOOTER_ANGLE_ZEROPOINT_OFFSET - Constants.SHOOTER_ANGLE_CORRECTION;
    // // Return the process variable measurement here
    //System.out.println(AngleDegrees);
    return AngleDegrees;
    
  }
}


