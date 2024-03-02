// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
//import com.revrobotics.;
import frc.robot.Constants;


public class ShooterAnglePIDSubsystem extends PIDSubsystem {
  /** Creates a new ShooterAnglePIDSubsystem. */

  public DutyCycleEncoder ShootAngleEncoder;
  public CANSparkMax ShootAngleMotor;
  //change to actual encoder
  public ShooterAnglePIDSubsystem() {
     
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.ShootAngleP, Constants.ShootAngleI, Constants.ShootAngleD));
         {
          
        };
        
           this.ShootAngleEncoder = new DutyCycleEncoder(Constants.ShootAngleEncoder_PIN);
           this.ShootAngleMotor = new CANSparkMax(Constants.ShootAngleMotorPin,MotorType.kBrushless);
           System.out.println("initshootangle");  
  }

  public double shootAngle() {
      return getMeasurement();   
          }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here

    ShootAngleMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    double AngleDegrees = (ShootAngleEncoder.getAbsolutePosition()-Constants.UPOFFSET)*360;
    // // Return the process variable measurement here
    System.out.println(AngleDegrees-Constants.UPOFFSET);
    return AngleDegrees;
  }
}


