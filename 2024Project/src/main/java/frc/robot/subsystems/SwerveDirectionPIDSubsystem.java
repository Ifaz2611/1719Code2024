// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
//import frc.robot.subsystems.SwerveDriveWheel;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;


import com.ctre.phoenix6.hardware.CANcoder;
//import frc.robot.subsystems.SwerveSubsystem;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SwerveDirectionPIDSubsystem extends PIDSubsystem {
  /** Creates a new SwerveDirectionPIDSubsystem. */
SwerveDriveWheel m_DriveWheel;
CANcoder directionSensor;
public int directionMotorpin;
public double directionInvert = 1;
public CANSparkMax speedMotor;
public RelativeEncoder directionEncoder;


public DoubleSupplier directionSetpoint;

public CANSparkMax directionMotor;

  public SwerveDirectionPIDSubsystem(int directionSensorPin, int directionPin) {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.DirectionP, Constants.DirectionI, Constants.DirectionD));
this.directionMotor = new CANSparkMax(directionPin, MotorType.kBrushless);
    System.out.println("PID actinit");
    getController().enableContinuousInput(0, 360);
this.directionSensor = new CANcoder(directionSensorPin);


  }

  @Override
  public void useOutput(double output, double setpoint) {


    directionMotor.set(output*directionInvert);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here


    return directionSensor.getAbsolutePosition().getValueAsDouble() * 360;
    
  }
  public void setDirection(double setpoint)
    {

       // double currentAngle = directionSensor.getAbsolutePosition().getValueAsDouble()*360;
        // find closest angle to setpoint
        // double setpointAngle = SwerveSubsystem.closestAngle(currentAngle, setpoint);
        // // find closest angle to setpoint + 180
        // double setpointAngleFlipped = SwerveSubsystem.closestAngle(currentAngle, setpoint + 180.0);
        // // if the closest angle to setpoint is shorter
        // if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        // {
        //     // unflip the motor direction use the setpoint
        //     this.directionInvert = 1; //TODO: get invert to work
        //     setSetpoint(currentAngle + setpointAngle);
        // }
        // // if the closest angle to setpoint + 180 is shorter
        // else
        // {
        //     // flip the motor direction and use the setpoint + 180
        //     this.directionInvert = -1;
        //     setSetpoint(currentAngle + setpointAngleFlipped);
        // }
   setSetpoint(setpoint);

    }
 
}
