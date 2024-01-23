// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.SwerveSubsystem;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveWheel extends SubsystemBase
{
 
    public int directionMotorpin;
    public boolean directionInvert;
    public CANSparkMax speedMotor;
    public DoubleSupplier directionSensor;
    public DoubleSupplier directionSetpoint;
    public double setpoint;
    public DoubleSupplier getSetpoint;
    public CANSparkMax directionMotor;
    

    public SwerveDriveWheel(DoubleSupplier directionSensor, int directionMotorpin, CANSparkMax speedMotor)
    {
        this.directionSensor = directionSensor;
        this.directionMotorpin = directionMotorpin;
        this.directionMotor = new CANSparkMax(directionMotorpin, MotorType.kBrushless);
        this.speedMotor = speedMotor;
        this.getSetpoint = ()-> getSetpointWheel();
   
    }

public double getSetpointWheel() {

   return this.setpoint;
}


// public void directionMotors(double output) {
//     directionMotor.set(output); 
// }
public void speedMotors(double output) {
    speedMotor.set(output);
}
public void directionMotors(double output) {
    directionMotor.set(output);
}

 public void setDirection(double setpoint)
    {

        double currentAngle = directionSensor.getAsDouble();
        // find closest angle to setpoint
        double setpointAngle = SwerveSubsystem.closestAngle(currentAngle, setpoint);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = SwerveSubsystem.closestAngle(currentAngle, setpoint + 180.0);
        // if the closest angle to setpoint is shorter
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        {
            // unflip the motor direction use the setpoint
           // this.directionInvert = (false); TODO: get invert to work
            this.setpoint = (currentAngle + setpointAngle);
        }
        // if the closest angle to setpoint + 180 is shorter
        else
        {
            // flip the motor direction and use the setpoint + 180
          //  this.directionInvert = (true);
            this.setpoint = currentAngle + setpointAngleFlipped;
        }


    }
    
}
