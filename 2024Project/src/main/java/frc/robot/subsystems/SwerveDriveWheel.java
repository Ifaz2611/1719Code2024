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

    public DoubleSupplier directionSetpoint;
    public double setpoint;
    public CANSparkMax directionMotor;
    SwerveDirectionPIDSubsystem directionController;
    

    public SwerveDriveWheel(CANSparkMax speedMotor, SwerveDirectionPIDSubsystem directionController)
    {

      //  this.directionMotorpin = directionMotorpin;

       // this.directionMotor = new CANSparkMax(directionMotorpin, MotorType.kBrushless);
        this.speedMotor = speedMotor;
this.directionController = directionController;
directionController.enable();
        
   
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
public void setDirection(double angle){
directionController.setDirection(angle);

}
// public void directionMotors(double output) {
//     directionMotor.set(output);
// }

 
    
}
