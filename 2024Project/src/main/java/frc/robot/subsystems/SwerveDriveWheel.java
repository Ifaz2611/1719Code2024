// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// JAVA
import java.util.function.DoubleSupplier;

// REV
import com.revrobotics.CANSparkMax;

// WPILIB
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveWheel extends SubsystemBase {
    public int directionMotorpin;
    public boolean directionInvert;
    public CANSparkMax speedMotor;
    public SwerveModuleState m_SwerveModuleState;
    public DoubleSupplier directionSetpoint;
    public double setpoint;
    public CANSparkMax directionMotor;
    SwerveDirectionPIDSubsystem directionController;
    
    public SwerveDriveWheel(CANSparkMax speedMotor, SwerveDirectionPIDSubsystem directionController) {
        this.speedMotor = speedMotor;
        this.directionController = directionController;
        directionController.enable();   
    }
    // Return the setpoint
    public double getSetpointWheel() {
        return this.setpoint;
    }

    // Set the speed of the motors
    public void speedMotors(double output) {
        speedMotor.set(output);
    }

    // Set the direction of the motors
    public void setDirection(double angle){
        directionController.setDirection(angle);
    }

    // Set the swerve state given power and angle
    public void SwerveStateSet(double power, double angle){
        speedMotor.set(power);
        directionController.setDirection(angle);
        this.m_SwerveModuleState = new SwerveModuleState(power,Rotation2d.fromDegrees(angle));
    }

    // Set the swerve state given a module state
    public void SwerveSetWithState(SwerveModuleState moduleState){
        speedMotor.set(moduleState.speedMetersPerSecond);
        directionController.setDirection(moduleState.angle.getDegrees());
    }
}
