/*
* THIS IS UNFINISHED AND UNTESTED
* This code has been written prior to the device being finished, all code isn't final!
*
* Essentially just the plan of the code :)
*/

package frc.robot.subsystems;

import edu.wpi.first.math.proto.System;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.databind.introspect.AnnotationCollector.OneAnnotation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
//import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Motors
public class DeviceSubsystem extends SubsystemBase {

    // control the arm 
    private static CANSparkMax ARM_MOTOR;

    // these 2 controll the the motors of the shooter more specifically intake
    private static CANSparkMax SHOOTER;
    private static CANSparkMax INTAKE;

    public DeviceSubsystem() {

    //    ARM_MOTOR_LEFT = new CANSparkMax(Constants.ARM_MOTOR_LEFT, MotorType.kBrushless);
    //    ARM_MOTOR = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless); // TEMP ID. VERIFY BEFORE RUNNING PLEASEE

        SHOOTER = new CANSparkMax(Constants.SHOOTER, MotorType.kBrushless);
        INTAKE = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);

    }

    // turns the motors a specific direction.
    // -1 is intake, 1 is shoot
    public void turnShooterMotors(int ShootBOF) {
        if (ShootBOF == 1) {
            // make motors turn inwards
            SHOOTER.set(Constants.SHOOTSPEED);

        } else if (ShootBOF == 0) {
            // make motors shoot ring out + speed
            // SHOOTER.set(0);//-Constants.SHOOTSPEED);
            SHOOTER.set(0);//Constants.SHOOTSPEED);

        } else {
            turnOffIntakeMotors();
        }
    }

    // turns the shoot motor on
    public void turnIntakeMotors(int onOrOff) {
        if (onOrOff == 1) {
            INTAKE.set(Constants.INTAKESPEED);
        } else {
            INTAKE.set(0);
        }
    }
    // turns off the shooter
    public void turnOffShooter(){
        INTAKE.set(0);
    }

    // turns off the intake motors
    public void turnOffIntakeMotors() {
        // SHOOTER.set(0);
        INTAKE.set(0);
    }
}
