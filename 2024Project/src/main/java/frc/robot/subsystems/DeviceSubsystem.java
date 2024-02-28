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
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
//import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Motors
public class DeviceSubsystem extends SubsystemBase {

    // these 2 motors controll the arm / angle of the shooter
    private static CANSparkMax ARM_MOTOR_LEFT;
    private static CANSparkMax ARM_MOTOR_RIGHT;

    // these 2 controll the the motors of the shooter more specifically intake
    private static CANSparkMax INTAKE_LEFT;
    private static CANSparkMax INTAKE_RIGHT;

    public DeviceSubsystem() {

      //  ARM_MOTOR_LEFT = new CANSparkMax(Constants.ARM_MOTOR_LEFT, MotorType.kBrushless);
     //   ARM_MOTOR_RIGHT = new CANSparkMax(Constants.ARM_MOTOR_RIGHT, MotorType.kBrushless); // temporary
        INTAKE_LEFT = new CANSparkMax(Constants.INTAKE_LEFT, MotorType.kBrushless);
        INTAKE_RIGHT = new CANSparkMax(Constants.INTAKE_RIGHT, MotorType.kBrushless);

    }

    // turns the motors a specific direction.
    // -1 is intake, 1 is shoot
    public void turnIntakeMotors(int ShootBOF) {
        if (ShootBOF == -1) {
            // make motors turn inwards
            INTAKE_LEFT.set(Constants.INTAKESPEED);
            INTAKE_RIGHT.set(Constants.INTAKESPEED);

        } else if (ShootBOF == 1) {
            // make motors shoot ring out + speed
            INTAKE_LEFT.set(0);//-Constants.SHOOTSPEED);
            INTAKE_RIGHT.set(0);//Constants.SHOOTSPEED);

        } else {
            turnOffIntakeMotors();
        }
    }

    public void turnOffIntakeMotors() {
        INTAKE_LEFT.set(0);
        INTAKE_RIGHT.set(0);
    }
}
