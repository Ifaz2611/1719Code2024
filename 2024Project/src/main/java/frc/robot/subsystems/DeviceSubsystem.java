/*
* THIS IS UNFINISHED AND UNTESTED
* This code has been written prior to the device being finished, all code isn't final!
*
* Essentially just the plan of the code :)
*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// import com.fasterxml.jackson.databind.introspect.AnnotationCollector.OneAnnotation;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;



// Motors
public class DeviceSubsystem extends SubsystemBase {

    // control the arm 


    // these 2 controll the the motors of the shooter and intake
    private static CANSparkMax SHOOTER;
    private static CANSparkMax INTAKE;


    private int proximity;
    private ColorSensorV3 m_colorSensor = Constants.m_colorSensor;


    public DeviceSubsystem() {

       SHOOTER = new CANSparkMax(Constants.SHOOTER, MotorType.kBrushless);
       INTAKE = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);


    }

    // turns the motors a specific direction.
    // -1 is intake, 1 is shoot

    //turns on the shooter motors
    public void turnShooterMotors(double speed) {
        SHOOTER.set(speed);
        
    }

    // turns the intake motor on
    public void turnIntakeMotors(double speed) { 
        INTAKE.set(speed*.75);
    }
    // turns off the shooter
    public void turnOffShooter(){
        SHOOTER.set(0);
    }

    // turns off the intake motors
    public void turnOffIntakeMotors() {
        // SHOOTER.set(0);
        INTAKE.set(0);
    }


    // REVERSE THIS HERE 
    public boolean checkRing(){ 
        proximity = m_colorSensor.getProximity();
        // System.out.println("proximity " + proximity);
        if (proximity >= Constants.DISTANCE_NOTE_IN) {
            return true;
        } 
        else {
            return false;
        }
    }
}

