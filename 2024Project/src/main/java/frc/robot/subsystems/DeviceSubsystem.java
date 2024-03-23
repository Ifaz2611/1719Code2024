/*
* THIS IS UNFINISHED AND UNTESTED
* This code has been written prior to the device being finished, all code isn't final!
*
* Essentially just the plan of the code :)
*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.Pigeon2;
// import com.fasterxml.jackson.databind.introspect.AnnotationCollector.OneAnnotation;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkPIDController;
//import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Motors
public class DeviceSubsystem extends SubsystemBase {

    // control the arm 
    // private static CANSparkMax ARM_MOTOR;

    // these 2 controll the the motors of the shooter more specifically intake
    private static CANSparkMax SHOOTER;
    private static CANSparkMax INTAKE;

    private static CANSparkMax ARM_MOTOR;

    private int proximity;
    private ColorSensorV3 m_colorSensor = Constants.m_colorSensor;


    public DeviceSubsystem() {

      // ARM_MOTOR_LEFT = new CANSparkMax(Constants.ARM_MOTOR_LEFT, MotorType.kBrushless);
       ARM_MOTOR = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless); // TEMP ID. VERIFY BEFORE RUNNING PLEASEE

       SHOOTER = new CANSparkMax(Constants.SHOOTER, MotorType.kBrushless);
       INTAKE = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);
        //System.out.println("Defined potentiometer");
    //noteSensor = new AnalogPotentiometer(Constants.ULTRASONIC_SENSOR_PIN,180,0);

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
        // if (onOrOff == 1) {
        //     INTAKE.set(Constants.INTAKESPEED);
        // } else if (onOrOff == -1){
        //     INTAKE.set(-Constants.INTAKESPEED);
        // } else {
        //     INTAKE.set(0);
        // }
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
    // TODO: GET CORRECT MEASUREMENTS SO TEST ! ! ! ! ! 
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

//   if (checkRing() == True) {
//     turnOffIntakeMotor();
//   }
// }
