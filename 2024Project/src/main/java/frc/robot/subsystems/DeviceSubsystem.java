/*
Device Subsytem: This subsystem controls the device motors and sensors

Specifcally control over the shooter and intake motors and the proximity sensor (for sensing the note!)
*/

package frc.robot.subsystems;

// REV
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

// WPILIB
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ROBOT
import frc.robot.Constants;

// Device Subsystem
public class DeviceSubsystem extends SubsystemBase {
    // Motor for the Shooter and Intake
    private static CANSparkMax SHOOTER;
    private static CANSparkMax INTAKE;
    private int proximity;
    private ColorSensorV3 m_colorSensor = Constants.m_colorSensor;
    /* Makes a new DeviceSubsystem */
    public DeviceSubsystem() {
       SHOOTER = new CANSparkMax(Constants.SHOOTER, MotorType.kBrushless);
       INTAKE = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);
    }

    // Turns the shooter motors a given speed (forward or backward)
    public void turnShooterMotors(double speed) {
        SHOOTER.set(speed);        
    }

    // Turns the intake motors a given speed (forward or backward)
    public void turnIntakeMotors(double speed) { 
        INTAKE.set(speed*0.5);
    }

    // Turns off the shooter motors
    public void turnOffShooter(){
        SHOOTER.set(0);
    }

    // Turns off the intake motors
    public void turnOffIntakeMotors() {
        INTAKE.set(0);
    }

    // Returns a boolean based on if the ring is in the system or not
    public boolean checkRing(){ 
        // Gets the proximity of the color sensor
        proximity = m_colorSensor.getProximity();
        // Returns true if the note is sensed (within the threshold)
        if (proximity >= Constants.DISTANCE_NOTE_IN) {
            return true;
        } else {
            return false;
        }
    }
}
