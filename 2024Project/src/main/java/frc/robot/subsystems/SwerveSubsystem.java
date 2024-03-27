// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveSubsystem extends SubsystemBase {
    /** Creates a new SwerveSubsystem. */

    // Motors
    private static CANSparkMax LEFT_FRONT_DRIVE_SPEED_MOTOR;
    private static CANSparkMax LEFT_BACK_DRIVE_SPEED_MOTOR;
    private static CANSparkMax RIGHT_FRONT_DRIVE_SPEED_MOTOR;
    private static CANSparkMax RIGHT_BACK_DRIVE_SPEED_MOTOR;

    public SwerveDirectionPIDSubsystem m_leftFrontDirection;
    public SwerveDirectionPIDSubsystem m_rightFrontDirection;
    public SwerveDirectionPIDSubsystem m_leftBackDirection;
    public SwerveDirectionPIDSubsystem m_rightBackDirection;

    Translation2d m_frontLeftLocation = new Translation2d(0.3,0.3);
    Translation2d m_frontRightLocation = new Translation2d(0.3,-0.3);
    Translation2d m_backLeftLocation = new Translation2d(-0.3,0.3);
    Translation2d m_backRightLocation = new Translation2d(-0.3 ,-0.3);;

    SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(m_frontLeftLocation,  m_backLeftLocation, m_frontRightLocation, m_backRightLocation);


    // Encoders
    public static RelativeEncoder LEFT_FRONT_DRIVE_DISTANCE_ENCODER;
    public static RelativeEncoder LEFT_BACK_DRIVE_DISTANCE_ENCODER;
    public static RelativeEncoder RIGHT_FRONT_DRIVE_DISTANCE_ENCODER;
    public static RelativeEncoder RIGHT_BACK_DRIVE_DISTANCE_ENCODER;


    SwerveDriveWheel LEFT_FRONT_DRIVE_WHEEL;
    SwerveDriveWheel LEFT_BACK_DRIVE_WHEEL;
    SwerveDriveWheel RIGHT_FRONT_DRIVE_WHEEL;
    SwerveDriveWheel RIGHT_BACK_DRIVE_WHEEL;

    // Gyro
    public static Pigeon2 DRIVE_GYRO;
    public SwerveDriveCoordinator SWERVE_DRIVE_COORDINATOR;

    public SwerveSubsystem(SwerveDirectionPIDSubsystem m_leftFrontDirection,
            SwerveDirectionPIDSubsystem m_leftBackDirection, SwerveDirectionPIDSubsystem m_rightFrontDirection,
            SwerveDirectionPIDSubsystem m_rightBackDirection) {
        // Motors
        LEFT_FRONT_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.LEFT_FRONT_DRIVE_SPEED_MOTOR_PIN,
                MotorType.kBrushless);
        LEFT_BACK_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.LEFT_BACK_DRIVE_SPEED_MOTOR_PIN, MotorType.kBrushless);
        RIGHT_FRONT_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.RIGHT_FRONT_DRIVE_SPEED_MOTOR_PIN,
                MotorType.kBrushless);
        RIGHT_BACK_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.RIGHT_BACK_DRIVE_SPEED_MOTOR_PIN,
                MotorType.kBrushless);

        this.m_leftFrontDirection = m_leftFrontDirection;
        this.m_rightFrontDirection = m_rightFrontDirection;
        this.m_leftBackDirection = m_leftBackDirection;
        this.m_rightBackDirection = m_rightBackDirection;

        // Gyro
        DRIVE_GYRO = Robot.getGYRO();

        // SwerveDriveWheels
        LEFT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(LEFT_FRONT_DRIVE_SPEED_MOTOR, m_leftFrontDirection);
        LEFT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(LEFT_BACK_DRIVE_SPEED_MOTOR, m_leftBackDirection);
        RIGHT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(RIGHT_FRONT_DRIVE_SPEED_MOTOR, m_rightFrontDirection);
        RIGHT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(RIGHT_BACK_DRIVE_SPEED_MOTOR, m_rightBackDirection);

        // SwerveDriveCoordinator
        SWERVE_DRIVE_COORDINATOR = new SwerveDriveCoordinator(LEFT_FRONT_DRIVE_WHEEL, LEFT_BACK_DRIVE_WHEEL,
                RIGHT_FRONT_DRIVE_WHEEL, RIGHT_BACK_DRIVE_WHEEL);


        // Encoders
        LEFT_FRONT_DRIVE_DISTANCE_ENCODER = LEFT_FRONT_DRIVE_SPEED_MOTOR.getEncoder();
        LEFT_BACK_DRIVE_DISTANCE_ENCODER = LEFT_BACK_DRIVE_SPEED_MOTOR.getEncoder();
        RIGHT_FRONT_DRIVE_DISTANCE_ENCODER = RIGHT_FRONT_DRIVE_SPEED_MOTOR.getEncoder();
        RIGHT_BACK_DRIVE_DISTANCE_ENCODER = RIGHT_BACK_DRIVE_SPEED_MOTOR.getEncoder();

    }


    public class SwerveDriveCoordinator {
        SwerveDriveWheel leftFrontWheel;
        SwerveDriveWheel leftBackWheel;
        SwerveDriveWheel rightFrontWheel;
        SwerveDriveWheel rightBackWheel;

        public SwerveDriveCoordinator(SwerveDriveWheel leftFrontWheel, SwerveDriveWheel leftBackWheel,
                SwerveDriveWheel rightFrontWheel, SwerveDriveWheel rightBackWheel) {
            this.leftFrontWheel = leftFrontWheel;
            this.leftBackWheel = leftBackWheel;
            this.rightFrontWheel = rightFrontWheel;
            this.rightBackWheel = rightBackWheel;

        }



      
        public void robotOrientedDrift(double direction, double translatePower, double turnPower){
            drifTranslate(direction+DRIVE_GYRO.getAngle(), translatePower, turnPower);
        }
        public void drifTranslate(double direction, double translatePower, double turnPower){
                      direction = direction - DRIVE_GYRO.getAngle(); //gyro on hootide hence +90 took out +90 +30 + 180

                      SwerveModuleState leftFrontPosition = convertToPower(translatePower, 225, turnPower, direction);
                      SwerveModuleState leftBackPosition = convertToPower(translatePower, 135, turnPower, direction);
                      SwerveModuleState rightFrontPosition = convertToPower(translatePower, 315, turnPower, direction);
                      SwerveModuleState rightBackPosition = convertToPower(translatePower, 45, turnPower, direction);
          leftFrontWheel.SwerveSetWithState(leftFrontPosition);
          leftBackWheel.SwerveSetWithState(leftBackPosition);
          rightFrontWheel.SwerveSetWithState(rightFrontPosition);
          rightBackWheel.SwerveSetWithState(rightBackPosition);
    }
    public SwerveModuleState convertToPower(double translatepower, double rotateDirection, double turnpower, double direction ){
        double turnangle = rotateDirection-direction;
        double forwardCoeficient = Math.cos(Math.toRadians(turnangle))*turnpower;
        double leftCoeficient = Math.sin(Math.toRadians(turnangle))*turnpower;
        forwardCoeficient+=translatepower;
        translatepower = Math.sqrt(forwardCoeficient*forwardCoeficient+leftCoeficient*leftCoeficient);
        Rotation2d turnangleRot = new Rotation2d(Math.atan2(leftCoeficient, forwardCoeficient)+ Math.toRadians(direction));
SwerveModuleState Moduleset = new SwerveModuleState( translatepower, turnangleRot);

return Moduleset;
    }

    }

    /**
     * Get the closest angle between the given angles.
     */
    public static double closestAngle(double a, double b) {
        // get direction
        double dir = modulo(b, 360.0) - modulo(a, 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0) {
            dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
    }

    private static double modulo(double b, double d) {

        return b % d;
    }

    // reset the distance motors back to 0
    public void resetDistanceMotors() {
        LEFT_FRONT_DRIVE_DISTANCE_ENCODER.setPosition(0);
        LEFT_BACK_DRIVE_DISTANCE_ENCODER.setPosition(0);
        RIGHT_FRONT_DRIVE_DISTANCE_ENCODER.setPosition(0);
        RIGHT_BACK_DRIVE_DISTANCE_ENCODER.setPosition(0);
    }
    

    public double returnAverageDistance() {
        return (LEFT_FRONT_DRIVE_DISTANCE_ENCODER.getPosition() +
         LEFT_BACK_DRIVE_DISTANCE_ENCODER.getPosition() + 
         RIGHT_FRONT_DRIVE_DISTANCE_ENCODER.getPosition() + 
         RIGHT_BACK_DRIVE_DISTANCE_ENCODER.getPosition())/4;
    } 
    public double getGYROAngle(){
        return DRIVE_GYRO.getAngle();
    
}

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
    }

}
