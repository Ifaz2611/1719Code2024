// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// REV
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Phoenix
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

// WPILIB
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// ROBOT
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveSubsystem extends SubsystemBase {
    // Motors
    private static CANSparkMax LEFT_FRONT_DRIVE_SPEED_MOTOR;
    private static CANSparkMax LEFT_BACK_DRIVE_SPEED_MOTOR;
    private static CANSparkMax RIGHT_FRONT_DRIVE_SPEED_MOTOR;
    private static CANSparkMax RIGHT_BACK_DRIVE_SPEED_MOTOR;
    // PID Subsystems
    public SwerveDirectionPIDSubsystem m_leftFrontDirection;
    public SwerveDirectionPIDSubsystem m_rightFrontDirection;
    public SwerveDirectionPIDSubsystem m_leftBackDirection;
    public SwerveDirectionPIDSubsystem m_rightBackDirection;
    // Locations of wheels - Not currently in use
    Translation2d m_frontLeftLocation = new Translation2d(0.3,0.3);
    Translation2d m_frontRightLocation = new Translation2d(0.3,-0.3);
    Translation2d m_backLeftLocation = new Translation2d(-0.3,0.3);
    Translation2d m_backRightLocation = new Translation2d(-0.3 ,-0.3);
    // Kinematics group - Not currently in use
    SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(m_frontLeftLocation,  m_backLeftLocation, m_frontRightLocation, m_backRightLocation);
    // Encoders
    public static RelativeEncoder LEFT_FRONT_DRIVE_DISTANCE_ENCODER;
    public static RelativeEncoder LEFT_BACK_DRIVE_DISTANCE_ENCODER;
    public static RelativeEncoder RIGHT_FRONT_DRIVE_DISTANCE_ENCODER;
    public static RelativeEncoder RIGHT_BACK_DRIVE_DISTANCE_ENCODER;
    // Wheel groups for each wheel
    SwerveDriveWheel LEFT_FRONT_DRIVE_WHEEL;
    SwerveDriveWheel LEFT_BACK_DRIVE_WHEEL;
    SwerveDriveWheel RIGHT_FRONT_DRIVE_WHEEL;
    SwerveDriveWheel RIGHT_BACK_DRIVE_WHEEL;
    // Gyroscope
    public static Pigeon2 DRIVE_GYRO;
    // The coordinator holds everything and is what you need to call to drive the robot
    public SwerveDriveCoordinator SWERVE_DRIVE_COORDINATOR;
    /** Creates a new SwerveSubsystem. Initiation of a swervesubsystem with 4 PIDsubsystems */
    public SwerveSubsystem(SwerveDirectionPIDSubsystem m_leftFrontDirection, SwerveDirectionPIDSubsystem m_leftBackDirection, SwerveDirectionPIDSubsystem m_rightFrontDirection, SwerveDirectionPIDSubsystem m_rightBackDirection) {
        // Motors
        LEFT_FRONT_DRIVE_SPEED_MOTOR = new CANSparkMax(
            Constants.LEFT_FRONT_DRIVE_SPEED_MOTOR_PIN,
            MotorType.kBrushless
        );
        LEFT_BACK_DRIVE_SPEED_MOTOR = new CANSparkMax(
            Constants.LEFT_BACK_DRIVE_SPEED_MOTOR_PIN, 
            MotorType.kBrushless
        );
        RIGHT_FRONT_DRIVE_SPEED_MOTOR = new CANSparkMax(
            Constants.RIGHT_FRONT_DRIVE_SPEED_MOTOR_PIN,
            MotorType.kBrushless
        );
        RIGHT_BACK_DRIVE_SPEED_MOTOR = new CANSparkMax(
            Constants.RIGHT_BACK_DRIVE_SPEED_MOTOR_PIN,
            MotorType.kBrushless
        );
        // Directions       
        this.m_leftFrontDirection = m_leftFrontDirection;
        this.m_rightFrontDirection = m_rightFrontDirection;
        this.m_leftBackDirection = m_leftBackDirection;
        this.m_rightBackDirection = m_rightBackDirection;
        // Gyro
        DRIVE_GYRO = Robot.GYRO;
        // SwerveDriveWheels
        LEFT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(LEFT_FRONT_DRIVE_SPEED_MOTOR, m_leftFrontDirection);
        LEFT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(LEFT_BACK_DRIVE_SPEED_MOTOR, m_leftBackDirection);
        RIGHT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(RIGHT_FRONT_DRIVE_SPEED_MOTOR, m_rightFrontDirection);
        RIGHT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(RIGHT_BACK_DRIVE_SPEED_MOTOR, m_rightBackDirection);
        // SwerveDriveCoordinator
        SWERVE_DRIVE_COORDINATOR = new SwerveDriveCoordinator(LEFT_FRONT_DRIVE_WHEEL, LEFT_BACK_DRIVE_WHEEL, RIGHT_FRONT_DRIVE_WHEEL, RIGHT_BACK_DRIVE_WHEEL);
        // Encoders
        LEFT_FRONT_DRIVE_DISTANCE_ENCODER = LEFT_FRONT_DRIVE_SPEED_MOTOR.getEncoder();
        LEFT_BACK_DRIVE_DISTANCE_ENCODER = LEFT_BACK_DRIVE_SPEED_MOTOR.getEncoder();
        RIGHT_FRONT_DRIVE_DISTANCE_ENCODER = RIGHT_FRONT_DRIVE_SPEED_MOTOR.getEncoder();
        RIGHT_BACK_DRIVE_DISTANCE_ENCODER = RIGHT_BACK_DRIVE_SPEED_MOTOR.getEncoder();
    }

    // Swerve Drive Coordinator
    public class SwerveDriveCoordinator {
        // 4 Swerve Wheels
        SwerveDriveWheel leftFrontWheel;
        SwerveDriveWheel leftBackWheel;
        SwerveDriveWheel rightFrontWheel;
        SwerveDriveWheel rightBackWheel;
        
        public SwerveDriveCoordinator(SwerveDriveWheel leftFrontWheel, SwerveDriveWheel leftBackWheel, SwerveDriveWheel rightFrontWheel, SwerveDriveWheel rightBackWheel) {
            // Initiate wheels
            this.leftFrontWheel = leftFrontWheel;
            this.leftBackWheel = leftBackWheel;
            this.rightFrontWheel = rightFrontWheel;
            this.rightBackWheel = rightBackWheel;
        }

        // Robot oriented drive by just calling the below drifTranslate but negating the Gyro
        public void robotOrientedDrift(double direction, double translatePower, double turnPower){
            drifTranslate(direction + DRIVE_GYRO.getAngle(), translatePower, turnPower);
        }
        // Drift Translate method moves all the wheels to move with proper direction, power, and angle
        public void drifTranslate(double direction, double translatePower, double turnPower) {
            direction = direction - DRIVE_GYRO.getAngle(); //field oriented by subtracting gyro
            // Uses convertToPower to figure out what each wheel should be doing. rotatedirection is based ont he location of the wheel
            SwerveModuleState leftFrontPosition = convertToPower(translatePower, 225, turnPower, direction);
            SwerveModuleState leftBackPosition = convertToPower(translatePower, 135, turnPower, direction);
            SwerveModuleState rightFrontPosition = convertToPower(translatePower, 315, turnPower, direction);
            SwerveModuleState rightBackPosition = convertToPower(translatePower, 45, turnPower, direction);
            // Gives the swerve states to the wheels.
            leftFrontWheel.SwerveSetWithState(leftFrontPosition);
            leftBackWheel.SwerveSetWithState(leftBackPosition);
            rightFrontWheel.SwerveSetWithState(rightFrontPosition);
            rightBackWheel.SwerveSetWithState(rightBackPosition);
        }

        // Does the trigonomotry to add vectors for direction, power, and twist.
        public SwerveModuleState convertToPower(double translatepower, double rotateDirection, double turnpower, double direction) {
            // Angle needed to turn minus the direction going is how much of the turning happens on that wheel
            double turnangle = rotateDirection - direction;
            // These 2 convert the angle to an y and x coordinate on the unit circle
            double forwardCoeficient = Math.cos(Math.toRadians(turnangle)) * turnpower;
            double leftCoeficient = Math.sin(Math.toRadians(turnangle)) * turnpower;
            // Makes the robot go forward equal to the given power
            forwardCoeficient += translatepower;
            // The power from the wheel is based on the hypotonuse of forward and left.
            translatepower = Math.sqrt(forwardCoeficient * forwardCoeficient + leftCoeficient * leftCoeficient);
            // The new rotation angle off of direction is based on the new left and forward coefficients
            Rotation2d turnangleRot = new Rotation2d(Math.atan2(leftCoeficient, forwardCoeficient) + Math.toRadians(direction));
            // Returns the state to set the swerve to
            SwerveModuleState Moduleset = new SwerveModuleState(translatepower, turnangleRot);
            return Moduleset;
        }
    }
    // Returns the closest angle between the given angles.
    public static double closestAngle(double a, double b) {
        // Get direction
        double dir = modulo(b, 360.0) - modulo(a, 360.0);
        // Convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0) {
            dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
    }

    // Returns the modulo operator of b and d
    private static double modulo(double b, double d) {
        return b % d;
    }

    // Reset the distance motors back to 0
    public void resetDistanceMotors() {
        LEFT_FRONT_DRIVE_DISTANCE_ENCODER.setPosition(0);
        LEFT_BACK_DRIVE_DISTANCE_ENCODER.setPosition(0);
        RIGHT_FRONT_DRIVE_DISTANCE_ENCODER.setPosition(0);
        RIGHT_BACK_DRIVE_DISTANCE_ENCODER.setPosition(0);
    }
    
    // Returns the average distance the motors have driven so far
    public double returnAverageDistance() {
        return (LEFT_FRONT_DRIVE_DISTANCE_ENCODER.getPosition() +
         LEFT_BACK_DRIVE_DISTANCE_ENCODER.getPosition() + 
         RIGHT_FRONT_DRIVE_DISTANCE_ENCODER.getPosition() + 
         RIGHT_BACK_DRIVE_DISTANCE_ENCODER.getPosition()) / 4;
    } 

    // Returns the current gyro angle
    public double getGYROAngle() {
        return DRIVE_GYRO.getAngle();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
