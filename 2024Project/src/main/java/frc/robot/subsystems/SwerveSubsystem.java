// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SwerveDirectionPIDCommand;
// import frc.robot.subsystems.SwerveSubsystem.SwerveDriveWheel.SwerveDriveCoordinator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveDriveWheel;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

// import frc.lib.util.SwerveModuleConstants;
// import frc.lib.util.CANSparkMaxUtil.Usage;
// import frc.lib.util.CANSparkMaxUtil;
// import frc.lib.math.OnboardModuleState;
// import frc.lib.util.CANCoderUtil;
// import frc.lib.util.CANCoderUtil.CCUsage;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  // Motors
  private static CANSparkMax LEFT_FRONT_DRIVE_SPEED_MOTOR;
  private static CANSparkMax LEFT_BACK_DRIVE_SPEED_MOTOR;
  private static CANSparkMax RIGHT_FRONT_DRIVE_SPEED_MOTOR;
  private static CANSparkMax RIGHT_BACK_DRIVE_SPEED_MOTOR;

//   private static CANSparkMax LEFT_FRONT_DRIVE_DIRECTION_MOTOR;
//   private static CANSparkMax LEFT_BACK_DRIVE_DIRECTION_MOTOR;
//   private static CANSparkMax RIGHT_FRONT_DRIVE_DIRECTION_MOTOR;
//   private static CANSparkMax RIGHT_BACK_DRIVE_DIRECTION_MOTOR;

  // Encoders
 // public static CANcoder LEFT_FRONT_DRIVE_DISTANCE_ENCODER;
//   public static CANcoder LEFT_BACK_DRIVE_DISTANCE_ENCODER;
//   public static CANcoder RIGHT_FRONT_DRIVE_DISTANCE_ENCODER;
//   public static CANcoder RIGHT_BACK_DRIVE_DISTANCE_ENCODER;
 // public static MedianPIDSource DRIVE_DISTANCE_ENCODERS;

  public static CANcoder LEFT_FRONT_DRIVE_DIRECTION_ENCODER;
  public static CANcoder LEFT_BACK_DRIVE_DIRECTION_ENCODER;
  public static CANcoder RIGHT_FRONT_DRIVE_DIRECTION_ENCODER;
  public static CANcoder RIGHT_BACK_DRIVE_DIRECTION_ENCODER;

  // Direction encoder wrapper that scales to degrees
  public static DoubleSupplier LEFT_FRONT_DRIVE_DIRECTION_SCALED;
  public static DoubleSupplier LEFT_BACK_DRIVE_DIRECTION_SCALED;
  public static DoubleSupplier RIGHT_FRONT_DRIVE_DIRECTION_SCALED;
  public static DoubleSupplier RIGHT_BACK_DRIVE_DIRECTION_SCALED;

SwerveDriveWheel LEFT_FRONT_DRIVE_WHEEL;
SwerveDriveWheel LEFT_BACK_DRIVE_WHEEL;
SwerveDriveWheel RIGHT_FRONT_DRIVE_WHEEL;
SwerveDriveWheel RIGHT_BACK_DRIVE_WHEEL;

  // Gyro
  public static Pigeon2 DRIVE_GYRO;
  public SwerveDriveCoordinator SWERVE_DRIVE_COORDINATOR;

  public SwerveSubsystem(){
     // Motors
     LEFT_FRONT_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.LEFT_FRONT_DRIVE_SPEED_MOTOR_PIN, MotorType.kBrushless);
     LEFT_BACK_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.LEFT_BACK_DRIVE_SPEED_MOTOR_PIN, MotorType.kBrushless);
     RIGHT_FRONT_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.RIGHT_FRONT_DRIVE_SPEED_MOTOR_PIN, MotorType.kBrushless);
     RIGHT_BACK_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.RIGHT_BACK_DRIVE_SPEED_MOTOR_PIN, MotorType.kBrushless);

    //  LEFT_FRONT_DRIVE_DIRECTION_MOTOR = new CANSparkMax(Constants.LEFT_FRONT_DRIVE_DIRECTION_MOTOR_PIN, MotorType.kBrushless);
    //  LEFT_BACK_DRIVE_DIRECTION_MOTOR = new CANSparkMax(Constants.LEFT_BACK_DRIVE_DIRECTION_MOTOR_PIN, MotorType.kBrushless);
    //  RIGHT_FRONT_DRIVE_DIRECTION_MOTOR = new CANSparkMax(Constants.RIGHT_FRONT_DRIVE_DIRECTION_MOTOR_PIN, MotorType.kBrushless);
    //  RIGHT_BACK_DRIVE_DIRECTION_MOTOR = new CANSparkMax(Constants.RIGHT_BACK_DRIVE_DIRECTION_MOTOR_PIN, MotorType.kBrushless);

     // Encoders
    //  LEFT_FRONT_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.LEFT_FRONT_DRIVE_DISTANCE_ENCODER_PIN);
    //  LEFT_BACK_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.LEFT_BACK_DRIVE_DISTANCE_ENCODER_PIN);
    //  RIGHT_FRONT_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.RIGHT_FRONT_DRIVE_DISTANCE_ENCODER_PIN);
    //  RIGHT_BACK_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.RIGHT_BACK_DRIVE_DISTANCE_ENCODER_PIN);

   //  DRIVE_ENCODERS = new MedianPIDSource(LEFT_FRONT_DRIVE_DISTANCE_ENCODER, LEFT_BACK_DRIVE_DISTANCE_ENCODER, RIGHT_FRONT_DRIVE_DISTANCE_ENCODER, RIGHT_BACK_DRIVE_DISTANCE_ENCODER);
     LEFT_FRONT_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN);
     LEFT_BACK_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN);
     RIGHT_FRONT_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN);
     RIGHT_BACK_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN);

     // Direction encoder wrapper that scales to degrees
     DoubleSupplier LEFT_FRONT_DRIVE_DIRECTION_SCALED = () -> LEFT_FRONT_DRIVE_DIRECTION_ENCODER.getPosition().getValueAsDouble() * 360;
     DoubleSupplier LEFT_BACK_DRIVE_DIRECTION_SCALED = () -> LEFT_FRONT_DRIVE_DIRECTION_ENCODER.getPosition().getValueAsDouble()* 360;
     DoubleSupplier RIGHT_FRONT_DRIVE_DIRECTION_SCALED =() -> LEFT_FRONT_DRIVE_DIRECTION_ENCODER.getPosition().getValueAsDouble()* 360;
     DoubleSupplier RIGHT_BACK_DRIVE_DIRECTION_SCALED = () -> LEFT_FRONT_DRIVE_DIRECTION_ENCODER.getPosition().getValueAsDouble()* 360;
     
     // Gyro
    // DRIVE_GYRO = new Pigeon2(Constants.MXP_PORT);

     // SwerveDriveWheels
    LEFT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(LEFT_FRONT_DRIVE_DIRECTION_SCALED, Constants.LEFT_FRONT_DRIVE_DIRECTION_MOTOR_PIN, LEFT_FRONT_DRIVE_SPEED_MOTOR);
    LEFT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(LEFT_BACK_DRIVE_DIRECTION_SCALED, Constants.LEFT_BACK_DRIVE_DIRECTION_MOTOR_PIN, LEFT_BACK_DRIVE_SPEED_MOTOR);
    RIGHT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel( RIGHT_FRONT_DRIVE_DIRECTION_SCALED, Constants.RIGHT_FRONT_DRIVE_DIRECTION_MOTOR_PIN, RIGHT_FRONT_DRIVE_SPEED_MOTOR);
    RIGHT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(RIGHT_BACK_DRIVE_DIRECTION_SCALED, Constants.RIGHT_BACK_DRIVE_DIRECTION_MOTOR_PIN, RIGHT_BACK_DRIVE_SPEED_MOTOR);
    
    // SwerveDriveCoordinator
     SWERVE_DRIVE_COORDINATOR = new SwerveDriveCoordinator(LEFT_FRONT_DRIVE_WHEEL, LEFT_BACK_DRIVE_WHEEL, RIGHT_FRONT_DRIVE_WHEEL, RIGHT_BACK_DRIVE_WHEEL);
    
}
public class degreeSupplier{
public CANcoder encoder;
public degreeSupplier(CANcoder encoder){
this.encoder=encoder;


} 
public double getDegrees(){
return (this.encoder.getPosition().getValueAsDouble() % 1)*360;

}
public double getAsDouble(){
return (this.encoder.getPosition().getValueAsDouble() % 1)*360;

}


}



public class SwerveDriveCoordinator
    {
        SwerveDriveWheel leftFrontWheel;
        SwerveDriveWheel leftBackWheel;
        SwerveDriveWheel rightFrontWheel;
        SwerveDriveWheel rightBackWheel;

    public SwerveDirectionPIDCommand leftFrontWheeldirectionController;
    public SwerveDirectionPIDCommand leftBackWheeldirectionController;
    public SwerveDirectionPIDCommand rightFrontWheeldirectionController;
    public SwerveDirectionPIDCommand rightBackWheeldirectionController;
    
        public SwerveDriveCoordinator(SwerveDriveWheel leftFrontWheel, SwerveDriveWheel leftBackWheel, SwerveDriveWheel rightFrontWheel, SwerveDriveWheel rightBackWheel)
        {
        this.leftFrontWheel = leftFrontWheel;
        this.leftBackWheel = leftBackWheel;
        this.rightFrontWheel = rightFrontWheel;
        this.rightBackWheel = rightBackWheel;

        this.leftFrontWheeldirectionController = new SwerveDirectionPIDCommand( leftFrontWheel.directionSensor,leftFrontWheel );
        this.leftBackWheeldirectionController = new SwerveDirectionPIDCommand( leftBackWheel.directionSensor,leftBackWheel );
        this.rightFrontWheeldirectionController = new SwerveDirectionPIDCommand( rightFrontWheel.directionSensor,rightFrontWheel );
        this.rightBackWheeldirectionController = new SwerveDirectionPIDCommand( rightBackWheel.directionSensor,rightBackWheel );
        }
        public void setSwerveDrive(double direction, double translatePower, double turnPower)
{
    if ((translatePower == 0.0) && (turnPower != 0.0))
    {
        inplaceTurn(turnPower);
    }
    else
    {
        translateTurn(direction, translatePower, turnPower);
    }
}
        public void translate(double direction, double power)
        {
            leftFrontWheel.setDirection(direction);
            leftBackWheel.setDirection(direction);
            rightFrontWheel.setDirection(direction);
            rightBackWheel.setDirection(direction);
        
            leftFrontWheel.speedMotors(power);
            leftBackWheel.speedMotors(power);
            rightFrontWheel.speedMotors(power);
            rightBackWheel.speedMotors(power);
        }
        public void inplaceTurn(double power)
{
    leftFrontWheel.setDirection(135.0);
    leftBackWheel.setDirection(45.0);
    rightFrontWheel.setDirection(-45.0);
    rightBackWheel.setDirection(-135.0);

    leftFrontWheel.speedMotors(power);
    leftBackWheel.speedMotors(power);
    rightFrontWheel.speedMotors(power);
    rightBackWheel.speedMotors(power);
}
public void translateTurn(double direction, double translatePower, double turnPower)
{
    double turnAngle = turnPower * 45.0;

    // if the left front wheel is in the front
    if (closestAngle(direction, 135.0) >= 90.0)
    {
        leftFrontWheel.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        leftFrontWheel.setDirection(direction - turnAngle);
    }
    // if the left back wheel is in the front
    if (closestAngle(direction, 225.0) > 90.0)
    {
        leftBackWheel.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        leftBackWheel.setDirection(direction - turnAngle);
    }
    // if the right front wheel is in the front
    if (closestAngle(direction, 45.0) > 90.0)
    {
        rightFrontWheel.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        rightFrontWheel.setDirection(direction - turnAngle);
    }
    // if the right back wheel is in the front
    if (closestAngle(direction, 315.0) >= 90.0)
    {
        rightBackWheel.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        rightBackWheel.setDirection(direction - turnAngle);
    }

    leftFrontWheel.speedMotors(translatePower);
    leftBackWheel.speedMotors(translatePower);
    rightFrontWheel.speedMotors(translatePower);
    rightBackWheel.speedMotors(translatePower);
}

    


   
}
/**
 * Get the closest angle between the given angles.
 */
public static double closestAngle(double a, double b)
{
        // get direction
        double dir = modulo(b, 360.0) - modulo(a, 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0)
        {
                dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
}
private static double modulo(double b, double d) {
    
    return b % d;
}


  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

}

