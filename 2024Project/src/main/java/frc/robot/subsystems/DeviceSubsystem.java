package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SwerveDirectionPIDCommand;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
//import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

 // Motors
public class DeviceSubsystem extends SubsystemBase {

private static CANSparkMax BOTTOM_DEVICE_MOTOR;
private static CANSparkMax TOP_DEVICE_MOTOR;
private static CANSparkMax BACK_DEVICE_MOTOR;



public DeviceSubsystem() {

     BOTTOM_DEVICE_MOTOR = new CANSparkMax(Constants.BOTTOM_DEVICE_MOTOR, MotorType.kBrushless);
     TOP_DEVICE_MOTOR = new CANSparkMax(Constants.TOP_DEVICE_MOTOR, MotorType.kBrushless); //temporary
     BACK_DEVICE_MOTOR = new CANSparkMax(Constants.BACK_DEVICE_MOTOR, MotorType.kBrushless);

}

}
