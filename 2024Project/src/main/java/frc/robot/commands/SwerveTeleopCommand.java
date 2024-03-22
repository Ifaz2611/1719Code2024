// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

// import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
//find correct one later
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTeleopCommand extends Command {
  /** Creates a new MecanumDriveCommand. */

  private SwerveSubsystem m_swerveSubsystem;
  // Change this later
  private DoubleSupplier m_getX;
  private DoubleSupplier m_getY;
  private DoubleSupplier m_getTwist;
  private int countCall = 0;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("MecanumDriveCommand_1");

  public SwerveTeleopCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier getX, DoubleSupplier getY,
      DoubleSupplier getTwist) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);

    this.table.getEntry("init").setString("done");
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_getX = getX;
    this.m_getY = getY;
    this.m_getTwist = getTwist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // X and y positions
    double x = this.m_getX.getAsDouble();
    double y = this.m_getY.getAsDouble();
    double phi = this.m_getTwist.getAsDouble();

    // Scaled x and y positions
    double scaledX = scaleJoystickInput(x,Constants.JOYSTICK_SCALE_FACTOR);
    double scaledY = scaleJoystickInput(y,Constants.JOYSTICK_SCALE_FACTOR);
    double scaledPhi = scaleJoystickInput(phi,Constants.TWIST_SCALE_FACTOR);

    this.table.getEntry("countCall").setNumber(this.countCall++);
    // double test1 = this.m_getY.getAsDouble();
    this.table.getEntry("m_getLeftY").setNumber(y); // THIS IS UNSCALED
    double translatePower = Math
        .sqrt((Math.pow(scaledY, 2) + Math.pow(-scaledX, 2)) / 2);
    double direction = angleFromXY(-scaledX, scaledY);
   
    this.m_swerveSubsystem.SWERVE_DRIVE_COORDINATOR.drifTranslate(direction, translatePower*Constants.TELEOPSPEEDMODIFIER,
         scaledPhi*Constants.TELEOPTWISTMODIFIER);
    //This has a resource leak!!!

   //  this.m_swerveSubsystem.SWERVE_DRIVE_COORDINATOR.CartesianChassisSpeeds(-this.m_getX.getAsDouble(), this.m_getY.getAsDouble(), this.m_getTwist.getAsDouble());
    //System.out.println((new CANcoder(Constants.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN).getPosition().getValueAsDouble() % 1) * 360);
    //  SmartDashboard.putNumber("leftf",(new CANcoder(Constants.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN).getPosition().getValueAsDouble() % 1) * 360);
    //  SmartDashboard.putNumber("leftb",(new CANcoder(Constants.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN).getPosition().getValueAsDouble() % 1) * 360);
    //  SmartDashboard.putNumber("rightf",(new CANcoder(Constants.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN).getPosition().getValueAsDouble() % 1) * 360);
    //  SmartDashboard.putNumber("rightb",(new CANcoder(Constants.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN).getPosition().getValueAsDouble() % 1) * 360);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_swerveSubsystem.SWERVE_DRIVE_COORDINATOR.drifTranslate(0.0, 0.0, 0.0); // change to setSwerveDrive(
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static double angleFromXY(double x, double y) {
    // ??
    if (Math.abs(x) < 0.05) {
      x = 0;
    }

    if (Math.abs(y) < 0.05) {
      y = 0;
    }

    // double degree = Math.toDegrees(Math.atan(x/y));
    // double returnVal = 0;
    double a = Math.toDegrees(Math.atan2(y, x));
    // if (x > 0 && y != 0) {
    // returnVal = degree;

    // } else if (x < 0 && y != 0){
    // returnVal = degree + 180;

    // } else if (x == 0 && y > 0){
    // returnVal = 90;

    // } else if (x != 0 && y > 0){
    // returnVal = 0;

    // } else if (x == 0 && y < 0){
    // returnVal = 270;

    // } else if (x < 0 && y == 0){
    // returnVal = 180;
    // }

    return a;
  }
  
  public double scaleJoystickInput(double input, double scale_factor) {
    return (Math.signum(input) * Math.pow(Math.abs(input), scale_factor));
  }
}
