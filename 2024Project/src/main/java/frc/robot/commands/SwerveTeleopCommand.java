// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// JAVA
import java.util.function.DoubleSupplier;

// WPILIB
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

// ROBOT
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTeleopCommand extends Command {  
  private SwerveSubsystem m_swerveSubsystem;
  private DoubleSupplier m_getX;
  private DoubleSupplier m_getY;
  private DoubleSupplier m_getTwist;
  private int countCall = 0;
  // Network Tables  
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("MecanumDriveCommand_1");
  
  /** Creates a new SwerveTeleopCommand. */
  public SwerveTeleopCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier getX, DoubleSupplier getY, DoubleSupplier getTwist) {
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get x,y positions and angle phi
    double x = this.m_getX.getAsDouble();
    double y = this.m_getY.getAsDouble();
    double phi = this.m_getTwist.getAsDouble();

    // Scaled x and y positions
    double scaledX = scaleJoystickInput(x,Constants.JOYSTICK_SCALE_FACTOR,0);
    double scaledY = scaleJoystickInput(y,Constants.JOYSTICK_SCALE_FACTOR,0);
    double scaledPhi = scaleJoystickInput(phi,Constants.TWIST_SCALE_FACTOR,Constants.TWIST_DEAD_ZONE);
//System.out.println("phi" + phi + " x:" + scaledPhi);
    // Update table "countCall"
    this.table.getEntry("countCall").setNumber(this.countCall++);
    // Update table "m_getLefty", smart dashboard shows unscaled y
    this.table.getEntry("m_getLeftY").setNumber(y);
    // Calculate translatePower and direction
    double translatePower = Math.sqrt((Math.pow(scaledY, 2) + Math.pow(-scaledX, 2)) / 2);
    double direction = angleFromXY(-scaledX, scaledY);
    // Send the command to move given direction and angle
    this.m_swerveSubsystem.SWERVE_DRIVE_COORDINATOR.drifTranslate(direction, translatePower*Constants.TELEOPSPEEDMODIFIER, scaledPhi*Constants.TELEOPTWISTMODIFIER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Sets the swerve drive to 0
    this.m_swerveSubsystem.SWERVE_DRIVE_COORDINATOR.drifTranslate(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Get angle from the x and y
  public static double angleFromXY(double x, double y) {
    // If below |0.5| then set to 0
    if (Math.abs(x) < 0.05) {
      x = 0;
    }
    if (Math.abs(y) < 0.05) {
      y = 0;
    }
    // Get the angle and return it
    double a = Math.toDegrees(Math.atan2(y, x));
    return a;
  }
  
  // Return scaled joystick input given a scale factor
  public double scaleJoystickInput(double input, double scale_factor, double dead_zone) {
    double v = Math.abs(input);
    if (v < dead_zone) return 0;

    return Math.signum(input) * (Math.pow((v-dead_zone)/(1-dead_zone), scale_factor));
  }
}
