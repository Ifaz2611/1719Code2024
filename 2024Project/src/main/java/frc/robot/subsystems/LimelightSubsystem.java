// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  // Angle of limelight from the horizontal
  private static final double limelightMountAngleDegrees = 0;
  // Offset of limelight center in inches from floor
  private static final  double limelightLensHeightInches = 31.0; 
  // Offset of goal center in inches from floor
  private static final double goalHeightInches = 15.0;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Get angle for shooter head (returns double angle from 0 to 360)
  public double getShootingAngle() {
    return Math.atan2(Constants.SPEAKER_HEIGHT, getDistance() - Constants.DISTANCE_LIMELIGHT_TO_SHOOTER);
  }

  // Get distance to april tag (returns double)
  public double getDistance() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tid = table.getEntry("tid");
    double aprilTagId = tid.getInteger(0);
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0; 

    // distance from the target to the floor
    double goalHeightInches = 60.0; 
    if (aprilTagId == 4 || aprilTagId == 7) {
      double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

      //calculate distance
      double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
      return distanceFromLimelightToGoalInches;
    }
    return 0.0;
  }
  
  // Returns angle to any april tag
  public double getAngleToTag() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    double targetOffsetAngle_Horizontal = tx.getDouble(0.0);
    return targetOffsetAngle_Horizontal;
  }
  // Returns angle to traps
  public double getAngleToTrap() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tid = table.getEntry("tid");
    double aprilTagId = tid.getInteger(0);
     if (aprilTagId == 15 || aprilTagId == 14 || aprilTagId == 16 || 
      aprilTagId == 11 || aprilTagId == 12 || aprilTagId == 13) {
      return getAngleToTag();
    }
    return 0.0;
  }
  // Returns angle to speakers
  public double getAngleToSpeaker() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tid = table.getEntry("tid");
    double aprilTagId = tid.getInteger(0);
     if (aprilTagId == 7 || aprilTagId == 4) {
      return getAngleToTag();
    }
    return 0.0;
  }
  // Returns angle to amps
  public double getAngleToAmp() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tid = table.getEntry("tid");
    double aprilTagId = tid.getInteger(0);
     if (aprilTagId == 6 || aprilTagId == 5) {
      return getAngleToTag();
    }
    return 0.0;
  }
}
