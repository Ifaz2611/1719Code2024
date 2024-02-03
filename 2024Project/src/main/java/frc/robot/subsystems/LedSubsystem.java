// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayDeque;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LedSubsystem extends SubsystemBase {
  private static Spark m_blinkin = null;
  // Static colors
  private static final double white = 0.93;
  private static final double red = 0.61;
  private static final double green = 0.77;
  private static final double gold = 0.67;
  // Patterns
  private static final double larson_scanner = -0.01;
  private static final double rainbow = 0.89;
  /** Creates a new LedSubsystem. */
  public LedSubsystem() {
    m_blinkin = new Spark(Constants.LED_PWM_PIN);
    set_at_value(gold);
    //solid_green();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  // Set at any color
  public void set_at_value(double value) {
    set(value);
  }  

  // Set to alliance color
  public void allianceColor() {
    boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
    if (isRed == true){
      set(-0.01);
      System.out.println("led RED");
    } else {
      set(0.19);
      System.out.println("led BLUE");
    }
  }

}
