// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.ArrayDeque;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.proto.System; // WHO THE FUCK WOULD MAKE A CLASS CALLED SYSTEM 
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
// import frc.robot.RobotContainer;

public class LedSubsystem extends SubsystemBase {
  private static Spark m_blinkin = null;

  /** Creates a new LedSubsystem. */
  public LedSubsystem() {
    m_blinkin = new Spark(Constants.LED_PWM_PIN);
    set_led_color(Constants.NO_NOTE_BASELINE_GOLD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  // Set all led's to given color
  public void set_led_color(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  // Set to alliance color
  public void allianceColor() {
    boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
    if (isRed == true){
      set_led_color(-0.01); // RED ALLIANCE
    } else {
      set_led_color(0.19); // BLUE ALLIANCE
    }
  }

  // // checks if the ring is there yessir TODO: GET CORRECT MEASUREMENTS SO TEST ! ! ! ! ! 
  // public boolean checkRing(){
  //   System.out.println("Note sensor get function:" + noteSensor.get());
  //   return noteSensor.get() > 5;
  // }
}
