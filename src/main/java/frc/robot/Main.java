// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.programs.Robot;
import frc.robot.programs.arm.ArmTest;
import frc.robot.programs.ramp.RampTest;

public final class Main {
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
