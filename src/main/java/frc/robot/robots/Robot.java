// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robots;

import com.pathplanner.lib.auto.AutoBuilder;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.Alignment;
import frc.robot.utilities.gamepieces.GamepieceManager;
import frc.robot.utilities.logging.HoundLog;

public class Robot extends TimedRobot {
  private Swerve swerve = new Swerve();
  private Superstructure structure = new Superstructure();
  private CommandXboxController xbox = new CommandXboxController(2);
  private CommandJoystick stick = new CommandJoystick(1);

  /** make a robot */
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);
    swerve.setDefaultCommand(swerve.angleCentric(xbox.getHID()));

    setupLogging();
    setupDriveController();
    setupOperatorController();
    setupAuto();
  }

  private void setupOperatorController() {
    Trigger levelOne = stick.button(10);
    Trigger levelTwo = stick.button(9);
    Trigger levelThree = stick.button(7);
    Trigger levelFour = stick.button(8);
    Trigger readyClimb = stick.button(5);
    Trigger climb = stick.button(6);
    Trigger stowButton = stick.button(11);
    Trigger coralIntake = stick.button(4);
    // Trigger groundIntake = stick.button(2);    // no z
    // Trigger readyProcessor = stick.button(3); // no z
    // Trigger placeAlgae = stick.button(1);
    // Trigger highAlgae = stick.povUp();
    // Trigger lowAlgae = stick.povDown();

    levelOne.onTrue(structure.readyLevel1());
    levelTwo.onTrue(structure.readyLevel2());
    levelThree.onTrue(structure.readyLevel3());
    levelFour.onTrue(structure.readyLevel4());
    readyClimb.onTrue(structure.readyClimb());
    climb.onTrue(structure.climb());
    coralIntake.onTrue(structure.intake());
    coralIntake.onFalse(structure.stow());
    stowButton.onTrue(structure.stow());
    // readyProcessor.onTrue(structure.readyProcessor());
    // placeAlgae.onTrue(structure.readyplacealgae());
    // highAlgae.onTrue(structure.);
    // lowAlgae.onTrue(structure.());

    SmartDashboard.putData("Ready Level 1", structure.readyLevel1());
    SmartDashboard.putData("Ready Level 2", structure.readyLevel2());
    SmartDashboard.putData("Ready Level 3", structure.readyLevel3());
    SmartDashboard.putData("Ready Level 4", structure.readyLevel4());
    SmartDashboard.putData("Ready Climb", structure.readyClimb());
    SmartDashboard.putData("Climb", structure.climb());
    SmartDashboard.putData("Intake", structure.intake());
    SmartDashboard.putData("Stow", structure.stow());
  }

  private void setupDriveController() {
    Trigger onBlue =
        new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);
    Trigger onRed = onBlue.negate();
    Trigger faceForwards = new Trigger(() -> xbox.getRightY() < -0.5);
    Trigger faceBackwards = new Trigger(() -> xbox.getRightY() > 0.5);
    Trigger resetHeading = xbox.a();
    Trigger alignReefLeft = xbox.povLeft();
    Trigger alignReefMiddle = xbox.povUp();
    Trigger alignReefRight = xbox.povRight();
    Trigger leftStationIntake = xbox.x();
    Trigger rightStationIntake = xbox.b();

    resetHeading.and(onBlue).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(0)));
    resetHeading.and(onRed).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(180)));
    faceForwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
    faceForwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));
    faceBackwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
    faceBackwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));
    alignReefLeft.whileTrue(swerve.alignToReef(Alignment.Left));
    alignReefMiddle.whileTrue(swerve.alignToReef(Alignment.Middle));
    alignReefRight.whileTrue(swerve.alignToReef(Alignment.Right));
    leftStationIntake.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(-55)));
    rightStationIntake.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(55)));
    leftStationIntake.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(125)));
    rightStationIntake.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(-125)));
    rightStationIntake.or(leftStationIntake).onTrue(structure.intake());
    rightStationIntake.or(leftStationIntake).onFalse(structure.stow());
  }

  public void setupAuto() {
    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", chooser);
    RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(chooser::getSelected));
  }

  public void setupLogging() {
    HoundLog.setEnabled(true);
    HoundLog.setPdh(new PowerDistribution());
    HoundLog.setOptions(
        new DogLogOptions(() -> !DriverStation.isFMSAttached(), true, true, true, true, 1000));
    GamepieceManager.resetField();
  }

  @Override
  public void robotPeriodic() {
    double start = Timer.getFPGATimestamp();
    HoundLog.log("Swerve", swerve);
    HoundLog.log("Superstrucutre", structure);
    double loggingLoop = Timer.getFPGATimestamp() - start;

    start = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();
    double commandsLoop = Timer.getFPGATimestamp() - start;

    HoundLog.log("DogLog", "Logging Loop Time", loggingLoop * 1000);
    HoundLog.log("DogLog", "Commands Loop Time", commandsLoop * 1000);
    HoundLog.log("DogLog", "Total Loop Time", 1000 * (commandsLoop + loggingLoop));
  }

  @Override
  public void simulationPeriodic() {
    GamepieceManager.simulate();
  }
}
