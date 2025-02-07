// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.programs;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.Superstructure.CoralState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.Alignment;
import frc.robot.utilities.gamepieces.GamepieceManager;
import frc.robot.utilities.logging.HoundLog;

public class Robot extends LoggedRobot {
  private Swerve swerve = new Swerve();
  private Superstructure structure = new Superstructure();
  private CommandXboxController xbox = new CommandXboxController(2);
  private CommandJoystick stick = new CommandJoystick(1);

  /** make a robot */
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);
    swerve.setDefaultCommand(swerve.angleCentric(xbox.getHID()));

    setupDriveController();
    setupOperatorController();
    setupAuto();

    SmartDashboard.putData(
        "SWERVE CHARACTERANDSTUFF", swerve.testDriveConversionFactor(Math.PI / 2, 10));
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

    levelOne.onTrue(structure.setNextState(CoralState.L1));
    levelTwo.onTrue(structure.setNextState(CoralState.L2));
    levelThree.onTrue(structure.setNextState(CoralState.L3));
    levelFour.onTrue(structure.setNextState(CoralState.L4));
    readyClimb.onTrue(structure.readyClimb());
    climb.onTrue(structure.climb());
    coralIntake.onTrue(structure.passthroughIntake());
    coralIntake.onFalse(structure.stow());
    stowButton.onTrue(structure.stow());

    SmartDashboard.putData("Ready Level 1", structure.readyLevel1());
    SmartDashboard.putData("Ready Level 2", structure.readyLevel2());
    SmartDashboard.putData("Ready Level 3", structure.readyLevel3());
    SmartDashboard.putData("Ready Level 4", structure.readyLevel4());
    SmartDashboard.putData("Ready Climb", structure.readyClimb());
    SmartDashboard.putData("Climb", structure.climb());
    SmartDashboard.putData("Intake", structure.passthroughIntake());
    SmartDashboard.putData("Stow", structure.stow());
    SmartDashboard.putData("Shoot", structure.shoot());
    SmartDashboard.putData("Ground Intake", structure.groundIntake());

    SmartDashboard.putData(
        "Swerve Characterization", swerve.testDriveConversionFactor(Math.PI, 10));
  }

  private void setupDriveController() {
    Trigger onBlue =
        new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);
    Trigger onRed = onBlue.negate();
    Trigger faceForwards = new Trigger(() -> xbox.getRightY() < -0.5);
    Trigger faceBackwards = new Trigger(() -> xbox.getRightY() > 0.5);
    Trigger resetHeading = xbox.a();
    Trigger passthroughIntake = xbox.povUp();
    Trigger backwardsIntake = xbox.povLeft();
    Trigger algaeGroundIntake = xbox.povRight();
    Trigger coralGroundIntake = xbox.povDown();
    Trigger alignReefLeft = xbox.leftBumper();
    Trigger alignReefMiddle = xbox.leftStick();
    Trigger alignReefRight = xbox.rightBumper();
    Trigger stow = xbox.y();
    Trigger readyProcessor = xbox.rightStick();
    Trigger shoot = xbox.rightTrigger();

    resetHeading.and(onBlue).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(0)));
    resetHeading.and(onRed).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(180)));
    faceForwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
    faceForwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));
    faceBackwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
    faceBackwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));
    alignReefLeft.whileTrue(swerve.alignToReef(Alignment.Left).alongWith(structure.readyNextLevel()));
    alignReefMiddle.whileTrue(swerve.alignToReef(Alignment.Middle));
    alignReefRight.whileTrue(swerve.alignToReef(Alignment.Right).alongWith(structure.readyNextLevel()));
    stow.onTrue(structure.stow());
    readyProcessor.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(-90)));
    readyProcessor.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(90)));
    // shoot.onTrue(structure.shoot().andThen(swerve.backup()).andThen(Commands.runOnce(() -> structure.stow().schedule())));
    shoot.onTrue(structure.shoot().andThen(Commands.runOnce(() -> structure.stow().schedule())));
    passthroughIntake.onTrue(structure.passthroughIntake());
    // passthroughIntake.and(onBlue).and(swerve.closerToRight).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(55)));
    passthroughIntake.and(onBlue).and(swerve.closerToRight.negate()).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(-55)));
    passthroughIntake.and(onRed).and(swerve.closerToRight).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(-125)));
    passthroughIntake.and(onRed).and(swerve.closerToRight.negate()).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(125)));
    passthroughIntake.onFalse(structure.stow());
    backwardsIntake.onTrue(structure.backwardsIntake());
    backwardsIntake.onFalse(structure.stow());
    coralGroundIntake.onTrue(structure.groundIntake());
    coralGroundIntake.onTrue(structure.stow());
    algaeGroundIntake.onTrue(structure.algaeGroundIntake());
    algaeGroundIntake.onFalse(structure.stow());
  }

  public void setupAuto() {
    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", chooser);
    RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(chooser::getSelected));
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
