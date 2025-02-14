// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.programs;

import static frc.robot.utilities.ScoringLocations.allianceFlip;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.Superstructure.AlgaeState;
import frc.robot.Superstructure.CoralState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.Alignment;
import frc.robot.utilities.ScoringLocations;
import frc.robot.utilities.gamepieces.GamepieceManager;
import frc.robot.utilities.logging.HoundLog;
import java.util.Set;

public class Robot extends LoggedRobot {
  private Swerve swerve;
  private Superstructure structure;
  private CommandXboxController xbox;
  private CommandJoystick stick;

  /** make a robot */
  public Robot() {
    swerve = new Swerve();
    structure = new Superstructure(swerve::getPose);
    DriverStation.silenceJoystickConnectionWarning(true);
    xbox = new CommandXboxController(2);
    stick = new CommandJoystick(1);
    swerve.setDefaultCommand(swerve.angleCentric(xbox.getHID()));

    setupDriveController();
    setupOperatorController();
    setupAuto();
  }

  private void setupOperatorController() {
    Trigger levelOne = stick.button(10);
    Trigger levelTwo = stick.button(9);
    Trigger levelThree = stick.button(7);
    Trigger levelFour = stick.button(8);
    Trigger algaeHigh = stick.button(6);
    Trigger algaeLow = stick.button(5);
    Trigger readyClimb = stick.povUp();
    Trigger climb = stick.povDown();
    Trigger stowButton = stick.button(11);
    Trigger coralIntake = stick.button(4);

    levelOne.onTrue(structure.setNextCoral(CoralState.L1));
    levelTwo.onTrue(structure.setNextCoral(CoralState.L2));
    levelThree.onTrue(structure.setNextCoral(CoralState.L3));
    levelFour.onTrue(structure.setNextCoral(CoralState.L4));
    algaeHigh.onTrue(structure.setNextAlgae(AlgaeState.HIGH));
    algaeLow.onTrue(structure.setNextAlgae(AlgaeState.LOW));
    readyClimb.onTrue(structure.readyClimb());
    climb.onTrue(structure.climb());
    coralIntake.onTrue(structure.passthroughIntake());
    coralIntake.onFalse(structure.stow());
    stowButton.onTrue(structure.stow());

    SmartDashboard.putData("Target L1", structure.setNextCoral(CoralState.L1));
    SmartDashboard.putData("Target L2", structure.setNextCoral(CoralState.L2));
    SmartDashboard.putData("Target L3", structure.setNextCoral(CoralState.L3));
    SmartDashboard.putData("Target L4", structure.setNextCoral(CoralState.L4));
    SmartDashboard.putData("Target High Algae", structure.setNextAlgae(AlgaeState.HIGH));
    SmartDashboard.putData("Target Low Algae", structure.setNextAlgae(AlgaeState.LOW));
    SmartDashboard.putData("Ready Climb", structure.readyClimb());
    SmartDashboard.putData("Climb", structure.climb());
    SmartDashboard.putData("Intake", structure.passthroughIntake());
    SmartDashboard.putData("Stow", structure.stow());
  }

  private void setupDriveController() {
    Trigger onBlue =
        new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);
    Trigger onRed = onBlue.negate();
    Trigger faceForwards = new Trigger(() -> xbox.getRightY() < -0.5);
    Trigger faceBackwards = new Trigger(() -> xbox.getRightY() > 0.5);
    Trigger resetHeading = xbox.a();
    Trigger passthroughIntake = xbox.povRight();
    Trigger backwardsIntake = xbox.povUp();
    Trigger algaeGroundIntake = xbox.povDown();
    Trigger coralGroundIntake = xbox.povLeft();
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
    alignReefLeft.whileTrue(
        swerve.alignToReef(Alignment.Left).alongWith(structure.readyNextCoral()));
    alignReefMiddle.whileTrue(
        swerve.alignToReef(Alignment.Middle).alongWith(structure.readyNextAlgae()));
    alignReefRight.whileTrue(
        swerve.alignToReef(Alignment.Right).alongWith(structure.readyNextCoral()));
    stow.onTrue(structure.stow());
    readyProcessor.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(-90)));
    readyProcessor.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(90)));
    shoot.onTrue(
        structure
            .shoot()
            .andThen(swerve.backup())
            .andThen(Commands.runOnce(() -> structure.stow().schedule())));
    passthroughIntake.onTrue(structure.passthroughIntake());
    passthroughIntake
        .and(onBlue)
        .and(swerve.closerToRight)
        .onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(55)));
    passthroughIntake
        .and(onBlue)
        .and(swerve.closerToRight.negate())
        .onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(-55)));
    passthroughIntake
        .and(onRed)
        .and(swerve.closerToRight)
        .onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(-125)));
    passthroughIntake
        .and(onRed)
        .and(swerve.closerToRight.negate())
        .onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(125)));
    passthroughIntake.onFalse(structure.stow());
    backwardsIntake.onTrue(structure.backwardsIntake());
    backwardsIntake.onFalse(structure.stow());
    coralGroundIntake.onTrue(structure.groundIntake());
    coralGroundIntake.onFalse(structure.stow());
    algaeGroundIntake.onTrue(structure.algaeGroundIntake());
    algaeGroundIntake.onFalse(structure.stow());
  }

  public void setupAuto() {
    NamedCommands.registerCommand(
        "To A",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.A)), Set.of(swerve)));
    NamedCommands.registerCommand(
        "To B",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.B)), Set.of(swerve)));
    NamedCommands.registerCommand(
        "To C",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.C)), Set.of(swerve)));
    NamedCommands.registerCommand(
        "To D",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.D)), Set.of(swerve)));
    NamedCommands.registerCommand(
        "To E",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.E)), Set.of(swerve)));
    NamedCommands.registerCommand(
        "To F",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.F)), Set.of(swerve)));
    NamedCommands.registerCommand(
        "To G",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.G)), Set.of(swerve)));
    NamedCommands.registerCommand(
        "To H",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.H)), Set.of(swerve)));
    NamedCommands.registerCommand(
        "To I",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.I)), Set.of(swerve)));
    NamedCommands.registerCommand(
        "To J",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.J)), Set.of(swerve)));
    NamedCommands.registerCommand(
        "To K",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.K)), Set.of(swerve)));
    NamedCommands.registerCommand(
        "To L",
        Commands.defer(() -> swerve.poseCentric(allianceFlip(ScoringLocations.L)), Set.of(swerve)));
    NamedCommands.registerCommand("Ready L4", structure.readyLevel4());
    NamedCommands.registerCommand("Shoot", structure.shoot());
    NamedCommands.registerCommand("Intake", structure.passthroughIntake());
    NamedCommands.registerCommand("Stow", structure.stow());

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
