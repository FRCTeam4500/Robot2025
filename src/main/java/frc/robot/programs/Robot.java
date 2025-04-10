// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.programs;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.utilities.ScoringLocations;
import frc.robot.utilities.gamepieces.GamepieceManager;
import frc.robot.utilities.logging.HoundLog;

public class Robot extends LoggedRobot {
  private Swerve swerve;
  private Superstructure structure;
  private CommandXboxController xbox;
  private CommandJoystick stick;
  private boolean climbing = false;

  /** make a robot */
  public Robot() {
    swerve = new Swerve();
    structure = new Superstructure(swerve::getPose);
    DriverStation.silenceJoystickConnectionWarning(true);
    xbox = new CommandXboxController(2);
    stick = new CommandJoystick(1);
    swerve.setDefaultCommand(swerve.angleCentric(xbox.getHID()));

    ScoringLocations.setupBlue(
        new Pose2d(3.00, 4.19, Rotation2d.fromDegrees(0)),
        new Pose2d(3.00, 3.86, Rotation2d.fromDegrees(0)),
        new Pose2d(3.09, 4.04, Rotation2d.fromDegrees(0)),
        new Pose2d(5.89, 4.02, Rotation2d.fromDegrees(180)));

    ScoringLocations.setupRed(
        new Pose2d(14.52, 3.88, Rotation2d.fromDegrees(180)),
        new Pose2d(14.52, 4.20, Rotation2d.fromDegrees(180)),
        new Pose2d(14.49, 3.98, Rotation2d.fromDegrees(180)),
        new Pose2d(11.64, 4.03, Rotation2d.fromDegrees(0)));

    setupDriveController();
    setupOperatorController();
    setupAuto();
    RobotModeTriggers.teleop().and(stick.axisGreaterThan(3, -0.1)).onTrue(structure.stow());
    RobotModeTriggers.disabled().and(() -> climbing).onTrue(structure.sing().ignoringDisable(true));
    RobotModeTriggers.disabled().onFalse(structure.stopSinging());
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
    Trigger backCoralIntake = stick.button(2).debounce(0.2);
    Trigger confirmIntake = stick.button(4);
    Trigger frontCoralIntake = stick.button(3).debounce(0.2);
    Trigger climbLocked = stick.axisGreaterThan(3, -0.1);
    Trigger climbActive = new Trigger(() -> climbing);

    levelOne.onTrue(structure.setNextCoral(CoralState.L1));
    levelTwo.onTrue(structure.setNextCoral(CoralState.L2));
    levelThree.onTrue(structure.setNextCoral(CoralState.L3));
    levelFour.onTrue(structure.setNextCoral(CoralState.L4));
    algaeHigh.onTrue(structure.setNextAlgae(AlgaeState.HIGH));
    algaeLow.onTrue(structure.setNextAlgae(AlgaeState.LOW));
    readyClimb.and(climbLocked.negate()).onTrue(structure.readyClimb());
    climb.and(climbLocked.negate()).and(climbActive.negate()).onTrue(structure.climb());
    climb
        .and(climbLocked.negate())
        .and(climbActive.negate())
        .onFalse(Commands.runOnce(() -> climbing = true));
    climb.and(climbLocked.negate()).and(climbActive).onTrue(structure.pauseClimb());
    climb
        .and(climbLocked.negate())
        .and(climbActive)
        .onFalse(Commands.runOnce(() -> climbing = false));
    backCoralIntake.onTrue(structure.backCoralIntake());
    backCoralIntake.onFalse(structure.stow());
    backCoralIntake.onTrue(swerve.targetCoralStation(false));
    stowButton.onTrue(structure.stow());
    confirmIntake.onTrue(structure.confirmIntake());
    confirmIntake.onFalse(structure.stopPlacer());
    frontCoralIntake.onTrue(structure.frontCoralIntake());
    frontCoralIntake.onFalse(structure.stow());
    frontCoralIntake.onTrue(swerve.targetCoralStation(true));

    SmartDashboard.putData("Buttons/Target L1", structure.setNextCoral(CoralState.L1));
    SmartDashboard.putData("Buttons/Target L2", structure.setNextCoral(CoralState.L2));
    SmartDashboard.putData("Buttons/Target L3", structure.setNextCoral(CoralState.L3));
    SmartDashboard.putData("Buttons/Target L4", structure.setNextCoral(CoralState.L4));
    SmartDashboard.putData("Buttons/Target High Algae", structure.setNextAlgae(AlgaeState.HIGH));
    SmartDashboard.putData("Buttons/Target Low Algae", structure.setNextAlgae(AlgaeState.LOW));
    SmartDashboard.putData("Buttons/Ready Climb", structure.readyClimb());
    SmartDashboard.putData("Buttons/Climb", structure.climb());
    SmartDashboard.putData("Buttons/Back Intake", structure.backCoralIntake());
    SmartDashboard.putData("Buttons/Front Intake", structure.frontCoralIntake());
    SmartDashboard.putData("Buttons/Ground Coral Intake", structure.groundIntake());
    SmartDashboard.putData("Buttons/Ground Algae Intake", structure.algaeGroundIntake());
    SmartDashboard.putData("Buttons/Stow", structure.stow());
    SmartDashboard.putData("Buttons/Face Back Intake", swerve.targetCoralStation(false));
    SmartDashboard.putData("Buttons/Face Front Intake", swerve.targetCoralStation(true));
  }

  private void setupDriveController() {
    Trigger onBlue =
        new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);
    Trigger onRed = onBlue.negate();
    Trigger faceForwards = new Trigger(() -> xbox.getRightY() < -0.5);
    Trigger faceBackwards = new Trigger(() -> xbox.getRightY() > 0.5);
    Trigger resetHeading = xbox.a();
    Trigger backCoralIntake = xbox.povRight().debounce(0.2);
    Trigger frontCoralIntake = xbox.povUp().debounce(0.2);
    Trigger algaeGroundIntake = xbox.povDown().debounce(0.2);
    Trigger coralGroundIntake = xbox.leftBumper();
    Trigger alignReefLeft = xbox.x();
    Trigger alignReefMiddle = xbox.leftStick();
    Trigger alignReefRight = xbox.b();
    Trigger stow = xbox.y();
    Trigger readyProcessor = xbox.rightStick();
    Trigger shoot = xbox.rightTrigger();
    Trigger faceReefCoral = xbox.rightBumper();
    Trigger stopMusic = xbox.start();

    structure.intook.and(RobotModeTriggers.teleop()).onTrue(structure.stow());

    resetHeading.and(onBlue).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(0)));
    resetHeading.and(onRed).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(180)));
    faceForwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
    faceForwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));
    faceBackwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
    faceBackwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));
    alignReefLeft
        .debounce(0.2)
        .whileTrue(
            swerve
                .reefCentric(xbox.getHID())
                .until(swerve.doesRightCameraSeeTag)
                .andThen(
                    swerve
                        .leftBranchCentric()
                        .alongWith(structure.readyNextCoral())
                        .andThen(structure.shoot())
                        .andThen(Commands.runOnce(() -> structure.stow().schedule()))));
    alignReefMiddle.onTrue(structure.readyNextAlgae());
    alignReefRight
        .debounce(0.2)
        .whileTrue(
            swerve
                .reefCentric(xbox.getHID())
                .until(swerve.doesLeftCameraSeeTag)
                .andThen(
                    swerve
                        .rightBranchCentric()
                        .alongWith(structure.readyNextCoral())
                        .andThen(structure.shoot())
                        .andThen(Commands.runOnce(() -> structure.stow().schedule()))));
    stow.onTrue(structure.stow());
    readyProcessor.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(-90)));
    readyProcessor.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(90)));

    shoot
        .and(structure.moveAfterShoot)
        .onTrue(
            structure
                .shootL1()
                .andThen(swerve.backup())
                .andThen(Commands.runOnce(() -> structure.stow().schedule())));
    shoot
        .and(structure.moveAfterShoot.negate())
        .onTrue(structure.shoot().andThen(Commands.runOnce(() -> structure.stow().schedule())));
    backCoralIntake.onTrue(swerve.targetCoralStation(false));
    frontCoralIntake.onTrue(structure.frontCoralIntake());
    frontCoralIntake.onFalse(structure.stow());
    frontCoralIntake.onTrue(swerve.targetCoralStation(true));
    coralGroundIntake.onTrue(structure.groundIntake());
    coralGroundIntake.onFalse(structure.stow());
    algaeGroundIntake.onTrue(structure.algaeGroundIntake());
    algaeGroundIntake.onFalse(structure.algaeGroundHold());
    faceReefCoral.debounce(0.1).whileTrue(swerve.reefCentric(xbox.getHID()));
    faceReefCoral.onFalse(structure.readyNextCoral());
    stopMusic.onTrue(structure.stopSinging());
  }

  private void setupAuto() {
    NamedCommands.registerCommand(
        "Ready L4", structure.readyLevel4Auto().andThen(structure.stopPlacer()));
    NamedCommands.registerCommand("Ready High Algae", structure.readyAlgaeHigh());
    NamedCommands.registerCommand("Ready Low Algae", structure.readyAlgaeLow());
    NamedCommands.registerCommand("Shoot", structure.shoot());
    NamedCommands.registerCommand("Intake", structure.backCoralIntake());
    NamedCommands.registerCommand("Stow", structure.stow());
    NamedCommands.registerCommand(
        "Auto Align Top", swerve.forwardBranchCentric().alongWith(structure.readyLevel4Auto()));
    NamedCommands.registerCommand(
        "Auto Align Bottom", swerve.backwardBranchCentric().alongWith(structure.readyLevel4Auto()));
    NamedCommands.registerCommand("Wait For Intake", Commands.waitUntil(structure.intook));

    SendableChooser<Command> chooser = new SendableChooser<>();
    chooser.setDefaultOption("None", Commands.none());
    chooser.addOption("3 Coral Left", new PathPlannerAuto("3 Piece"));
    chooser.addOption("3 Coral Right", new PathPlannerAuto("3 Piece", true));
    chooser.addOption("1 Coral Backup", new PathPlannerAuto("Backup"));
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
