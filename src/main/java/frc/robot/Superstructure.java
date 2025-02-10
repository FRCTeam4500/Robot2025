package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.Set;
import java.util.function.Supplier;

/**
 * A class that holds together the top half of our robot. Basically everything except the
 * drivetrain. It exposes command factories which combine the various subsystems
 */
public class Superstructure implements Loggable {
  // Create objects for all non-drivebase subsystems
  private Mechanism2d robotMech;
  private Climber climber;
  private Elevator elevator;
  private Ramp ramp;
  private Arm arm;
  private Placer placer;
  private Supplier<Pose2d> robotPose;

  private CoralState nextState;

  public Superstructure(Supplier<Pose2d> robotPose) {
    robotMech = new Mechanism2d(3, 3);
    climber = new Climber();
    elevator = new Elevator();
    ramp = new Ramp();
    arm = new Arm();
    placer = new Placer();
    this.robotPose = robotPose;
    nextState = CoralState.L4;

    configureMech();

    RobotModeTriggers.teleop().onTrue(stow());
  }

  private void configureMech() {
    robotMech.getRoot("Climber Root", 1.05, 0.3).append(climber.mech);
    elevator.armHolder.append(arm.mech);
    robotMech.getRoot("Elevator Root", 1.7, 0.1).append(elevator.mech);
    robotMech.getRoot("Ramp Root", 1.6, 0.3).append(ramp.mech);
    SmartDashboard.putData("Robot Mech", robotMech);
  }

  public void log(String path) {
    // Call log() methods for contained subsystems
    HoundLog.log(path, "Climber", climber);
    HoundLog.log(path, "Elevator", elevator);
    HoundLog.log(path, "Placer", placer);
    HoundLog.log(path, "Ramp", ramp);
    HoundLog.log(path, "Arm", arm);
    HoundLog.log(path, "Next State", nextState);

    double percentUp = elevator.getExtension() / 0.95;
    Transform3d elevatorStagePose =
        new Transform3d(0.09, 0, 0.14 + .55 * percentUp, new Rotation3d());
    Transform3d carriagePose = new Transform3d(0.09, 0, 0.2 + 1.13 * percentUp, new Rotation3d());
    Transform3d armPose =
        new Transform3d(
            0.167,
            0,
            0.41 + 1.13 * percentUp,
            new Rotation3d(0, Math.toRadians(-arm.getAngle()), 0));
    Transform3d rampPose =
        new Transform3d(0, 0, 0.62, new Rotation3d(0, Math.toRadians(-ramp.getAngle()), 0));
    Transform3d climberPose =
        new Transform3d(
            -0.407, 0, 0.255, new Rotation3d(0, Math.toRadians(-climber.getAngle()), 0));
    HoundLog.log(
        path,
        "Component Poses",
        new Transform3d[] {elevatorStagePose, carriagePose, armPose, rampPose, climberPose});

    Pose3d robot = new Pose3d(robotPose.get());
    Transform3d piece = new Transform3d(0.49, 0, -0.1, new Rotation3d(0, Math.PI / 2, 0));
    Transform3d pieceSideways = new Transform3d(0.49, 0, -0.1, new Rotation3d(0, 0, Math.PI / 2));
    HoundLog.log("Held Piece", robot.transformBy(armPose).transformBy(piece));
    HoundLog.log("Held Piece Sideways", robot.transformBy(armPose).transformBy(pieceSideways));
  }

  public Command setNextState(CoralState state) {
    return Commands.runOnce(() -> nextState = state);
  }

  public Command readyNextLevel() {
    return Commands.defer(
        () -> {
          switch (nextState) {
            case L1:
              return readyLevel1();
            case L2:
              return readyLevel2();
            case L3:
              return readyLevel3();
            case L4:
              return readyLevel4();
          }
          return Commands.none();
        },
        Set.of());
  }

  public Command readyLevel1() {
    return arm.placeL1()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.level1()));
  }

  public Command readyLevel2() {
    return arm.placeL2()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.level2()));
  }

  public Command readyLevel3() {
    return arm.placeL3()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.level3()));
  }

  public Command readyLevel4() {
    return arm.placeL4()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.level4()));
  }

  public Command readyClimb() {
    return arm.stow()
        .alongWith(placer.stop())
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.stow()))
        .alongWith(ramp.hide().andThen(climber.ready()));
  }

  public Command climb() {
    return climber.climb();
  }

  public Command algaeGroundIntake() {
    return Commands.none();
  }

  public Command groundIntake() {
    return arm.ground()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.groundPickup()))
        .alongWith(placer.intake());
  }

  public Command passthroughIntake() {
    return arm.ground()
        .alongWith(ramp.show())
        .until(arm.canMoveElevator)
        .andThen(elevator.handoff())
        .andThen(arm.handoff().alongWith(placer.intake()));
  }

  public Command backwardsIntake() {
    return arm.stationPickup()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.stationPickup()))
        .andThen(placer.intake());
  }

  public Command shoot() {
    return placer.eject().andThen(Commands.waitSeconds(0.5));
  }

  public Command stow() {
    return placer
        .stop()
        .andThen(arm.stow())
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.stow()))
        .alongWith(climber.stow().andThen(ramp.show()));
  }

  public static enum CoralState {
    L1,
    L2,
    L3,
    L4;
  }
}
