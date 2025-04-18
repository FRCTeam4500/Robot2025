package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.orchestra.Orc;
import frc.robot.subsystems.placer.Placer;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.utilities.StopTilting;
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
  private Climber climber;
  private Elevator elevator;
  private Ramp ramp;
  private Arm arm;
  private Placer placer;
  private Supplier<Pose2d> robotPose;

  private CoralState nextCoral;
  private AlgaeState nextAlgae;

  private boolean isPlacingAlgae = false;

  private boolean shouldMoveBackAfterShoot = false;
  public Trigger moveAfterShoot = new Trigger(() -> shouldMoveBackAfterShoot);
  public Trigger intook;

  public Superstructure(Supplier<Pose2d> robotPose) {
    climber = new Climber();
    elevator = new Elevator();
    ramp = new Ramp();
    arm = new Arm();
    placer = new Placer();
    intook = placer.intook;
    this.robotPose = robotPose;
    nextCoral = CoralState.L4;
    nextAlgae = AlgaeState.HIGH;
    StopTilting.setupSuperstructure(
        new Transform3d[] {
          new Transform3d(0.25, 0, 0, Rotation3d.kZero),
          new Transform3d(0, 0, 0.4, Rotation3d.kZero),
          new Transform3d(0, 0, 0.1, Rotation3d.kZero),
          new Transform3d(0.2, 0, 0, Rotation3d.kZero),
          new Transform3d(0.1, 0, 0, Rotation3d.kZero)
        },
        new double[] {5.44311, 2.5197056, 1.81437, 3.17515, 2.5});
  }

  public void log(String path) {
    // Call log() methods for contained subsystems
    HoundLog.log(path, "Climber", climber);
    HoundLog.log(path, "Elevator", elevator);
    HoundLog.log(path, "Placer", placer);
    HoundLog.log(path, "Ramp", ramp);
    HoundLog.log(path, "Arm", arm);

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
    HoundLog.log("Held Piece", robot.transformBy(armPose).transformBy(piece));
    StopTilting.updateCenterOfMass(
        new Transform3d[] {armPose, elevatorStagePose, carriagePose, climberPose, rampPose});
  }

  public Command confirmIntake() {
    return placer.intake();
  }

  public Command setNextCoral(CoralState state) {
    return Commands.runOnce(() -> nextCoral = state);
  }

  public Command setNextAlgae(AlgaeState state) {
    return Commands.runOnce(() -> nextAlgae = state);
  }

  public Command readyNextCoral() {
    return Commands.defer(
        () -> {
          isPlacingAlgae = false;
          switch (nextCoral) {
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

  public Command readyNextAlgae() {
    return Commands.defer(
        () -> {
          isPlacingAlgae = true;
          switch (nextAlgae) {
            case LOW:
              return readyAlgaeLow();
            case HIGH:
              return readyAlgaeHigh();
          }
          return Commands.none();
        },
        Set.of());
  }

  public Command readyLevel1() {
    return arm.placeL1()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.level1()))
        .alongWith(Commands.runOnce(() -> shouldMoveBackAfterShoot = true));
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
    return arm.stow()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.level4()))
        .andThen(arm.placeL4());
  }

  public Command readyLevel4Auto() {
    return arm.placeL4()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.level4()));
  }

  public Command readyAlgaeHigh() {
    return arm.dislodge()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.highAlgae()))
        .alongWith(placer.eject(placer.dealgifySpeed));
  }

  public Command readyAlgaeLow() {
    return arm.dislodge()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.lowAlgae()))
        .alongWith(placer.eject(placer.dealgifySpeed));
  }

  public Command stopPlacer() {
    return placer.stop();
  }

  public Command algaeGroundHold() {
    return placer
        .stop()
        .andThen(arm.algaeGroundHold())
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.groundAlgaeHold()));
  }

  public Command readyClimb() {
    return arm.climb()
        .alongWith(placer.stop())
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.stow()))
        .alongWith(ramp.hide().andThen(climber.ready()));
  }

  public Command climb() {
    return climber.climb();
  }

  public Command pauseClimb() {
    return climber.pause();
  }

  public Command sing() {
    return Orc.startSinging();
  }

  public Command stopSinging() {
    return Orc.stopSinging();
  }

  public Command algaeGroundIntake() {
    return arm.algaeGroundIntake()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.groundAlgaeIntake()))
        .alongWith(placer.intakeGround());
  }

  public Command groundIntake() {
    return arm.ground()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.groundPickup()))
        .alongWith(placer.intakeGround());
  }

  public Command backCoralIntake() {
    return arm.ground()
        .alongWith(ramp.show())
        .until(arm.canMoveElevator)
        .andThen(elevator.handoff())
        .andThen(arm.handoff().alongWith(placer.intake()));
  }

  public Command frontCoralIntake() {
    return arm.stationPickup()
        .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.stationPickup()))
        .andThen(placer.intake());
  }

  public Command shoot() {
    return Commands.defer(
        () ->
            placer
                .eject(
                    isPlacingAlgae
                        ? placer.algaeEjectSpeed
                        : nextCoral == CoralState.L4 ? placer.l4EjectSpeed : placer.coralEjectSpeed)
                .andThen(Commands.waitSeconds(0.35)),
        Set.of());
  }

  public Command shootL1() {
    return Commands.defer(
        () ->
            placer
                .eject(isPlacingAlgae ? placer.algaeEjectSpeed : placer.l1EjectSpeed)
                .andThen(Commands.waitSeconds(0.35)),
        Set.of());
  }

  public Command stow() {
    return ramp.show()
        .alongWith(climber.off())
        .andThen(
            arm.stow()
                .alongWith(Commands.waitSeconds(0.5).andThen(placer.stop()))
                .alongWith(Commands.waitUntil(arm.canMoveElevator).andThen(elevator.stow()))
                .alongWith(Commands.runOnce(() -> shouldMoveBackAfterShoot = false)));
  }

  public static enum CoralState {
    L1(Color.kYellow),
    L2(Color.kSkyBlue),
    L3(Color.kLimeGreen),
    L4(Color.kRed);

    public final String color;

    private CoralState(Color color) {
      this.color = color.toHexString();
    }
  }

  public static enum AlgaeState {
    LOW,
    HIGH;
  }
}
