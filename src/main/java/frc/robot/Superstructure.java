package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.placer.Placer;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

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

  public Superstructure() {
    robotMech = new Mechanism2d(3, 3);
    climber = new Climber();
    elevator = new Elevator();
    ramp = new Ramp();
    arm = new Arm();
    placer = new Placer();

    configureMech();
  }

  private void configureMech() {
    robotMech.getRoot("Climber Root", 1, 0.1).append(climber.mech);
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
  }

  public Command readyLevel1() {
    return arm.placeL1()
        .alongWith(Commands.waitUntil(arm.canMoveElevator()).andThen(elevator.level1()));
  }

  public Command readyLevel2() {
    return arm.placeL2()
        .alongWith(Commands.waitUntil(arm.canMoveElevator()).andThen(elevator.level2()));
  }

  public Command readyLevel3() {
    return arm.placeL3()
        .alongWith(Commands.waitUntil(arm.canMoveElevator()).andThen(elevator.level3()));
  }

  public Command readyLevel4() {
    return arm.placeL4()
        .alongWith(Commands.waitUntil(arm.canMoveElevator()).andThen(elevator.level4()));
  }

  public Command readyClimb() {
    return stow().andThen(ramp.hide()).andThen(climber.ready());
  }

  public Command climb() {
    return climber.climb();
  }

  public Command intake() {
    return arm.stow()
        .withDeadline(Commands.waitUntil(arm.canMoveElevator()).andThen(elevator.handoff()))
        .andThen(arm.handoff())
        .andThen(placer.intake());
  }

  public Command stow() {
    return placer
        .stop()
        .andThen(arm.stow())
        .alongWith(Commands.waitUntil(arm.canMoveElevator()).andThen(elevator.stow()))
        .andThen(climber.stow())
        .andThen(ramp.show());
  }
}
