package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.placer.Placer;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

/**
 * A class that holds together the top half of our robot. Basically everything except the
 * drivetrain. It exposes command factories which combine the various subsystems to vimal tasks
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
    return elevator.level1().alongWith(arm.placeL1());
  }

  public Command readyLevel2() {
    return elevator.level2().alongWith(arm.placeL2());
  }

  public Command readyLevel3() {
    return elevator.level3().alongWith(arm.placeL3());
  }

  public Command readyLevel4() {
    return elevator.level4().alongWith(arm.placeL4());
  }

  public Command readyClimb() {
    return stow().andThen(ramp.raiseRamp()).andThen(climber.ready());
  }

  public Command climb() {
    return climber.climb();
  }

  public Command intake() {
    return arm.stow().andThen(elevator.handoff()).andThen(arm.handoff()).andThen(placer.intake());
  }

  public Command stow() {
    return arm.stow()
        .alongWith(placer.stop())
        .andThen(elevator.stow())
        .alongWith(climber.stow().andThen(ramp.lowerRamp()));
  }
}
