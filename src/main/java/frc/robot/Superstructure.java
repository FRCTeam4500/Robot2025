package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.groundintake.GroundIntake;
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
  private GroundIntake intake;
  private Elevator elevator;
  private Ramp ramp;

  public Superstructure() {
    robotMech = new Mechanism2d(3, 3);
    climber = new Climber();
    intake = new GroundIntake();
    elevator = new Elevator();
    ramp = new Ramp();
  }

  public void log(String path) {
    // Call log() methods for contained subsystems
    HoundLog.log(path, "Climber", climber);
    HoundLog.log(path, "Ground Intake", intake);
    HoundLog.log(path, "Elevator", elevator);
  }

  // Put Command Factories Here

  // ready level 1
  public Command readyLevel1() {
    return elevator.level1();
  }

  // ready level 2
  public Command readyLevel2() {
    return elevator.level2();
  }

  // ready level 3
  public Command readyLevel3() {
    return elevator.level3();
  }

  // ready level 4
  public Command readyLevel4() {
    return elevator.level4();
  }

  // ready climb
  public Command readyClimb() {
    return climber.up();
  }

  // climb
  public Command climb() {
    return climber.down();
  }

  public Command intake() {
    return Commands.none();
  }

  // start ground intake
  public Command readyGroundIntake() {
    return intake.intake();
  }

  // end ground intake
  public Command endGroundIntake() {
    return intake.stowIntake();
  }

  // stow
  public Command stow() {
    return elevator.stow().alongWith(intake.stowIntake()).alongWith(climber.down());
  }
}
