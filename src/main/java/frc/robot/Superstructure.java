package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.groundintake.GroundIntake;
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
  private GroundIntake intake;
  private Elevator elevator;
  private Ramp ramp;
  private Arm arm;
  private Placer placer;

  public Superstructure() {
    robotMech = new Mechanism2d(3, 3);
    climber = new Climber();
    intake = new GroundIntake();
    elevator = new Elevator();
    ramp = new Ramp();
    arm = new Arm();
    placer = new Placer();
  }

  public void log(String path) {
    // Call log() methods for contained subsystems
    HoundLog.log(path, "Climber", climber);
    HoundLog.log(path, "Ground Intake", intake);
    HoundLog.log(path, "Elevator", elevator);
  }

  public Command readyLevel1() {
    return elevator.level1();
  }

  public Command readyLevel2() {
    return elevator.level2();
  }

  public Command readyLevel3() {
    return elevator.level3();
  }

  public Command readyLevel4() {
    return elevator.level4();
  }

  public Command readyClimb() {
    return climber.up();
  }

  public Command climb() {
    return climber.down();
  }

  public Command intake() {
    return Commands.none();
  }

  public Command readyGroundIntake() {
    return intake.intake();
  }

  public Command endGroundIntake() {
    return intake.stowIntake();
  }

  public Command stow() {
    return elevator.stow().alongWith(intake.stowIntake()).alongWith(climber.down());
  }
}
