package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

/**
 * A class that holds together the top half of our robot. Basically everything except the
 * drivetrain. It exposes command factories which combine the various subsystems to preform tasks
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
    configureMech();
  }

  private void configureMech() {
    robotMech.getRoot("Climber Root", 1, .1).append(climber.mech);
    robotMech.getRoot("Intake Root", 2, .1).append(intake.mech);
    robotMech.getRoot("Elevator Root", 1.5, .1).append(elevator.mech);
    SmartDashboard.putData("Robot Mech", robotMech);
    SmartDashboard.putData("Start Climb", climber.up());
    SmartDashboard.putData("End Climb", climber.down());
    SmartDashboard.putData("Start Intake", intake.readyIntake());
    SmartDashboard.putData("End Intake", intake.stowIntake());
    SmartDashboard.putData("Stow Elevator", elevator.stow());
    SmartDashboard.putData("L4 Elevator", elevator.level4());
  }

  public void log(String name) {
    // Call log() methods for contained subsystems
    HoundLog.log(name, "Climber", climber);
    HoundLog.log(name, "Ground Intake", intake);
    HoundLog.log(name, "Elevator", elevator);
  }

  // Put Command Factories Here
}
