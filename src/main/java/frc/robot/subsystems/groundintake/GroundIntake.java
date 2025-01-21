package frc.robot.subsystems.groundintake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

/**
 * The GroundIntake subsystem is on the same side of the robot as the placer and feeds the placer
 * from the ground.
 */
public class GroundIntake extends SubsystemBase implements Loggable {

  private Motor tiltMotor;

  private Motor runMotor;

  public final MechanismLigament2d mech;

  private double stowPosition = 90;
  private double intakePosition = 0; // TODO: change

  private double intakeSpeed = 3000;
  private double outtakeSpeed = -3000;

  /** Creates a new ground intake. */
  public GroundIntake() {
    tiltMotor =
        Motor.fromIdealSim(
            FeedbackController.fromPID( // Using PID for our feedback
                new PIDController(5, 0, 0), // Our PID values
                pid -> { // Configuring the pid controller
                  pid.setTolerance(1); // Within one unit to our goal is good enough
                }),
            TargetType.Position, // This motor goes to a position
            90 // The starting position of the motor is 90 units
            );
    runMotor =
        Motor.fromIdealSim(
            FeedbackController.fromPID( // Using PID for our feedback
                new PIDController(5, 0, 0), // Our PID values
                pid -> { // Configuring the pid controller
                  pid.setTolerance(1); // Within one unit to our goal is good enough
                }),
            TargetType.Velocity, // This motor goes to a velocity
            0 // The starting position of the motor is 0 units
            );
      mech = new MechanismLigament2d("Intake Tilt", .5, 90);
  }

  /**
   * Moves the intake to the target angle.
   *
   * @param targetAngle angle in rad
   */
  private void moveIntake(double targetAngle) {
    tiltMotor.setTarget(targetAngle);
  }

  /**
   * Sets velocity of runMotor to target velocity
   *
   * @param targetVelocity velocity in rad/s
   */
  private void runIntake(double targetVelocity) {
    runMotor.setTarget(targetVelocity);
  }

  /**
   * @return Command to move intake to intake position
   */
  public Command readyIntake() {
    return runOnce(() -> moveIntake(intakePosition));
  }

  /**
   * @return Command to begin running the wheels to intake
   */
  public Command intake() {
    return runOnce(() -> runIntake(intakeSpeed));
  }

  /**
   * @return Command to begin running the wheels to eject
   */
  public Command outtake() {
    return runOnce(() -> runIntake(outtakeSpeed));
  }

  /**
   * @return Command to move intake to stow position and stop wheels
   */
  public Command stowIntake() {
    return runOnce(() -> moveIntake(stowPosition)).andThen(runOnce(() -> runIntake(0)));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Tilt Motor", tiltMotor);
    HoundLog.log(path, "Run Motor", runMotor);
  }
  
  @Override
  public void periodic() {
    mech.setAngle(tiltMotor.getPosition());
  }
}
