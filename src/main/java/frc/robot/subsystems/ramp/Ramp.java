package frc.robot.subsystems.ramp;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

/** The Ramp subsystem is used by the robot to intake game pieces. */
public class Ramp extends SubsystemBase implements Loggable {
  private Motor tiltMotor;
  public final MechanismLigament2d mech;

  private double intakeAngle = 155;
  private double stowAngle = 90;

  /** Creates a new Ramp subsystem. */
  public Ramp() {
    tiltMotor =
        Motor.fromIdealSim( // Make an ideal sim
            FeedbackController.fromProfiledPID(
                new ProfiledPIDController(0, 0, 0, new Constraints(90, 180)),
                (ProfiledPIDController pid) -> {
                  pid.setTolerance(1);
                }),
            TargetType.Position, // This motor goes to a position
            intakeAngle // The starting position of the motor is 0 units
            );
    mech = new MechanismLigament2d("Ramp", 0.4, intakeAngle);
  }

  /**
   * Moves the ramp to the specified target angle.
   *
   * @param targetAngle the target angle in rad.
   */
  public void moveRamp(double targetAngle) {
    tiltMotor.setTarget(targetAngle);
  }

  /**
   * Moves the ramp to the stow angle.
   *
   * @return Command to move ramp to the stow angle.
   */
  public Command hide() {
    return Commands.runOnce(() -> moveRamp(stowAngle));
  }

  /**
   * Moves the ramp to the intake angle.
   *
   * @param Command to move ramp to the intake angle.
   */
  public Command show() {
    return Commands.runOnce(() -> moveRamp(intakeAngle));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Motor", tiltMotor);
    mech.setAngle(tiltMotor.getPosition());
  }
}
