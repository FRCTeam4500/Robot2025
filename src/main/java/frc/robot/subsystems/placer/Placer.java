package frc.robot.subsystems.placer;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.WiringConstants.PlacerWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Placer extends SubsystemBase implements Loggable {
  private Motor runMotor;

  private final double intakeSpeed = 2;
  private final double ejectSpeed = 2;

  public Placer() {
    runMotor = Motor.fromTalonFX(
      PlacerWiring.PLACER_ID,
      (TalonFX fx) -> {},
      (FeedforwardSim sim) -> {},
      0,
      FeedbackController.fromPID(
        new PIDController(1, 0, 0),
        (PIDController pid) -> {
          pid.setTolerance(1);
        }
      ),
      Optional.empty(),
      TargetType.Velocity
    );
  }

  /**
   * @return A command that stowps the grabber
   */
  public Command stop() {
    return Commands.runOnce(
            () -> {
              runMotor.setTarget(0);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return runMotor.atTarget();
                }));
  }

  /**
   * @return A command that makes the intaker suck
   */
  public Command intake() {
    return Commands.runOnce(
            () -> {
              runMotor.setTarget(intakeSpeed);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return runMotor.atTarget();
                }));
  }

  /**
   * @return A command that makes the shooter?? intaker?? spit
   */
  public Command eject() {
    return Commands.runOnce(
            () -> {
              runMotor.setTarget(ejectSpeed);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return runMotor.atTarget();
                }));
  }

  public void log(String path) {
    HoundLog.log(path, "Speed Motor", runMotor);
  }
}
