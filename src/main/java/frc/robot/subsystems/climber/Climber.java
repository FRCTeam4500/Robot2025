package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Climber extends SubsystemBase implements Loggable {
  private Motor extendMotor;
  private Motor tiltyMotor;

  public Climber() {
    extendMotor =
        Motor.fromIdealSim(
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                pid -> {
                  pid.setTolerance(0.5);
                }),
            TargetType.Position,
            0);

    tiltyMotor =
        Motor.fromIdealSim(
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                pid -> {
                  pid.setTolerance(0.5);
                }),
            TargetType.Position,
            0);
  }

  public Command up() {
    return Commands.runOnce(
            () -> {
              extendMotor.setTarget(0);
              tiltyMotor.setTarget(0);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return extendMotor.atTarget() && tiltyMotor.atTarget();
                }));
  }

  public Command down() {
    return Commands.runOnce(
            () -> {
              extendMotor.setTarget(0);
              tiltyMotor.setTarget(0);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return extendMotor.atTarget() && tiltyMotor.atTarget();
                }));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "extend Motor", extendMotor);
    HoundLog.log(path, "tilty Motor", tiltyMotor);
  }
}
