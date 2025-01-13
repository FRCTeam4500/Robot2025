package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Elevator extends SubsystemBase implements Loggable {
  private Motor upMotor;

  public Elevator() {
    upMotor =
        Motor.fromIdealSim(
            FeedbackController.fromPID(
                new PIDController(10, 0, 0),
                pid -> {
                  pid.setTolerance(0.01);
                }),
            TargetType.Position,
            0);
  }

  /**
   * @return A command that stows the elevator
   */
  public Command stow() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(0.5);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that makes the elevator go up to the fourth level
   */
  public Command level4() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(2);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that makes the elevator go to the third level
   */
  public Command level3() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(1.5);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that moves the elevator up to the second level
   */
  public Command level2() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(1);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that moves the elevator up to the first level
   */
  public Command level1() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(0.5);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

 /**
   * @return A command that moves the elevator down to the ramp, waits .1 second, then
   */
public Command algaeFromRamp() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(0);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }))
                .andThen(
                  Commands.waitSeconds(.1)
                )
                .andThen(
                  Commands.runOnce(()-> {
                    upMotor.setTarget(0.5);
                  }), this)
                  .andThen(
                    Commands.waitUntil(() ->{
                      return upMotor.atTarget();
                    }));

  }
  /**
   * @return A command that moves the elevator to the level of the lower algae
   */
  public Command lowAlgae() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(.75);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that moves the elevator to the level of the higher algae
   */
  public Command highAlgae() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(1.75);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Up Motor", upMotor);
  }
}
