package frc.robot.subsystems.grabber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Grabber extends SubsystemBase implements Loggable {
  private Motor speedstr;

  public Grabber() {
    speedstr =
        Motor.fromIdealSim(
            FeedbackController.fromPID(
                new PIDController(10, 0, 0),
                pid -> {
                  pid.setTolerance(0.01);
                }),
            TargetType.Velocity,
            0);
  }


public class Grabber extends SubsystemBase{
    
    private Motor speedstr;

    public Grabber() {
      speedstr =
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
     * @return A command that stowps the grabber
     */
    public Command stop() {
      return Commands.runOnce(
              () -> {
                speedstr.setTarget(0.5);
              },
              this)
          .andThen(
              Commands.waitUntil(
                  () -> {
                    return speedstr.atTarget();
                  }));
    }
  
    /**
     * @return A command that makes the intaker suck
     */
    public Command intake() {
      return Commands.runOnce(
              () -> {
                speedstr.setTarget(2);
              },
              this)
          .andThen(
              Commands.waitUntil(
                  () -> {
                    return speedstr.atTarget();
                  }));
    }
  /**
   * @return A command that stowps the grabber
   */
  public Command stop() {
    return Commands.runOnce(
            () -> {
              speedstr.setTarget(0);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return speedstr.atTarget();
                }));
  }

  /**
   * @return A command that makes the intaker suck
   */
  public Command intake() {
    return Commands.runOnce(
            () -> {
              speedstr.setTarget(2);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return speedstr.atTarget();
                }));
  }

  /**
   * @return A command that makes the shooter?? intaker?? spit
   */
  public Command eject() {
    return Commands.runOnce(
            () -> {
              speedstr.setTarget(2);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return speedstr.atTarget();
                }));
  }

  public void log(String path) {
    HoundLog.log(path, "Speed Motor", speedstr);
  }
}
