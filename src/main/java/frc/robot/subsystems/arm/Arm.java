package frc.robot.subsystems.arm;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Arm extends SubsystemBase implements Loggable {
    private Motor tiltMotor;
    
    private final double startAngle = 110;
    private final double stowAngle = 90;
    private final double placeL4Angle = 45;
    private final double placeL3Angle = 75;
    private final double placeL2Angle = 75;
    private final double placeL1Angle = 0;
    private final double handoffAngle = -100;
    private final double stationAngle = 75;

    public Arm() {
        tiltMotor = Motor.fromTalonFX(
            0, 
            (TalonFX fx) -> {

            }, 
            (FeedforwardSim sim) -> {

            }, 
            startAngle, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0), 
                (PIDController pid) -> {
                    pid.setTolerance(1);
                }
            ), 
            Optional.empty(),
            TargetType.Rotation
        );
    }

    public Command stow() {
      return Commands.runOnce(
        () -> {
          tiltMotor.setTarget(stowAngle);
        }, this
      ).andThen(
        Commands.waitUntil(() -> {
            return tiltMotor.atTarget();
        })
      );
    }

    public Command handoff() {
      return Commands.runOnce(
        () -> {
          tiltMotor.setTarget(handoffAngle);
        }, this
      ).andThen(
        Commands.waitUntil(() -> {
            return tiltMotor.atTarget();
        })
      );
    }

    public Command placeL4() {
      return Commands.runOnce(
        () -> {
          tiltMotor.setTarget(placeL4Angle);
        }, this
      ).andThen(
        Commands.waitUntil(() -> {
            return tiltMotor.atTarget();
        })
      );
    }

    public Command placeL3() {
        return Commands.runOnce(
            () -> {
                tiltMotor.setTarget(placeL3Angle);
            }, this
        ).andThen(
            Commands.waitUntil(() -> {
                return tiltMotor.atTarget();
            })
        );
    }
    
    public Command placeL2() {
        return Commands.runOnce(
            () -> {
                tiltMotor.setTarget(placeL2Angle);
            }, this
        ).andThen(
            Commands.waitUntil(() -> {
                return tiltMotor.atTarget();
            })
        );
    }

    public Command placeL1() {
      return Commands.runOnce(
        () -> {
          tiltMotor.setTarget(placeL1Angle);
        }, this
      ).andThen(
        Commands.waitUntil(
          () -> {
            return tiltMotor.atTarget();
          }
        )
      );
    }
    
    public Command stationPickup() {
      return Commands.runOnce(
        () -> {
          tiltMotor.setTarget(stationAngle);
        }, this
      ).andThen(
        Commands.waitUntil(
          () -> {
            return tiltMotor.atTarget();
          }
        )
      );
    }

    @Override
    public void log(String path) {
      HoundLog.log("Tilt Motor", tiltMotor);
    }
}
