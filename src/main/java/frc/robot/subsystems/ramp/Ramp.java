package frc.robot.subsystems.ramp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;

/**
 * The Ramp subsystem is used by the robot to intake game pieces.
 */
public class Ramp {

    /**
     * The motor that tilts the ramp.
     */
    private Motor tiltMotor;
    
    private double intakeAngle = 0;
    private double stowAngle = 10; // TODO: change !
    
    /**
     * Creates a new Ramp subsystem.
     */
    public Ramp() {
        tiltMotor = Motor.fromIdealSim(     // Make an ideal sim
            FeedbackController.fromPID(     // Using PID for our feedback
                new PIDController(5, 0, 0), // Our PID values
                pid -> {                    // Configuring the pid controller
                    pid.setTolerance(1);    // Within one unit to our goal is good enough
                }
            ),
            TargetType.Position,            // This motor goes to a position
            0                               // The starting position of the motor is 0 units
        );
    }

    /**
     * Moves the ramp to the specified target angle.
     */
    public void moveRamp(double target) {
        tiltMotor.setTarget(target);
    }
    
    /**
     * Moves the ramp to the stow angle.
     */
    public Command raiseRamp() {
        return Commands.runOnce(
            () -> moveRamp(stowAngle)
        );
    }

    /**
     * Moves the ramp to the intake angle.
     */
    public Command lowerRamp() {
        return Commands.runOnce(
            () -> moveRamp(intakeAngle)
        );
    }
}
