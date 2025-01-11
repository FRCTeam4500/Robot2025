package frc.robot.subsystems.groundintake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class GroundIntake extends SubsystemBase implements Loggable {

    private Motor tiltMotor;
    
    private Motor runMotor;

    private double stowPosition = 0;
    private double intakePosition = 10; // TODO: change

    private double intakeSpeed = 3;
    
    /**
     * Creates a new ground intake.
     */
    public GroundIntake() {
        tiltMotor = 
          Motor.fromIdealSim(
              FeedbackController.fromPID( // Using PID for our feedback
                  new PIDController(5, 0, 0), // Our PID values
                  pid -> { // Configuring the pid controller
                    pid.setTolerance(1); // Within one unit to our goal is good enough
                  }),
              TargetType.Position, // This motor goes to a position
              0 // The starting position of the motor is 0 units
            );
        runMotor =
          Motor.fromIdealSim(
              FeedbackController.fromPID( // Using PID for our feedback
                  new PIDController(5, 0, 0), // Our PID values
                  pid -> { // Configuring the pid controller
                    pid.setTolerance(1); // Within one unit to our goal is good enough
                  }),
              TargetType.Velocity, // This motor goes to a position
              0 // The starting position of the motor is 0 units
            );
    }
    
    /**
     * Moves the intake to the target angle.
     * @param targetAngle angle in rad
     */
    private void moveIntake(double targetAngle) {
      tiltMotor.setTarget(targetAngle);
    }

    /**
     * Sets velocity of runMotor to target velocity
     * @param targetVelocity velocity in rad/s
     */
    private void runIntake(double targetVelocity) {
        runMotor.setVoltage(targetVelocity);
    }
    
    /**
     * @return Command to move intake to intake position and begin running wheels
     */
    public Command readyIntake() {
      return runOnce(() -> moveIntake(intakePosition)).andThen(runOnce(() -> runIntake(intakeSpeed)));
    }

    /**
     * 
     * @return Command to move intake to stow position and stop wheels
     */
    public Command stowIntake() {
      return runOnce(
        () -> moveIntake(stowPosition)).andThen(runOnce(() -> runIntake(0)));
    }

    @Override
    public void log(String path) {
      HoundLog.log(path, "Position", tiltMotor.getPosition());
      HoundLog.log(path, "Speed", runMotor.getVelocity());
    }
}