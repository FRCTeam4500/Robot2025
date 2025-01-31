package frc.robot.robots;

import java.util.Optional;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.WiringConstants.ElevatorWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.FeedforwardConstants;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;

public class ElevatorTest extends LoggedRobot {
    private Motor motor;

    public ElevatorTest() {
        motor = Motor.fromSparkMax(
            ElevatorWiring.ELEVATOR_ID, 
            false, 
            (SparkMax spark) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.idleMode(IdleMode.kCoast);
                config.encoder.positionConversionFactor(1 / 216.5);
                config.encoder.velocityConversionFactor(1 / 216.5 / 60);
                config.smartCurrentLimit(60);
                spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            (FeedforwardSim sim) -> {
                sim.withHardstops(0, 1);
            }, 
            0, 
            FeedbackController.fromPID(
                new PIDController(15, 0, 0), 
                (PIDController pid) -> {
                    pid.setTolerance(0.05);
                }
            ), 
            Optional.of(new FeedforwardConstants(0.15472, 0.098108, 27.673, 3.4548)), 
            TargetType.Meters
        );
        SmartDashboard.putData("Target: 0", Commands.runOnce(() -> motor.setTarget(0)));
        SmartDashboard.putData("Target: 0.3", Commands.runOnce(() -> motor.setTarget(0.3)));
        SmartDashboard.putData("Target: 0.6", Commands.runOnce(() -> motor.setTarget(0.6)));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        HoundLog.log("Elevator Motor", motor);
    }
}
