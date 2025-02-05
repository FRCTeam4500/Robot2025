package frc.robot.robots;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.WiringConstants.RampWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;

public class RampTest extends LoggedRobot {
    private Motor motor;
    private double voltage;

    public RampTest() { 
        motor =
        Motor.fromSparkMax(
            RampWiring.RAMP_ID,
            false,
            (SparkMax max) -> {
                SparkMaxConfig config =new SparkMaxConfig();
                config.idleMode(IdleMode.kBrake);
                config.encoder.positionConversionFactor((1.0/((60/12)*(60/18)))*360);
                config.encoder.velocityConversionFactor((1.0/((60/12)*(60/18)))*360/60);
                config.smartCurrentLimit(60);
                max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim jim) -> {},
            -105,
            FeedbackController.fromPID(
                new PIDController(0.03, 0, 0), (PIDController pid)->{
                    pid.setTolerance(5);
                }),
                FeedforwardController.forArmGravity(0.31, 0.07, 0, 0),
                TargetType.Position);
                SmartDashboard.putData("Target: -45", Commands.runOnce(() -> motor.setTarget(-45)));
                SmartDashboard.putData("Target: 0", Commands.runOnce(() -> motor.setTarget(0)));
                SmartDashboard.putData("Target: 45", Commands.runOnce(() -> motor.setTarget(45)));
    }

    @Override
    public void robotPeriodic() {
        HoundLog.log("Motor", motor);
        CommandScheduler.getInstance().run();
       // motor.setVoltage(voltage);
    }

}
