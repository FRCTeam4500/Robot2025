package frc.robot.robots;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.WiringConstants.PlacerWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.FeedforwardConstants;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;

public class PlacerTest extends LoggedRobot{
    private Motor motor;

    public PlacerTest() {
        motor = Motor.fromTalonFX(
            PlacerWiring.PLACER_ID,
            (TalonFX fx) -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.Audio.AllowMusicDurDisable = true;
                config.CurrentLimits.StatorCurrentLimit = 40;
                config.CurrentLimits.StatorCurrentLimitEnable = true;
                config.CurrentLimits.SupplyCurrentLimitEnable = false;
                config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                fx.getConfigurator().apply(config);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                (PIDController pid) -> {
                    pid.setTolerance(0.5);
                }
            ),
            Optional.of(new FeedforwardConstants(0, 0.47622, 0.12973, 0.01321)),
            TargetType.Velocity
        );

        SmartDashboard.putData("Speed: 50", Commands.runOnce(() -> motor.setTarget(50)));
        SmartDashboard.putData("Speed: 25", Commands.runOnce(() -> motor.setTarget(25)));
        SmartDashboard.putData("Speed: 0", Commands.runOnce(() -> motor.setTarget(0)));
        SmartDashboard.putData("Speed: -25", Commands.runOnce(() -> motor.setTarget(-25)));
        SmartDashboard.putData("Speed: -50", Commands.runOnce(() -> motor.setTarget(-50)));
    }

    @Override
    public void robotPeriodic() {
        HoundLog.log("Motor", motor);
        CommandScheduler.getInstance().run();
    }
    
}
