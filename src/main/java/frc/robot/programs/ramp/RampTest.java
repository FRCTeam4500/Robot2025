package frc.robot.programs.ramp;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.WiringConstants.RampWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.programs.LoggedRobot;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;

import frc.robot.utilities.SysIDCommands;

public class RampTest extends LoggedRobot {
  private Motor motor;

  public RampTest() {
    motor =
        Motor.fromSparkMax(
            RampWiring.RAMP_ID,
            false,
            (SparkMax max) -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.idleMode(IdleMode.kBrake);
              config.encoder.positionConversionFactor((1.0 / ((60 / 12) * (60 / 18))) * 360);
              config.encoder.velocityConversionFactor((1.0 / ((60 / 12) * (60 / 18))) * 360);
              config.smartCurrentLimit(60);
              max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim jim) -> {
              jim.withHardstops(-90, 90);
            },
            -90,
            FeedbackController.fromPID(
                new PIDController(0.03, 0, 0),
                (PIDController pid) -> {
                  pid.setTolerance(5);
                }),
            FeedforwardController.forArmGravity(0.31, 0.07, 0, 0),
            TargetType.Position);

    SysIDCommands  commands = motor.getSysIDCommands("Ramp", 0.2, 0.5, 4);
    SmartDashboard.putData("Ramp Dynamic Forward", commands.dynamicForward());
    SmartDashboard.putData("Ramp Dynamic Reverse", commands.dynamicReverse());
    SmartDashboard.putData("Ramp Quasistatic Forward", commands.quasistaticForward());
    SmartDashboard.putData("Ramp Quasistatic Reverse", commands.quasistaticReverse());
  //   SmartDashboard.putData("Target: -45", Commands.runOnce(() -> motor.setTarget(-45)));
  //   SmartDashboard.putData("Target: 0", Commands.runOnce(() -> motor.setTarget(0)));
  //   SmartDashboard.putData("Target: 45", Commands.runOnce(() -> motor.setTarget(45)));
  }

  @Override
  public void robotPeriodic() {
    HoundLog.log("Motor", motor);
    CommandScheduler.getInstance().run();
  }
}
