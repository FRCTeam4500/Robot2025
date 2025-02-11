package frc.robot.programs.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.WiringConstants.ElevatorWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.programs.LoggedRobot;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;

public class ElevatorTest extends LoggedRobot {
  private Motor motor;
  private double voltage;

  public ElevatorTest() {
    motor =
        Motor.fromSparkMax(
            ElevatorWiring.ELEVATOR_ID,
            false,
            (SparkMax spark) -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.idleMode(IdleMode.kCoast);
              config.encoder.positionConversionFactor(1 / 63.1167979003);
              config.encoder.velocityConversionFactor(1 / 63.1167979003);
              config.smartCurrentLimit(60);
              spark.configure(
                  config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim sim) -> {
              sim.withHardstops(0, 1);
            },
            0,
            FeedbackController.fromPID(
                new PIDController(75, 0, 0),
                (PIDController pid) -> {
                  pid.setTolerance(0.05);
                }),
            FeedforwardController.forConstantGravity(0.775, 0.21877, 8.0517, 2.143),
            TargetType.Position);
    SmartDashboard.putData("Target: 0", Commands.runOnce(() -> motor.setTarget(0)));
    SmartDashboard.putData("Target: 0.3", Commands.runOnce(() -> motor.setTarget(0.3)));
    SmartDashboard.putData("Target: 0.6", Commands.runOnce(() -> motor.setTarget(0.6)));

    SysIDCommands commands = motor.getSysIDCommands("elevator", 0.5, 2, 5);
    SmartDashboard.putData("elevatorQuasistaticForward", commands.quasistaticForward());
    SmartDashboard.putData("elevatorQuasistaticReverse", commands.quasistaticReverse());
    SmartDashboard.putData("elevatorDyanamicForward", commands.dynamicForward());
    SmartDashboard.putData("elevatorDynamicReverse", commands.dynamicReverse());
    Sendable sendable = new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Voltage", () -> {return voltage;}, (double nextVoltage) -> {
          voltage = nextVoltage;
        });
      }
      
    };
    SmartDashboard.putData("Voltage Setter", sendable);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    HoundLog.log("Elevator Motor", motor);
    // motor.setVoltage(voltage);
  }
}
