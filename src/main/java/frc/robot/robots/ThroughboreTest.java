package frc.robot.robots;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ThroughboreTest extends LoggedRobot {
  private DutyCycleEncoder encoder;

  public ThroughboreTest() {
    encoder = new DutyCycleEncoder(7);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", encoder.get());
  }
}
