package frc.robot.robots;

import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.utilities.gamepieces.GamepieceManager;
import frc.robot.utilities.logging.HoundLog;

public class LoggedRobot extends TimedRobot {
    public LoggedRobot() {
        HoundLog.setEnabled(true);
        HoundLog.setPdh(new PowerDistribution());
        HoundLog.setOptions(
            new DogLogOptions(() -> !DriverStation.isFMSAttached(), true, true, true, true, 1000));
        GamepieceManager.resetField();
    }
}
