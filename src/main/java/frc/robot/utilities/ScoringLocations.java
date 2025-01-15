package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ScoringLocations {
  public static final Pose2d A = new Pose2d(3.23, 4.21, Rotation2d.fromDegrees(0));
  public static final Pose2d B = new Pose2d(3.26, 3.87, Rotation2d.fromDegrees(0));
  public static final Pose2d C = new Pose2d(3.70, 3.05, Rotation2d.fromDegrees(60));
  public static final Pose2d D = new Pose2d(4.00, 2.90, Rotation2d.fromDegrees(60));
  public static final Pose2d E = new Pose2d(4.93, 2.84, Rotation2d.fromDegrees(120));
  public static final Pose2d F = new Pose2d(5.25, 3.03, Rotation2d.fromDegrees(120));
  public static final Pose2d G = new Pose2d(5.69, 3.86, Rotation2d.fromDegrees(180));
  public static final Pose2d H = new Pose2d(5.69, 4.17, Rotation2d.fromDegrees(180));
  public static final Pose2d I = new Pose2d(5.25, 5.00, Rotation2d.fromDegrees(240));
  public static final Pose2d J = new Pose2d(4.94, 5.18, Rotation2d.fromDegrees(240));
  public static final Pose2d K = new Pose2d(4.03, 5.16, Rotation2d.fromDegrees(300));
  public static final Pose2d L = new Pose2d(3.74, 4.99, Rotation2d.fromDegrees(300));

  public static Pose2d allianceFlip(Pose2d blue) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
      return blue;
    }
    return flip(blue);
  }

  public static Pose2d flip(Pose2d og) {
    return new Pose2d(
        17.55 - og.getX(),
        8.05 - og.getY(),
        new Rotation2d(og.getRotation().getRadians() - Math.PI));
  }
}
