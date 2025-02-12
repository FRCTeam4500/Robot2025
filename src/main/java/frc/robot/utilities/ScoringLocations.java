package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ScoringLocations {
  public static final Pose2d A = new Pose2d(2.99, 4.19, Rotation2d.fromDegrees(0));
  public static final Pose2d B = new Pose2d(2.99, 3.86, Rotation2d.fromDegrees(0));
  public static final Pose2d C = new Pose2d(3.60, 2.79, Rotation2d.fromDegrees(60));
  public static final Pose2d D = new Pose2d(3.89, 2.63, Rotation2d.fromDegrees(60));
  public static final Pose2d E = new Pose2d(5.09, 2.65, Rotation2d.fromDegrees(120));
  public static final Pose2d F = new Pose2d(5.37, 2.83, Rotation2d.fromDegrees(120));
  public static final Pose2d G = new Pose2d(5.97, 3.86, Rotation2d.fromDegrees(180));
  public static final Pose2d H = new Pose2d(5.97, 4.19, Rotation2d.fromDegrees(180));
  public static final Pose2d I = new Pose2d(5.37, 5.22, Rotation2d.fromDegrees(240));
  public static final Pose2d J = new Pose2d(5.09, 5.38, Rotation2d.fromDegrees(240));
  public static final Pose2d K = new Pose2d(3.89, 5.38, Rotation2d.fromDegrees(300));
  public static final Pose2d L = new Pose2d(3.60, 5.22, Rotation2d.fromDegrees(300));

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
