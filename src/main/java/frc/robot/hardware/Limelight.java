package frc.robot.hardware;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.gamepieces.GamepieceManager;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Wrapper class for limelights. An object of this class doesn't own the underlying hardware. In
 * other words, multiple objects can be made for one limelight, and they will all work. That is not
 * recommended though, since each object could change the pipeline.
 *
 * <pre>
 * //Example Usage
 * // Create a limelight, using the limelight named hehehe
 * // Set the pipeline to 0
 * Limelight limelight = new Limelight("limelight-hehehe", 0);
 *
 * double verticalOffset = limelight.getTY();
 * PoseEstimate estimate = limelight.getPoseMT1();
 * if (estimate.exists()) { // Only add the measurement if we have an estimate
 *     double timestamp = Timer.getFPGATimestamp() - estimate.latencySeconds();
 *     estimator.addVisionMeasurement(estimate.pose(), timestamp);
 * }
 * </pre>
 */
public class Limelight implements Loggable {
  private NetworkTable table;
  private String name;
  private PhotonCamera camera;
  private VisionSystemSim sim;
  private boolean enabled = true;

  private final PoseEstimate kEmpty = new PoseEstimate(
    new Pose2d(), 
    0, 
    0, 
    0, 
    0, 
    false
  );

  /**
   * Make a limelight with the given name and pipeline
   *
   * @param name The name of the limelight. Should be "limelight-xxx"
   * @param pipeline The pipeline to be used. These are configured in a web browser.
   */
  public Limelight(String name, int pipeline, Transform3d robotToCamera) {
    this.name = name;
    table = NetworkTableInstance.getDefault().getTable(this.name);
    table.getEntry("pipline").setInteger(pipeline);
    Sendable isEnabledSendable =
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.addBooleanProperty(
                name,
                () -> enabled,
                (boolean val) -> {
                  enabled = val;
                });
          }
        };
    SmartDashboard.putData("[Limelight] " + this.name + " Enabled", isEnabledSendable);
    if (RobotBase.isSimulation()) {
      sim = new VisionSystemSim(name);
      sim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
      SimCameraProperties camProperties = new SimCameraProperties();
      camProperties.setCalibration(1280, 960, Rotation2d.fromDegrees(79.3565372228));
      camProperties.setCalibError(0.25, 0.08);
      camProperties.setFPS(15);
      camProperties.setAvgLatencyMs(70.593);
      camProperties.setLatencyStdDevMs(4.318);
      camera = new PhotonCamera(name);
      PhotonCameraSim cameraSim = new PhotonCameraSim(camera, camProperties);
      sim.addCamera(cameraSim, robotToCamera);
    }
  }

  /**
   * Make a limelight with the given name and pipeline 0
   *
   * @param name The name of the limelight. Should be "limelight-xxx"
   */
  public Limelight(String name, Transform3d robotToCamera) {
    this(name, 0, robotToCamera);
  }

  /**
   * Make a gamepiece tracking limelight. This camera is automatically added to the {@link
   * GamePieceManager}
   *
   * @param name The name of the limelight. Should be "limelight-xxx"
   * @param pipeline The pipeline to be used. These are configured in a web browser.
   * @param pose The pose of this limelight relative to the bottom center of the robot. This pose is
   *     used to update the reading of gamepieces in sim
   */
  public Limelight(String name, int pipeline, Pose3d pose) {
    table = NetworkTableInstance.getDefault().getTable(name);
    table.getEntry("pipline").setInteger(pipeline);
    GamepieceManager.addCamera(name, pose);
  }

  /**
   * Make a gamepiece tracking limelight with the pipeline 0. This camera is automatically added to
   * the {@link GamePieceManager}
   *
   * @param name The name of the limelight. Should be "limelight-xxx"
   * @param pose The pose of this limelight relative to the bottom center of the robot. This pose is
   *     used to update the reading of gamepieces in sim
   */
  public Limelight(String name, Pose3d pose) {
    this(name, 0, pose);
  }

  /**
   * @return whether the limelight sees a target
   */
  public boolean hasTargets() {
    return table.getEntry("tv").getInteger(0) == 1;
  }

  /**
   * @return the horizontal offset from the target in counterclockwise positive degrees. Defaults to
   *     0 if no target is seen
   */
  public double getTX() {
    return -table.getEntry("tx").getDouble(0);
  }

  /**
   * @return the vertical offset from the target in degrees. Defaults to 0 if no target is seen
   */
  public double getTY() {
    return table.getEntry("ty").getDouble(0);
  }

  /**
   * @return the area taken up by the target in %. Defaults to 0 if no target is seen
   */
  public double getTA() {
    return table.getEntry("ta").getDouble(0);
  }

  /**
   * @return the total latency of all data from the limelight to the code
   */
  public double getLatency() {
    return table.getEntry("cl").getDouble(0) + table.getEntry("tl").getDouble(0);
  }

  public int getID() {
    return (int) table.getEntry("tid").getInteger(-1);
  }

  /**
   * @return a {@link PoseEstimate} holding information about an estimated pose obtained using the
   *     megatag 1 algorithm.
   */
  public PoseEstimate getPoseMT1() {
    if (RobotBase.isSimulation()) {
      return kEmpty;
    }
    double[] raw = table.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);
    return new PoseEstimate(
        new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5])),
        raw[6] / 1000,
        (int) raw[7],
        raw[9],
        raw[10],
        hasTargets());
  }

  /**
   * @param currentRotation the current field relative rotation of the robot
   * @return a {@link PoseEstimate} holding information about an estimated pose obtained using the
   *     megatag 2 algorithm.
   */
  public PoseEstimate getPoseMT2(Rotation2d currentRotation, Rotation2d speedPerSec) {
    table
        .getEntry("robot_orientation_set")
        .setDoubleArray(
            new double[] {currentRotation.getDegrees(), speedPerSec.getDegrees(), 0, 0, 0, 0});
    NetworkTableInstance.getDefault().flush();
    return getPoseMT2();
  }

  private PoseEstimate getPoseMT2() {
    if (RobotBase.isSimulation()) {
      return kEmpty;
    }
    double[] raw = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[11]);
    return new PoseEstimate(
        new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5])),
        raw[6] / 1000,
        (int) raw[7],
        raw[9],
        raw[10],
        hasTargets());
  }

  public Pair<Transform2d, Integer> getTargetPoseCameraSpace() {
    if (RobotBase.isReal()) {
      double[] raw = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[11]);
      return new Pair<>(
          new Transform2d(raw[2], raw[0], Rotation2d.fromDegrees(raw[5])),
          hasTargets() ? (int) table.getEntry("tid").getInteger(-1) : -1);
    } else {
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();
      if (results.size() == 0) {
        return new Pair<>(null, -1);
      }
      PhotonPipelineResult latestResult = results.get(results.size() - 1);
      PhotonTrackedTarget target = latestResult.getBestTarget();
      if (target == null) {
        return new Pair<>(null, -1);
      }
      Transform3d transform = target.getBestCameraToTarget();
      return new Pair<Transform2d, Integer>(
          new Transform2d(
              transform.getTranslation().toTranslation2d(),
              transform.getRotation().toRotation2d().plus(Rotation2d.kPi)),
          target.getFiducialId());
    }
  }

  public void updateSim(Pose2d robotPose) {
    if (RobotBase.isReal()) {
      return;
    }
    sim.update(robotPose);
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.size() == 0) {
      return;
    }
    PhotonPipelineResult result = results.get(results.size() - 1);
    if (!result.hasTargets()) {
      table.getEntry("tv").setInteger(0);
      table.getEntry("tx").setDouble(0);
      table.getEntry("ty").setDouble(0);
      table.getEntry("ta").setDouble(0);
      table.getEntry("cl").setDouble(0);
      table.getEntry("tl").setDouble(0);
      table.getEntry("tid").setDouble(0);
      return;
    }
    PhotonTrackedTarget target = result.getBestTarget();
    table.getEntry("tv").setInteger(1);
    table.getEntry("tx").setDouble(-target.getYaw());
    table.getEntry("ty").setDouble(target.getPitch());
    table.getEntry("ta").setDouble(target.getArea());
    table.getEntry("cl").setDouble(0);
    table.getEntry("tl").setDouble(camera.getCameraTable().getEntry("latencyMillis").getDouble(0));
    table.getEntry("tid").setDouble(target.getFiducialId());
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "MT1 Pose", getPoseMT1().pose());
    HoundLog.log(path, "MT2 Pose", getPoseMT2().pose());
    HoundLog.log(path, "Estimate Seconds", getPoseMT1().latencySeconds);
    HoundLog.log(path, "tx", getTX());
    HoundLog.log(path, "ty", getTY());
    HoundLog.log(path, "ta", getTA());
    HoundLog.log(path, "id", getID());
  }

  public String getName() {
    return name;
  }

  public boolean isEnabled() {
    return enabled;
  }

  /** Holds an estimated position from a vison system. */
  public static record PoseEstimate(
      Pose2d pose,
      double latencySeconds,
      int tagCount,
      double averageDistance,
      double averageArea,
      boolean exists) {}
}
