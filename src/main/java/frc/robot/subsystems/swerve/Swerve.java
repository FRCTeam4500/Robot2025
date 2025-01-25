package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.utilities.ExtendedMath.withHardDeadzone;
import static frc.robot.utilities.ScoringLocations.allianceFlip;
import static frc.robot.utilities.ScoringLocations.flip;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.Gyro;
import frc.robot.hardware.Limelight;
import frc.robot.hardware.Limelight.PoseEstimate;
import frc.robot.utilities.ExtendedMath;
import frc.robot.utilities.ScoringLocations;
import frc.robot.utilities.gamepieces.GamepieceManager;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

import java.util.Set;

/** The subsystem that controls our drivetrain, which is known as a swerve drive. */
public class Swerve extends SubsystemBase implements Loggable {
  private Gyro gyro;
  private SwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator estimator;
  private Limelight[] tagCameras;
  private Rotation2d targetHeading;
  private PIDController headingPID;

  /** Creates a new {@link Swerve} using the constants defined in {@link SwerveConstants} */
  public Swerve() {
    tagCameras = new Limelight[] {new Limelight("limelight-hehehe")};
    if (RobotBase.isReal()) { // running on hardware robot
      gyro = Gyro.fromNavX(navx -> {});
    } else { // running robot simulation
      gyro = Gyro.fromSim(() -> getSpeeds().omegaRadiansPerSecond);
    }
    modules =
        new SwerveModule[] {
          FRONT_LEFT_MODULE, FRONT_RIGHT_MODULE, BACK_LEFT_MODULE, BACK_RIGHT_MODULE
        };
    kinematics =
        new SwerveDriveKinematics(
            FRONT_LEFT_TRANSLATION,
            FRONT_RIGHT_TRANSLATION,
            BACK_LEFT_TRANSLATION,
            BACK_RIGHT_TRANSLATION);
    estimator =
        new SwerveDrivePoseEstimator(
            kinematics, gyro.getAngle(), getModulePositions(), new Pose2d());
    targetHeading = new Rotation2d();
    headingPID = new PIDController(5, 0, 0);
    headingPID.enableContinuousInput(-Math.PI, Math.PI);
    headingPID.setTolerance(Math.PI / 32, Math.PI / 32);
    headingPID.setSetpoint(0);

    GamepieceManager.setRobotPoseSupplier(estimator::getEstimatedPosition);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      Alert alert = new Alert("READING AUTO CONFIG FILE FAILED!!", AlertType.kError);
      alert.set(true);
      alert.close();
      System.out.println(e.getMessage());
      config =
          new RobotConfig(
              68, // robot's mass in kg
              6.884, // Robot's moment of inertia
              new ModuleConfig(0.5, 6, 1.2, DCMotor.getKrakenX60(1).withReduction(5.143), 60, 1),
              FRONT_LEFT_TRANSLATION.getY()
                  * 2 // the trackwidth of the robot (dist. from top left to top right for example)
              );
    }
    AutoBuilder.configure(
        estimator::getEstimatedPosition,
        this::resetPose,
        this::getSpeeds,
        this::drive,
        new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(0.5)),
        config,
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          return alliance == Alliance.Red;
        },
        this);

    NamedCommands.registerCommand("Idle", Commands.idle(this));
    NamedCommands.registerCommand(
        "To A", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.A)), Set.of(this)));
    NamedCommands.registerCommand(
        "To B", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.B)), Set.of(this)));
    NamedCommands.registerCommand(
        "To C", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.C)), Set.of(this)));
    NamedCommands.registerCommand(
        "To D", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.D)), Set.of(this)));
    NamedCommands.registerCommand(
        "To E", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.E)), Set.of(this)));
    NamedCommands.registerCommand(
        "To F", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.F)), Set.of(this)));
    NamedCommands.registerCommand(
        "To G", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.G)), Set.of(this)));
    NamedCommands.registerCommand(
        "To H", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.H)), Set.of(this)));
    NamedCommands.registerCommand(
        "To I", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.I)), Set.of(this)));
    NamedCommands.registerCommand(
        "To J", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.J)), Set.of(this)));
    NamedCommands.registerCommand(
        "To K", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.K)), Set.of(this)));
    NamedCommands.registerCommand(
        "To L", Commands.defer(() -> poseCentric(allianceFlip(ScoringLocations.L)), Set.of(this)));
  }

  /**
   * @param xbox The {@link XboxController that will control the driving}
   * @return a {@link Command} that drives the robot using a {@link XboxController}.
   *     <ul>
   *       <li>Translation of the robot is controlled with the left stick, field relative
   *       <li>Rotation of the robot is controlled with the right stick
   *       <li>A target angle can be set using {@link #setTargetHeading}, which the robot will then
   *           turn to.
   *     </ul>
   */
  public Command angleCentric(XboxController xbox) {
    return Commands.run(
            () -> {
              drive(calculateVelRobotRel(xbox));
            },
            this)
        .beforeStarting(() -> targetHeading = estimator.getEstimatedPosition().getRotation());
  }

  public Command robotCentric(XboxController xbox) {
    return Commands.run(
        () -> {
          double coefficient = Math.max(1 - xbox.getLeftTriggerAxis(), MIN_COEFFICIENT);
          drive(
              new ChassisSpeeds(
                  coefficient
                      * withHardDeadzone(-xbox.getLeftY(), 0.1)
                      * MAX_SPEEDS.vxMetersPerSecond,
                  coefficient
                      * withHardDeadzone(-xbox.getLeftX(), 0.1)
                      * MAX_SPEEDS.vyMetersPerSecond,
                  coefficient
                      * withHardDeadzone(-xbox.getRightX(), 0.1)
                      * MAX_SPEEDS.omegaRadiansPerSecond));
        },
        this);
  }

  @SuppressWarnings("resource")
  public Command poseCentric(Pose2d target) {
    PIDController forwardPID = new PIDController(3, 0, 0);
    PIDController sidewaysPID = new PIDController(3, 0, 0);
    PIDController rotationalPID = new PIDController(8, 0, 0);
    rotationalPID.enableContinuousInput(0, 2 * Math.PI); // "0-360 degrees"
    return Commands.run(
        () -> {
          Pose2d current = estimator.getEstimatedPosition();
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  forwardPID.calculate(current.getX(), target.getX()),
                  sidewaysPID.calculate(current.getY(), target.getY()),
                  rotationalPID.calculate(
                      current.getRotation().getRadians(), target.getRotation().getRadians()));
          drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, current.getRotation()));
        },
        this);
  }

  public Command alignToReef(boolean alignRight) {
    // use current position v center of reef
    // operate in range of angles
    // areas BASED ON SIDE OF REEF DUUUUHHHHH
    // 6 areas; 30 to -30,30 to 90,etc.
    return Commands.defer(
        () -> {
          Translation2d robert = estimator.getEstimatedPosition().getTranslation();
          Translation2d reef;
          Command poseCentricCommand = Commands.none();
          switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
            case Blue:
              reef = new Translation2d(4.5, 4);
              double angle = robert.minus(reef).getAngle().getDegrees();
              angle = MathUtil.inputModulus(angle, -30, 330);
              if (angle <= 30 && angle >= -30) {
                if (alignRight) poseCentricCommand = poseCentric(ScoringLocations.H);
                else poseCentricCommand = poseCentric(ScoringLocations.G);
              } else if (angle <= 90 && angle >= 30) {
                if (alignRight) poseCentricCommand = poseCentric(ScoringLocations.J);
                else poseCentricCommand = poseCentric(ScoringLocations.I);
              } else if (angle <= 150 && angle >= 90) {
                if (alignRight) poseCentricCommand = poseCentric(ScoringLocations.L);
                else poseCentricCommand = poseCentric(ScoringLocations.K);
              } else if (angle <= 210 && angle >= 150) {
                if (alignRight) poseCentricCommand = poseCentric(ScoringLocations.B);
                else poseCentricCommand = poseCentric(ScoringLocations.A);
              } else if (angle <= 270 && angle >= 210) {
                if (alignRight) poseCentricCommand = poseCentric(ScoringLocations.D);
                else poseCentricCommand = poseCentric(ScoringLocations.C);
              } else {
                if (alignRight) poseCentricCommand = poseCentric(ScoringLocations.F);
                else poseCentricCommand = poseCentric(ScoringLocations.E);
              }
              break;
            case Red:
              reef = new Translation2d(13.1, 4);
              double angleRed = robert.minus(reef).getAngle().getDegrees();
              angleRed = MathUtil.inputModulus(angleRed, -30, 330);
              if (angleRed <= 30 && angleRed >= -30) {
                if (alignRight) poseCentricCommand = poseCentric(flip(ScoringLocations.B));
                else poseCentricCommand = poseCentric(flip(ScoringLocations.A));
              } else if (angleRed <= 90 && angleRed >= 30) {
                if (alignRight) poseCentricCommand = poseCentric(flip(ScoringLocations.D));
                else poseCentricCommand = poseCentric(flip(ScoringLocations.C));
              } else if (angleRed <= 150 && angleRed >= 90) {
                if (alignRight) poseCentricCommand = poseCentric(flip(ScoringLocations.F));
                else poseCentricCommand = poseCentric(flip(ScoringLocations.E));
              } else if (angleRed <= 210 && angleRed >= 150) {
                if (alignRight) poseCentricCommand = poseCentric(flip(ScoringLocations.H));
                else poseCentricCommand = poseCentric(flip(ScoringLocations.G));
              } else if (angleRed <= 270 && angleRed >= 210) {
                if (alignRight) poseCentricCommand = poseCentric(flip(ScoringLocations.J));
                else poseCentricCommand = poseCentric(flip(ScoringLocations.I));
              } else {
                if (alignRight) poseCentricCommand = poseCentric(flip(ScoringLocations.L));
                else poseCentricCommand = poseCentric(flip(ScoringLocations.K));
              }
              break;
          }
          return poseCentricCommand;
        },
        Set.of(this));
  }

  /**
   * Updates the heading of the robot
   *
   * @param newHeading The robots new heading
   * @return A {@link Command} that can be bound to a {@link Trigger}
   */
  public Command resetHeading(Rotation2d newHeading) {
    return Commands.runOnce(
        () -> {
          resetPose(new Pose2d(estimator.getEstimatedPosition().getTranslation(), newHeading));
          targetHeading = new Rotation2d();
        });
  }

  /**
   * Sets the target heading for the robot in tele-op only
   *
   * @param targetHeading The target heading
   * @return A {@link Command} that can be bound to a {@link Trigger}
   */
  public Command setTargetHeading(Rotation2d targetHeading) {
    return Commands.runOnce(() -> this.targetHeading = targetHeading);
  }

  private ChassisSpeeds calculateVelRobotRel(XboxController xbox) {
    double speedCoefficient = Math.max(1 - xbox.getLeftTriggerAxis(), MIN_COEFFICIENT);
    Rotation2d currentHeading = estimator.getEstimatedPosition().getRotation();
    targetHeading =
        Rotation2d.fromRadians(
            targetHeading.getRadians()
                - withHardDeadzone(xbox.getRightX(), 0.1)
                    * speedCoefficient
                    * MAX_SPEEDS.omegaRadiansPerSecond
                    * 0.02);
    double rotational =
        headingPID.calculate(currentHeading.getRadians(), targetHeading.getRadians());
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      speedCoefficient *= -1;
    }
    double forward =
        speedCoefficient * withHardDeadzone(xbox.getLeftY(), 0.1) * MAX_SPEEDS.vxMetersPerSecond;
    double sideways =
        speedCoefficient * withHardDeadzone(xbox.getLeftX(), 0.1) * MAX_SPEEDS.vyMetersPerSecond;
    ChassisSpeeds fieldRel = new ChassisSpeeds(forward, sideways, rotational);
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRel, currentHeading);
  }

  private ChassisSpeeds applySkewCorrection(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    double angle = SKEW_COEFFICIENT * speeds.omegaRadiansPerSecond;
    return new ChassisSpeeds(
        Math.cos(angle) * speeds.vxMetersPerSecond - Math.sin(angle) * speeds.vyMetersPerSecond,
        Math.sin(angle) * speeds.vxMetersPerSecond + Math.cos(angle) * speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond);
  }

  public Command skewTest() {
    return run(() -> {
          ChassisSpeeds fieldRel = new ChassisSpeeds(3, 0, Math.PI);
          drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRel, estimator.getEstimatedPosition().getRotation()));
        })
        .finallyDo(() -> drive(new ChassisSpeeds()));
  }

  private void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_MODULE_SPEED);
    speeds = kinematics.toChassisSpeeds(states);
    states = kinematics.toSwerveModuleStates(applySkewCorrection(speeds));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_MODULE_SPEED);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(states[i]);
    }
  }

  private void resetPose(Pose2d pose) {
    estimator.resetPosition(gyro.getAngle(), getModulePositions(), pose);
  }

  private ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getCurrentState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {

      states[i] = modules[i].getCurrentPosition();
    }
    return states;
  }

  @Override
  public void periodic() {
    estimator.update(gyro.getAngle(), getModulePositions());
    boolean speedLimit =
        (ExtendedMath.within(getSpeeds(), new ChassisSpeeds(), new ChassisSpeeds(1, 1, 2 * Math.PI))
            || !DriverStation.isAutonomous());
    for (Limelight camera : tagCameras) {
      PoseEstimate estimate = camera.getPoseMT1();
      if (estimate.exists()
          && speedLimit
          && (estimate.tagCount() > 1 || estimate.averageDistance() < 4))
        estimator.addVisionMeasurement(
            estimate.pose(), Timer.getFPGATimestamp() - estimate.latencySeconds());
    }
    for (SwerveModule module : modules) {
      module.periodic();
    }
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Modules", getModuleStates());
    HoundLog.log(path, "Speeds", getSpeeds());
    HoundLog.log(path, "Pose", estimator.getEstimatedPosition());
    HoundLog.log(path, "Target Heading", targetHeading);
    HoundLog.log(path, "Gyro Angle", gyro.getAngle());
    HoundLog.log(path, "Front Left Module", modules[0]);
    HoundLog.log(path, "Front Right Module", modules[1]);
    HoundLog.log(path, "Back Left Module", modules[2]);
    HoundLog.log(path, "Back Right Module", modules[3]);
    HoundLog.log(path, "Gyro", gyro);
  }
}
