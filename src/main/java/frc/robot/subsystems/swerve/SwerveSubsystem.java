package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Config.Subsystems.GAME_PIECE_VISION_ENABLED;
import static frc.robot.Config.Subsystems.VISION_ENABLED;
import static frc.robot.GlobalConstants.FieldMap.Coordinates.SPEAKER;
import static frc.robot.GlobalConstants.MODE;
import static frc.robot.subsystems.swerve.SwerveMap.*;
import static frc.robot.subsystems.swerve.SwerveMap.GyroMap.GYRO_TYPE;
import static frc.robot.subsystems.vision.VisionMap.CameraType.LIMELIGHT;
import static frc.robot.subsystems.vision.apriltagvision.AprilTagVisionMap.*;
import static java.lang.Math.PI;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.vision.apriltagvision.AprilTagVision;
import frc.robot.subsystems.vision.apriltagvision.AprilTagVisionIOPhotonReal;
import frc.robot.subsystems.vision.apriltagvision.AprilTagVisionIOPhotonSim;
import frc.robot.subsystems.vision.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.vision.gamepiecevision.GamePieceVisionIOLimelightReal;
import frc.robot.subsystems.vision.gamepiecevision.GamePieceVisionIOPhotonSim;
import frc.robot.subsystems.vision.gamepiecevision.GamePieceVisionMap;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {

  private final AprilTagVision vision =
      VISION_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new AprilTagVision(
                  new AprilTagVisionIOPhotonReal(LEFT_CAM_CONSTANTS, RIGHT_CAM_CONSTANTS))
              : new AprilTagVision(
                  new AprilTagVisionIOPhotonSim(
                      this::getPose, LEFT_CAM_CONSTANTS, RIGHT_CAM_CONSTANTS)))
          : null;

  private final BuiltInAccelerometer accelerometer =
      new BuiltInAccelerometer(BuiltInAccelerometer.Range.k4G);
  private final Timer driftTimer = new Timer();

  private final GamePieceVision gamePieceVision =
      GAME_PIECE_VISION_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new GamePieceVision(
                  new GamePieceVisionIOLimelightReal(GamePieceVisionMap.NOTE_CAM_CONSTANTS))
              : new GamePieceVision(
                  new GamePieceVisionIOPhotonSim(
                      this::getPose, GamePieceVisionMap.NOTE_CAM_CONSTANTS)))
          : null;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
      };

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation,
          lastModulePositions,
          new Pose2d(),
          VecBuilder.fill(1, 1, 0.01),
          VecBuilder.fill(0.01, 0.01, PI));

  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  // add the CANCoder id between the rotator id and offset params
  private static final ModuleConstants frontLeft =
      new ModuleConstants("Front Left", FLD_ID, FLR_ID, Rotation2d.fromRadians(-PI / 2));
  private static final ModuleConstants frontRight =
      new ModuleConstants("Front Right", FRD_ID, FRR_ID, Rotation2d.fromRadians(0));
  private static final ModuleConstants backLeft =
      new ModuleConstants("Back Left", BLD_ID, BLR_ID, Rotation2d.fromRadians(PI));
  private static final ModuleConstants backRight =
      new ModuleConstants("Back Right", BRD_ID, BRR_ID, Rotation2d.fromRadians(PI / 2));

  protected final SysIdRoutine sysId;

  public SwerveSubsystem(GyroIO gyroIO, ModuleIO[] moduleIOS) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(moduleIOS[0]);
    modules[1] = new Module(moduleIOS[1]);
    modules[2] = new Module(moduleIOS[2]);
    modules[3] = new Module(moduleIOS[3]);
    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            TRANSLATION_CONSTANTS,
            ROTATION_CONSTANTS,
            MAX_LINEAR_SPEED,
            DRIVE_BASE_RADIUS,
            REPLANNING_CONFIG),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        this);
    PathPlannerLogging.setLogActivePathCallback(
        activePath ->
            Logger.recordOutput(
                "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
    PathPlannerLogging.setLogTargetPoseCallback(
        targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    //    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  // only 4-mod swerves, please and thank you build team
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  private Optional<Rotation2d> getRotationTargetOverride() {
    if (gamePieceVision.getTargetToRobotOffset(this::getPose).equals(new Transform2d()))
      return Optional.of(
          getPose()
              .getTranslation()
              .minus(SPEAKER.getPose().getTranslation())
              .unaryMinus()
              .getAngle());
    else
      return Optional.of(
          gamePieceVision
              .getTargetToRobotOffset(this::getPose)
              .getTranslation()
              .getAngle()
              .rotateBy(Rotation2d.fromDegrees(-90)));

    //    if (DriverStation.getAlliance().isPresent()
    //        && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
    //      if (getPose().getX() > Constants.FieldMap.FIELD_LENGTH_METERS / 2)
    //        return Optional.of(
    //            getPose()
    //                .getTranslation()
    //                .minus(Constants.FieldMap.Coordinates.SPEAKER.getPose().getTranslation())
    //                .unaryMinus()
    //                .getAngle());
    //    return Optional.empty();
  }

  public double getNoteOffset() {
    return hasNote() ? gamePieceVision.getNoteAngle() : 0;
  }

  public boolean hasNote() {
    return gamePieceVision.hasNote()
        && gamePieceVision.getNoteDistance() < LIMELIGHT.getNoisyDistance();
  }

  public Command pidToPoseRobotRelative(Supplier<Pose2d> pose) {
    TrapezoidProfile.Constraints X_CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAX_LINEAR_SPEED, MAX_LINEAR_ACCELERATION);

    ProfiledPIDController xController = new ProfiledPIDController(20, 0, 0, X_CONSTRAINTS);
    ProfiledPIDController yController = new ProfiledPIDController(20, 0, 0, X_CONSTRAINTS);

    PIDController omegaPID = new PIDController(10, 0, 0);

    xController.setTolerance(0.05);
    xController.setTolerance(0.05);
    omegaPID.setTolerance(1.5);
    omegaPID.enableContinuousInput(-180, 180);

    return this.defer(
        () ->
            new FunctionalCommand(
                () -> {},
                () -> {
                  double xSpeed = xController.calculate(0, pose.get().getX());
                  double ySpeed = yController.calculate(0, pose.get().getY());
                  double omegaSpeed = omegaPID.calculate(0, pose.get().getRotation().getDegrees());
                  this.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
                },
                interrupted -> {
                  this.runVelocity(new ChassisSpeeds(0, 0, 0));
                  omegaPID.close();
                },
                () -> omegaPID.atSetpoint() && xController.atGoal() && yController.atGoal(),
                this));
  }

  /*
  do *NOT* make this a proxy! we only want schedule-on-schedule, not end-on-end.
  if end-on-end behavior is desired, use interruption instead
  */
  public Command pathFindCommand(Supplier<Pose2d> pose) {
    return AutoBuilder.pathfindToPose(
        pose.get(),
        new PathConstraints(
            MAX_LINEAR_SPEED, MAX_LINEAR_ACCELERATION, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION),
        0,
        0);
  }

  public Command pathFindThenFollowPathCommand(Supplier<PathPlannerPath> path) {
    return AutoBuilder.pathfindThenFollowPath(
        path.get(),
        new PathConstraints(
            MAX_LINEAR_SPEED, MAX_LINEAR_ACCELERATION, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION),
        0);
  }

  public Command followPathCommand(Supplier<PathPlannerPath> path) {
    return AutoBuilder.followPath(path.get());
  }

  public static GyroIO getRealGyro() {
    return switch (GYRO_TYPE) {
      case PIGEON -> new GyroIOPigeon2(USING_TALON_DRIVE);
      case NAVX -> new GyroIONavX();
      case ADIS -> new GyroIOADIS();
    };
  }

  public static ModuleIO[] getRealModules() {
    return USING_TALON_DRIVE
        ? new ModuleIO[] {
          new ModuleIOTalonFX(frontLeft),
          new ModuleIOTalonFX(frontRight),
          new ModuleIOTalonFX(backLeft),
          new ModuleIOTalonFX(backRight),
        }
        : new ModuleIO[] {
          USING_VORTEX_DRIVE ? new ModuleIOSparkFlex(frontLeft) : new ModuleIOSparkMax(frontLeft),
          USING_VORTEX_DRIVE ? new ModuleIOSparkFlex(frontRight) : new ModuleIOSparkMax(frontRight),
          USING_VORTEX_DRIVE ? new ModuleIOSparkFlex(backLeft) : new ModuleIOSparkMax(backLeft),
          USING_VORTEX_DRIVE ? new ModuleIOSparkFlex(backRight) : new ModuleIOSparkMax(backRight),
        };
  }

  public static ModuleIO[] getSimModules() {
    return new ModuleIOSim[] {
      new ModuleIOSim(frontLeft.name()),
      new ModuleIOSim(frontRight.name()),
      new ModuleIOSim(backLeft.name()),
      new ModuleIOSim(backRight.name()),
    };
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    Logger.recordOutput("Drive/Note Offset", getNoteOffset() + 180);

    Logger.recordOutput("Drive/Zeroed Pose", new Pose2d());
    Logger.recordOutput("Drive/MechZero", new Pose3d[] {new Pose3d()});

    Logger.recordOutput(
        "Drive/SpeakerOffset",
        SPEAKER.getPose().getTranslation().minus(getPose().getTranslation()).getAngle().getDegrees()
            - getRotation().getDegrees());

    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.yawPosition;
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, getModulePositions());
      // Apply vision updates
      if (VISION_ENABLED) updateVision();
      if (GAME_PIECE_VISION_ENABLED) updateGamePieceVision();
    }
  }

  private void updateGamePieceVision() {
    Logger.recordOutput(
        "Note offset",
        gamePieceVision
            .getTargetToRobotOffset(this::getPose)
            .getTranslation()
            .getAngle()
            .rotateBy(Rotation2d.fromDegrees(-90))
            .getDegrees());
    Logger.recordOutput("Transform", gamePieceVision.getTargetToRobotOffset(this::getPose));
  }

  private void updateVision() {
    vision
        .processPoseEstimates()
        .forEach(
            timestampedUpdate ->
                poseEstimator.addVisionMeasurement(
                    poseEstimator
                        .getEstimatedPosition()
                        .interpolate(timestampedUpdate.poseEstimate(), 0.25),
                    timestampedUpdate.timestamp(),
                    // todo *NO*
                    timestampedUpdate.stdDevs()));
  }

  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all the modules. */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
      new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
      new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
      new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
    };
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }
}
