package frc.robot.subsystems.swerve;

import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class SwerveMap {
  public static final class GyroMap {

    public static enum GyroType {
      PIGEON,
      NAVX,
      ADIS,
    }

    public static final GyroType GYRO_TYPE = GyroType.PIGEON;

    public static final int PIGEON_ID = 30;
  }

  public static final int FRD_ID = 6;
  public static final int FRR_ID = 5;
  public static final int FLD_ID = 2;
  public static final int FLR_ID = 1;

  public static final int BRD_ID = 8;
  public static final int BRR_ID = 7;
  public static final int BLD_ID = 4;
  public static final int BLR_ID = 3;

  static final double ODOMETRY_FREQUENCY = 250.0;

  // TODO refactor into separate constants files in swerve
  public static final boolean USING_TALON_DRIVE = false; // change to using kraken FOC?
  public static final boolean USING_VORTEX_DRIVE = false && !USING_TALON_DRIVE;

  public static final double TRACK_WIDTH = Units.inchesToMeters(23.5);
  public static final double WHEEL_BASE = Units.inchesToMeters(23.5);

  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

  /** feet per second -> meters per second */
  public static final double MAX_LINEAR_SPEED = 10 /*Units.feetToMeters(10)*/;
  /** radians per second */
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  public static final double WHEEL_RADIUS = Units.inchesToMeters(1.5);

  // this and below for choreo and pathfinding
  public static final double DRIVE_BASE_MASS = Units.lbsToKilograms(60.0);

  public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
  public static final int ROTATOR_MOTOR_CURRENT_LIMIT = 20;

  public static final double DRIVE_MOTOR_MAX_TORQUE =
      USING_TALON_DRIVE
          ? DCMotor.getKrakenX60Foc(1).getTorque(DRIVE_MOTOR_CURRENT_LIMIT)
          : (USING_VORTEX_DRIVE
              ? DCMotor.getNeoVortex(1).getTorque(DRIVE_MOTOR_CURRENT_LIMIT)
              : DCMotor.getNEO(1).getTorque(DRIVE_MOTOR_CURRENT_LIMIT));

  public static final double DRIVE_GEAR_RATIO =
      USING_TALON_DRIVE ? (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0) : 4.71;
  public static final double TURN_GEAR_RATIO = USING_TALON_DRIVE ? 150.0 / 7.0 : 9424.0 / 203.0;

  public static final double MAX_LINEAR_ACCELERATION =
      4 * (DRIVE_GEAR_RATIO * DRIVE_MOTOR_MAX_TORQUE) / WHEEL_RADIUS / DRIVE_BASE_MASS;
  public static final double MAX_ANGULAR_ACCELERATION =
      4 * (DRIVE_GEAR_RATIO * DRIVE_MOTOR_MAX_TORQUE) / WHEEL_RADIUS * DRIVE_BASE_RADIUS / 15.0;

  public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(10, 0, 0);
  public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(10, 0, 0);
  public static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, true, 3, 0.1);
}
