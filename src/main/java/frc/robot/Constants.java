package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;
  public static final double fullThres = 0.99;
  public static final double heldThres = -0.9;
  public static final boolean IsLimeLightAprilTags = false;
  public static final int FALCON500_COUNTS_PER_REVOLUTION = 2048;
  public static final double GRAVITY = 9.8066;

  public static final int LEFT_FRONT_SPIN = 1;
  public static final int RIGHT_FRONT_SPIN = 3;
  public static final int LEFT_BACK_SPIN = 5;
  public static final int RIGHT_BACK_SPIN = 7;

  public static final int LEFT_FRONT_STEER = 2;
  public static final int RIGHT_FRONT_STEER = 4;
  public static final int LEFT_BACK_STEER = 6;
  public static final int RIGHT_BACK_STEER = 8;
  static class FieldConstants {
    static final double length = Units.feetToMeters(54);
    static final double width = Units.feetToMeters(27);
  }

  public static class TemporaryConstants {
    public static final int LEFT_LEADER = 3;
    public static final int RIGHT_LEADER =
     10;
    public static final int LEFT_FOLLOWER = 4;
    public static final int RIGHT_FOLLOWER = 2;
  }
  public static final int MOTORS_CHECKED_PER_TICK = 1;
  public static final int SRX_MAG_COUNTS_PER_REVOLUTION = 1024;
  public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
  public static final Transform3d robotToLimeLight = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


  public static class RobotConstants {
    public static final int ARM_ROTATION_MOTOR = 23;
    public static final int ARM_EXTENSION_MOTOR = 24;
    public static final int ORIENTATION_ELEVATED_MOTOR = 21;
    public static final int ORIENTATION_RAMP_MOTOR = 20;
    public static final int[] ORIENTATION_RAMP_SOLENOID_PORTS = {0, 1};
    public static final int[] ORIENTATION_ELEVATED_SOLENOID_PORTS = {2, 3};

  }

  public static class GripperConstants {
    public static final double GRIPPER_KP = 0.1;
    public static final double GRIPPER_SPEED = 1.0;
    public static final double GRIPPER_GEAR_RATIO = 1.6;
    public static final double GRIPPER_TOLERANCE = Units.degreesToRadians(2);
    public static final int[] GRIPPER_DOUBLE_SOLENOID_PORTS = {4, 5};
  }


  public static class ArmConstants {
    public static final double ARM_KA = 0.0;
    public static final double ARM_KV = 0.0;
    public static final double ARM_KG = 0.0;
    public static final double ARM_KS = 0.0;

    // public static final double ARM_ROTATION_KP = 0.05;
    public static final double ARM_ROTATION_KP = 0.0;
    public static final double ARM_ROTATION_KI = 0.0;
    public static final double ARM_ROTATION_KD = 0.0;
    public static final double ARM_ROTATION_KF = 0.0;
    public static final double ARM_REVOLUTIONS_PER_DEGREE = 4432;
    public static final double ARM_ROTATION_TOLERANCE = 1 * 4432;
    public static final double ARM_EXTENSION_KP = 0.8;
    public static final double ARM_EXTENSION_KI = 0.0;
    public static final double ARM_EXTENSION_KD = 0.0;
    public static final double ARM_FORWARD_SOFT_LIMIT = 110 * ARM_REVOLUTIONS_PER_DEGREE;
    public static final double ARM_REVERSE_SOFT_LIMIT = -100 * ARM_REVOLUTIONS_PER_DEGREE;

    public static final double ARM_ROTATION_GEAR_RATIO = 776.25;
    public static final double ARM_EXTENSION_GEAR_RATIO = 2.3;
    public static final double DISTANCE_PER_REVOLUTION_CM = 1.568;

    public static final double ARM_EXTENSION_TOLERANCE = 1 / DISTANCE_PER_REVOLUTION_CM * SRX_MAG_COUNTS_PER_REVOLUTION;


    
  }
}