// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.primitive.gripper.setGripperState;
import frc.robot.commands.primitive.orientation.setOrientationSpeed;
import frc.robot.commands.primitive.orientation.setRampSolenoid;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.util.SequenceType;
import frc.util.humanIO.CommandPS5Controller;
import frc.util.humanIO.JoystickAxis;
import frc.robot.SwerveModuleConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  Arm arm = Arm.getInstance();
  Gripper gripperV2 = Gripper.getInstance();
  public static final SwerveModuleConstants FLModule = new SwerveModuleConstants(2, 3,
  SwerveModuleConstants.cancoderTLOffset, 10, false, false);
  public static final SwerveModuleConstants FRModule = new SwerveModuleConstants(4, 5,
  SwerveModuleConstants.cancoderTROffset, 11, false, false);
  public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(6, 7,
  SwerveModuleConstants.cancoderBLOffset, 12, false, false);
  public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(8, 9,
  SwerveModuleConstants.cancoderBROffset, 13, false, false);



  // Controller
  private static final CommandPS5Controller controller = new CommandPS5Controller(0);
  private static final JoystickAxis R2Trigger = new JoystickAxis(controller, 4);
  private static final JoystickAxis L2Trigger = new JoystickAxis(controller, 3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
  private static boolean inverted = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Pneumatics.getInstance();
    Gripper.getInstance();


    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        // drive = new Drive(new DriveIOFalcon500());
        // flywheel = new Flywheel(new FlywheelIOFalcon500());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        break;

      // Replayed robot, disable IO implementations
      default:
        //drive = new Drive(new DriveIO() {
        //});
        break;
    }
    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // controller.cross().onTrue(new toggleOrienationMotors(0.6));
    // controller.square().onTrue(new toggleRampSolenoid());
    controller.circle().onTrue(new setGripperState(SequenceType.Cone));
    controller.triangle().onTrue(new setGripperState(SequenceType.Cube));
    controller.options().onTrue(new setGripperState(SequenceType.Off));
    controller.L1().whileTrue(new setOrientationSpeed());
    controller.R1().onTrue(new setRampSolenoid(false)).onFalse(new setRampSolenoid(true));

    R2Trigger.onTrue(new InstantCommand(() -> arm.setFalconPO(0.5))).onFalse(new InstantCommand(() -> arm.setFalconPO(0)));
    L2Trigger.onTrue(new InstantCommand(() -> arm.setFalconPO(-0.5))).onFalse(new InstantCommand(() -> arm.setFalconPO(0)));

    controller.povLeft().onTrue(new InstantCommand(() -> arm.set775PO(0.5))).onFalse(new InstantCommand(() -> arm.set775PO(0)));
    controller.povRight().onTrue(new InstantCommand(() -> arm.set775PO(-0.5))).onFalse(new InstantCommand(() -> arm.set775PO(0)));
    // controller.povDown().onTrue(new ExtendOrRotateArm(SequenceType.Arm, 0));
    // controller.L1().onTrue(new toggleOrienationSoleniod());
    // controller.R1().onTrue(new toggleOrienationSoleniod());
    // controller.povRight().onTrue(new ExtendOrRotateArm(SequenceType.Arm, 0));
    // controller.povUp().onTrue(new ExtendOrRotateArm(SequenceType.Arm, 91));
    // controller.povLeft().onTrue(new ExtendOrRotateArm(SequenceType.Arm, -17));
  }
  public static SwerveModuleState stateFromController() {
    double y = -controller.getLeftY();
    double x = controller.getLeftX();
    if (Math.abs(x) < 0.1)
      x = 0;
    if (Math.abs(y) < 0.1)
      y = 0;

    return new SwerveModuleState(Math.sqrt(x * x + y * y) * 0.5, new Rotation2d(x, y));
  }

  public static ChassisSpeeds speedsFromController() {
    double x = controller.getLeftX();
    double y = -controller.getLeftY();
    double rot = controller.getRightX();

    if (Math.abs(x) < 0.2)
      x = 0;
    if (Math.abs(y) < 0.2)
      y = 0;
    if (Math.abs(rot) < 0.2)
      rot = 0;

    return new ChassisSpeeds(1.5 * x, 1.5 * y, rot);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static double calculateDeadBand(double value) {
    if (Math.abs(value) < 0.2)
      return 0;
    return value;
  }

  public static double getRawAxis(int axis) {
    return calculateDeadBand(controller.getRawAxis(axis));
  }

}


