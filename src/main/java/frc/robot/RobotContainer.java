// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.primitive.TankDrive;
import frc.robot.Commands.primitive.humenArmAndRotate;
import frc.robot.Commands.primitive.toggleGripperV2Solenoid;
import frc.robot.subsystems.Tank;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripperV2.GripperV2;
import frc.robot.subsystems.orientation.Orientation;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.util.humanIO.CommandPS5Controller;

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
  GripperV2 gripperV2 = GripperV2.getInstance();
  Orientation orientation = Orientation.getInstance();
  Pneumatics pneumatics = Pneumatics.getInstance();
  Tank tank = Tank.getinstance();
  double speed = 0.0;
  double rot = 0.0;



  // Controller
  private static final CommandPS5Controller controller = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    controller.circle().onTrue(new toggleGripperV2Solenoid(1));
    controller.square().onTrue(new toggleGripperV2Solenoid(2));
    controller.triangle().onTrue(new toggleGripperV2Solenoid(3));
    Tank.getinstance().setDefaultCommand(new TankDrive());
    Arm.getInstance().setDefaultCommand(new humenArmAndRotate());
  }

  public static double getRawAxis(int axis){
    return controller.getRawAxis(axis);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
