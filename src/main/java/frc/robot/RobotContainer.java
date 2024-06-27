// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LockTargetTele;
import frc.robot.commands.Rotate;
import frc.robot.commands.Shoot;
import frc.robot.commands.Untouch;
import frc.robot.commands.GoThere;
import frc.robot.commands.IRS;
import frc.robot.commands.LockTargetAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.SASubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final SASubsystem m_robotSA = new SASubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final SendableChooser<Command> autoChooser;
  private final ClimberSubsystem m_robotClimber = new ClimberSubsystem();

  // The driver's controller
 // XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
 // XboxController m_shooterController = new XboxController(OIConstants.kShooterControllerPort);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("IRS", new IRS(m_robotIntake,m_robotDrive));
    NamedCommands.registerCommand("LockTargetAuto", new LockTargetAuto(0,0,m_robotDrive,m_robotSA));
    NamedCommands.registerCommand("Shoot", new Shoot(0,0,m_robotDrive,m_robotSA,m_robotIntake));
    NamedCommands.registerCommand("Untouch", new Untouch(m_robotIntake));


    // Configure the button bindings
    configureButtonBindings();
       // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(OI.m_driverController.getLeftY()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OI.m_driverController.getLeftX()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OI.m_driverController.getRightX()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
    m_robotSA.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotSA.stop(),
            m_robotSA));
     m_robotIntake.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotIntake.stop(),
            m_robotIntake));
    m_robotClimber.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotClimber.climb(OI.m_shooterController.getRightTriggerAxis()-OI.m_shooterController.getLeftTriggerAxis()),
            m_robotClimber));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(OI.m_driverController, Button.kB.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(OI.m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.track(
                  -MathUtil.applyDeadband(OI.m_driverController.getLeftY()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OI.m_driverController.getLeftX()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OI.m_driverController.getRightX()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                true, true
            ),
            m_robotDrive));
    new JoystickButton(OI.m_driverController, Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.eat(
                  -MathUtil.applyDeadband(OI.m_driverController.getLeftY()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OI.m_driverController.getLeftX()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OI.m_driverController.getRightX()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                true, true, 0.5
            ),
            m_robotDrive));
    new JoystickButton(OI.m_driverController, Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(
                  -MathUtil.applyDeadband(OI.m_driverController.getLeftY()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OI.m_driverController.getLeftX()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OI.m_driverController.getRightX()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                false, true
            ),
            m_robotDrive));
        // Shooter binds
        new JoystickButton(OI.m_shooterController, Button.kY.value)
        .whileTrue(  new LockTargetTele(
                -MathUtil.applyDeadband(OI.m_driverController.getLeftY()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OI.m_driverController.getLeftX()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                 m_robotDrive, m_robotSA));
           
   
         new JoystickButton(OI.m_driverController, Button.kStart.value)
        .whileTrue(new GoThere(-0.9, m_robotDrive));
          new JoystickButton(OI.m_shooterController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotIntake.engulf(-1),
            m_robotIntake));
          new JoystickButton(OI.m_shooterController, Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotIntake.engulf(0.15),
            m_robotIntake));
          new JoystickButton(OI.m_shooterController, Button.kA.value)
        .whileTrue(new RunCommand(
          () -> m_robotSA.zAim(100.0),
            m_robotSA));
          new JoystickButton(OI.m_shooterController, Button.kB.value)
        .whileTrue(new RunCommand(
            () -> m_robotSA.zAim(186.0),
            m_robotSA));
            new JoystickButton(OI.m_shooterController, Button.kStart.value)
        .whileTrue(new RunCommand(
            () -> m_robotClimber.unlock(),
            m_robotClimber));
             new JoystickButton(OI.m_shooterController, Button.kBack.value)
        .whileTrue(new RunCommand(
            () -> m_robotClimber.lock(),
            m_robotClimber));
             new JoystickButton(OI.m_shooterController, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotSA.shoot(80),m_robotSA));
            new JoystickButton(OI.m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotSA.zAim(170),m_robotSA));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
   
  }
}
