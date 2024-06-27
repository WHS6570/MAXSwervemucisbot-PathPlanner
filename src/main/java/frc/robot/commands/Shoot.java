package frc.robot.commands;

import java.util.Timer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.SASubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class Shoot extends Command {
  // The subsystem the command runs on
  private final DriveSubsystem m_robotDrive;
  private final SASubsystem m_robotSAI;
  private final IntakeSubsystem m_robotIntake;
  private double m_startTime;
  private double xspeed;
  private double yspeed;
  private double angle;

  public Shoot(double xspeed, double yspeed, DriveSubsystem drive, SASubsystem SAI, IntakeSubsystem intake) {
    m_robotDrive = drive;
    m_robotSAI = SAI;
    m_robotIntake = intake;
    addRequirements(m_robotDrive, m_robotSAI, m_robotIntake);
  }

  @Override
  public void initialize() {
    m_startTime = WPIUtilJNI.now() * 1e-6;

}

  @Override
  public void execute() {
   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //NetworkTableEntry tx = table.getEntry("tx");
    
    NetworkTableEntry ty = table.getEntry("ty");
    //NetworkTableEntry ta = table.getEntry("ta");
    //NetworkTableEntry botpose = table.getEntry("botpose");
    //   double x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //read values periodically
    //double x = tx.getDouble(0.0);
    double angle = ty.getDouble(0.0);
    //double area = ta.getDouble(2.0);
    //double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    //double posetx = botpose[1];
    //post to smart dashboard periodically
    //SmartDashboard.putNumber("LimelightX", x);
    //SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightArea", area);
    //SmartDashboard.putNumber("LimelightTX", posetx);
    xspeed = -MathUtil.applyDeadband(OI.m_driverController.getLeftY()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband);
    yspeed = -MathUtil.applyDeadband(OI.m_driverController.getLeftX()*(1-(OI.m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband);
   m_robotSAI.shoot(70); //was 110, now 100, now 70
   m_robotDrive.yAimAuto(xspeed, yspeed, 0, false, false);
   //m_robotSAI.zAim(127.5 - (angle -(0.2 *(angle - 12)))); //was 127.5, now 127.5, again. Was -(0.3, now -(0.2))
   m_robotSAI.zAim(138+(0.6*(Math.abs(angle-22))));

SmartDashboard.putNumber("armangle", (m_robotSAI.getArmAngle()));
   m_robotIntake.engulf(-1);
 }

  @Override
  public boolean isFinished() {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_startTime;
    if (elapsedTime>0.5) {return true;} else {return false;}
  }
  @Override
  public void end(boolean interrupted) {
  m_robotSAI.zAim(105);
  m_robotSAI.shoot(0);
  }
  }
