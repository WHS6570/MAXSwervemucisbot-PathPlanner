package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.SASubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class LockTargetAuto extends Command {
  // The subsystem the command runs on
  private final DriveSubsystem m_robotDrive;
  private final SASubsystem m_robotSAI;
  private double xspeed;
  private double yspeed;
  private double angle;
  public double targetangle;

  public LockTargetAuto(double xspeed, double yspeed, DriveSubsystem drive, SASubsystem SAI) {
    m_robotDrive = drive;
    m_robotSAI = SAI;

    addRequirements(m_robotDrive, m_robotSAI);
  }

  @Override
  public void initialize() {

}

  @Override
  public void execute() {
   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tz = table.getEntry("tz");
    //NetworkTableEntry botpose = table.getEntry("botpose");
    //   double x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //read values periodically
    //double x = tx.getDouble(0.0);
    double angle = ty.getDouble(0.0);
    double distance = tz.getDouble(0);
    double xangle = tx.getDouble(0);

    targetangle = 138+(0.6*(Math.abs(angle-22)));
    //double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    //double posetx = botpose[1];
    //post to smart dashboard periodically
    //SmartDashboard.putNumber("LimelightX", x);
    //SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightArea", area);
    //SmartDashboard.putNumber("LimelightTX", posetx);
    xspeed = 0;
    yspeed = 0;
   m_robotSAI.shoot(70); //was 110, now 100, again
   m_robotDrive.yAimAuto(xspeed, yspeed, 0, false, false);
   //m_robotSAI.zAim(127.5 -(angle*0.5) - (distance));// - (angle -(0.8 *(angle - 12)))); //was 127.5, now 127.5. Was -(0.3, now -(0.8))
   m_robotSAI.zAim(targetangle);
SmartDashboard.putNumber("armangle", (m_robotSAI.getArmAngle()));
SmartDashboard.putNumber("targetangle", (targetangle));
SmartDashboard.putNumber("xangle", (xangle));
SmartDashboard.putNumber("RPS", m_robotSAI.getShooterSpeed());

 }

  @Override
  public boolean isFinished() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    double xangle = tx.getDouble(0.0);
    //double yCurrent = (138+(0.625*(Math.abs(angle-22))));
    double ymax = targetangle + 0;
    double ymin = targetangle - 6;
    SmartDashboard.putNumber("Ymin", ymin);
    SmartDashboard.putNumber("Ymax", ymax);
 if  ((ymin<=m_robotSAI.getArmAngle() &&
     ymax>=m_robotSAI.getArmAngle()) &&
     (m_robotSAI.getShooterSpeed()>= 68) &&
     (xangle>=-9.0 && xangle<=9.0)) {
    return true;
    } else {
    return false;}
    }
  }