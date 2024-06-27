package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SASubsystem;

public class GoThere extends Command {
private final DriveSubsystem m_robotDrive;
double distance;    

  public GoThere(double targdist, DriveSubsystem drive) {
    distance = targdist;
    m_robotDrive = drive;
    addRequirements(m_robotDrive);
  }
  @Override
  public void execute() {
    //m_robotIntake.engulf(-1);
    m_robotDrive.driveto(distance);
  }
  @Override
 public boolean isFinished() {
  double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[6]);
  double posetx = botpose[2];
    if ((posetx >=distance-0.1) && (posetx<=distance+0.1)) {
        return true;
    } else {
        return false;
    }
  }
  //@Override
  //public void end(boolean interrupted) {
  //  m_robotIntake.engulf(-1);
  //  wait(100);
  //  m_robotIntake.engulf(0);
   // }
}
