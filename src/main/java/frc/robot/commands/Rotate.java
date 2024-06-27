package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SASubsystem;

public class Rotate extends Command {
private final DriveSubsystem m_robotDrive;
double angle;    

  public Rotate(double rangle, DriveSubsystem drive) {
    angle = rangle;
    m_robotDrive = drive;
    addRequirements(m_robotDrive);
  }
  @Override
  public void execute() {
    //m_robotIntake.engulf(-1);
    m_robotDrive.rotate(angle);
  }
  @Override
 public boolean isFinished() {
    if ((m_robotDrive.getHeading()>=angle-5) && (m_robotDrive.getHeading()<=angle+5)) {
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
