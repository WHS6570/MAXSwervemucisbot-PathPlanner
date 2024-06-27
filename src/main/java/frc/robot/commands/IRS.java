package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SASubsystem;

public class IRS extends Command {
private final IntakeSubsystem m_robotIntake;
private final DriveSubsystem m_robotDrive;
    
  public IRS(IntakeSubsystem intake, DriveSubsystem drive) {
    m_robotIntake = intake;
    m_robotDrive = drive;
    addRequirements(m_robotIntake, m_robotDrive);
  }
  @Override
  public void execute() {
    m_robotIntake.engulf(-1);
    m_robotDrive.eat(0, 0, 0, true, false, 0.2);
  }
  @Override
  public boolean isFinished() {
    if (m_robotIntake.seeIR()<=5) {
        return true;
    } else {
        return false;
    }
  }
  @Override
  public void end(boolean interrupted) {
  //  m_robotIntake.engulf(-1);
    //wait(100);
    m_robotIntake.engulf(0);
  }
}
