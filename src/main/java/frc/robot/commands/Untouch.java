package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SASubsystem;

public class Untouch extends Command {
private final IntakeSubsystem m_robotIntake;
    
  public Untouch(IntakeSubsystem intake) {
    m_robotIntake = intake;
    addRequirements(m_robotIntake);
  }
  @Override
  public void execute() {
    m_robotIntake.engulf(0.2);
  }
  @Override
  public boolean isFinished() {
    if (m_robotIntake.seeIR()>=6) {
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
