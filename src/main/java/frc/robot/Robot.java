package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Robot extends TimedRobot {
  private DifferentialDrive m_robotDrive;
  private Joystick m_leftStick;

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final CANSparkMax m_leftMotor = new CANSparkMax(0, MotorType.kBrushless);
  private  Encoder m_leftEncoder = new Encoder(2, 3);
  private final CANSparkMax m_rightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private  Encoder m_rightEncoder = new Encoder(4,5);


  
  DifferentialDriveOdometry talhasus;
  Pose2d my_posem;

  @Override
  public void robotInit() {
      m_rightMotor.setInverted(true);
      m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
      m_leftStick = new Joystick(0);
      talhasus = new DifferentialDriveOdometry( m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
      
      m_leftEncoder.setDistancePerPulse(31);
      m_rightEncoder.setDistancePerPulse(31);
  }
  public void robotPeriodic(){
  my_posem =talhasus.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(),m_rightEncoder.getDistance());

    
    SmartDashboard.putNumber("poseX", my_posem.getX());
    SmartDashboard.putNumber("poseY", my_posem.getY());


  }
  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(-m_leftStick.getY(), -m_leftStick.getX());
    SmartDashboard.putNumber("xx", m_leftStick.getX());
    SmartDashboard.putNumber("yy", m_leftStick.getY());

  }
}
