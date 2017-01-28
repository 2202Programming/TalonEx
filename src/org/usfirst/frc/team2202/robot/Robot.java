package org.usfirst.frc.team2202.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	CANTalon _talon = new CANTalon(13);
	XboxController _joy = XboxController.getXboxController();
	private double curSpeed = 0;

	public void robotInit() {
		/* first choose the sensor */
		_talon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		//SmartDashboard.putBoolean("encExists", _talon.);
		_talon.reverseSensor(false);

		/* set the peak and nominal outputs, 12V means full */
		_talon.configNominalOutputVoltage( +0.0f, -0.0f);
		_talon.configPeakOutputVoltage( +12.0f, 0f);
		/* set closed loop gains in slot0 */
		_talon.changeControlMode(TalonControlMode.Speed);
		
		_talon.configEncoderCodesPerRev(1024);
		_talon.setProfile(0);
		_talon.setF(0.008562);
		_talon.setP(0.02046);
		_talon.setI(0);
		_talon.setD(0);
	}
	
	public void teleopInit(){
		_talon.setEncPosition(0);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		_joy.teleopPeriodic();
		/* get gamepad axis */
		double leftYstick = _joy.getLeftJoystickY();

		/* Speed mode */
		double targetSpeed = leftYstick * 100.0; /* RPM in either direction */
		SmartDashboard.putNumber("speed", targetSpeed);
		if(_joy.getAPressed()){
			curSpeed += 200;
		}
		if(_joy.getBPressed()){
			curSpeed -= 200;
		}
		if(_joy.getYPressed()){
			curSpeed = 0;
		}
		if(_joy.getXPressed()){
			curSpeed += 1000;
		}
		SmartDashboard.putNumber("voltage", curSpeed);
		_talon.set(curSpeed);
		SmartDashboard.putNumber("realSpeed", _talon.getSpeed());
		SmartDashboard.putNumber("enc", _talon.getEncPosition());
		SmartDashboard.putNumber("err", _talon.getError());
		
	}
}