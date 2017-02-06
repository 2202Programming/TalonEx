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
	int code_state = 0;

	public void robotInit() {
		this.code_state = 1;
		/* first choose the sensor */
		_talon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		_talon.configEncoderCodesPerRev(4096);   //1024, but measured 4096
		
		//SmartDashboard.putBoolean("encExists", _talon.);
		_talon.reverseSensor(false);

		/* set the peak and nominal outputs, 12V means full */
		_talon.configNominalOutputVoltage( +0.0f, -0.0f);
		_talon.configPeakOutputVoltage( +12.0f, -12.0f);
		/* set closed loop gains in slot0 */
		_talon.changeControlMode(TalonControlMode.Speed);
		
		_talon.setProfile(0);

		_talon.setF(0.013);    /// 0.008562
		_talon.setP(0.018);    // 102/4000  .1*1023/4000Uerror  = 0.0255
		_talon.setI(0.00002);   // set P/100 to start
		_talon.setD(0.0000);
		_talon.setAllowableClosedLoopErr(0);
		
		//limit the error wind up, greater than IZone, zeros the integrator.
		//
		//    err= 500RPM    (50r/min)*(4096U/rev)*(1/60 min/s)*(1/10 s/100ms)
		//       =  3410  U/100ms    (U is pulses or native units) 
		//			
		_talon.setIZone(0);   //should be in native-units (pulses/100ms) 
		//Izone seems to break the CL control by zeroing the iAcc when the err gets to zero
		//this feels like a bug in the SRX.  - Derek
		_talon.setEncPosition(0);
		_talon.ClearIaccum();
		
	}
	
	public void breaking(){
        for (;true;) System.out.println("Hello, World");
	}
	
	public void teleopInit(){
		this.code_state++;
		_talon.setEncPosition(0);
		this.curSpeed = 0;
		_talon.ClearIaccum();
		_talon.clearStickyFaults();
		_talon.setAllowableClosedLoopErr(20);
	}

	/**
	 * This function is called periodically during operator control  - called every 20mS
	 */
	public void teleopPeriodic() {
		_joy.teleopPeriodic();
		/* get gamepad axis */
		double leftYstick = _joy.getLeftJoystickY();
		
		if(_joy.getAPressed()){
			curSpeed += 50;
		}
		if(_joy.getBPressed()){
			curSpeed -= 50;
		}
		if(_joy.getYPressed()){
			this.code_state = 0;
			_talon.setEncPosition(0);
			_talon.clearIAccum();
			_talon.ClearIaccum();
			curSpeed = curSpeed;
		}
		if(_joy.getXPressed()){
			curSpeed += 200;
		}
		SmartDashboard.putNumber("firmware", _talon.GetFirmwareVersion());
		SmartDashboard.putNumber("curSpd_cmd", curSpeed);
		_talon.set(curSpeed);
		SmartDashboard.putNumber("realSpeed ", _talon.getSpeed());
		SmartDashboard.putNumber("encPos (U)", _talon.getEncPosition());
		SmartDashboard.putNumber("err  (U)", _talon.getError());
		SmartDashboard.putNumber("CLerr  (U)", _talon.getClosedLoopError());
		//SmartDashboard.putInt("xxx", _talon.get);
		
		SmartDashboard.putNumber("iaccum  (U)", _talon.GetIaccum());
		SmartDashboard.putNumber("get (rpm)", _talon.get() );    
		SmartDashboard.putNumber("i ", _talon.getOutputCurrent());
		SmartDashboard.putNumber("V ", _talon.getOutputVoltage());	
		SmartDashboard.putNumber("codestate ", this.code_state);
		
	}
}
