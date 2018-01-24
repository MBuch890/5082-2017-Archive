package org.usfirst.frc.team5082.robot;

import java.sql.Time;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDOutput;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.AnalogInput;


public class Robot extends IterativeRobot implements PIDOutput {
	AHRS ahrs;
	RobotDrive topCim;
	RobotDrive midCim;
	RobotDrive lowCim;
	Joystick stick;
	int autoLoopCounter;
	DoubleSolenoid transSolenoid;
	DoubleSolenoid transSolenoid2;
	private int defense = 0;// initialize default mode;
	String autoSelected;
	ADXRS450_Gyro gyro;
	DigitalInput limitSwitch;
	int count;
	SendableChooser<Integer> chooser = new SendableChooser<Integer>();
	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	private VisionThread visionThread;
	double airpressure;
	Spark rope2;
	Talon agitator;
	public AnalogInput analogPsi;
	private double centerX = 0.0;	
	Timer timer = new Timer();
	static int finishAuto = 0;
	private final Object imgLock = new Object();
	Spark Rope;//
	//Servo ballServo;
	int buttonToggle;
	Encoder enc, enc2;
	PIDController turnController;
    double rotateToAngleRate;
    Preferences prefs;
    static final double kI = 0.009844; // Ti = Tu/2.5 = .192, Kp/Ti*dt = 0.009844
    static final double kD = 0.354;     // Kd = Kp*3*Tu/20/dt = 3*0.48/20/.02*kP = 3.6*kP
    static final double kF = 0.00;
    static final double kP = .7*0.135; //kU  .135, Tu = 0.48 sec, dt = 20 msec, Kp = 0.7*Ku
//
    static final double kToleranceDegrees = 2.0f;    
    static final double bias = 0.00; // built in turn to overcome frame
                      // used 0.5 to straighten drive at full speed
    Spark motorTopRight;
    Spark motorMiddleRight;
    Spark motorBottomRight;
    Spark motorTopLeft;
    Spark motorMiddleLeft;
    Spark motorBottomLeft;
    Talon shooter;
    
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		gyro = new ADXRS450_Gyro();
    	motorTopLeft = new Spark(1);
        motorMiddleLeft = new Spark(6);
        motorBottomLeft = new Spark(3);
        motorTopRight = new Spark(5);
        motorMiddleRight = new Spark(2);
        motorBottomRight = new Spark(4);
    	topCim =  new RobotDrive(motorTopLeft,motorTopRight); //change these values to assign motors
    	midCim = new RobotDrive(motorMiddleLeft,motorMiddleRight);
    	lowCim = new RobotDrive(motorBottomLeft,motorBottomRight);
    	enc = new Encoder(2, 3, false, Encoder.EncodingType.k2X); //change the 2x to 4x if the encoder still isn't working properly
    	enc2 = new Encoder(4, 5, false, Encoder.EncodingType.k2X); //change the 2x to 4x if the encoder still isn't working properly
    	transSolenoid = new DoubleSolenoid(1,0);
    	transSolenoid2 = new DoubleSolenoid(3,2);
    	//ballServo = new Servo(9);
    	limitSwitch = new DigitalInput(1);
    	analogPsi = new AnalogInput(0);
    	stick = new Joystick(0);
    	gyro.calibrate();
    	gyro.reset();
        prefs = Preferences.getInstance();
    	buttonToggle = 2;
        try {
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
    	LiveWindow.addActuator("DriveSystem", "RotateController", turnController); 
    	Rope = new Spark(8);
    	rope2 = new Spark(9);
    	shooter = new Talon(7);
    	agitator = new Talon(0);
    	SmartDashboard.putData("Autonomous Mode Selector: ", chooser);
    	chooser.addDefault("Blue Shoot Then Drive", 1);
    	chooser.addObject("Red Shoot Then Drive", 2);
    	chooser.addObject("Position 3", 3);
    	chooser.addObject("Drive Straight", 4);
    	chooser.addObject("Center Gear Drop", 5);
    	chooser.addObject("None", 6);
	    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    visionThread = new VisionThread(camera, new pipeline(), pipeline -> {
	        if (!pipeline.findContoursOutput().isEmpty()) {
	            Rect r = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
	            synchronized (imgLock) {
	                centerX = r.x + (r.width / 2);
	            }
	        }
	    });
	    visionThread.start();
	    gyro.calibrate();
	    enc2.reset();
	    enc.reset();
	    topCim.setSafetyEnabled(false);
	    midCim.setSafetyEnabled(false);
	    lowCim.setSafetyEnabled(false);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		defense = (int) chooser.getSelected();
		timer.reset();
		timer.start();
	}
//
	//
	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
        int finish = 0;
		double distance = enc.get() / 1440;
        double adjustment;
        double difference; // angle error - previous angle error
        double sum; // error added over time
        double lastAngle;
        double angle;
		enc2.reset();
		double shootTime=5.0;
		switch(defense) {
    	case 1:
    		
    		timer.start();
    		
    		shooter.set(1.0);
    		timer.delay(shootTime);
    		shooter.set(0.0);
    		
    		enc.reset();
			 enc2.reset();
			 gyro.reset();
			
             sum=0;
             lastAngle=0;
			  do 
			  {
			   distance = enc2.get()/180;
			   if (timer.get() <= shootTime+1.5) 
			    {
			      angle = gyro.getAngle();
			      difference=angle-lastAngle;
			      sum=sum+angle;
			      lastAngle=angle;
			      adjustment = kP*angle + difference*kD + sum*kI  ;
			      topCim.arcadeDrive(-0.75, -adjustment); // drive towards heading 0
	              midCim.arcadeDrive(-0.75, -adjustment); // drive towards heading 0
	              lowCim.arcadeDrive(-0.75, -adjustment); 
	              
	              SmartDashboard.putNumber("Gyroscopic Autonomous Angle: ", gyro.getAngle()); 
        	      SmartDashboard.putNumber("Encoder 2 Get Distance :", enc2.getDistance());
        	      SmartDashboard.putNumber("Encoder 1 Get Distance :", enc.getDistance());
        	      SmartDashboard.putNumber("Calculated Distance :", distance); 
        	      SmartDashboard.putNumber("Time :", timer.get());
        	      System.out.printf("time: %6.3f angle: %6.3f  ",timer.get(), angle);
        	      timer.delay(0.016); // raise loop time to 20 msec
        	     
			    }
			  else
        	    {	  finish = 1;
        	    };
			  } while (finish == 0);
			  topCim.drive(0.0, 0.0);
	    	  midCim.drive(0.0, 0.0);
	    	  lowCim.drive(0.0, 0.0);
        		finishAuto = 1;//
        		
    		
    		
			break;
		case 2:									//DO NOT GO(DRIVE)
			
    		timer.start();
    		
    		shooter.set(1.0);
    		timer.delay(shootTime);
    		shooter.set(0.0);
    		
    		enc.reset();
			 enc2.reset();
			 gyro.reset();
			
             sum=0;
             lastAngle=0;
			  do 
			  {
			   distance = enc2.get()/180;
			   if (timer.get() <= shootTime+1.5) 
			    {
			      angle = gyro.getAngle();
			      difference=angle-lastAngle;
			      sum=sum+angle;
			      lastAngle=angle;
			      adjustment = kP*angle + difference*kD + sum*kI  ;
			      topCim.arcadeDrive(0.75, -adjustment); // drive towards heading 0
	              midCim.arcadeDrive(0.75, -adjustment); // drive towards heading 0
	              lowCim.arcadeDrive(0.75, -adjustment); 
	              
	              SmartDashboard.putNumber("Gyroscopic Autonomous Angle: ", gyro.getAngle()); 
        	      SmartDashboard.putNumber("Encoder 2 Get Distance :", enc2.getDistance());
        	      SmartDashboard.putNumber("Encoder 1 Get Distance :", enc.getDistance());
        	      SmartDashboard.putNumber("Calculated Distance :", distance); 
        	      SmartDashboard.putNumber("Time :", timer.get());
        	      System.out.printf("time: %6.3f angle: %6.3f  ",timer.get(), angle);
        	      timer.delay(0.016); // raise loop time to 20 msec
        	     
			    }
			  else
        	    {	  finish = 1;
        	    };
			  } while (finish == 0);
			  topCim.drive(0.0, 0.0);
	    	  midCim.drive(0.0, 0.0);
	    	  lowCim.drive(0.0, 0.0);
        		finishAuto = 1;
				break;
	/*	case 3:
			topCim.drive(0.75, 0.0);
    		midCim.drive(0.75, 0.0);
    		lowCim.drive(0.75, 0.0);
    		do {
    			distance = distance;
    			if (distance == 10) {
    				ahrs.zeroYaw();
    				topCim.drive(0.0, 0.0);
    				midCim.drive(0.0, 0.0);
    				lowCim.drive(0.0, 0.0);
    				Timer.delay(0.5);
					turnController.setSetpoint(270.0f);
            		turnController.enable();
            		double currentRotationRate = rotateToAngleRate;
            		topCim.arcadeDrive(0, currentRotationRate);
            		Timer.delay(1.0); 
    			}
    			else if (distance == 15) {
    				topCim.drive(0.0, 0.0);
        			midCim.drive(0.0, 0.0);
        			lowCim.drive(0.0, 0.0);
    				transSolenoid2.set(DoubleSolenoid.Value.kReverse); //inwards
    				Timer.delay(1.00);
    				transSolenoid2.set(DoubleSolenoid.Value.kForward); //inwards
    				finish = 1;
    			}
    		} while (finish == 0);
			break; */ //
		case 4:
			
			if (finishAuto == 0) // only run one time
			{ 
				enc.reset();
				enc2.reset();
				gyro.reset();
				timer.start();
				sum=0;
				lastAngle=0;
				do {
					distance = enc2.get() / 180;
					if (timer.get() <= 2.5) {
						angle = gyro.getAngle();
						difference=angle-lastAngle;
						sum=sum+angle;
						lastAngle=angle;
						adjustment = kP*angle + difference*kD + sum*kI  ;
						topCim.arcadeDrive(-0.75, -adjustment); // drive towards heading 0
						midCim.arcadeDrive(-0.75, -adjustment); // drive towards heading 0
						lowCim.arcadeDrive(-0.75, -adjustment); 
	              
						SmartDashboard.putNumber("Gyroscopic Autonomous Angle: ", gyro.getAngle()); 
						SmartDashboard.putNumber("Encoder 2 Get Distance :", enc2.getDistance());
						SmartDashboard.putNumber("Encoder 1 Get Distance :", enc.getDistance());
						SmartDashboard.putNumber("Calculated Distance :", distance); 
						SmartDashboard.putNumber("Time :", timer.get());
						System.out.printf("time: %6.3f angle: %6.3f  ",timer.get(), angle);
						timer.delay(0.016); // raise loop time to 20 msec		
					}
					else {
						finish = 1;
					};
				} while (finish == 0);
				topCim.drive(0.0, 0.0);
				midCim.drive(0.0, 0.0);
				lowCim.drive(0.0, 0.0);
				finishAuto = 1;} // don't run this auto a second time.
				break;
		case 5:
			if (finishAuto == 0) // only run one time
			{ 
				enc.reset();
				enc2.reset();
				gyro.reset();
				timer.start();
               
				do {
					distance = enc2.get() / 180;
					if (timer.get() <= 1.30) {
						angle = gyro.getAngle();
						adjustment = kP*angle;
						topCim.arcadeDrive(-0.75, -adjustment); // drive towards heading 0
						midCim.arcadeDrive(-0.75, -adjustment); // drive towards heading 0
						lowCim.arcadeDrive(-0.75, -adjustment); 
						SmartDashboard.putNumber("Gyroscopic Autonomous Angle: ", gyro.getAngle()); 
						SmartDashboard.putNumber("Encoder 2 Get Distance :", enc2.getDistance());
						SmartDashboard.putNumber("Encoder 1 Get Distance :", enc.getDistance());
						SmartDashboard.putNumber("Calculated Distance :", distance); 
						SmartDashboard.putNumber("Time :", timer.get());
						System.out.printf("time: %6.3f angle: %6.3f  ",timer.get(), angle);
						timer.delay(0.016);
					}
					else                     
					{	  finish = 1;
					};                         //
	           
				} while (finish == 0);
				topCim.drive(0.0, 0.0);
				midCim.drive(0.0, 0.0);
				lowCim.drive(0.0, 0.0);
				Timer.delay(.5);
				transSolenoid2.set(DoubleSolenoid.Value.kForward);; 
				Timer.delay(1);
				topCim.drive(0.6, 0.0);
				midCim.drive(0.6, 0.0);
				lowCim.drive(0.6, 0.0);
				Timer.delay(.6);
				topCim.drive(0.0, 0.0);
				midCim.drive(0.0, 0.0);
				lowCim.drive(0.0, 0.0);
				transSolenoid2.set(DoubleSolenoid.Value.kReverse);; 
				Timer.delay(1);
				finishAuto = 1; // don't run this auto a second time.
				}
			break;
		case 6:
			System.out.println("No Movement");
			break;
		}
		Timer.delay(4);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putNumber("Gyroscopic Realtime Angle: ", gyro.getAngle());
		SmartDashboard.putNumber("Gyroscopic Realtime Rate: ", gyro.getRate());
		SmartDashboard.putNumber("Encoder 2 Rate of Change: ", enc2.getRate());
		SmartDashboard.putNumber("Encoder 2 Measured Distance :", enc2.getDistance());
		SmartDashboard.putNumber("Encoder 1 Measured Distance :", enc.getDistance());
		SmartDashboard.putNumber("Encoder 1 Rate of Change: ", enc.getRate());
		SmartDashboard.putNumber("Pressure Sensor Value: ", 250.0 * analogPsi.getVoltage() / 5.0 - 25.0);
		int reset = 0;
		double modBias;  // adjusted bias value for speed
		double speedStick;   // reading on the joystick for speed
		topCim.setSafetyEnabled(true);
		midCim.setSafetyEnabled(true);
		lowCim.setSafetyEnabled(true);
		gyro.reset();
		topCim.setMaxOutput(0.9);
		midCim.setMaxOutput(0.9);
		lowCim.setMaxOutput(0.9);
		if (stick.getRawButton(7)) {
			buttonToggle = 1;         // please define this as "forward" or "reverse"
			System.out.print(buttonToggle);
		}
		if (stick.getRawButton(8)) {
			buttonToggle = 2;            // please define this as "forward" or "reverse"
			System.out.print(buttonToggle);
		}
		if (stick.getRawButton(10)) {
			System.out.print(buttonToggle);
		}
		if (buttonToggle == 2) {
			speedStick = stick.getRawAxis(1);	
			modBias = speedStick*bias;
			if (speedStick<0) {
				modBias = modBias*-1;
			}
			topCim.arcadeDrive(speedStick*-1, stick.getRawAxis(4)+modBias); // if not working change
			midCim.arcadeDrive(speedStick*-1, stick.getRawAxis(4)+modBias); // to -modBias
			lowCim.arcadeDrive(speedStick*-1, stick.getRawAxis(4)+modBias);
			if (Math.abs(stick.getRawAxis(2)) > .15)  {
				if (reset < 1) {
					gyro.reset();
					reset++;
					System.out.println("Success! Gyro was reset.");
				}
				double angle = gyro.getAngle() * kP; // get current heading
				SmartDashboard.putNumber("Gyroscopic Realtime Angle:", gyro.getAngle());
				topCim.arcadeDrive(stick.getRawAxis(2)*1, -angle); // drive towards heading 0
				midCim.arcadeDrive(stick.getRawAxis(2)*1, -angle); // drive towards heading 0
				lowCim.arcadeDrive(stick.getRawAxis(2)*-1, angle);
			}	
			if (Math.abs(stick.getRawAxis(3)) > .15)  {
				if (reset < 1) {
					gyro.reset();
					reset++;
					System.out.println("Success! Gyro was reset.");
				}
				double angle = gyro.getAngle() * kP; // get current heading
				SmartDashboard.putNumber("Gyroscopic Realtime Angle:", gyro.getAngle());
				topCim.arcadeDrive(stick.getRawAxis(3)*1, angle); // drive towards heading 0
				midCim.arcadeDrive(stick.getRawAxis(3)*1, angle); // drive towards heading 0
				lowCim.arcadeDrive(stick.getRawAxis(3)*-1, -angle);
			}
		}
		else if (buttonToggle == 1) {
			speedStick = stick.getRawAxis(1);	// bias is a constant turn
		    modBias = speedStick*bias;         // adjust bias, more at high speed
		    if (speedStick<0) {
		    	modBias = modBias*-1;
		    }
			
			topCim.arcadeDrive(stick.getRawAxis(1)*1, stick.getRawAxis(4)-modBias); // if not working, change
			midCim.arcadeDrive(stick.getRawAxis(1)*1, stick.getRawAxis(4)-modBias); // to +modBias
			lowCim.arcadeDrive(stick.getRawAxis(1)*1, stick.getRawAxis(4)-modBias);
	        if (Math.abs(stick.getRawAxis(2)) > .1)  {
	        	if (reset < 1) {
					gyro.reset();
					reset++;
					System.out.println("Success! Gyro was reset.");
				}
	        	double angle = gyro.getAngle() * kP; // get current heading
	        	SmartDashboard.putNumber("Gyroscopic Realtime Angle:", gyro.getAngle());
	            topCim.arcadeDrive(stick.getRawAxis(2)*1, angle); // drive towards heading 0
	            midCim.arcadeDrive(stick.getRawAxis(2)*1, angle); // drive towards heading 0
	            lowCim.arcadeDrive(stick.getRawAxis(2)*-1, -angle);
	        }
	        if (Math.abs(stick.getRawAxis(3)) > .1)  {
	        	if (reset < 1) {
					gyro.reset();
					reset++;
					System.out.println("Success! Gyro was reset.");
				}
	        	double angle = gyro.getAngle() * kP; // get current heading
	        	SmartDashboard.putNumber("Gyroscopic Realtime Angle:", gyro.getAngle());
	            topCim.arcadeDrive(stick.getRawAxis(3)*-1, -angle); // drive towards heading 0
	            midCim.arcadeDrive(stick.getRawAxis(3)*-1, -angle); // drive towards heading 0
	            lowCim.arcadeDrive(stick.getRawAxis(3)*1, angle);

	        }
		}
		
        if (stick.getRawButton(6)) { //refers to buttons numbered on top of the joyStick.
        	transSolenoid.set(DoubleSolenoid.Value.kReverse);; //inwards
        }
        else if (stick.getRawButton(5)) {
        	transSolenoid.set(DoubleSolenoid.Value.kForward);; //outwards
        }
        else{
        	transSolenoid.set(DoubleSolenoid.Value.kOff);; //outwards
        }
        
        
        
        if (stick.getRawButton(1)) { //refers to buttons numbered on top of the joyStick.
        	transSolenoid2.set(DoubleSolenoid.Value.kForward);; //inwards
        }
        else{
        	transSolenoid2.set(DoubleSolenoid.Value.kReverse);; //outwards
        }
        
        
        
        if (stick.getRawButton(3)){
    		Rope.set(-1.00);
    		rope2.set(1.00);
        }
        else if (stick.getRawButton(4)) {
        	Rope.set(1.00);
        	rope2.set(-1.00);
        }
        else {
        	Rope.set(0.00);
        	rope2.set(0.00);
        }
        
        
        
        if (stick.getRawButton(2)) {
        	shooter.set(1.0);
        	agitator.set(-0.5);
        }
        else shooter.set(0.0);
        	 agitator.set(0.0);
	    }
        	
     //   if (stick.getRawButton(2)) {
      //  	ballServo.set(1.0);
     //   }
      //  else {
        //		ballServo.set(0.0);
        //	}
        //if (limitSwitch.equals(true)) {
        //	System.out.println("Limit Switch Working");
        //	Timer.delay(1);
        //   }


	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	 public void pidWrite(double output) {
	        rotateToAngleRate = output;
	    }
}

