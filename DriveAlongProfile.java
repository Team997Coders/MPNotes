import org.usfirst.frc.team135.robot.Robot;
import org.usfirst.frc.team135.robot.RobotMap;
import org.usfirst.frc.team135.robot.RobotMap.COMPETITION.DRIVETRAIN;
import org.usfirst.frc.team135.robot.util.PIDIn;
import org.usfirst.frc.team135.robot.util.PIDOut;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
/**
 *
 */
public class DriveAlongProfile extends Command implements RobotMap {

	private Waypoint[] _points;
	private Trajectory _leftTrajectory, _rightTrajectory;
	private EncoderFollower _leftEncoderFollower, _rightEncoderFollower;
	
	private static final Trajectory.FitMethod _FIT_METHOD = Trajectory.FitMethod.HERMITE_CUBIC;

	private static final int _SAMPLES = Trajectory.Config.SAMPLES_HIGH;
	
	private static final double
		_TIMESTEP = 0.05, //In seconds
		_MAX_VELOCITY = DRIVETRAIN.MAX_VELOCITY_TICKS * CONVERSIONS.TICKS2METERS,
		_MAX_ACCELERATION = DRIVETRAIN.MAX_ACCELERATION_TICKS * CONVERSIONS.TICKS2METERS,
		_MAX_JERK = DRIVETRAIN.MAX_JERK_TICKS * CONVERSIONS.TICKS2METERS;
	
	private static final double _VELOCITY_RATIO = 1 / _MAX_VELOCITY;
	
	private static final double 
		_drivekP = 1.0,
		_drivekI = 0.01,
		_drivekD = 10.0,
		_drivekA = 0; //Acceleration gain. Tweak this if you want to go to accelerate faster.
	
	private PIDController _angleController;
	private PIDOut _buffer;
	private PIDIn _pidSource;
	
	private static final double 
		_turnkP = .267,
		_turnkI = 0.00267,
		_turnkD = 2.67,
		_turnkF = 0; //There might be some error causing real world process we can take out with kF
	
	private static final double _TRACK_WIDTH = DRIVETRAIN.TRACK_WIDTH * CONVERSIONS.INCHES2METERS; //Jaci calls this wheelbase width
	
	private Timer _timer;
	private double _timeout = 5.0;
	
	private boolean _done = false;
	
    public DriveAlongProfile(Waypoint[] points, double timeout) {
    	requires(Robot.drivetrain);
  
    	this._timer = new Timer();
    	this._timeout = timeout;
    	
    	this._points = points.clone(); //Don't want a reference of points, I want a copy of it
    	
    	this._buffer = new PIDOut();
    	this._pidSource = new PIDIn(() -> Robot.navx.getFusedAngle(), PIDSourceType.kDisplacement);
    	
		this._angleController = new PIDController(DriveAlongProfile._turnkP, 
													DriveAlongProfile._turnkI,
													DriveAlongProfile._turnkD, 
													DriveAlongProfile._turnkF, 
													this._pidSource, 
													this._buffer);

		Trajectory.Config baseTrajectoryConfig = new Trajectory.Config(DriveAlongProfile._FIT_METHOD,
																		DriveAlongProfile._SAMPLES, 
																		DriveAlongProfile._TIMESTEP, 
																		DriveAlongProfile._MAX_VELOCITY,
																		DriveAlongProfile._MAX_ACCELERATION, 
																		DriveAlongProfile._MAX_JERK);

		Trajectory baseTrajectory = Pathfinder.generate(this._points, baseTrajectoryConfig);

		TankModifier modifier = new TankModifier(baseTrajectory);

		modifier.modify(DriveAlongProfile._TRACK_WIDTH);

		this._leftTrajectory = modifier.getLeftTrajectory();
		this._rightTrajectory = modifier.getRightTrajectory();

		this._leftEncoderFollower = new EncoderFollower(this._leftTrajectory);
		this._rightEncoderFollower = new EncoderFollower(this._rightTrajectory);

    }

    // Called just before this Command runs the first time
    protected void initialize() {    	
    	
    	this._leftEncoderFollower.configureEncoder((int)Robot.drivetrain.getEncoderCounts(Robot.drivetrain.rearLeftTalon), 
													(int)CONVERSIONS.REVS2TICKS, 
													DRIVETRAIN.WHEEL_DIAMETER * CONVERSIONS.INCHES2METERS);  
    	
    	this._rightEncoderFollower.configureEncoder((int)Robot.drivetrain.getEncoderCounts(Robot.drivetrain.rearRightTalon), 
													(int)CONVERSIONS.REVS2TICKS, 
													DRIVETRAIN.WHEEL_DIAMETER * CONVERSIONS.INCHES2METERS);
    	
    	this._leftEncoderFollower.configurePIDVA(DriveAlongProfile._drivekP, 
													DriveAlongProfile._drivekI, 
													DriveAlongProfile._drivekD, 
													DriveAlongProfile._VELOCITY_RATIO, 
													DriveAlongProfile._drivekA);
    	
    	this._rightEncoderFollower.configurePIDVA(DriveAlongProfile._drivekP, 
													DriveAlongProfile._drivekI, 
													DriveAlongProfile._drivekD, 
													DriveAlongProfile._VELOCITY_RATIO, 
													DriveAlongProfile._drivekA);
    	
    	this._angleController.setInputRange(0, 360);
    	this._angleController.setContinuous(true);
    	this._angleController.setOutputRange(-.5, .5);
    	this._angleController.setAbsoluteTolerance(.2);

    	this._angleController.enable();
    	this._timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	double left = this._leftEncoderFollower.calculate((int) Robot.drivetrain.getEncoderCounts(Robot.drivetrain.frontLeftTalon));
    	double right = this._rightEncoderFollower.calculate((int) Robot.drivetrain.getEncoderCounts(Robot.drivetrain.frontRightTalon));
    	double heading = (this._leftEncoderFollower.getHeading() + this._rightEncoderFollower.getHeading()) / 2;
    	
    	this._angleController.setSetpoint(Pathfinder.r2d(heading) % 360);
    	
    	if (this._timer.get() >= this._timeout)
    	{
    		this._done = true;
    		return;
    	} 	
    	else if (left == 0 && right == 0 && this._buffer.output == 0)
    	{
    		this._done = true;
    		return;
    	}
    	
    	left += this._buffer.output;
    	right -= this._buffer.output;
    	
    	Robot.drivetrain.driveTank(left, right);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return this._done;
    }

    // Called once after isFinished returns true
    protected void end() 
    {
    	Robot.drivetrain.stopMotors();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() 
    {
    	this.end();
    }
}
