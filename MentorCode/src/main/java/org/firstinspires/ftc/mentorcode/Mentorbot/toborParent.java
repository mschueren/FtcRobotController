package org.firstinspires.ftc.mentorcode.Mentorbot;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.mentorcode.Mentorbot.toborVuforia;


@TeleOp(name = "ToborParent", group = "NSR")
@Disabled
public abstract class toborParent extends OpMode {

    //all distances in in millimeters, all angles in degrees, all time in seconds

    final int topOfLift = 13400;
    final double wheelDiameter = 101.6;//diameter of the robot wheels
    final double triggerThreshold = .5;//threshold for pushing a gamepad trigger
    final double degreeTurningTolerance = 4;//accuracy tolerance for turns
    static double[] tempPowers = new double[4];//double array to store temporary motor powers
    final double ballSensingThreshold = .26; //2.5
    //declaration of all of the hardware variables
    DcMotor shooter;
    DcMotor collector;
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor liftLeft;
    DcMotor liftRight;
    DcMotor greenLED;
    DcMotor redLED;
    Servo buttonRight;
    Servo buttonLeft;
    Servo blocker;
    Servo aim;
    Servo ballGrabber;
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor beaconColor;
    ColorSensor lineSensorBack;
    AnalogInput lineSensorFront;
    ModernRoboticsI2cGyro gyroSensor;
    AnalogInput ballSensor;
    //numbers to keep track of the color sensor data while reading the beacon
    int blueTotal;
    int redTotal;

    //two timers to be used indipendantly by the autonomous programs
    ElapsedTime timer;
    ElapsedTime tempTimer;
    ElapsedTime thirdTimer;

    @Override
    public void init() {
        redLED = hardwareMap.dcMotor.get("red");
        greenLED = hardwareMap.dcMotor.get("green");

        //assignment of all hardware items from the phone's hardware map
        shooter = hardwareMap.dcMotor.get("shooter");
        collector = hardwareMap.dcMotor.get("collector");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("liftRight");
        buttonRight = hardwareMap.servo.get("buttonRight");
        buttonLeft = hardwareMap.servo.get("buttonLeft");
        aim = hardwareMap.servo.get("aim");
        blocker = hardwareMap.servo.get("blocker");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        beaconColor = hardwareMap.colorSensor.get("beaconColor");
        lineSensorBack = hardwareMap.colorSensor.get("lineSensorBack");
        lineSensorFront = hardwareMap.analogInput.get("lineSensorFront");
        gyroSensor = (ModernRoboticsI2cGyro)hardwareMap.get("gyro");
        ballSensor = hardwareMap.analogInput.get("ballSensor");
        ballGrabber = hardwareMap.servo.get("grabber");
        //set all of the drive motors to run with encoders (activate PID control)
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //reverse some motor to account for mirroring
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //run the lift with encoders to force the two motors to travel at the same speed
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set the I2C addresses of the color sensors
        beaconColor.setI2cAddress(I2cAddr.create8bit(0x42));
        lineSensorBack.setI2cAddress(I2cAddr.create8bit(0x3c));
        beaconColor.enableLed(false);
        //set the resolution of the timers
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        tempTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        thirdTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        moveBlocker(false);
        moveAimServo(true);
        moveGrabber(false);
    }
    @Override
    public void init_loop()
    {

        if (liftRight.getMode()!=DcMotor.RunMode.RUN_TO_POSITION||liftLeft.getMode()!=DcMotor.RunMode.RUN_TO_POSITION)
        {
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (frontRight.getMode()!=DcMotor.RunMode.RUN_USING_ENCODER||backRight.getMode()!=DcMotor.RunMode.RUN_USING_ENCODER||frontLeft.getMode()!=DcMotor.RunMode.RUN_USING_ENCODER||backLeft.getMode()!=DcMotor.RunMode.RUN_USING_ENCODER)
        {
            setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        timer.reset();
            //reset the timers
        //tempTimer.reset();
        //continuously calibrate the gyro sensor during the time between pressing init and
        //starting the program.
    }
    void setGreenLED(boolean on)
    {
        greenLED.setPower(on?.9:0);
    }
    void setRedLED(boolean on)
    {
        redLED.setPower(on?.75:0);
    }
    void moveGrabber(boolean isGrabbing)
    {
        ballGrabber.setPosition(isGrabbing?0:.999);
    }
    boolean isAtPosition(DcMotor motor, int tolerance)
    {
        return Math.abs(motor.getCurrentPosition()-motor.getTargetPosition())<=tolerance;
    }
    boolean isAtPosition(DcMotor motor1,DcMotor motor2, int tolerance)
    {
        return isAtPosition(motor1,tolerance)&&isAtPosition(motor2,tolerance);
    }
    public boolean ballInShooter()
    {
        return ballSensor.getVoltage()<=ballSensingThreshold;
    }
    public double shooterPosition()
    {
        return (shooter.getCurrentPosition())%1440;
    }
    public void setAllMotorModes(DcMotor.RunMode mode)
    {
        //simplification method to allow the modes of all motors to be set in one line
        frontRight.setMode(mode);
        frontLeft.setMode(mode);
        backRight.setMode(mode);
        backLeft.setMode(mode);
    }
    public int averageEncoderCount()
    {
        //return the average encoder count of all four wheels
        return (frontRight.getCurrentPosition() + frontLeft.getCurrentPosition() + backRight.getCurrentPosition() + backLeft.getCurrentPosition()) / 4;
    }
    public double encoderToDistance(int encoderCounts)
    {
        //method to convert encoder counts to millimeter distance
        return ((encoderCounts/Math.sqrt(2))/1120)*(wheelDiameter*Math.PI);
    }
    public void moveBlocker(boolean raise)
    {
        if (raise)
        {
            blocker.setPosition(.600);
        }
        else
        {
            blocker.setPosition(.230);
        }
    }
    public void moveAimServo(boolean vertical)
    {
        if (vertical)
        {
            aim.setPosition(.61);
        }
        else
        {
            aim.setPosition(.999);
        }
    }
    public void moveButtonServo(boolean raise, boolean isRight)
    {
        //single method to handle the movement of button servos based on two boolean inputs
        if (raise&&!isRight)
        {
            buttonLeft.setPosition(.314);
        }
        else if (raise&&isRight)
        {
            buttonRight.setPosition(.824);
        }
        else if (!raise&&!isRight)
        {
            buttonLeft.setPosition(.882);
        }
        else
        {
            buttonRight.setPosition(.275);
        }
    }
    //launch a ball
    //read the red and blue values from the beacon and add them to the totals
    public void readBeacon()
    {
        redTotal += beaconColor.red();
        blueTotal += beaconColor.blue();
    }
    //set all drive motors to the same power
    public void driveStraight(double power)
    {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }
    //set all of the motor powers to the given array of values
    public void setMotorPowers(double[] values)
    {
        frontRight.setPower(values[0]);
        backRight.setPower(values[1]);
        frontLeft.setPower(values[2]);
        backLeft.setPower(values[3]);
    }
    public double[] getMotorPowers()
    {
        return new double[] {frontRight.getPower(),backRight.getPower(),frontLeft.getPower(),backLeft.getPower()};
    }
    //retun an array of motor powers for sideways mechanum movement in a given direction
    //at a given speed
    public double[] getMechanumPowers(boolean isRight, double power)
    {
        if (isRight)
        {
            return new double[] {-power,power,power,-power};
        }
        else
        {
            return new double[] {power,-power,-power,power};
        }
    }
    //slide to the left using the mechanum wheels
    public void mechanumLeft(double power)
    {
        frontRight.setPower(power);
        backLeft.setPower(power);
        frontLeft.setPower(-power);
        backRight.setPower(-power);
    }
    //slide to the right using the mechanum wheels
    public void angleAdjustedMechanumMovement(boolean isRight, double power, int angle, int distanceFromWall)
    {
        tempPowers = getMechanumPowers(isRight,power);
            if (gyroHeading() <angle) {
                tempPowers[0] += .1;
                tempPowers[1] += .1;
            } else if (gyroHeading() >angle) {
                tempPowers[2] += .1;
                tempPowers[3] += .1;
            }
            if (rangeSensor.getDistance(DistanceUnit.MM) <= distanceFromWall) {
                tempPowers[0] += .1;
                tempPowers[1] += .1;
                tempPowers[2] += .1;
                tempPowers[3] += .1;
            } else {
                tempPowers[0] += -.1;
                tempPowers[1] += -.1;
                tempPowers[2] += -.1;
                tempPowers[3] += -.1;
            }
            setMotorPowers(tempPowers);
    }
    public void mechanumRight(double power)
    {
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        frontLeft.setPower(power);
        backRight.setPower(power);
    }
    //stop all drive motors
    public void stopDrive()
    {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    //turn based on a given set of parameters. isRight is self explanatory, backwards handles
    //"back first" movement, the power of the turn, and a boolean to indicate whether to carry
    //out a pivot turn or a sweep turn
    public double[] getPivotValues(boolean isRight, double power)
    {
       return new double[] {isRight?-1:1*power,isRight?-1:1*power,isRight?1:-1*power,isRight?1:-1*power};
    }
    public void turn(boolean isRight, boolean backwards, double power, boolean pivot)
    {
        if (backwards) {
            if (isRight && pivot) {
                frontLeft.setPower(power);
                backLeft.setPower(power);
                frontRight.setPower(-power);
                backRight.setPower(-power);
            } else if (isRight && !pivot) {
                frontLeft.setPower(power);
                backLeft.setPower(power);
                frontRight.setPower(.65 * power);
                backRight.setPower(.65 * power);
            } else if (!isRight && pivot) {
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
                frontRight.setPower(power);
                backRight.setPower(power);
            } else {
                frontRight.setPower(power);
                backRight.setPower(power);
                frontLeft.setPower(.65 * power);
                backLeft.setPower(.65 * power);
            }
        }
        else
        {
            if (!isRight&&pivot)
            {
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
                frontRight.setPower(power);
                backRight.setPower(power);
            }
            else if (!isRight&&!pivot)
            {
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
                frontRight.setPower(-.65*power);
                backRight.setPower(-.65*power);
            }
            else if (isRight&&pivot)
            {
                frontLeft.setPower(power);
                backLeft.setPower(power);
                frontRight.setPower(-power);
                backRight.setPower(-power);
            }
            else
            {
                frontRight.setPower(-power);
                backRight.setPower(-power);
                frontLeft.setPower(-.4*power);
                backLeft.setPower(-.4*power);
            }
        }
    }
    //definition of scaling method for the gyro sensor heading. Becuase we used a standardized
    //direction system on the field, the raw values from the gyro sensor need to be scaled to our
    //system, however, this operation is different for different starting positions, as the gyro
    //always initializes to zero. Marking the method abstract forces it to be overridden in any
    //derivative class, such as both of our autonomous programs.
    public abstract int gyroHeading();
    //return an array of values for the motor powers for a turn to a specific angle
    public double[] getTurnValues(double targetAngle, boolean useVuforia)
    {
        if (!isAtAngle(targetAngle,useVuforia))
        {
            //if the robot is not at the current angle, return values which indicate a turn
            return new double[]{
                    Math.signum(targetAngle - toborVuforia.getHeading()) * .3,
                    Math.signum(targetAngle - toborVuforia.getHeading()) * .3,
                    Math.signum(targetAngle - toborVuforia.getHeading()) * -.3,
                    Math.signum(targetAngle - toborVuforia.getHeading()) * -.3
            };
        } else
        {
            //if the robot is at the target angle, return 0 for all motors
            return new double[] {0,0,0,0};
        }
    }
    //check to see if the robot is at a given target angle. This method is necessary because of
    //the special case of angles close to the boundary between +180 and -180
    public boolean isAtAngle(double targetAngle, boolean useVuforia)
    {
        if (targetAngle<=-180+degreeTurningTolerance||targetAngle>=180-degreeTurningTolerance)
        {
            return Math.abs(Math.abs(targetAngle) - Math.abs(useVuforia?toborVuforia.getHeading():gyroHeading())) < degreeTurningTolerance;
        }
        else
        {
            return Math.abs(targetAngle - (useVuforia?toborVuforia.getHeading():gyroHeading())) < degreeTurningTolerance;
        }
    }
    //method to drive to any given coordinate on the field using vuforia for navigation
    public boolean driveToCoordinate(double[] targetLocation, boolean forwards)
    {
        //forwards specifies the difference between going back-first and going front-first
        if (forwards)
        {
            if (isAtAngle(0, true) && Math.abs(targetLocation[0] - toborVuforia.getLocation()[0]) <= 30 && Math.abs(targetLocation[1] - toborVuforia.getLocation()[1]) <= 30) {
                //if the robot is in the correct position and at the right angle, return true
                //to indicate that the operation is complete
                stopDrive();
                return true;
            } else {
                //assign the tempPowers array based on the required powers for turning to the
                //correct angle
                tempPowers = getTurnValues(0, true);
                tempPowers = multiplyToArray(tempPowers,.25);
                tempPowers[0] /= 4;
                tempPowers[1] /= 4;
                tempPowers[2] /= 4;
                tempPowers[3] /= 4;
                //check the x position of the robot and add the appropriate adjustment (a direct
                // forwards or backwards motion) to the array of motor values
                if (targetLocation[0] - toborVuforia.getLocation()[0] > 25) {
                    tempPowers[0] += .2;
                    tempPowers[1] += .2;
                    tempPowers[2] += .2;
                    tempPowers[3] += .2;
                } else if (targetLocation[0] - toborVuforia.getLocation()[0] < -25) {
                    tempPowers[0] += -.2;
                    tempPowers[1] += -.2;
                    tempPowers[2] += -.2;
                    tempPowers[3] += -.2;
                }
                //check the y position of the robot and add the appropriate adjustment (a mechanum
                //slid movement) to the array of motor values
                if (targetLocation[1] - toborVuforia.getLocation()[1] > 25) {
                    tempPowers[0] += getMechanumPowers(false, .1)[0];
                    tempPowers[1] += getMechanumPowers(false, .1)[1];
                    tempPowers[2] += getMechanumPowers(false, .1)[2];
                    tempPowers[3] += getMechanumPowers(false, .1)[3];
                } else if (targetLocation[1] - toborVuforia.getLocation()[1] < -25) {
                    tempPowers[0] += getMechanumPowers(true, .1)[0];
                    tempPowers[1] += getMechanumPowers(true, .1)[1];
                    tempPowers[2] += getMechanumPowers(true, .1)[2];
                    tempPowers[3] += getMechanumPowers(true, .1)[3];
                }
                tempPowers[0] /=1.7;
                tempPowers[1] /=1.7;
                tempPowers[2] /=1.7;
                tempPowers[3] /=1.7;
                //actually set the motor powers
                setMotorPowers(tempPowers);
                return false;
            }
        }
        else
        {
            //handle the case of driving back-first. All of the methods are equivalent
            double adjustedTargetAngle = headingBetweenCoordinates(toborVuforia.getLocation(), targetLocation);
            if (isAtAngle(-90, true) && Math.abs(targetLocation[0] - toborVuforia.getLocation()[0]) <= 30 && Math.abs(targetLocation[1] - toborVuforia.getLocation()[1]) <= 30) {
                stopDrive();
                return true;
            } else {
                tempPowers = getTurnValues(-90, true);
                tempPowers[0] /= 4;
                tempPowers[1] /= 4;
                tempPowers[2] /= 4;
                tempPowers[3] /= 4;

                if (targetLocation[1] - toborVuforia.getLocation()[1] > 25) {
                    tempPowers[0] += -.2;
                    tempPowers[1] += -.2;
                    tempPowers[2] += -.2;
                    tempPowers[3] += -.2;
                } else if (targetLocation[1] - toborVuforia.getLocation()[1] < -25) {
                    tempPowers[0] += .2;
                    tempPowers[1] += .2;
                    tempPowers[2] += .2;
                    tempPowers[3] += .2;
                }

                if (targetLocation[0] - toborVuforia.getLocation()[0] > 25) {
                    tempPowers[0] += getMechanumPowers(false, .1)[0];
                    tempPowers[1] += getMechanumPowers(false, .1)[1];
                    tempPowers[2] += getMechanumPowers(false, .1)[2];
                    tempPowers[3] += getMechanumPowers(false, .1)[3];
                } else if (targetLocation[0] - toborVuforia.getLocation()[0] < -25) {
                    tempPowers[0] += getMechanumPowers(true, .1)[0];
                    tempPowers[1] += getMechanumPowers(true, .1)[1];
                    tempPowers[2] += getMechanumPowers(true, .1)[2];
                    tempPowers[3] += getMechanumPowers(true, .1)[3];
                }
                tempPowers[0] /=1.7;
                tempPowers[1] /=1.7;
                tempPowers[2] /=1.7;
                tempPowers[3] /=1.7;
                setMotorPowers(tempPowers);
                return false;
            }
        }
    }
    public double[] addToArray(double[] first, double[] second)
    {
        if (first.length<second.length){throw new IllegalArgumentException("The second array may not be longer than the first.");}
        double[] arrayBuilder = new double[first.length];
        for(int i = 0;i<first.length;i++)
        {
            arrayBuilder[i] = first[i]+second[i];
        }
        return arrayBuilder;
    }
    public double[] addToArray(double[] first, double second)
    {
        double[] arrayBuilder = new double[first.length];
        for(int i = 0;i<first.length;i++)
        {
            arrayBuilder[i] = first[i]+second;
        }
        return arrayBuilder;
    }
    public double[] multiplyToArray(double[] first, double[] second)
    {
        if (first.length<second.length){throw new IllegalArgumentException("The second array may not be longer than the first.");}
        double[] arrayBuilder = new double[first.length];
        for(int i = 0;i<first.length;i++)
        {
            arrayBuilder[i] = first[i]*second[i];
        }
        return arrayBuilder;
    }
    public double[] multiplyToArray(double[] first, double second)
    {
        double[] arrayBuilder = new double[first.length];
        for(int i = 0;i<first.length;i++)
        {
            arrayBuilder[i] = first[i]*second;
        }
        return arrayBuilder;
    }
    public double[] slideInDirectionPowers(int targetDirection, double speed)
    {
        tempPowers = getMechanumPowers(Math.sin(Math.toRadians(targetDirection - gyroHeading())) < 0, Math.abs(Math.sin(Math.toRadians(targetDirection - gyroHeading()))) * speed);
        tempPowers = addToArray(tempPowers, Math.cos(Math.toRadians(targetDirection - gyroHeading())) * speed);
        return tempPowers;
    }
    //method to return the heading between two given coordinates
    public double headingBetweenCoordinates(double[] first, double[] second)
    {
        double dX = second[0]-first[0];
        double dY = second[1]-first[1];
        return Math.toDegrees(Math.atan2(dY,dX))<0?Math.toDegrees(Math.atan2(dY,dX))+180:Math.toDegrees(Math.atan2(dY,dX))-180;
    }
    public int d_padDirection(int controller)
    {
        if (controller==1)
        {
            if (gamepad1.dpad_up){return 0;}
            else if (gamepad1.dpad_down){return 180;}
            else if (gamepad1.dpad_right){return 90;}
            else if (gamepad1.dpad_left){return -90;}
        }
        else if (controller==2)
        {
            if (gamepad2.dpad_up){return 0;}
            else if (gamepad2.dpad_down){return 180;}
            else if (gamepad2.dpad_right){return 90;}
            else if (gamepad2.dpad_left){return -90;}
        }
        return 0;
    }
    public void loop()
    {
        if (shooter.getPower()>0)
        {
            shooter.setPower(-shooter.getPower());
        }
        //the loop is empty becuase this class is intended to be overridden
    }
    @Override
    public void stop()
    {
        stopDrive();
        collector.setPower(0);
        shooter.setPower(0);
        liftRight.setPower(0);
        liftLeft.setPower(0);
        setRedLED(false);
        setGreenLED(false);
    }
}
