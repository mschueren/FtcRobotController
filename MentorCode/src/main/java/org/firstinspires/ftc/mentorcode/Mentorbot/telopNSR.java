package org.firstinspires.ftc.mentorcode.Mentorbot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop", group="World")
@Disabled
public class telopNSR extends toborParent {

    enum driveMode{shooting, lifting}
    boolean xPressed;
    boolean yPressed;
    driveMode currentMode;
    boolean currentlyTurning;
    boolean liftDown =false;
    boolean liftGoingDown = false;
    boolean liftGoingUp = false;
    boolean liftingToTop = false;
    boolean isShooting = false;
    int targetHeading;
    double[] motorPowers;
    boolean canShoot = false;
    ElapsedTime shooterTimer;


    @Override
    public void init() {
        super.init();
        currentMode = driveMode.shooting;
        moveButtonServo(true, true);
        moveButtonServo(true,false);
        shooterTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    @Override
    public void init_loop()
    {
        if (shooter.getMode()!=DcMotor.RunMode.RUN_TO_POSITION)
        {
            shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (!isAtPosition(shooter,20)){shooter.setTargetPosition(shooter.getCurrentPosition());}
        super.init_loop();
        timer.reset();
    }
    //call the init method of the parent op mode, which initializes all of the motors,
    //servos, and sensors.


    public int gyroHeading() {
        return gyroSensor.getIntegratedZValue();
    }

    //The gyroHeading() method is marked abstract in the parent op mode because both autonomous
    //programs must have slightly different versions of it. Because it is marked abstract in the
    //parent op mode, it must also be overridden in the teleop, although it is not used.
    @Override
    public void loop() {

        motorPowers = new double[] {0,0,0,0};
        telemetry.addLine((ballInShooter() ? "Ball in shooter" : "No ball in shooter") + " (" + String.format("%.2f", ballSensor.getVoltage()) + ")");

        //blocker, aim
        setGreenLED(ballInShooter());
        if (!isAtPosition(shooter,30))
        {
            moveBlocker(false);
            moveAimServo(false);
        }
        else {
            if (gamepad2.a) {
                moveBlocker(true);
                moveAimServo(true);
            } else {
                if (ballInShooter()) {
                    moveBlocker(false);
                    moveAimServo(false);
                } else {
                    moveBlocker(true);
                    moveAimServo(true);
                }
            }
        }

        //collector
        if (gamepad2.y) {
            collector.setPower(.9);
            yPressed = true;
        } else if (!gamepad2.y && yPressed) {
            yPressed = false;
            collector.setPower(0);
        } else if (gamepad2.x && !xPressed) {
            collector.setPower(Math.abs(collector.getPower()) > .1 ? 0 : -.9);
            xPressed = true;
        } else if (!gamepad2.x) {
            xPressed = false;
        }

        if (tempTimer.time()>=.5&&gamepad1.a)
        {
            currentMode = (currentMode==driveMode.shooting)?driveMode.lifting:driveMode.shooting;
            tempTimer.reset();
        }

        if (gamepad2.right_stick_y>.5){moveGrabber(true);}
        else if (gamepad2.right_stick_y<-.5){moveGrabber(false);}

        if (gamepad2.dpad_left&&liftLeft.getCurrentPosition()<=100)
        {
            liftGoingDown = true;
            liftRight.setTargetPosition(1000);
            liftLeft.setTargetPosition(1000);
            currentMode = driveMode.lifting;
        }
        if (liftGoingDown&&isAtPosition(liftRight,liftLeft,20))
        {
            liftGoingDown = false;
            liftGoingUp = true;
            liftRight.setTargetPosition(500);
            liftLeft.setTargetPosition(200);
        }
        if (liftGoingUp&&isAtPosition(liftRight,liftLeft,20))
        {
            liftDown = true;
            liftGoingUp = false;
        }
        if (liftDown&&gamepad2.dpad_up)
        {
            liftingToTop = true;
            liftLeft.setTargetPosition(topOfLift);
            liftRight.setTargetPosition(topOfLift+300);
        }

        if (liftGoingDown||liftGoingUp||liftingToTop)
        {
            liftRight.setPower(.9);
            liftLeft.setPower(.9);
        }
        else
        {
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }
        if (gamepad2.dpad_right)
        {
            liftGoingDown = false;
            liftGoingUp = false;
            liftingToTop = false;
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }

        if (gamepad2.dpad_down)
        {
            liftGoingDown = false;
            liftGoingUp = false;
            liftingToTop = false;
            if (isAtPosition(liftLeft,liftRight,60))
            {
                liftLeft.setTargetPosition(liftLeft.getCurrentPosition() - 300);
                liftRight.setTargetPosition(liftRight.getCurrentPosition() - 300);
            }
            liftRight.setPower(.4);
            liftLeft.setPower(.4);
        }

        if (gamepad2.b){canShoot = true;}
        if (gamepad2.b&&isAtPosition(shooter,30))
        {
            shooter.setTargetPosition(shooter.getCurrentPosition()+4700);
        }
        if (canShoot){shooter.setPower(.85);}
        else{shooter.setPower(0);}

        setRedLED(currentMode==driveMode.lifting);
           switch(currentMode)
            {
                case shooting:
                    motorPowers = slideInDirectionPowers(90+(int)Math.toDegrees(Math.atan2(-gamepad1.right_stick_y,gamepad1.right_stick_x)),Math.sqrt(Math.pow(gamepad1.right_stick_x,2)+Math.pow(gamepad1.right_stick_y,2)));
                    if (gamepad1.dpad_up||gamepad1.dpad_down||gamepad1.dpad_right||gamepad1.dpad_left)
                    {
                        currentlyTurning = true;
                        targetHeading = d_padDirection(1);
                    }
                    else if (Math.abs((gyroHeading()%180)-targetHeading)<7)
                    {
                        currentlyTurning = false;
                    }
                    if (currentlyTurning&&Math.abs((gyroHeading()%180)-targetHeading)>=20)
                    {
                        motorPowers = addToArray(motorPowers,getPivotValues(gyroHeading()%180>targetHeading,.9));
                    }
                    else if (currentlyTurning)
                    {
                        motorPowers = addToArray(motorPowers,getPivotValues(gyroHeading()%180>targetHeading,.3));
                    }
                    else if (Math.abs(gamepad1.left_stick_y)>=.4)
                    {
                        motorPowers = addToArray(motorPowers, getPivotValues(gamepad1.left_stick_y<0, .9));
                    }
                    break;
                case lifting:
                    if (gamepad1.right_bumper||gamepad1.right_trigger>=triggerThreshold) {
                        //use the mechanum wheels to slide sideways to the right
                        motorPowers = getMechanumPowers(true,.9);
                    } else if (gamepad1.left_bumper||gamepad1.left_trigger>=triggerThreshold) {
                        //use the mechanum wheels to slide sideways to the left
                        motorPowers = getMechanumPowers(false,.9);
                    }
                    else
                    {
                        motorPowers = new double[]
                                {
                                        -gamepad1.right_stick_y*.5,
                                        -gamepad1.right_stick_y*.5,
                                        -gamepad1.left_stick_y*.5,
                                        -gamepad1.left_stick_y*.5
                                };
                        if (gamepad1.right_stick_button||gamepad1.left_stick_button)
                        {
                            motorPowers = multiplyToArray(motorPowers,2.0);
                        }
                    }
                    break;
            }
        //slideInDirectionPowers((int)Math.toDegrees(Math.atan2(gamepad1.right_stick_y,gamepad1.right_stick_x)),.5,gyroSensor.getIntegratedZValue()%180);


        //the following four if statements raise and lower the back button pushers
        //according to the drivers commands (see the moveButtonServo method in the parent op mode)
        if (gamepad2.right_bumper) {
            moveButtonServo(true, true);
        }
        if (gamepad2.right_trigger >= triggerThreshold) {
            moveButtonServo(false, true);
        }
        if (gamepad2.left_bumper) {
            moveButtonServo(true, false);
        }
        if (gamepad2.left_trigger >= triggerThreshold) {
            moveButtonServo(false, false);
        }

//        if (gamepad2.dpad_up) {
//            liftLeft.setPower(1);
//            liftRight.setPower(1);
//        } else if (gamepad2.dpad_down) {
//            liftLeft.setPower(-.5);
//            liftRight.setPower(-.5);
//        } else {
//            liftLeft.setPower(0);
//            liftRight.setPower(0);
//        }

        if (Math.abs(liftLeft.getPower())>=.2||Math.abs(liftRight.getPower())>=.2) {
            moveButtonServo(true, true);
            moveButtonServo(true, false);
        }
        if (!isAtPosition(shooter,30))
        {
            motorPowers = new double[] {0,0,0,0};
        }
        setMotorPowers(motorPowers);


        telemetry.addData("mode:",currentMode);
        telemetry.addData("lift left position",liftLeft.getCurrentPosition());
        telemetry.addData("lift right position",liftRight.getCurrentPosition());
        telemetry.addData("FR",frontRight.getPower());
        telemetry.addData("FL",frontLeft.getPower());
        telemetry.addData("BR",backRight.getPower());
        telemetry.addData("BL",backLeft.getPower());

        //the following two if statements allow one button to be used to switch the position
        //of the servo blocking the path from the collector into the shooter, allowing the
        //robot to store balls in the collector
    }
    @Override
    public void stop()
    {
        super.stop();
    }

}