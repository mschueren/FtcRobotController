package org.firstinspires.ftc.examplecode.Driving;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "Mechanum Drive", group = "Example")
//@Disabled

public class ExampleMechanumDrive extends OpMode {

    DcMotor frMotor, flMotor, brMotor, blMotor;
    BNO055IMU imu;
    Orientation gyroAngles;

    double frPower=0, flPower=0, brPower=0, blPower=0;
    double maxMotorPower=0;
    double driveSpeed = 0;              //The speed the robot should move
    double driveRotation = 0;           //Causes the robot to rotate
    double x, y, joystickAngle, joystickAngle360;
    double desiredRobotHeading;
    int rotations = 0;

    @Override
    public void init() {
        frMotor = hardwareMap.dcMotor.get("frontright");
        flMotor = hardwareMap.dcMotor.get("frontleft");
        brMotor = hardwareMap.dcMotor.get("backright");
        blMotor = hardwareMap.dcMotor.get("backleft");

        frMotor.setDirection(DcMotor.Direction.REVERSE);
        brMotor.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status:", "Robot is Initialized");
    }


    @Override
    public void init_loop() {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("1", "Integrated Heading: " + getIntegratedHeading());
        telemetry.addData("2", "heading: " + gyroAngles.firstAngle);
        telemetry.addData("1 Right Motor Pos", frMotor.getCurrentPosition());
        telemetry.addData("2 Left Motor Pos", flMotor.getCurrentPosition());
    }


    @Override
    public void start() { }


    @Override
    public void loop() {
        //Get gyro heading
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //Positive values for x axis are joystick right
        //Positive values for y axis are joystick down
        y = Range.clip(-gamepad1.right_stick_y,-1,1);
        x = Range.clip(gamepad1.right_stick_x,-1,1);
        joystickAngle = Math.atan2(-x,y);
        joystickAngle360 = joystickAngle >= 0 ? joystickAngle : (2*Math.PI) + joystickAngle;

        //finding the hypotenuse to calculate drive speed
        driveSpeed = Range.clip(Math.sqrt(y * y + x * x), -1, 1);

        //Determine if the operator wants the robot to rotate and if so in what direction.
        //If the robot is rotated and is not supposed to have rotated, it will return to the robotHeading
        if (gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
            driveRotation = gamepad1.left_trigger - gamepad1.right_trigger;
            desiredRobotHeading = getIntegratedHeading();
        } else if (Math.abs(desiredRobotHeading - getIntegratedHeading() ) > 5){
            driveRotation = (desiredRobotHeading - getIntegratedHeading()) / (Math.abs(desiredRobotHeading - getIntegratedHeading()))  * .5;
        } else {
            driveRotation = 0;
        }

        //Calculate the power for each motor to translate in the desired direction
        flPower = Math.cos(joystickAngle360 - Math.toRadians(desiredRobotHeading) + Math.PI / 4);
        frPower = Math.sin(joystickAngle360 - Math.toRadians(desiredRobotHeading) + Math.PI / 4);
        blPower = Math.sin(joystickAngle360 - Math.toRadians(desiredRobotHeading) + Math.PI / 4);
        brPower = Math.cos(joystickAngle360 - Math.toRadians(desiredRobotHeading) + Math.PI / 4);

        //Determine the maximum calulated motor power
        maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));

        //Ratio the powers for disrection
        flPower = flPower / maxMotorPower;
        frPower = frPower / maxMotorPower;
        blPower = blPower / maxMotorPower;
        brPower = brPower / maxMotorPower;

        //Calculate the power for each motor to go the direction, speed and rotation gathered above
        flPower = driveSpeed * flPower - driveRotation;
        frPower = driveSpeed * frPower + driveRotation;
        blPower = driveSpeed * blPower - driveRotation;
        brPower = driveSpeed * brPower + driveRotation;

        //Determine the maximum calulated motor power
        maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));

        //If any motor power is not in the range {-1, 1}, scale all motors to fit in the range and still move as directed
        if (Math.abs(maxMotorPower) > 1) {
            flPower = flPower / maxMotorPower;
            frPower = frPower / maxMotorPower;
            blPower = blPower / maxMotorPower;
            brPower = brPower / maxMotorPower;
        } else if(Math.abs(maxMotorPower) < .03) {
            flPower = 0;
            frPower = 0;
            blPower = 0;
            brPower = 0;
        }

        //Set the actual motor powers
        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Left front", flPower);
        telemetry.addData("Right front", frPower);
        telemetry.addData("Left back", blPower);
        telemetry.addData("Right back", brPower);
        telemetry.addData("Drive Rotation", driveRotation);
        telemetry.addData("d_Robot Heading", desiredRobotHeading);
        telemetry.addData("Gyro", gyroAngles);
        telemetry.addData("Gyro Int ZTot", getIntegratedHeading());
        telemetry.addData("Gyro Rotations", rotations);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("Joystick Angle", Math.toDegrees(joystickAngle));
        telemetry.addData("Joystock Angle360", Math.toDegrees(joystickAngle360));
    }


    @Override
    public void stop() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }


    private double getIntegratedHeading() {
        if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        }
        else if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }

        return (rotations * 360 + gyroAngles.firstAngle);
    }

}