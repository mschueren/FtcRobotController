/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.mentorcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

@TeleOp(name = "Example Mechanum Drive", group = "MentorCode")
@Disabled

public class ExampleMechanumDrive extends OpMode {
    int intZValue = 0;                  //Variable for the integrated Z value (The direction to robot is actually facing
    double driveDirection = 0;          //Direction the robot should move
    double driveSpeed = 0;              //The speed the robot should move
    double driveRotation = 0;           //Causes the robot to rotate
    short robotHeading = 0;             //The direction the robot should be facing
    double flPower = 0, frPower = 0, blPower = 0, brPower = 0;  //calculated motor powers
    double maxMotorPower = 0;           //Variable used to figure out the max motor power so all motors can be scaled between 0 and 1

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    byte[] gyro1Cache;         //The read will return an array of bytes.  They are stored in this variable

    public static final byte GYRO1ADDRESS = 0x20;           //Default I2C address for MR gyro
    public static final int GYRO1_REG_START = 0x04;         //Register to start reading
    public static final int GYRO1_READ_LENGTH = 14;         //Number of byte to read
    public static final int GYRO_COMMAND_REGISTER = 0x03;   //Register to issue commands to the gyro
    public static final byte GYRO_CALIBRATE_COMMAND = 0x4E; //Command to calibrate the gyro

    private I2cAddr gyro1Addr;
    public I2cDevice gyro1;
    public I2cDeviceSynch gyro1Synch;


    @Override
    public void init() {
        frontRight = hardwareMap.dcMotor.get("frMotor");
        frontLeft = hardwareMap.dcMotor.get("flMotor");
        backRight = hardwareMap.dcMotor.get("brMotor");
        backLeft = hardwareMap.dcMotor.get("blMotor");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        gyro1 = hardwareMap.i2cDevice.get("gyro1MR");
        gyro1Addr = I2cAddr.create8bit(GYRO1ADDRESS);
        gyro1Synch = new I2cDeviceSynchImpl(gyro1, gyro1Addr, false);
        gyro1Synch.engage();
        gyro1Synch.setReadWindow(new I2cDeviceSynch.ReadWindow(GYRO1_REG_START, GYRO1_READ_LENGTH, I2cDeviceSynch.ReadMode.REPEAT));
    }

    @Override
    public void init_loop() {
        //Calibrate the gyro if it is not reading 0
        if (gyro1Synch.read8(4) != 0) {
            gyro1Synch.write8(GYRO_COMMAND_REGISTER, GYRO_CALIBRATE_COMMAND);
        }
    }


    @Override
    public void loop() {
        //Get sensor data
        gyro1Cache = gyro1Synch.read(GYRO1_REG_START, GYRO1_READ_LENGTH);

        //One byte only goes to 256 values.  Two bytes must be combined to get 360 degrees
        intZValue = 0X000000FF & (int) gyro1Cache[3];
        intZValue = (intZValue << 8 | (0X000000FF & (int) gyro1Cache[2]));

        //Get the desired drive direction and speed from the right stick on gamepad1
        //the values are scaled by 1.4 so driving forward or backwards can be at full speed
        float y = -gamepad1.right_stick_y * 1.4f;
        float x = gamepad1.right_stick_x * 1.4f;

        //finding the hypotenuse to calculate drive speed
        driveSpeed = Math.sqrt(y * y + x * x);

        //Find the drive direction is radians.  Java uses radians in its trig functions
        if (x < 0 && y > 0) {
            driveDirection = Math.atan(-x / y);
        } else if (x < 0 && y == 0) {
            driveDirection = Math.PI / 2;
        } else if (x < 0 && y < 0) {
            driveDirection = Math.atan(-y / -x) + (Math.PI / 2);
        } else if (x == 0 && y < 0) {
            driveDirection = Math.PI;
        } else if (x > 0 && y < 0) {
            driveDirection = Math.atan(x / -y) + Math.PI;
        } else if (x > 0 && y == 0) {
            driveDirection = Math.PI * 3 / 2;
        } else if (x > 0 && y > 0) {
            driveDirection = Math.atan(y / x) + (Math.PI * 3 / 2);
        } else {
            driveDirection = 0;
        }

        //Determine if the operator wants the robot to rotate and if so in what direction.
        //If the robot is rotated and is not supposed to have rotated, it will return to the robotHeading
        if (gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
            driveRotation = gamepad1.right_trigger - gamepad1.left_trigger;
            robotHeading = (short) intZValue;
        } else if (Math.abs((short) intZValue - robotHeading) > 5){
            driveRotation = ((short) intZValue - robotHeading) / (Math.abs((short) intZValue - robotHeading))  * .5;
        } else {
            driveRotation = 0;
        }

        //Calculate the power for each motor to go the direction, speed and rotation gathered above
        flPower = driveSpeed * Math.cos(driveDirection - Math.toRadians(robotHeading) + Math.PI / 4) - driveRotation;
        frPower = driveSpeed * Math.sin(driveDirection - Math.toRadians(robotHeading) + Math.PI / 4) + driveRotation;
        blPower = driveSpeed * Math.sin(driveDirection - Math.toRadians(robotHeading) + Math.PI / 4) - driveRotation;
        brPower = driveSpeed * Math.cos(driveDirection - Math.toRadians(robotHeading) + Math.PI / 4) + driveRotation;

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
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Left front", flPower);
        telemetry.addData("Left back", blPower);
        telemetry.addData("Right front", frPower);
        telemetry.addData("Right back", brPower);
        telemetry.addData("Drive Rotation", driveRotation);
        telemetry.addData("d_Robot Heading", robotHeading);
        telemetry.addData("Gyro Int ZTot", (short) intZValue);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Drive Direction", Math.toDegrees(driveDirection));
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Math", driveDirection - Math.toRadians(robotHeading));
    }


    @Override
    public void stop() {

    }
}