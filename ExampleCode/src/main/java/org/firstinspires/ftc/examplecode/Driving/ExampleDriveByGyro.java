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

package org.firstinspires.ftc.examplecode.Driving;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp(name = "Example Drive by Gyro", group = "DriveExample")
@Disabled

public class ExampleDriveByGyro extends OpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;
    BNO055IMU imu;

    Orientation angles;
    double heading;
    int rotations = 0;
    int targetDirection = 0;
    double directionCorrectSpd = 0;

    @Override
    public void init() {
        rightMotor = hardwareMap.dcMotor.get("right motor");
        leftMotor = hardwareMap.dcMotor.get("left motor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status:", "Robot is Initialized");
    }

    @Override
    public void init_loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("1", "Integrated Heading: " + getIntegratedHeading());
        telemetry.addData("2", "heading: " + angles.firstAngle);
        telemetry.addData("1 Right Motor Pos", rightMotor.getCurrentPosition());
        telemetry.addData("2 Left Motor Pos", leftMotor.getCurrentPosition());
    }


    @Override
    public void start() {
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(gamepad1.dpad_left){
            targetDirection++;
        } else if(gamepad1.dpad_right){
            targetDirection--;
        }

        if(gamepad1.a) {
            directionCorrectSpd = (targetDirection - getIntegratedHeading())/100;
            rightMotor.setPower(.3 + directionCorrectSpd);
            leftMotor.setPower(.3 - directionCorrectSpd);
        }
        else {
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }

        // send the info back to driver station using telemetry function.
        telemetry.addData("1 Right Motor Power", rightMotor.getCurrentPosition());
        telemetry.addData("2 Left Motor Power", leftMotor.getCurrentPosition());
        telemetry.addData("3", "Integrated Heading: " + getIntegratedHeading());
        telemetry.addData("4", "heading: " + formatAngle(angles.angleUnit, angles.firstAngle));
    }


    @Override
    public void stop() {
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getIntegratedHeading() {
        if(heading - (rotations * 360 + angles.firstAngle) > 200) {
            rotations++;
        }
        else if(heading - (rotations * 360 + angles.firstAngle) < -200) {
            rotations--;
        }

        heading = rotations * 360 + angles.firstAngle;
        return heading;
    }
}