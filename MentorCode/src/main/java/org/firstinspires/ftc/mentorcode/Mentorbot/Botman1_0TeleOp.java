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

package org.firstinspires.ftc.mentorcode.Mentorbot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Botman 1.0 TeleOp", group = "Botman")
//@Disabled

public class Botman1_0TeleOp extends OpMode {

    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    DcMotorSimple armDC;
    Servo rightGripper;
    Servo leftGripper;

    double rightGripperOpen = .05;
    double rightGripperClose = .95;
    double leftGripperOpen = .95;
    double leftGripperClose = .06;
    double speed = 1;


    @Override
    public void init() {
        frontRightMotor = hardwareMap.get(DcMotor.class,"frontright");
        frontLeftMotor = hardwareMap.get(DcMotor.class,"frontleft");
        backRightMotor = hardwareMap.get(DcMotor.class,"backright");
        backLeftMotor = hardwareMap.get(DcMotor.class,"backleft");
        armDC = hardwareMap.get(DcMotorSimple.class,"arm");
        rightGripper = hardwareMap.get(Servo.class,"rightGripper");
        leftGripper = hardwareMap.get(Servo.class,"leftGripper");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status:", "Robot is Initialized");

        rightGripper.setPosition(rightGripperOpen);
        leftGripper.setPosition(leftGripperOpen);
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() {
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        if(gamepad1.dpad_down) {
            speed = .5;
        } else if (gamepad1.dpad_up) {
            speed = 1;
        }

        //Set drive motor speeds
        if(gamepad1.right_trigger > 0.1) {
            frontRightMotor.setPower(.8 * speed);
            backRightMotor.setPower(-.8 * speed);
            frontLeftMotor.setPower(-.8 * speed);
            backLeftMotor.setPower(.8 * speed);
        }
        else if(gamepad1.left_trigger > 0.1) {
            frontRightMotor.setPower(-.8 * speed);
            backRightMotor.setPower(.8 * speed);
            frontLeftMotor.setPower(.8 * speed);
            backLeftMotor.setPower(-.8 * speed);
        }
        else {
            frontRightMotor.setPower(Range.clip(gamepad1.right_stick_y, -1, 1) * speed);
            backRightMotor.setPower(Range.clip(gamepad1.right_stick_y, -1, 1) * speed);
            frontLeftMotor.setPower(Range.clip(gamepad1.left_stick_y, -1, 1) * speed);
            backLeftMotor.setPower(Range.clip(gamepad1.left_stick_y, -1, 1) * speed);
        }


        //Set arm motor speed
        if(gamepad1.y) {
            armDC.setPower(1);
        } else if(gamepad1.a){
            armDC.setPower(-.15);
        } else {
            armDC.setPower(0);
        }

        //Set gripper position
        if (gamepad1.x) {
            rightGripper.setPosition(rightGripperOpen);
            leftGripper.setPosition(leftGripperOpen);
        } else if(gamepad1.b) {
            rightGripper.setPosition(rightGripperClose);
            leftGripper.setPosition(leftGripperClose);
        }


        // send the info back to driver station using telemetry function.
        telemetry.addData("Right Motor Power", frontRightMotor.getPower());
        telemetry.addData("Left Motor Power", frontLeftMotor.getPower());
        telemetry.addData("0", "frMotor: " + frontRightMotor.getCurrentPosition());
        telemetry.addData("1", "flMotor: " + frontLeftMotor.getCurrentPosition());
        telemetry.addData("2", "brMotor: " + backRightMotor.getCurrentPosition());
        telemetry.addData("3", "blMotor: " + backLeftMotor.getCurrentPosition());
    }


    @Override
    public void stop() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        armDC.setPower(0);
    }
}