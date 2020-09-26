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

@TeleOp(name = "Mentorbot Tank Drive", group = "Mentorbot")
//@Disabled

public class RobotMentorbotDrive extends OpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor leftArmMotor;
    DcMotor rightArmMotor;
    DcMotor armExtend;
    Servo Servo1;
    Servo Servo2;

    @Override
    public void init() {
        rightMotor = hardwareMap.dcMotor.get("right motor");
        leftMotor = hardwareMap.dcMotor.get("left motor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftArmMotor = hardwareMap.dcMotor.get("left arm motor");
        rightArmMotor = hardwareMap.dcMotor.get("right arm motor");
        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armExtend = hardwareMap.dcMotor.get("arm extend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);

        Servo1 = hardwareMap.servo.get("gripper");
        Servo2 = hardwareMap.servo.get("gripper2");
    }

    @Override
    public void init_loop() {

    }


    @Override
    public void loop() {

        rightMotor.setPower(gamepad1.right_stick_y);
        leftMotor.setPower(gamepad1.left_stick_y);

        leftArmMotor.setPower(gamepad1.right_trigger*.75 - gamepad1.left_trigger*.3);
        rightArmMotor.setPower(gamepad1.right_trigger*.75 - gamepad1.left_trigger*.3);

            if (gamepad1.y)
            { armExtend.setPower(1);}
            else if (gamepad1.a)
            { armExtend.setPower(-1);}
            else
            { armExtend.setPower(0);}
        //Servo1.setPosition(gamepad1.right_bumper);   disabled this line to create variable speed
            if (gamepad1.right_bumper)
            { Servo1.setPosition(1);
            Servo2.setPosition(0);}
            else if (gamepad1.left_bumper)
            { Servo1.setPosition(0);
            Servo2.setPosition(1);}
            else if (gamepad1.b)
            { Servo1.setPosition(0.5);
            Servo2.setPosition(0.5);}



        // send the info back to driver station using telemetry function.
        telemetry.addData("Left front", rightMotor.getPower());
        telemetry.addData("Left back", leftMotor.getPower());

    }


    @Override
    public void stop() {

    }
}