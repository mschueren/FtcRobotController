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

package org.firstinspires.ftc.examplecode.Examplebot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Examplebot Drive Square", group = "Examplebot")
@Disabled

public class ExamplebotDriveSquare extends OpMode {

    ExampleHardwareExamplebot robot = new ExampleHardwareExamplebot();

    int targetPosition = 0;
    int count = 1;

    State programState;

    public enum State {
        STRAIGHT,
        TURN,
        END
    }

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rightMotor.setTargetPosition(targetPosition);
        robot.leftMotor.setTargetPosition(targetPosition);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        programState = State.STRAIGHT;

        telemetry.addData("Status:", "Robot is Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addData("1 Right Motor Pos", robot.rightMotor.getCurrentPosition());
        telemetry.addData("2 Left Motor Pos", robot.leftMotor.getCurrentPosition());
    }

    @Override
    public void start() {
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        targetPosition = (int) (24 * robot.countsPerInch());
        robot.rightMotor.setTargetPosition(targetPosition + robot.rightMotor.getCurrentPosition());
        robot.leftMotor.setTargetPosition(targetPosition + robot.leftMotor.getCurrentPosition());
    }


    @Override
    public void loop() {

        switch (programState) {
            case STRAIGHT:
                robot.rightMotor.setPower(.8);
                robot.leftMotor.setPower(.8);

                if (!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy()) {
                    programState = State.TURN;
                    targetPosition = (int) (90 * robot.countsPerTurnDegree());
                    robot.rightMotor.setTargetPosition(targetPosition + robot.rightMotor.getCurrentPosition());
                    robot.leftMotor.setTargetPosition(robot.leftMotor.getCurrentPosition());
                }
                break;

            case TURN:
                robot.rightMotor.setPower(.8);
                robot.leftMotor.setPower(0);

                if (!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy() && count <= 4) {
                    programState = State.STRAIGHT;
                    targetPosition = (int) (24 * robot.countsPerInch());
                    robot.rightMotor.setTargetPosition(targetPosition + robot.rightMotor.getCurrentPosition());
                    robot.leftMotor.setTargetPosition(targetPosition + robot.leftMotor.getCurrentPosition());
                    count++;
                }
                else if (count > 4) {
                    programState = State.END;
                }
                break;

            case END:
                robot.rightMotor.setPower(0);
                robot.leftMotor.setPower(0);
                break;
        }


        // send the info back to driver station using telemetry function.
        telemetry.addData("1 Right Motor Power", robot.rightMotor.getCurrentPosition());
        telemetry.addData("2 Left Motor Power", robot.leftMotor.getCurrentPosition());
        telemetry.addData("3 Target Position", targetPosition);
        telemetry.addData("4 Square Side", count);
    }


    @Override
    public void stop() {
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
    }
}