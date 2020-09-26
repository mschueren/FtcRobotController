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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

@TeleOp(name = "Read Adafruit LIDAR", group = "MentorCode")
//@Disabled
public class ExampleI2cLIDARRead extends OpMode {

    public static final int VL53L0X_ADDR =                  0x52;
    public static final int VL53L0X_2_8V_REG =              0x89;
    public static final int VL53L0X_I2C_REG =               0x88;
    public static final int VL53L0X_RANGE_START_REG =       0x00;
    public static final int IDENTIFICATION_MODEL_ID =       0xC0;
    public static final int VL53L0X_RESULT_RANGE_STATUS =   0x14;

    public static final byte VL53L0X_CONT_READ_MODE =       0x02;
    public static final byte VL53L0X_2_8V_MODE =            0x01;
    public static final byte VL53L0X_I2C_MODE =             0x00;

    public int range_mm = 0;
    public int prev_range_mm = 0;
    public int num_bogus_conversions = 0;

    public I2cAddr lidarAddr;
    public I2cDevice lidarSensor;
    public I2cDeviceSynch lidarSynch;

    @Override
    public void init() {
        lidarSensor = hardwareMap.i2cDevice.get("lidar");
        lidarAddr = I2cAddr.create8bit(VL53L0X_ADDR);
        lidarSynch = new I2cDeviceSynchImpl(lidarSensor, lidarAddr, false);
        lidarSynch.engage();

        telemetry.addData("Sensor", "Initialzing");
        telemetry.addData("2.8V Byte before write (0): ", lidarSynch.read8(VL53L0X_2_8V_REG));
        telemetry.addData("I2C Mode Byte before write (36): ", lidarSynch.read8(VL53L0X_I2C_REG));
        telemetry.addData("Measurement Type Byte before write(0): ", lidarSynch.read8(VL53L0X_RANGE_START_REG));

        //Set the sensor to run in 2.8V
        lidarSynch.write8(VL53L0X_2_8V_REG, VL53L0X_2_8V_MODE);

        //Set the I2C communication to standard mode
        lidarSynch.write8(VL53L0X_I2C_REG, VL53L0X_I2C_MODE);

        //Set the sensor to start reading continuously. Sensor should update every 20-30ms.
        lidarSynch.write8(VL53L0X_RANGE_START_REG, VL53L0X_CONT_READ_MODE);

        telemetry.addData("Sensor", "Initialzed");
        telemetry.addData("2.8V Byte 0X89 (1) ", lidarSynch.read8(VL53L0X_2_8V_REG));
        telemetry.addData("I2C Byte 0X88 (0)", lidarSynch.read8(VL53L0X_I2C_REG));
        telemetry.addData("Cont. Read Byte 0X00 (2)", lidarSynch.read8(VL53L0X_RANGE_START_REG));
        telemetry.addData("Lidar Model (238)", lidarSynch.read8(IDENTIFICATION_MODEL_ID) + 256);
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() { }

    @Override
    public void loop() {

        range_mm = read_range_mm();

        // so, the max range of this gizmo is about 2000mm, but we will use it in
        // "default mode," which is specified to 1200mm (30 ms range timing budget).
        // note: 20mm or 8190mm seems to be returned for bogus readings, so trap/limit:
        if ((range_mm <= 20) || (8190 == range_mm)) {
            range_mm = prev_range_mm;       // bad reading -- use previous
            ++num_bogus_conversions;        // count the bad readings
        } else if (range_mm > 1200) {       // our defined max
            range_mm = 1200;                // out-of-spec reading -- just limit
            ++num_bogus_conversions;        // but also count the out-of-spec readings
        } else {
            prev_range_mm = range_mm;       // good reading (hopefully) -- save
            num_bogus_conversions = 0;
        }

        // --- super-simple "calibration" by surlee:
        if (range_mm > 400) range_mm -= 50;
        else if (range_mm > 160) range_mm -= 40;
        else if (range_mm > 100) range_mm -= 35;
        else range_mm -= 30;

        // send the info back to driver station using telemetry function.
        telemetry.addData("Range Reading (mm):", range_mm);
        telemetry.addData("Range Reading (in):", range_mm/25.4);
        telemetry.addData("Bad Count:", num_bogus_conversions);
    }


    @Override
    public void stop() { }


    //------------------------------------------------------------------------------
    /*
     Returns a range reading in millimeters.
     range in mm (or 0 for timeout, or 20 or 8190 for bogus measurement)
     */
    public int read_range_mm() {
        byte[] reading;
        int result;

        // get range in mm (just deal with errouneous readings in calling routine).
        reading = lidarSynch.read(VL53L0X_RESULT_RANGE_STATUS + 10, 2);

        result = 0X000000FF & (int)reading[0] + 256;
        result = (result << 8 | (0X000000FF & (int)reading[1]));

        return result;
    }


}