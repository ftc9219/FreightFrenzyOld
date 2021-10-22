package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DriverControlled", group = "TeleOp")

public class DriverControlled extends LinearOpMode {

    Hardware hardware = new Hardware();



    //variables
    int n = 0;

    @Override
    public void runOpMode() {

        hardware.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {

            //controller
            float LSX1 = gamepad1.left_stick_x;
            float LSY1 = gamepad1.left_stick_y;
            float RSX1 = gamepad1.right_stick_x;
            float RSY1 = gamepad1.right_stick_y;
            boolean AB1 = gamepad1.a;
            boolean BB1 = gamepad1.b;
            boolean XB1 = gamepad1.x;
            boolean YB1 = gamepad1.y;
            float LT1 = gamepad1.left_trigger;
            float RT1 = gamepad1.right_trigger;
            boolean AB2 = gamepad2.a;
            boolean BB2 = gamepad2.b;
            boolean XB2 = gamepad2.x;
            boolean YB2 = gamepad2.y;
            float LT2 = gamepad2.left_trigger;
            float RT2 = gamepad2.right_trigger;

            //code


            //updates location every 10 cycles
            if (n == 10) {
                hardware.updateLocation();
                n = 0;
            }
            n++;
        }


    }

}