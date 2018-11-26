package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name ="REVTribotLiftTest", group = "REVTrixbot")
public class TestREVTrixbotLifter extends LinearOpMode {
    REVTrixbot robot = new REVTrixbot();
    DcMotor motor;

    @Override
    public void runOpMode() {
       // robot.roverRuckusRevTrixBotLift.init(hardwareMap);
        motor = hardwareMap.get(DcMotor.class, "EH1motor1");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

       // robot.runTime.reset();

        while(opModeIsActive()){
           // robot.roverRuckusRevTrixBotLift.moveToMaxPos(1);
           motor.setPower(0.25);
           sleep(500);
           // robot.roverRuckusRevTrixBotLift.moveToMinPos(1);
            sleep(1000);
        }
    }
}
