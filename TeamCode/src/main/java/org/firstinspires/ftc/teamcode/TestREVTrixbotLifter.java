package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name ="REVTribotLiftTest", group = "REVTrixbot")
public class TestREVTrixbotLifter extends LinearOpMode {
    REVTrixbot robot = new REVTrixbot();
    //DcMotor motor;
    //counterclock wise retrat lifter.Counter clockwise is forward.


    @Override
    public void runOpMode() {
       robot.roverRuckusRevTrixBotLift.init(hardwareMap);
          /*
        motor = hardwareMap.get(DcMotor.class, "EH2motor1");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE); //revers and postive value tunrs clockwise
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */
        waitForStart();

       // robot.runTime.reset();

        while(opModeIsActive()){
           //robot.roverRuckusRevTrixBotLift.moveToMaxPos(1);
                //motor.setPower(0.025);


           telemetry.addData("Lift Rotations", robot.roverRuckusRevTrixBotLift.getCurrentPosition());
           updateTelemetry(telemetry);
        }
    }
}
