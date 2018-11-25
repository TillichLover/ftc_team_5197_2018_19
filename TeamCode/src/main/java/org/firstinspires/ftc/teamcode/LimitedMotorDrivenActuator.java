package org.firstinspires.ftc.teamcode;

import android.support.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * THIS IS NOT AN OPMODE
 * IT is a way of creating a Dc motorized actuator that has limits to movement.
 *
 * Version History
 * =================
 * v0.1  @Author Lorenzo Pedroza 11/24/18 //TODO Test this.
 * */


public class LimitedMotorDrivenActuator implements FTCModularizableSystems{
    private DcMotor motor;
    private final double INIT_MOTOR_SPEED;
    private final String MOTOR_NAME;
    private final Integer MINIMUM_ROTATIONS; //using object int to allow @Nullable anotation.
    private final Integer MAXIMUM_ROTAIONS;
    private DcMotor.Direction direction;
    private final boolean HAS_MINIMUM_LIMIT_SWITCH;
    private final boolean HAS_MAXIMUM_LIMIT_SWITCH;
    private final boolean HAS_ENCODER;
    private final boolean GO_TO_MIN_AT_INIT;
    private final boolean GO_TO_MAX_AT_INIT;

    private DigitalChannel minimumLimitSwitch;
    private final String MINIMUM_LIMIT_SWITCH_NAME;

    private DigitalChannel maximumLimitSwitch;
    private final String MAXIMUM_LIMIT_SWITCH_NAME;

    LimitedMotorDrivenActuator(final String MOTOR_NAME, @Nullable final Integer MINIMUM_ROTATIONS,
                               @Nullable final Integer MAXIMUM_ROTAIONS, DcMotor.Direction direction,
                               final boolean HAS_MINIMUM_LIMIT_SWITCH,
                               final boolean HAS_MAXIMUM_LIMIT_SWITCH,
                               final boolean HAS_ENCODER,
                               @Nullable final String MINIMUM_LIMIT_SWITCH_NAME,
                               @Nullable final String MAXIMUM_LIMIT_SWITCH_NAME,
                               final boolean GO_TO_MIN_AT_INIT, final boolean GO_TO_MAX_AT_INIT,
                               final double INIT_MOTOR_SPEED) throws IllegalArgumentException {

        this.HAS_MINIMUM_LIMIT_SWITCH = HAS_MINIMUM_LIMIT_SWITCH;
        this.HAS_MAXIMUM_LIMIT_SWITCH = HAS_MAXIMUM_LIMIT_SWITCH;
        this.HAS_ENCODER = HAS_ENCODER;
        this.GO_TO_MIN_AT_INIT = GO_TO_MIN_AT_INIT;
        this.GO_TO_MAX_AT_INIT = GO_TO_MAX_AT_INIT;

        if(!HAS_ENCODER && !HAS_MAXIMUM_LIMIT_SWITCH && !HAS_MINIMUM_LIMIT_SWITCH)
            throw new IllegalArgumentException("Cannot limit motion without encoding or limit switch");

        if(GO_TO_MIN_AT_INIT && GO_TO_MAX_AT_INIT)
            throw new IllegalArgumentException("Cannot start at min and max");

        if(HAS_ENCODER && (MINIMUM_ROTATIONS == null || MAXIMUM_ROTAIONS == null))
            throw new IllegalArgumentException("Encoded actuators must specify min and max rotations regardless of min or max limit switch as fail safe");

        if(!HAS_ENCODER && (MAXIMUM_ROTAIONS != null || MINIMUM_ROTATIONS != null))
            throw new IllegalArgumentException("Cannot use max and min rotations without an encoder");

        this.MOTOR_NAME = MOTOR_NAME;
        this.MINIMUM_ROTATIONS = MINIMUM_ROTATIONS;
        this.MAXIMUM_ROTAIONS = MAXIMUM_ROTAIONS;
        this.direction = direction;

        this.MAXIMUM_LIMIT_SWITCH_NAME = MAXIMUM_LIMIT_SWITCH_NAME;
        this.MINIMUM_LIMIT_SWITCH_NAME = MINIMUM_LIMIT_SWITCH_NAME;

        this.INIT_MOTOR_SPEED = INIT_MOTOR_SPEED;
    }

    public void init(HardwareMap ahwMap) {
        motor = ahwMap.get(DcMotor.class, MOTOR_NAME);
        motor.setDirection(direction);
        motor.setPower(0);

        if(HAS_ENCODER){
            if(GO_TO_MIN_AT_INIT){
                moveToMinPos(INIT_MOTOR_SPEED);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(HAS_MINIMUM_LIMIT_SWITCH){
            minimumLimitSwitch = ahwMap.get(DigitalChannel.class, MINIMUM_LIMIT_SWITCH_NAME);
            minimumLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        }

        if(HAS_MAXIMUM_LIMIT_SWITCH){
            maximumLimitSwitch = ahwMap.get(DigitalChannel.class, MAXIMUM_LIMIT_SWITCH_NAME);
            maximumLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        }

        if(GO_TO_MIN_AT_INIT){
            moveToMinPos(INIT_MOTOR_SPEED);
        }
        if(GO_TO_MAX_AT_INIT){
            moveToMaxPos(INIT_MOTOR_SPEED);
        }

    }

    public int getCurrentPosition(){
        if (HAS_ENCODER){
            return motor.getCurrentPosition();
        }
        else return -1;

    }

    public void move(double speed, @Nullable Integer rotations) throws IllegalArgumentException {
        int rotationTarget;
        if (rotations == null &&( !HAS_MAXIMUM_LIMIT_SWITCH || !HAS_MINIMUM_LIMIT_SWITCH ) && !HAS_ENCODER) {
            throw new IllegalArgumentException("Cannot limit motion without encoders at both limits if missing encoder.");
        }


        if((HAS_MINIMUM_LIMIT_SWITCH || (rotations == MINIMUM_ROTATIONS && HAS_MINIMUM_LIMIT_SWITCH))){
            speed = -Math.abs(speed);
            while (!minimumLimitSwitch.getState() && speed <0) {
                motor.setPower(speed);
            }
            if(HAS_ENCODER){
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setPower(0); //then don't forget to stop motor.
        }

        if (HAS_MAXIMUM_LIMIT_SWITCH || (rotations == MAXIMUM_ROTAIONS && HAS_MAXIMUM_LIMIT_SWITCH)) {  //default to these commands if limit switch. For maximum, exit method after. For minimum, it is possible to reset encoder, so no need to exit.
            speed = Math.abs(speed);
            while (!maximumLimitSwitch.getState() && speed > 0) {
                motor.setPower(speed);
            }
            motor.setPower(0);
            return;
        }

        if (HAS_ENCODER && rotations != null) {

            if (HAS_MINIMUM_LIMIT_SWITCH && motor.getCurrentPosition() < (0.5*(MAXIMUM_ROTAIONS-MINIMUM_ROTATIONS))) {  //resonably should be closer to 0 position. This also compensates for magnetic limit switches using one magnetic limit switch and a magnet at both ends
                if (minimumLimitSwitch.getState()) {
                    if (MINIMUM_ROTATIONS == 0){
                        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //set to zero. Unfortunately, I have found no other way to change the rotation value of encoders.
                        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    return;
                }
            }

            if (HAS_MAXIMUM_LIMIT_SWITCH && motor.getCurrentPosition() > (0.5*MAXIMUM_ROTAIONS-MINIMUM_ROTATIONS)) {  //account for magnetic limit switch. If rotaitons is bigger than midpoint, most likely upper limit.
                if (maximumLimitSwitch.getState()) {
                    return;
                }
            }
            if (motor.getCurrentPosition() >= MAXIMUM_ROTAIONS && (rotations > 0)) //abort if already erect
                return;
            if ((motor.getCurrentPosition() <= MINIMUM_ROTATIONS) && (rotations < 0)) //another reasons to abort
                return;
            rotationTarget = motor.getCurrentPosition() + rotations;
            motor.setTargetPosition(rotationTarget);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.abs(speed)); //direction set.
            while (motor.isBusy()) ;//wait for motor to move
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void moveToMaxPos(double speed){
        move(Math.abs(speed), MAXIMUM_ROTAIONS);
    }

    public void moveToMinPos(double speed){
        move(Math.abs(speed), MINIMUM_ROTATIONS);
    }

    public void teleOpMove(boolean moveToMaxPosButton, boolean moveToMinPosButton, double speed){
        //always test limit switches first. Best limit test as sense physical limit.
        if (HAS_MAXIMUM_LIMIT_SWITCH){
            if(!maximumLimitSwitch.getState() && moveToMaxPosButton){
                motor.setPower(Math.abs(speed));
            }
        }

        if(HAS_MINIMUM_LIMIT_SWITCH){
            if(!minimumLimitSwitch.getState() && moveToMinPosButton){
                motor.setPower(-(Math.abs(speed)));
            }
        }

        if (HAS_ENCODER){
            if((motor.getCurrentPosition() <= MAXIMUM_ROTAIONS) && moveToMaxPosButton){
                motor.setPower(Math.abs(speed));
            }

            if ((motor.getCurrentPosition() >= MINIMUM_ROTATIONS) && moveToMinPosButton){
                motor.setPower(-(Math.abs(speed)));
            }
        }
    }

    public void teleOpMoveToMinPos(boolean moveToMinPosButton, double speed){
        if (moveToMinPosButton)
            moveToMinPos(speed);
    }

    public void teleOpMoveToMaxPos(boolean moveToMaxPosButton, double speed){
        if(moveToMaxPosButton)
            moveToMaxPos(speed);
    }


}
