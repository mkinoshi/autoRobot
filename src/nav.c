/**
 * Matt Martin, Makoto Kinoshita
 * Feb 26, 2017
 * CS363 Robotics
 * Main navigation file
 */

#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "Mage.h"
#include <ipc.h>
#include <GCM_IPC.h>
#include <SVM_VisionModule.h>
#include "robot.h"
#include "utils.h"
#include "movements.h"
#include "particle.h"
#include "../inc/robot.h"

/**
 * recieves messages from IPC and stores it in the robot
 * @param msgInstance   unused
 * @param callData      IPC message return
 * @param clientData    pointer to Robot
 */
void IPCMessageHandler(MSG_INSTANCE msgInstance, void *callData, void *clientData) {
    *(((Robot*)(clientData))->SVMData) = *((SVM_Operator_Data *)callData);
    IPC_freeData(IPC_msgFormatter(SVM_DATA_RESPONSE), callData);
    return;
}

/**
 * sends a message and subscribes to IPC module
 * @param robot
 */
void subscribeToMessage(Robot *robot) {
    SVM_Operator_Command svmCommand;

    // command line arguments, initialized to reasonable defaults
    int operatorID = robot->svm_operator;
    int priority = 1;
    int ptzatt = SVM_PTZ_DEFAULT;
    float ptzpos[3] = {0.0, 1.571, 0.0};
    int arg = 0;
    int streaming = 100;
    int timing = SVM_TIMING_STOCHASTIC;
    char *hname = NULL;

    // This fills out the data structure used to send a command to SVM
    // publish the command to SVM
    svmCommand.operatorID = (SVM_Operator)operatorID; // which operator to turn on
    svmCommand.priority = priority;                   // priority for the operator, 0 is off, non-zero is on
    svmCommand.timing = (SVM_Operator_Timing)timing;  // stochastic timing (0) or fixed timing (1)
    svmCommand.ptzatt = (SVM_PTZ_Attribute)ptzatt;    // default (0), fixed (1), search (2), tracking(3)
    svmCommand.ptzpos[0] = ptzpos[0];   // pan
    svmCommand.ptzpos[1] = ptzpos[1];   // tilt
    svmCommand.ptzpos[2] = ptzpos[2];   // zoom
    svmCommand.ptzMaxMisses = 0;
    svmCommand.ptzMaxFrames = 0;
    svmCommand.notifyMiss = 1;
    svmCommand.arg = arg;               // operator specific argument
    svmCommand.streaming = streaming;   // 0 (not streaming or the minimum time between
    // broadcasts of detections in ms

    printf("Sending command:\n");
    printf("  operatorID = %d\n", svmCommand.operatorID);
    printf("  priority =   %d\n", svmCommand.priority);
    printf("  ptzatt =     %d\n", svmCommand.ptzatt);
    printf("  ptzpos[0] =  %f\n", svmCommand.ptzpos[0]);
    printf("  ptzpos[1] =  %f\n", svmCommand.ptzpos[1]);
    printf("  ptzpos[2] =  %f\n", svmCommand.ptzpos[2]);
    printf("  MaxMisses =  %d\n", svmCommand.ptzMaxMisses);
    printf("  MaxFrames =  %d\n", svmCommand.ptzMaxFrames);
    printf("  notifyMiss = %d\n", svmCommand.notifyMiss);
    printf("  arg =        %d\n", svmCommand.arg);
    printf("  streaming =  %d\n\n", svmCommand.streaming);

    // this sends the command to SVM
    IPC_publishData(SVM_COMMAND, &svmCommand);

    IPC_subscribeData(SVM_DATA_RESPONSE, IPCMessageHandler, robot);
}

void settingPriorityTo0(Robot *robot) {
    SVM_Operator_Command svmCommand;

    // command line arguments, initialized to reasonable defaults
    int operatorID = robot->svm_previous_operator;
    int priority = 0;
    int ptzatt = SVM_PTZ_DEFAULT;
    float ptzpos[3] = {0.0, 1.571, 0.0};
    int arg = 0;
    int streaming = 100;
    int timing = SVM_TIMING_STOCHASTIC;
    char *hname = NULL;

    // This fills out the data structure used to send a command to SVM
    // publish the command to SVM
    svmCommand.operatorID = (SVM_Operator) operatorID; // which operator to turn on
    svmCommand.priority = priority;                   // priority for the operator, 0 is off, non-zero is on
    svmCommand.timing = (SVM_Operator_Timing) timing;  // stochastic timing (0) or fixed timing (1)
    svmCommand.ptzatt = (SVM_PTZ_Attribute) ptzatt;    // default (0), fixed (1), search (2), tracking(3)
    svmCommand.ptzpos[0] = ptzpos[0];   // pan
    svmCommand.ptzpos[1] = ptzpos[1];   // tilt
    svmCommand.ptzpos[2] = ptzpos[2];   // zoom
    svmCommand.ptzMaxMisses = 0;
    svmCommand.ptzMaxFrames = 0;
    svmCommand.notifyMiss = 1;
    svmCommand.arg = arg;               // operator specific argument
    svmCommand.streaming = streaming;   // 0 (not streaming or the minimum time between
    // broadcasts of detections in ms

    printf("Sending command:\n");
    printf("  operatorID = %d\n", svmCommand.operatorID);
    printf("  priority =   %d\n", svmCommand.priority);
    printf("  ptzatt =     %d\n", svmCommand.ptzatt);
    printf("  ptzpos[0] =  %f\n", svmCommand.ptzpos[0]);
    printf("  ptzpos[1] =  %f\n", svmCommand.ptzpos[1]);
    printf("  ptzpos[2] =  %f\n", svmCommand.ptzpos[2]);
    printf("  MaxMisses =  %d\n", svmCommand.ptzMaxMisses);
    printf("  MaxFrames =  %d\n", svmCommand.ptzMaxFrames);
    printf("  notifyMiss = %d\n", svmCommand.notifyMiss);
    printf("  arg =        %d\n", svmCommand.arg);
    printf("  streaming =  %d\n\n", svmCommand.streaming);

    // this sends the command to SVM
    IPC_publishData(SVM_COMMAND, &svmCommand);
}

/**
 * called everytime at beginning of main loop
 * updates all sensors and listens to IPC
 * @param robot     poitner to Robot
 */
void updateState(Robot *robot) {
	// update sensor readings
	printf("getting sensor readings\n");
    if (robot->connectToIPC) {
        if (IPC_listenClear(100) == 1) {
            robot->IPCLostCount = 0;
        } else {
            robot->IPCLostCount++;
        }
        robot->goalOrientation = robot->SVMData->cols/2;
    }
    if (robot->svm_command_change == 1) {
        settingPriorityTo0(robot);
        subscribeToMessage(robot);
        robot->svm_command_change = 0;
    }
	getVirtualSensorReadings(robot);
	getLaserSensorReadings(robot);
	convertLaserToXY(robot);
    buildOccupancyGrid(robot);
	convertOrientation(robot);
// TODO uncomment
// updatePFSensors(robot);
}

/**
 * creates and inits robot, reads in command line, and connects to IPC if necessary, seeds random number gen
 * @param argc      command line number arg
 * @param argv      command line args
 */
Robot* initialize(int argc, char const *argv[]) {
    Robot *robot = robotCreate();
    robotInit(robot);

    // handle command line args
    if (handleCommandLine(argc, argv, robot) == -1) {
        printf("Command line errors, exiting...\n");
        exit(-1);
    }

    connectRobot(robot->State, MAGE_MODEL_MAGELLAN, (char *) "/dev/ttyUSB0");
    irOn(); // Start up IRs
    sonarOn(); // Start up Sonars
    // catch cntl-c to get a clean shutdown
    signal(SIGINT, sighandler);

    // This connects to the IPC central server using a call to the GCM library
    // init IPC
    if (robot->connectToIPC) {
        GCM_initIPC((char *) "listener", NULL);
        subscribeToMessage(robot);
    }

    // seed random num
    struct timeval t;
    gettimeofday(&t,NULL);
    srand(t.tv_sec);

    return robot;
}

/**
 * Main loop divided into three steps:
 *		state update step
 *		nav step
 *		action step
 */
int main(int argc, char const *argv[]) {
    Robot *robot = initialize(argc,argv);

	// integer values for use in loop
	int vt,wt,go;
	go = 1;
    vt = 0;
    wt = 0;

	printf("entering loop...\n");
    gettimeofday(&(robot->time),NULL);

	while (go) {

		// state update step
		updateState(robot);
		// get vm commands
		switch(robot->task) {
			case TEST:
				handle_test(robot,&vt,&wt);
				break;
			case IDLE:
				handle_idle(robot,&vt,&wt);
				break;
			case POINT_FOLLOW:
				handle_point_follow(robot,&vt,&wt);
				break;
			case WALL_FIND:
				handle_wall_find(robot,&vt,&wt);
				break;
			case WALL_FOLLOW:
				handle_wall_follow(robot,&vt,&wt);
				break;
			case MAZE:
				handle_maze(robot, &vt, &wt);
				break;
			case ROTATE:
				handle_rotate(robot,&vt,&wt);
				break;
			case WANDER:
				handle_wander(robot,&vt,&wt);
				break;
            case GOTO:
                handle_go_to(robot,&vt,&wt);
                break;
            case PINK_FIND:
                handle_pink_find(robot,&vt,&wt);
                break;
            case PINK_TRACK:
                handle_pink_track(robot,&vt,&wt);
                break;
            case HOUGH_WALL_FOLLOW:
                handle_hough_wall_follow(robot,&vt,&wt);
                break;
            case FACE_FIND:
                handle_face_find(robot, &vt, &wt);
                break;
            case FACE_TRACK:
                handle_face_track(robot, &vt, &wt);
                break;
            case TAKE_PICTURE:
                handle_take_picture(robot, &vt, &wt);
                break;
            case LOCALIZE:
                handle_localize(robot, &vt, &wt);
                break;
            case WAIT_FOR_FACE:
                handle_wait_for_face(robot, &vt, &wt);
                break;
            case FOLLOW_PERSON:
                handle_follow_person(robot, &vt, &wt);
                break;
			case QUIT:
				handle_quit(robot,&vt,&wt);
				go = 0;
				break;
			default:
				printf("invalid task, pretending it is IDLE\n");
				handle_idle(robot,&vt,&wt);
				break;
		}
		printf("raw v, w = %d,%d\n", vt,wt);
		// bound vt and wt
		limitVMCommands(robot,&vt,&wt);

		//last minute check for obstacles
		stopForObstacles(robot,&vt,&wt);

		printf("Current task: %d\nCommanding vt: %d wt: %d\n", robot->task,vt,wt);
		vm(vt,wt);

		robot->vt = vt;
		robot->wt = wt;
        robot->iterations++;

        if (!robot->connectToIPC) {
            // sleeping is done in IPC if we are connected
            usleep(100);
        }

		printf("\n");
	}

	printf("finished loop\n");
    if (robot->connectToIPC) {
        GCM_terminateIPC((char *) "Listener");
    }

    irOff();
    sonarOff();
    usleep(500000);
    disconnectRobot();
	freeRobot(robot);
	return 0;
}
