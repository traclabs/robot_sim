/*******************************************************************************
**
**      GSC-18128-1, "Core Flight Executive Version 6.7"
**
**      Copyright (c) 2006-2019 United States Government as represented by
**      the Administrator of the National Aeronautics and Space Administration.
**      All Rights Reserved.
**
**      Licensed under the Apache License, Version 2.0 (the "License");
**      you may not use this file except in compliance with the License.
**      You may obtain a copy of the License at
**
**        http://www.apache.org/licenses/LICENSE-2.0
**
**      Unless required by applicable law or agreed to in writing, software
**      distributed under the License is distributed on an "AS IS" BASIS,
**      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**      See the License for the specific language governing permissions and
**      limitations under the License.
**
** File: robot_sim.c
**
** Purpose:
**   This file contains the source code for the ros App.
**
*******************************************************************************/

/*
** Include Files:
*/
#include "robot_sim_events.h"
#include "robot_sim_version.h"
#include "robot_sim.h"
#include "robot_sim_table.h"

#include <string.h>

#include <math.h>

/*
** global data
*/
RobotSimData_t RobotSimData;
RobotSimData_t RobotSimGoal;
RobotSimTlmState_t StateMsg;
float Kp = 0.01;

void HighRateControLoop(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/* RobotSimMain() -- Application entry point and main process loop         */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void RobotSimMain(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(ROBOT_SIM_PERF_ID);

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = RobotSimInit();

    if (status != CFE_SUCCESS)
    {
        RobotSimData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    /*
    ** ros Runloop
    */
    while (CFE_ES_RunLoop(&RobotSimData.RunStatus) == true)
    {
        /*
        ** Performance Log Exit Stamp
        */
        CFE_ES_PerfLogExit(ROBOT_SIM_PERF_ID);

        /* Pend on receipt of command packet */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, RobotSimData.CommandPipe, CFE_SB_PEND_FOREVER);

        if (status == CFE_SUCCESS)
        {
            RobotSimProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(ROBOT_SIM_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Robot Sim: SB Pipe Read Error, App Will Exit");

            RobotSimData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }

        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(ROBOT_SIM_PERF_ID);
    }

    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(ROBOT_SIM_PERF_ID);

    CFE_ES_ExitApp(RobotSimData.RunStatus);

} /* End of RobotSimMain() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* RobotSimInit() --  initialization                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 RobotSimInit(void)
{
    int32 status;

    RobotSimData.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Initialize app command execution counters
    */
    RobotSimData.CmdCounter = 0;
    RobotSimData.ErrCounter = 0;
    RobotSimData.square_counter = 0;
    RobotSimData.hk_counter = 0;
    RobotSimData.angle = 0.0;

    RobotSimData.HkTlm.Payload.state.joint0 = 0.0;
    RobotSimData.HkTlm.Payload.state.joint1 = 0.0;
    RobotSimData.HkTlm.Payload.state.joint2 = 0.0;
    RobotSimData.HkTlm.Payload.state.joint3 = 0.0;
    RobotSimData.HkTlm.Payload.state.joint4 = 0.0;
    RobotSimData.HkTlm.Payload.state.joint5 = 0.0;
    RobotSimData.HkTlm.Payload.state.joint6 = 0.0;

    /*
    ** Initialize app configuration data
    */
    RobotSimData.PipeDepth = ROBOT_SIM_PIPE_DEPTH;

    strncpy(RobotSimData.PipeName, "ROBOT_SIM_PIPE", sizeof(RobotSimData.PipeName));
    RobotSimData.PipeName[sizeof(RobotSimData.PipeName) - 1] = 0;

    /*
    ** Initialize event filter table...
    */
    RobotSimData.EventFilters[0].EventID = ROBOT_SIM_STARTUP_INF_EID;
    RobotSimData.EventFilters[0].Mask    = 0x0000;
    RobotSimData.EventFilters[1].EventID = ROBOT_SIM_COMMAND_ERR_EID;
    RobotSimData.EventFilters[1].Mask    = 0x0000;
    RobotSimData.EventFilters[2].EventID = ROBOT_SIM_COMMANDNOP_INF_EID;
    RobotSimData.EventFilters[2].Mask    = 0x0000;
    RobotSimData.EventFilters[3].EventID = ROBOT_SIM_COMMANDJNT_INF_EID;
    RobotSimData.EventFilters[3].Mask    = 0x0000;
    RobotSimData.EventFilters[4].EventID = ROBOT_SIM_INVALID_MSGID_ERR_EID;
    RobotSimData.EventFilters[4].Mask    = 0x0000;
    RobotSimData.EventFilters[5].EventID = ROBOT_SIM_LEN_ERR_EID;
    RobotSimData.EventFilters[5].Mask    = 0x0000;
    RobotSimData.EventFilters[6].EventID = ROBOT_SIM_PIPE_ERR_EID;
    RobotSimData.EventFilters[6].Mask    = 0x0000;

    status = CFE_EVS_Register(RobotSimData.EventFilters, ROBOT_SIM_EVENT_COUNTS, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Robot Sim: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Initialize housekeeping packet (clear user data area).
    */
    CFE_MSG_Init(&RobotSimData.HkTlm.TlmHeader.Msg, CFE_SB_ValueToMsgId(ROBOT_SIM_HK_TLM_MID), sizeof(RobotSimData.HkTlm));
    CFE_MSG_Init(&StateMsg.TlmHeader.Msg, CFE_SB_ValueToMsgId(ROBOT_SIM_STATE_TLM_MID), sizeof(RobotSimTlmState_t));

    /*
    ** Create Software Bus message pipe.
    */
    status = CFE_SB_CreatePipe(&RobotSimData.CommandPipe, RobotSimData.PipeDepth, RobotSimData.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Robot Sim: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Subscribe to Housekeeping request commands
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ROBOT_SIM_SEND_HK_MID), RobotSimData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Robot Sim: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Subscribe to ground command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ROBOT_SIM_CMD_MID), RobotSimData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Robot Sim: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }
    
    /*
    ** Subscribe to HR wakeup
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ROBOT_SIM_HR_CONTROL_MID), RobotSimData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Robot Sim: Error Subscribing to HR Wakeup Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }

    CFE_EVS_SendEvent(ROBOT_SIM_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "Robot Sim Initialized.%s",
                      ROBOT_SIM_VERSION_STRING);

    return (CFE_SUCCESS);

} /* End of RobotSimInit() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  RobotSimProcessCommandPacket                                    */
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the ros    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void RobotSimProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
    //printf("RobotSimProcessCommandPacket() -- we're processing the cmd from MID: 0x%04x\n", CFE_SB_MsgIdToValue(MsgId));
    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case ROBOT_SIM_CMD_MID:
            RobotSimProcessGroundCommand(SBBufPtr);
            break;

        case ROBOT_SIM_SEND_HK_MID:
            RobotSimReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case ROBOT_SIM_HR_CONTROL_MID:
            HighRateControLoop();
            break;
            
        default:
            CFE_EVS_SendEvent(ROBOT_SIM_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "robot sim: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }

    return;

} /* End RobotSimProcessCommandPacket */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* RobotSimProcessGroundCommand() -- robot sim ground commands                */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void RobotSimProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    printf("RobotSimProcessGroundCommand() -- we're getting a ground command...%d\n", CommandCode);

    /*
    ** Process "known" ros app ground commands
    */
    switch (CommandCode)
    {
        case ROBOT_SIM_NOOP_CC:
            if (RobotSimVerifyCmdLength(&SBBufPtr->Msg, sizeof(RobotSimNoopCmd_t)))
            {
                RobotSimNoop((RobotSimNoopCmd_t *)SBBufPtr);
            }

            break;

        case ROBOT_SIM_SET_JOINTS_CC:
            // if (RobotSimVerifyCmdLength(&SBBufPtr->Msg, sizeof(RobotSimJointStateCmd_t)))
            {
                RobotSimCmdJointState((RobotSimJointStateCmd_t *)SBBufPtr);
            }

            break;

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(ROBOT_SIM_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }


    return;

} /* End of RobotSimProcessGroundCommand() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  RobotSimReportHousekeeping                                          */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 RobotSimReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{
    printf("RobotSimReportHousekeeping() -- sending joint states as part of housekeeping...\n");
    
    /*
    ** Get command execution counters...
    */
    RobotSimData.HkTlm.Payload.CommandErrorCounter = RobotSimData.ErrCounter*2;
    RobotSimData.ErrCounter++;
    RobotSimData.HkTlm.Payload.CommandCounter      = RobotSimData.CmdCounter++;

    /*
    ** Send housekeeping telemetry packet...
    */
    // RobotSimData.HkTlm.Payload.state.joint0 = sin(RobotSimData.angle);
    // RobotSimData.HkTlm.Payload.state.joint1 = sin(RobotSimData.angle);
    // RobotSimData.HkTlm.Payload.state.joint2 = sin(RobotSimData.angle);
    // RobotSimData.HkTlm.Payload.state.joint3 = sin(RobotSimData.angle);
    // RobotSimData.HkTlm.Payload.state.joint4 = sin(RobotSimData.angle);
    // RobotSimData.HkTlm.Payload.state.joint5 = sin(RobotSimData.angle);
    // RobotSimData.HkTlm.Payload.state.joint6 = sin(RobotSimData.angle);
    RobotSimData.angle += 0.043; // 5 deg per publish


    // RobotSimData.HkTlm.Payload.state.joint0 = 0.2;
    // RobotSimData.HkTlm.Payload.state.joint1 = 0.0;
    // RobotSimData.HkTlm.Payload.state.joint2 = 0.0;
    // RobotSimData.HkTlm.Payload.state.joint3 = -.3;
    // RobotSimData.HkTlm.Payload.state.joint4 = 0.0;
    // RobotSimData.HkTlm.Payload.state.joint5 = 0.0;
    // RobotSimData.HkTlm.Payload.state.joint6 = 0.27;



    OS_printf("\nRobotSimReportHousekeeping [%d]:\n---------------------\n", (int)RobotSimData.hk_counter);
    OS_printf("joint0: %f\n", RobotSimData.HkTlm.Payload.state.joint0);
    OS_printf("joint1: %f\n", RobotSimData.HkTlm.Payload.state.joint1);
    OS_printf("joint2: %f\n", RobotSimData.HkTlm.Payload.state.joint2);
    OS_printf("joint3: %f\n", RobotSimData.HkTlm.Payload.state.joint3);
    OS_printf("joint4: %f\n", RobotSimData.HkTlm.Payload.state.joint4);
    OS_printf("joint5: %f\n", RobotSimData.HkTlm.Payload.state.joint5);
    OS_printf("joint6: %f\n", RobotSimData.HkTlm.Payload.state.joint6);


    // if (RobotSimData.hk_counter%4 == 0)
    {
        OS_printf("++++++++++++++++++++ sending robot HK data\n");
        CFE_SB_TimeStampMsg(&RobotSimData.HkTlm.TlmHeader.Msg);
        CFE_SB_TransmitMsg(&RobotSimData.HkTlm.TlmHeader.Msg, true);
        RobotSimData.hk_counter = 0;
    }
    RobotSimData.hk_counter++;


    return CFE_SUCCESS;

} /* End of RobotSimReportHousekeeping() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* RobotSimNoop -- ROS NOOP commands                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 RobotSimNoop(const RobotSimNoopCmd_t *Msg)
{
    RobotSimData.CmdCounter++;

    CFE_EVS_SendEvent(ROBOT_SIM_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "robot sim: NOOP command %s",
                      ROBOT_SIM_VERSION);

    return CFE_SUCCESS;
} /* End of RobotSimNoop */


int32 RobotSimCmdJointState(const RobotSimJointStateCmd_t *Msg)
{
    RobotSimData.CmdCounter++;

    OS_printf("RobotSimData.CmdCounter: %d\n", RobotSimData.CmdCounter);
    OS_printf("RobotSimData.CmdCounter mod: %d\n", (RobotSimData.CmdCounter%100));

    // RobotSimGoal.HkTlm.Payload.state.joint0 = 0.0;//Msg->joint0;
    // RobotSimGoal.HkTlm.Payload.state.joint1 = 0.0;//Msg->joint1; 
    // RobotSimGoal.HkTlm.Payload.state.joint2 = 0.0;//Msg->joint2;
    // // RobotSimGoal.HkTlm.Payload.state.joint3 = 0.0;//Msg->joint3;
    // RobotSimGoal.HkTlm.Payload.state.joint4 = 0.0;//Msg->joint4;
    // RobotSimGoal.HkTlm.Payload.state.joint5 = 0.0;//Msg->joint5;
    // RobotSimGoal.HkTlm.Payload.state.joint6 = 0.0;//Msg->joint6;

    RobotSimGoal.HkTlm.Payload.state.joint0 = Msg->joint0;
    RobotSimGoal.HkTlm.Payload.state.joint1 = Msg->joint1; 
    RobotSimGoal.HkTlm.Payload.state.joint2 = Msg->joint2;
    RobotSimGoal.HkTlm.Payload.state.joint3 = Msg->joint3;
    RobotSimGoal.HkTlm.Payload.state.joint4 = Msg->joint4;
    RobotSimGoal.HkTlm.Payload.state.joint5 = Msg->joint5;
    RobotSimGoal.HkTlm.Payload.state.joint6 = Msg->joint6;


#if 1
    OS_printf("\nGoal:\n---------------------\n");
    OS_printf("joint0: %f\n", RobotSimGoal.HkTlm.Payload.state.joint0);
    OS_printf("joint1: %f\n", RobotSimGoal.HkTlm.Payload.state.joint1);
    OS_printf("joint2: %f\n", RobotSimGoal.HkTlm.Payload.state.joint2);
    OS_printf("joint3: %f\n", RobotSimGoal.HkTlm.Payload.state.joint3);
    OS_printf("joint4: %f\n", RobotSimGoal.HkTlm.Payload.state.joint4);
    OS_printf("joint5: %f\n", RobotSimGoal.HkTlm.Payload.state.joint5);
    OS_printf("joint6: %f\n", RobotSimGoal.HkTlm.Payload.state.joint6);
#endif

    CFE_EVS_SendEvent(ROBOT_SIM_COMMANDJNT_INF_EID, CFE_EVS_EventType_INFORMATION, "robot sim: joint state command %s",
                      ROBOT_SIM_VERSION);

    return CFE_SUCCESS;
    
}

void HighRateControLoop(void) {

    // RobotSimData.square_counter++;
    // if (RobotSimData.square_counter < 500)
    // {
    //     RobotSimGoal.HkTlm.Payload.state.joint3 = -1.0;//Msg->joint3;
    // }
    // else if (RobotSimData.square_counter < 1000)
    // {
    //     RobotSimGoal.HkTlm.Payload.state.joint3 = 1.0;//Msg->joint3;
    // }
    // else if (RobotSimData.square_counter == 1000)
    // {
    //     RobotSimData.square_counter = 0;   
    // }

    // if (RobotSimData.square_counter%100 == 0)
    // {
    //     OS_printf("counter: %d,  j: %f\n", (int)RobotSimData.square_counter, RobotSimGoal.HkTlm.Payload.state.joint3);
    // }

    // RobotSimTlmState_t *st = &StateMsg; //RobotSimGoal.StateTlm;
    RobotSimTlmState_t *st = &StateMsg; //RobotSimGoal.StateTlm;

    st->errors[0] = (RobotSimGoal.HkTlm.Payload.state.joint0 - RobotSimData.HkTlm.Payload.state.joint0);
    st->errors[1] = (RobotSimGoal.HkTlm.Payload.state.joint1 - RobotSimData.HkTlm.Payload.state.joint1);
    st->errors[2] = (RobotSimGoal.HkTlm.Payload.state.joint2 - RobotSimData.HkTlm.Payload.state.joint2);
    st->errors[3] = (RobotSimGoal.HkTlm.Payload.state.joint3 - RobotSimData.HkTlm.Payload.state.joint3);
    st->errors[4] = (RobotSimGoal.HkTlm.Payload.state.joint4 - RobotSimData.HkTlm.Payload.state.joint4);
    st->errors[5] = (RobotSimGoal.HkTlm.Payload.state.joint5 - RobotSimData.HkTlm.Payload.state.joint5);
    st->errors[6] = (RobotSimGoal.HkTlm.Payload.state.joint6 - RobotSimData.HkTlm.Payload.state.joint6);

#if 1
    OS_printf("\n---------------------\n");
    OS_printf("---------------------\n");
    OS_printf("---------------------\n");

    OS_printf("\nCurrent:\n---------------------\n");
    OS_printf("joint0: %f\n", RobotSimData.HkTlm.Payload.state.joint0);
    OS_printf("joint1: %f\n", RobotSimData.HkTlm.Payload.state.joint1);
    OS_printf("joint2: %f\n", RobotSimData.HkTlm.Payload.state.joint2);
    OS_printf("joint3: %f\n", RobotSimData.HkTlm.Payload.state.joint3);
    OS_printf("joint4: %f\n", RobotSimData.HkTlm.Payload.state.joint4);
    OS_printf("joint5: %f\n", RobotSimData.HkTlm.Payload.state.joint5);
    OS_printf("joint6: %f\n", RobotSimData.HkTlm.Payload.state.joint6);

    OS_printf("\nError:\n---------------------\n");
    OS_printf("joint0: %f\n", st->errors[0]);
    OS_printf("joint1: %f\n", st->errors[1]);
    OS_printf("joint2: %f\n", st->errors[2]);
    OS_printf("joint3: %f\n", st->errors[3]);
    OS_printf("joint4: %f\n", st->errors[4]);
    OS_printf("joint5: %f\n", st->errors[5]);
    OS_printf("joint6: %f\n", st->errors[6]);
#endif
    
    RobotSimData.HkTlm.Payload.state.joint0 = RobotSimData.HkTlm.Payload.state.joint0 + Kp * st->errors[0];
    RobotSimData.HkTlm.Payload.state.joint1 = RobotSimData.HkTlm.Payload.state.joint1 + Kp * st->errors[1];
    RobotSimData.HkTlm.Payload.state.joint2 = RobotSimData.HkTlm.Payload.state.joint2 + Kp * st->errors[2];
    RobotSimData.HkTlm.Payload.state.joint3 = RobotSimData.HkTlm.Payload.state.joint3 + Kp * st->errors[3];
    RobotSimData.HkTlm.Payload.state.joint4 = RobotSimData.HkTlm.Payload.state.joint4 + Kp * st->errors[4];
    RobotSimData.HkTlm.Payload.state.joint5 = RobotSimData.HkTlm.Payload.state.joint5 + Kp * st->errors[5];
    RobotSimData.HkTlm.Payload.state.joint6 = RobotSimData.HkTlm.Payload.state.joint6 + Kp * st->errors[6];

#if 0
    printf("\nDesired:\n---------------------\n");
    // printf("joint0: %f\n", RobotSimData.HkTlm.Payload.state.joint0);
    // printf("joint1: %f\n", RobotSimData.HkTlm.Payload.state.joint1);
    // printf("joint2: %f\n", RobotSimData.HkTlm.Payload.state.joint2);
    printf("joint3: %f\n", RobotSimData.HkTlm.Payload.state.joint3);
    // printf("joint4: %f\n", RobotSimData.HkTlm.Payload.state.joint4);
    // printf("joint5: %f\n", RobotSimData.HkTlm.Payload.state.joint5);
    // printf("joint6: %f\n", RobotSimData.HkTlm.Payload.state.joint6);
#endif

    // st->errors[3] = RobotSimGoal.HkTlm.Payload.state.joint3;

    st->Kp = Kp;
    memcpy(&st->joints, &RobotSimData.HkTlm.Payload.state, sizeof(RobotSimSSRMS_t) );
    
    // if (RobotSimData.square_counter%1000 == 0)
    {
    CFE_SB_TimeStampMsg(&st->TlmHeader.Msg);
    CFE_SB_TransmitMsg(&st->TlmHeader.Msg, true);
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* RobotSimVerifyCmdLength() -- Verify command packet length                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool RobotSimVerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    size_t            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_FcnCode_t FcnCode      = 0;

    printf("RobotSimVerifyCmdLength() --\n");

    CFE_MSG_GetSize(MsgPtr, &ActualLength);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        CFE_MSG_GetMsgId(MsgPtr, &MsgId);
        CFE_MSG_GetFcnCode(MsgPtr, &FcnCode);

        CFE_EVS_SendEvent(ROBOT_SIM_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        RobotSimData.ErrCounter++;
    }

    return (result);

} /* End of RobotSimVerifyCmdLength() */
