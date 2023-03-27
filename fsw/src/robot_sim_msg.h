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
** File: robot_sim_msg.h
**
** Purpose:
**  Define Robot Sim Messages and info
**
** Notes:
**
**
*******************************************************************************/
#ifndef _robot_sim_msg_h_
#define _robot_sim_msg_h_

/*
** Robot Sim command codes
*/
#define ROBOT_SIM_NOOP_CC           0
#define ROBOT_SIM_SET_JOINTS_CC     1

/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader; /**< \brief Command header */
} RobotSimNoArgsCmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader; /**< \brief Command header */
    float joint0;
    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;
} RobotSimJointCmd_t;

/*
** The following commands all share the "NoArgs" format
**
** They are each given their own type name matching the command name, which
** allows them to change independently in the future without changing the prototype
** of the handler function
*/
typedef RobotSimNoArgsCmd_t RobotSimNoopCmd_t;
typedef RobotSimJointCmd_t  RobotSimJointStateCmd_t;

/*************************************************************************/
/*
** Type definition (Robot Sim housekeeping)
*/

typedef struct
{
    uint8 index;
    float position;
} RobotSimJoint_t;

typedef struct
{
    float joint0;
    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;
} RobotSimSSRMS_t;

typedef struct
{
    uint8 CommandErrorCounter;
    uint8 CommandCounter;
    RobotSimSSRMS_t state;
} RobotSimHkTlmPayload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TlmHeader; /**< \brief Telemetry header */
    RobotSimHkTlmPayload_t Payload;   /**< \brief Telemetry payload */
} RobotSimHkTlm_t;

#endif /* _robot_sim_msg_h_ */

/************************/
/*  End of File Comment */
/************************/
