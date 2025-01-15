/**********************************************************************
* Copyright (C) 2020  NATORS£¬All Rights Reserved
*
* File name: SCANControl.h
* Author   : Nators
* Version  : 1.4.7
*
* This is the software interface to the Nano Positioning System.
* Please refer to the Programmer's Guide for a detailed documentation.
*
* THIS  SOFTWARE, DOCUMENTS, FILES AND INFORMATION ARE PROVIDED 'AS IS'
* WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
* BUT  NOT  LIMITED  TO,  THE  IMPLIED  WARRANTIES  OF MERCHANTABILITY,
* FITNESS FOR A PURPOSE, OR THE WARRANTY OF NON-INFRINGEMENT.
* THE  ENTIRE  RISK  ARISING OUT OF USE OR PERFORMANCE OF THIS SOFTWARE
* REMAINS WITH YOU.
* IN  NO  EVENT  SHALL  THE  SMARACT  GMBH  BE  LIABLE  FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL, CONSEQUENTIAL OR OTHER DAMAGES ARISING
* OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE.
**********************************************************************/
#ifndef SCANCONTROL_H
#define SCANCONTROL_H

#ifdef SCANCONTROL_EXPORTS
	#define SCANCONTROL_API __declspec(dllexport)
#else
	#define SCANCONTROL_API __declspec(dllimport)
#endif

#define SCANCONTROL_CC  __cdecl


typedef unsigned int SCAN_STATUS;
typedef unsigned int SCAN_INDEX;
typedef unsigned int SCAN_PACKET_TYPE;

typedef struct SCAN_packet {
	SCAN_PACKET_TYPE packetType; // type of packet
	SCAN_INDEX channelIndex; // source channel
	unsigned int data1; // data field
	signed int data2; // data field
	signed int data3; // data field
	unsigned int data4; // data field
} SCAN_PACKET;


// function status return values
#define SCAN_OK                                       0
#define SCAN_INITIALIZATION_ERROR                     1
#define SCAN_NOT_INITIALIZED_ERROR                    2
#define SCAN_NO_SYSTEMS_FOUND_ERROR                   3
#define SCAN_TOO_MANY_SYSTEMS_ERROR                   4
#define SCAN_INVALID_SYSTEM_INDEX_ERROR               5
#define SCAN_INVALID_CHANNEL_INDEX_ERROR              6
#define SCAN_TRANSMIT_ERROR                           7
#define SCAN_WRITE_ERROR                              8
#define SCAN_INVALID_PARAMETER_ERROR                  9
#define SCAN_READ_ERROR                               10
#define SCAN_INTERNAL_ERROR                           12
#define SCAN_WRONG_MODE_ERROR                         13
#define SCAN_PROTOCOL_ERROR                           14
#define SCAN_TIMEOUT_ERROR                            15
#define SCAN_ID_LIST_TOO_SMALL_ERROR                  17
#define SCAN_SYSTEM_ALREADY_ADDED_ERROR               18
#define SCAN_WRONG_CHANNEL_TYPE_ERROR                 19
#define SCAN_CANCELED_ERROR                           20
#define SCAN_INVALID_SYSTEM_LOCATOR_ERROR             21
#define SCAN_INPUT_BUFFER_OVERFLOW_ERROR              22
#define SCAN_QUERYBUFFER_SIZE_ERROR                   23
#define SCAN_DRIVER_ERROR                             24
#define SCAN_NO_SENSOR_PRESESCAN_ERROR                  129
#define SCAN_AMPLITUDE_TOO_LOW_ERROR                  130
#define SCAN_AMPLITUDE_TOO_HIGH_ERROR                 131
#define SCAN_FREQUENCY_TOO_LOW_ERROR                  132
#define SCAN_FREQUENCY_TOO_HIGH_ERROR                 133
#define SCAN_SCAN_TARGET_TOO_HIGH_ERROR               135
#define SCAN_SCAN_SPEED_TOO_LOW_ERROR                 136
#define SCAN_SCAN_SPEED_TOO_HIGH_ERROR                137
#define SCAN_SENSOR_DISABLED_ERROR                    140
#define SCAN_COMMAND_OVERRIDDEN_ERROR                 141
#define SCAN_END_STOP_REACHED_ERROR                   142
#define SCAN_WRONG_SENSOR_TYPE_ERROR                  143
#define SCAN_COULD_NOT_FIND_REF_ERROR                 144
#define SCAN_WRONG_END_EFFECTOR_TYPE_ERROR            145
#define SCAN_MOVEMESCAN_LOCKED_ERROR                    146
#define SCAN_RANGE_LIMIT_REACHED_ERROR                147
#define SCAN_PHYSICAL_POSITION_UNKNOWN_ERROR          148
#define SCAN_OUTPUT_BUFFER_OVERFLOW_ERROR             149
#define SCAN_COMMAND_NOT_PROCESSABLE_ERROR            150
#define SCAN_WAITING_FOR_TRIGGER_ERROR                151
#define SCAN_COMMAND_NOT_TRIGGERABLE_ERROR            152
#define SCAN_COMMAND_QUEUE_FULL_ERROR                 153
#define SCAN_INVALID_COMPONESCAN_ERROR                  154
#define SCAN_INVALID_SUB_COMPONESCAN_ERROR              155
#define SCAN_INVALID_PROPERTY_ERROR                   156
#define SCAN_PERMISSION_DENIED_ERROR                  157
#define SCAN_CALIBRATION_FAILED_ERROR                 160
#define SCAN_UNKNOWN_COMMAND_ERROR                    240
#define SCAN_OTHER_ERROR                              255
#define SCAN_DATA_ERROR                               25

// general definitions
#define SCAN_UNDEFINED                                0
#define SCAN_FALSE                                    0
#define SCAN_TRUE                                     1
#define SCAN_DISABLED                                 0
#define SCAN_ENABLED                                  1
#define SCAN_FALLING_EDGE                             0
#define SCAN_RISING_EDGE                              1
#define SCAN_FORWARD                                  0
#define SCAN_BACKWARD                                 1


// configuration flags for SCAN_InitDevices
#define SCAN_SYNCHRONOUS_COMMUNICATION                0
#define SCAN_ASYNCHRONOUS_COMMUNICATION               1
#define SCAN_HARDWARE_RESET                           2

// return values from SCAN_GetInitState
#define SCAN_INIT_STATE_NONE                          0
#define SCAN_INIT_STATE_SYNC                          1
#define SCAN_INIT_STATE_ASYNC                         2

// Hand Control Module modes for SCAN_SetHCMEnabled
#define SCAN_HCM_DISABLED                             0
#define SCAN_HCM_ENABLED                              1

// configuration values for SCAN_SetAccumulateRelativePositions_X
#define SCAN_NO_ACCUMULATE_RELATIVE_POSITIONS         0
#define SCAN_ACCUMULATE_RELATIVE_POSITIONS            1

// configuration values for SCAN_SetSensorEnabled_X
#define SCAN_SENSOR_DISABLED                          0
#define SCAN_SENSOR_ENABLED                           1

// infinite timeout for functions that wait
#define SCAN_TIMEOUT_INFINITE                         0xFFFFFFFF

// packet types for asynchronous mode
#define SCAN_NO_PACKET_TYPE                           0
#define SCAN_ERROR_PACKET_TYPE                        1
#define SCAN_POSITION_PACKET_TYPE                     2
#define SCAN_COMPLETED_PACKET_TYPE                    3
#define SCAN_STATUS_PACKET_TYPE                       4
#define SCAN_ANGLE_PACKET_TYPE                        5
#define SCAN_VOLTAGE_LEVEL_PACKET_TYPE                6
#define SCAN_SENSOR_TYPE_PACKET_TYPE                  7
#define SCAN_SENSOR_ENABLED_PACKET_TYPE               8
#define SCAN_END_EFFECTOR_TYPE_PACKET_TYPE            9
#define SCAN_GRIPPER_OPENING_PACKET_TYPE              10
#define SCAN_FORCE_PACKET_TYPE                        11
#define SCAN_MOVE_SPEED_PACKET_TYPE                   12
#define SCAN_PHYSICAL_POSITION_KNOWN_PACKET_TYPE      13
#define SCAN_POSITION_LIMIT_PACKET_TYPE               14
#define SCAN_ANGLE_LIMIT_PACKET_TYPE                  15
#define SCAN_SAFE_DIRECTION_PACKET_TYPE               16
#define SCAN_SCALE_PACKET_TYPE                        17
#define SCAN_MOVE_ACCELERATION_PACKET_TYPE            18
#define SCAN_CHANNEL_PROPERTY_PACKET_TYPE             19
#define SCAN_CAPTURE_BUFFER_PACKET_TYPE               20
#define SCAN_TRIGGERED_PACKET_TYPE                    21
#define SCAN_INVALID_PACKET_TYPE                      255

// channel status codes
#define SCAN_STOPPED_STATUS                           0
#define SCAN_STEPPING_STATUS                          1
#define SCAN_SCANNING_STATUS                          2
#define SCAN_HOLDING_STATUS                           3
#define SCAN_TARGET_STATUS                            4
#define SCAN_SENSOR_CLOSED_STATUS                     5
#define SCAN_LIMIT_POSITION_STATUS                    10
#define SCAN_SHORT_CIRCUIT_STATUS                     11


#ifdef __cplusplus
extern "C" {
#endif
	/***********************************************************************
	General note:
	All functions have a return value of SCAN_STATUS
	indicating success (SCAN_OK) or failure of execution. See the above
	definitions for a list of error codes.
	***********************************************************************/

	/************************************************************************
	*************************************************************************
	**                 Section I: Initialization Functions                 **
	*************************************************************************
	************************************************************************/

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetDLLVersion(unsigned int* version);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_OpenSystem(SCAN_INDEX* systemIndex, const char* systemLocator, const char* options);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_CloseSystem(SCAN_INDEX systemIndex);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_FindSystems(const char* options, char* outBuffer, unsigned int* ioBufferSize);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetSystemLocator(SCAN_INDEX systemIndex, char* outBuffer, unsigned int* ioBufferSize);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_SetHCMEnabled(SCAN_INDEX systemIndex, unsigned int enabled);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetNumberOfChannels(SCAN_INDEX systemIndex, unsigned int* channels);

	/************************************************************************
	*************************************************************************
	**        Section IIa:  Functions for SYNCHRONOUS communication        **
	*************************************************************************
	************************************************************************/
	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_StepMove_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int steps, unsigned int amplitude, unsigned int frequency);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_Stop_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GotoPositionAbsolute_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int position, unsigned int holdTime);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GotoPositionRelative_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int diff, unsigned int holdTime);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetStatus_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, unsigned int* status);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_SetPosition_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int position);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetPosition_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int* position);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_SetSensorEnabled_S(SCAN_INDEX systemIndex, unsigned int enabled);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetSensorEnabled_S(SCAN_INDEX systemIndex, unsigned int* enabled);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetVoltageLevel_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, unsigned int* level);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC  SCAN_ScanMoveRelative_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int diff, unsigned int sacnStep, unsigned int scanDelay);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_ScanMoveAbsolute_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, unsigned int target, unsigned int scanStep, unsigned int scanDelay);
	
	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_ScanMoveNormal_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, unsigned int voltage);
	
	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_SetAccumulateRelativePositions_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, unsigned int accumulate);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC  SCAN_SetVoltageOscillation_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, unsigned int minvoltage, unsigned int maxvoltage, float frequency);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC  SCAN_StopOscillation_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC  SCAN_MoveOscillation_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int addvoltage);

	SCANCONTROL_API
	    SCAN_STATUS SCANCONTROL_CC  SCAN_SetOscillationFrequency_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, float ferquency);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_SetOscillationMinvoltage_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, unsigned int minvoltage);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_SetOscillationMaxvoltage_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, unsigned int maxvoltage);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetVoltageOscillation_S(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, unsigned int* maxvoltage);
	/************************************************************************
	*************************************************************************
	**       Section IIb:  Functions for ASYNCHRONOUS communication        **
	*************************************************************************
	************************************************************************/
	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_StepMove_A(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int steps, unsigned int amplitude, unsigned int frequency);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_Stop_A(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetStatus_A(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GotoPositionRelative_A(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int diff, unsigned int holdTime);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GotoPositionAbsolute_A(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int position, unsigned int holdTime);
		
	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetPosition_A(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_SetPosition_A(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int position);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_SetSensorEnabled_A(SCAN_INDEX systemIndex, unsigned int enabled);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetSensorEnabled_A(SCAN_INDEX systemIndex);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_GetVoltageLevel_A(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_ScanMoveRelative_A(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, signed int diff, unsigned int scanSpeed);
		
	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_ScanMoveAbsolute_A(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, unsigned int target, unsigned int scanSpeed);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_SetAccumulateRelativePositions_A(SCAN_INDEX systemIndex, SCAN_INDEX channelIndex, unsigned int accumulate);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_ReceiveNextPacket_A(SCAN_INDEX systemIndex, unsigned int timeout, SCAN_PACKET* packet);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_CancelWaitForPacket_A(SCAN_INDEX systemIndex);

	SCANCONTROL_API
		SCAN_STATUS SCANCONTROL_CC SCAN_DiscardPacket_A(SCAN_INDEX systemIndex);

#ifdef __cplusplus
}
#endif 

#endif   /* NTCONTROL_H */ 