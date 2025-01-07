//! \file SystemInterface_EtherCAT_EthercatCore.h
//!
//! \brief Automatically generated header file for the EtherCAT system interface
//!
// This file is part of NRMKPlatform SDK, Windows-based development tool and SDK
// for Real-time Linux Embedded EtherCAT master controller (STEP)
//
// Copyright (C) 2013-2016 Neuromeka <http://www.neuromeka.com>

#ifndef SYSTEMINTERFACE_ETHERCAT_ETHERCATCORE_H_
#define SYSTEMINTERFACE_ETHERCAT_ETHERCATCORE_H_

#pragma once 

//EtherCAT Library ******************************************************************
#include "ecrt.h"

#include "CoE.h"

#include "PDOConfig.h"
#include <stdio.h>
#include <memory>

class SystemInterface_EtherCAT_EthercatCore
{ 
	public:
		enum
		{
			NUM_NEUROMEKA_NRMK_DRIVE_AXES = 1,
		};
		
		enum
		{
			NUM_MOTION_AXIS = NUM_NEUROMEKA_NRMK_DRIVE_AXES,
			MAX_TORQUE = 2000,
		};
								
		struct NEUROMEKA_NRMK_DRIVE_IN
			{
				UINT16		Controlword; 	// 0x6040
				INT32		Targetposition; 	// 0x607a
				INT32		Targetvelocity; 	// 0x60ff
				INT16		Targettorque; 	// 0x6071
				INT8		Modesofoperation; 	// 0x6060				
			};
			struct NEUROMEKA_NRMK_DRIVE_OUT
			{
				UINT16		Statusword; 	// 0x6041
				INT32		Positionactualvalue; 	// 0x6064
				INT32		Velocityactualvalue; 	// 0x606c
				INT16		Torqueactualvalue; 	// 0x6077
				INT8		Modesofoperationdisplay; 	// 0x6061				
			};
			struct NEUROMEKA_NRMK_DRIVE
			{
				UINT32 Index;
				UINT32 Alias;
				UINT32 Position;
				SLAVE_CONFIG Config;
				
				NEUROMEKA_NRMK_DRIVE_IN 	InParam;
				NEUROMEKA_NRMK_DRIVE_OUT 	OutParam;
				
				UINT32 offControlword;
				UINT32 offTargetposition;
				UINT32 offTargetvelocity;
				UINT32 offTargettorque;
				UINT32 offModesofoperation;
				UINT32 offStatusword;
				UINT32 offPositionactualvalue;
				UINT32 offVelocityactualvalue;
				UINT32 offTorqueactualvalue;
				UINT32 offModesofoperationdisplay;
				UINT32 bitoffControlword;
				UINT32 bitoffTargetposition;
				UINT32 bitoffTargetvelocity;
				UINT32 bitoffTargettorque;
				UINT32 bitoffModesofoperation;
				UINT32 bitoffStatusword;
				UINT32 bitoffPositionactualvalue;
				UINT32 bitoffVelocityactualvalue;
				UINT32 bitoffTorqueactualvalue;
				UINT32 bitoffModesofoperationdisplay;								
				
			};
			

		typedef std::auto_ptr<struct ecat_variables> EcatSystemVars;
		
	public: 
		SystemInterface_EtherCAT_EthercatCore();
		~SystemInterface_EtherCAT_EthercatCore();
		
		int init(INT8 ModeOp, UINT32 CycleNano)
		{
			_setMasterCycle(CycleNano);
				
			// TODO: Initiate Axes' parameters here
			for (int i=0; i<NUM_NEUROMEKA_NRMK_DRIVE_AXES; i++)
			{
				_systemReady[_neuromeka_NRMK_Drive[i].Index]=0;
				_servoOn[_neuromeka_NRMK_Drive[i].Index] = false;
				// TODO: Init params here. 
				_neuromeka_NRMK_Drive[i].InParam.Modesofoperation = ModeOp;															
			}
			
			
			if (_initMaster() < 0)
			{
				printf("Init Master Failed.\n");
				return -1;
			}					

			if (_initSlaves() == -1)
			{
				printf("Init Slaves Failed.\n");
				deinit();
				return -1;
			}

			if (_initDomains() == -1)
			{
				printf("Init Domains Failed.\n");
				deinit();
				return -1;
			}

			if (_activateMaster() == -1)
			{
				deinit();
				return -1;
			}

			return 0;
		}

		int deinit();
	
		void processTxDomain();
		void processRxDomain();
		
		void readBuffer(int EntryID, void * const data);
		void writeBuffer(int EntryID, void * const data);
		
		void syncEcatMaster();
		
		void readSDO(int EntryID, void * const data);
		void writeSDO(int EntryID, void * const data);
		
		int getRxDomainStatus();
		int getTxDomainStatus();
		int getMasterStatus(unsigned int & NumSlaves, unsigned int & State);
		int getAxisEcatStatus(unsigned int AxisIdx, unsigned int & State);
		
		
		void setServoOn(unsigned int AxisIdx)
		{
			_servoOn[AxisIdx] = true;
		}
		void setServoOff(unsigned int AxisIdx)
		{
			_servoOn[AxisIdx] = false;
		}
		bool isServoOn(unsigned int AxisIdx)
		{
			return _servoOn[AxisIdx];
		}
	
		bool isSystemReady()
		{
			for (int i=0; i<NUM_MOTION_AXIS; ++i)
				if (!_systemReady[i])
					return false;

			return true;
		}
	
	private:
		void _setMasterCycle(UINT32 DCCycle);
		int	_initMaster();
		int _initSlaves();
		int _initDomains();		
		int _activateMaster();
		
	private: 
		EcatSystemVars _systemVars;
		
		bool _servoOn[NUM_MOTION_AXIS];
		unsigned int _systemReady[NUM_MOTION_AXIS];	
		
		/* EtherCAT Slaves */
		NEUROMEKA_NRMK_DRIVE _neuromeka_NRMK_Drive[NUM_NEUROMEKA_NRMK_DRIVE_AXES];
						
}; 

#endif /* SYSTEMINTERFACE_ETHERCAT_ETHERCATCORE_H_ */
