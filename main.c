/*****************************************************************************/
/*  Copyright (c) 2011 NXP B.V.  All rights  are reserved.                   */
/*  Reproduction in whole or in part is prohibited without the prior         */
/*  written consent of the copyright owner.                                  */
/*                                                                           */
/*  This software and any compilation or derivative thereof is, and          */
/*  shall remain the proprietary information of NXP and is                   */
/*  highly confidential in nature. Any and all use hereof is restricted      */
/*  and is subject to the terms and conditions set forth in the              */
/*  software license agreement concluded with NXP B.V.                       */
/*                                                                           */
/*****************************************************************************/

#include "Dirana3_ABB_HIFI0_DefExtHW.h"
  
#include "Dirana3_ABB_HIFI0_tmFW.h"
#include "Dirana3_ABB_HIFI0_tmISR.h"
#include "Dirana3_ABB_HIFI0_tmStartup.h"
#include "Dirana3_ABB_HIFI0_tmCmdQueue.h"
#include "Dirana3_ABB_HIFI0_tmPinCtrl.h"
#include "tmSinGen.h"
#include <xtensa/tie/xt_hifi2.h>

//..add srs..//

#include "srs_csdecoder_api.h"

#define SRS_XTENSA  1 //open xtensa

#if SRS_XTENSA
#include <sys/times.h>
#define CPU_LOAD_WINDOW 4096
#endif


/*define place for saving return result*/

 int myResult[100];

/*define place for return value for Conti 0x21232250U, to write 1 if i2c properly recevived, if not put error code as 0x0002~0x0006, default value is  0x0000*/
 
 int Conti_Regitster_ACK;//set default value 0
 int conti_Bootup_ACK;
 
 
/* Block size can be any positive integer */
#define kBlockSize		256

static void SPM1(int32_t* Data1, int32_t* Data2);

static void MSG_Handler(uint8_t *msg);

#define MONOCHCNT       (1)
#define STEREOCHCNT     (2)
#define PRIMCHCNT       (6)
#define SEC1CHCNT       (2)
#define SEC2CHCNT       (2)
#define NAVCHCNT        (1)
#define PHOCHCNT        (1)
#define HDRCHCNT        (2)
#define EXTCHCNT        (2)

ptmFW_t pStartup_FW[TMSTARTUP_FW_MAX];


uint32_t runchk=0;

#define SPM1_BLOCK_SIZE     256//(8)

#define NUMCHANNELS0   (STEREOCHCNT)
#define NUMCHANNELS1   (MONOCHCNT)
#define NUMCHANNELS2   (MONOCHCNT)
#define NUMCHANNELS3   (6)	//for dts 6CH
#define NUMCHANNELS4   (STEREOCHCNT+MONOCHCNT)
#define NUMCHANNELS5   (MONOCHCNT)
#define NUMCHANNELS6   (MONOCHCNT)

#define FIFOSIZE0   (4)
#define FIFOSIZE1   (2)
#define FIFOSIZE2   (2)
#define FIFOSIZE3   (4)
#define FIFOSIZE4   (2)
#define FIFOSIZE5   (2)
#define FIFOSIZE6   (2)

#define BLOCKSIZE0  SPM1_BLOCK_SIZE
#define BLOCKSIZE3  SPM1_BLOCK_SIZE


int errorNumber;
static int32_t data0[NUMCHANNELS0*BLOCKSIZE0*FIFOSIZE0] __attribute__ ((section(".dram1.bss")));
static int32_t data3[NUMCHANNELS3*BLOCKSIZE3*FIFOSIZE3] __attribute__ ((section(".dram1.bss")));

static uint8_t msg[TMCMDQUEUE_MAXMSGSIZE];

//add for bypass function
bool isBypass;

typedef enum ctrlOpCode
{
    cmd_SRS_CSD_SetInputGain=0 ,


	cmd_SRS_CSD_SetProcMode=1,
	cmd_SRS_CSD_SetOutMode=2,
	cmd_SRS_CSD_SetPhantomEnable=3,
	cmd_SRS_CSD_SetCenterFullBandEnable=4,
	
	
	cmd_SRS_CSD_SetTBFrontEnable=5,
	cmd_SRS_CSD_SetTBSubEnable=6,
	cmd_SRS_CSD_SetTBRearEnable=7,
	cmd_SRS_CSD_SetFocusCenterEnable=8,
	cmd_SRS_CSD_SetFocusFrontEnable=9,
	cmd_SRS_CSD_SetFocusRearEnable=10,

	cmd_SRS_CSD_SetTBFrontLevel=11,
	
	cmd_SRS_CSD_SetTBSubLevel=12,
	cmd_SRS_CSD_SetTBRearLevel=13,
	cmd_SRS_CSD_SetTBFrontSpksz=14,
	cmd_SRS_CSD_SetTBSubSpksz=15,
	
	cmd_SRS_CSD_SetTBRearSpksz=16,
	cmd_SRS_CSD_SetFocusCenterLvl=17,
	cmd_SRS_CSD_SetFocusFrontLvl=18,
	 cmd_SRS_CSD_SetFocusRearLvl=19,
	  cmd_SRS_CSD_SetFocusF2RLvl=20,
	  cmd_SRS_CSD_SetFocusC2RLvl=21,

	cmd_SRS_CSD_SetBaypass=22,	//add for bypass function
	cmd_SRS_CSD_GetInputGain=23,
    cmd_SRS_CSD_GetProcMode=24,
    cmd_SRS_CSD_GetOutMode=25,
    cmd_SRS_CSD_GetPhantomEnable=26,
    cmd_SRS_CSD_GetCenterFullBandEnable=27,
    cmd_SRS_CSD_GetTBFrontEnable=28,
    cmd_SRS_CSD_GetTBSubEnable=29,
    cmd_SRS_CSD_GetTBRearEnable=30,
    cmd_SRS_CSD_GetFocusCenterEnable=31,
    cmd_SRS_CSD_GetFocusFrontEnable=32,
    cmd_SRS_CSD_GetFocusRearEnable=33,
    cmd_SRS_CSD_GetTBFrontSpksz=34,
    cmd_SRS_CSD_GetTBSubSpksz=35,
    cmd_SRS_CSD_GetTBRearSpksz=36,
    cmd_SRS_CSD_GetFocusCenterLvl=37,
    cmd_SRS_CSD_GetFocusFrontLvl=38,
    cmd_SRS_CSD_GetFocusRearLvl=39,
    cmd_SRS_CSD_GetFocusF2RLvl=40,
    cmd_SRS_CSD_GetFocusC2RLvl=41,
    cmd_SRS_Get_Bypass=42,
    cmd_SRS_CSD_GetTBFrontLevel=43,
    cmd_SRS_CSD_GetTBSubLevel=44,
    cmd_SRS_CSD_GetTBRearLevel=45,
    cmd_SRS_LAST
} ctrlOpCode_t;

#define TWOBYTESHIFT    16
#define ONEBYTESHIFT    8

//claim for dis
	SRSCSDObj	csdObj;
	void			*workspace;
	void			*Objbuf;
	SRSStereoCh	input;
	SRS6Point1Ch	output;

	

int main(int argc, char *argv[]) {

 Conti_Regitster_ACK = 0x8888; //for testing
 conti_Bootup_ACK = 0x7777;
    
    uint16_t i;


    /* Framework definitions */
    uint16_t fwcount = BLOCK_SIZE;
    tmFW_t  framework;
    tmFWStream_t streams[TMFW_STREAMS_MAX];

//stream for dts 6CH
    /* Stream 0 definitions */
    bool isRead0=true;
    uint16_t blksize0 = BLOCKSIZE0;
    uint16_t fifosize0 = FIFOSIZE0;
    uint16_t chanCount0=STEREOCHCNT;
    uint8_t channels0[STEREOCHCNT];
    ptmFWStream_t pstreamID0;

//stream for dts 6CH
    /* Stream 3 definitions */
    bool isRead3=false;
    uint16_t blksize3 = BLOCKSIZE3;
    uint16_t fifosize3 = FIFOSIZE3;
    uint16_t chanCount3=NUMCHANNELS3;
    uint8_t channels3[NUMCHANNELS3];
    ptmFWStream_t pstreamID3;

//stream for dts 6CH
    int32_t *paudiosig1_input;
    int32_t audiosig1_output[(6)*blksize3];

    tmStartupIntDesc_t intdesc[TMSTARTUP_FW_MAX];




/************adding srs********************/


	/*******************************************************************************************/
	/* DTS */
	int				n;


/* Allocate memory */
	input.Left=(SRSInt32*)malloc(sizeof(SRSInt32)*kBlockSize);
	input.Right=(SRSInt32*)malloc(sizeof(SRSInt32)*kBlockSize);

	output.Left=(SRSInt32*)malloc(sizeof(SRSInt32)*kBlockSize);
	output.Right=(SRSInt32*)malloc(sizeof(SRSInt32)*kBlockSize);
	output.Center=(SRSInt32*)malloc(sizeof(SRSInt32)*kBlockSize);
	output.Sub=(SRSInt32*)malloc(sizeof(SRSInt32)*kBlockSize);
	output.SLeft=(SRSInt32*)malloc(sizeof(SRSInt32)*kBlockSize);
	output.SRight=(SRSInt32*)malloc(sizeof(SRSInt32)*kBlockSize);
	output.SCenter=(SRSInt32*)malloc(sizeof(SRSInt32)*kBlockSize);

	Objbuf=malloc(SRS_CSD_GetObjSize());
	workspace= malloc(SRS_CSD_WORKSPACE_SIZE(kBlockSize));


	for (i=0;i<100;i++){
	myResult[i] = 0x2222;
}


	SRS_CSD_CreateObj(&csdObj, Objbuf, 0, 0);	

//	printf(">>>> This is a Fixed point implementation.\n\n");

	SRS_CSD_InitObj48k(csdObj);
	SRS_CSD_SetControlDefaults(csdObj);
	if(!csdObj){
		conti_Bootup_ACK = 0x0002;
	}
	/*SRS_CSD_SetInputGain(csdObj, 0x7fff);
	SRS_CSD_SetProcMode(csdObj, SRS_CSD_MUSIC);
	SRS_CSD_SetOutMode(csdObj, SRS_CSD_MULTICHS);
	SRS_CSD_SetPhantomEnable(csdObj, 0);
	SRS_CSD_SetCenterFullBandEnable(csdObj, 0);
	
	
	SRS_CSD_SetTBFrontEnable(csdObj, 1);
	SRS_CSD_SetTBSubEnable(csdObj,1
);
	SRS_CSD_SetTBRearEnable(csdObj, 1);
	SRS_CSD_SetFocusCenterEnable(csdObj,1);
	SRS_CSD_SetFocusFrontEnable(csdObj, 1);
	SRS_CSD_SetFocusRearEnable(csdObj,1);

	SRS_CSD_SetTBFrontLevel(csdObj,0x7fff);
	
	SRS_CSD_SetTBSubLevel(csdObj,0x7fff);
	SRS_CSD_SetTBRearLevel(csdObj,0x7fff);
	SRS_CSD_SetTBFrontSpksz(csdObj, SRS_TBHD_SPEAKER_LF_RESPONSE_60HZ);
	SRS_CSD_SetTBSubSpksz(csdObj,SRS_TBHD_SPEAKER_LF_RESPONSE_60HZ);
	
	SRS_CSD_SetTBRearSpksz(csdObj,SRS_TBHD_SPEAKER_LF_RESPONSE_60HZ);
	SRS_CSD_SetFocusCenterLvl(csdObj,0x7fff);
	SRS_CSD_SetFocusFrontLvl(csdObj, 0x7fff);
	 SRS_CSD_SetFocusRearLvl(csdObj,0x0fff);
	  SRS_CSD_SetFocusF2RLvl(csdObj,0x0000);
	  SRS_CSD_SetFocusC2RLvl(csdObj,0x7fff);*/
	
/*............end srs............*/

    /********* Initialize the stream pointers in the framework and the framework ***************/

    runchk=0x11111111;

    /* Initialize the stream pointers in the framework */
    for (i=0;i<TMFW_STREAMS_MAX;i++)
    {
        framework.pstreams[i]=&(streams[i]);
    }

    /* Initialize the framework */
    tmFW_Init(&framework, fwcount);

    /************************** Finished framework initialization ******************************/

    /************************** Configure sine generator ***************************************/


    /************************** Finished configuring sine generator ****************************/

    /*************** Configure the streams in the framework and open them **********************/
    channels0[0]=TMFW_CHANNEL_FL;
    channels0[1]=TMFW_CHANNEL_FR;

//    channels1[0]=TMFW_CHANNEL_FL;

//    channels2[0]=TMFW_CHANNEL_FR;

    channels3[0]=TMFW_CHANNEL_FL;
    channels3[1]=TMFW_CHANNEL_FR;
    channels3[2]=TMFW_CHANNEL_C;
    channels3[3]=TMFW_CHANNEL_SW;
    channels3[4]=TMFW_CHANNEL_RL;
    channels3[5]=TMFW_CHANNEL_RR;

//    channels4[0]=TMFW_CHANNEL_RL;
//    channels4[1]=TMFW_CHANNEL_RR;
//    channels4[2]=TMFW_CHANNEL_C;

//    channels5[0]=TMFW_CHANNEL_S1L;

//    channels6[0]=TMFW_CHANNEL_S1R;

    /* Open all streams */
    pstreamID0=tmFW_Open(&framework, isRead0, blksize0, chanCount0, channels0, data0, fifosize0);
//    pstreamID1=tmFW_Open(&framework, isRead1, blksize1, chanCount1, channels1, data1, fifosize1);
 //   pstreamID2=tmFW_Open(&framework, isRead2, blksize2, chanCount2, channels2, data2, fifosize2);
    pstreamID3=tmFW_Open(&framework, isRead3, blksize3, chanCount3, channels3, data3, fifosize3);
//    pstreamID4=tmFW_Open(&framework, isRead4, blksize4, chanCount4, channels4, data4, fifosize4);
//    pstreamID5=tmFW_Open(&framework, isRead5, blksize5, chanCount5, channels5, data5, fifosize5);
//    pstreamID6=tmFW_Open(&framework, isRead6, blksize6, chanCount6, channels6, data6, fifosize6);

    /************************** Finished opening the streams ***********************************/

    /***************************** Start the basic hw initialization ***************************/

    /* Connect the ISR handlers to be used */
    intdesc[0].intidx = INT_HIFI_BAUBLOCK;
    intdesc[0].inthandler=&tmISR_BAU_Handler;
    intdesc[0].pframework=&framework;

    tmStartup_ConnectISR(intdesc);

    tmStartup_InitHw();

	isBypass = 0;

    /*************************** Finished the basic hw initialization **************************/

    /**************************** Enter main processing loop ***********************************/
    while (1)
    {
        if (1==tmFW_Read(pstreamID0, &paudiosig1_input))
        {
            /* input data block available */
            SPM1(paudiosig1_input, audiosig1_output);
            (void)tmFW_Write(pstreamID3, audiosig1_output);
        }
        

        if (tmCmdQueue_ReadQ(msg) != 0)
        {
            /* Parse message */
            MSG_Handler(msg);
        }
    }

    tmFW_Close(pstreamID0);
    tmFW_Close(pstreamID3);

    return 0;
}

/********** Local functions ********/
/* Advanced Audio processing */

static void SPM1(int32_t* Data1, int32_t* Data2)
{
    uint16_t i;

	if (isBypass)
	{
		int32_t left,right;
		
	//copy 2ch to 6ch
		for (i=0; i<SPM1_BLOCK_SIZE; i++)
		{
			left = Data1[2*i];
			right = Data1[(2*i)+1];

			//front
			Data2[6*i] =	left;
			Data2[6*i+1] = right;

			//Cen/Sub
			Data2[6*i+2] = left;
			Data2[6*i+3] = right;

			//rear
			Data2[6*i+4] =left;
			Data2[6*i+5] = right;
		}
	}
	else
	{
		//dts get data from D3
		for (i=0; i<SPM1_BLOCK_SIZE; i++)
		{
			input.Left[i] = Data1[2*i]<<8;
			input.Right[i] = Data1[(2*i)+1]<<8;
		}

		//dts process
		errorNumber= SRS_CSD_Process(csdObj, &input, &output, SPM1_BLOCK_SIZE, workspace);

		//dts output
		for (i=0; i<SPM1_BLOCK_SIZE; i++)
		{
			Data2[6*i] =(SRSInt32)((output.Left[i])>>8);
			Data2[6*i+1] = (SRSInt32)((output.Right[i])>>8);
			Data2[6*i+2] = (SRSInt32)((output.Center[i])>>8);
			Data2[6*i+3] = (SRSInt32)((output.Sub[i])>>8);
			Data2[6*i+4] =(SRSInt32)((output.SLeft[i])>>8);
			Data2[6*i+5] = (SRSInt32)((output.SRight[i])>>8);
		}
	}
}


static void MSG_Handler(uint8_t *msg)
{
    uint16_t opcode = msg[0];

    /* read opcode */
    switch (opcode)
    {
    case cmd_SRS_CSD_SetInputGain:
		myResult[0] = ((msg[2]|msg[1]<<8) < 4125 )? (-1000) : SRS_CSD_SetInputGain(csdObj, (msg[2]|msg[1]<<8));
            if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
    
		break;
		
  	case  cmd_SRS_CSD_SetProcMode:
		myResult[0] =SRS_CSD_SetProcMode(csdObj,(msg[2]|msg[1]<<8));
          if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		break;

	case cmd_SRS_CSD_SetOutMode:
		myResult[0] =SRS_CSD_SetOutMode(csdObj,(msg[2]|msg[1]<<8));
            if (!myResult[0]) {
                Conti_Regitster_ACK=0x0001;
                
            }
		break;

	case cmd_SRS_CSD_SetPhantomEnable:
		SRS_CSD_SetPhantomEnable(csdObj, (msg[2]|msg[1]<<8));
		Conti_Regitster_ACK=0x0001;
		break;

	case cmd_SRS_CSD_SetCenterFullBandEnable:
		SRS_CSD_SetCenterFullBandEnable(csdObj, (msg[2]|msg[1]<<8));
		Conti_Regitster_ACK=0x0001;
		break;
	
	case cmd_SRS_CSD_SetTBFrontEnable:
		SRS_CSD_SetTBFrontEnable(csdObj, (msg[2]|msg[1]<<8));
		Conti_Regitster_ACK=0x0001;
		break;
		
	case cmd_SRS_CSD_SetTBSubEnable:
		SRS_CSD_SetTBSubEnable(csdObj,(msg[2]|msg[1]<<8));
		Conti_Regitster_ACK=0x0001;
		break;
		
	case cmd_SRS_CSD_SetTBRearEnable:
		SRS_CSD_SetTBRearEnable(csdObj, (msg[2]|msg[1]<<8));
		Conti_Regitster_ACK=0x0001;
		break;
		
	case cmd_SRS_CSD_SetFocusCenterEnable:
		SRS_CSD_SetFocusCenterEnable(csdObj,(msg[2]|msg[1]<<8));
		Conti_Regitster_ACK=0x0001;
		break;
		
	case cmd_SRS_CSD_SetFocusFrontEnable:
		SRS_CSD_SetFocusFrontEnable(csdObj, (msg[2]|msg[1]<<8));
		Conti_Regitster_ACK=0x0001;
		break;
		
	case cmd_SRS_CSD_SetFocusRearEnable:
		SRS_CSD_SetFocusRearEnable(csdObj,(msg[2]|msg[1]<<8));
		Conti_Regitster_ACK=0x0001;
		break;

	case cmd_SRS_CSD_SetTBFrontLevel:
		myResult[0] =SRS_CSD_SetTBFrontLevel(csdObj,(msg[2]|msg[1]<<8));
		if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		break;
	
	case cmd_SRS_CSD_SetTBSubLevel:
		myResult[0] =SRS_CSD_SetTBSubLevel(csdObj,(msg[2]|msg[1]<<8));
		if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		break;
		
	case cmd_SRS_CSD_SetTBRearLevel:
		myResult[0] =SRS_CSD_SetTBRearLevel(csdObj,(msg[2]|msg[1]<<8));
		if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		break;
		
	case cmd_SRS_CSD_SetTBFrontSpksz:
		myResult[0] =SRS_CSD_SetTBFrontSpksz(csdObj, (msg[2]|msg[1]<<8));
		if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		break;
		
	case cmd_SRS_CSD_SetTBSubSpksz:
		myResult[0] =SRS_CSD_SetTBSubSpksz(csdObj,(msg[2]|msg[1]<<8));
		if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		break;
	
	case cmd_SRS_CSD_SetTBRearSpksz:
		myResult[0] =SRS_CSD_SetTBRearSpksz(csdObj,(msg[2]|msg[1]<<8));
		if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		break;
		
	case cmd_SRS_CSD_SetFocusCenterLvl:
		myResult[0] =SRS_CSD_SetFocusCenterLvl(csdObj,(msg[2]|msg[1]<<8));
		if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		break;
		
	case cmd_SRS_CSD_SetFocusFrontLvl:
		myResult[0] =SRS_CSD_SetFocusFrontLvl(csdObj, (msg[2]|msg[1]<<8));
		if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		break;
		
	 case cmd_SRS_CSD_SetFocusRearLvl:
	 	myResult[0] = SRS_CSD_SetFocusRearLvl(csdObj,(msg[2]|msg[1]<<8));
	 	if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		break;
		
	  case cmd_SRS_CSD_SetFocusF2RLvl:
	  myResult[0] =	SRS_CSD_SetFocusF2RLvl(csdObj,(msg[2]|msg[1]<<8));
	  if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		break;
		
	  case cmd_SRS_CSD_SetFocusC2RLvl:
		myResult[0] =SRS_CSD_SetFocusC2RLvl(csdObj,(msg[2]|msg[1]<<8));
		if (!myResult[0]){
            Conti_Regitster_ACK=0x0001;
            }
		  break;
			  

	  case cmd_SRS_CSD_SetBaypass:
		isBypass = msg[1]? true : false;
		Conti_Regitster_ACK=0x0001;
		if (csdObj){conti_Bootup_ACK=0x0001;}
		else conti_Bootup_ACK =0x0002; //missing dts object or hifi is not working

		break;
            
               case cmd_SRS_CSD_GetInputGain:
            myResult[0]=SRS_CSD_GetInputGain(csdObj);
            break;
        
        case cmd_SRS_CSD_GetProcMode:
            myResult[0]=SRS_CSD_GetProcMode(csdObj);
            break;
        case    cmd_SRS_CSD_GetOutMode:
            myResult[0]=SRS_CSD_GetOutMode(csdObj);
            break;
       case cmd_SRS_CSD_GetPhantomEnable:
            myResult[0]=(SRS_CSD_GetPhantomEnable(csdObj))? true : false;
            break;
        case cmd_SRS_CSD_GetCenterFullBandEnable:
            myResult[0]=(SRS_CSD_GetCenterFullBandEnable(csdObj))? true : false;
            break;

            
        case cmd_SRS_CSD_GetTBFrontEnable:
            myResult[0]=SRS_CSD_GetTBFrontEnable(csdObj);
            break;
        case cmd_SRS_CSD_GetTBSubEnable:
            myResult[0]=SRS_CSD_GetTBSubEnable(csdObj);
            break;

        case cmd_SRS_CSD_GetTBRearEnable:
            
            myResult[0]=SRS_CSD_GetTBRearEnable(csdObj);
            break;

        case cmd_SRS_CSD_GetFocusCenterEnable:
            myResult[0]=(SRS_CSD_GetFocusCenterEnable(csdObj))? true : false;
            break;

        case cmd_SRS_CSD_GetFocusFrontEnable:
            myResult[0]=SRS_CSD_GetFocusFrontEnable(csdObj);
            break;

        case cmd_SRS_CSD_GetFocusRearEnable:
            myResult[0]=SRS_CSD_GetFocusRearEnable(csdObj);
            break;

        case cmd_SRS_CSD_GetTBFrontSpksz:
            myResult[0]=SRS_CSD_GetTBFrontSpksz(csdObj);
            break;

        case cmd_SRS_CSD_GetTBSubSpksz:
            myResult[0]=SRS_CSD_GetTBSubSpksz(csdObj);
            break;

        case cmd_SRS_CSD_GetTBRearSpksz:
            myResult[0]=SRS_CSD_GetTBRearSpksz(csdObj);
            break;

        case cmd_SRS_CSD_GetFocusCenterLvl:
            myResult[0]=SRS_CSD_GetFocusCenterLvl(csdObj);
            break;

        case cmd_SRS_CSD_GetFocusFrontLvl:
           myResult[0]=SRS_CSD_GetFocusFrontLvl(csdObj);
            break;

        case cmd_SRS_CSD_GetFocusRearLvl:
            myResult[0]=SRS_CSD_GetFocusRearLvl(csdObj);
            break;

        case cmd_SRS_CSD_GetFocusF2RLvl:
           myResult[0]=SRS_CSD_GetFocusF2RLvl(csdObj);
            break;

        case cmd_SRS_CSD_GetFocusC2RLvl:
            myResult[0]=SRS_CSD_GetFocusC2RLvl(csdObj);
            break;

        case cmd_SRS_Get_Bypass:
            myResult[0] = isBypass;
            break;
           
        case cmd_SRS_CSD_GetTBFrontLevel:
            myResult[0] = SRS_CSD_GetTBFrontLevel(csdObj);
            break;
           
        case cmd_SRS_CSD_GetTBSubLevel:
            myResult[0] = SRS_CSD_GetTBSubLevel(csdObj);        
            break;
            
        case cmd_SRS_CSD_GetTBRearLevel:
            myResult[0] = SRS_CSD_GetTBRearLevel(csdObj);        
            break;
            
        default:
            break;
    }
}

