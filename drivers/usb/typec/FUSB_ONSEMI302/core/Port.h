/*******************************************************************************
 * @file     Port.h
 * @author   USB PD Firmware Team
 *
 * Copyright 2018 ON Semiconductor. All rights reserved.
 *
 * This software and/or documentation is licensed by ON Semiconductor under
 * limited terms and conditions. The terms and conditions pertaining to the
 * software and/or documentation are available at
 * http://www.onsemi.com/site/pdf/ONSEMI_T&C.pdf
 * ("ON Semiconductor Standard Terms and Conditions of Sale, Section 8 Software").
 *
 * DO NOT USE THIS SOFTWARE AND/OR DOCUMENTATION UNLESS YOU HAVE CAREFULLY
 * READ AND YOU AGREE TO THE LIMITED TERMS AND CONDITIONS. BY USING THIS
 * SOFTWARE AND/OR DOCUMENTATION, YOU AGREE TO THE LIMITED TERMS AND CONDITIONS.
 ******************************************************************************/
#ifndef _PORT_H_
#define _PORT_H_

#include "TypeC_Types.h"
#include "PD_Types.h"
#include "fusb30X.h"
#include "platform.h"
#include "vdm/vdm_callbacks_defs.h"
#include "vdm/DisplayPort/dp.h"
#include "modules/observer.h"
#include "Log.h"
//#include "modules/dpm.h"
#include "timer.h"

#ifdef CONFIG_PRODUCT_MOBA
#define SUPPORT_ONSEMI_PDCONTROL
#endif

#ifdef SUPPORT_ONSEMI_PDCONTROL
#include <linux/mutex.h>
#endif

/* Size of Rx/Tx FIFO protocol buffers */
#define FSC_PROTOCOL_BUFFER_SIZE 64

/* Number of timer objects in list */
#define FSC_NUM_TIMERS 10

typedef enum
{
    dpmSelectSourceCap,
    dpmIdle,
} DpmState_t;


struct devicePolicy_t
{
    Port_t          *ports[NUM_PORTS];     ///< List of port managed
    FSC_U8           num_ports;
    DpmState_t       dpm_state;
};


typedef struct devicePolicy_t DevicePolicy_t;
typedef DevicePolicy_t* DevicePolicyPtr_t;


typedef struct
{
    USBTypeCPort    PortType;                  /* Snk/Src/DRP */
    FSC_BOOL        SrcPreferred;              /* Source preferred (DRP) */
    FSC_BOOL        SnkPreferred;              /* Sink preferred (DRP) */
    FSC_BOOL        SinkGotoMinCompatible;     /* Sink GotoMin supported. */
    FSC_BOOL        SinkUSBSuspendOperation;   /* USB suspend capable */
    FSC_BOOL        SinkUSBCommCapable;        /* USB communications capable */
    FSC_U32         SinkRequestMaxVoltage;     /* Sink Maximum voltage */
    FSC_U32         SinkRequestMaxPower;       /* Sink Maximum power */
    FSC_U32         SinkRequestOpPower;        /* Sink Operating power */
    FSC_BOOL        audioAccSupport;           /* Audio Acc support */
    FSC_BOOL        poweredAccSupport;         /* Powered Acc support */
    FSC_BOOL        reqDRSwapToDfpAsSink;      /* Request DR swap as sink */
    FSC_BOOL        reqDRSwapToUfpAsSrc;       /* Request DR swap as source */
    FSC_BOOL        reqVconnSwapToOnAsSink;    /* Request Vconn swap */
    FSC_BOOL        reqVconnSwapToOffAsSrc;    /* Request Vconn swap */
    FSC_BOOL        reqPRSwapAsSrc;            /* Request PR swap as source */
    FSC_BOOL        reqPRSwapAsSnk;            /* Request PR swap as sink*/
    USBTypeCCurrent RpVal;                     /* Pull up value to use */
    FSC_U8          PdRevPreferred;           /* PD Rev to use */
} PortConfig_t;



/**
 * All state variables here for now.
 */
struct Port
{
    DevicePolicy_t*         dpm;
    PortConfig_t            PortConfig;
    FSC_U8                  PortID;                     /* Optional Port ID */
    FSC_U8                  I2cAddr;
    DeviceReg_t             Registers;
    FSC_BOOL                TCIdle;                     /* True: Type-C idle */
    FSC_BOOL                PEIdle;                     /* True: PE idle */

    /* All Type C State Machine variables */
    CCTermType              CCTerm;                     /* Active CC */
    CCTermType              CCTermCCDebounce;           /* Debounced CC */
    CCTermType              CCTermPDDebounce;
    CCTermType              CCTermPDDebouncePrevious;
    CCTermType              VCONNTerm;

    SourceOrSink            sourceOrSink;               /* TypeC Src or Snk */
    USBTypeCCurrent         SinkCurrent;                /* PP Current */
    USBTypeCCurrent         SourceCurrent;              /* Our Current */
    CCOrientation           CCPin;                      /* CC == CC1 or CC2 */
    FSC_BOOL                SMEnabled;               /* TypeC SM Enabled */
    ConnectionState         ConnState;                  /* TypeC State */
    FSC_U8                  TypeCSubState;              /* TypeC Substate */
    FSC_U16                 DetachThreshold;            /* TypeC detach level */
    FSC_U8                  loopCounter;                /* Count and prevent
                                                           attach/detach loop */
    FSC_BOOL                C2ACable;                   /* Possible C-to-A type
                                                           cable detected */
    /* All Policy Engine variables */
    PolicyState_t           PolicyState;                /* PE State */
    FSC_U8                  PolicySubIndex;             /* PE Substate */
    SopType                 PolicyMsgTxSop;             /* Tx to SOP? */
    FSC_BOOL                USBPDActive;                /* PE Active */
    FSC_BOOL                USBPDEnabled;               /* PE Enabled */
    FSC_BOOL                PolicyIsSource;             /* Policy Src/Snk? */
    FSC_BOOL                PolicyIsDFP;                /* Policy DFP/UFP? */
    FSC_BOOL                PolicyHasContract;          /* Have contract */
    FSC_BOOL                isContractValid;            /* PD Contract Valid */
    FSC_BOOL                IsHardReset;                /* HR is occurring */
    FSC_BOOL                IsPRSwap;                   /* PR is occurring */
    FSC_BOOL                IsVCONNSource;              /* VConn state */
    FSC_BOOL                USBPDTxFlag;                /* Have msg to Tx */
    FSC_U8                  CollisionCounter;           /* Collisions for PE */
    FSC_U8                  HardResetCounter;
    FSC_U8                  CapsCounter;                /* Startup caps tx'd */

    sopMainHeader_t         src_cap_header;
    sopMainHeader_t         snk_cap_header;
    doDataObject_t          src_caps[7];
    doDataObject_t          snk_caps[7];

    FSC_U8                  PdRevSop;              /* Partner spec rev */
    FSC_U8                  PdRevCable;                 /* Cable spec rev */
    FSC_BOOL                PpsEnabled;                 /* True if PPS mode */

    FSC_BOOL                WaitingOnHR;                /* HR shortcut */
    FSC_BOOL                WaitSent;                   /* Waiting on PR Swap */

    FSC_BOOL                WaitInSReady;               /* Snk/SrcRdy Delay */

    /* All Protocol State Machine Variables */
    ProtocolState_t         ProtocolState;              /* Protocol State */
    sopMainHeader_t         PolicyRxHeader;             /* Header Rx'ing */
    sopMainHeader_t         PolicyTxHeader;             /* Header Tx'ing */
    sopMainHeader_t         PDTransmitHeader;           /* Header to Tx */
    sopMainHeader_t         SrcCapsHeaderReceived;      /* Recent caps */
    sopMainHeader_t         SnkCapsHeaderReceived;      /* Recent caps */
    doDataObject_t          PolicyRxDataObj[7];         /* Rx'ing objects */
    doDataObject_t          PolicyTxDataObj[7];         /* Tx'ing objects */
    doDataObject_t          PDTransmitObjects[7];       /* Objects to Tx */
    doDataObject_t          SrcCapsReceived[7];         /* Recent caps header */
    doDataObject_t          SnkCapsReceived[7];         /* Recent caps header */
    doDataObject_t          USBPDContract;              /* Current PD request */
    doDataObject_t          SinkRequest;                /* Sink request  */
    doDataObject_t          PartnerCaps;                /* PP's Sink Caps */
    doPDO_t                 vendor_info_source[7];      /* Caps def'd by VI */
    doPDO_t                 vendor_info_sink[7];        /* Caps def'd by VI */
    SopType                 ProtocolMsgRxSop;           /* SOP of msg Rx'd */
    SopType                 ProtocolMsgTxSop;           /* SOP of msg Tx'd */
    PDTxStatus_t            PDTxStatus;                 /* Protocol Tx state */
    FSC_BOOL                ProtocolMsgRx;              /* Msg Rx'd Flag */
    FSC_BOOL                ProtocolMsgRxPending;       /* Msg in FIFO */
    FSC_U8                  ProtocolTxBytes;            /* Bytes to Tx */
    FSC_U8                  ProtocolTxBuffer[FSC_PROTOCOL_BUFFER_SIZE];
    FSC_U8                  ProtocolRxBuffer[FSC_PROTOCOL_BUFFER_SIZE];
    FSC_U8                  MessageIDCounter[SOP_TYPE_NUM]; /* Local ID count */
    FSC_U8                  MessageID[SOP_TYPE_NUM];    /* Local last msg ID */

    FSC_BOOL                DoTxFlush;                  /* Collision -> Flush */

    /* Timer objects */
    struct TimerObj         PDDebounceTimer;            /* First debounce */
    struct TimerObj         CCDebounceTimer;            /* Second debounce */
    struct TimerObj         StateTimer;                 /* TypeC state timer */
    struct TimerObj         LoopCountTimer;             /* Loop delayed clear */
    struct TimerObj         PolicyStateTimer;           /* PE state timer */
    struct TimerObj         ProtocolTimer;              /* Prtcl state timer */
    struct TimerObj         SwapSourceStartTimer;       /* PR swap delay */
    struct TimerObj         PpsTimer;                   /* PPS timeout timer */
    struct TimerObj         VBusPollTimer;              /* VBus monitor timer */
    struct TimerObj         VdmTimer;                   /* VDM timer */

    struct TimerObj         *Timers[FSC_NUM_TIMERS];

    ExtMsgState_t           ExtTxOrRx;                  /* Tx' or Rx'ing  */
    ExtHeader_t             ExtTxHeader;
    ExtHeader_t             ExtRxHeader;
    FSC_BOOL                ExtWaitTxRx;                /* Waiting to Tx/Rx */
    FSC_U16                 ExtChunkOffset;             /* Next chunk offset */
    FSC_U8                  ExtMsgBuffer[260];
    FSC_U8                  ExtChunkNum;                /* Next chunk number */
    //ExtHeader_t             ExtHeader;                  /* For sending NS */
    //FSC_BOOL                WaitForNotSupported;        /* Wait for timer */


    FSC_BOOL                SourceCapsUpdated;          /* GUI new caps flag */


    VdmDiscoveryState_t     AutoVdmState;
    VdmManager              vdmm;
    PolicyState_t           vdm_next_ps;
    PolicyState_t           originalPolicyState;
    SvidInfo                core_svid_info;
    SopType                 VdmMsgTxSop;
    doDataObject_t          vdm_msg_obj[7];
    FSC_U32                 my_mode;
    FSC_U32                 vdm_msg_length;
    FSC_BOOL                sendingVdmData;
    FSC_BOOL                VdmTimerStarted;
    FSC_BOOL                vdm_timeout;
    FSC_BOOL                vdm_expectingresponse;
    FSC_BOOL                svid_enable;
    FSC_BOOL                mode_enable;
    FSC_BOOL                mode_entered;
    FSC_BOOL                AutoModeEntryEnabled;
    FSC_S32                 AutoModeEntryObjPos;
    FSC_S16                 svid_discvry_idx;
    FSC_BOOL                svid_discvry_done;
    FSC_U16                 my_svid;
    FSC_U16                 discoverIdCounter;
    FSC_BOOL                cblPresent;
    CableResetState_t       cblRstState;

    DisplayPortData_t       DisplayPortData;


    StateLog                TypeCStateLog;          /* Log for TypeC states */
    StateLog                PDStateLog;             /* Log for PE states */
    FSC_U8                  USBPDBuf[PDBUFSIZE];    /* Circular PD msg buffer */
    FSC_U8                  USBPDBufStart;          /* PD msg buffer head */
    FSC_U8                  USBPDBufEnd;            /* PD msg buffer tail */
    FSC_BOOL                USBPDBufOverflow;       /* PD Log overflow flag */
#ifdef SUPPORT_ONSEMI_PDCONTROL
	struct mutex thread_lock;                       // Prevent thread re-etrance
#endif
};

/**
 * @brief Initializes the port structure and state machine behaviors
 *
 * Initializes the port structure with default values.
 * Also sets the i2c address of the device and enables the TypeC state machines.
 *
 * @param port Pointer to the port structure
 * @param i2c_address 8-bit value with bit zero (R/W bit) set to zero.
 * @return None
 */
void PortInit(struct Port *port, FSC_U8 i2cAddr);

/**
 * @brief Set the next Type-C state.
 *
 * Also clears substate and logs the new state value.
 *
 * @param port Pointer to the port structure
 * @param state Next Type-C state
 * @return None
 */
void SetTypeCState(struct Port *port, ConnectionState state);

/**
 * @brief Set the next Policy Engine state.
 *
 * Also clears substate and logs the new state value.
 *
 * @param port Pointer to the port structure
 * @param state Next Type-C state
 * @return None
 */
void SetPEState(struct Port *port, PolicyState_t state);

/**
 * @brief Revert the ports current setting to the configured value.
 *
 * @param port Pointer to the port structure
 * @return None
 */
void SetConfiguredCurrent(struct Port *port);

/**
 * @brief Initializes the DPM object pointer
 * @param[in] DPM pointer type object
 */
void DPM_Init(DevicePolicyPtr_t *dpm);

/**
 * @brief Adds port to the list of ports managed by dpm
 * @param[in] dpm object to which the port is added
 * @param[in] port object which is added to DPM list
 */
void DPM_AddPort(DevicePolicyPtr_t dpm, Port_t *port);

/**
 * @brief Get source cap header for the port object.
 * @param[in] dpm pointer to device policy object
 * @param[in] port requesting source cap header
 */
sopMainHeader_t* DPM_GetSourceCapHeader(DevicePolicyPtr_t dpm, Port_t *port);

/**
 * @brief Get sink cap header for the port object
 * @param[in] dpm pointer to device policy object
 * @param[in] port object requesting sink cap header
 */
sopMainHeader_t* DPM_GetSinkCapHeader(DevicePolicyPtr_t dpm, Port_t *port);

/**
 * @brief Get the source cap for the port object
 * @param[in] dpm pointer to device policy object
 * @param[in] port object requesting source caps
 */
doDataObject_t* DPM_GetSourceCap(DevicePolicyPtr_t dpm, Port_t *port);

/**
 * @brief Get sink cap for the port object
 * @param[in] dpm pointer to device policy object
 * @param[in] port object requesting sink cap
 */
doDataObject_t* DPM_GetSinkCap(DevicePolicyPtr_t dpm, Port_t *port);

/**
 * @brief Called by the usb PD/TypeC core to ask device policy to transition
 * to the capability advertised specified by port and index.
 * @param[in] dpm pointer to the device policy object
 * @param[in] port advertising the source capability
 * @param[in] index of the source capability object
 */
FSC_BOOL DPM_TransitionSource(DevicePolicyPtr_t dpm,
                              Port_t *port, FSC_U8 index);

/**
 * @brief Called by usb PD/TypeC core to ask device policy if the source
 * is ready after the transition. It returns true if the source transition has
 * completed and successful.
 * @param[in] dpm pointer to the device policy object
 * @param[in] port advertising the source capability
 * @param[in] index of the source capability object
 */
FSC_BOOL DPM_IsSourceCapEnabled(DevicePolicyPtr_t dpm,
                                Port_t *port, FSC_U8 index);

/**
 * @brief Returns appropriate spec revision value per SOP*.
 * @param[in] current port object
 * @param[in] SOP in question
 */
SpecRev DPM_SpecRev(Port_t *port, SopType sop);

/**
 * @brief Sets appropriate spec revision value per SOP* and adjusts for
 * compatibility.
 * @param[in] current port object
 * @param[in] SOP in question
 */
void DPM_SetSpecRev(Port_t *port, SopType sop, SpecRev rev);

/**
 * @brief Returns appropriate SVDM revision value per SOP*.
 * @param[in] current port object
 * @param[in] SOP in question
 */
SvdmVersion DPM_SVdmVer(Port_t *port, SopType sop);


/**
 * @brief Returns appropriate number of retries (based on spec rev) per SOP*.
 * @param[in] current port object
 * @param[in] SOP in question
 */
FSC_U8 DPM_Retries(Port_t *port, SopType sop);

#endif /* _PORT_H_ */
