// =========================================================================
//
//  File Name:         pcieVcInterface.h
//  Design Unit Name:
//  Revision:          OSVVM MODELS STANDARD VERSION
//
//  Maintainer:        Simon Southwell email:  simon.southwell@gmail.com
//  Contributor(s):
//    Simon Southwell      simon.southwell@gmail.com
//
//  Description:
//    Header for PCIe VC model C++ interface code between bus independent
//    model port and PCIe link ports
//
//  Revision History:
//    Date      Version    Description
//    09/2025   ????.??    Initial Version
//
//  This file is part of OSVVM.
//
//  Copyright (c) 2025 by [OSVVM Authors](../../AUTHORS.md)
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
// =========================================================================

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <queue>

#include "pcieModelClass.h"
#include "OsvvmPcieAdapter.h"

extern "C" {
#include "ltssm.h"
}

#ifndef _PCIEVCINTERFACE_H_
#define _PCIEVCINTERFACE_H_

// -------------------------------------------------------------------------
// DEFINES
// -------------------------------------------------------------------------

#ifndef LO_NIBBLE_MASK
#define LO_NIBBLE_MASK                               0x0f
#endif

#ifndef HI_NIBBLE_MASK
#define HI_NIBBLE_MASK                               0xf0
#endif

// -------------------------------------------------------------------------
// Class definition
// -------------------------------------------------------------------------

class pcieVcInterface
{
public:

    // -------------------------------
    // Class constant definitions
    // -------------------------------

// **** If the below values change, also update ../src/PcieVcInterfacePkg.vhd ****
//  v    v    v    v    v    v    v    v    v    v    v    v    v    v    v    v
    // These following commented out constants are defined by the pcievhost
    // model's code in pcie_vhost_map.h

    // #define             PVH_STOP                    -3
    // #define             PVH_FINISH                  -2
    // #define             PVH_FATAL                   -1
    // #define
    // #define             LINKADDR0                    0
    // #define             LINKADDR1                    1
    // #define             LINKADDR2                    2
    // #define             LINKADDR3                    3
    // #define             LINKADDR4                    4
    // #define             LINKADDR5                    5
    // #define             LINKADDR6                    6
    // #define             LINKADDR7                    7
    // #define             LINKADDR8                    8
    // #define             LINKADDR9                    9
    // #define             LINKADDR10                  10
    // #define             LINKADDR11                  11
    // #define             LINKADDR12                  12
    // #define             LINKADDR13                  13
    // #define             LINKADDR14                  14
    // #define             LINKADDR15                  15
    // #define
    // #define             NODENUMADDR                200
    // #define             LANESADDR                  201
    // #define             PVH_INVERT                 202
    // #define             EP_ADDR                    203
    // #define             CLK_COUNT                  204
    // #define             LINK_STATE                 205
    // #define             RESET_STATE                206

    // Generics address offsets
    static constexpr int   REQID_ADDR            =    300;
    static constexpr int   PIPE_ADDR             =    301;
    static constexpr int   EN_ECRC_ADDR          =    302;
    static constexpr int   INITPHY_ADDR          =    303;

    // Transaction interface options address offsets
    static constexpr int   GETNEXTTRANS          =    400;
    static constexpr int   GETINTTOMODEL         =    401;
    static constexpr int   GETBOOLTOMODEL        =    402;
    static constexpr int   GETTIMETOMODEL        =    403;
    static constexpr int   GETADDRESS            =    404;
    static constexpr int   GETADDRESSWIDTH       =    405;
    static constexpr int   GETDATATOMODEL        =    406;
    static constexpr int   GETDATAWIDTH          =    407;
    static constexpr int   GETPARAMS             =    408;
    static constexpr int   GETOPTIONS            =    409;
    static constexpr int   ACKTRANS              =    410;
    static constexpr int   SETDATAFROMMODEL      =    411;
    static constexpr int   SETBOOLFROMMODEL      =    412;
    static constexpr int   SETINTFROMMODEL       =    413;
    static constexpr int   POPDATA               =    414;
    static constexpr int   PUSHDATA              =    415;

    static constexpr int   VCOPTIONSTART         =   1000;
    static constexpr int   ENDMODELRUN           =   VCOPTIONSTART;

    static constexpr int   SETTRANSMODE          =   1001;
    static constexpr int   INITDLL               =   1002;
    static constexpr int   INITPHY               =   1003;
    static constexpr int   SETRDLCK              =   1004;
    static constexpr int   SETCMPLRID            =   1005;
    static constexpr int   SETCMPLCID            =   1006;
    static constexpr int   SETCMPLRLEN           =   1007;
    static constexpr int   SETCMPLTAG            =   1008;
    static constexpr int   SETREQTAG             =   1009;
    static constexpr int   SETCFGSPC             =   1010;
    static constexpr int   SETCFGSPCMASK         =   1011;
    static constexpr int   SETCFGSPCOFFSET       =   1012;
    static constexpr int   SETMEMADDRLO          =   1013;
    static constexpr int   SETMEMADDRHI          =   1014;
    static constexpr int   SETMEMDATA            =   1015;


    static constexpr int   GETLASTCMPLSTATUS     =   2000;
    static constexpr int   GETLASTRXREQTAG       =   2001;
    static constexpr int   GETMEMDATA            =   2002;

//  ^    ^    ^    ^    ^    ^    ^    ^    ^    ^    ^    ^    ^    ^    ^    ^
// **** If the above values change, also update ../src/PcieVcInterfacePkg.vhd ****

    static constexpr int   DELTACYCLE            =     -1;
    static constexpr int   CLOCKEDCYCLE          =      0;
    static constexpr int   PIPE_MODE_ENABLED     =      1;
    static constexpr int   PIPE_MODE_DISABLED    =      0;
    static constexpr int   EP_MODE_ENABLED       =      1;
    static constexpr int   EP_MODE_DISABLED      =      0;
    static constexpr int   DIGEST_MODE_ENABLED   =      1;
    static constexpr int   DIGEST_MODE_DISABLED  =      0;
    static constexpr int   STRBUFSIZE            =    256;
    static constexpr int   DATABUFSIZE           =   4096;

    static constexpr int   FREERUNSIM            =      0;
    static constexpr int   STOPSIM               =      1;
    static constexpr int   FINISHSIM             =      2;

    static constexpr int   CMPL_ADDR_MASK        =   0x7c;
    static constexpr int   CMPL_STATUS_VOID      = 0xffff;

    static constexpr int   TLP_TAG_AUTO          =  0x100;

    // -------------------------------
    // Class type definitions
    // -------------------------------

    // Single data buffer vector type
    typedef std::vector<PktData_t> DataVec_t;

    // Receive data structure type
    typedef  struct {
        PktData_t cpl_status;
        PktData_t tag;
        PktData_t loaddr;
        DataVec_t rxbuf;
    } DataBuf_t;

    // Receive data queue type
    typedef std::queue<DataBuf_t> DataBufQueue_t;

    // Enumerated type for different TLPs
    typedef enum pcie_trans_mode_e
    {
        MEM_TRANS,
        IO_TRANS,
        CFG_SPC_TRANS,
        MSG_TRANS,
        CPL_TRANS,
        PART_CPL_TRANS
    } pcie_trans_mode_t;

    // -------------------------------
    // Public methods
    // -------------------------------

public:

    // Constructor
    pcieVcInterface (const unsigned nodeIn) : node (nodeIn)
                {
                    // Create a PCIe API object
                    pcie        = new pcieModelClass(nodeIn);

                    // Default the internal state member variables
                    reset_state      = 0;
                    link_width       = 0;
                    rid              = node;
                    pipe_mode        = PIPE_MODE_DISABLED;
                    ep_mode          = EP_MODE_DISABLED;
                    digest_mode      = DIGEST_MODE_DISABLED;

                    trans_mode       = MEM_TRANS;
                    rd_lck           = false;
                    tag              = 0;
                    cmplrid          = 0;
                    cmplcid          = 0;
                    cmpltag          = 0;
                    cmplrlen         = 0;
                    cfgspc_offset    = 0;
                    mem_addr         = 0;
                    last_cpl_status  = CMPL_STATUS_VOID;

                    txdatabuf        = new PktData_t[DATABUFSIZE];
                };

    // Run main PCIe VC function
    void        run(void);

    // Run automatic EP mode
    void        runAutoEp(void);

    // Input data callback, passing in a packet, error status
    void        InputCallback (pPkt_t pkt, int status);

   // -------------------------------
   // Private member variable
   // -------------------------------

private:

    uint32_t           node;
    pcieModelClass    *pcie;

    unsigned           tag;
    unsigned           reset_state;
    unsigned           link_width;
    unsigned           rid;
    unsigned           pipe_mode;
    unsigned           ep_mode;
    unsigned           digest_mode;
    pcie_trans_mode_t  trans_mode;
    bool               rd_lck;
    unsigned           cmplrid;
    unsigned           cmplcid;
    unsigned           cmpltag;
    unsigned           cmplrlen;
    unsigned           cfgspc_offset;
    uint64_t           mem_addr;
    char               sbuf[STRBUFSIZE];
    pPktData_t         txdatabuf;

    // Queue for RX buffers, for use by input callback
    DataBufQueue_t     rxbufq;

    PktData_t          last_cpl_status;
    PktData_t          last_rx_tag;

    // -------------------------------
    // Private methods
    // -------------------------------

private:
    // *EXAMPLE* Type 0 configuration setup for automatic EP model
    void        ConfigureType0PcieCfg (pcieModelClass* pcie);

    // Input callback for automatic EP model (static so can be used as a callback).
    static void VUserInputAutoEp      (pPkt_t pkt, int status, void* usrptr);

};

#endif