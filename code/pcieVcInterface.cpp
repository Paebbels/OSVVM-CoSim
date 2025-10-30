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
//    PCIe VC model C++ interface code between bus independent model port
//    and PCIe link ports
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

#include "OsvvmCosim.h"
#include "pcieVcInterface.h"

//-------------------------------------------------------------
// CalcBe()
//
// Returns the 8 bit LBE/FBE TLP header field, based on
// address and byte length.
//
//-------------------------------------------------------------

static inline int CalcBe (const int inaddr, const int byte_len)
{
    int val, endpos;
    int addr = inaddr & 0x3;

    endpos = addr + byte_len;

    // First BE
    val = (0xf << (addr & 0x3)) & 0xf;

    // Only one double word---combine start and end BEs
    if (endpos <= 4)
    {
        val &= 0xf >> (4-endpos);
    // More than one DW
    }
    else
    {
        endpos %= 4;
        endpos += endpos ? 0 : 4;
        val |= (0xf0 >> (4-endpos)) & 0xf0;
    }
    return val;
}

//--------------------------------------------------------------
// CalcWordCount()
//
// Calculate number of words required
//
//-------------------------------------------------------------

static inline int CalcWordCount (const int byte_len, const int be)
{
    int num_words, adj_byte_len;


    int fbe = be & 0xf;
    int lbe = (be >> 4) & 0xf;


    if (lbe == 0)
    {
        num_words = 1;
    }
    else
    {
        adj_byte_len = byte_len + ((fbe == 0xe) ? 1 : (fbe == 0xc) ? 2 : (fbe == 0x8) ? 3 : 0);
        num_words = (adj_byte_len/4) + ((adj_byte_len%4) ? 1 : 0);

        DebugVPrint("===> CalcWordCount: fbe = %x lbe = %x byte_len = %d adj_byte_len = %d num_words = %d\n", fbe, lbe, byte_len, adj_byte_len, num_words);
    }

    return num_words;
}

//-------------------------------------------------------------
// VUserInput()
//
// Singleton re-entrant wrapper for pcieVcInterface object's
// input callback function, with object instance pointer passed
// in with obj_instance.
//
//-------------------------------------------------------------

static void VUserInput(pPkt_t pkt, int status, void* obj_instance)
{
    ((pcieVcInterface*)obj_instance)->InputCallback(pkt, status);
}

//-------------------------------------------------------------
// pcieVcInterface::InputCallback()
//
// pcieVcInterface input packet callback method
//
//-------------------------------------------------------------

void pcieVcInterface::InputCallback(pPkt_t pkt, int status)
{
    int idx;

    PktData_t tlp_type = GET_TLP_TYPE(pkt->data);

    // Process DLLPs
    if (pkt->seq == DLLP_SEQ_ID)
    {
        DebugVPrint("---> VUserInput_0 received DLLP\n");
        free(pkt->data);
        free(pkt);
    }
    // Process completions
    else if (tlp_type == TL_CPL || tlp_type == TL_CPLD || tlp_type == TL_CPLLK || tlp_type == TL_CPLDLK)
    {
        DebugVPrint("---> InputCallback received TLP completion,  sequence %d of %d bytes\n", pkt->seq, pkt->ByteCount);

        // Create a new entry in the queue for the completion
        rxbufq.push(DataBuf_t());

        // Extract the completion status from the packet.
        rxbufq.back().cpl_status = GET_CPL_STATUS(pkt->data);
        rxbufq.back().tag        = GET_CPL_TAG(pkt->data);
        rxbufq.back().loaddr     = pkt->data[CPL_LOW_ADDR_OFFSET] & 0x7f;

        // Warn if a bad (non-zero) status
        if (rxbufq.back().cpl_status != CPL_SUCCESS)
        {
            VPrint("**WARNING: InputCallback() received packet with status %s at node %d. Discarding.\n",
                    (rxbufq.back().cpl_status == CPL_UNSUPPORTED) ? "UNSUPPORTED" :
                    (rxbufq.back().cpl_status == CPL_CRS)         ? "CRS"         :
                    (rxbufq.back().cpl_status == CPL_ABORT)       ? "ABORT"       :
                                                                    "UNKNOWN", node);
        }
        // If a successful completion with data, extract the TPL payload data
        else if (pkt->ByteCount)
        {
            // Size the buffer for the incoming data
            rxbufq.back().rxbuf.resize(pkt->ByteCount);

            // Get a pointer to the start of the payload data
            pPktData_t payload = GET_TLP_PAYLOAD_PTR(pkt->data);

            // Fetch data and put in the receive buffer
            DebugVPrint("---> ");
            for (idx = 0; idx < pkt->ByteCount; idx++)
            {
                rxbufq.back().rxbuf[idx] = payload[idx];

                DebugVPrint("%02x ", rxbufq.back().rxbuf[idx]);
                if ((idx % 16) == 15)
                {
                    DebugVPrint("\n---> ");
                }
            }

            if ((idx % 16) != 0)
            {
                DebugVPrint("\n");
            }
        }

        // Once input packet is finished with, the allocated space *must* be freed.
        // All input packets have their own memory space to avoid overwrites with
        // shared buffers.
        DISCARD_PACKET(pkt);
    }
    // Process requests (mem, config space, i/o, message)
    else
    {
        // TO DO
    }
}

//-------------------------------------------------------------
// pcieVcInterface::run()
//
// pcieVcInterface main program.
//
//-------------------------------------------------------------

void pcieVcInterface::run(void)
{
    int        error = 0;
    bool       end   = false;
    int        halt  = 0;

    int        byteidx;
    unsigned   operation;
    unsigned   int_to_model;
    unsigned   option;
    int        pad_offset;

    uint32_t   status;
    uint32_t   be;
    uint32_t   popdata;

    uint64_t   rdata;
    uint64_t   wdata;
    uint64_t   wdatawidth;
    uint64_t   rdatawidth;
    uint64_t   word_len;
    uint64_t   remaining_len;
    uint64_t   address;
    uint64_t   addrlo;

    // Initialise PCIe VHost, with input callback function and no user pointer.
    pcie->initialisePcie(VUserInput, this);

    pcie->getPcieVersionStr(sbuf, strbufsize);
    VPrint("  %s\n", sbuf);

    DebugVPrint("pcieVcInterface::run: on node %d\n", node);

    // Fetch the model parameters
    VRead(LANESADDR,    &link_width,  DELTACYCLE, node);
    VRead(PIPE_ADDR,    &pipe_mode,   DELTACYCLE, node);
    VRead(EP_ADDR,      &ep_mode,     DELTACYCLE, node);
    VRead(EN_ECRC_ADDR, &digest_mode, DELTACYCLE, node);
    VRead(REQID_ADDR,   &rid,         DELTACYCLE, node);

    // When in PIPE mode, disable codec and scrambling
    if (pipe_mode)
    {
        pcie->configurePcie(CONFIG_DISABLE_SCRAMBLING);
        pcie->configurePcie(CONFIG_DISABLE_8B10B);
    }

    // Make sure the link is out of electrical idle
    VWrite(LINK_STATE, 0, DELTACYCLE, node);

    // Use node number as seed
    pcie->pcieSeed(node);

    // Send out idles until reset de-asserted
    do
    {
        pcie->sendIdle();
        VRead(RESET_STATE, &reset_state, CLOCKEDCYCLE, node);
    } while(reset_state);

    // Loop forever, processing commands and driving the PCIe link
    while (!error && !end)
    {
        // Ack transaction
        VWrite(ACKTRANS, 1, DELTACYCLE, node);

        // Check if there is a new transaction (delta)
        VRead(GETNEXTTRANS, &operation, DELTA_CYCLE, node);

        switch (operation)
        {
            case GET_MODEL_OPTIONS :

                VRead(GETOPTIONS,    &option,       DELTACYCLE, node);
                switch (option)
                {
                case GETLASTCMPLSTATUS :
                    VWrite(SETINTFROMMODEL, last_cpl_status, DELTACYCLE, node);
                    break;
                case GETLASTRXREQTAG :
                    VWrite(SETINTFROMMODEL, last_rx_tag, DELTACYCLE, node);
                    break;
                default:
                    VPrint("pcieVcInterface::run : ***ERROR. Unrecognised GET_MODEL_OPTIONS option (%d)\n", option);
                    error++;
                    break;
                }

                break;

            case SET_MODEL_OPTIONS :

                VRead(GETOPTIONS,    &option,       DELTACYCLE, node);
                VRead(GETINTTOMODEL, &int_to_model, DELTACYCLE, node);

                // If a PCIe C model config option, pass straight to model
                if (option < VCOPTIONSTART)
                {
                    pcie->configurePcie(static_cast<config_t>(option), int_to_model);
                }
                else
                {
                    switch(option)
                    {
                    case ENDMODELRUN:
                        end = true;
                        halt = int_to_model;
                        break;

                    // Do PHY layer link training initialisation.
                    case INITPHY:
                        InitLink(link_width, node);
                        break;

                    // Do data link layer flow control initialisation
                    case INITDLL:
                        // Initialise flow control
                        pcie->initFc();
                        break;

                    // Set the transaction layer mode---memory, I/O, config space, completion or message
                    case SETTRANSMODE:
                        trans_mode = (pcie_trans_mode_t)int_to_model;
                        break;

                    case SETCMPLRID:
                        cmplrid = int_to_model;
                        break;

                    case SETCMPLCID:
                        cmplcid = int_to_model;
                        break;
                        
                    case SETCMPLRLEN:
                        cmplrlen = int_to_model;
                        break;

                    case SETCMPLTAG:
                        cmpltag = int_to_model;
                        break;

                    case SETRDLCK:
                        rd_lck = (bool)int_to_model;
                        break;

                    case SETREQTAG:
                        // Update the tag if it is the valid range
                        if (int_to_model >= 0 && int_to_model < TLP_TAG_AUTO)
                        {
                            tag = int_to_model;
                        }
                        break;

                    default:
                        VPrint("pcieVcInterface::run : ***ERROR. Unrecognised SET_MODEL_OPTIONS option (%d)\n", option);
                        error++;
                    }
                }
                break;

            case WRITE_OP :
            case ASYNC_WRITE_ADDRESS :

                VRead64(GETADDRESS,     &address,    DELTACYCLE, node);
                VRead64(GETDATATOMODEL, &wdata,      DELTACYCLE, node);
                VRead64(GETDATAWIDTH,   &wdatawidth, DELTACYCLE, node);

                // For completions, the data bytes will start at an offset into the first word, determined
                // by the address low 2 bits
                pad_offset = (trans_mode == CPL_TRANS) ? (address & 0x3) : 0;

                // Place data into a PCIe model byte buffer, padding beginning with 0s if needed
                for (byteidx = -pad_offset; byteidx < int(wdatawidth/8); byteidx++)
                {
                    txdatabuf[byteidx + pad_offset] = (byteidx < 0 ) ? 0 : ((wdata >> (byteidx<<3)) & 0xff);
                }

                switch(trans_mode)
                {
                case MEM_TRANS :
                    // Do a posted memory write (no completion to wait for)
                    pcie->memWrite(address, txdatabuf, wdatawidth/8, tag++, rid, false, digest_mode);
                    break;

                case MSG_TRANS :
                    if (ASYNC_WRITE_ADDRESS)
                    {
                        pcie->message(address, txdatabuf, wdatawidth/8, tag++, rid, false, digest_mode);
                    }
                    else
                    {
                        pcie->message(address, NULL, 0, tag++, rid, false, digest_mode);
                    }
                    break;

                case CFG_SPC_TRANS :
                    if (!ep_mode)
                    {
                        pcie->cfgWrite(address, txdatabuf, wdatawidth/8, tag++, rid, false, digest_mode);

                        // Non-posted transaction, so do a wait for the status completion
                        pcie->waitForCompletion();

                        last_cpl_status = rxbufq.front().cpl_status;
                        last_rx_tag     = rxbufq.front().tag;

                        // Flag any bad status
                        if (rxbufq.front().cpl_status)
                        {
                            VPrint("pcieVcInterface::run : ***ERROR. Received bad status (%d) on WRITE_OP\n", rxbufq.front().cpl_status);
                            error++;
                        }
                    }
                    else
                    {
                        VPrint("pcieVcInterface::run : ***ERROR. Issuing a configuration space write when an endpoint on WRITE_OP\n");
                        error++;
                    }
                    break;

                case IO_TRANS :

                    pcie->ioWrite(address, txdatabuf, wdatawidth/8, tag++, rid, false, digest_mode);

                    // Non-posted transaction, so do a wait for the status completion
                    pcie->waitForCompletion();

                    last_cpl_status = rxbufq.front().cpl_status;
                    last_rx_tag     = rxbufq.front().tag;

                    // Flag any bad status
                    if (rxbufq.front().cpl_status)
                    {
                        VPrint("pcieVcInterface::run : ***WARNING. Received bad status (%d) on WRITE_OP\n", rxbufq.front().cpl_status);
                        //error++;
                    }
                    break;

                case CPL_TRANS :
                case PART_CPL_TRANS :

                    status        = CPL_SUCCESS;
                    be            = CalcBe(address, wdatawidth/8);
                    word_len      = CalcWordCount(wdatawidth/8, be);
                    remaining_len = (trans_mode == CPL_TRANS) ? word_len : cmplrlen;

                    // Do a completion (effectively posted, so nothing to wait for)
                    pcie->partCompletionDelay(address & CMPL_ADDR_MASK, txdatabuf, status, be & 0xf, (be >> 4) & 0xf, remaining_len, word_len, cmpltag, cmplcid, cmplrid, false, false, digest_mode);
                    break;

                default :
                    VPrint("pcieVcInterface::run : ***ERROR. Unrecognised transaction mode on WRITE_OP (%d)\n", trans_mode);
                    error++;
                    break;
                }

                // For non-posted transactions, pop the completion from the queue
                //if (trans_mode != MEM_TRANS && trans_mode != MSG_TRANS/* && trans_mode != CPL_TRANS*/)
                if (trans_mode == CFG_SPC_TRANS || trans_mode == IO_TRANS)
                {
                    rxbufq.pop();
                }
                break;

            case READ_OP :
            case ASYNC_READ_ADDRESS :
            case READ_ADDRESS :
            case READ_DATA :
            case ASYNC_READ_DATA :

                VRead64(GETADDRESS,   &address,    DELTACYCLE, node);
                VRead64(GETDATAWIDTH, &rdatawidth, DELTACYCLE, node);

                if (operation != READ_DATA && operation != ASYNC_READ_DATA)
                {
                    switch(trans_mode)
                    {
                    case MEM_TRANS :
                        // Instigate a memory read
                        pcie->memRead(address, rdatawidth/8, tag++, rid, false, digest_mode);
                        break;

                    case CFG_SPC_TRANS :
                        if (!ep_mode)
                        {
                            // Instigate a configuration space read
                            pcie->cfgRead(address, rdatawidth/8, tag++, rid, false, digest_mode);
                        }
                        else
                        {
                            VPrint("pcieVcInterface::run : ***ERROR. Issuing a configuration space read when an endpoint on READ_OP\n");
                            error++;
                        }
                        break;

                    case IO_TRANS :
                        // Instigate a configuration space read
                        pcie->ioRead(address, rdatawidth/8, tag++, rid, false, digest_mode);
                        break;

                    default :
                        VPrint("pcieVcInterface::run : ***ERROR. Unrecognised transaction mode on WRITE_OP (%d)\n", trans_mode);
                        error++;
                        break;
                    }
                }

                if (operation != READ_ADDRESS && operation != ASYNC_READ_ADDRESS)
                {
                    // Blocking read, so do a wait for the completion
                    pcie->waitForCompletion();

                    last_cpl_status = rxbufq.front().cpl_status;
                    last_rx_tag     = rxbufq.front().tag;

                    // If a successful completion returned, extract data
                    if (!rxbufq.front().cpl_status)
                    {
                        // Get data
                        addrlo = rxbufq.front().loaddr & 0x3ULL;
                        for (rdata = 0, byteidx = 0; byteidx < (rdatawidth/8); byteidx++)
                        {
                            rdata |= (rxbufq.front().rxbuf[byteidx+addrlo] & 0xff) << (8 * byteidx);
                        }
                    }
                    else
                    {
                        rdata = 0;
                        VWrite(SETBOOLFROMMODEL, 1, DELTACYCLE, node);
                    }

                    // Pop the received packet from the queue
                    rxbufq.pop();

                    // Update transaction record return data
                    VWrite64(SETDATAFROMMODEL, rdata, DELTACYCLE, node);
                }

                break;

            case WRITE_BURST :

                VRead64(GETADDRESS,     &address,    DELTACYCLE, node);
                VRead64(GETDATAWIDTH,   &wdatawidth, DELTACYCLE, node);

                // For completions, the data bytes will start at an offset into the first word, determined
                // by the address low 2 bits
                pad_offset = (trans_mode == CPL_TRANS || trans_mode == PART_CPL_TRANS) ? (address & 0x3) : 0;

                for (int pidx = -pad_offset; pidx < (int)wdatawidth; pidx++)
                {
                    if (pidx < 0)
                    {
                        txdatabuf[pidx + pad_offset] = 0;
                    }
                    else
                    {
                      VRead(POPDATA, &popdata, DELTACYCLE, node);
                      txdatabuf[pidx + pad_offset] = popdata & 0xff;
                    }
                }

                switch(trans_mode)
                {
                case MEM_TRANS :

                    pcie->memWrite(address, txdatabuf, wdatawidth, tag++, rid, false, digest_mode);
                    break;

                case CPL_TRANS :
                case PART_CPL_TRANS :

                    status        = CPL_SUCCESS;
                    be            = CalcBe(address, wdatawidth);
                    word_len      = CalcWordCount(wdatawidth, be);
                    remaining_len = (trans_mode == CPL_TRANS) ? word_len : cmplrlen;

                    // Do a completion (effectively posted, so nothing to wait for). Align the address to a word
                    // boundary and only use the needed lower 7 bits.
                    pcie->partCompletionDelay(address & CMPL_ADDR_MASK, txdatabuf, status, be & 0xf, (be >> 4) & 0xf, remaining_len, word_len, cmpltag, cmplcid, cmplrid, false, false, digest_mode);
                    break;

                default:
                    break;
                }

                break;

            case READ_BURST :

                VRead64(GETADDRESS,     &address,    DELTACYCLE, node);
                VRead64(GETDATAWIDTH,   &rdatawidth, DELTACYCLE, node);

                pcie->memRead(address, rdatawidth, tag++, rid, false, digest_mode);

                // Blocking read, so do a wait for the completion
                pcie->waitForCompletion();

                last_cpl_status = rxbufq.front().cpl_status;
                last_rx_tag     = rxbufq.front().tag;

                // If a successful completion returned, extract data
                if (!rxbufq.front().cpl_status)
                {
                    // Get data
                    addrlo = rxbufq.front().loaddr & 0x3ULL;
                    for (byteidx = 0; byteidx < rdatawidth; byteidx++)
                    {
                        VWrite(PUSHDATA, rxbufq.front().rxbuf[byteidx+addrlo], DELTACYCLE, node);
                    }
                    rxbufq.pop();
                }
                else
                {
                    rdata = 0;
                    VWrite(SETBOOLFROMMODEL, 1, DELTACYCLE, node);
                }

                break;

            case WAIT_FOR_CLOCK :

                VRead(GETINTTOMODEL, &int_to_model, DELTACYCLE, node);
                pcie->sendIdle(int_to_model);
                break;

            case WAIT_FOR_TRANSACTION:

                pcie->waitForCompletion();
                break;

            default :
                VPrint("pcieVcInterface::run : ***ERROR. Unrecognised operation (%d)\n", operation);
                error++;
                break;
        }
    }

    if (error)
    {
        VPrint("***Error: pcieVcInterface::run() had an error\n");

        // Send the simulation with an error
        VWrite(PVH_FATAL, error, 0, node);
    }
    else if (end)
    {
        if (halt == FINISHSIM)
        {
            VWrite(PVH_FINISH, 0, 0, node);
        }
        else if (halt == STOPSIM)
        {
            VWrite(PVH_STOP, 0, 0, node);
        }
    }

    // If reached here without stop/finish, send out idles forever
    // to allow simulation to continue
    while (true)
    {
        pcie->sendIdle(10000);
    }
}