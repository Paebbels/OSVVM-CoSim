// =========================================================================
//
//  File Name:         OsvvmVUserVPrint.h
//  Design Unit Name:
//  Revision:          OSVVM MODELS STANDARD VERSION
//
//  Maintainer:        Simon Southwell email:  simon.southwell@gmail.com
//  Contributor(s):
//     Simon Southwell      simon.southwell@gmail.com
//
//
//  Description:
//      OSVVM definition for VPrint, for PCIe model compatibility
//
//  Revision History:
//    Date      Version    Description
//    10/2025   2026.01    Initial revision
//
//
//  This file is part of OSVVM.
//
//  Copyright (c) 2025 by [OSVVM Authors](../AUTHORS.md)
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

#ifndef _OSVVM_VUSER_VPRINT_H_
#define _OSVVM_VUSER_VPRINT_H_

// -------------------------------------------------------------------------
// DEFINES AND MACROS
// -------------------------------------------------------------------------

#if defined (ACTIVEHDL)
#include <vhpi_user.h>
# define VPrint(...) {vhpi_printf(__VA_ARGS__);}
#else
# define VPrint(...) {printf(__VA_ARGS__);}
#endif

#ifdef DEBUG
#define DebugVPrint VPrint
#else
#define DebugVPrint(...) {}
#endif
  
#endif
