<<<<<<< HEAD
/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComRom.cpp
    \brief    global variables & functions
*/

#include "TComRom.h"
#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
#include <iomanip>
#include <assert.h>
#include "TComDataCU.h"
#include "Debug.h"
// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

//! \ingroup TLibCommon
//! \{

const Char* nalUnitTypeToString(NalUnitType type)
{
  switch (type)
  {
  case NAL_UNIT_CODED_SLICE_TRAIL_R:    return "TRAIL_R";
  case NAL_UNIT_CODED_SLICE_TRAIL_N:    return "TRAIL_N";
  case NAL_UNIT_CODED_SLICE_TSA_R:      return "TSA_R";
  case NAL_UNIT_CODED_SLICE_TSA_N:      return "TSA_N";
  case NAL_UNIT_CODED_SLICE_STSA_R:     return "STSA_R";
  case NAL_UNIT_CODED_SLICE_STSA_N:     return "STSA_N";
  case NAL_UNIT_CODED_SLICE_BLA_W_LP:   return "BLA_W_LP";
  case NAL_UNIT_CODED_SLICE_BLA_W_RADL: return "BLA_W_RADL";
  case NAL_UNIT_CODED_SLICE_BLA_N_LP:   return "BLA_N_LP";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
  case NAL_UNIT_CODED_SLICE_RADL_R:     return "RADL_R";
  case NAL_UNIT_CODED_SLICE_RADL_N:     return "RADL_N";
  case NAL_UNIT_CODED_SLICE_RASL_R:     return "RASL_R";
  case NAL_UNIT_CODED_SLICE_RASL_N:     return "RASL_N";
  case NAL_UNIT_VPS:                    return "VPS";
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_FILLER_DATA:            return "FILLER";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  default:                              return "UNK";
  }
}

class ScanGenerator
{
private:
  UInt m_line, m_column;
  const UInt m_blockWidth, m_blockHeight;
  const UInt m_stride;
  const COEFF_SCAN_TYPE m_scanType;

public:
  ScanGenerator(UInt blockWidth, UInt blockHeight, UInt stride, COEFF_SCAN_TYPE scanType)
    : m_line(0), m_column(0), m_blockWidth(blockWidth), m_blockHeight(blockHeight), m_stride(stride), m_scanType(scanType)
  { }

  UInt GetCurrentX() const { return m_column; }
  UInt GetCurrentY() const { return m_line; }

  UInt GetNextIndex(UInt blockOffsetX, UInt blockOffsetY)
  {
    Int rtn=((m_line + blockOffsetY) * m_stride) + m_column + blockOffsetX;

    //advance line and column to the next position
    switch (m_scanType)
    {
      //------------------------------------------------

      case SCAN_DIAG:
        {
          if ((m_column == (m_blockWidth - 1)) || (m_line == 0)) //if we reach the end of a rank, go diagonally down to the next one
          {
            m_line   += m_column + 1;
            m_column  = 0;

            if (m_line >= m_blockHeight) //if that takes us outside the block, adjust so that we are back on the bottom row
            {
              m_column += m_line - (m_blockHeight - 1);
              m_line    = m_blockHeight - 1;
            }
          }
          else
          {
            m_column++;
            m_line--;
          }
        }
        break;

      //------------------------------------------------

      case SCAN_HOR:
        {
          if (m_column == (m_blockWidth - 1))
          {
            m_line++;
            m_column = 0;
          }
          else
          {
            m_column++;
          }
        }
        break;

      //------------------------------------------------

      case SCAN_VER:
        {
          if (m_line == (m_blockHeight - 1))
          {
            m_column++;
            m_line = 0;
          }
          else
          {
            m_line++;
          }
        }
        break;

      //------------------------------------------------

      default:
        {
          std::cerr << "ERROR: Unknown scan type \"" << m_scanType << "\"in ScanGenerator::GetNextIndex" << std::endl;
          exit(1);
        }
        break;
    }

    return rtn;
  }
};

// initialize ROM variables
Void initROM()
{
  Int i, c;

  // g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, ...
  ::memset( g_aucConvertToBit,   -1, sizeof( g_aucConvertToBit ) );
  c=0;
  for ( i=4; i<=MAX_CU_SIZE; i*=2 )
  {
    g_aucConvertToBit[ i ] = c;
    c++;
  }

  // initialise scan orders
  for(UInt log2BlockHeight = 0; log2BlockHeight < MAX_CU_DEPTH; log2BlockHeight++)
  {
    for(UInt log2BlockWidth = 0; log2BlockWidth < MAX_CU_DEPTH; log2BlockWidth++)
    {
      const UInt blockWidth  = 1 << log2BlockWidth;
      const UInt blockHeight = 1 << log2BlockHeight;
      const UInt totalValues = blockWidth * blockHeight;

      //--------------------------------------------------------------------------------------------------

      //non-grouped scan orders

      for (UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const COEFF_SCAN_TYPE scanType = COEFF_SCAN_TYPE(scanTypeIndex);

        g_scanOrder[SCAN_UNGROUPED][scanType][log2BlockWidth][log2BlockHeight] = new UInt[totalValues];

        ScanGenerator fullBlockScan(blockWidth, blockHeight, blockWidth, scanType);

        for (UInt scanPosition = 0; scanPosition < totalValues; scanPosition++)
        {
          g_scanOrder[SCAN_UNGROUPED][scanType][log2BlockWidth][log2BlockHeight][scanPosition] = fullBlockScan.GetNextIndex(0, 0);
        }
      }

      //--------------------------------------------------------------------------------------------------

      //grouped scan orders

      const UInt  groupWidth           = 1           << MLS_CG_LOG2_WIDTH;
      const UInt  groupHeight          = 1           << MLS_CG_LOG2_HEIGHT;
      const UInt  widthInGroups        = blockWidth  >> MLS_CG_LOG2_WIDTH;
      const UInt  heightInGroups       = blockHeight >> MLS_CG_LOG2_HEIGHT;

      const UInt  groupSize            = groupWidth    * groupHeight;
      const UInt  totalGroups          = widthInGroups * heightInGroups;

      for (UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const COEFF_SCAN_TYPE scanType = COEFF_SCAN_TYPE(scanTypeIndex);

        g_scanOrder[SCAN_GROUPED_4x4][scanType][log2BlockWidth][log2BlockHeight] = new UInt[totalValues];

        ScanGenerator fullBlockScan(widthInGroups, heightInGroups, groupWidth, scanType);

        for (UInt groupIndex = 0; groupIndex < totalGroups; groupIndex++)
        {
          const UInt groupPositionY  = fullBlockScan.GetCurrentY();
          const UInt groupPositionX  = fullBlockScan.GetCurrentX();
          const UInt groupOffsetX    = groupPositionX * groupWidth;
          const UInt groupOffsetY    = groupPositionY * groupHeight;
          const UInt groupOffsetScan = groupIndex     * groupSize;

          ScanGenerator groupScan(groupWidth, groupHeight, blockWidth, scanType);

          for (UInt scanPosition = 0; scanPosition < groupSize; scanPosition++)
          {
            g_scanOrder[SCAN_GROUPED_4x4][scanType][log2BlockWidth][log2BlockHeight][groupOffsetScan + scanPosition] = groupScan.GetNextIndex(groupOffsetX, groupOffsetY);
          }

          fullBlockScan.GetNextIndex(0,0);
        }
      }

      //--------------------------------------------------------------------------------------------------
    }
  }
}

Void destroyROM()
{
  for(UInt groupTypeIndex = 0; groupTypeIndex < SCAN_NUMBER_OF_GROUP_TYPES; groupTypeIndex++)
  {
    for (UInt scanOrderIndex = 0; scanOrderIndex < SCAN_NUMBER_OF_TYPES; scanOrderIndex++)
    {
      for (UInt log2BlockWidth = 0; log2BlockWidth < MAX_CU_DEPTH; log2BlockWidth++)
      {
        for (UInt log2BlockHeight = 0; log2BlockHeight < MAX_CU_DEPTH; log2BlockHeight++)
        {
          delete [] g_scanOrder[groupTypeIndex][scanOrderIndex][log2BlockWidth][log2BlockHeight];
        }
      }
    }
  }
}

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

UInt g_auiZscanToRaster [ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ] = { 0, };
UInt g_auiRasterToZscan [ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ] = { 0, };
UInt g_auiRasterToPelX  [ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ] = { 0, };
UInt g_auiRasterToPelY  [ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ] = { 0, };

const UInt g_auiPUOffset[NUMBER_OF_PART_SIZES] = { 0, 8, 4, 4, 2, 10, 1, 5};

Void initZscanToRaster ( Int iMaxDepth, Int iDepth, UInt uiStartVal, UInt*& rpuiCurrIdx )
{
  Int iStride = 1 << ( iMaxDepth - 1 );

  if ( iDepth == iMaxDepth )
  {
    rpuiCurrIdx[0] = uiStartVal;
    rpuiCurrIdx++;
  }
  else
  {
    Int iStep = iStride >> iDepth;
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal,                     rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep,               rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep*iStride,       rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep*iStride+iStep, rpuiCurrIdx );
  }
}

Void initRasterToZscan ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth )
{
  UInt  uiMinCUWidth  = uiMaxCUWidth  >> ( uiMaxDepth - 1 );
  UInt  uiMinCUHeight = uiMaxCUHeight >> ( uiMaxDepth - 1 );

  UInt  uiNumPartInWidth  = (UInt)uiMaxCUWidth  / uiMinCUWidth;
  UInt  uiNumPartInHeight = (UInt)uiMaxCUHeight / uiMinCUHeight;

  for ( UInt i = 0; i < uiNumPartInWidth*uiNumPartInHeight; i++ )
  {
    g_auiRasterToZscan[ g_auiZscanToRaster[i] ] = i;
  }
}

Void initRasterToPelXY ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth )
{
  UInt    i;

  UInt* uiTempX = &g_auiRasterToPelX[0];
  UInt* uiTempY = &g_auiRasterToPelY[0];

  UInt  uiMinCUWidth  = uiMaxCUWidth  >> ( uiMaxDepth - 1 );
  UInt  uiMinCUHeight = uiMaxCUHeight >> ( uiMaxDepth - 1 );

  UInt  uiNumPartInWidth  = uiMaxCUWidth  / uiMinCUWidth;
  UInt  uiNumPartInHeight = uiMaxCUHeight / uiMinCUHeight;

  uiTempX[0] = 0; uiTempX++;
  for ( i = 1; i < uiNumPartInWidth; i++ )
  {
    uiTempX[0] = uiTempX[-1] + uiMinCUWidth; uiTempX++;
  }
  for ( i = 1; i < uiNumPartInHeight; i++ )
  {
    memcpy(uiTempX, uiTempX-uiNumPartInWidth, sizeof(UInt)*uiNumPartInWidth);
    uiTempX += uiNumPartInWidth;
  }

  for ( i = 1; i < uiNumPartInWidth*uiNumPartInHeight; i++ )
  {
    uiTempY[i] = ( i / uiNumPartInWidth ) * uiMinCUWidth;
  }
}

const Int g_quantScales[SCALING_LIST_REM_NUM] =
{
  26214,23302,20560,18396,16384,14564
};

const Int g_invQuantScales[SCALING_LIST_REM_NUM] =
{
  40,45,51,57,64,72
};

//--------------------------------------------------------------------------------------------------

//structures

#define DEFINE_DST4x4_MATRIX(a,b,c,d) \
{ \
  {  a,  b,  c,  d }, \
  {  c,  c,  0, -c }, \
  {  d, -a, -c,  b }, \
  {  b, -d,  c, -a }, \
}

#define DEFINE_DCT4x4_MATRIX(a,b,c) \
{ \
  { a,  a,  a,  a}, \
  { b,  c, -c, -b}, \
  { a, -a, -a,  a}, \
  { c, -b,  b, -c}  \
}

#define DEFINE_DCT8x8_MATRIX(a,b,c,d,e,f,g) \
{ \
  { a,  a,  a,  a,  a,  a,  a,  a}, \
  { d,  e,  f,  g, -g, -f, -e, -d}, \
  { b,  c, -c, -b, -b, -c,  c,  b}, \
  { e, -g, -d, -f,  f,  d,  g, -e}, \
  { a, -a, -a,  a,  a, -a, -a,  a}, \
  { f, -d,  g,  e, -e, -g,  d, -f}, \
  { c, -b,  b, -c, -c,  b, -b,  c}, \
  { g, -f,  e, -d,  d, -e,  f, -g}  \
}

#define DEFINE_DCT16x16_MATRIX(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o) \
{ \
  { a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a}, \
  { h,  i,  j,  k,  l,  m,  n,  o, -o, -n, -m, -l, -k, -j, -i, -h}, \
  { d,  e,  f,  g, -g, -f, -e, -d, -d, -e, -f, -g,  g,  f,  e,  d}, \
  { i,  l,  o, -m, -j, -h, -k, -n,  n,  k,  h,  j,  m, -o, -l, -i}, \
  { b,  c, -c, -b, -b, -c,  c,  b,  b,  c, -c, -b, -b, -c,  c,  b}, \
  { j,  o, -k, -i, -n,  l,  h,  m, -m, -h, -l,  n,  i,  k, -o, -j}, \
  { e, -g, -d, -f,  f,  d,  g, -e, -e,  g,  d,  f, -f, -d, -g,  e}, \
  { k, -m, -i,  o,  h,  n, -j, -l,  l,  j, -n, -h, -o,  i,  m, -k}, \
  { a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a}, \
  { l, -j, -n,  h, -o, -i,  m,  k, -k, -m,  i,  o, -h,  n,  j, -l}, \
  { f, -d,  g,  e, -e, -g,  d, -f, -f,  d, -g, -e,  e,  g, -d,  f}, \
  { m, -h,  l,  n, -i,  k,  o, -j,  j, -o, -k,  i, -n, -l,  h, -m}, \
  { c, -b,  b, -c, -c,  b, -b,  c,  c, -b,  b, -c, -c,  b, -b,  c}, \
  { n, -k,  h, -j,  m,  o, -l,  i, -i,  l, -o, -m,  j, -h,  k, -n}, \
  { g, -f,  e, -d,  d, -e,  f, -g, -g,  f, -e,  d, -d,  e, -f,  g}, \
  { o, -n,  m, -l,  k, -j,  i, -h,  h, -i,  j, -k,  l, -m,  n, -o}  \
}

#define DEFINE_DCT32x32_MATRIX(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,A,B,C,D,E) \
{ \
  { a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a}, \
  { p,  q,  r,  s,  t,  u,  v,  w,  x,  y,  z,  A,  B,  C,  D,  E, -E, -D, -C, -B, -A, -z, -y, -x, -w, -v, -u, -t, -s, -r, -q, -p}, \
  { h,  i,  j,  k,  l,  m,  n,  o, -o, -n, -m, -l, -k, -j, -i, -h, -h, -i, -j, -k, -l, -m, -n, -o,  o,  n,  m,  l,  k,  j,  i,  h}, \
  { q,  t,  w,  z,  C, -E, -B, -y, -v, -s, -p, -r, -u, -x, -A, -D,  D,  A,  x,  u,  r,  p,  s,  v,  y,  B,  E, -C, -z, -w, -t, -q}, \
  { d,  e,  f,  g, -g, -f, -e, -d, -d, -e, -f, -g,  g,  f,  e,  d,  d,  e,  f,  g, -g, -f, -e, -d, -d, -e, -f, -g,  g,  f,  e,  d}, \
  { r,  w,  B, -D, -y, -t, -p, -u, -z, -E,  A,  v,  q,  s,  x,  C, -C, -x, -s, -q, -v, -A,  E,  z,  u,  p,  t,  y,  D, -B, -w, -r}, \
  { i,  l,  o, -m, -j, -h, -k, -n,  n,  k,  h,  j,  m, -o, -l, -i, -i, -l, -o,  m,  j,  h,  k,  n, -n, -k, -h, -j, -m,  o,  l,  i}, \
  { s,  z, -D, -w, -p, -v, -C,  A,  t,  r,  y, -E, -x, -q, -u, -B,  B,  u,  q,  x,  E, -y, -r, -t, -A,  C,  v,  p,  w,  D, -z, -s}, \
  { b,  c, -c, -b, -b, -c,  c,  b,  b,  c, -c, -b, -b, -c,  c,  b,  b,  c, -c, -b, -b, -c,  c,  b,  b,  c, -c, -b, -b, -c,  c,  b}, \
  { t,  C, -y, -p, -x,  D,  u,  s,  B, -z, -q, -w,  E,  v,  r,  A, -A, -r, -v, -E,  w,  q,  z, -B, -s, -u, -D,  x,  p,  y, -C, -t}, \
  { j,  o, -k, -i, -n,  l,  h,  m, -m, -h, -l,  n,  i,  k, -o, -j, -j, -o,  k,  i,  n, -l, -h, -m,  m,  h,  l, -n, -i, -k,  o,  j}, \
  { u, -E, -t, -v,  D,  s,  w, -C, -r, -x,  B,  q,  y, -A, -p, -z,  z,  p,  A, -y, -q, -B,  x,  r,  C, -w, -s, -D,  v,  t,  E, -u}, \
  { e, -g, -d, -f,  f,  d,  g, -e, -e,  g,  d,  f, -f, -d, -g,  e,  e, -g, -d, -f,  f,  d,  g, -e, -e,  g,  d,  f, -f, -d, -g,  e}, \
  { v, -B, -p, -C,  u,  w, -A, -q, -D,  t,  x, -z, -r, -E,  s,  y, -y, -s,  E,  r,  z, -x, -t,  D,  q,  A, -w, -u,  C,  p,  B, -v}, \
  { k, -m, -i,  o,  h,  n, -j, -l,  l,  j, -n, -h, -o,  i,  m, -k, -k,  m,  i, -o, -h, -n,  j,  l, -l, -j,  n,  h,  o, -i, -m,  k}, \
  { w, -y, -u,  A,  s, -C, -q,  E,  p,  D, -r, -B,  t,  z, -v, -x,  x,  v, -z, -t,  B,  r, -D, -p, -E,  q,  C, -s, -A,  u,  y, -w}, \
  { a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a}, \
  { x, -v, -z,  t,  B, -r, -D,  p, -E, -q,  C,  s, -A, -u,  y,  w, -w, -y,  u,  A, -s, -C,  q,  E, -p,  D,  r, -B, -t,  z,  v, -x}, \
  { l, -j, -n,  h, -o, -i,  m,  k, -k, -m,  i,  o, -h,  n,  j, -l, -l,  j,  n, -h,  o,  i, -m, -k,  k,  m, -i, -o,  h, -n, -j,  l}, \
  { y, -s, -E,  r, -z, -x,  t,  D, -q,  A,  w, -u, -C,  p, -B, -v,  v,  B, -p,  C,  u, -w, -A,  q, -D, -t,  x,  z, -r,  E,  s, -y}, \
  { f, -d,  g,  e, -e, -g,  d, -f, -f,  d, -g, -e,  e,  g, -d,  f,  f, -d,  g,  e, -e, -g,  d, -f, -f,  d, -g, -e,  e,  g, -d,  f}, \
  { z, -p,  A,  y, -q,  B,  x, -r,  C,  w, -s,  D,  v, -t,  E,  u, -u, -E,  t, -v, -D,  s, -w, -C,  r, -x, -B,  q, -y, -A,  p, -z}, \
  { m, -h,  l,  n, -i,  k,  o, -j,  j, -o, -k,  i, -n, -l,  h, -m, -m,  h, -l, -n,  i, -k, -o,  j, -j,  o,  k, -i,  n,  l, -h,  m}, \
  { A, -r,  v, -E, -w,  q, -z, -B,  s, -u,  D,  x, -p,  y,  C, -t,  t, -C, -y,  p, -x, -D,  u, -s,  B,  z, -q,  w,  E, -v,  r, -A}, \
  { c, -b,  b, -c, -c,  b, -b,  c,  c, -b,  b, -c, -c,  b, -b,  c,  c, -b,  b, -c, -c,  b, -b,  c,  c, -b,  b, -c, -c,  b, -b,  c}, \
  { B, -u,  q, -x,  E,  y, -r,  t, -A, -C,  v, -p,  w, -D, -z,  s, -s,  z,  D, -w,  p, -v,  C,  A, -t,  r, -y, -E,  x, -q,  u, -B}, \
  { n, -k,  h, -j,  m,  o, -l,  i, -i,  l, -o, -m,  j, -h,  k, -n, -n,  k, -h,  j, -m, -o,  l, -i,  i, -l,  o,  m, -j,  h, -k,  n}, \
  { C, -x,  s, -q,  v, -A, -E,  z, -u,  p, -t,  y, -D, -B,  w, -r,  r, -w,  B,  D, -y,  t, -p,  u, -z,  E,  A, -v,  q, -s,  x, -C}, \
  { g, -f,  e, -d,  d, -e,  f, -g, -g,  f, -e,  d, -d,  e, -f,  g,  g, -f,  e, -d,  d, -e,  f, -g, -g,  f, -e,  d, -d,  e, -f,  g}, \
  { D, -A,  x, -u,  r, -p,  s, -v,  y, -B,  E,  C, -z,  w, -t,  q, -q,  t, -w,  z, -C, -E,  B, -y,  v, -s,  p, -r,  u, -x,  A, -D}, \
  { o, -n,  m, -l,  k, -j,  i, -h,  h, -i,  j, -k,  l, -m,  n, -o, -o,  n, -m,  l, -k,  j, -i,  h, -h,  i, -j,  k, -l,  m, -n,  o}, \
  { E, -D,  C, -B,  A, -z,  y, -x,  w, -v,  u, -t,  s, -r,  q, -p,  p, -q,  r, -s,  t, -u,  v, -w,  x, -y,  z, -A,  B, -C,  D, -E}  \
}

//--------------------------------------------------------------------------------------------------

//coefficients

#if RExt__HIGH_PRECISION_FORWARD_TRANSFORM
const TMatrixCoeff g_aiT4 [TRANSFORM_NUMBER_OF_DIRECTIONS][4][4]   =
{
  DEFINE_DCT4x4_MATRIX  (16384, 21266,  9224),
  DEFINE_DCT4x4_MATRIX  (   64,    83,    36)
};

const TMatrixCoeff g_aiT8 [TRANSFORM_NUMBER_OF_DIRECTIONS][8][8]   =
{
  DEFINE_DCT8x8_MATRIX  (16384, 21266,  9224, 22813, 19244, 12769,  4563),
  DEFINE_DCT8x8_MATRIX  (   64,    83,    36,    89,    75,    50,    18)
};

const TMatrixCoeff g_aiT16[TRANSFORM_NUMBER_OF_DIRECTIONS][16][16] =
{
  DEFINE_DCT16x16_MATRIX(16384, 21266,  9224, 22813, 19244, 12769,  4563, 23120, 22063, 20450, 17972, 14642, 11109,  6446,  2316),
  DEFINE_DCT16x16_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9)
};

const TMatrixCoeff g_aiT32[TRANSFORM_NUMBER_OF_DIRECTIONS][32][32] =
{
  DEFINE_DCT32x32_MATRIX(16384, 21266,  9224, 22813, 19244, 12769,  4563, 23120, 22063, 20450, 17972, 14642, 11109,  6446,  2316, 23106, 22852, 22445, 21848, 20995, 19810, 18601, 17143, 15718, 13853, 11749,  9846,  7908,  5573,  3281,   946),
  DEFINE_DCT32x32_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9,    90,    90,    88,    85,    82,    78,    73,    67,    61,    54,    46,    38,    31,    22,    13,     4)
};

const TMatrixCoeff g_as_DST_MAT_4[TRANSFORM_NUMBER_OF_DIRECTIONS][4][4] =
{
  DEFINE_DST4x4_MATRIX( 7424, 14081, 18893, 21505),
  DEFINE_DST4x4_MATRIX(   29,    55,    74,    84)
};

#else

const TMatrixCoeff g_aiT4 [TRANSFORM_NUMBER_OF_DIRECTIONS][4][4]   =
{
  DEFINE_DCT4x4_MATRIX  (   64,    83,    36),
  DEFINE_DCT4x4_MATRIX  (   64,    83,    36)
};

const TMatrixCoeff g_aiT8 [TRANSFORM_NUMBER_OF_DIRECTIONS][8][8]   =
{
  DEFINE_DCT8x8_MATRIX  (   64,    83,    36,    89,    75,    50,    18),
  DEFINE_DCT8x8_MATRIX  (   64,    83,    36,    89,    75,    50,    18)
};

const TMatrixCoeff g_aiT16[TRANSFORM_NUMBER_OF_DIRECTIONS][16][16] =
{
  DEFINE_DCT16x16_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9),
  DEFINE_DCT16x16_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9)
};

const TMatrixCoeff g_aiT32[TRANSFORM_NUMBER_OF_DIRECTIONS][32][32] =
{
  DEFINE_DCT32x32_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9,    90,    90,    88,    85,    82,    78,    73,    67,    61,    54,    46,    38,    31,    22,    13,     4),
  DEFINE_DCT32x32_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9,    90,    90,    88,    85,    82,    78,    73,    67,    61,    54,    46,    38,    31,    22,    13,     4)
};

const TMatrixCoeff g_as_DST_MAT_4[TRANSFORM_NUMBER_OF_DIRECTIONS][4][4] =
{
  DEFINE_DST4x4_MATRIX(   29,    55,    74,    84),
  DEFINE_DST4x4_MATRIX(   29,    55,    74,    84)
};
#endif


//--------------------------------------------------------------------------------------------------

#undef DEFINE_DST4x4_MATRIX
#undef DEFINE_DCT4x4_MATRIX
#undef DEFINE_DCT8x8_MATRIX
#undef DEFINE_DCT16x16_MATRIX
#undef DEFINE_DCT32x32_MATRIX

//--------------------------------------------------------------------------------------------------


const UChar g_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize]=
{
  //0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,29,30,31,32,33,33,34,34,35,35,36,36,37,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,51,51,51,51,51 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,51,51,51,51,51 }
};

// ====================================================================================================================
// Intra prediction
// ====================================================================================================================

const UChar g_aucIntraModeNumFast_UseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  8,  //   4x4
  8,  //   8x8
  3,  //  16x16
  3,  //  32x32
  3   //  64x64
};
const UChar g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  9,  //   4x4
  9,  //   8x8
  4,  //  16x16   33
  4,  //  32x32   33
  5   //  64x64   33
};

const UChar g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE] =
  //0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, DM
  { 0, 1, 2, 2, 2, 2, 3, 5, 7, 8, 10, 12, 13, 15, 17, 18, 19, 20, 21, 22, 23, 23, 24, 24, 25, 25, 26, 27, 27, 28, 28, 29, 29, 30, 31, DM_CHROMA_IDX};

// ====================================================================================================================
// Misc.
// ====================================================================================================================

Char  g_aucConvertToBit  [ MAX_CU_SIZE+1 ];

#if ENC_DEC_TRACE
FILE*  g_hTrace = NULL; // Set to NULL to open up a file. Set to stdout to use the current output
const Bool g_bEncDecTraceEnable  = true;
const Bool g_bEncDecTraceDisable = false;
Bool   g_HLSTraceEnable = true;
Bool   g_bJustDoIt = false;
UInt64 g_nSymbolCounter = 0;
#endif
// ====================================================================================================================
// Scanning order & context model mapping
// ====================================================================================================================

// scanning order table
UInt* g_scanOrder[SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][ MAX_CU_DEPTH ][ MAX_CU_DEPTH ];

const UInt ctxIndMap4x4[4*4] =
{
  0, 1, 4, 5,
  2, 3, 4, 5,
  6, 6, 8, 8,
  7, 7, 8, 8
};

const UInt g_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ] = {0,1,2,3,4,6,8,12,16,24};
const UInt g_uiGroupIdx[ MAX_TU_SIZE ]   = {0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9};

const Char *MatrixType[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
    "INTRA4X4_LUMA",
    "INTRA4X4_CHROMAU",
    "INTRA4X4_CHROMAV",
    "INTER4X4_LUMA",
    "INTER4X4_CHROMAU",
    "INTER4X4_CHROMAV"
  },
  {
    "INTRA8X8_LUMA",
    "INTRA8X8_CHROMAU",
    "INTRA8X8_CHROMAV",
    "INTER8X8_LUMA",
    "INTER8X8_CHROMAU",
    "INTER8X8_CHROMAV"
  },
  {
    "INTRA16X16_LUMA",
    "INTRA16X16_CHROMAU",
    "INTRA16X16_CHROMAV",
    "INTER16X16_LUMA",
    "INTER16X16_CHROMAU",
    "INTER16X16_CHROMAV"
  },
  {
   "INTRA32X32_LUMA",
   "INTRA32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTRA32X32_CHROMAV_FROM16x16_CHROMAV",
   "INTER32X32_LUMA",
   "INTER32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTER32X32_CHROMAV_FROM16x16_CHROMAV"
  },
};

const Char *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
  },
  {
  },
  {
    "INTRA16X16_LUMA_DC",
    "INTRA16X16_CHROMAU_DC",
    "INTRA16X16_CHROMAV_DC",
    "INTER16X16_LUMA_DC",
    "INTER16X16_CHROMAU_DC",
    "INTER16X16_CHROMAV_DC"
  },
  {
    "INTRA32X32_LUMA_DC",
    "INTRA32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTRA32X32_CHROMAV_DC_FROM16x16_CHROMAV",
    "INTER32X32_LUMA_DC",
    "INTER32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTER32X32_CHROMAV_DC_FROM16x16_CHROMAV"
  },
};

const Int g_quantTSDefault4x4[4*4] =
{
  16,16,16,16,
  16,16,16,16,
  16,16,16,16,
  16,16,16,16
};

const Int g_quantIntraDefault8x8[8*8] =
{
  16,16,16,16,17,18,21,24,
  16,16,16,16,17,19,22,25,
  16,16,17,18,20,22,25,29,
  16,16,18,21,24,27,31,36,
  17,17,20,24,30,35,41,47,
  18,19,22,27,35,44,54,65,
  21,22,25,31,41,54,70,88,
  24,25,29,36,47,65,88,115
};

const Int g_quantInterDefault8x8[8*8] =
{
  16,16,16,16,17,18,20,24,
  16,16,16,17,18,20,24,25,
  16,16,17,18,20,24,25,28,
  16,17,18,20,24,25,28,33,
  17,18,20,24,25,28,33,41,
  18,20,24,25,28,33,41,54,
  20,24,25,28,33,41,54,71,
  24,25,28,33,41,54,71,91
};

const UInt g_scalingListSize   [SCALING_LIST_SIZE_NUM] = {16,64,256,1024};
const UInt g_scalingListSizeX  [SCALING_LIST_SIZE_NUM] = { 4, 8, 16,  32};

//! \}
=======
/* ====================================================================================================================

  The copyright in this software is being made available under the License included below.
  This software may be subject to other third party and   contributor rights, including patent rights, and no such
  rights are granted under this license.

  Copyright (c) 2010, SAMSUNG ELECTRONICS CO., LTD. and BRITISH BROADCASTING CORPORATION
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted only for
  the purpose of developing standards within the Joint Collaborative Team on Video Coding and for testing and
  promoting such standards. The following conditions are required to be met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
      the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
      the following disclaimer in the documentation and/or other materials provided with the distribution.
    * Neither the name of SAMSUNG ELECTRONICS CO., LTD. nor the name of the BRITISH BROADCASTING CORPORATION
      may be used to endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 * ====================================================================================================================
*/

/** \file     TComRom.cpp
    \brief    global variables & functions
*/

#include "TComRom.h"
#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

// initialize ROM variables
Void initROM()
{
  Int i, c;

  // g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, ...
  ::memset( g_aucConvertToBit,   -1, sizeof( g_aucConvertToBit ) );
  c=0;
  for ( i=4; i<MAX_CU_SIZE; i*=2 )
  {
    g_aucConvertToBit[ i ] = c;
    c++;
  }
  g_aucConvertToBit[ i ] = c;

  // g_auiFrameScanXY[ g_aucConvertToBit[ transformSize ] ]: zigzag scan array for transformSize
#if HHI_RQT
  c=2;
#else
  c=4;
#endif
  for ( i=0; i<MAX_CU_DEPTH; i++ )
  {
    g_auiFrameScanXY[ i ] = new UInt[ c*c ];
    g_auiFrameScanX [ i ] = new UInt[ c*c ];
    g_auiFrameScanY [ i ] = new UInt[ c*c ];
    initFrameScanXY( g_auiFrameScanXY[i], g_auiFrameScanX[i], g_auiFrameScanY[i], c, c );
    c <<= 1;
  }

#if HHI_TRANSFORM_CODING
  // init adaptive scan for sig/last SE coding
  for ( i = 0; i < MAX_CU_DEPTH+1; i++ )
  {
    const int   iBlockSize    = 1 << i;
    const UInt  uiNumScanPos  = UInt( iBlockSize * iBlockSize );
    g_auiSigLastScan[ i ][ 0 ] = new UInt[ uiNumScanPos ];
    g_auiSigLastScan[ i ][ 1 ] = new UInt[ uiNumScanPos ];
    initSigLastScanPattern( g_auiSigLastScan[ i ][ 1 ], i, true  );
    initSigLastScanPattern( g_auiSigLastScan[ i ][ 0 ], i, false );
  }
#endif

#if QC_MDDT
  int ipredmode;
  for(ipredmode=0; ipredmode<9; ipredmode++)
  {
    scanOrder4x4[ipredmode] = new UInt[ 4*4 ];
    scanOrder4x4X[ipredmode]= new UInt[ 4*4 ];
    scanOrder4x4Y[ipredmode]= new UInt[ 4*4 ];
    
    scanStats4x4[ipredmode] = new UInt[ 4*4 ];
    
    
    scanOrder8x8[ipredmode] = new UInt[ 8*8 ];
    scanOrder8x8X[ipredmode]= new UInt[ 8*8 ];
    scanOrder8x8Y[ipredmode]= new UInt[ 8*8 ];
    
    scanStats8x8[ipredmode] = new UInt[ 8*8 ];
  }
  
  // 16x16
  for (int z=0; z < NUM_SCANS_16x16; z++)
  {
    scanOrder16x16[z] = new UInt[ 16*16 ];
    scanOrder16x16X[z] = new UInt[ 16*16 ];
    scanOrder16x16Y[z] = new UInt[ 16*16 ];
    scanStats16x16[z] = new UInt[ 16*16 ];
  }
  
  // 32x32
  for (int z=0; z < NUM_SCANS_32x32; z++)
  {
    scanOrder32x32[z] = new UInt[ 32*32 ];
    scanOrder32x32X[z] = new UInt[ 32*32 ];
    scanOrder32x32Y[z] = new UInt[ 32*32 ];
    scanStats32x32[z] = new UInt[ 32*32 ];
  }
  
  // 64x64
  for (int z=0; z < NUM_SCANS_64x64; z++)
  {
    scanOrder64x64[z] = new UInt[ 64*64 ];
    scanOrder64x64X[z] = new UInt[ 64*64 ];
    scanOrder64x64Y[z] = new UInt[ 64*64 ];
    scanStats64x64[z] = new UInt[ 64*64 ];
  }
#endif
}

Void destroyROM()
{
  Int i;

  for ( i=0; i<MAX_CU_DEPTH; i++ )
  {
    delete[] g_auiFrameScanXY[i];
    delete[] g_auiFrameScanX [i];
    delete[] g_auiFrameScanY [i];
  }
  
#if HHI_TRANSFORM_CODING
  for ( i=0; i<MAX_CU_DEPTH+1; i++ )
  {
    delete[] g_auiSigLastScan[i][0];
    delete[] g_auiSigLastScan[i][1];
  }
#endif

#if QC_MDDT //ADAPTIVE_SCAN
  int ipredmode;
  for(ipredmode=0; ipredmode<9; ipredmode++)
  {       
    delete [] scanOrder4x4[ipredmode];      
    delete [] scanOrder4x4X[ipredmode];      
    delete [] scanOrder4x4Y[ipredmode];
    delete [] scanStats4x4[ipredmode];
    
    delete [] scanOrder8x8[ipredmode];
    delete [] scanOrder8x8X[ipredmode];
    delete [] scanOrder8x8Y[ipredmode];
    delete [] scanStats8x8[ipredmode];
  }
  
  // 16x16
  for (int z=0; z < NUM_SCANS_16x16; z++)
  {
    delete [] scanOrder16x16[z];
    delete [] scanOrder16x16X[z];
    delete [] scanOrder16x16Y[z];
    delete [] scanStats16x16[z];
  }
  // 32x32
  for (int z=0; z < NUM_SCANS_32x32; z++)
  {
    delete [] scanOrder32x32[z];
    delete [] scanOrder32x32X[z];
    delete [] scanOrder32x32Y[z];
    delete [] scanStats32x32[z];
  }
  // 64x64
  for (int z=0; z < NUM_SCANS_64x64; z++)
  {
    delete [] scanOrder64x64[z];
    delete [] scanOrder64x64X[z];
    delete [] scanOrder64x64Y[z];
    delete [] scanStats64x64[z];
  }
#endif
}

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

UInt g_uiMaxCUWidth  = MAX_CU_SIZE;
UInt g_uiMaxCUHeight = MAX_CU_SIZE;
UInt g_uiMaxCUDepth  = MAX_CU_DEPTH;
UInt g_uiAddCUDepth  = 0;

UInt g_auiZscanToRaster [ MAX_NUM_SPU_W*MAX_NUM_SPU_W ] = { 0, };
UInt g_auiRasterToZscan [ MAX_NUM_SPU_W*MAX_NUM_SPU_W ] = { 0, };
UInt g_auiRasterToPelX  [ MAX_NUM_SPU_W*MAX_NUM_SPU_W ] = { 0, };
UInt g_auiRasterToPelY  [ MAX_NUM_SPU_W*MAX_NUM_SPU_W ] = { 0, };

#if HHI_MRG_PU
UInt g_auiPUOffset[8] = { 0, 8, 4, 4, 2, 10, 1, 5 };
#endif

Void initZscanToRaster ( Int iMaxDepth, Int iDepth, UInt uiStartVal, UInt*& rpuiCurrIdx )
{
  Int iStride = 1 << ( iMaxDepth - 1 );

  if ( iDepth == iMaxDepth )
  {
    rpuiCurrIdx[0] = uiStartVal;
    rpuiCurrIdx++;
  }
  else
  {
    Int iStep = iStride >> iDepth;
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal,                     rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep,               rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep*iStride,       rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep*iStride+iStep, rpuiCurrIdx );
  }
}

Void initRasterToZscan ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth )
{
  UInt  uiMinCUWidth  = uiMaxCUWidth  >> ( uiMaxDepth - 1 );
  UInt  uiMinCUHeight = uiMaxCUHeight >> ( uiMaxDepth - 1 );

  UInt  uiNumPartInWidth  = (UInt)uiMaxCUWidth  / uiMinCUWidth;
  UInt  uiNumPartInHeight = (UInt)uiMaxCUHeight / uiMinCUHeight;

  for ( UInt i = 0; i < uiNumPartInWidth*uiNumPartInHeight; i++ )
  {
    g_auiRasterToZscan[ g_auiZscanToRaster[i] ] = i;
  }
}

Void initRasterToPelXY ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth )
{
  UInt    i;

  UInt* uiTempX = &g_auiRasterToPelX[0];
  UInt* uiTempY = &g_auiRasterToPelY[0];

  UInt  uiMinCUWidth  = uiMaxCUWidth  >> ( uiMaxDepth - 1 );
  UInt  uiMinCUHeight = uiMaxCUHeight >> ( uiMaxDepth - 1 );

  UInt  uiNumPartInWidth  = uiMaxCUWidth  / uiMinCUWidth;
  UInt  uiNumPartInHeight = uiMaxCUHeight / uiMinCUHeight;

  uiTempX[0] = 0; uiTempX++;
  for ( i = 1; i < uiNumPartInWidth; i++ )
  {
    uiTempX[0] = uiTempX[-1] + uiMinCUWidth; uiTempX++;
  }
  for ( i = 1; i < uiNumPartInHeight; i++ )
  {
    memcpy(uiTempX, uiTempX-uiNumPartInWidth, sizeof(UInt)*uiNumPartInWidth);
    uiTempX += uiNumPartInWidth;
  }

  for ( i = 1; i < uiNumPartInWidth*uiNumPartInHeight; i++ )
  {
    uiTempY[i] = ( i / uiNumPartInWidth ) * uiMinCUWidth;
  }
};

// ====================================================================================================================
// Quantization & DeQuantization
// ====================================================================================================================

UInt g_aiQuantCoef4[6] =
{
  102, 93, 79, 73, 64, 57
};

Int g_aiDequantCoef4[6] =
{
  10, 11, 13, 14, 16,18
};

UInt g_aiQuantCoef[6][16] =
{
  { 13107, 8066,13107, 8066,
     8066, 5243, 8066, 5243,
    13107, 8066,13107, 8066,
     8066, 5243, 8066, 5243
  },
  { 11916, 7490,11916, 7490,
     7490, 4660, 7490, 4660,
    11916, 7490,11916, 7490,
     7490, 4660, 7490, 4660
  },
  { 10082, 6554,10082, 6554,
     6554, 4194, 6554, 4194,
    10082, 6554,10082, 6554,
     6554, 4194, 6554, 4194
  },
  {  9362, 5825, 9362, 5825,
     5825, 3647, 5825, 3647,
     9362, 5825, 9362, 5825,
     5825, 3647, 5825, 3647
  },
  {  8192, 5243, 8192, 5243,
     5243, 3355, 5243, 3355,
     8192, 5243, 8192, 5243,
     5243, 3355, 5243, 3355
  },
  {  7282, 4559, 7282, 4559,
     4559, 2893, 4559, 2893,
     7282, 4559, 7282, 4559,
     4559, 2893, 4559, 2893
  }
};

Int g_aiDequantCoef[6][16] =
{
  { 10, 13, 10, 13,
    13, 16, 13, 16,
    10, 13, 10, 13,
    13, 16, 13, 16
  },
  { 11, 14, 11, 14,
    14, 18, 14, 18,
    11, 14, 11, 14,
    14, 18, 14, 18
  },
  { 13, 16, 13, 16,
    16, 20, 16, 20,
    13, 16, 13, 16,
    16, 20, 16, 20
  },
  { 14, 18, 14, 18,
    18, 23, 18, 23,
    14, 18, 14, 18,
    18, 23, 18, 23
  },
  { 16, 20, 16, 20,
    20, 25, 20, 25,
    16, 20, 16, 20,
    20, 25, 20, 25
  },
  {
    18, 23, 18, 23,
    23, 29, 23, 29,
    18, 23, 18, 23,
    23, 29, 23, 29
  }
};

UInt g_aiQuantCoef4096[6]
 = {26, 23, 20, 18, 16, 14}; // 9
// = {51, 47, 39, 37, 32, 28}; // 10
// = {102, 93, 79, 73, 64, 57}; // 11
UInt g_aiDeQuantCoef4096[6]
 = {10, 11, 13, 14, 16, 18}; // 9
// = {20, 22, 26, 28, 32, 36}; // 10
// = {40, 44, 52, 56, 64, 72}; // 11

UInt g_aiQuantCoef1024[6][1024] =
{
  {
    102, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 102, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 103, 103, 103, 104, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 104, 103, 103, 103, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 103, 103, 103, 104, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 104, 103, 103, 103, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 103, 103, 103, 104, 103, 103, 103, 103, 103,
    103, 104, 104, 104, 103, 104, 104, 103, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 103, 104, 104, 103, 104, 104, 104,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 104, 104, 104, 103, 104, 104, 103, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 103, 104, 104, 103, 104, 104, 104,
    103, 103, 103, 103, 103, 103, 104, 103, 103, 103, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 103, 103, 103, 104, 103, 103, 103, 103, 103,
    103, 104, 104, 104, 103, 104, 104, 103, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 103, 104, 104, 103, 104, 104, 104,
    102, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 102, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 104, 104, 104, 103, 104, 104, 103, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 103, 104, 104, 103, 104, 104, 104,
    103, 103, 103, 103, 103, 103, 104, 103, 103, 103, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 103, 103, 103, 104, 103, 103, 103, 103, 103,
    103, 104, 104, 104, 103, 104, 104, 103, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 103, 104, 104, 103, 104, 104, 104,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 104, 104, 104, 103, 104, 104, 103, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 103, 104, 104, 103, 104, 104, 104,
    103, 103, 103, 103, 103, 103, 104, 103, 103, 103, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 103, 103, 103, 104, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 104, 103, 103, 103, 103, 104, 103, 103, 103, 104, 104, 103, 104, 104, 104, 103, 104, 104, 104, 103, 104, 104, 103, 103, 103, 104, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103,
    103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 104, 103, 103, 103, 103, 103, 103, 103, 103, 103, 103
  },
  {
    93, 94, 94, 94, 93, 94, 94, 93, 93, 94, 94, 94, 93, 94, 94, 94, 93, 94, 94, 94, 93, 94, 94, 94, 93, 93, 94, 94, 93, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    93, 94, 94, 94, 93, 94, 94, 93, 93, 94, 94, 94, 93, 94, 94, 94, 93, 94, 94, 94, 93, 94, 94, 94, 93, 93, 94, 94, 93, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    93, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 93, 94, 94, 94,
    93, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 93, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    93, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 93, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    93, 94, 94, 94, 93, 94, 94, 93, 93, 94, 94, 94, 93, 94, 94, 94, 93, 94, 94, 94, 93, 94, 94, 94, 93, 93, 94, 94, 93, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    93, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 93, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    93, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    93, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 93, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 93, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    93, 94, 94, 94, 93, 94, 94, 93, 93, 94, 94, 94, 93, 94, 94, 94, 93, 94, 94, 94, 93, 94, 94, 94, 94, 93, 94, 94, 93, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94,
    94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94, 94
  },
  {
    79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 79, 80, 80, 79, 80, 80, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 79, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 79, 79, 79, 80, 80, 79, 80, 80, 80,
    79, 80, 80, 79, 79, 80, 80, 79, 79, 79, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 79, 79, 79, 80, 80, 79, 79, 80, 80,
    79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 79, 80, 80, 79, 80, 80, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 79, 80, 80, 79, 80, 80, 80,
    79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 80, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79,
    79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 80, 79, 80, 79, 80, 79, 80, 79, 80, 79, 80, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79,
    79, 80, 79, 79, 79, 80, 80, 79, 79, 79, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 79, 79, 79, 80, 79, 79, 79, 79, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 79, 80, 80, 79, 80, 80, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 80,
    79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 80, 79, 80, 79, 80, 79, 80, 79, 80, 79, 80, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79,
    79, 80, 80, 80, 79, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 79, 80, 80, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 79, 80, 80, 79, 80, 80, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 80,
    79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79,
    79, 80, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 79, 80, 80, 79, 80, 80, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 79, 80, 80, 80,
    79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 80, 79, 80, 79, 80, 79, 80, 79, 80, 79, 80, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79,
    79, 80, 80, 80, 79, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 79, 80, 80, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 79, 80, 80, 79, 80, 80, 80,
    79, 80, 79, 79, 79, 80, 80, 79, 79, 79, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 79, 79, 79, 80, 79, 79, 79, 79, 80,
    79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 80, 79, 80, 79, 80, 79, 80, 79, 80, 79, 80, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79,
    79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 80, 79, 79, 79, 80, 79, 80, 79, 79, 79, 80, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 79, 80, 80, 79, 80, 80, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 79, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 79, 79, 79, 80, 80, 79, 79, 80, 80,
    79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79,
    79, 80, 80, 79, 79, 80, 80, 79, 79, 79, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 79, 79, 79, 80, 79, 79, 79, 79, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 79, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 79, 79, 79, 80, 80, 79, 79, 80, 80,
    79, 80, 80, 80, 79, 80, 80, 79, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 80, 80, 80, 79, 79, 80, 80, 79, 80, 80, 80
  },
  {
    73, 74, 74, 73, 73, 74, 74, 73, 73, 73, 74, 74, 73, 74, 74, 74, 73, 74, 74, 74, 73, 74, 74, 73, 73, 73, 74, 73, 73, 73, 73, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    73, 74, 74, 74, 73, 74, 74, 73, 73, 74, 74, 74, 73, 74, 74, 74, 73, 74, 74, 74, 73, 74, 74, 74, 73, 73, 74, 74, 73, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    73, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74,
    73, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74,
    73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    73, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    73, 74, 73, 73, 73, 74, 74, 73, 73, 73, 74, 74, 73, 74, 74, 74, 73, 74, 74, 74, 73, 74, 74, 73, 73, 73, 74, 73, 73, 73, 73, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    73, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    73, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74,
    73, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    73, 74, 74, 74, 73, 74, 74, 73, 73, 74, 74, 74, 73, 74, 74, 74, 73, 74, 74, 74, 73, 74, 74, 74, 73, 73, 74, 74, 73, 74, 74, 74,
    73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74,
    74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74
  },
  {
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 64, 64, 64, 64, 64, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 64, 64, 64, 64, 64,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 64, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 64, 64, 64, 64, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 64, 64, 64, 64, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65
  },
  {
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 57, 57, 57, 58, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 58, 57, 57, 57, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 57, 57, 57, 58, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 58, 57, 57, 57, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 57, 57, 57, 58, 57, 57, 57, 57, 57,
    57, 58, 58, 58, 57, 58, 58, 57, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 57, 58, 58, 57, 58, 58, 58,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 58, 58, 58, 57, 58, 58, 57, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 57, 58, 58, 57, 58, 58, 58,
    57, 57, 57, 57, 57, 57, 58, 57, 57, 57, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 57, 57, 57, 58, 57, 57, 57, 57, 57,
    57, 58, 58, 58, 57, 58, 58, 57, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 57, 58, 58, 57, 58, 58, 58,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 58, 58, 58, 57, 58, 58, 57, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 57, 58, 58, 57, 58, 58, 58,
    57, 57, 57, 57, 57, 57, 58, 57, 57, 57, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 57, 57, 57, 58, 57, 57, 57, 57, 57,
    57, 58, 58, 58, 57, 58, 58, 57, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 57, 58, 58, 57, 58, 58, 58,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 58, 58, 58, 57, 58, 58, 57, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 57, 58, 58, 57, 58, 58, 58,
    57, 57, 57, 57, 57, 57, 58, 57, 57, 57, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 57, 57, 57, 58, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 58, 57, 57, 57, 57, 58, 57, 57, 57, 58, 58, 57, 58, 58, 58, 57, 58, 58, 58, 57, 58, 58, 57, 57, 57, 58, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57
  }
};
UInt g_aiDeQuantCoef1024[6][1024] =
{
  {
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40
  },
  {
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 45, 44, 44, 44, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 45, 44, 44, 44, 45, 45, 44, 44, 44, 45, 45, 44, 45, 45, 45, 44, 45, 45, 45, 44, 45, 45, 44, 44, 44, 45, 44, 44, 44, 44, 45,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 45, 44, 44, 44, 45, 45, 44, 44, 44, 45, 45, 44, 45, 45, 45, 44, 45, 45, 45, 44, 45, 45, 44, 44, 44, 45, 44, 44, 44, 44, 45,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 45, 45, 44, 44, 45, 45, 44, 44, 44, 45, 45, 44, 45, 45, 45, 44, 45, 45, 45, 44, 45, 45, 44, 44, 44, 45, 44, 44, 44, 44, 45,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 45, 45, 44, 44, 45, 45, 44, 44, 44, 45, 45, 44, 45, 45, 45, 44, 45, 45, 45, 44, 45, 45, 44, 44, 44, 45, 45, 44, 44, 45, 45,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 45, 44, 44, 44, 45, 45, 44, 44, 44, 45, 45, 44, 45, 45, 45, 44, 45, 45, 45, 44, 45, 45, 44, 44, 44, 45, 44, 44, 44, 44, 45,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 45, 45, 44, 44, 45, 45, 44, 44, 44, 45, 45, 44, 45, 45, 45, 44, 45, 45, 45, 44, 45, 45, 44, 44, 44, 45, 44, 44, 44, 44, 45,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 44, 44, 44, 45, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44
  },
  {
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
    52, 53, 53, 53, 52, 53, 53, 52, 52, 52, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 52, 52, 52, 53, 53, 52, 52, 53, 53,
    52, 53, 52, 52, 52, 53, 53, 52, 52, 52, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 52, 52, 52, 53, 52, 52, 52, 52, 53,
    52, 53, 52, 52, 52, 52, 53, 52, 52, 52, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 52, 52, 52, 53, 52, 52, 52, 52, 52,
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
    52, 53, 53, 52, 52, 53, 53, 52, 52, 52, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 52, 52, 52, 53, 53, 52, 52, 53, 53,
    52, 53, 53, 53, 52, 53, 53, 52, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 52, 53, 53, 52, 53, 53, 53,
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 53, 52, 53, 52, 53, 52, 53, 52, 53, 52, 53, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
    52, 52, 52, 52, 52, 52, 53, 52, 52, 52, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 52, 52, 52, 53, 52, 52, 52, 52, 52,
    52, 53, 53, 53, 52, 53, 53, 52, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 52, 53, 53, 52, 53, 53, 53,
    52, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53,
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 53, 52, 53, 52, 53, 52, 53, 52, 53, 52, 53, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
    52, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53,
    52, 53, 53, 53, 52, 53, 53, 52, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 52, 53, 53, 52, 53, 53, 53,
    52, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53,
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
    52, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53,
    52, 53, 53, 53, 52, 53, 53, 52, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 52, 53, 53, 52, 53, 53, 53,
    52, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53,
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 53, 52, 53, 52, 53, 52, 53, 52, 53, 52, 53, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
    52, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 53, 53, 53, 53, 53, 53, 52, 53, 53, 52, 53, 53, 53,
    52, 53, 53, 53, 52, 53, 53, 52, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 52, 53, 53, 52, 53, 53, 53,
    52, 52, 52, 52, 52, 52, 53, 52, 52, 52, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 52, 52, 52, 53, 52, 52, 52, 52, 52,
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 53, 52, 53, 52, 53, 52, 53, 52, 53, 52, 53, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
    52, 53, 53, 53, 52, 53, 53, 52, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 52, 53, 53, 52, 53, 53, 53,
    52, 53, 52, 52, 52, 53, 53, 52, 52, 52, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 52, 52, 52, 53, 52, 52, 52, 52, 53,
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
    52, 52, 52, 52, 52, 52, 53, 52, 52, 52, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 52, 52, 52, 53, 52, 52, 52, 52, 52,
    52, 53, 52, 52, 52, 53, 53, 52, 52, 52, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 52, 52, 52, 53, 52, 52, 52, 52, 53,
    52, 53, 53, 52, 52, 53, 53, 52, 52, 52, 53, 53, 52, 53, 53, 53, 52, 53, 53, 53, 52, 53, 53, 52, 52, 52, 53, 53, 52, 52, 53, 53
  },
  {
    56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 56, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 56, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 56, 57, 57, 56, 57, 57, 57,
    56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 56, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 56, 57, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 56, 57, 57, 57,
    56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56,
    56, 56, 56, 56, 56, 56, 57, 56, 56, 56, 56, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 56, 56, 56, 57, 56, 56, 56, 56, 56,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 56, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 56, 56, 56, 57, 57, 56, 56, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57,
    56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 56, 56, 56, 57, 56, 56, 56, 56, 56,
    56, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 56, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57,
    56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56,
    56, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 56, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57,
    56, 56, 56, 56, 56, 56, 57, 56, 56, 56, 57, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 56, 56, 56, 57, 56, 56, 56, 56, 56,
    56, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 56, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 56, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 56, 56, 56, 57, 57, 56, 57, 57, 57,
    56, 56, 56, 56, 56, 56, 57, 56, 56, 56, 57, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 56, 56, 56, 57, 56, 56, 56, 56, 56,
    56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 57, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56,
    56, 57, 57, 57, 56, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 56, 57, 57, 56, 57, 57, 57,
    56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 56, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 56, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 56, 57, 57, 56, 57, 57, 57,
    56, 57, 57, 57, 56, 57, 57, 56, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 57, 57, 57, 56, 56, 57, 57, 56, 57, 57, 57
  },
  {
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 64, 64, 64, 64, 64, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 64, 64, 64, 64, 64,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 64, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 64, 64, 64, 64, 65, 64, 64, 64, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 64, 64, 64, 65, 64, 64, 64, 64, 65,
    64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 64, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65,
    64, 65, 65, 65, 64, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65
  },
  {
    72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 73, 72, 73, 72, 73, 72, 73, 72, 73, 72, 73, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 72, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 72, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 72, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 72, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 72, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 72, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73,
    72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72,
    72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 72, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 72, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 72, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 72, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 72, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 73, 73, 73, 72, 72, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 73, 72, 73, 72, 73, 72, 73, 72, 73, 72, 73, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73,
    72, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 73, 73
  }
};

UInt g_aiQuantCoef256[6][256] =
{
  {
    207, 209, 209, 208, 207, 209, 208, 209, 207, 209, 208, 209, 207, 208, 209, 209,
    209, 212, 212, 211, 209, 211, 210, 212, 209, 212, 210, 211, 209, 211, 212, 212,
    209, 212, 212, 211, 209, 211, 210, 212, 209, 212, 210, 211, 209, 211, 212, 212,
    208, 211, 211, 210, 208, 210, 209, 211, 208, 211, 209, 210, 208, 210, 211, 211,
    207, 209, 209, 208, 207, 208, 208, 209, 207, 209, 208, 208, 207, 208, 209, 209,
    209, 211, 211, 210, 208, 210, 210, 211, 209, 211, 210, 210, 208, 210, 211, 211,
    208, 210, 210, 209, 208, 210, 209, 210, 208, 211, 209, 210, 208, 209, 210, 210,
    209, 212, 212, 211, 209, 211, 210, 212, 209, 212, 210, 211, 209, 211, 212, 212,
    207, 209, 209, 208, 207, 209, 208, 209, 207, 210, 208, 209, 207, 208, 209, 209,
    209, 212, 212, 211, 209, 211, 211, 212, 210, 212, 210, 211, 209, 211, 212, 212,
    208, 210, 210, 209, 208, 210, 209, 210, 208, 210, 209, 210, 208, 209, 210, 210,
    209, 211, 211, 210, 208, 210, 210, 211, 209, 211, 210, 210, 208, 210, 211, 211,
    207, 209, 209, 208, 207, 208, 208, 209, 207, 209, 208, 208, 207, 208, 209, 209,
    208, 211, 211, 210, 208, 210, 209, 211, 208, 211, 209, 210, 208, 210, 211, 211,
    209, 212, 212, 211, 209, 211, 210, 212, 209, 212, 210, 211, 209, 211, 212, 212,
    209, 212, 212, 211, 209, 211, 210, 212, 209, 212, 210, 211, 209, 211, 212, 212
  },
  {
    188, 190, 190, 189, 188, 190, 189, 190, 188, 190, 189, 190, 188, 189, 190, 190,
    190, 192, 192, 191, 190, 192, 191, 192, 190, 193, 191, 192, 190, 191, 192, 192,
    190, 192, 192, 191, 190, 192, 191, 192, 190, 192, 191, 192, 190, 191, 192, 192,
    189, 191, 191, 190, 189, 191, 190, 191, 189, 192, 190, 191, 189, 190, 191, 191,
    188, 190, 190, 189, 188, 189, 189, 190, 188, 190, 189, 189, 188, 189, 190, 190,
    190, 192, 192, 191, 189, 191, 191, 192, 190, 192, 191, 191, 189, 191, 192, 192,
    189, 191, 191, 190, 189, 191, 190, 191, 189, 191, 190, 191, 189, 190, 191, 191,
    190, 192, 192, 191, 190, 192, 191, 193, 190, 193, 191, 192, 190, 191, 192, 192,
    188, 190, 190, 189, 188, 190, 189, 190, 188, 190, 189, 190, 188, 189, 190, 190,
    190, 193, 192, 192, 190, 192, 191, 193, 190, 193, 191, 192, 190, 192, 192, 193,
    189, 191, 191, 190, 189, 191, 190, 191, 189, 191, 190, 191, 189, 190, 191, 191,
    190, 192, 192, 191, 189, 191, 191, 192, 190, 192, 191, 191, 189, 191, 192, 192,
    188, 190, 190, 189, 188, 189, 189, 190, 188, 190, 189, 189, 188, 189, 190, 190,
    189, 191, 191, 190, 189, 191, 190, 191, 189, 192, 190, 191, 189, 190, 191, 191,
    190, 192, 192, 191, 190, 192, 191, 192, 190, 192, 191, 192, 190, 191, 192, 192,
    190, 192, 192, 191, 190, 192, 191, 192, 190, 193, 191, 192, 190, 191, 192, 192
  },
  {
    159, 161, 161, 160, 159, 160, 160, 161, 159, 161, 160, 160, 159, 160, 161, 161,
    161, 163, 163, 162, 161, 162, 162, 163, 161, 163, 162, 162, 161, 162, 163, 163,
    161, 163, 163, 162, 161, 162, 162, 163, 161, 163, 162, 162, 161, 162, 163, 163,
    160, 162, 162, 161, 160, 161, 161, 162, 160, 162, 161, 161, 160, 161, 162, 162,
    159, 161, 161, 160, 159, 160, 160, 161, 159, 161, 160, 160, 159, 160, 161, 161,
    160, 162, 162, 161, 160, 162, 161, 162, 160, 162, 161, 162, 160, 161, 162, 162,
    160, 162, 162, 161, 160, 161, 161, 162, 160, 162, 161, 161, 160, 161, 162, 162,
    161, 163, 163, 162, 161, 162, 162, 163, 161, 163, 162, 162, 161, 162, 163, 163,
    159, 161, 161, 160, 159, 160, 160, 161, 159, 161, 160, 160, 159, 160, 161, 161,
    161, 163, 163, 162, 161, 162, 162, 163, 161, 163, 162, 162, 161, 162, 163, 163,
    160, 162, 162, 161, 160, 161, 161, 162, 160, 162, 161, 161, 160, 161, 162, 162,
    160, 162, 162, 161, 160, 162, 161, 162, 160, 162, 161, 162, 160, 161, 162, 162,
    159, 161, 161, 160, 159, 160, 160, 161, 159, 161, 160, 160, 159, 160, 161, 161,
    160, 162, 162, 161, 160, 161, 161, 162, 160, 162, 161, 161, 160, 161, 162, 162,
    161, 163, 163, 162, 161, 162, 162, 163, 161, 163, 162, 162, 161, 162, 163, 163,
    161, 163, 163, 162, 161, 162, 162, 163, 161, 163, 162, 162, 161, 162, 163, 163
  },
  {
    148, 150, 150, 149, 148, 149, 149, 150, 148, 150, 149, 149, 148, 149, 150, 150,
    150, 151, 151, 150, 149, 151, 150, 151, 150, 151, 150, 151, 149, 150, 151, 151,
    150, 151, 151, 150, 149, 151, 150, 151, 150, 151, 150, 151, 149, 150, 151, 151,
    149, 150, 150, 150, 149, 150, 150, 150, 149, 150, 150, 150, 149, 150, 150, 150,
    148, 149, 149, 149, 148, 149, 149, 149, 148, 149, 149, 149, 148, 149, 149, 149,
    149, 151, 151, 150, 149, 150, 150, 151, 149, 151, 150, 150, 149, 150, 151, 151,
    149, 150, 150, 150, 149, 150, 149, 150, 149, 150, 149, 150, 149, 150, 150, 150,
    150, 151, 151, 150, 149, 151, 150, 151, 150, 151, 150, 151, 149, 150, 151, 151,
    148, 150, 150, 149, 148, 149, 149, 150, 148, 150, 149, 149, 148, 149, 150, 150,
    150, 151, 151, 150, 149, 151, 150, 151, 150, 151, 150, 151, 149, 150, 151, 151,
    149, 150, 150, 150, 149, 150, 149, 150, 149, 150, 149, 150, 149, 150, 150, 150,
    149, 151, 151, 150, 149, 150, 150, 151, 149, 151, 150, 150, 149, 150, 151, 151,
    148, 149, 149, 149, 148, 149, 149, 149, 148, 149, 149, 149, 148, 149, 149, 149,
    149, 150, 150, 150, 149, 150, 150, 150, 149, 150, 150, 150, 149, 150, 150, 150,
    150, 151, 151, 150, 149, 151, 150, 151, 150, 151, 150, 151, 149, 150, 151, 151,
    150, 151, 151, 150, 149, 151, 150, 151, 150, 151, 150, 151, 149, 150, 151, 151
  },
  {
    129, 131, 131, 130, 129, 130, 130, 131, 129, 131, 130, 130, 129, 130, 131, 131,
    131, 132, 132, 132, 131, 132, 131, 132, 131, 132, 131, 132, 131, 132, 132, 132,
    131, 132, 132, 132, 131, 132, 131, 132, 131, 132, 131, 132, 131, 132, 132, 132,
    130, 132, 132, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 132, 132,
    129, 131, 131, 130, 129, 130, 130, 131, 129, 131, 130, 130, 129, 130, 131, 131,
    130, 132, 132, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 132, 132,
    130, 131, 131, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 131, 131,
    131, 132, 132, 132, 131, 132, 132, 132, 131, 132, 132, 132, 131, 132, 132, 132,
    129, 131, 131, 130, 129, 130, 130, 131, 129, 131, 130, 130, 129, 130, 131, 131,
    131, 132, 132, 132, 131, 132, 132, 132, 131, 132, 132, 132, 131, 132, 132, 132,
    130, 131, 131, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 131, 131,
    130, 132, 132, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 132, 132,
    129, 131, 131, 130, 129, 130, 130, 131, 129, 131, 130, 130, 129, 130, 131, 131,
    130, 132, 132, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 132, 132,
    131, 132, 132, 132, 131, 132, 131, 132, 131, 132, 131, 132, 131, 132, 132, 132,
    131, 132, 132, 132, 131, 132, 131, 132, 131, 132, 131, 132, 131, 132, 132, 132
  },
  {
    115, 116, 116, 116, 115, 116, 116, 116, 115, 116, 116, 116, 115, 116, 116, 116,
    116, 118, 118, 117, 116, 117, 117, 118, 116, 118, 117, 117, 116, 117, 118, 118,
    116, 118, 118, 117, 116, 117, 117, 118, 116, 118, 117, 117, 116, 117, 118, 118,
    116, 117, 117, 116, 116, 117, 116, 117, 116, 117, 116, 117, 116, 116, 117, 117,
    115, 116, 116, 116, 115, 116, 116, 116, 115, 116, 116, 116, 115, 116, 116, 116,
    116, 117, 117, 117, 116, 117, 116, 117, 116, 117, 116, 117, 116, 117, 117, 117,
    116, 117, 117, 116, 116, 116, 116, 117, 116, 117, 116, 116, 116, 116, 117, 117,
    116, 118, 118, 117, 116, 117, 117, 118, 116, 118, 117, 117, 116, 117, 118, 118,
    115, 116, 116, 116, 115, 116, 116, 116, 115, 116, 116, 116, 115, 116, 116, 116,
    116, 118, 118, 117, 116, 117, 117, 118, 116, 118, 117, 117, 116, 117, 118, 118,
    116, 117, 117, 116, 116, 116, 116, 117, 116, 117, 116, 116, 116, 116, 117, 117,
    116, 117, 117, 117, 116, 117, 116, 117, 116, 117, 116, 117, 116, 117, 117, 117,
    115, 116, 116, 116, 115, 116, 116, 116, 115, 116, 116, 116, 115, 116, 116, 116,
    116, 117, 117, 116, 116, 117, 116, 117, 116, 117, 116, 117, 116, 116, 117, 117,
    116, 118, 118, 117, 116, 117, 117, 118, 116, 118, 117, 117, 116, 117, 118, 118,
    116, 118, 118, 117, 116, 117, 117, 118, 116, 118, 117, 117, 116, 117, 118, 118
  }

};
UInt g_aiDeQuantCoef256[6][256] =
{
  {
    81, 82, 82, 81, 81, 81, 81, 82, 81, 82, 81, 81, 81, 81, 82, 82,
    82, 83, 83, 82, 82, 82, 82, 83, 82, 83, 82, 82, 82, 82, 83, 83,
    82, 83, 83, 82, 82, 82, 82, 83, 82, 83, 82, 82, 82, 82, 83, 83,
    81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82,
    81, 82, 82, 81, 81, 81, 81, 82, 81, 82, 81, 81, 81, 81, 82, 82,
    81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82,
    81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82,
    82, 83, 83, 82, 82, 82, 82, 83, 82, 83, 82, 82, 82, 82, 83, 83,
    81, 82, 82, 81, 81, 81, 81, 82, 81, 82, 81, 81, 81, 81, 82, 82,
    82, 83, 83, 82, 82, 82, 82, 83, 82, 83, 82, 82, 82, 82, 83, 83,
    81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82,
    81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82,
    81, 82, 82, 81, 81, 81, 81, 82, 81, 82, 81, 81, 81, 81, 82, 82,
    81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82, 81, 82, 82, 82,
    82, 83, 83, 82, 82, 82, 82, 83, 82, 83, 82, 82, 82, 82, 83, 83,
    82, 83, 83, 82, 82, 82, 82, 83, 82, 83, 82, 82, 82, 82, 83, 83
  },
  {
    89, 90, 90, 90, 89, 90, 89, 90, 89, 90, 89, 90, 89, 90, 90, 90,
    90, 91, 91, 90, 90, 91, 90, 91, 90, 91, 90, 91, 90, 90, 91, 91,
    90, 91, 91, 90, 90, 91, 90, 91, 90, 91, 90, 91, 90, 90, 91, 91,
    90, 90, 90, 90, 89, 90, 90, 91, 90, 91, 90, 90, 89, 90, 90, 90,
    89, 90, 90, 89, 89, 90, 89, 90, 89, 90, 89, 90, 89, 89, 90, 90,
    90, 91, 91, 90, 90, 90, 90, 91, 90, 91, 90, 90, 90, 90, 91, 91,
    89, 90, 90, 90, 89, 90, 90, 90, 89, 90, 90, 90, 89, 90, 90, 90,
    90, 91, 91, 91, 90, 91, 90, 91, 90, 91, 90, 91, 90, 91, 91, 91,
    89, 90, 90, 90, 89, 90, 89, 90, 89, 90, 89, 90, 89, 90, 90, 90,
    90, 91, 91, 91, 90, 91, 90, 91, 90, 91, 90, 91, 90, 91, 91, 91,
    89, 90, 90, 90, 89, 90, 90, 90, 89, 90, 90, 90, 89, 90, 90, 90,
    90, 91, 91, 90, 90, 90, 90, 91, 90, 91, 90, 90, 90, 90, 91, 91,
    89, 90, 90, 89, 89, 90, 89, 90, 89, 90, 89, 90, 89, 89, 90, 90,
    90, 90, 90, 90, 89, 90, 90, 91, 90, 91, 90, 90, 89, 90, 90, 90,
    90, 91, 91, 90, 90, 91, 90, 91, 90, 91, 90, 91, 90, 90, 91, 91,
    90, 91, 91, 90, 90, 91, 90, 91, 90, 91, 90, 91, 90, 90, 91, 91
  },
  {
    105, 106, 106, 106, 105, 106, 106, 106, 105, 106, 106, 106, 105, 106, 106, 106,
    106, 107, 107, 107, 106, 107, 107, 108, 106, 108, 107, 107, 106, 107, 107, 107,
    106, 107, 107, 107, 106, 107, 107, 107, 106, 108, 107, 107, 106, 107, 107, 107,
    106, 107, 107, 106, 106, 107, 106, 107, 106, 107, 106, 107, 106, 106, 107, 107,
    105, 106, 106, 106, 105, 106, 106, 106, 105, 106, 106, 106, 105, 106, 106, 106,
    106, 107, 107, 107, 106, 107, 106, 107, 106, 107, 106, 107, 106, 107, 107, 107,
    106, 107, 107, 106, 106, 106, 106, 107, 106, 107, 106, 106, 106, 106, 107, 107,
    106, 108, 107, 107, 106, 107, 107, 108, 106, 108, 107, 107, 106, 107, 107, 108,
    105, 106, 106, 106, 105, 106, 106, 106, 105, 106, 106, 106, 105, 106, 106, 106,
    106, 108, 108, 107, 106, 107, 107, 108, 106, 108, 107, 107, 106, 107, 108, 108,
    106, 107, 107, 106, 106, 106, 106, 107, 106, 107, 106, 106, 106, 106, 107, 107,
    106, 107, 107, 107, 106, 107, 106, 107, 106, 107, 106, 107, 106, 107, 107, 107,
    105, 106, 106, 106, 105, 106, 106, 106, 105, 106, 106, 106, 105, 106, 106, 106,
    106, 107, 107, 106, 106, 107, 106, 107, 106, 107, 106, 107, 106, 106, 107, 107,
    106, 107, 107, 107, 106, 107, 107, 107, 106, 108, 107, 107, 106, 107, 107, 107,
    106, 107, 107, 107, 106, 107, 107, 108, 106, 108, 107, 107, 106, 107, 107, 107
  },
  {
    113, 115, 114, 114, 113, 114, 114, 115, 113, 115, 114, 114, 113, 114, 114, 114,
    115, 116, 116, 115, 114, 115, 115, 116, 115, 116, 115, 115, 114, 115, 116, 116,
    114, 116, 116, 115, 114, 115, 115, 116, 114, 116, 115, 115, 114, 115, 116, 116,
    114, 115, 115, 115, 114, 115, 114, 115, 114, 115, 114, 115, 114, 115, 115, 115,
    113, 114, 114, 114, 113, 114, 114, 114, 113, 114, 114, 114, 113, 114, 114, 114,
    114, 115, 115, 115, 114, 115, 115, 115, 114, 115, 115, 115, 114, 115, 115, 115,
    114, 115, 115, 114, 114, 115, 114, 115, 114, 115, 114, 115, 114, 114, 115, 115,
    115, 116, 116, 115, 114, 115, 115, 116, 115, 116, 115, 115, 114, 115, 116, 116,
    113, 115, 114, 114, 113, 114, 114, 115, 113, 115, 114, 114, 113, 114, 114, 115,
    115, 116, 116, 115, 114, 115, 115, 116, 115, 116, 115, 115, 114, 115, 116, 116,
    114, 115, 115, 114, 114, 115, 114, 115, 114, 115, 114, 115, 114, 114, 115, 115,
    114, 115, 115, 115, 114, 115, 115, 115, 114, 115, 115, 115, 114, 115, 115, 115,
    113, 114, 114, 114, 113, 114, 114, 114, 113, 114, 114, 114, 113, 114, 114, 114,
    114, 115, 115, 115, 114, 115, 114, 115, 114, 115, 114, 115, 114, 115, 115, 115,
    114, 116, 116, 115, 114, 115, 115, 116, 114, 116, 115, 115, 114, 115, 116, 116,
    114, 116, 116, 115, 114, 115, 115, 116, 115, 116, 115, 115, 114, 115, 116, 116
  },
  {
    129, 131, 131, 130, 129, 130, 130, 131, 129, 131, 130, 130, 129, 130, 131, 131,
    131, 132, 132, 132, 131, 132, 131, 132, 131, 132, 131, 132, 131, 132, 132, 132,
    131, 132, 132, 132, 131, 132, 131, 132, 131, 132, 131, 132, 131, 132, 132, 132,
    130, 132, 132, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 132, 132,
    129, 131, 131, 130, 129, 130, 130, 131, 129, 131, 130, 130, 129, 130, 131, 131,
    130, 132, 132, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 132, 132,
    130, 131, 131, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 131, 131,
    131, 132, 132, 132, 131, 132, 132, 132, 131, 132, 132, 132, 131, 132, 132, 132,
    129, 131, 131, 130, 129, 130, 130, 131, 129, 131, 130, 130, 129, 130, 131, 131,
    131, 132, 132, 132, 131, 132, 132, 132, 131, 132, 132, 132, 131, 132, 132, 132,
    130, 131, 131, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 131, 131,
    130, 132, 132, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 132, 132,
    129, 131, 131, 130, 129, 130, 130, 131, 129, 131, 130, 130, 129, 130, 131, 131,
    130, 132, 132, 131, 130, 131, 131, 132, 130, 132, 131, 131, 130, 131, 132, 132,
    131, 132, 132, 132, 131, 132, 131, 132, 131, 132, 131, 132, 131, 132, 132, 132,
    131, 132, 132, 132, 131, 132, 131, 132, 131, 132, 131, 132, 131, 132, 132, 132
  },
  {
    146, 147, 147, 146, 146, 147, 146, 147, 146, 147, 146, 147, 145, 146, 147, 147,
    147, 149, 149, 148, 147, 148, 148, 149, 147, 149, 148, 148, 147, 148, 149, 149,
    147, 149, 149, 148, 147, 148, 148, 149, 147, 149, 148, 148, 147, 148, 149, 149,
    146, 148, 148, 147, 146, 147, 147, 148, 146, 148, 147, 147, 146, 147, 148, 148,
    146, 147, 147, 146, 145, 147, 146, 147, 146, 147, 146, 147, 145, 146, 147, 147,
    147, 148, 148, 147, 147, 148, 147, 148, 147, 148, 147, 148, 147, 147, 148, 148,
    146, 148, 148, 147, 146, 147, 147, 148, 146, 148, 147, 147, 146, 147, 148, 148,
    147, 149, 149, 148, 147, 148, 148, 149, 147, 149, 148, 148, 147, 148, 149, 149,
    146, 147, 147, 146, 146, 147, 146, 147, 146, 147, 146, 147, 146, 146, 147, 147,
    147, 149, 149, 148, 147, 148, 148, 149, 147, 149, 148, 148, 147, 148, 149, 149,
    146, 148, 148, 147, 146, 147, 147, 148, 146, 148, 147, 147, 146, 147, 148, 148,
    147, 148, 148, 147, 147, 148, 147, 148, 147, 148, 147, 148, 147, 147, 148, 148,
    145, 147, 147, 146, 145, 147, 146, 147, 146, 147, 146, 147, 145, 146, 147, 147,
    146, 148, 148, 147, 146, 147, 147, 148, 146, 148, 147, 147, 146, 147, 148, 148,
    147, 149, 149, 148, 147, 148, 148, 149, 147, 149, 148, 148, 147, 148, 149, 149,
    147, 149, 149, 148, 147, 148, 148, 149, 147, 149, 148, 148, 147, 148, 149, 149
  }
};

const UChar g_aucChromaScale[52]=
{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,
   12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,
   28,29,29,30,31,32,32,33,34,34,35,35,36,36,37,37,
   37,38,38,38,39,39,39,39
};

Int g_aiDequantCoef64[6][64] =
{
  {
    20,  19, 25, 19, 20, 19, 25, 19,
    19,  18, 24, 18, 19, 18, 24, 18,
    25,  24, 32, 24, 25, 24, 32, 24,
    19,  18, 24, 18, 19, 18, 24, 18,
    20,  19, 25, 19, 20, 19, 25, 19,
    19,  18, 24, 18, 19, 18, 24, 18,
    25,  24, 32, 24, 25, 24, 32, 24,
    19,  18, 24, 18, 19, 18, 24, 18
  },
  {
    22,  21, 28, 21, 22, 21, 28, 21,
    21,  19, 26, 19, 21, 19, 26, 19,
    28,  26, 35, 26, 28, 26, 35, 26,
    21,  19, 26, 19, 21, 19, 26, 19,
    22,  21, 28, 21, 22, 21, 28, 21,
    21,  19, 26, 19, 21, 19, 26, 19,
    28,  26, 35, 26, 28, 26, 35, 26,
    21,  19, 26, 19, 21, 19, 26, 19
  },
  {
    26,  24, 33, 24, 26, 24, 33, 24,
    24,  23, 31, 23, 24, 23, 31, 23,
    33,  31, 42, 31, 33, 31, 42, 31,
    24,  23, 31, 23, 24, 23, 31, 23,
    26,  24, 33, 24, 26, 24, 33, 24,
    24,  23, 31, 23, 24, 23, 31, 23,
    33,  31, 42, 31, 33, 31, 42, 31,
    24,  23, 31, 23, 24, 23, 31, 23
  },
  {
    28,  26, 35, 26, 28, 26, 35, 26,
    26,  25, 33, 25, 26, 25, 33, 25,
    35,  33, 45, 33, 35, 33, 45, 33,
    26,  25, 33, 25, 26, 25, 33, 25,
    28,  26, 35, 26, 28, 26, 35, 26,
    26,  25, 33, 25, 26, 25, 33, 25,
    35,  33, 45, 33, 35, 33, 45, 33,
    26,  25, 33, 25, 26, 25, 33, 25
  },
  {
    32,  30, 40, 30, 32, 30, 40, 30,
    30,  28, 38, 28, 30, 28, 38, 28,
    40,  38, 51, 38, 40, 38, 51, 38,
    30,  28, 38, 28, 30, 28, 38, 28,
    32,  30, 40, 30, 32, 30, 40, 30,
    30,  28, 38, 28, 30, 28, 38, 28,
    40,  38, 51, 38, 40, 38, 51, 38,
    30,  28, 38, 28, 30, 28, 38, 28
  },
  {
    36,  34, 46, 34, 36, 34, 46, 34,
    34,  32, 43, 32, 34, 32, 43, 32,
    46,  43, 58, 43, 46, 43, 58, 43,
    34,  32, 43, 32, 34, 32, 43, 32,
    36,  34, 46, 34, 36, 34, 46, 34,
    34,  32, 43, 32, 34, 32, 43, 32,
    46,  43, 58, 43, 46, 43, 58, 43,
    34,  32, 43, 32, 34, 32, 43, 32
  }

};

UInt g_aiQuantCoef64[6][64] =
{
  {
    0x3333, 0x2fbe, 0x4189, 0x2fbe, 0x3333, 0x2fbe, 0x4189, 0x2fbe,
    0x2fbe, 0x2ca4, 0x3c79, 0x2ca4, 0x2fbe, 0x2ca4, 0x3c79, 0x2ca4,
    0x4189, 0x3c79, 0x51ec, 0x3c79, 0x4189, 0x3c79, 0x51ec, 0x3c79,
    0x2fbe, 0x2ca4, 0x3c79, 0x2ca4, 0x2fbe, 0x2ca4, 0x3c79, 0x2ca4,
    0x3333, 0x2fbe, 0x4189, 0x2fbe, 0x3333, 0x2fbe, 0x4189, 0x2fbe,
    0x2fbe, 0x2ca4, 0x3c79, 0x2ca4, 0x2fbe, 0x2ca4, 0x3c79, 0x2ca4,
    0x4189, 0x3c79, 0x51ec, 0x3c79, 0x4189, 0x3c79, 0x51ec, 0x3c79,
    0x2fbe, 0x2ca4, 0x3c79, 0x2ca4, 0x2fbe, 0x2ca4, 0x3c79, 0x2ca4,
  },
  {
    0x2e8c, 0x2b32, 0x3a84, 0x2b32, 0x2e8c, 0x2b32, 0x3a84, 0x2b32,
    0x2b32, 0x2a4a, 0x37d2, 0x2a4a, 0x2b32, 0x2a4a, 0x37d2, 0x2a4a,
    0x3a84, 0x37d2, 0x4ae6, 0x37d2, 0x3a84, 0x37d2, 0x4ae6, 0x37d2,
    0x2b32, 0x2a4a, 0x37d2, 0x2a4a, 0x2b32, 0x2a4a, 0x37d2, 0x2a4a,
    0x2e8c, 0x2b32, 0x3a84, 0x2b32, 0x2e8c, 0x2b32, 0x3a84, 0x2b32,
    0x2b32, 0x2a4a, 0x37d2, 0x2a4a, 0x2b32, 0x2a4a, 0x37d2, 0x2a4a,
    0x3a84, 0x37d2, 0x4ae6, 0x37d2, 0x3a84, 0x37d2, 0x4ae6, 0x37d2,
    0x2b32, 0x2a4a, 0x37d2, 0x2a4a, 0x2b32, 0x2a4a, 0x37d2, 0x2a4a,
  },
  {
    0x2762, 0x25cb, 0x31a6, 0x25cb, 0x2762, 0x25cb, 0x31a6, 0x25cb,
    0x25cb, 0x22ef, 0x2ed1, 0x22ef, 0x25cb, 0x22ef, 0x2ed1, 0x22ef,
    0x31a6, 0x2ed1, 0x3e6a, 0x2ed1, 0x31a6, 0x2ed1, 0x3e6a, 0x2ed1,
    0x25cb, 0x22ef, 0x2ed1, 0x22ef, 0x25cb, 0x22ef, 0x2ed1, 0x22ef,
    0x2762, 0x25cb, 0x31a6, 0x25cb, 0x2762, 0x25cb, 0x31a6, 0x25cb,
    0x25cb, 0x22ef, 0x2ed1, 0x22ef, 0x25cb, 0x22ef, 0x2ed1, 0x22ef,
    0x31a6, 0x2ed1, 0x3e6a, 0x2ed1, 0x31a6, 0x2ed1, 0x3e6a, 0x2ed1,
    0x25cb, 0x22ef, 0x2ed1, 0x22ef, 0x25cb, 0x22ef, 0x2ed1, 0x22ef,
  },
  {
    0x2492, 0x22e3, 0x2ed0, 0x22e3, 0x2492, 0x22e3, 0x2ed0, 0x22e3,
    0x22e3, 0x2024, 0x2bfb, 0x2024, 0x22e3, 0x2024, 0x2bfb, 0x2024,
    0x2ed0, 0x2bfb, 0x3a41, 0x2bfb, 0x2ed0, 0x2bfb, 0x3a41, 0x2bfb,
    0x22e3, 0x2024, 0x2bfb, 0x2024, 0x22e3, 0x2024, 0x2bfb, 0x2024,
    0x2492, 0x22e3, 0x2ed0, 0x22e3, 0x2492, 0x22e3, 0x2ed0, 0x22e3,
    0x22e3, 0x2024, 0x2bfb, 0x2024, 0x22e3, 0x2024, 0x2bfb, 0x2024,
    0x2ed0, 0x2bfb, 0x3a41, 0x2bfb, 0x2ed0, 0x2bfb, 0x3a41, 0x2bfb,
    0x22e3, 0x2024, 0x2bfb, 0x2024, 0x22e3, 0x2024, 0x2bfb, 0x2024,
  },
  {
    0x2000, 0x1e3c, 0x28f6, 0x1e3c, 0x2000, 0x1e3c, 0x28f6, 0x1e3c,
    0x1e3c, 0x1cb2, 0x2631, 0x1cb2, 0x1e3c, 0x1cb2, 0x2631, 0x1cb2,
    0x28f6, 0x2631, 0x3367, 0x2631, 0x28f6, 0x2631, 0x3367, 0x2631,
    0x1e3c, 0x1cb2, 0x2631, 0x1cb2, 0x1e3c, 0x1cb2, 0x2631, 0x1cb2,
    0x2000, 0x1e3c, 0x28f6, 0x1e3c, 0x2000, 0x1e3c, 0x28f6, 0x1e3c,
    0x1e3c, 0x1cb2, 0x2631, 0x1cb2, 0x1e3c, 0x1cb2, 0x2631, 0x1cb2,
    0x28f6, 0x2631, 0x3367, 0x2631, 0x28f6, 0x2631, 0x3367, 0x2631,
    0x1e3c, 0x1cb2, 0x2631, 0x1cb2, 0x1e3c, 0x1cb2, 0x2631, 0x1cb2,
  },
  {
    0x1c72, 0x1aae, 0x239e, 0x1aae, 0x1c72, 0x1aae, 0x239e, 0x1aae,
    0x1aae, 0x191c, 0x21c0, 0x191c, 0x1aae, 0x191c, 0x21c0, 0x191c,
    0x239e, 0x21c0, 0x2d32, 0x21c0, 0x239e, 0x21c0, 0x2d32, 0x21c0,
    0x1aae, 0x191c, 0x21c0, 0x191c, 0x1aae, 0x191c, 0x21c0, 0x191c,
    0x1c72, 0x1aae, 0x239e, 0x1aae, 0x1c72, 0x1aae, 0x239e, 0x1aae,
    0x1aae, 0x191c, 0x21c0, 0x191c, 0x1aae, 0x191c, 0x21c0, 0x191c,
    0x239e, 0x21c0, 0x2d32, 0x21c0, 0x239e, 0x21c0, 0x2d32, 0x21c0,
    0x1aae, 0x191c, 0x21c0, 0x191c, 0x1aae, 0x191c, 0x21c0, 0x191c,
  }
};

// ====================================================================================================================
// TENTM VLC table
// ====================================================================================================================

#if LCEC_PHASE2
const UInt g_auiCBPTableE[2][8] = 
{
  {2,0,6,4,5,3,7,1},
  {0,1,6,3,4,2,7,5}
};
const UInt g_auiCBPTableD[2][8] = 
{
  {1,7,0,5,3,4,2,6},
  {0,1,5,3,4,7,2,6}
};


// Below table need to be optimized

const UInt g_auiCbpVlcNum[2][8] =
{
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};
#endif


#if LCEC_PHASE2
const UInt g_auiMI1TableE[8] = {0,2,1,4,3,6,5,7};
const UInt g_auiMI1TableD[8] = {0,2,1,4,3,6,5,7};
const UInt g_auiMI2TableE[15] = {0,1,3,2,6,5,4,7,9,8,13,12,11,14,10};
const UInt g_auiMI2TableD[15] = {0,1,3,2,6,5,4,7,9,8,14,12,11,10,13};

// Below table need to be optimized
const UInt g_auiMITableVlcNum[15] = 
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#endif



#if LCEC_PHASE2

const UInt g_auiLPTableE4[3][32] = {
{0,1,2,3,5,4,7,6,9,11,14,8,16,15,10,13,12,17,18,19,25,23,20,22,28,26,29,24,30,31,27,21},  //4x4I
{0,1,2,7,5,3,6,4,11,8,12,10,9,14,13,15,16,17,21,27,26,18,19,23,29,20,25,28,22,30,24,31},  //4x4P
{0,1,2,7,5,3,6,4,11,8,12,10,9,14,13,15,16,17,21,27,26,18,19,23,29,20,25,28,22,30,24,31}}; //4x4B

const UInt g_auiLPTableD4[3][32] = {
{0,1,2,3,5,4,7,6,11,8,14,9,16,15,10,13,12,17,18,19,22,31,23,21,27,20,25,30,24,26,28,29},  //4x4I
{0,1,2,5,7,4,6,3,9,12,11,8,10,14,13,15,16,17,21,22,25,18,28,23,30,26,20,19,27,24,29,31},  //4x4P
{0,1,2,5,7,4,6,3,9,12,11,8,10,14,13,15,16,17,21,22,25,18,28,23,30,26,20,19,27,24,29,31}}; //4x4B

const UInt g_auiLPTableE8[8][128] = {
{0,2,1,4,7,3,6,11,13,9,18,22,19,17,8,10,20,24,28,27,21,26,38,42,37,31,23,12,16,33,30,44,45,49,46,36,57,50,55,47,39,43,35,75,66,56,60,58,54,70,63,29,77,87,69,48,51,64,71,65,80,85,76,94,5,15,14,34,52,25,41,72,74,53,59,81,82,68,40,62,95,127,89,84,67,73,126,79,125,93,90,61,32,124,123,83,86,122,88,78,121,92,91,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96}, //U
{0,2,1,4,7,3,6,11,13,9,18,22,19,17,8,10,20,24,28,27,21,26,38,42,37,31,23,12,16,33,30,44,45,49,46,36,57,50,55,47,39,43,35,75,66,56,60,58,54,70,63,29,77,87,69,48,51,64,71,65,80,85,76,94,5,15,14,34,52,25,41,72,74,53,59,81,82,68,40,62,95,127,89,84,67,73,126,79,125,93,90,61,32,124,123,83,86,122,88,78,121,92,91,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96}, //V

{0,1,2,6,7,3,5,12,11,10,8,17,20,22,9,13,32,21,14,25,16,29,62,59,42,28,15,18,19,26,35,36,30,48,56,39,91,72,63,37,38,44,23,27,41,47,40,58,77,83,52,65,57,33,53,60,54,69,89,45,34,51,46,49,4,31,24,71,88,70,92,103,119,93,55,95,122,127,106,123,125,76,66,126,108,112,117,107,124,111,68,97,85,114,113,121,116,115,79,82,105,120,110,118,109,102,100,84,101,90,104,94,96,78,86,99,81,75,87,98,80,73,74,67,64,61,43,50}, //8x8I
{0,1,2,7,5,3,11,16,6,8,14,18,13,19,9,15,21,22,24,25,20,31,44,36,37,30,23,10,4,26,28,33,39,45,49,40,55,56,59,43,42,32,12,17,38,48,50,60,74,79,75,47,51,27,35,53,65,73,76,66,46,80,77,83,29,71,69,88,92,81,90,106,97,86,85,116,127,115,93,101,114,126,113,102,87,84,104,125,124,123,122,82,34,103,121,120,107,112,94,41,68,105,111,119,110,95,61,57,96,109,118,108,52,54,100,117,99,62,78,98,91,58,64,89,67,72,63,70}, //8x8P
{0,1,2,7,5,3,11,16,6,8,14,18,13,19,9,15,21,22,24,25,20,31,44,36,37,30,23,10,4,26,28,33,39,45,49,40,55,56,59,43,42,32,12,17,38,48,50,60,74,79,75,47,51,27,35,53,65,73,76,66,46,80,77,83,29,71,69,88,92,81,90,106,97,86,85,116,127,115,93,101,114,126,113,102,87,84,104,125,124,123,122,82,34,103,121,120,107,112,94,41,68,105,111,119,110,95,61,57,96,109,118,108,52,54,100,117,99,62,78,98,91,58,64,89,67,72,63,70}, //8x8B

{0,1,2,5,4,3,6,7,8,9,10,11,13,12,15,16,20,17,14,18,23,29,26,19,24,38,22,34,32,46,21,39,41,30,28,45,27,40,56,59,35,42,51,48,44,43,47,54,58,36,53,63,61,37,55,65,50,62,60,52,57,49,31,25,33,64,67,69,72,70,80,86,95,99,89,123,88,96,94,113,101,91,98,93,83,108,110,107,103,105,79,77,90,85,127,81,114,104,118,84,97,102,122,87,106,109,92,117,126,78,116,71,112,120,82,121,75,66,111,124,76,100,125,68,73,115,119,74}, //16x16I
{0,2,1,4,5,3,6,7,8,11,14,13,12,10,9,15,17,16,20,23,22,26,31,30,27,21,25,19,24,32,28,29,36,35,40,37,47,42,44,38,33,34,41,43,45,39,48,49,54,59,60,57,52,51,56,58,61,55,65,62,63,64,67,69,18,50,46,72,66,53,70,73,71,74,89,81,79,77,68,83,78,84,80,87,90,96,127,93,95,86,94,75,82,99,107,126,92,100,88,76,85,106,125,124,98,97,105,104,123,122,121,103,91,102,120,119,118,117,101,116,115,114,113,112,111,110,109,108}, //16x16P
{0,2,1,4,5,3,6,7,8,11,14,13,12,10,9,15,17,16,20,23,22,26,31,30,27,21,25,19,24,32,28,29,36,35,40,37,47,42,44,38,33,34,41,43,45,39,48,49,54,59,60,57,52,51,56,58,61,55,65,62,63,64,67,69,18,50,46,72,66,53,70,73,71,74,89,81,79,77,68,83,78,84,80,87,90,96,127,93,95,86,94,75,82,99,107,126,92,100,88,76,85,106,125,124,98,97,105,104,123,122,121,103,91,102,120,119,118,117,101,116,115,114,113,112,111,110,109,108}};//16x16B 

const UInt g_auiLPTableD8[8][128] = {
{0,2,1,5,3,64,6,4,14,9,15,7,27,8,66,65,28,13,10,12,16,20,11,26,17,69,21,19,18,51,30,25,92,29,67,42,35,24,22,40,78,70,23,41,31,32,34,39,55,33,37,56,68,73,48,38,45,36,47,74,46,91,79,50,57,59,44,84,77,54,49,58,71,85,72,43,62,52,99,87,60,75,76,95,83,61,96,53,98,82,90,102,101,89,63,80,127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,100,97,94,93,88,86,81}, //U
{0,2,1,5,3,64,6,4,14,9,15,7,27,8,66,65,28,13,10,12,16,20,11,26,17,69,21,19,18,51,30,25,92,29,67,42,35,24,22,40,78,70,23,41,31,32,34,39,55,33,37,56,68,73,48,38,45,36,47,74,46,91,79,50,57,59,44,84,77,54,49,58,71,85,72,43,62,52,99,87,60,75,76,95,83,61,96,53,98,82,90,102,101,89,63,80,127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,100,97,94,93,88,86,81}, //V 

{0,1,2,5,64,6,3,4,10,14,9,8,7,15,18,26,20,11,27,28,12,17,13,42,66,19,29,43,25,21,32,65,16,53,60,30,31,39,40,35,46,44,24,126,41,59,62,45,33,63,127,61,50,54,56,74,34,52,47,23,55,125,22,38,124,51,82,123,90,57,69,67,37,121,122,117,81,48,113,98,120,116,99,49,107,92,114,118,68,58,109,36,70,73,111,75,112,91,119,115,106,108,105,71,110,100,78,87,84,104,102,89,85,94,93,97,96,86,103,72,101,95,76,79,88,80,83,77}, //8x8I
{0,1,2,5,28,4,8,3,9,14,27,6,42,12,10,15,7,43,11,13,20,16,17,26,18,19,29,53,30,64,25,21,41,31,92,54,23,24,44,32,35,99,40,39,22,33,60,51,45,34,46,52,112,55,113,36,37,107,121,38,47,106,117,126,122,56,59,124,100,66,127,65,125,57,48,50,58,62,118,49,61,69,91,63,85,74,73,84,67,123,70,120,68,78,98,105,108,72,119,116,114,79,83,93,86,101,71,96,111,109,104,102,97,82,80,77,75,115,110,103,95,94,90,89,88,87,81,76}, //8x8P
{0,1,2,5,28,4,8,3,9,14,27,6,42,12,10,15,7,43,11,13,20,16,17,26,18,19,29,53,30,64,25,21,41,31,92,54,23,24,44,32,35,99,40,39,22,33,60,51,45,34,46,52,112,55,113,36,37,107,121,38,47,106,117,126,122,56,59,124,100,66,127,65,125,57,48,50,58,62,118,49,61,69,91,63,85,74,73,84,67,123,70,120,68,78,98,105,108,72,119,116,114,79,83,93,86,101,71,96,111,109,104,102,97,82,80,77,75,115,110,103,95,94,90,89,88,87,81,76}, //8x8B

{0,1,2,5,4,3,6,7,8,9,10,11,13,12,18,14,15,17,19,23,16,30,26,20,24,63,22,36,34,21,33,62,28,64,27,40,49,53,25,31,37,32,41,45,44,35,29,46,43,61,56,42,59,50,47,54,38,60,48,39,58,52,57,51,65,55,117,66,123,67,69,111,68,124,127,116,120,91,109,90,70,95,114,84,99,93,71,103,76,74,92,81,106,83,78,72,77,100,82,73,121,80,101,88,97,89,104,87,85,105,86,118,112,79,96,125,110,107,98,126,113,115,102,75,119,122,108,94}, //16x16I
{0,2,1,5,3,4,6,7,8,14,13,9,12,11,10,15,17,16,64,27,18,25,20,19,28,26,21,24,30,31,23,22,29,40,41,33,32,35,39,45,34,42,37,43,38,44,66,36,46,47,65,53,52,69,48,57,54,51,55,49,50,56,59,60,61,58,68,62,78,63,70,72,67,71,73,91,99,77,80,76,82,75,92,79,81,100,89,83,98,74,84,112,96,87,90,88,85,105,104,93,97,118,113,111,107,106,101,94,127,126,125,124,123,122,121,120,119,117,116,115,114,110,109,108,103,102,95,86}, //16x16P
{0,2,1,5,3,4,6,7,8,14,13,9,12,11,10,15,17,16,64,27,18,25,20,19,28,26,21,24,30,31,23,22,29,40,41,33,32,35,39,45,34,42,37,43,38,44,66,36,46,47,65,53,52,69,48,57,54,51,55,49,50,56,59,60,61,58,68,62,78,63,70,72,67,71,73,91,99,77,80,76,82,75,92,79,81,100,89,83,98,74,84,112,96,87,90,88,85,105,104,93,97,118,113,111,107,106,101,94,127,126,125,124,123,122,121,120,119,117,116,115,114,110,109,108,103,102,95,86}};//16x16B  


#else

const UInt g_auiLPTableE8[10][128] = {{0,2,1,4,7,3,6,11,13,9,18,22,19,17,8,10,20,24,28,27,21,26,38,42,37,31,23,12,16,33,30,44,45,49,46,36,57,50,55,47,39,43,35,75,66,56,60,58,54,70,63,29,77,87,69,48,51,64,71,65,80,85,76,94,5,15,14,34,52,25,41,72,74,53,59,81,82,68,40,62,95,127,89,84,67,73,126,79,125,93,90,61,32,124,123,83,86,122,88,78,121,92,91,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96}, //U, UV
{0,2,1,4,7,3,6,11,13,9,18,22,19,17,8,10,20,24,28,27,21,26,38,42,37,31,23,12,16,33,30,44,45,49,46,36,57,50,55,47,39,43,35,75,66,56,60,58,54,70,63,29,77,87,69,48,51,64,71,65,80,85,76,94,5,15,14,34,52,25,41,72,74,53,59,81,82,68,40,62,95,127,89,84,67,73,126,79,125,93,90,61,32,124,123,83,86,122,88,78,121,92,91,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96}, //V
{0,1,2,9,14,3,5,17,15,6,8,16,23,18,4,11,24,27,25,20,10,26,50,40,34,31,21,12,13,28,33,32,35,43,57,46,76,65,55,45,38,36,22,39,47,49,53,60,75,95,70,61,59,51,64,63,68,101,111,69,67,74,99,97,7,30,19,42,66,41,54,83,72,29,37,71,81,86,52,58,79,85,84,77,48,62,104,92,89,80,78,56,44,88,93,87,94,96,105,82,127,114,110,109,100,91,73,90,106,103,126,118,125,124,113,117,107,98,112,116,115,123,122,102,108,121,120,119}, //8x8I
{0,3,1,4,6,5,12,16,7,2,10,14,15,17,8,13,22,21,18,20,19,37,35,30,26,24,23,11,9,27,29,28,31,38,45,57,71,50,41,36,34,32,25,33,39,40,44,55,79,90,62,46,49,42,54,53,63,87,94,60,64,74,86,95,43,58,52,51,65,66,67,76,59,47,56,61,73,81,69,72,88,84,78,75,70,77,98,85,83,82,92,68,48,91,97,93,100,102,106,112,127,110,107,104,96,101,80,89,103,109,117,113,126,125,124,116,105,99,108,115,114,123,122,121,111,120,119,118}, //8x8P
{0,3,1,4,6,5,12,16,7,2,10,14,15,17,8,13,22,21,18,20,19,37,35,30,26,24,23,11,9,27,29,28,31,38,45,57,71,50,41,36,34,32,25,33,39,40,44,55,79,90,62,46,49,42,54,53,63,87,94,60,64,74,86,95,43,58,52,51,65,66,67,76,59,47,56,61,73,81,69,72,88,84,78,75,70,77,98,85,83,82,92,68,48,91,97,93,100,102,106,112,127,110,107,104,96,101,80,89,103,109,117,113,126,125,124,116,105,99,108,115,114,123,122,121,111,120,119,118}, //8x8B
{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127}, //16x16I
{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127}, //16x16P
{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127},//16x16B
{0,3,1,4,6,5,12,16,7,2,10,14,15,17,8,13,22,21,18,20,19,37,35,30,26,24,23,11,9,27,29,28,31,38,45,57,71,50,41,36,34,32,25,33,39,40,44,55,79,90,62,46,49,42,54,53,63,87,94,60,64,74,86,95,43,58,52,51,65,66,67,76,59,47,56,61,73,81,69,72,88,84,78,75,70,77,98,85,83,82,92,68,48,91,97,93,100,102,106,112,127,110,107,104,96,101,80,89,103,109,117,113,126,125,124,116,105,99,108,115,114,123,122,121,111,120,119,118},//SVTP_TYPE1
{0,3,1,4,6,5,12,16,7,2,10,14,15,17,8,13,22,21,18,20,19,37,35,30,26,24,23,11,9,27,29,28,31,38,45,57,71,50,41,36,34,32,25,33,39,40,44,55,79,90,62,46,49,42,54,53,63,87,94,60,64,74,86,95,43,58,52,51,65,66,67,76,59,47,56,61,73,81,69,72,88,84,78,75,70,77,98,85,83,82,92,68,48,91,97,93,100,102,106,112,127,110,107,104,96,101,80,89,103,109,117,113,126,125,124,116,105,99,108,115,114,123,122,121,111,120,119,118}}; //SVTP_TYPE0

const UInt g_auiLPTableD8[10][128] = {{0,2,1,5,3,64,6,4,14,9,15,7,27,8,66,65,28,13,10,12,16,20,11,26,17,69,21,19,18,51,30,25,92,29,67,42,35,24,22,40,78,70,23,41,31,32,34,39,55,33,37,56,68,73,48,38,45,36,47,74,46,91,79,50,57,59,44,84,77,54,49,58,71,85,72,43,62,52,99,87,60,75,76,95,83,61,96,53,98,82,90,102,101,89,63,80,127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,100,97,94,93,88,86,81},
{0,2,1,5,3,64,6,4,14,9,15,7,27,8,66,65,28,13,10,12,16,20,11,26,17,69,21,19,18,51,30,25,92,29,67,42,35,24,22,40,78,70,23,41,31,32,34,39,55,33,37,56,68,73,48,38,45,36,47,74,46,91,79,50,57,59,44,84,77,54,49,58,71,85,72,43,62,52,99,87,60,75,76,95,83,61,96,53,98,82,90,102,101,89,63,80,127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,100,97,94,93,88,86,81},
{0,1,2,5,14,6,9,64,10,3,20,15,27,28,4,8,11,7,13,66,19,26,42,12,16,18,21,17,29,73,65,25,31,30,24,32,41,74,40,43,23,69,67,33,92,39,35,44,84,45,22,53,78,46,70,38,91,34,79,52,47,51,85,55,54,37,68,60,56,59,50,75,72,106,61,48,36,83,90,80,89,76,99,71,82,81,77,95,93,88,107,105,87,94,96,49,97,63,117,62,104,57,123,109,86,98,108,116,124,103,102,58,118,114,101,120,119,115,111,127,126,125,122,121,113,112,110,100},
{0,2,9,1,3,5,4,8,14,28,10,27,6,15,11,12,7,13,18,20,19,17,16,26,25,42,24,29,31,30,23,32,41,43,40,22,39,21,33,44,45,38,53,64,46,34,51,73,92,52,37,67,66,55,54,47,74,35,65,72,59,75,50,56,60,68,69,70,91,78,84,36,79,76,61,83,71,85,82,48,106,77,89,88,81,87,62,57,80,107,49,93,90,95,58,63,104,94,86,117,96,105,97,108,103,116,98,102,118,109,101,124,99,111,120,119,115,110,127,126,125,123,122,121,114,113,112,100},
{0,2,9,1,3,5,4,8,14,28,10,27,6,15,11,12,7,13,18,20,19,17,16,26,25,42,24,29,31,30,23,32,41,43,40,22,39,21,33,44,45,38,53,64,46,34,51,73,92,52,37,67,66,55,54,47,74,35,65,72,59,75,50,56,60,68,69,70,91,78,84,36,79,76,61,83,71,85,82,48,106,77,89,88,81,87,62,57,80,107,49,93,90,95,58,63,104,94,86,117,96,105,97,108,103,116,98,102,118,109,101,124,99,111,120,119,115,110,127,126,125,123,122,121,114,113,112,100},
{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127}, //16x16I
{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127},
{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127},
{0,2,9,1,3,5,4,8,14,28,10,27,6,15,11,12,7,13,18,20,19,17,16,26,25,42,24,29,31,30,23,32,41,43,40,22,39,21,33,44,45,38,53,64,46,34,51,73,92,52,37,67,66,55,54,47,74,35,65,72,59,75,50,56,60,68,69,70,91,78,84,36,79,76,61,83,71,85,82,48,106,77,89,88,81,87,62,57,80,107,49,93,90,95,58,63,104,94,86,117,96,105,97,108,103,116,98,102,118,109,101,124,99,111,120,119,115,110,127,126,125,123,122,121,114,113,112,100},//SVTP_TYPE1
{0,2,9,1,3,5,4,8,14,28,10,27,6,15,11,12,7,13,18,20,19,17,16,26,25,42,24,29,31,30,23,32,41,43,40,22,39,21,33,44,45,38,53,64,46,34,51,73,92,52,37,67,66,55,54,47,74,35,65,72,59,75,50,56,60,68,69,70,91,78,84,36,79,76,61,83,71,85,82,48,106,77,89,88,81,87,62,57,80,107,49,93,90,95,58,63,104,94,86,117,96,105,97,108,103,116,98,102,118,109,101,124,99,111,120,119,115,110,127,126,125,123,122,121,114,113,112,100}}; //SVTP_TYPE0

const UInt g_auiLPTableE4[3][32] = {{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31},  //4x4I
{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31},  //4x4P
{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}}; //4x4B

const UInt g_auiLPTableD4[3][32] = {{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31},  //4x4I
{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31},  //4x4P
{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}}; //4x4B

#endif

const UInt g_auiLastPosVlcIndex[10] = {0,0,0,0,0,0,0,0,0,0};

const UInt g_auiLastPosVlcNum[10][17] =
{
    {10,10,10,10, 2,2,2,7,9,9,9,9,9,4,4,4,4},
    {10,10,10,10,10,2,9,9,9,9,9,9,9,4,4,4,4},
    { 2, 2, 2, 2, 2,7,7,7,7,7,7,7,7,7,4,4,4},
    { 2, 2, 2, 2, 7,7,7,7,7,7,7,7,7,7,7,7,4},
    { 2, 2, 2, 2, 7,7,7,7,7,7,7,7,7,7,7,7,4},
    {10, 1, 2, 2, 2,2,7,7,7,7,9,9,9,4,4,4,4},
    {10,10, 2, 2, 7,7,7,7,7,7,7,7,4,4,4,4,4},
    {10,10, 2, 2, 7,7,7,7,7,7,7,7,4,4,4,4,4},
    { 2, 2, 2, 2, 7,7,7,7,7,7,7,7,7,7,7,7,4},
    { 2, 2, 2, 2, 7,7,7,7,7,7,7,7,7,7,7,7,4}
};

const UInt g_auiLumaRun8x8[29][2][64] =
{
        /* 0 */
  {
            {  1,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
  },
        /* 1 */
  {
            {  2,  1,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {  4,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
  },
        /* 2 */
  {
            {  1,  3,  2,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {  4,  6,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
  },
        /* 3 */
{
            {  2,  1,  3,  4,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {  6,  5,  7,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
  },
        /* 4 */
  {
            {  1,  5,  3,  2,  4,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {  6, 10,  8,  7,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
  },
        /* 5 */
  {
            {  1,  2,  6,  5,  3,  4,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {  7,  8, 11, 10,  9, 12, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
  },
        /* 6 */
{
            {  2,  1,  3,  5,  4,  7,  6,  0, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {  9,  8, 10, 12, 11, 13, 14, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        /* 7 */
{
            {  1,  5,  4,  2,  3,  6,  8,  7,  0, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {  9, 13, 12, 10, 11, 14, 15, 16, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        /* 8 */
{
            {  1,  3,  8,  7,  5,  2,  4,  9,  6,  0, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 10, 12, 16, 15, 14, 11, 13, 18, 17, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        /* 9 */
{
            {  1,  2,  5, 10,  9,  7,  3,  4, 11,  6,  0, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {  8, 12, 15, 17, 18, 16, 13, 14, 20, 19, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
{
            {  2,  1,  3,  4,  7,  8,  5,  6,  9, 11, 10,  0, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 13, 12, 14, 15, 17, 18, 16, 19, 20, 22, 21, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  4,  3,  2,  5,  7,  6,  8, 10, 11, 12,  9,  0, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 13, 17, 15, 14, 16, 19, 18, 20, 22, 23, 24, 21, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  2,  6,  7,  5,  3,  4,  8,  9, 13, 11, 12, 10,  0, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 14, 16, 18, 21, 19, 15, 17, 20, 22, 26, 23, 24, 25, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  2,  4,  8,  9,  7,  6,  3,  5, 12, 14, 13, 11, 10,  0, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 15, 16, 19, 21, 23, 22, 20, 17, 18, 26, 28, 27, 24, 25, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  2,  3,  5,  9, 10,  8,  7,  4,  6, 13, 16, 15, 14, 12,  0,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 11, 17, 18, 20, 22, 24, 23, 26, 19, 21, 27, 30, 29, 25, 28, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  2,  3,  4,  5,  8, 10,  7,  6,  9, 11, 13, 18, 15, 16, 12,
               0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 14, 17, 19, 20, 21, 23, 25, 24, 22, 26, 27, 29, 31, 28, 32, 30,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  3,  4,  2,  5,  6, 10,  9,  7,  8, 11, 12, 13, 16, 17, 18,
              14,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 15, 19, 22, 20, 21, 24, 26, 25, 23, 27, 32, 29, 28, 30, 33, 34,
              31, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  2,  4,  7,  6,  3,  5,  8,  9, 10, 11, 18, 15, 13, 14, 17,
              19, 12,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 16, 21, 24, 25, 26, 20, 22, 27, 23, 28, 30, 35, 31, 29, 34, 33,
              36, 32, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  2,  5,  7,  9, 10,  6,  3,  4,  8, 11, 17, 20, 18, 15, 13,
              14, 19, 12,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 16, 21, 24, 25, 29, 28, 27, 23, 22, 26, 31, 36, 34, 37, 33, 32,
              30, 38, 35, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  3,  5,  7,  9, 14, 12,  8,  6,  2,  4, 15, 18, 21, 20, 16,
              10, 11, 22, 13,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 17, 23, 25, 27, 29, 31, 33, 30, 26, 19, 24, 35, 37, 39, 40, 36,
              32, 28, 38, 34, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  0,  2,  4,  7, 10,  9, 16, 15, 14,  8,  3,  5, 17, 19, 22, 20,
              18, 11, 13, 21, 12,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {  6, 23, 25, 29, 27, 31, 30, 34, 38, 32, 24, 26, 36, 82, 39, 37,
               83, 28, 33, 84, 35, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  2,  1,  3,  4,  5,  8,  9, 11, 13, 10,  6,  7, 12, 14, 21, 24,
              23, 18, 15, 19, 22, 16,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 20, 17, 25, 26, 27, 28, 31, 34, 35, 30, 29, 32, 33, 36, 37, 43,
              44, 38, 39, 40, 41, 42, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  5,  4,  2,  3,  6,  8,  7, 11, 12,  9, 10, 13, 14, 15, 18,
              22, 21, 17, 19, 20, 25, 16,  0, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 26, 28, 27, 23, 24, 29, 31, 32, 38, 34, 30, 33, 35, 37, 36, 43,
              44, 40, 41, 85, 42, 86, 39, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  4,  8,  6,  5,  2,  3,  7,  9, 12, 10, 11, 13, 16, 15, 14,
              18, 20, 21, 19, 23, 22, 25, 17,  0, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 24, 28, 33, 29, 31, 26, 27, 34, 35, 36, 32, 30, 37, 40, 38, 42,
              45, 47, 39, 43, 46, 41, 87, 44, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  0,  3,  7, 12, 10,  6,  5,  2,  4,  8,  9, 11, 13, 17, 24, 21,
              16, 15, 19, 20, 22, 25, 26, 23, 18,  1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 14, 28, 34, 36, 33, 35, 31, 27, 30, 29, 32, 37, 40, 46, 38, 44,
              39, 47, 48, 49, 45, 41, 42, 50, 43, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  0,  2,  5,  9, 13, 11, 10,  8,  6,  3,  4,  7, 12, 15, 19, 26,
              23, 20, 16, 14, 18, 22, 27, 24, 25, 21,  1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 17, 28, 31, 36, 35, 39, 37, 34, 33, 30, 29, 32, 38, 43, 86, 87,
              45, 46, 44, 41, 40, 88, 48, 89, 47, 42, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  1,  2,  4,  7, 11, 15, 12,  9, 10,  8,  5,  3,  6, 14, 16, 23,
              24, 25, 21, 20, 13, 19, 27, 28, 26, 22, 17,  0, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            { 18, 29, 31, 33, 39, 36, 35, 38, 41, 40, 34, 30, 32, 43, 48, 53,
              90, 49, 42, 50, 37, 44, 51, 47, 52, 45, 46, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  0,  2,  3,  5,  7, 11, 15, 14, 12, 10, 13,  8,  4,  9, 17, 19,
              22, 24, 26, 21, 23, 16, 20, 29, 30, 28, 25, 18,  1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {  6, 27, 31, 33, 34, 38, 39, 37, 40, 42, 41, 35, 32, 36, 44, 45,
              87, 48, 88, 49, 46, 89, 43, 47, 90, 50, 51, 91, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        },
        {
            {  0,  1,  2,  3,  4,  5,  7,  9, 10, 13,  8, 12, 11,  6, 14, 15,
              18, 20, 24, 31, 32, 30, 28, 26, 22, 25, 33, 37, 38, 17, 19, 21,
              23, 29, 36, 34, 45, 40, 43, 42, 44, 39, 35, 46, 48, 53, 54, 51,
              61, 62, 67, 66, 68, 65, 73, 74, 71, 69, 86, 82, 90, 88, 98, 94},
            { 16, 27, 41, 47, 50, 57, 58, 60, 59, 55, 52, 63, 56, 49, 64, 70,
              72, 77, 76, 80, 84, 81, 79, 78, 75, 83, 85, 87, 91, 95, 92,104,
              89, 93, 96, 99,100,102,101, 97,105,103,106,107,108,109,110,111,
             112,113,114,115,116,117,118,119,120,121,122,123,124,125,126, -1}
        }
};

const UInt g_auiVlcTable8x8[28] = {8,0,0,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6};

const LastCoeffStruct g_acstructLumaRun8x8[29][127] =
{
        {{0,1},{0,0},{1,0},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,2},{0,1},{0,0},{1,1},{1,0},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,3},{0,0},{0,2},{0,1},{1,0},{1,2},{1,1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,4},{0,1},{0,0},{0,2},{0,3},{1,1},{1,0},{1,2},{1,3},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,5},{0,0},{0,3},{0,2},{0,4},{0,1},{1,0},{1,3},{1,2},{1,4},{1,1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,6},{0,0},{0,1},{0,4},{0,5},{0,3},{0,2},{1,0},{1,1},{1,4},{1,3},{1,2},{1,5},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,7},{0,1},{0,0},{0,2},{0,4},{0,3},{0,6},{0,5},{1,1},{1,0},{1,2},{1,4},{1,3},{1,5},{1,6},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,8},{0,0},{0,3},{0,4},{0,2},{0,1},{0,5},{0,7},{0,6},{1,0},{1,3},{1,4},{1,2},{1,1},{1,5},{1,6},{1,7},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,9},{0,0},{0,5},{0,1},{0,6},{0,4},{0,8},{0,3},{0,2},{0,7},{1,0},{1,5},{1,1},{1,6},{1,4},{1,3},{1,2},{1,8},{1,7},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1}},

        {{0,10},{0,0},{0,1},{0,6},{0,7},{0,2},{0,9},{0,5},{1,0},{0,4},{0,3},{0,8},{1,1},{1,6},{1,7},{1,2},{1,5},{1,3},{1,4},{1,9},{1,8},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1}},

        {{0,11},{0,1},{0,0},{0,2},{0,3},{0,6},{0,7},{0,4},{0,5},{0,8},{0,10},{0,9},{1,1},{1,0},{1,2},{1,3},{1,6},{1,4},{1,5},{1,7},{1,8},{1,10},
         {1,9},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1}},

        {{0,12},{0,0},{0,3},{0,2},{0,1},{0,4},{0,6},{0,5},{0,7},{0,11},{0,8},{0,9},{0,10},{1,0},{1,3},{1,2},{1,4},{1,1},{1,6},{1,5},{1,7},{1,11},
         {1,8},{1,9},{1,10},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1}},

        {{0,13},{0,0},{0,1},{0,5},{0,6},{0,4},{0,2},{0,3},{0,7},{0,8},{0,12},{0,10},{0,11},{0,9},{1,0},{1,5},{1,1},{1,6},{1,2},{1,4},{1,7},{1,3},
         {1,8},{1,10},{1,11},{1,12},{1,9},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1}},

        {{0,14},{0,0},{0,1},{0,7},{0,2},{0,8},{0,6},{0,5},{0,3},{0,4},{0,13},{0,12},{0,9},{0,11},{0,10},{1,0},{1,1},{1,7},{1,8},{1,2},{1,6},{1,3},
         {1,5},{1,4},{1,12},{1,13},{1,9},{1,11},{1,10},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1}},

        {{0,15},{0,0},{0,1},{0,2},{0,8},{0,3},{0,9},{0,7},{0,6},{0,4},{0,5},{1,0},{0,14},{0,10},{0,13},{0,12},{0,11},{1,1},{1,2},{1,8},{1,3},{1,9},
         {1,4},{1,6},{1,5},{1,13},{1,7},{1,10},{1,14},{1,12},{1,11},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1}},

        {{0,16},{0,0},{0,1},{0,2},{0,3},{0,4},{0,8},{0,7},{0,5},{0,9},{0,6},{0,10},{0,15},{0,11},{1,0},{0,13},{0,14},{1,1},{0,12},{1,2},{1,3},{1,4},
         {1,8},{1,5},{1,7},{1,6},{1,9},{1,10},{1,13},{1,11},{1,15},{1,12},{1,14},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1}},

        {{0,17},{0,0},{0,3},{0,1},{0,2},{0,4},{0,5},{0,8},{0,9},{0,7},{0,6},{0,10},{0,11},{0,12},{0,16},{1,0},{0,13},{0,14},{0,15},{1,1},{1,3},
         {1,4},{1,2},{1,8},{1,5},{1,7},{1,6},{1,9},{1,12},{1,11},{1,13},{1,16},{1,10},{1,14},{1,15},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1}},

        {{0,18},{0,0},{0,1},{0,5},{0,2},{0,6},{0,4},{0,3},{0,7},{0,8},{0,9},{0,10},{0,17},{0,13},{0,14},{0,12},{1,0},{0,15},{0,11},{0,16},{1,5},
         {1,1},{1,6},{1,8},{1,2},{1,3},{1,4},{1,7},{1,9},{1,13},{1,10},{1,12},{1,17},{1,15},{1,14},{1,11},{1,16},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1}},

        {{0,19},{0,0},{0,1},{0,7},{0,8},{0,2},{0,6},{0,3},{0,9},{0,4},{0,5},{0,10},{0,18},{0,15},{0,16},{0,14},{1,0},{0,11},{0,13},{0,17},{0,12},
         {1,1},{1,8},{1,7},{1,2},{1,3},{1,9},{1,6},{1,5},{1,4},{1,16},{1,10},{1,15},{1,14},{1,12},{1,18},{1,11},{1,13},{1,17},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1}},

        {{0,20},{0,0},{0,9},{0,1},{0,10},{0,2},{0,8},{0,3},{0,7},{0,4},{0,16},{0,17},{0,6},{0,19},{0,5},{0,11},{0,15},{1,0},{0,12},{1,9},{0,14},
         {0,13},{0,18},{1,1},{1,10},{1,2},{1,8},{1,3},{1,17},{1,4},{1,7},{1,5},{1,16},{1,6},{1,19},{1,11},{1,15},{1,12},{1,18},{1,13},{1,14},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1}},

        {{0,0},{0,21},{0,1},{0,10},{0,2},{0,11},{1,0},{0,3},{0,9},{0,5},{0,4},{0,17},{0,20},{0,18},{0,8},{0,7},{0,6},{0,12},{0,16},{0,13},{0,15},
         {0,19},{0,14},{1,1},{1,10},{1,2},{1,11},{1,4},{1,17},{1,3},{1,6},{1,5},{1,9},{1,18},{1,7},{1,20},{1,12},{1,15},{1,8},{1,14},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{1,13},{1,16},{1,19},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1}},

        {{0,22},{0,1},{0,0},{0,2},{0,3},{0,4},{0,10},{0,11},{0,5},{0,6},{0,9},{0,7},{0,12},{0,8},{0,13},{0,18},{0,21},{1,1},{0,17},{0,19},{1,0},
         {0,14},{0,20},{0,16},{0,15},{1,2},{1,3},{1,4},{1,5},{1,10},{1,9},{1,6},{1,11},{1,12},{1,7},{1,8},{1,13},{1,14},{1,17},{1,18},{1,19},{1,20},
         {1,21},{1,15},{1,16},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,23},{0,0},{0,3},{0,4},{0,2},{0,1},{0,5},{0,7},{0,6},{0,10},{0,11},{0,8},{0,9},{0,12},{0,13},{0,14},{0,22},{0,18},{0,15},{0,19},{0,20},
         {0,17},{0,16},{1,3},{1,4},{0,21},{1,0},{1,2},{1,1},{1,5},{1,10},{1,6},{1,7},{1,11},{1,9},{1,12},{1,14},{1,13},{1,8},{1,22},{1,17},{1,18},
         {1,20},{1,15},{1,16},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{1,19},{1,21},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,24},{0,0},{0,5},{0,6},{0,1},{0,4},{0,3},{0,7},{0,2},{0,8},{0,10},{0,11},{0,9},{0,12},{0,15},{0,14},{0,13},{0,23},{0,16},{0,19},{0,17},
         {0,18},{0,21},{0,20},{1,0},{0,22},{1,5},{1,6},{1,1},{1,3},{1,11},{1,4},{1,10},{1,2},{1,7},{1,8},{1,9},{1,12},{1,14},{1,18},{1,13},{1,21},
         {1,15},{1,19},{1,23},{1,16},{1,20},{1,17},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{1,22},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},
        {{0,0},{0,25},{0,7},{0,1},{0,8},{0,6},{0,5},{0,2},{0,9},{0,10},{0,4},{0,11},{0,3},{0,12},{1,0},{0,17},{0,16},{0,13},{0,24},{0,18},{0,19},
         {0,15},{0,20},{0,23},{0,14},{0,21},{0,22},{1,7},{1,1},{1,9},{1,8},{1,6},{1,10},{1,4},{1,2},{1,5},{1,3},{1,11},{1,14},{1,16},{1,12},{1,21},
         {1,22},{1,24},{1,15},{1,20},{1,13},{1,17},{1,18},{1,19},{1,23},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},
        {{0,0},{0,26},{0,1},{0,9},{0,10},{0,2},{0,8},{0,11},{0,7},{0,3},{0,6},{0,5},{0,12},{0,4},{0,19},{0,13},{0,18},{1,0},{0,20},{0,14},{0,17},
         {0,25},{0,21},{0,16},{0,23},{0,24},{0,15},{0,22},{1,1},{1,10},{1,9},{1,2},{1,11},{1,8},{1,7},{1,4},{1,3},{1,6},{1,12},{1,5},{1,20},{1,19},
         {1,25},{1,13},{1,18},{1,16},{1,17},{1,24},{1,22},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{1,14},{1,15},{1,21},{1,23},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},
        {{0,27},{0,0},{0,1},{0,11},{0,2},{0,10},{0,12},{0,3},{0,9},{0,7},{0,8},{0,4},{0,6},{0,20},{0,13},{0,5},{0,14},{0,26},{1,0},{0,21},{0,19},
         {0,18},{0,25},{0,15},{0,16},{0,17},{0,24},{0,22},{0,23},{1,1},{1,11},{1,2},{1,12},{1,3},{1,10},{1,6},{1,5},{1,20},{1,7},{1,4},{1,9},{1,8},
         {1,18},{1,13},{1,21},{1,25},{1,26},{1,23},{1,14},{1,17},{1,19},{1,22},{1,24},{1,15},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{1,16},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,0},{0,28},{0,1},{0,2},{0,12},{0,3},{1,0},{0,4},{0,11},{0,13},{0,9},{0,5},{0,8},{0,10},{0,7},{0,6},{0,21},{0,14},{0,27},{0,15},{0,22},
         {0,19},{0,16},{0,20},{0,17},{0,26},{0,18},{1,1},{0,25},{0,23},{0,24},{1,2},{1,12},{1,3},{1,4},{1,11},{1,13},{1,7},{1,5},{1,6},{1,8},{1,10},
         {1,9},{1,22},{1,14},{1,15},{1,20},{1,23},{1,17},{1,19},{1,25},{1,26},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{1,16},{1,18},{1,21},{1,24},{1,27},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
         {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},

        {{0,0},{0,1},{0,2},{0,3},{0,4},{0,5},{0,13},{0,6},{0,10},{0,7},{0,8},{0,12},{0,11},{0,9},{0,14},{0,15},{1,0},{0,29},{0,16},{0,30},{0,17},
         {0,31},{0,24},{0,32},{0,18},{0,25},{0,23},{1,1},{0,22},{0,33},{0,21},{0,19},{0,20},{0,26},{0,35},{0,42},{0,34},{0,27},{0,28},{0,41},{0,37},
         {1,2},{0,39},{0,38},{0,40},{0,36},{0,43},{1,3},{0,44},{1,13},{1,4},{0,47},{1,10},{0,45},{0,46},{1,9},{1,12},{1,5},{1,6},{1,8},{1,7},{0,48},
         {0,49},{1,11},{1,14},{0,53},{0,51},{0,50},{0,52},{0,57},{1,15},{0,56},{1,16},{0,54},{0,55},{1,24},{1,18},{1,17},{1,23},{1,22},{1,19},
         {1,21},{0,59},{1,25},{1,20},{1,26},{0,58},{1,27},{0,61},{1,32},{0,60},{1,28},{1,30},{1,33},{0,63},{1,29},{1,34},{1,39},{0,62},{1,35},
         {1,36},{1,38},{1,37},{1,41},{1,31},{1,40},{1,42},{1,43},{1,44},{1,45},{1,46},{1,47},{1,48},{1,49},{1,50},{1,51},{1,52},{1,53},{1,54},
         {1,55},{1,56},{1,57},{1,58},{1,59},{1,60},{1,61},{1,62}}
};

// ====================================================================================================================
// ADI
// ====================================================================================================================

// context pixel filtering flag for each CU size and modes
const UChar g_aucIntraFilter[7][40] =
{
  {0,0,0,0,0,  0,0,0,0,0,    // 2x2
   0,0,0,0,0,  0,0,0,0,0,
   0,0,0,0,0,  0,0,0,0,0,
   0,0,0,0,0,  0,0,0,0,0
  },
  {0,0,0,0,1,  1,1,1,1,1,    // 4x4
   1,1,1,1,1,  1,1,1,1,1,
   0,0,0,0,0,  0,0,0,0,0,
   0,0,0,0,0,  0,0,0,0,0
  },
  {1,1,1,1,2,  2,2,2,2,2,    // 8x8
   2,2,2,2,2,  2,2,2,1,1,
   1,1,1,1,1,  1,1,1,1,1,
   1,1,1,1,1,  1,1,1,1,1
  },
  {0,0,0,0,1,  1,1,1,1,1,  // 16x16
   1,1,1,1,1,  1,1,1,1,1,
   1,1,1,1,1,  1,1,1,1,1,
   1,1,1,1,1,  1,1,1,1,1,
  },
  {0,0,0,0,1,  1,1,1,1,1,   // 32x32
   1,1,1,1,1,  1,1,1,1,1,
   1,1,1,1,1,  1,1,1,1,1,
   1,1,1,1,1,  1,1,1,1,1,
  },
  {0,0,0,0,1,  1,1,1,1,1,   // 64x64
   1,1,1,1,1,  1,1,1,1,1,
   1,1,1,1,1,  1,1,1,1,1,
   1,1,1,1,1,  1,1,1,1,1,
  },
  {0,0,0,0,1,  1,1,1,1,1,   // 128x128
   1,1,1,1,1,  1,1,1,1,1,
   1,1,1,1,1,  1,1,1,1,1,
   1,1,1,1,1,  1,1,1,1,1,
  }
};

const UChar g_aucIntraModeOrder[7][40] =
{
  {0,1,2,0,0,  0,0,0,0,0,    // 2x2
   0,0,0,0,0,  0,0,0,0,0,
   0,0,0,0,0,  0,0,0,0,0,
   0,0,0,0,0,  0,0,0,0,0
  },
  {0, 1, 2, 4, 5,   6, 7, 8, 9,10,    // 4x4
   11,12,13,14,15,  16,17,18,19,20,
   21,22,23,24,25,  26,27,28,29,30,
   31,32,33,34,35,  36,37,38,39,40
  },
  {0, 1, 2, 4, 5,   6, 7, 8, 9,10,    // 8x8
   11,12,13,14,15,  16,17,18,19,20,
   21,22,23,24,25,  26,27,28,29,30,
   31,32,33,34,35,  36,37,38,39,40
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8, 9,   // 16x16
   10,11,12,13,14,  15,16,17,18,19,
   20,21,22,23,24,  25,26,27,28,29,
   30,31,32,33,34,  35,36,37,38,39
  },
  {0, 1, 2, 3, 4,   5, 6, 7, 8, 9,   // 32x32
   10,11,12,13,14,  15,16,17,18,19,
   20,21,22,23,24,  25,26,27,28,29,
   30,31,32,33,34,  35,36,37,38,39
  },
  {0, 1, 2, 3, 4,   5, 6, 7, 8, 9,   // 64x64
   10,11,12,13,14,  15,16,17,18,19,
   20,21,22,23,24,  25,26,27,28,29,
   30,31,32,33,34,  35,36,37,38,39
  },
  {0, 1, 2, 3, 4,   5, 6, 7, 8, 9,   // 128x128
   10,11,12,13,14,  15,16,17,18,19,
   20,21,22,23,24,  25,26,27,28,29,
   30,31,32,33,34,  35,36,37,38,39
  },
};

const UChar g_aucIntraModeConv[7][40] = // intra mode conversion for most probable
{
  {0, 1, 2, 3, 4,  5, 6, 7, 8, 2,   // 2x2
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8, 2,   // 4x4
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8, 2,   // 8x8
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2
  },
  {0, 1, 2, 2, 3,   4, 5, 6, 7, 8,   // 16x16
   0, 3, 3, 6, 5,   0, 3, 8, 0, 7,
   3, 3, 3, 8, 1,   1, 6, 5, 5, 5,
   3, 3, 2, 8, 8,   8, 2, 2, 2, 2
  },
  {0, 1, 2, 2, 3,   4, 5, 6, 7, 8,   // 32x32
   0, 3, 3, 6, 5,   0, 3, 8, 0, 7,
   3, 3, 3, 8, 1,   1, 6, 5, 5, 5,
   3, 3, 2, 8, 8,   8, 2, 2, 2, 2
  },
  {0, 1, 2, 2, 3,   4, 5, 6, 7, 8,   // 64x64
   0, 3, 3, 6, 5,   0, 3, 8, 0, 7,
   3, 3, 3, 8, 1,   1, 6, 5, 5, 5,
   3, 3, 2, 8, 8,   8, 2, 2, 2, 2
  },
  {0, 1, 2, 2, 3,   4, 5, 6, 7, 8,   // 128x128
   0, 3, 3, 6, 5,   0, 3, 8, 0, 7,
   3, 3, 3, 8, 1,   1, 6, 5, 5, 5,
   3, 3, 2, 8, 8,   8, 2, 2, 2, 2
  },
};

const UChar g_aucIntraModeConvInv[7][9] = // intra mode conversion for most probable
{
  {0, 1, 2, 3, 4,  5, 6, 7, 8    //2x2
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8    //4x4
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8    //8x8
  },
  {0, 1, 2, 4, 5,  6, 7, 8, 9   //16x16
  },
  {0, 1, 2, 4, 5,  6, 7, 8, 9   //32x32
  },
  {0, 1, 2, 4, 5,  6, 7, 8, 9   //64x64
  },
  {0, 1, 2, 4, 5,  6, 7, 8, 9   //128x128
  }
};

const UChar g_aucIntraModeNum[7] =
{
    3,  //   2x2
    9,  //   4x4
    9,  //   8x8
   33,  //  16x16   33
   33,  //  32x32   33
    5,  //  64x64   33
    5   // 128x128  5
};

const UChar g_aucIntraModeBits[7] =
{
   2,  //   2x2   1+1
   4,  //   4x4   3+1
   4,  //   8x8   3+1
   6,  //  16x16   33   5+1
   6,  //  32x32   33   5+1
   3,  //  64x64   33   4+1
   3   // 128x128  33   4+1
};

const UChar g_aucIntraModeNumFast[7] =
{
   3,  //   2x2
   9,  //   4x4
   9,  //   8x8
   4,  //  16x16   33
   4,  //  32x32   33
   5,  //  64x64   33
   4   // 128x128  33
};

const UChar g_aucIntraAvail[40][2] =
{
  {1,0},
  {0,1},
  {0,0},
  {1,1},
  {1,0},
  {1,1},
  {1,1},
  {1,1},
  {1,0},
  {0,1},

  {1,0},
  {1,0},
  {1,0},
  {1,1},
  {1,1},
  {1,1},
  {0,1},
  {0,1},
  {1,0},
  {1,0},

  {1,0},
  {1,0},
  {1,0},
  {1,0},
  {1,1},
  {1,1},
  {1,1},
  {1,1},
  {1,1},
  {1,1},

  {0,1},
  {0,1},
  {0,1},
  {0,1},
  {0,1},
  {0,1},
  {0,1},
  {0,1},
  {0,1},
  {0,1}
};

const UChar g_aucXYflg[40]=
{
   0, 0, 0, 0,
   0, 0, 0, 0, 0,
   1, 0, 0, 0, 0,
   0, 0, 1, 1, 0,
   0, 0, 0, 0, 0,
   0, 0, 0, 0, 0,
   0, 1, 1, 1, 1,
   1, 1,
   1, 1, 1, 1
};

const Char g_aucDirDx[32] =
{
   1,  1,  1,  2,  1,
   2,  2,  5, 10, 11,
   4,  1,  1, 12,  1,
   3,  3,  5,  7,  7,
  11,  6,  8,  5,  5,
   2,  5,  4,  5,  5,
   8, 10
};

const Char g_aucDirDy[32] =
{
   -1,   1,   2,   1,  -2,
   -1, -11,  -7,  -7,   3,
    3,  11,  -1,  -3, -11,
  -10,  -5,  -6,  -6,  -4,
    1,   1,   3,   3,   7,
    7,  -7,  -3,  -3,  -1,
   -3,  -1
};

// chroma
const UChar g_aucIntraModeConvC[7][40] = // intra mode conversion for most probable
{
  {0, 1, 2, 3, 4,  2, 2, 2, 2, 2,   // 2x2        4 mode
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2
  },
  {0, 1, 2, 3, 4,  2, 2, 2, 2, 2,    // 4x4       4 mode
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8, 2,    // 8x8       9 mode
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8, 2,    // 16x16     9 mode
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8, 2,    // 32x32     9 mode
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2
  },
  /*
  {0, 1, 2, 3, 4,  5, 6, 7, 8, 2,    // 32x32     33 mode
   0, 4, 4, 7, 6,  0, 4, 2, 0, 8,
   4, 4, 4, 2, 1,  1, 7, 6, 6, 6,
   4, 4, 3, 2, 2,  2, 2, 2, 2, 2
  },
  */
  {0, 1, 2, 3, 4,  5, 6, 7, 8, 2,    // 64x64     9 mode
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8, 2,    // 128x128   9 mode
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2,
   2, 2, 2, 2, 2,  2, 2, 2, 2, 2
  },
};

const UChar g_aucIntraModeConvInvC[7][9] = // intra mode conversion for most probable
{
  {0, 1, 2, 3, 4,  5, 6, 7, 8    //2x2
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8    //4x4
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8    //8x8
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8   //16x16
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8   //32x32
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8   //64x64
  },
  {0, 1, 2, 3, 4,  5, 6, 7, 8   //128x128
  }
};

const UChar g_aucIntraModeNumC[7] =
{

    4,  //   2x2
    4,  //   4x4
    9,  //   8x8     9
    9,  //  16x16    9
    9,  //  32x32    9
    9,  //  64x64    9
    9   // 128x128   9


};

const UChar g_aucIntraModeBitsC[7] =
{
   3,  //   2x2   2+1
   3,  //   4x4   2+1
   5,  //   8x8   2+1
   5,  //  16x16      4+1
   5,  //  32x32      4+1
   5,  //  64x64      4+1
   5   // 128x128     4+1
};

const UChar g_aucConvertTxtTypeToIdx[4] = { 0, 1, 1, 2 };

#if ANG_INTRA
// ====================================================================================================================
// Angular Intra prediction
// ====================================================================================================================

// g_aucAngIntraModeOrder
//   Indexing this array with the mode indicated in the bitstream
//   gives a logical index used in the prediction functions.
const UChar g_aucAngIntraModeOrder[34] =
{     //  ModeOrder LogicalOrderInPredFunctions
   9, //  0 VER     DC
  25, //  1 HOR     VER-8 (diagonal from top-left to bottom-right = HOR-8)
   0, //  2 DC      VER-7
   1, //  3 VER-8   VER-6
   5, //  4 VER-4   VER-5
  13, //  5 VER+4   VER-4
  17, //  6 VER+8   VER-3
  21, //  7 HOR-4   VER-2
  29, //  8 HOR+4   VER-1
  33, //  9 HOR+8   VER
   3, // 10 VER-6   VER+1
   7, // 11 VER-2   VER+2
  11, // 12 VER+2   VER+3
  15, // 13 VER+6   VER+4
  19, // 14 HOR-6   VER+5
  23, // 15 HOR-2   VER+6
  27, // 16 HOR+2   VER+7
  31, // 17 HOR+6   VER+8
   2, // 18 VER-7   HOR-7
   4, // 19 VER-5   HOR-6
   6, // 20 VER-3   HOR-5
   8, // 21 VER-1   HOR-4
  10, // 22 VER+1   HOR-3
  12, // 23 VER+3   HOR-2
  14, // 24 VER+5   HOR-1
  16, // 25 VER+7   HOR
  18, // 26 HOR-7   HOR+1
  20, // 27 HOR-5   HOR+2
  22, // 28 HOR-3   HOR+3
  24, // 29 HOR-1   HOR+4
  26, // 30 HOR+1   HOR+5
  28, // 31 HOR+3   HOR+6
  30, // 32 HOR+5   HOR+7
  32, // 33 HOR+7   HOR+8
};

# if UNIFIED_DIRECTIONAL_INTRA
const UChar g_aucIntraModeNumAng[7] =
{
    3,  //   2x2
   17,  //   4x4
   34,  //   8x8
   34,  //  16x16
   34,  //  32x32
    5,  //  64x64
    5   // 128x128
};

const UChar g_aucIntraModeBitsAng[7] =
{
   2,  //   2x2     3   1+1
   5,  //   4x4    17   4+1
   6,  //   8x8    34   5+esc
   6,  //  16x16   34   5+esc
   6,  //  32x32   34   5+esc
   3,  //  64x64    5   2+1
   3   // 128x128   5   2+1
};

const UChar g_aucAngModeMapping[3][34] = // intra mode conversion for most probable
{
  {2,3,2,2,4, 4,4,0,0,0, 0,0,0,0,2, 2,2,2,2,2, 2,1,1,1,1, 1,1,1,1,1, 2,2,2,2},               // conversion to 5 modes
  {2,3,3,2,4, 4,4,2,0,0, 0,2,5,5,5, 2,6,6,6,2, 7,7,7,2,1, 1,1,2,8,8, 8,2,2,2},               // conversion to 9 modes
  {2,3,3,10,10, 4,11,11,0,0, 0,12,12,5,5, 13,13,6,14,14, 7,7,15,15,1, 1,1,16,16,8, 8,2,2,9}  // conversion to 17 modes
};
# endif
#endif

#if QC_MDDT
# if UNIFIED_DIRECTIONAL_INTRA
// Mapping each UDI prediction direction to MDDT transform directions
const UChar g_aucAngIntra9Mode[34] =
{ //0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33
    0, 1, 2, 4, 5, 7, 3, 6, 8, 3, 5, 0, 0, 7, 6, 1, 1, 8, 4, 5, 5, 0, 0, 7, 7, 3, 4, 6, 6, 1, 1, 8, 8, 3
};

# else
const UChar g_aucAngIntra9Mode[34] =
{
    0, 1, 2, 4, 5, 7, 3, 6, 8, 3, 4, 0, 0, 3, 4, 1, 1, 3, 4, 5, 5, 0, 0,
    7, 7, 3, 4, 6, 6, 1, 1, 8, 8, 3
};
# endif
#endif

#if QC_MDDT
# if UNIFIED_DIRECTIONAL_INTRA
// Same table for all sizes
const UChar g_aucIntra9Mode[34] =
{
    0, 1, 2, 4, 5, 7, 3, 6, 8, 3, 5, 0, 0, 7, 6, 1, 1, 8, 4, 5, 5, 0, 0, 7, 7, 3, 4, 6, 6, 1, 1, 8, 8, 3
};
# elif ANG_INTRA == 2
const UChar g_aucIntra9Mode[34] =
{
    0, 1, 2, 4, 5, 7, 3, 6, 8, 3, 4, 0, 0, 3, 4, 1, 1, 3, 4, 5, 5, 0, 0,
    7, 7, 3, 4, 6, 6, 1, 1, 8, 8, 3
};
# else
const UChar g_aucIntra9Mode[33] =
{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 3, 3, 6, 4, 0, 3, 8, 0, 7, 7,
    3, 3, 8, 1, 1, 6, 6, 4, 5, 3, 3, 8, 1
};
# endif
#endif

// ====================================================================================================================
// ROT
// ====================================================================================================================

const Int g_INV_ROT_MATRIX_4[4][18]=
  {
   { 248,   -55,   -28,   -60,  -244,   -52,   -16,    57,  -249,   250,   -49,   -26,   -54,  -240,   -71,   -11,    75,  -245,},
   { 255,    22,    -8,   -22,   254,   -17,     6,    17,   255,   241,   -84,   -18,    69,   221,  -110,    52,    98,   231,},
   { 237,   -96,    -9,    90,   229,   -73,    35,    64,   245,   243,   -80,    -6,    70,   222,  -107,    38,   100,   233,},
   { 250,   -18,   -54,   -13,   220,  -131,    55,   130,   213,   212,  -143,     9,   134,   192,  -103,    51,    90,   234,},
};
const Int g_INV_ROT_MATRIX_8[4][36]=
{
   { 251,   -36,   -31,    23,   237,   -94,    42,    90,   236,   236,  -100,     3,   100,   234,   -25,     7,    24,   255,   251,   -36,   -31,    23,   237,   -94,    42,    90,   236,   236,  -100,     3,   100,   234,   -25,     7,    24,   255,},
   { 243,   -61,   -50,   -75,  -230,   -83,   -25,    94,  -237,   254,   -27,   -18,    22,   249,   -56,    24,    54,   249,   243,   -61,   -50,   -75,  -230,   -83,   -25,    94,  -237,   254,   -27,   -18,    22,   249,   -56,    24,    54,   249,},
   { 243,   -81,    12,    81,   232,   -70,    11,    71,   246,   252,   -41,    -9,   -42,  -251,   -28,    -4,    29,  -254,   243,   -81,    12,    81,   232,   -70,    11,    71,   246,   252,   -41,    -9,   -42,  -251,   -28,    -4,    29,  -254,},
   { 174,  -186,    23,   181,   158,   -87,    49,    75,   240,   236,   -99,   -10,   -99,  -235,   -25,     1,    27,  -255,   174,  -186,    23,   181,   158,   -87,    49,    75,   240,   236,   -99,   -10,   -99,  -235,   -25,     1,    27,  -255,},
};
const Int g_FWD_ROT_MATRIX_4[4][18]=
  {
  { 3981,  -881,  -458,  -955, -3886,  -828,  -248,   911, -3987,  3999,  -776,  -417,  -870, -3837, -1136,  -172,  1194, -3907,},
  { 4079,   345,  -119,  -360,  4079,  -263,   104,   283,  4091,  3858, -1351,  -295,  1101,  3532, -1746,   825,  1577,  3685,},
  { 3796, -1537,  -141,  1433,  3647, -1157,   566,  1030,  3930,  3894, -1271,   -90,  1125,  3546, -1706,   617,  1596,  3715,},
  { 3988,  -277,  -861,  -199,  3510, -2091,   889,  2088,  3419,  3394, -2292,   142,  2146,  3078, -1651,   814,  1443,  3749,},
};
const Int g_FWD_ROT_MATRIX_8[4][36]=
  {
  { 4032,  -587,  -494,   357,  3791, -1509,   672,  1433,  3777,  3763, -1603,    48,  1597,  3756,  -397,   112,   387,  4073,  4032,  -587,  -494,   357,  3791, -1509,   672,  1433,  3777,  3763, -1603,    48,  1597,  3756,  -397,   112,   387,  4073,},
  { 3903,  -983,  -802, -1200, -3686, -1335,  -403,  1498, -3788,  4062,  -426,  -299,   359,  3978,  -897,   374,   864,  3988,  3903,  -983,  -802, -1200, -3686, -1335,  -403,  1498, -3788,  4062,  -426,  -299,   359,  3978,  -897,   374,   864,  3988,},
  { 3874, -1292,   200,  1298,  3725, -1133,   180,  1123,  3930,  4048,  -662,  -139,  -669, -4016,  -448,   -70,   466, -4074,  3874, -1292,   200,  1298,  3725, -1133,   180,  1123,  3930,  4048,  -662,  -139,  -669, -4016,  -448,   -70,   466, -4074,},
  { 2784, -2988,   365,  2904,  2545, -1388,   786,  1209,  3831,  3776, -1575,  -152, -1590, -3749,  -403,     8,   429, -4067,  2784, -2988,   365,  2904,  2545, -1388,   786,  1209,  3831,  3776, -1575,  -152, -1590, -3749,  -403,     8,   429, -4067,},
};

const  Int g_auiROTFwdShift[5] =
{
  -6,  // 4x4
  -7,  // 8x8
  -3,  // 16x16
  -4,  // 32x32
  -5  // 64x64
};

// ====================================================================================================================
// Bit-depth
// ====================================================================================================================

UInt g_uiBitDepth     = 8;    // base bit-depth
UInt g_uiBitIncrement = 0;    // increments
UInt g_uiIBDI_MAX     = 255;  // max. value after  IBDI
UInt g_uiBASE_MAX     = 255;  // max. value before IBDI

// ====================================================================================================================
// Misc.
// ====================================================================================================================

Char  g_aucConvertToBit  [ MAX_CU_SIZE+1 ];

#if HHI_RQT
#if ENC_DEC_TRACE
FILE*  g_hTrace = NULL;
const Bool g_bEncDecTraceEnable  = true;
const Bool g_bEncDecTraceDisable = false;
Bool   g_bJustDoIt = false;
UInt64 g_nSymbolCounter = 0;
#endif
#endif
// ====================================================================================================================
// Scanning order & context model mapping
// ====================================================================================================================

// scanning order table
UInt* g_auiFrameScanXY[ MAX_CU_DEPTH  ];
UInt* g_auiFrameScanX [ MAX_CU_DEPTH  ];
UInt* g_auiFrameScanY [ MAX_CU_DEPTH  ];

#if HHI_TRANSFORM_CODING
UInt* g_auiSigLastScan[ MAX_CU_DEPTH+1  ][ 2 ];
#endif

// scanning order to 8x8 context model mapping table
UInt  g_auiAntiScan8  [64];

// initialize g_auiFrameScanXY
Void initFrameScanXY( UInt* pBuff, UInt* pBuffX, UInt* pBuffY, Int iWidth, Int iHeight )
{
  Int x, y, c = 0;

  // starting point
  pBuffX[ c ] = 0;
  pBuffY[ c ] = 0;
  pBuff[ c++ ] = 0;

  // loop
  x=1; y=0;
  while (1)
  {
    // decrease loop
    while ( x>=0 )
    {
      if ( x >= 0 && x < iWidth && y >= 0 && y < iHeight )
      {
        pBuffX[ c ] = x;
        pBuffY[ c ] = y;
        pBuff[ c++ ] = x+y*iWidth;
      }
      x--; y++;
    }
    x=0;

    // increase loop
    while ( y>=0 )
    {
      if ( x >= 0 && x < iWidth && y >= 0 && y < iHeight )
      {
        pBuffX[ c ] = x;
        pBuffY[ c ] = y;
        pBuff[ c++ ] = x+y*iWidth;
      }
      x++; y--;
    }
    y=0;

    // termination condition
    if ( c >= iWidth*iHeight ) break;
  }

  // LTR_2D_CONTEXT_MAPPING
  if (iWidth == 8 && iHeight == 8)
  {
    for( c = 0; c < iWidth*iHeight; c++)
    {
      g_auiAntiScan8[pBuff[c]] = c;
    }
  }
}

#if HHI_TRANSFORM_CODING
Void initSigLastScanPattern( UInt* puiScanPattern, const UInt uiLog2BlockSize, const bool bDownLeft )
{
  const int   iBlockSize    = 1 << uiLog2BlockSize;
  const UInt  uiNumScanPos  = UInt( iBlockSize * iBlockSize );
  UInt        uiNextScanPos = 0;

  for( UInt uiScanLine = 0; uiNextScanPos < uiNumScanPos; uiScanLine++ )
  {
    int    iPrimDim  = int( uiScanLine );
    int    iScndDim  = 0;
    while( iPrimDim >= iBlockSize )
    {
      iScndDim++;
      iPrimDim--;
    }
    while( iPrimDim >= 0 && iScndDim < iBlockSize )
    {
      puiScanPattern[ uiNextScanPos++ ] = ( bDownLeft ? iScndDim * iBlockSize + iPrimDim : iPrimDim * iBlockSize + iScndDim );
      iScndDim++;
      iPrimDim--;
    }
  }
  return;
}
#endif

#if QC_MDDT
static UInt count4x4[9] = {0};
static UInt count8x8[9] = {0};
static UInt count16x16[9] = {0};
static UInt count32x32[9] = {0};
UInt g_aiQuantCoef_klt[6][16] =
{
  { 410, 410, 410,410,
    410, 410, 410,410, 
    410, 410, 410,410, 
    410, 410, 410,410
  },
  { 364, 364, 364, 364,
    364, 364, 364, 364,
    364, 364, 364, 364,
    364, 364, 364, 364
  },
  { 328, 328, 328, 328,
    328, 328, 328, 328,
    328, 328, 328, 328,
    328, 328, 328, 328
  },
  {  287, 287, 287, 287, 
     287, 287, 287, 287, 
     287, 287, 287, 287, 
     287, 287, 287, 287
  },
  {  260, 260, 260, 260, 
     260, 260, 260, 260,
     260, 260, 260, 260,
     260, 260, 260, 260
  },
  {  231, 231, 231, 231, 
     231, 231, 231, 231,
     231, 231, 231, 231,
     231, 231, 231, 231
  }
};
Int g_aiDequantCoef_klt[6][16] =
{
  { 40, 40, 40, 40, 
    40, 40, 40, 40, 
    40, 40, 40, 40, 
    40, 40, 40, 40, 
  },
  { 45, 45, 45, 45, 
    45, 45, 45, 45, 
    45, 45, 45, 45, 
    45, 45, 45, 45, 
  },
  { 50, 50, 50, 50, 
    50, 50, 50, 50, 
    50, 50, 50, 50, 
    50, 50, 50, 50, 
  },
  { 57, 57, 57, 57, 
    57, 57, 57, 57, 
    57, 57, 57, 57, 
    57, 57, 57, 57, 
  },
  { 63, 63, 63, 63,
    63, 63, 63, 63,
    63, 63, 63, 63,
    63, 63, 63, 63,
  },
  { 71, 71, 71, 71, 
    71, 71, 71, 71, 
    71, 71, 71, 71, 
    71, 71, 71, 71, 
  }
};
UInt g_aiQuantCoef64_klt[6][64] =
{
  { 410, 410, 410, 410, 410, 410, 410,410,
    410, 410, 410, 410, 410, 410, 410,410, 
    410, 410, 410, 410, 410, 410, 410,410,
    410, 410, 410, 410, 410, 410, 410,410,
    410, 410, 410, 410, 410, 410, 410,410,
    410, 410, 410, 410, 410, 410, 410,410,
    410, 410, 410, 410, 410, 410, 410,410,
    410, 410, 410, 410, 410, 410, 410,410
  },
  { 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364
  },
  { 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328
  },
  {  287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287,
     287, 287, 287, 287, 287, 287, 287, 287,
     287, 287, 287, 287, 287, 287, 287, 287,
     287, 287, 287, 287, 287, 287, 287, 287,
     287, 287, 287, 287, 287, 287, 287, 287
  },
  {  260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260
  },
  {  231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231
  }
};
Int g_aiDequantCoef64_klt[6][64] =
{
  { 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40
  },
  { 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45
  },
  { 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50
  },
  { 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57
  },
  { 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63
  },
  { 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71
  }

};

UInt g_aiQuantCoef256_klt[6][256] =
{
  { 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410,
    410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410, 410, 410, 410,410
  },
  { 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 
    364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364, 364
  },
  { 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328,
    328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328
  },
  {  287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 
     287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287, 287
  },
  {  260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 
     260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260
  },
  {  231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 
     231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231
  }
};
UInt g_aiDequantCoef256_klt[6][256] =
{
  { 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40 
  },
  { 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45
  },
  { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50
  },
  { 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 
    57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57
  },
  { 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63
  },
  { 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 
    71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71
  }
};
// 8x8 KLTs
const short kltRow8x8[9][8][8] = {
//kltRow8x8[0][8][8]
{
{  31,  40,  45,  49,  51,  51,  49,  43},
{ -52, -59, -47, -23,   6,  35,  54,  57},
{ -58, -42,  10,  57,  60,  21, -30, -56},
{  55,   5, -57, -42,  34,  64,   5, -56},
{ -55,  33,  53, -37, -45,  45,  41, -49},
{  47, -59,   4,  52, -50, -11,  61, -40},
{ -31,  60, -57,  21,  25, -57,  59, -28},
{ -15,  36, -53,  63, -63,  52, -35,  14},
          },
//kltRow8x8[1][8][8]
{
{  18,  28,  37,  45,  51,  55,  57,  56},
{  42,  58,  57,  41,  14, -19, -46, -61},
{ -56, -53, -11,  41,  61,  38, -16, -56},
{  58,  20, -49, -55,  19,  64,  16, -50},
{ -58,  22,  59, -29, -54,  38,  45, -42},
{  53, -56,  -3,  54, -44, -18,  63, -36},
{ -37,  62, -53,  15,  28, -57,  59, -26},
{  16, -38,  55, -64,  62, -50,  33, -12},
    },
//kltRow8x8[2][8][8]       DC
{
{  32,  40,  44,  46,  49,  50,  50,  49},
{  56,  59,  46,  24,  -4, -31, -53, -58},
{ -57, -40,  13,  56,  62,  26, -29, -54},
{  53,   4, -58, -44,  34,  65,   7, -52},
{ -53,  33,  54, -38, -46,  47,  38, -46},
{  47, -62,   8,  48, -45, -11,  64, -42},
{ -33,  60, -54,  22,  24, -55,  61, -31},
{  12, -33,  53, -66,  65, -52,  32, -12},
      },
//kltRow8x8[3][8][8]
{
{  37,  45,  49,  51,  50,  47,  43,  38},
{  61,  59,  38,   8, -21, -43, -52, -52},
{ -63, -27,  31,  63,  47,   3, -38, -56},
{  55, -15, -62, -18,  52,  54,  -8, -57},
{ -48,  49,  34, -56, -26,  58,  28, -51},
{  36, -61,  26,  38, -61,   9,  59, -45},
{ -23,  53, -60,  35,   9, -52,  66, -33},
{  11, -31,  48, -60,  64, -57,  42, -17},
      },
//kltRow8x8[4][8][8]
{
{  10,  21,  33,  45,  54,  60,  59,  53},
{  25,  49,  61,  54,  24, -17, -49, -59},
{ -40, -60, -36,  22,  61,  45, -16, -59},
{  53,  47, -23, -62,  -2,  62,  20, -53},
{ -61,  -9,  63,  -2, -60,  26,  49, -44},
{  64, -38, -29,  60, -31, -27,  60, -34},
{ -51,  63, -41,   1,  37, -57,  54, -24},
{  27, -46,  58, -61,  57, -46,  31, -12},
        },
//kltRow8x8[5][8][8]
{
{  11,  22,  34,  45,  53,  59,  59,  54},
{  25,  49,  61,  53,  24, -16, -49, -60},
{ -38, -60, -37,  21,  62,  46, -15, -57},
{  54,  48, -25, -62,  -1,  61,  20, -51},
{ -65,  -7,  62,  -7, -57,  29,  47, -44},
{  64, -43, -22,  58, -35, -23,  59, -35},
{ -49,  64, -48,   9,  31, -55,  54, -25},
{  22, -41,  54, -61,  61, -50,  35, -14},
        },
//kltRow8x8[6][8][8]
{
{  10,  21,  33,  44,  53,  59,  60,  55},
{  27,  50,  59,  53,  26, -13, -46, -63},
{ -45, -60, -33,  20,  58,  47, -10, -60},
{  59,  42, -28, -60,  -3,  59,  26, -50},
{ -62,   2,  61,  -7, -61,  22,  53, -42},
{  62, -48, -21,  61, -32, -29,  58, -30},
{ -43,  64, -49,   7,  36, -59,  53, -22},
{  21, -43,  57, -62,  59, -48,  31, -12},
          },
//kltRow8x8[7][8][8]
{
{  36,  43,  48,  50,  50,  48,  45,  40},
{ -58, -60, -42, -13,  16,  41,  53,  52},
{ -60, -33,  23,  63,  53,   8, -37, -54},
{  56,  -7, -61, -27,  47,  57,  -6, -57},
{ -52,  44,  41, -49, -31,  55,  31, -51},
{  39, -61,  18,  44, -58,   3,  59, -45},
{ -26,  56, -59,  30,  15, -54,  64, -32},
{ -13,  33, -50,  61, -63,  55, -40,  17},
          },
//kltRow8x8[8][8][8]
{
{  10,  21,  34,  44,  53,  58,  59,  55},
{  33,  54,  63,  50,  19, -17, -47, -56},
{ -54, -59, -21,  34,  61,  34, -20, -56},
{  63,  28, -41, -51,  17,  62,  15, -54},
{ -66,  20,  57, -29, -50,  37,  42, -44},
{  52, -58,   3,  54, -47, -14,  60, -37},
{ -33,  59, -53,  19,  27, -59,  60, -28},
{  17, -38,  55, -64,  62, -51,  32, -11},
}
};

const short kltCol8x8[9][8][8] = {
//kltCol8x8[0][8][8]
{
{  17,  28,  37,  45,  51,  55,  57,  56},
{  36,  54,  58,  45,  17, -18, -48, -62},
{ -49, -54, -22,  32,  64,  44, -14, -58},
{  63,  29, -40, -57,   7,  60,  24, -50},
{ -62,  15,  58, -17, -55,  29,  52, -45},
{  53, -54, -12,  61, -38, -27,  61, -32},
{ -36,  62, -50,   8,  38, -62,  54, -21},
{  20, -44,  59, -63,  59, -47,  29, -10},
  },
//kltCol8x8[1][8][8]
  {
{  27,  36,  41,  46,  49,  51,  53,  52},
{ -50, -57, -47, -30,  -8,  20,  50,  66},
{ -56, -48,  -3,  47,  64,  36, -17, -55},
{  50,  11, -51, -54,  14,  67,  27, -53},
{ -59,  24,  61, -21, -55,  27,  51, -43},
{  48, -53, -10,  60, -38, -30,  64, -33},
{ -36,  64, -50,   7,  40, -62,  51, -20},
{ -20,  43, -58,  64, -60,  46, -28,  10},
    },
//kltCol8x8[2][8][8]            DC
    {
{  25,  35,  42,  46,  50,  52,  53,  52},
{ -58, -61, -48, -27,   0,  26,  48,  57},
{ -60, -39,  12,  51,  58,  29, -25, -60},
{  57,   1, -58, -41,  29,  64,  16, -53},
{ -51,  37,  50, -38, -50,  39,  48, -46},
{  42, -61,   6,  56, -46, -20,  63, -35},
{ -30,  60, -57,  21,  31, -60,  55, -24},
{ -15,  36, -55,  65, -63,  50, -31,  12},
    },
//kltCol8x8[3][8][8]
    {
{   9,  21,  34,  45,  54,  59,  58,  53},
{  24,  49,  63,  54,  22, -18, -48, -58},
{ -42, -62, -34,  27,  63,  38, -19, -56},
{  58,  44, -30, -56,   9,  61,  17, -55},
{ -66,  -1,  59, -14, -54,  32,  47, -47},
{  60, -44, -18,  59, -37, -24,  62, -35},
{ -46,  62, -46,   7,  35, -60,  55, -23},
{  24, -44,  57, -62,  59, -48,  31, -11},
      },

//kltCol8x8[4][8][8]
{
{   9,  20,  33,  45,  55,  60,  59,  51},
{  23,  48,  62,  54,  24, -17, -50, -60},
{ -41, -62, -37,  21,  60,  42, -17, -58},
{  56,  46, -27, -59,   2,  59,  20, -54},
{ -64,  -5,  61,  -9, -57,  28,  48, -45},
{  61, -40, -24,  61, -34, -27,  61, -35},
{ -49,  63, -42,   3,  37, -59,  53, -23},
{  27, -46,  57, -61,  57, -47,  31, -12},
  },
//kltCol8x8[5][8][8]
  {
{   9,  19,  30,  42,  52,  59,  61,  57},
{  24,  46,  60,  56,  31,  -8, -45, -63},
{ -45, -61, -39,  14,  58,  48,  -8, -57},
{  61,  43, -26, -58,  -6,  58,  28, -51},
{ -65,   1,  59,  -5, -59,  19,  55, -43},
{  58, -46, -24,  63, -28, -34,  60, -30},
{ -42,  64, -44,  -2,  44, -62,  50, -19},
{  25, -48,  61, -63,  56, -43,  26,  -9},
    },
//kltCol8x8[6][8][8]
    {
{  10,  21,  33,  44,  53,  59,  60,  54},
{  23,  47,  60,  54,  27, -12, -48, -63},
{ -39, -60, -39,  17,  59,  48,  -9, -59},
{  56,  49, -24, -62,  -5,  58,  25, -50},
{ -64,  -6,  63,  -7, -58,  23,  51, -42},
{  63, -42, -23,  60, -32, -29,  60, -33},
{ -49,  64, -46,   6,  35, -58,  52, -22},
{  24, -42,  55, -62,  60, -49,  31, -12},
    },
//kltCol8x8[7][8][8]
    {
{  10,  22,  34,  44,  53,  58,  59,  56},
{  28,  52,  62,  51,  22, -15, -47, -60},
{ -47, -59, -29,  27,  62,  41, -14, -58},
{  62,  36, -34, -55,   5,  60,  23, -53},
{ -67,  10,  58, -17, -54,  27,  51, -44},
{  55, -50, -13,  62, -39, -26,  60, -33},
{ -38,  62, -50,   8,  37, -62,  53, -21},
{  23, -44,  58, -62,  59, -48,  30, -11},
      },
//kltCol8x8[8][8][8]
{
{  32,  40,  44,  45,  44,  47,  53,  53},
{ -69, -67, -39,  -9,  13,  30,  45,  50},
{ -60, -21,  42,  68,  47,   6, -34, -51},
{  53, -21, -66,  -5,  62,  51,  -8, -47},
{ -45,  48,  24, -61, -13,  65,  27, -51},
{  39, -63,  31,  29, -62,  19,  54, -46},
{ -24,  52, -63,  49,  -9, -39,  63, -34},
{  -8,  22, -36,  51, -64,  66, -53,  23},
}
};
// 4x4 KLTs
const short kltRow4x4[9][4][4] = {
//kltRow4x4[0][4][4]
  {
{  40,  64,  78,  68},
{ -82, -60,  31,  70},
{  76, -52, -56,  69},
{  47, -77,  79, -46},
  },
//kltRow4x4[1][4][4]
{
{  30,  55,  74,  84},
{ -67, -76,  -4,  78},
{ -83,  16,  81, -52},
{ -64,  85, -66,  25},
    },
//kltRow4x4[2][4][4]
{
{  44,  64,  73,  70},
{ -81, -58,  28,  75},
{  75, -51, -63,  65},
{  47, -79,  80, -40},
    },
//kltRow4x4[3][4][4]
{
{  50,  69,  72,  62},
{ -89, -43,  42,  70},
{  69, -68, -43,  71},
{  36, -71,  87, -50},
      },
//kltRow4x4[4][4][4]
{
{  26,  59,  81,  76},
{ -63, -80,   6,  77},
{  86, -15, -72,  60},
{  66, -79,  68, -33},
      },
//kltRow4x4[5][4][4]
{
{  22,  52,  82,  81},
{ -61, -86,   0,  73},
{  88, -13, -72,  58},
{  67, -78,  67, -36},
        },
//kltRow4x4[6][4][4]
{
{  26,  57,  78,  79},
{ -65, -77,  -1,  79},
{ -87,  15,  75, -56},
{ -63,  83, -68,  29},
        },
//kltRow4x4[7][4][4]
{
{  50,  70,  72,  62},
{ -87, -45,  45,  69},
{  69, -67, -44,  72},
{  39, -71,  85, -51},
          },
//kltRow4x4[8][4][4]
{
{  29,  59,  77,  78},
{ -71, -73,   3,  77},
{ -85,  25,  72, -58},
{ -57,  84, -72,  30},
          },
};
const short kltCol4x4[9][4][4] = {
//kltCol4x4[0][4][4]
{
{  32,  56,  74,  82},
{ -69, -74,  -3,  79},
{ -81,  17,  82, -53},
{ -63,  87, -65,  24},
  },
//kltCol4x4[1][4][4]
{
{  29,  55,  80,  78},
{ -78, -73,  11,  69},
{  81, -35, -67,  64},
{  55, -82,  73, -37},
    },
//kltCol4x4[2][4][4]
{
{  52,  65,  70,  68},
{ -84, -48,  33,  77},
{  70, -59, -61,  65},
{  41, -80,  82, -40},
    },
//kltCol4x4[3][4][4]
{
{  21,  54,  80,  81},
{ -64, -82,  -2,  74},
{  88, -15, -71,  57},
{  64, -80,  70, -32},
      },
//kltCol4x4[4][4][4]
{
{  24,  56,  81,  78},
{ -62, -81,   0,  77},
{  87, -12, -73,  58},
{  66, -81,  67, -32},
      },
//kltCol4x4[5][4][4]
{
{  25,  56,  79,  80},
{ -64, -79,  -3,  78},
{ -86,  11,  76, -56},
{ -65,  84, -66,  28},
        },
//kltCol4x4[6][4][4]
{
{  10,  32,  73, 100},
{ -50, -92, -38,  63},
{  89,  15, -80,  44},
{  77, -82,  57, -24},
        },
//kltCol4x4[7][4][4]
{
{  26,  57,  78,  80},
{ -64, -78,  -2,  79},
{ -85,  13,  77, -55},
{ -65,  83, -67,  28},
          },
//kltCol4x4[8][4][4]
{
{  40,  38,  60,  99},
{ -97, -62,  15,  54},
{  65, -80, -63,  43},
{  34, -69,  93, -44},
          },
};



//ADAPTIVE_SCAN
  UInt *scanOrder4x4[9];
  UInt *scanOrder4x4X[9];
  UInt *scanOrder4x4Y[9];
  UInt *scanOrder8x8[9];
  UInt *scanOrder8x8X[9];
  UInt *scanOrder8x8Y[9];
  UInt *scanStats4x4[9];
  UInt *scanStats8x8[9];
  int  update4x4[9];
  int  update8x8[9];
  int  update4x4Count[9];
  int  update8x8Count[9];
  int  update4x4Thres[9];
  int  update8x8Thres[9];
  

  UInt *scanOrder16x16[NUM_SCANS_16x16];
  UInt *scanOrder16x16X[NUM_SCANS_16x16];
  UInt *scanOrder16x16Y[NUM_SCANS_16x16];
  UInt *scanStats16x16[NUM_SCANS_16x16];

  UInt *scanOrder32x32[NUM_SCANS_32x32];
  UInt *scanOrder32x32X[NUM_SCANS_32x32];
  UInt *scanOrder32x32Y[NUM_SCANS_32x32];
  UInt *scanStats32x32[NUM_SCANS_32x32];

  UInt *scanOrder64x64[NUM_SCANS_64x64];
  UInt *scanOrder64x64X[NUM_SCANS_64x64];
  UInt *scanOrder64x64Y[NUM_SCANS_64x64];
  UInt *scanStats64x64[NUM_SCANS_64x64];

  Bool g_bUpdateStats;

#if SCAN_LUT_FIX
const char LUT16x16[5][34] = 
{
  {1, 0, 2, 2, 3, 2, 0, 7, 7, 2, 1, 2, 2, 2, 0, 0, 0, 2, 3, 3, 1, 1, 3, 4, 2, 6, 7, 0, 0, 0, 7, 7, 6, 3 },
  {1, 0, 8, 2, 3, 2, 7, 7, 7, 4, 3, 2, 2, 7, 0, 0, 7, 2, 4, 3, 1, 8, 3, 2, 2, 6, 7, 0, 0, 0, 7, 6, 6, 3 },
  {2, 0, 8, 2, 3, 6, 7, 7, 7, 4, 3, 2, 8, 6, 0, 0, 6, 2, 3, 3, 1, 1, 2, 4, 2, 6, 7, 7, 0, 0, 7, 7, 6, 3 },
  {1, 0, 8, 2, 4, 2, 7, 7, 7, 4, 2, 2, 2, 6, 0, 0, 6, 4, 4, 3, 1, 1, 3, 4, 2, 6, 7, 7, 0, 0, 7, 7, 6, 3 },
  {2, 0, 8, 2, 4, 2, 7, 7, 7, 4, 3, 2, 4, 6, 0, 0, 6, 2, 4, 3, 1, 1, 3, 2, 6, 6, 7, 7, 0, 0, 7, 7, 6, 3 },
};

const char LUT32x32[5][34] = 
{
  {1, 0, 2, 2, 3, 2, 0, 7, 7, 2, 1, 2, 2, 2, 0, 0, 0, 2, 3, 3, 1, 1, 3, 4, 2, 6, 7, 0, 0, 0, 7, 7, 6, 3 },
  {1, 0, 8, 2, 3, 2, 7, 7, 7, 4, 3, 2, 2, 7, 0, 0, 7, 2, 4, 3, 1, 8, 3, 2, 2, 6, 7, 0, 0, 0, 7, 6, 6, 3 },
  {2, 0, 8, 2, 3, 6, 7, 7, 7, 4, 3, 2, 8, 6, 0, 0, 6, 2, 3, 3, 1, 1, 2, 4, 2, 6, 7, 7, 0, 0, 7, 7, 6, 3 },
  {1, 0, 8, 2, 4, 2, 7, 7, 7, 4, 2, 2, 2, 6, 0, 0, 6, 4, 4, 3, 1, 1, 3, 4, 2, 6, 7, 7, 0, 0, 7, 7, 6, 3 },
  {2, 0, 8, 2, 4, 2, 7, 7, 7, 4, 3, 2, 4, 6, 0, 0, 6, 2, 4, 3, 1, 1, 3, 2, 6, 6, 7, 7, 0, 0, 7, 7, 6, 3 },
};

const char LUT64x64[5][5] = 
{
	{1, 2, 2, 2, 3, },
	{2, 2, 2, 7, 2, },
	{2, 2, 2, 7, 2, },
	{1, 2, 2, 5, 2, },
	{2, 2, 2, 5, 2}
};
#else
const char LUT16x16[5][33] = 
{
	{1, 0, 2, 2, 5, 5, 4, 7, 4, 7, 1, 4, 6, 7, 6, 1, 5, 7, 2, 3, 4, 5, 6, 6, 0, 0, 7, 6, 4, 3, 2, 6, 8, },
	{1, 0, 2, 2, 5, 5, 4, 7, 4, 7, 3, 4, 6, 7, 6, 1, 5, 7, 2, 3, 4, 5, 5, 6, 0, 0, 7, 6, 4, 3, 2, 6, 8, },
	{1, 0, 2, 2, 5, 5, 4, 7, 3, 7, 3, 4, 6, 7, 6, 1, 5, 7, 2, 3, 4, 4, 5, 6, 0, 0, 7, 6, 4, 3, 2, 6, 8, },
	{1, 0, 2, 2, 5, 5, 3, 7, 3, 7, 2, 4, 6, 7, 6, 1, 5, 7, 2, 3, 4, 4, 5, 6, 0, 0, 7, 6, 4, 3, 2, 6, 8, },
	{1, 0, 2, 2, 5, 5, 4, 7, 3, 7, 1, 4, 6, 7, 6, 1, 5, 7, 2, 3, 3, 4, 5, 6, 0, 0, 7, 6, 4, 3, 2, 6, 8, },
};

const char LUT32x32[5][33] = 
{
	{1, 0, 2, 2, 5, 5, 4, 2, 3, 7, 1, 4, 6, 7, 6, 1, 6, 0, 1, 3, 4, 2, 6, 6, 0, 2, 2, 6, 4, 3, 2, 6, 2, },
	{1, 0, 2, 2, 5, 5, 3, 2, 4, 7, 2, 4, 8, 7, 6, 1, 5, 7, 1, 3, 3, 2, 2, 6, 6, 0, 2, 2, 4, 3, 2, 6, 2, },
	{1, 0, 2, 2, 2, 5, 3, 2, 3, 7, 1, 3, 6, 2, 6, 1, 5, 7, 1, 2, 3, 2, 8, 6, 0, 0, 2, 6, 4, 1, 2, 6, 8, },
	{1, 0, 2, 2, 2, 5, 3, 2, 3, 7, 2, 2, 6, 2, 6, 1, 5, 0, 1, 3, 3, 2, 6, 6, 0, 2, 2, 6, 4, 2, 2, 6, 8, },
	{1, 0, 2, 2, 5, 5, 3, 2, 3, 2, 1, 4, 8, 2, 6, 1, 5, 0, 1, 3, 3, 2, 5, 6, 0, 2, 7, 6, 4, 3, 2, 6, 8, },
};

const char LUT64x64[5][5] = 
{
	{1, 2, 2, 2, 2, },
	{2, 2, 2, 2, 5, },
	{2, 2, 2, 2, 5, },
	{1, 2, 2, 2, 2, },
	{2, 2, 2, 2, 2}
};
#endif

static const int SCANSTATS64x64[9][4096] = 
{
	{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
90, 56, 38, 37, 30, 27, 25, 23, 20, 21, 22, 23, 17, 21, 19, 21, 19, 19, 16, 18, 16, 19, 15, 14, 14, 16, 17, 13, 14, 16, 16, 18, 12, 11, 13, 15, 13, 9, 10, 13, 11, 10, 8, 8, 10, 10, 8, 7, 5, 9, 6, 5, 5, 5, 4, 5, 3, 3, 3, 5, 3, 4, 3, 9, 
69, 25, 16, 14, 13, 12, 11, 10, 9, 8, 12, 9, 7, 9, 9, 12, 11, 10, 9, 9, 11, 12, 8, 7, 9, 9, 9, 8, 8, 9, 10, 11, 7, 6, 9, 9, 8, 6, 6, 7, 5, 7, 4, 3, 6, 6, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 
28, 17, 12, 11, 9, 8, 6, 7, 6, 6, 4, 4, 6, 4, 4, 5, 5, 4, 3, 3, 3, 4, 3, 3, 4, 3, 3, 3, 3, 3, 4, 5, 3, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
26, 15, 12, 10, 8, 8, 6, 6, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 
17, 13, 11, 9, 9, 7, 6, 5, 4, 4, 4, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
15, 12, 10, 8, 7, 6, 6, 5, 4, 4, 3, 3, 2, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
12, 10, 9, 7, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
12, 10, 9, 7, 6, 5, 4, 4, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
11, 9, 7, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
11, 9, 7, 6, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
9, 7, 6, 5, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
8, 7, 6, 4, 4, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
7, 6, 5, 4, 4, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
7, 6, 5, 4, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
7, 5, 4, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
6, 4, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 4, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 3, 3, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 1, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 1, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
90, 78, 67, 64, 59, 56, 53, 51, 49, 46, 45, 41, 38, 36, 34, 31, 30, 27, 26, 24, 23, 22, 20, 19, 19, 18, 17, 16, 16, 15, 15, 15, 18, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 5, 5, 4, 4, 3, 3, 3, 3, 2, 3, 2, 4, 1, 8, 2, 25, 
77, 66, 61, 58, 54, 52, 49, 47, 44, 43, 41, 39, 35, 34, 31, 29, 27, 25, 23, 22, 21, 20, 19, 18, 17, 16, 16, 15, 15, 14, 14, 13, 12, 11, 11, 10, 10, 9, 9, 9, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 2, 
67, 61, 57, 55, 52, 49, 46, 43, 42, 40, 38, 36, 34, 31, 28, 27, 24, 22, 21, 20, 19, 18, 17, 16, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 10, 9, 9, 8, 8, 8, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 2, 
64, 58, 56, 53, 50, 47, 44, 42, 40, 37, 35, 33, 31, 28, 26, 24, 22, 20, 19, 18, 17, 16, 15, 15, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 
60, 56, 54, 51, 48, 45, 43, 41, 39, 37, 34, 31, 29, 27, 25, 23, 21, 20, 19, 18, 17, 16, 15, 14, 13, 13, 12, 11, 11, 11, 10, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 
57, 53, 51, 49, 46, 44, 40, 39, 37, 35, 33, 30, 28, 26, 24, 21, 20, 19, 18, 17, 16, 15, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 
54, 50, 48, 47, 45, 43, 39, 39, 35, 33, 32, 29, 26, 24, 23, 21, 19, 18, 17, 16, 15, 14, 13, 13, 12, 12, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 
51, 48, 46, 45, 43, 41, 38, 36, 34, 32, 31, 28, 25, 23, 21, 19, 18, 17, 16, 15, 15, 14, 13, 13, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 
48, 47, 44, 43, 41, 39, 37, 34, 32, 30, 28, 26, 23, 22, 20, 19, 17, 17, 16, 15, 14, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 
47, 45, 43, 41, 39, 38, 34, 33, 30, 29, 26, 24, 22, 21, 19, 18, 17, 16, 15, 14, 14, 13, 12, 12, 11, 10, 10, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 
45, 42, 41, 39, 38, 35, 33, 30, 28, 26, 24, 22, 21, 19, 18, 17, 16, 15, 14, 14, 13, 13, 12, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 
42, 40, 39, 36, 35, 32, 31, 28, 26, 24, 22, 21, 19, 18, 17, 16, 15, 15, 14, 13, 13, 12, 11, 11, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 
38, 36, 34, 33, 32, 30, 27, 25, 24, 22, 20, 19, 18, 17, 16, 15, 15, 14, 13, 13, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 
34, 33, 32, 30, 28, 27, 25, 24, 22, 20, 19, 18, 17, 16, 15, 14, 14, 13, 13, 12, 11, 11, 10, 10, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 
32, 30, 29, 27, 27, 25, 23, 22, 20, 19, 18, 17, 16, 15, 15, 14, 13, 13, 12, 11, 11, 10, 10, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 
29, 27, 27, 25, 24, 23, 22, 21, 19, 18, 17, 16, 15, 14, 14, 13, 13, 12, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 
27, 25, 25, 24, 23, 22, 20, 19, 18, 17, 16, 16, 15, 14, 13, 13, 12, 12, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 
26, 24, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 14, 13, 12, 12, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 
25, 24, 23, 22, 21, 20, 19, 19, 17, 16, 15, 15, 14, 13, 12, 12, 11, 11, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 
24, 23, 22, 21, 20, 19, 19, 18, 17, 16, 14, 14, 13, 13, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 
22, 21, 21, 20, 20, 19, 18, 16, 16, 15, 14, 13, 13, 12, 11, 11, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
21, 20, 19, 19, 18, 17, 16, 16, 15, 14, 13, 12, 12, 11, 11, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
20, 19, 18, 17, 17, 16, 15, 14, 13, 13, 12, 12, 11, 10, 10, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
19, 18, 17, 16, 16, 15, 14, 14, 13, 12, 11, 11, 10, 10, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
18, 17, 16, 15, 15, 14, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
17, 16, 15, 14, 14, 13, 13, 12, 11, 11, 10, 10, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
16, 15, 15, 14, 13, 13, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
16, 15, 14, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
15, 14, 13, 13, 12, 11, 11, 10, 10, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
14, 13, 12, 12, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
14, 13, 12, 11, 11, 10, 10, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
13, 12, 11, 11, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
12, 11, 11, 10, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
12, 11, 10, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
11, 10, 10, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
10, 10, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
10, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
10, 9, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
9, 8, 8, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
8, 7, 7, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
7, 6, 6, 6, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
6, 6, 6, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
6, 6, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
6, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 5, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
11, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
90, 52, 38, 29, 23, 20, 16, 14, 12, 9, 10, 8, 6, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 5, 1, 18, 
52, 40, 34, 24, 21, 16, 14, 12, 12, 9, 7, 7, 7, 5, 5, 4, 3, 3, 2, 3, 3, 2, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 
41, 35, 32, 25, 21, 16, 14, 13, 11, 9, 8, 8, 7, 5, 5, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
34, 32, 28, 26, 22, 17, 14, 12, 10, 9, 8, 7, 6, 4, 4, 4, 4, 2, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
30, 26, 26, 22, 18, 17, 13, 11, 9, 10, 7, 7, 6, 5, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
24, 24, 21, 19, 16, 16, 13, 11, 9, 8, 6, 5, 6, 5, 4, 3, 3, 3, 2, 2, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
22, 19, 20, 17, 15, 15, 15, 12, 10, 8, 6, 7, 4, 4, 4, 3, 3, 2, 3, 2, 2, 2, 2, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
19, 18, 17, 17, 14, 14, 11, 10, 9, 7, 6, 6, 4, 4, 3, 3, 3, 2, 3, 3, 2, 2, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
16, 13, 15, 14, 12, 11, 10, 9, 9, 6, 6, 4, 4, 4, 3, 3, 3, 2, 2, 3, 2, 2, 2, 1, 1, 2, 2, 2, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
14, 13, 11, 11, 11, 10, 8, 8, 7, 8, 5, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 2, 1, 1, 2, 2, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
12, 11, 11, 10, 9, 8, 7, 7, 5, 5, 6, 4, 4, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1, 2, 1, 1, 2, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
11, 10, 9, 9, 7, 7, 7, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 2, 2, 2, 2, 1, 2, 1, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
9, 9, 8, 7, 7, 6, 7, 5, 5, 4, 4, 4, 4, 5, 2, 2, 2, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
8, 7, 7, 6, 6, 6, 5, 4, 4, 3, 4, 3, 3, 4, 4, 2, 3, 3, 3, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
7, 5, 6, 6, 5, 6, 4, 5, 4, 3, 3, 4, 3, 3, 3, 3, 3, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
7, 5, 5, 5, 4, 5, 4, 3, 3, 3, 4, 3, 3, 3, 4, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
6, 4, 4, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 3, 2, 3, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 4, 4, 3, 4, 4, 4, 3, 3, 4, 2, 2, 3, 2, 2, 3, 2, 3, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 4, 4, 3, 4, 4, 3, 3, 3, 3, 2, 3, 2, 2, 2, 3, 2, 2, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 3, 2, 3, 2, 2, 2, 1, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 1, 2, 2, 2, 2, 1, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 3, 4, 3, 3, 2, 3, 3, 2, 2, 2, 2, 2, 2, 1, 2, 2, 1, 2, 1, 1, 2, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 2, 3, 3, 4, 3, 2, 2, 3, 2, 2, 1, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 2, 3, 3, 2, 2, 3, 2, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 2, 3, 2, 2, 2, 1, 2, 1, 2, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 2, 3, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 3, 2, 3, 2, 2, 1, 2, 2, 2, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 2, 2, 2, 2, 2, 1, 2, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 2, 2, 2, 1, 2, 1, 1, 1, 2, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
3, 2, 2, 2, 1, 1, 1, 1, 2, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 1, 3, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
2, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

};

static const int SCANSTATS32x32[9][1024] =
{
	{90, 50, 30, 20, 11, 9, 6, 5, 4, 4, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 8, 62, 33, 22, 14, 9, 6, 5, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 51, 26, 17, 12, 7, 5, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 39, 22, 15, 10, 6, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 31, 17, 12, 7, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 14, 10, 6, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 23, 12, 8, 5, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 10, 7, 4, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 14, 8, 6, 3, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 7, 5, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11, 6, 4, 3, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 5, 4, 3, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 5, 4, 3, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 5, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 4, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 4, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 4, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
	{90, 54, 41, 33, 24, 21, 17, 16, 13, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 7, 6, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 4, 49, 27, 21, 15, 12, 9, 8, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 28, 18, 15, 10, 7, 6, 5, 3, 3, 2, 3, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 20, 14, 11, 8, 6, 4, 3, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 13, 9, 7, 5, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11, 7, 6, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 6, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 3, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 3, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
	{90, 71, 54, 43, 32, 28, 23, 19, 16, 14, 12, 11, 10, 9, 8, 8, 8, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 3, 1, 4, 1, 14, 73, 56, 46, 37, 28, 24, 20, 17, 14, 12, 11, 10, 9, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 1, 1, 1, 59, 48, 40, 33, 25, 22, 18, 15, 13, 11, 10, 9, 8, 8, 7, 6, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 1, 0, 48, 41, 35, 29, 23, 20, 17, 14, 12, 10, 10, 9, 8, 7, 7, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 38, 34, 29, 25, 19, 16, 14, 12, 11, 10, 9, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 33, 29, 26, 21, 16, 14, 12, 11, 10, 9, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 28, 25, 22, 19, 14, 13, 11, 10, 9, 8, 7, 7, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 23, 21, 19, 16, 13, 11, 10, 9, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 20, 17, 16, 14, 12, 10, 9, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 17, 16, 14, 12, 11, 9, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 16, 14, 13, 11, 10, 9, 8, 7, 6, 6, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 14, 12, 11, 10, 9, 8, 7, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 13, 11, 10, 9, 8, 7, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 11, 10, 9, 8, 7, 6, 6, 5, 5, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 11, 9, 8, 7, 6, 6, 5, 5, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 10, 8, 7, 7, 6, 5, 5, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 9, 8, 7, 6, 5, 5, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 8, 7, 6, 5, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 7, 6, 6, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 6, 6, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 6, 5, 5, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 5, 5, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 5, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 4, 4, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
	{90, 75, 62, 47, 29, 23, 15, 11, 8, 6, 4, 3, 3, 3, 3, 3, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1, 0, 1, 0, 2, 1, 6, 65, 53, 45, 38, 29, 22, 17, 12, 8, 6, 5, 6, 4, 5, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 41, 34, 32, 26, 23, 19, 17, 13, 11, 9, 8, 6, 5, 4, 4, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 28, 24, 21, 18, 14, 15, 12, 11, 10, 10, 8, 9, 6, 6, 4, 3, 3, 3, 2, 3, 2, 2, 1, 2, 1, 1, 1, 0, 0, 0, 0, 0, 18, 15, 13, 11, 8, 7, 6, 6, 6, 6, 8, 5, 8, 6, 7, 4, 3, 3, 2, 2, 3, 1, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 13, 11, 10, 7, 6, 7, 4, 5, 4, 4, 6, 6, 4, 6, 6, 5, 4, 4, 5, 4, 3, 2, 1, 2, 1, 1, 0, 0, 0, 0, 0, 0, 10, 8, 7, 6, 5, 4, 3, 3, 3, 4, 3, 3, 3, 4, 5, 3, 4, 4, 4, 4, 4, 2, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 7, 6, 6, 4, 3, 3, 3, 2, 2, 2, 2, 3, 2, 4, 2, 2, 3, 3, 3, 5, 4, 5, 4, 3, 3, 2, 1, 1, 0, 1, 0, 0, 6, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 3, 2, 2, 2, 2, 2, 4, 2, 4, 3, 4, 3, 2, 1, 2, 1, 0, 0, 4, 4, 4, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 2, 2, 1, 1, 1, 2, 1, 3, 3, 4, 2, 3, 1, 1, 4, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 2, 2, 1, 1, 2, 1, 3, 2, 3, 1, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 2, 1, 2, 1, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 4, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 2, 1, 2, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
	{90, 71, 53, 32, 20, 12, 8, 5, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 5, 64, 52, 44, 32, 20, 13, 8, 5, 4, 2, 2, 3, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 39, 36, 36, 30, 22, 18, 8, 7, 4, 3, 2, 1, 2, 1, 2, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 26, 24, 24, 22, 21, 19, 15, 9, 5, 4, 3, 3, 2, 1, 1, 1, 2, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 14, 12, 12, 11, 12, 15, 12, 10, 6, 5, 4, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 8, 9, 7, 7, 7, 11, 11, 10, 7, 4, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 8, 7, 6, 5, 5, 4, 4, 7, 9, 8, 7, 6, 3, 3, 3, 2, 2, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 6, 5, 5, 5, 4, 2, 3, 4, 4, 7, 11, 8, 5, 3, 3, 4, 2, 2, 2, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 3, 3, 3, 3, 2, 5, 9, 9, 10, 8, 5, 2, 2, 2, 2, 3, 1, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 2, 4, 2, 2, 2, 2, 2, 3, 8, 10, 10, 7, 3, 4, 2, 2, 2, 3, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 3, 2, 2, 2, 2, 2, 1, 1, 2, 1, 3, 3, 6, 9, 12, 9, 5, 2, 1, 2, 1, 3, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 4, 6, 10, 9, 3, 3, 4, 2, 1, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 5, 6, 9, 3, 5, 1, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 4, 4, 9, 7, 6, 4, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 4, 8, 9, 8, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 2, 2, 3, 7, 8, 7, 8, 7, 1, 2, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 2, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 2, 4, 7, 10, 8, 7, 3, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 2, 3, 5, 9, 9, 5, 2, 1, 0, 0, 0, 0, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 3, 2, 4, 7, 6, 4, 3, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 2, 2, 1, 4, 4, 4, 2, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 2, 1, 1, 3, 6, 3, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 3, 6, 4, 2, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 5, 4, 3, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 4, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
	{90, 63, 44, 25, 15, 11, 8, 5, 5, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 2, 0, 9, 65, 51, 39, 25, 14, 10, 7, 5, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 45, 37, 33, 25, 15, 9, 6, 6, 4, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 27, 27, 23, 16, 10, 6, 4, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 21, 17, 16, 16, 14, 10, 7, 4, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 12, 11, 10, 10, 10, 8, 5, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 9, 9, 7, 7, 9, 9, 7, 4, 3, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 6, 6, 6, 5, 5, 6, 7, 7, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 4, 4, 4, 3, 4, 6, 6, 7, 6, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 3, 4, 3, 4, 4, 5, 6, 4, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 2, 2, 2, 3, 3, 5, 6, 4, 3, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 2, 2, 2, 2, 2, 2, 2, 3, 5, 6, 4, 3, 2, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 1, 1, 1, 2, 1, 2, 3, 5, 3, 3, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 1, 1, 1, 1, 2, 1, 1, 2, 2, 2, 4, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 3, 3, 3, 4, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 3, 2, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 5, 4, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 5, 5, 4, 3, 3, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 2, 3, 3, 3, 3, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 2, 3, 5, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 3, 3, 4, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 2, 1, 1, 1, 4, 4, 3, 2, 2, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 3, 3, 3, 2, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 2, 3, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 2, 3, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 2, 2, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, },
	{90, 63, 38, 25, 14, 11, 8, 6, 5, 4, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 1, 0, 3, 0, 14, 68, 50, 35, 21, 14, 10, 7, 5, 4, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 50, 39, 31, 21, 12, 9, 7, 5, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 34, 32, 27, 20, 12, 8, 6, 5, 3, 3, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 24, 20, 20, 17, 11, 7, 6, 4, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 18, 15, 15, 14, 11, 7, 5, 4, 4, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 13, 11, 11, 10, 8, 6, 4, 3, 3, 2, 2, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 10, 8, 9, 8, 7, 6, 4, 4, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 6, 7, 6, 7, 7, 6, 5, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 6, 5, 4, 5, 6, 6, 5, 4, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 5, 4, 4, 4, 6, 5, 5, 4, 3, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 4, 4, 3, 3, 4, 5, 5, 4, 4, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 4, 3, 2, 3, 3, 4, 5, 4, 3, 3, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 4, 3, 2, 2, 2, 2, 3, 4, 3, 2, 3, 2, 2, 2, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 3, 2, 2, 2, 1, 2, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 4, 3, 2, 2, 2, 2, 1, 1, 2, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 2, 1, 1, 1, 1, 1, 1, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 2, 1, 1, 1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
	{90, 55, 33, 22, 14, 11, 8, 6, 5, 5, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 2, 0, 5, 0, 24, 72, 46, 30, 20, 12, 9, 7, 6, 4, 4, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 59, 40, 27, 16, 11, 8, 6, 5, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 44, 36, 25, 16, 10, 7, 5, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 31, 28, 22, 14, 8, 6, 4, 4, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 24, 20, 13, 7, 5, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 19, 21, 19, 13, 7, 5, 4, 3, 2, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 16, 16, 12, 7, 4, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 12, 13, 10, 6, 4, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 9, 11, 10, 6, 4, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 7, 9, 9, 6, 4, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 7, 8, 5, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 6, 7, 5, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 6, 5, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 5, 5, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 4, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 3, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 3, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 2, 3, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
	{90, 71, 54, 35, 24, 18, 15, 12, 9, 8, 7, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 2, 1, 2, 1, 2, 0, 5, 0, 23, 73, 60, 47, 33, 21, 17, 13, 10, 8, 7, 6, 5, 5, 4, 4, 3, 2, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 60, 52, 44, 31, 20, 15, 12, 10, 7, 7, 5, 5, 5, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 49, 45, 40, 32, 21, 15, 11, 9, 7, 6, 6, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 37, 34, 31, 26, 18, 14, 10, 8, 7, 5, 5, 5, 3, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 34, 29, 25, 22, 18, 15, 10, 8, 6, 6, 5, 4, 4, 4, 3, 3, 2, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 29, 23, 21, 18, 15, 14, 12, 9, 6, 6, 5, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 25, 20, 18, 14, 12, 12, 12, 8, 6, 5, 4, 4, 3, 3, 2, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 20, 16, 14, 12, 10, 10, 10, 8, 7, 5, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 17, 15, 12, 10, 9, 8, 9, 8, 8, 7, 5, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 15, 12, 10, 9, 8, 6, 7, 7, 7, 7, 5, 4, 3, 3, 2, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 14, 11, 9, 8, 6, 6, 6, 6, 6, 6, 5, 4, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 12, 9, 9, 7, 6, 4, 5, 5, 5, 5, 4, 5, 4, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 12, 10, 8, 6, 5, 4, 4, 4, 4, 4, 4, 5, 5, 4, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 12, 10, 8, 6, 4, 4, 3, 3, 4, 4, 4, 4, 5, 4, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 10, 9, 6, 5, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 8, 6, 5, 4, 3, 3, 3, 3, 3, 2, 3, 3, 3, 3, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 7, 6, 5, 4, 3, 3, 3, 2, 2, 2, 2, 2, 3, 2, 3, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 7, 5, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 3, 3, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 6, 5, 4, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 6, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 4, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 5, 4, 3, 3, 3, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1, 2, 1, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 5, 3, 3, 2, 2, 2, 2, 1, 2, 2, 1, 2, 1, 2, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 5, 4, 2, 2, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 3, 2, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 3, 2, 2, 2, 1, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

static const int SCANSTATS16x16[9][256] = 
{
	{90, 45, 22, 14, 8, 6, 5, 4, 3, 3, 2, 2, 1, 1, 0, 3, 67, 33, 16, 10, 6, 4, 3, 3, 2, 2, 1, 1, 1, 0, 0, 0, 52, 23, 10, 7, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 44, 19, 8, 5, 4, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 36, 14, 6, 4, 3, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 31, 12, 5, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 30, 11, 5, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 27, 9, 4, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 24, 9, 4, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 20, 8, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 18, 7, 3, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 15, 6, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 14, 5, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 12, 4, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 10, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 9, 3, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, },
	{90, 58, 39, 32, 25, 24, 21, 19, 17, 15, 13, 11, 8, 6, 4, 5, 42, 26, 17, 13, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 0, 20, 12, 7, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 1, 0, 0, 14, 9, 5, 4, 3, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 9, 5, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 7, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 5, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 4, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 4, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
	{90, 67, 41, 31, 20, 16, 13, 11, 10, 8, 6, 5, 3, 3, 1, 10, 67, 48, 31, 23, 16, 12, 10, 8, 7, 6, 5, 3, 2, 1, 1, 1, 43, 32, 21, 16, 13, 10, 9, 7, 6, 5, 4, 3, 2, 1, 1, 0, 33, 26, 17, 13, 11, 9, 7, 6, 5, 4, 3, 2, 2, 1, 1, 0, 22, 17, 14, 11, 9, 8, 6, 5, 4, 4, 3, 2, 2, 1, 1, 0, 18, 14, 11, 9, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 0, 0, 15, 11, 9, 7, 6, 5, 5, 4, 3, 3, 2, 2, 1, 1, 0, 0, 12, 9, 7, 6, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 0, 0, 10, 8, 6, 5, 4, 3, 3, 3, 3, 2, 2, 1, 1, 0, 0, 0, 8, 6, 5, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 0, 0, 0, 7, 5, 4, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 0, 0, 0, 5, 4, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 5, 4, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 4, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 4, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, },
	{90, 76, 55, 44, 29, 22, 16, 12, 8, 6, 4, 3, 2, 2, 1, 7, 57, 49, 37, 32, 25, 22, 18, 14, 11, 8, 5, 4, 2, 1, 1, 0, 28, 23, 17, 15, 15, 15, 14, 12, 11, 9, 6, 4, 2, 1, 1, 0, 20, 16, 12, 10, 9, 9, 8, 9, 8, 8, 6, 4, 3, 2, 1, 0, 12, 9, 8, 7, 6, 5, 5, 5, 5, 5, 4, 4, 3, 2, 1, 1, 10, 8, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 7, 6, 5, 4, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 6, 5, 4, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 5, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 6, 6, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 4, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, },
	{90, 70, 44, 30, 16, 11, 8, 6, 5, 4, 3, 2, 1, 1, 0, 5, 61, 51, 38, 28, 17, 11, 8, 6, 4, 3, 3, 2, 1, 1, 0, 0, 32, 28, 23, 21, 18, 13, 9, 6, 5, 4, 3, 2, 1, 1, 0, 0, 22, 19, 15, 15, 15, 14, 11, 8, 6, 4, 3, 2, 1, 1, 0, 0, 12, 10, 10, 9, 9, 10, 10, 9, 7, 5, 3, 2, 1, 1, 0, 0, 9, 8, 7, 6, 5, 6, 7, 7, 7, 5, 4, 3, 2, 1, 0, 0, 7, 6, 5, 4, 4, 4, 4, 5, 5, 5, 4, 3, 2, 1, 1, 0, 5, 4, 4, 3, 3, 3, 2, 3, 3, 4, 4, 3, 3, 1, 1, 0, 4, 4, 3, 3, 2, 2, 2, 2, 2, 2, 3, 3, 2, 2, 1, 0, 4, 3, 2, 2, 2, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 0, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, },
	{90, 65, 37, 24, 14, 10, 7, 6, 5, 4, 3, 2, 1, 2, 0, 7, 65, 52, 34, 22, 13, 9, 6, 5, 4, 3, 2, 2, 1, 1, 0, 0, 39, 34, 26, 20, 13, 9, 6, 4, 4, 3, 2, 1, 1, 1, 0, 0, 27, 24, 21, 19, 15, 9, 6, 4, 3, 2, 2, 1, 1, 0, 0, 0, 17, 14, 14, 15, 14, 11, 7, 4, 3, 2, 2, 1, 1, 0, 0, 0, 12, 10, 10, 9, 11, 12, 9, 5, 3, 2, 2, 1, 1, 0, 0, 0, 9, 8, 7, 6, 7, 9, 10, 7, 4, 2, 2, 1, 1, 0, 0, 0, 7, 6, 5, 5, 4, 5, 7, 8, 6, 3, 2, 1, 1, 0, 0, 0, 5, 4, 4, 3, 3, 3, 4, 6, 6, 6, 3, 2, 1, 0, 0, 0, 4, 3, 3, 3, 2, 2, 2, 3, 5, 6, 5, 3, 1, 0, 0, 0, 3, 2, 2, 2, 2, 2, 2, 2, 2, 5, 6, 5, 2, 1, 0, 0, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 4, 4, 3, 1, 0, 0, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 3, 3, 1, 0, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 2, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, },
	{90, 63, 35, 22, 13, 10, 7, 6, 6, 4, 3, 3, 1, 2, 1, 9, 69, 53, 32, 20, 12, 9, 6, 5, 4, 4, 3, 2, 1, 1, 1, 0, 45, 39, 27, 18, 11, 8, 7, 5, 4, 3, 2, 2, 1, 1, 0, 0, 32, 32, 25, 19, 12, 8, 6, 4, 3, 3, 2, 2, 1, 1, 0, 0, 20, 19, 21, 18, 12, 7, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 14, 13, 15, 16, 13, 8, 5, 4, 3, 2, 2, 1, 1, 0, 0, 0, 11, 10, 10, 12, 12, 9, 5, 3, 3, 2, 2, 1, 1, 0, 0, 0, 8, 7, 7, 8, 10, 9, 6, 3, 3, 2, 1, 1, 1, 0, 0, 0, 6, 6, 5, 5, 6, 7, 6, 4, 3, 2, 1, 1, 1, 0, 0, 0, 5, 4, 4, 4, 4, 5, 5, 4, 3, 2, 1, 1, 1, 0, 0, 0, 4, 3, 3, 3, 3, 4, 4, 4, 4, 2, 2, 1, 1, 0, 0, 0, 3, 3, 2, 2, 2, 2, 4, 4, 4, 3, 2, 1, 1, 0, 0, 0, 3, 2, 2, 2, 2, 2, 3, 3, 3, 3, 2, 1, 1, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 3, 2, 2, 1, 1, 0, 0, 0, 2, 2, 2, 2, 1, 1, 1, 2, 2, 2, 2, 1, 1, 0, 0, 0, 2, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 0, 0, 0, },
	{90, 55, 28, 19, 12, 10, 8, 7, 7, 6, 4, 3, 2, 3, 1, 14, 75, 48, 25, 16, 10, 8, 6, 6, 6, 4, 3, 2, 2, 1, 1, 1, 59, 39, 20, 12, 9, 7, 6, 5, 6, 4, 3, 2, 1, 1, 1, 1, 49, 37, 19, 11, 8, 6, 5, 5, 5, 4, 2, 2, 1, 1, 1, 0, 35, 29, 19, 11, 7, 5, 5, 4, 4, 4, 2, 1, 1, 1, 1, 1, 27, 26, 19, 11, 6, 5, 4, 4, 3, 3, 2, 1, 1, 1, 1, 0, 21, 22, 18, 11, 6, 4, 4, 3, 3, 3, 2, 1, 1, 1, 0, 0, 15, 17, 16, 11, 6, 4, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 11, 14, 13, 10, 6, 4, 3, 2, 2, 2, 2, 1, 1, 0, 0, 0, 8, 10, 11, 9, 5, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 6, 8, 9, 7, 5, 3, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 5, 6, 7, 6, 4, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 4, 4, 6, 6, 4, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 3, 3, 4, 5, 4, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 3, 2, 3, 4, 4, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 3, 2, 2, 3, 3, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, },
	{90, 64, 40, 30, 19, 16, 13, 10, 9, 8, 6, 6, 3, 6, 1, 25, 69, 50, 30, 24, 16, 13, 10, 8, 7, 6, 5, 3, 2, 2, 1, 1, 48, 35, 21, 16, 13, 10, 8, 7, 6, 5, 4, 3, 2, 1, 1, 0, 39, 27, 17, 13, 10, 8, 7, 6, 5, 4, 4, 2, 2, 1, 1, 0, 28, 19, 14, 11, 9, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 23, 17, 12, 9, 8, 6, 6, 4, 4, 4, 3, 2, 2, 1, 1, 0, 20, 14, 10, 7, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 1, 0, 17, 12, 8, 6, 6, 5, 4, 4, 3, 3, 3, 2, 1, 1, 0, 0, 15, 10, 8, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, 0, 12, 8, 6, 5, 4, 3, 3, 3, 3, 3, 2, 2, 1, 0, 0, 0, 10, 7, 5, 4, 4, 3, 3, 3, 3, 3, 2, 1, 1, 0, 0, 0, 9, 6, 4, 3, 4, 3, 3, 2, 3, 2, 2, 1, 1, 0, 0, 0, 8, 6, 4, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 7, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 6, 4, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 6, 4, 3, 3, 3, 2, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0}
};


  static const int SCANSTATS8x8[9][64] = 
{
  //{85, 67, 46, 41, 32, 25, 15,  9, 44, 31, 26, 21, 15, 10,  5,  2, 27, 23, 18, 16, 10,  6,  3,  1, 18, 16, 15, 11,  8,  3,  2,  0, 13, 10, 10,  8,  6,  2,  1,  0,  7,  7,  5,  4,  3,  2,  1,  0,  4,  3,  3,  1,  1,  1,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0, },
  //{79, 32, 15, 10,  7,  4,  2,  1, 56, 23, 13,  9,  6,  3,  2,  0, 38, 19, 13,  8,  4,  3,  1,  0, 34, 18, 11,  7,  4,  2,  1,  0, 36, 15,  9,  6,  4,  2,  1,  0, 26, 10,  6,  5,  3,  2,  1,  0, 17,  6,  3,  2,  2,  1,  0,  0,  6,  1,  1,  0,  0,  0,  0,  0, },
  //{88, 38, 21, 12,  8,  4,  2,  1, 53, 21, 15, 10,  6,  3,  2,  1, 26, 15, 12,  9,  4,  2,  1,  0, 20, 11, 10,  6,  5,  2,  1,  0, 15, 11,  8,  6,  4,  2,  1,  0, 12,  7,  6,  5,  3,  1,  1,  0,  7,  3,  2,  3,  1,  1,  0,  0,  2,  1,  0,  1,  1,  0,  0,  0, },
  //{88, 67, 41, 27, 17, 13,  5,  3, 65, 45, 33, 25, 18, 11,  6,  1, 42, 35, 31, 24, 13, 12,  4,  1, 31, 24, 27, 21, 14,  9,  5,  2, 21, 24, 18, 15, 12,  7,  5,  2, 15, 11, 10, 10,  7,  7,  2,  1,  9,  6,  4,  4,  2,  2,  1,  0,  1,  0,  0,  0,  0,  0,  0,  0, },
  //{85, 69, 42, 29, 20, 12,  6,  2, 66, 53, 44, 29, 18, 10,  5,  1, 43, 46, 41, 29, 16,  7,  5,  1, 30, 30, 32, 26, 20,  9,  4,  1, 25, 21, 22, 23, 19,  9,  4,  1, 15, 14, 12, 15, 10,  7,  2,  1,  8,  6,  5,  5,  5,  3,  1,  0,  1,  1,  1,  0,  1,  0,  0,  0, },
  //{89, 69, 42, 30, 22, 13,  6,  1, 63, 46, 38, 29, 22, 10,  7,  1, 38, 34, 28, 22, 15,  9,  7,  1, 25, 25, 22, 12, 11,  8,  4,  1, 21, 16, 15, 12,  7,  5,  2,  1, 13,  8,  9,  7,  3,  4,  1,  0,  4,  3,  3,  2,  2,  1,  1,  0,  1,  0,  0,  0,  0,  0,  0,  0, },
  //{86, 57, 34, 20, 15,  7,  2,  0, 69, 42, 29, 19, 13,  6,  3,  0, 49, 40, 30, 19, 10,  5,  2,  0, 36, 33, 27, 18,  9,  4,  3,  1, 24, 23, 22, 13, 10,  4,  3,  0, 16, 14, 13, 12,  5,  3,  1,  0,  7,  5,  4,  4,  5,  2,  0,  0,  1,  1,  0,  1,  0,  0,  0,  0, },
  //{88, 70, 51, 37, 26, 15,  7,  2, 61, 46, 40, 31, 21, 12,  7,  2, 39, 33, 27, 24, 17, 10,  4,  1, 27, 21, 23, 17, 13,  8,  4,  1, 19, 16, 16, 12, 11,  5,  3,  1, 11, 12,  9,  8,  5,  4,  2,  0,  5,  4,  4,  4,  3,  2,  1,  0,  1,  0,  0,  0,  0,  0,  0,  0, },
  //{87, 57, 31, 20, 13,  7,  3,  1, 70, 44, 30, 18, 13,  6,  3,  1, 47, 40, 28, 19, 11,  5,  3,  1, 34, 32, 23, 16, 10,  5,  2,  1, 26, 26, 20, 14,  9,  3,  2,  1, 17, 15, 12, 10,  6,  2,  1,  0,  8,  8,  6,  4,  3,  1,  0,  0,  2,  1,  1,  1,  0,  0,  0,  0, },

{128, 79, 56, 44, 34, 25, 16, 8, 38, 27, 22, 19, 15, 11, 8, 4, 17, 13, 11, 9, 8, 6, 5, 2, 10, 8, 7, 6, 5, 4, 3, 1, 7, 5, 4, 4, 3, 3, 2, 1, 4, 3, 3, 3, 2, 2, 1, 1, 3, 2, 2, 2, 2, 1, 1, 0, 2, 1, 1, 1, 1, 1, 0, 0, },
{128, 34, 14, 9, 6, 5, 3, 2, 88, 27, 12, 7, 5, 4, 2, 1, 61, 23, 10, 7, 4, 3, 2, 1, 54, 21, 10, 6, 4, 3, 2, 1, 46, 18, 8, 5, 3, 2, 1, 1, 31, 13, 7, 4, 3, 2, 1, 0, 20, 10, 5, 3, 2, 1, 1, 0, 11, 6, 4, 2, 1, 1, 0, 0, },
{128, 58, 33, 23, 18, 13, 9, 6, 63, 27, 17, 12, 10, 7, 5, 2, 33, 17, 12, 9, 7, 5, 4, 2, 21, 11, 8, 6, 5, 4, 3, 1, 15, 8, 6, 4, 4, 3, 2, 1, 10, 6, 4, 4, 3, 2, 1, 1, 8, 4, 3, 3, 2, 2, 1, 0, 4, 3, 2, 2, 1, 1, 1, 0, },
{128, 75, 37, 21, 13, 8, 4, 3, 76, 52, 35, 20, 12, 7, 4, 2, 40, 36, 28, 19, 11, 7, 4, 1, 23, 20, 18, 13, 10, 6, 3, 1, 14, 13, 10, 9, 8, 5, 3, 1, 9, 7, 7, 6, 5, 4, 3, 1, 5, 4, 4, 4, 4, 3, 2, 1, 3, 2, 2, 2, 2, 2, 1, 0, },
{128, 76, 40, 24, 15, 9, 5, 3, 80, 60, 40, 23, 14, 7, 4, 2, 40, 42, 36, 24, 12, 7, 4, 1, 23, 22, 23, 19, 12, 7, 3, 1, 14, 13, 12, 12, 10, 7, 4, 1, 8, 7, 7, 7, 7, 6, 4, 1, 5, 4, 4, 4, 4, 4, 3, 1, 3, 2, 2, 2, 2, 2, 1, 0, },
{128, 90, 57, 37, 23, 14, 7, 4, 61, 49, 43, 34, 24, 14, 8, 3, 26, 23, 22, 22, 18, 13, 8, 3, 15, 13, 12, 11, 11, 9, 6, 3, 9, 8, 7, 7, 6, 6, 4, 2, 6, 5, 5, 5, 4, 3, 3, 1, 4, 3, 3, 3, 3, 2, 1, 1, 2, 2, 2, 2, 1, 1, 1, 0, },
{128, 58, 25, 14, 9, 6, 3, 2, 93, 48, 22, 13, 8, 5, 3, 1, 63, 43, 21, 12, 7, 4, 3, 1, 36, 33, 19, 10, 6, 4, 2, 1, 21, 21, 15, 9, 5, 4, 2, 1, 12, 11, 11, 8, 5, 3, 2, 1, 7, 7, 7, 6, 5, 3, 1, 1, 4, 4, 4, 4, 3, 2, 1, 0, },
{128, 91, 57, 37, 23, 14, 7, 4, 67, 50, 40, 31, 22, 13, 7, 3, 30, 27, 23, 20, 16, 12, 7, 3, 18, 15, 13, 11, 10, 8, 6, 3, 11, 10, 8, 7, 6, 5, 4, 2, 7, 6, 5, 5, 4, 3, 2, 1, 4, 4, 4, 3, 3, 2, 2, 1, 3, 2, 2, 2, 2, 1, 1, 0, },
{128, 58, 26, 17, 11, 7, 4, 3, 88, 47, 26, 16, 10, 6, 4, 2, 55, 37, 21, 13, 8, 5, 3, 1, 33, 26, 16, 9, 6, 4, 3, 1, 20, 17, 12, 7, 5, 3, 2, 1, 11, 10, 9, 6, 4, 3, 2, 1, 6, 6, 6, 5, 3, 2, 1, 1, 3, 3, 3, 3, 2, 1, 1, 0, },
  
  };

static const int SCANSTATS4x4[9][16] = 
{
  //{80, 66, 47, 27, 46, 35, 21,  8, 28, 23, 13,  4, 14, 11,  5,  2, },
  //{75, 35, 19,  9, 66, 32, 17,  7, 60, 27, 14,  5, 42, 15,  7,  2, },
  //{87, 43, 26, 12, 55, 33, 20,  8, 37, 27, 15,  6, 21, 12,  7,  2, },
  //{86, 60, 36, 15, 63, 47, 31, 13, 39, 34, 24,  8, 18, 13,  9,  3, },
  //{82, 54, 28,  9, 58, 53, 35,  9, 35, 41, 33, 10, 13, 15, 14,  5, },
  //{85, 61, 36, 15, 56, 44, 30, 14, 33, 28, 18,  9, 15, 13,  8,  3, },
  //{80, 47, 24, 11, 64, 43, 23,  9, 48, 41, 20,  9, 27, 23, 15,  4, },
  //{85, 61, 39, 17, 56, 44, 31, 12, 36, 30, 19,  7, 17, 12,  8,  3, },
  //{83, 44, 23,  8, 66, 41, 21,  9, 51, 40, 17,  7, 30, 24, 10,  3, },

{128, 83, 61, 37, 30, 26, 20, 11, 11, 9, 7, 4, 6, 5, 3, 0, },
{128, 30, 12, 5, 90, 25, 10, 4, 65, 19, 7, 3, 40, 11, 4, 0, },
{128, 59, 33, 18, 53, 27, 16, 8, 23, 13, 9, 5, 10, 7, 5, 0, },
{128, 69, 30, 11, 67, 44, 26, 10, 25, 21, 16, 8, 8, 8, 7, 0, },
{128, 68, 24, 7, 72, 53, 28, 8, 23, 28, 21, 9, 8, 10, 10, 0, },
{128, 86, 48, 18, 42, 42, 34, 19, 15, 15, 16, 10, 6, 7, 5, 0, },
{128, 40, 14, 6, 85, 38, 14, 5, 53, 32, 15, 5, 18, 19, 11, 0, },
{128, 87, 48, 19, 53, 41, 31, 16, 16, 18, 15, 8, 7, 7, 6, 0, },
{128, 38, 11, 5, 86, 34, 13, 6, 54, 29, 12, 4, 20, 17, 8, 0, },

};





#ifdef COMBINED_MAP
static UInt FIX_SCANSTATS64x64[9][4096] = 
{
	{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
45945, 28671, 19375, 18727, 15474, 13889, 12610, 11563, 10413, 10575, 11003, 11515, 8447, 10521, 9759, 10756, 9559, 9778, 7993, 9145, 8140, 9940, 7640, 7181, 7249, 8168, 8889, 6746, 7220, 8001, 8271, 9132, 6291, 5574, 6792, 7892, 6881, 4830, 5218, 6667, 5368, 5252, 4176, 4333, 5276, 5288, 4231, 3469, 2783, 4475, 3119, 2455, 2442, 2526, 2178, 2426, 1341, 1629, 1407, 2594, 1497, 2167, 1350, 4567, 
35434, 12901, 8044, 6947, 6453, 5907, 5568, 4884, 4405, 4059, 5914, 4724, 3467, 4659, 4757, 6095, 5581, 5122, 4727, 4534, 5763, 6093, 3836, 3797, 4634, 4512, 4451, 4155, 4042, 4360, 5191, 5822, 3616, 3152, 4799, 4438, 4119, 3028, 2851, 3638, 2696, 3460, 1873, 1646, 3036, 3247, 2399, 1863, 1865, 2082, 1755, 1309, 1288, 884, 1214, 1028, 872, 701, 663, 659, 641, 560, 688, 687, 
14119, 8625, 6188, 5432, 4388, 3955, 3287, 3457, 2944, 2993, 2235, 2025, 2815, 2042, 2183, 2359, 2579, 2165, 1569, 1622, 1579, 1886, 1345, 1461, 1896, 1649, 1416, 1728, 1490, 1653, 1861, 2431, 1771, 1807, 1607, 1630, 1318, 1311, 1233, 1210, 1066, 1059, 1010, 1011, 1111, 1116, 1108, 962, 910, 930, 824, 707, 616, 616, 528, 498, 479, 488, 484, 545, 470, 572, 536, 678, 
13338, 7535, 6086, 5094, 4333, 3839, 2973, 2841, 2679, 2140, 2075, 2170, 1360, 1298, 1406, 1007, 1000, 860, 763, 815, 997, 688, 726, 748, 601, 457, 672, 672, 611, 561, 591, 494, 492, 401, 464, 373, 320, 283, 311, 304, 298, 300, 320, 224, 241, 267, 294, 330, 267, 214, 207, 211, 189, 162, 191, 174, 162, 147, 155, 176, 206, 200, 215, 258, 
8601, 6633, 5377, 4673, 4341, 3501, 3174, 2783, 2288, 1902, 1834, 1514, 1205, 1140, 1031, 1021, 837, 988, 815, 801, 641, 733, 679, 569, 369, 359, 402, 489, 267, 321, 292, 401, 348, 381, 255, 274, 230, 350, 301, 217, 253, 223, 208, 153, 149, 181, 146, 119, 113, 97, 86, 63, 85, 82, 57, 73, 41, 35, 37, 37, 38, 25, 23, 40, 
7431, 6197, 5138, 3981, 3648, 2928, 2855, 2382, 2215, 1846, 1370, 1343, 1168, 987, 885, 716, 681, 837, 532, 522, 753, 566, 449, 401, 340, 308, 278, 242, 304, 180, 209, 223, 195, 226, 240, 150, 168, 162, 148, 134, 121, 141, 113, 95, 86, 91, 93, 71, 66, 64, 49, 26, 46, 42, 37, 31, 13, 10, 14, 9, 9, 18, 16, 17, 
6368, 5237, 4342, 3487, 3168, 2788, 2156, 1997, 1669, 1510, 1333, 1116, 904, 833, 789, 663, 645, 602, 549, 563, 423, 435, 325, 336, 283, 316, 291, 260, 170, 179, 179, 160, 205, 201, 147, 214, 120, 166, 114, 151, 152, 108, 81, 68, 74, 103, 79, 55, 52, 52, 36, 33, 34, 28, 21, 22, 19, 14, 8, 5, 9, 5, 13, 12, 
5958, 5208, 4373, 3392, 3129, 2576, 2283, 1858, 1657, 1467, 1261, 1117, 994, 784, 652, 610, 533, 514, 595, 491, 484, 403, 316, 267, 328, 237, 239, 193, 164, 237, 175, 131, 152, 121, 174, 183, 148, 123, 99, 134, 79, 90, 120, 108, 73, 82, 73, 78, 47, 43, 32, 42, 35, 10, 24, 13, 11, 10, 11, 10, 3, 9, 10, 8, 
5732, 4517, 3825, 2991, 2744, 2282, 1900, 1691, 1533, 1278, 1133, 977, 863, 709, 592, 563, 568, 449, 422, 432, 377, 353, 327, 267, 258, 249, 223, 179, 195, 144, 157, 121, 143, 105, 126, 151, 145, 98, 94, 145, 98, 150, 59, 73, 91, 88, 66, 47, 47, 32, 33, 30, 26, 14, 22, 17, 8, 11, 9, 4, 4, 1, 5, 5, 
5362, 4504, 3515, 2861, 2395, 2251, 1931, 1658, 1442, 1302, 1082, 898, 739, 668, 541, 547, 577, 527, 445, 449, 407, 405, 308, 253, 237, 195, 203, 187, 161, 142, 137, 118, 111, 134, 144, 103, 111, 104, 79, 80, 72, 85, 86, 71, 89, 50, 71, 37, 34, 34, 22, 33, 25, 20, 15, 15, 12, 10, 9, 4, 5, 7, 5, 4, 
4518, 3549, 3037, 2565, 2201, 1983, 1649, 1353, 1244, 1105, 933, 703, 648, 603, 546, 475, 489, 451, 407, 387, 319, 352, 241, 225, 213, 179, 187, 155, 158, 119, 142, 113, 121, 111, 135, 124, 114, 81, 81, 82, 116, 92, 72, 90, 65, 68, 43, 42, 38, 37, 32, 21, 14, 32, 14, 13, 9, 6, 9, 5, 6, 6, 7, 7, 
4263, 3514, 2897, 2197, 2193, 1739, 1497, 1176, 1077, 870, 855, 678, 623, 569, 494, 508, 403, 466, 393, 346, 330, 273, 224, 248, 207, 180, 215, 194, 110, 165, 153, 82, 90, 120, 92, 87, 94, 96, 113, 75, 91, 112, 126, 95, 81, 61, 71, 62, 45, 31, 46, 27, 18, 20, 15, 15, 10, 12, 22, 4, 7, 1, 8, 4, 
3768, 3095, 2549, 2280, 1794, 1561, 1239, 1108, 944, 826, 673, 655, 613, 468, 412, 420, 427, 358, 318, 276, 295, 243, 201, 199, 167, 141, 135, 130, 109, 94, 95, 91, 109, 82, 108, 96, 88, 89, 82, 65, 75, 64, 46, 44, 58, 47, 30, 68, 47, 33, 26, 26, 24, 16, 19, 8, 8, 11, 9, 2, 5, 5, 2, 5, 
3734, 2971, 2436, 1931, 1535, 1409, 1085, 951, 726, 649, 648, 545, 453, 451, 382, 347, 298, 265, 290, 245, 210, 176, 182, 153, 148, 156, 131, 108, 83, 95, 88, 110, 72, 86, 91, 67, 64, 86, 51, 78, 68, 70, 42, 66, 65, 37, 39, 58, 31, 33, 33, 22, 23, 17, 18, 12, 13, 8, 6, 5, 10, 7, 4, 6, 
3404, 2602, 2143, 1760, 1431, 1098, 935, 769, 623, 618, 552, 495, 404, 378, 334, 304, 259, 239, 208, 169, 186, 156, 131, 126, 95, 90, 115, 74, 98, 67, 74, 61, 61, 66, 75, 68, 61, 61, 56, 69, 53, 40, 32, 63, 36, 35, 30, 37, 30, 32, 24, 15, 11, 20, 10, 10, 11, 9, 4, 2, 2, 3, 7, 8, 
2847, 2253, 1906, 1520, 1255, 1028, 891, 689, 623, 527, 469, 487, 438, 384, 309, 282, 257, 241, 231, 175, 149, 137, 137, 122, 133, 96, 71, 94, 67, 71, 84, 72, 56, 69, 91, 63, 64, 48, 59, 47, 47, 50, 54, 36, 37, 37, 36, 28, 19, 22, 11, 8, 16, 13, 7, 4, 6, 7, 4, 2, 4, 2, 2, 7, 
2524, 2203, 1652, 1431, 1109, 1013, 758, 665, 615, 513, 407, 400, 390, 310, 277, 241, 205, 184, 163, 167, 127, 144, 158, 114, 98, 109, 94, 71, 77, 67, 79, 69, 47, 61, 78, 60, 42, 35, 44, 62, 44, 29, 32, 36, 53, 34, 31, 23, 27, 13, 12, 12, 10, 9, 11, 7, 3, 1, 3, 5, 4, 6, 1, 0, 
2546, 2004, 1538, 1238, 1091, 777, 758, 626, 562, 478, 421, 376, 360, 280, 251, 213, 205, 203, 155, 141, 141, 125, 117, 110, 101, 96, 91, 90, 94, 88, 69, 66, 74, 74, 50, 58, 54, 60, 47, 43, 61, 48, 40, 42, 37, 41, 32, 17, 25, 18, 17, 13, 11, 7, 7, 5, 1, 4, 1, 6, 4, 2, 1, 6, 
2719, 2140, 1630, 1173, 1016, 874, 699, 567, 503, 408, 383, 369, 359, 260, 258, 198, 188, 192, 158, 150, 136, 142, 141, 103, 117, 98, 98, 90, 88, 94, 68, 72, 64, 47, 61, 59, 50, 43, 45, 38, 42, 41, 29, 27, 33, 39, 30, 19, 12, 15, 11, 22, 12, 7, 7, 12, 13, 4, 3, 5, 4, 4, 1, 2, 
2468, 1922, 1465, 1170, 1029, 857, 661, 619, 489, 470, 368, 341, 358, 272, 246, 228, 198, 169, 126, 139, 179, 137, 133, 106, 94, 108, 103, 86, 67, 64, 79, 79, 43, 48, 54, 47, 44, 40, 37, 36, 43, 32, 42, 26, 29, 32, 19, 15, 10, 18, 11, 10, 9, 5, 4, 12, 2, 5, 8, 3, 9, 2, 3, 3, 
2205, 2047, 1489, 1252, 1116, 827, 672, 566, 474, 439, 432, 378, 346, 329, 253, 224, 215, 207, 171, 171, 151, 126, 171, 133, 136, 144, 99, 116, 95, 84, 101, 123, 95, 88, 69, 55, 44, 53, 55, 56, 64, 61, 45, 42, 37, 41, 32, 32, 25, 27, 14, 26, 24, 22, 26, 19, 17, 21, 18, 13, 20, 9, 13, 1, 
1913, 1607, 1312, 1130, 919, 700, 561, 547, 456, 357, 341, 251, 273, 201, 220, 236, 153, 131, 129, 129, 116, 119, 149, 138, 103, 92, 84, 60, 63, 75, 54, 67, 70, 63, 54, 63, 48, 60, 31, 35, 37, 41, 33, 28, 27, 37, 28, 15, 31, 16, 17, 14, 30, 8, 6, 25, 3, 5, 6, 7, 3, 3, 1, 1, 
2089, 1734, 1411, 1004, 887, 585, 552, 415, 347, 320, 291, 247, 196, 186, 206, 160, 166, 112, 125, 145, 134, 89, 93, 87, 74, 90, 87, 69, 66, 86, 62, 62, 50, 51, 47, 43, 49, 44, 49, 28, 38, 52, 27, 21, 28, 22, 21, 23, 15, 16, 9, 7, 10, 5, 9, 9, 3, 4, 1, 3, 3, 4, 3, 3, 
2142, 1664, 1488, 966, 787, 661, 453, 429, 363, 294, 258, 287, 244, 181, 153, 197, 152, 137, 124, 114, 162, 141, 114, 74, 78, 105, 88, 67, 55, 58, 83, 78, 70, 48, 49, 49, 52, 47, 53, 37, 41, 35, 50, 38, 26, 23, 32, 23, 23, 23, 15, 7, 9, 6, 3, 12, 4, 2, 6, 4, 5, 1, 1, 3, 
1726, 1372, 1065, 751, 673, 571, 420, 319, 278, 242, 231, 241, 239, 188, 141, 143, 131, 117, 106, 88, 85, 93, 94, 78, 74, 83, 68, 74, 60, 58, 56, 52, 43, 45, 40, 34, 30, 32, 40, 33, 43, 36, 24, 22, 25, 32, 25, 17, 9, 10, 13, 9, 18, 2, 0, 6, 4, 0, 3, 4, 1, 3, 1, 3, 
1709, 1354, 1045, 745, 576, 496, 333, 398, 279, 206, 173, 181, 208, 161, 142, 122, 110, 113, 117, 85, 71, 84, 83, 82, 82, 75, 64, 85, 75, 59, 51, 51, 50, 42, 35, 49, 41, 25, 30, 34, 41, 32, 32, 30, 18, 21, 19, 20, 17, 10, 5, 9, 7, 6, 9, 8, 3, 2, 1, 1, 3, 1, 2, 2, 
1560, 1629, 1036, 751, 612, 486, 386, 352, 304, 223, 201, 157, 183, 153, 114, 143, 100, 118, 91, 95, 90, 64, 83, 75, 83, 63, 58, 63, 47, 58, 59, 56, 60, 39, 35, 49, 34, 37, 38, 33, 35, 27, 39, 29, 27, 15, 24, 16, 15, 13, 12, 19, 15, 10, 7, 3, 1, 3, 1, 2, 3, 0, 0, 0, 
1806, 1380, 1301, 801, 612, 555, 409, 323, 236, 246, 215, 141, 152, 144, 151, 117, 111, 118, 99, 108, 86, 90, 57, 75, 56, 58, 51, 55, 63, 62, 61, 66, 46, 41, 46, 40, 35, 42, 46, 29, 31, 36, 38, 28, 27, 20, 21, 22, 14, 15, 10, 14, 8, 10, 10, 4, 4, 2, 1, 1, 1, 2, 1, 1, 
1450, 1475, 921, 740, 670, 532, 425, 308, 258, 179, 210, 172, 143, 116, 95, 100, 101, 93, 84, 80, 85, 65, 61, 78, 67, 75, 59, 33, 53, 60, 52, 48, 34, 37, 42, 36, 44, 36, 33, 41, 29, 33, 40, 26, 18, 22, 19, 13, 10, 7, 9, 10, 12, 2, 6, 4, 1, 5, 1, 0, 1, 3, 1, 2, 
1435, 1208, 1008, 709, 733, 516, 368, 279, 230, 228, 209, 171, 155, 120, 119, 99, 126, 85, 94, 74, 63, 70, 79, 78, 62, 77, 59, 46, 45, 50, 49, 48, 44, 38, 30, 38, 41, 39, 31, 28, 32, 20, 21, 23, 12, 23, 14, 10, 9, 7, 10, 6, 6, 6, 5, 4, 0, 2, 1, 1, 1, 2, 0, 0, 
1197, 1166, 964, 801, 581, 517, 416, 306, 229, 202, 165, 179, 143, 131, 94, 95, 112, 81, 82, 91, 62, 75, 63, 66, 70, 75, 74, 80, 48, 52, 57, 44, 39, 61, 51, 42, 39, 38, 43, 24, 31, 33, 24, 26, 27, 14, 22, 20, 17, 9, 8, 6, 8, 3, 11, 4, 0, 2, 5, 3, 1, 2, 0, 0, 
1264, 1231, 1157, 727, 532, 496, 383, 254, 226, 182, 164, 177, 183, 137, 113, 90, 91, 110, 82, 64, 79, 79, 77, 78, 85, 74, 64, 64, 52, 60, 44, 37, 36, 48, 39, 46, 35, 34, 33, 33, 40, 33, 25, 27, 23, 15, 15, 13, 6, 4, 8, 5, 8, 4, 0, 4, 0, 0, 3, 3, 2, 2, 2, 1, 
1337, 1327, 1013, 823, 540, 400, 317, 223, 209, 196, 178, 128, 121, 104, 113, 105, 64, 99, 94, 84, 103, 76, 86, 66, 63, 69, 55, 53, 60, 43, 45, 38, 37, 37, 34, 31, 38, 34, 45, 23, 33, 31, 20, 25, 19, 18, 20, 27, 13, 10, 8, 1, 5, 1, 0, 4, 1, 3, 2, 1, 1, 0, 0, 2, 
1198, 1043, 828, 617, 437, 362, 282, 266, 204, 147, 147, 102, 111, 103, 98, 81, 54, 57, 62, 71, 71, 76, 51, 57, 54, 43, 56, 57, 39, 49, 53, 39, 43, 33, 33, 38, 28, 30, 27, 36, 31, 27, 25, 20, 16, 20, 16, 13, 9, 7, 8, 7, 6, 5, 5, 5, 2, 3, 2, 1, 0, 1, 5, 1, 
1096, 940, 830, 612, 476, 375, 270, 272, 182, 150, 102, 103, 134, 101, 76, 69, 73, 55, 60, 74, 69, 56, 58, 57, 56, 75, 59, 60, 51, 39, 44, 39, 38, 42, 49, 40, 24, 43, 41, 41, 38, 30, 29, 20, 16, 17, 9, 9, 3, 10, 10, 4, 5, 1, 2, 6, 0, 4, 1, 0, 0, 0, 0, 1, 
1099, 993, 990, 621, 469, 277, 253, 184, 139, 130, 114, 77, 77, 89, 71, 73, 41, 60, 73, 56, 71, 46, 64, 60, 53, 68, 47, 52, 55, 56, 30, 46, 29, 28, 35, 26, 31, 34, 31, 32, 19, 21, 40, 18, 27, 20, 14, 15, 13, 9, 6, 4, 7, 3, 0, 1, 0, 1, 2, 1, 0, 0, 0, 1, 
1094, 1158, 921, 855, 487, 318, 223, 212, 143, 135, 98, 96, 75, 66, 84, 90, 56, 60, 58, 84, 63, 80, 62, 56, 62, 56, 32, 50, 38, 35, 43, 46, 32, 45, 28, 26, 33, 30, 29, 29, 24, 28, 32, 16, 16, 13, 12, 14, 8, 7, 8, 6, 2, 2, 3, 2, 0, 1, 2, 1, 0, 1, 1, 1, 
1063, 705, 914, 562, 404, 332, 239, 202, 145, 127, 113, 82, 71, 79, 78, 88, 88, 71, 78, 66, 63, 53, 69, 42, 59, 51, 47, 43, 35, 37, 38, 32, 55, 31, 34, 39, 38, 32, 34, 31, 21, 29, 24, 22, 16, 15, 14, 11, 5, 13, 3, 5, 6, 1, 1, 4, 0, 1, 0, 0, 0, 2, 0, 1, 
1031, 756, 654, 487, 365, 277, 214, 184, 148, 122, 107, 86, 91, 71, 85, 63, 58, 77, 59, 75, 59, 67, 48, 52, 66, 45, 50, 37, 47, 36, 31, 35, 59, 35, 41, 44, 37, 38, 31, 32, 26, 32, 20, 29, 14, 14, 15, 12, 3, 6, 4, 8, 1, 2, 2, 2, 0, 0, 2, 2, 0, 0, 1, 1, 
979, 657, 645, 501, 361, 268, 252, 151, 150, 122, 109, 93, 88, 81, 79, 56, 48, 62, 63, 57, 85, 57, 58, 65, 65, 39, 52, 62, 52, 41, 35, 45, 34, 27, 37, 50, 34, 34, 36, 39, 29, 30, 30, 15, 22, 9, 6, 14, 9, 12, 8, 5, 5, 4, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 
928, 953, 818, 766, 343, 277, 237, 147, 109, 117, 96, 106, 81, 85, 87, 81, 85, 68, 53, 78, 66, 72, 58, 79, 76, 49, 28, 48, 37, 44, 47, 31, 38, 39, 37, 31, 30, 33, 22, 27, 21, 22, 17, 21, 11, 12, 14, 8, 8, 1, 5, 5, 4, 5, 2, 2, 0, 1, 2, 0, 1, 0, 3, 0, 
822, 637, 887, 626, 327, 264, 260, 251, 138, 115, 87, 99, 94, 70, 73, 81, 76, 40, 49, 45, 52, 56, 55, 56, 50, 44, 58, 49, 40, 47, 39, 30, 48, 46, 39, 24, 37, 45, 37, 23, 24, 26, 18, 12, 22, 19, 10, 9, 7, 5, 4, 3, 2, 1, 2, 1, 0, 0, 1, 0, 0, 0, 0, 0, 
823, 681, 596, 466, 329, 197, 176, 145, 122, 83, 76, 64, 91, 46, 49, 71, 81, 91, 74, 52, 43, 63, 57, 63, 57, 46, 58, 52, 36, 49, 49, 46, 37, 30, 28, 30, 28, 37, 27, 33, 24, 25, 15, 19, 13, 8, 11, 11, 3, 5, 5, 3, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
854, 650, 497, 438, 331, 276, 177, 186, 159, 108, 84, 61, 54, 51, 49, 64, 60, 49, 58, 61, 61, 64, 44, 45, 55, 46, 61, 41, 43, 39, 40, 53, 29, 33, 25, 30, 25, 30, 26, 17, 13, 31, 19, 18, 13, 15, 9, 9, 6, 5, 6, 3, 2, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 
805, 528, 481, 405, 330, 195, 236, 161, 101, 106, 108, 71, 58, 95, 44, 38, 71, 53, 40, 70, 70, 55, 47, 39, 49, 53, 42, 61, 48, 42, 37, 33, 27, 37, 35, 23, 32, 31, 24, 31, 21, 23, 15, 16, 14, 16, 3, 6, 6, 8, 8, 2, 2, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 
705, 531, 733, 645, 314, 199, 183, 121, 105, 82, 73, 69, 66, 51, 66, 66, 82, 50, 62, 53, 55, 54, 53, 53, 48, 51, 44, 48, 36, 47, 56, 40, 34, 36, 37, 35, 27, 25, 25, 24, 32, 19, 10, 16, 16, 6, 9, 9, 4, 5, 4, 2, 4, 1, 3, 1, 0, 1, 0, 0, 1, 0, 1, 0, 
632, 484, 442, 387, 248, 218, 173, 137, 100, 80, 80, 59, 67, 55, 62, 66, 61, 59, 57, 50, 68, 72, 57, 48, 60, 37, 52, 42, 55, 42, 48, 42, 37, 28, 34, 47, 30, 27, 21, 21, 22, 18, 27, 19, 14, 11, 7, 12, 4, 3, 3, 1, 3, 0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 
690, 499, 400, 372, 263, 202, 161, 135, 105, 91, 69, 46, 65, 47, 70, 55, 57, 60, 49, 38, 49, 63, 57, 41, 43, 53, 51, 47, 44, 52, 37, 43, 36, 36, 24, 31, 39, 22, 27, 24, 18, 17, 19, 14, 12, 8, 10, 3, 4, 5, 3, 2, 2, 3, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 
595, 455, 435, 309, 252, 181, 176, 127, 108, 66, 71, 74, 61, 57, 60, 63, 63, 61, 46, 51, 45, 64, 64, 60, 46, 50, 42, 40, 36, 39, 40, 43, 24, 21, 27, 25, 26, 23, 20, 16, 14, 17, 17, 13, 9, 9, 5, 3, 3, 6, 3, 3, 1, 5, 1, 2, 0, 0, 0, 0, 1, 0, 0, 0, 
548, 414, 408, 575, 232, 184, 165, 118, 92, 91, 69, 78, 74, 61, 60, 54, 47, 55, 58, 53, 63, 50, 40, 48, 53, 55, 52, 51, 56, 37, 35, 43, 28, 31, 31, 33, 26, 22, 25, 22, 14, 24, 14, 11, 7, 5, 8, 5, 4, 4, 8, 3, 3, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 
575, 462, 525, 348, 235, 166, 170, 134, 93, 82, 77, 76, 66, 83, 57, 53, 55, 64, 70, 65, 47, 50, 54, 60, 59, 57, 59, 48, 43, 36, 42, 36, 30, 25, 28, 33, 25, 19, 22, 19, 13, 17, 19, 12, 13, 8, 14, 5, 3, 4, 4, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
506, 328, 310, 268, 214, 146, 135, 129, 109, 106, 101, 80, 72, 64, 58, 63, 71, 56, 52, 71, 50, 52, 52, 44, 57, 57, 47, 38, 57, 36, 41, 43, 29, 26, 31, 32, 32, 28, 11, 20, 15, 23, 13, 14, 10, 3, 8, 4, 3, 2, 2, 1, 3, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 
424, 371, 295, 269, 178, 176, 160, 183, 179, 141, 101, 87, 78, 71, 60, 67, 73, 60, 48, 54, 49, 47, 49, 55, 64, 52, 49, 49, 47, 38, 45, 35, 23, 30, 34, 26, 22, 22, 20, 22, 18, 17, 11, 15, 12, 6, 6, 8, 1, 2, 2, 1, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
405, 328, 248, 219, 176, 127, 156, 114, 93, 99, 77, 47, 66, 60, 53, 54, 85, 55, 50, 46, 60, 53, 53, 58, 53, 45, 58, 49, 47, 41, 50, 34, 26, 40, 25, 19, 23, 31, 22, 29, 20, 14, 12, 13, 10, 7, 8, 6, 3, 3, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
386, 285, 308, 247, 199, 191, 155, 134, 88, 91, 61, 73, 63, 58, 40, 61, 51, 57, 36, 61, 59, 38, 53, 53, 48, 43, 39, 34, 32, 30, 30, 30, 29, 25, 31, 25, 31, 21, 23, 20, 12, 14, 12, 18, 7, 6, 4, 5, 1, 3, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 
397, 319, 263, 198, 156, 124, 125, 102, 101, 68, 86, 57, 57, 53, 46, 54, 62, 55, 46, 53, 60, 42, 55, 46, 52, 61, 58, 52, 31, 29, 30, 31, 30, 21, 40, 31, 26, 25, 28, 28, 15, 8, 15, 9, 11, 9, 2, 5, 2, 1, 3, 1, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
392, 285, 264, 197, 151, 107, 115, 98, 82, 80, 60, 64, 76, 63, 58, 42, 56, 48, 53, 56, 54, 54, 64, 43, 56, 35, 32, 45, 29, 39, 33, 25, 25, 26, 23, 18, 15, 11, 14, 15, 6, 8, 6, 8, 3, 5, 1, 3, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
387, 285, 227, 200, 125, 105, 112, 88, 84, 69, 75, 64, 64, 56, 51, 64, 54, 61, 55, 74, 52, 56, 54, 53, 62, 45, 42, 52, 49, 33, 39, 27, 25, 29, 22, 31, 19, 17, 16, 14, 16, 15, 8, 7, 7, 8, 7, 7, 2, 1, 1, 2, 1, 1, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 
356, 225, 187, 181, 134, 93, 93, 82, 95, 89, 68, 61, 71, 63, 44, 41, 50, 49, 51, 58, 43, 56, 55, 53, 42, 39, 46, 43, 44, 29, 33, 34, 34, 35, 20, 22, 21, 18, 17, 13, 13, 14, 19, 10, 6, 9, 7, 5, 1, 1, 1, 1, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 
325, 231, 223, 198, 157, 114, 102, 81, 86, 62, 64, 66, 77, 73, 56, 54, 52, 60, 58, 51, 47, 59, 37, 51, 49, 43, 51, 54, 38, 30, 22, 28, 28, 34, 30, 29, 21, 24, 18, 17, 16, 12, 8, 8, 19, 9, 8, 2, 3, 4, 1, 0, 0, 3, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
309, 224, 238, 178, 143, 111, 125, 103, 85, 63, 58, 55, 57, 51, 58, 54, 60, 52, 54, 58, 48, 58, 56, 45, 46, 52, 45, 40, 34, 35, 26, 25, 21, 26, 24, 19, 18, 15, 18, 16, 15, 17, 12, 13, 5, 10, 5, 4, 1, 1, 2, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
271, 212, 196, 188, 135, 106, 98, 113, 65, 67, 61, 67, 59, 61, 67, 49, 58, 49, 56, 46, 57, 56, 57, 65, 44, 45, 43, 41, 39, 38, 43, 27, 24, 28, 16, 19, 19, 23, 24, 10, 16, 11, 9, 8, 5, 4, 4, 4, 3, 4, 2, 1, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
294, 178, 227, 218, 148, 121, 91, 99, 97, 69, 89, 53, 60, 52, 55, 57, 54, 38, 63, 61, 63, 48, 47, 59, 42, 52, 51, 55, 34, 32, 26, 31, 29, 33, 28, 26, 23, 23, 16, 29, 12, 14, 12, 9, 8, 3, 1, 3, 4, 1, 1, 3, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 
508, 306, 239, 207, 153, 113, 110, 82, 76, 74, 77, 49, 65, 53, 55, 70, 62, 57, 49, 46, 59, 44, 42, 37, 40, 56, 54, 38, 37, 35, 37, 16, 48, 34, 30, 20, 19, 22, 23, 20, 9, 15, 4, 7, 13, 3, 8, 5, 2, 4, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, },
{
670588, 581182, 500196, 477932, 440557, 418469, 394804, 378772, 363737, 345132, 333963, 305894, 285084, 271866, 252533, 232181, 221075, 202137, 190226, 177210, 170175, 162643, 150425, 143600, 138000, 131688, 126194, 120737, 117040, 113568, 109694, 108423, 134803, 91467, 84362, 83518, 74595, 71479, 69835, 65320, 60016, 59001, 50265, 50552, 45953, 45799, 40966, 40257, 36740, 36970, 29418, 29211, 21580, 25725, 19657, 23008, 13856, 23459, 11947, 30395, 10726, 58150, 15709, 184153, 
576054, 493417, 455221, 431793, 404936, 384359, 364561, 348370, 331210, 316981, 303388, 287624, 262401, 251737, 228036, 218895, 199537, 183788, 173071, 164273, 157062, 146820, 140529, 134642, 128298, 122539, 120709, 115062, 110070, 106825, 105221, 98736, 90540, 84720, 83918, 76380, 75526, 66720, 64279, 64562, 58525, 53122, 48707, 48392, 45290, 40282, 39442, 36232, 32777, 32267, 28159, 23977, 21176, 19666, 18090, 15539, 13725, 13049, 11526, 9881, 8471, 8490, 10181, 17456, 
500919, 456301, 427837, 411659, 387589, 366711, 343065, 320009, 316068, 299242, 283268, 265785, 249833, 229278, 212184, 198415, 181078, 166687, 153842, 146145, 138287, 131434, 125521, 118299, 114890, 110688, 109564, 103916, 101657, 98417, 94206, 89878, 83541, 77843, 74177, 70757, 65975, 62515, 58683, 58382, 55890, 51081, 49125, 45901, 43456, 40692, 38085, 34642, 30830, 28987, 25617, 24382, 21679, 20745, 19588, 16932, 14836, 14032, 12526, 11230, 10168, 9558, 9847, 11447, 
475064, 432608, 413789, 397196, 371046, 346966, 326303, 313256, 296379, 279188, 259006, 247213, 227945, 210438, 193962, 179138, 162239, 150283, 141400, 132349, 126604, 121144, 112605, 109830, 103934, 97210, 96296, 92041, 87596, 84201, 80113, 74530, 67585, 63954, 62542, 59956, 56824, 53559, 49575, 49634, 46816, 44781, 41638, 40002, 37730, 35779, 32724, 29162, 26992, 24965, 23370, 22126, 20176, 20277, 18301, 16021, 14371, 13351, 11775, 9997, 9236, 8790, 8160, 8060, 
444513, 416396, 403168, 379136, 360717, 336986, 318151, 304922, 288444, 272616, 252267, 232190, 215541, 204774, 189251, 174167, 157635, 145491, 138220, 131145, 123950, 117246, 111473, 106363, 99179, 93277, 91916, 85354, 82198, 79692, 75283, 70445, 66129, 62080, 59992, 57223, 54206, 51949, 48029, 45457, 42469, 39855, 37369, 35447, 33056, 30323, 28920, 26746, 24088, 22455, 21124, 18890, 17679, 17308, 15581, 12793, 11116, 9743, 8281, 6918, 6422, 5862, 5231, 4736, 
425063, 395893, 376510, 361584, 342035, 325251, 301639, 290126, 277519, 258186, 245896, 227038, 207178, 194265, 176366, 160053, 146804, 137963, 131377, 125665, 117901, 112688, 104482, 99749, 95421, 90336, 86363, 83123, 79163, 74679, 70836, 67456, 62858, 59932, 57453, 54153, 51676, 48086, 45926, 44318, 40027, 39138, 36087, 33646, 32444, 28764, 27410, 25392, 23466, 21807, 18992, 17626, 15708, 13817, 12422, 10565, 8589, 7546, 6036, 4951, 4167, 3860, 3621, 3268, 
399190, 373936, 356743, 348691, 337357, 320005, 292257, 287626, 262590, 248435, 235789, 217094, 196003, 182031, 169943, 154509, 142685, 130553, 125721, 120044, 114663, 106350, 100319, 95092, 90462, 86201, 82182, 77877, 74430, 72374, 69852, 66212, 62383, 58742, 55400, 54768, 51365, 49351, 44934, 41671, 40134, 37065, 34986, 31992, 30128, 28971, 26237, 24296, 21571, 20833, 18473, 16646, 14896, 13312, 11641, 9742, 7819, 6643, 5475, 4342, 3489, 3107, 2861, 2674, 
378870, 357213, 342540, 332879, 318751, 307694, 285143, 270338, 251963, 237724, 228008, 204914, 186673, 173469, 158633, 145053, 135366, 127479, 120028, 113500, 108236, 104301, 97783, 93301, 89206, 83162, 78570, 73941, 71500, 69409, 65800, 61641, 58657, 55226, 52124, 51186, 49945, 46671, 44076, 41891, 39342, 37657, 33958, 31013, 30497, 27999, 25619, 23247, 20615, 19306, 18205, 16413, 14286, 12298, 10918, 9173, 7026, 6089, 4853, 3836, 3211, 2674, 2391, 2390, 
357307, 347658, 329736, 319571, 303576, 292351, 272755, 256357, 238572, 223542, 206636, 192079, 175028, 161572, 150289, 138322, 130158, 123077, 115591, 110154, 105044, 98518, 92933, 88093, 85209, 81545, 75533, 71571, 69473, 67450, 62740, 58720, 55584, 52255, 51419, 49338, 46287, 45743, 43361, 40548, 38516, 34766, 34059, 31564, 30081, 28910, 26396, 22986, 20748, 19763, 17730, 16642, 13946, 12393, 11185, 8852, 6667, 5589, 4362, 3446, 2789, 2391, 2109, 2064, 
353543, 336423, 319655, 307324, 292630, 282394, 256327, 244851, 224932, 214815, 196945, 178463, 165193, 153254, 142580, 131743, 124586, 116726, 109814, 105873, 100817, 95107, 88213, 86454, 80535, 75606, 71878, 69156, 65526, 61870, 59875, 56815, 53337, 51272, 48709, 47221, 45367, 42423, 40084, 39593, 35837, 33525, 31804, 29107, 28247, 25888, 24626, 22653, 20272, 19153, 18451, 16141, 14603, 12927, 11180, 9518, 6440, 5478, 4335, 3390, 2766, 2400, 1995, 1983, 
333297, 315732, 308128, 293958, 282148, 262720, 243383, 226284, 211558, 193386, 179453, 166127, 152843, 143218, 132850, 126322, 118491, 112154, 107727, 104166, 97703, 93266, 87965, 82626, 77790, 74018, 70199, 66236, 63745, 60834, 56687, 54313, 50758, 48646, 48058, 45735, 42636, 41046, 38648, 36615, 33732, 31685, 28753, 27707, 26160, 25128, 23585, 21012, 18383, 18250, 16111, 15691, 13336, 13262, 11277, 8594, 7353, 5294, 4242, 3844, 3083, 2355, 1926, 1715, 
314273, 294585, 287092, 271190, 258460, 241900, 227585, 205169, 192490, 177499, 167165, 152914, 143385, 134070, 127610, 120570, 112954, 109127, 103286, 99522, 95632, 89669, 84157, 79409, 74033, 70326, 66550, 64381, 60305, 57236, 54742, 52631, 49340, 47534, 45035, 43252, 40641, 38365, 35954, 34328, 32230, 30156, 27663, 26519, 24920, 23475, 21428, 19701, 17413, 16374, 15214, 13787, 12316, 11171, 9703, 7931, 6155, 5277, 3892, 3684, 2892, 2653, 1845, 1770, 
286070, 269930, 256520, 247944, 235036, 220312, 202741, 189080, 175973, 165990, 151791, 142296, 133761, 127748, 121042, 113555, 108737, 102718, 99214, 93906, 90439, 84495, 79589, 75466, 72206, 68050, 63345, 61652, 58684, 55070, 53070, 49560, 47389, 45599, 42624, 40762, 39276, 37422, 34071, 33110, 30621, 28257, 27467, 25698, 23797, 22610, 21167, 18793, 16631, 16461, 14079, 13033, 11447, 9619, 8500, 7070, 5576, 4532, 3641, 2753, 2655, 2056, 1786, 1640, 
254999, 244306, 236540, 224598, 212176, 201375, 188420, 176124, 162577, 151550, 141463, 133490, 125456, 119097, 112254, 107691, 102688, 97407, 95277, 90496, 85215, 81746, 76252, 72103, 68264, 65453, 63073, 59719, 56168, 53183, 50832, 48310, 44814, 42854, 40699, 40107, 37743, 34530, 32659, 32193, 29564, 28118, 25955, 24056, 22870, 21260, 19446, 17986, 15935, 15298, 14110, 12102, 10211, 9640, 8135, 6487, 5007, 4320, 3133, 2452, 2020, 1633, 1527, 1249, 
238039, 223627, 214617, 203860, 197810, 182917, 172086, 161801, 151039, 141885, 133612, 126679, 117719, 111015, 109158, 103581, 99194, 93556, 89200, 84036, 80135, 76641, 72876, 69061, 66507, 62890, 59060, 57529, 53865, 51258, 48384, 47543, 43504, 41956, 39999, 37789, 35176, 34274, 31432, 30373, 28499, 26560, 25074, 23242, 22352, 20801, 18991, 17422, 15698, 14621, 13401, 11188, 9736, 9143, 7073, 5870, 4919, 3492, 2603, 2244, 1555, 1397, 1173, 1206, 
216878, 204539, 197504, 188599, 180854, 171756, 163082, 153203, 141747, 133850, 126616, 120296, 112913, 106881, 102895, 99186, 94160, 90414, 85493, 81758, 76971, 74673, 69274, 66296, 64353, 60690, 56733, 54515, 52489, 50052, 46750, 44861, 40782, 40721, 38016, 35648, 35268, 33153, 30696, 29125, 27363, 25692, 23850, 23108, 20950, 19321, 18844, 17450, 15096, 13535, 12428, 10984, 9599, 7950, 6753, 5246, 3866, 3137, 2304, 1795, 1428, 1164, 1114, 1080, 
200347, 189001, 183918, 177195, 172270, 161121, 152699, 144218, 136160, 126887, 122102, 115756, 108844, 103710, 98229, 94313, 90723, 87022, 81645, 78198, 74130, 70415, 67031, 64171, 59515, 57239, 54847, 52901, 50926, 48503, 45477, 43028, 39582, 38907, 37597, 35179, 32764, 31740, 29743, 27744, 26645, 24414, 22872, 21721, 20277, 19006, 17512, 15653, 14472, 13399, 11799, 10555, 8895, 7525, 5910, 4786, 3786, 2766, 2057, 1698, 1263, 1083, 961, 911, 
194659, 181570, 176930, 168050, 162631, 155061, 149736, 138322, 131563, 124282, 118217, 113410, 107355, 101392, 96543, 89898, 86975, 82706, 78037, 74173, 71629, 68264, 63666, 60098, 56679, 54536, 52258, 49360, 48774, 45722, 43825, 42024, 38150, 36033, 35780, 33402, 30840, 29292, 28708, 26361, 24961, 22763, 22240, 21007, 19796, 17508, 17028, 15624, 13651, 12576, 11059, 10155, 8622, 7213, 5862, 4317, 3235, 2501, 1788, 1376, 1146, 992, 887, 890, 
188586, 177919, 170153, 163542, 157177, 149817, 145022, 138391, 129189, 121674, 115017, 109499, 103479, 99093, 90683, 88000, 82626, 79217, 74958, 70400, 67848, 64225, 61388, 56659, 53513, 52233, 49887, 47237, 45680, 43905, 41455, 38064, 36719, 35554, 33442, 31602, 31070, 28275, 27213, 25177, 24176, 22133, 20786, 20009, 18072, 17583, 16235, 14860, 13694, 11888, 10509, 9241, 7956, 6708, 5309, 4164, 3012, 2292, 1674, 1369, 1053, 897, 814, 776, 
180913, 169035, 161749, 155320, 152688, 145069, 139759, 131571, 123594, 116246, 107831, 104065, 98840, 93519, 88310, 83869, 79915, 74227, 72278, 67712, 64414, 60808, 57684, 54900, 52509, 50287, 47339, 44839, 43143, 41875, 39462, 37842, 34777, 33812, 32351, 31466, 29157, 26848, 25376, 25165, 22891, 21580, 20356, 18328, 17375, 16053, 15538, 13732, 12288, 11101, 10013, 8605, 7498, 6419, 5031, 3906, 2746, 1993, 1544, 1231, 983, 798, 711, 693, 
167272, 159105, 153164, 148652, 145788, 138936, 130748, 122758, 116105, 111138, 103420, 100481, 93799, 87272, 85368, 78547, 76618, 70713, 67719, 63900, 60606, 58366, 55138, 52080, 49154, 47938, 45948, 43741, 40737, 39953, 38465, 36000, 35351, 32651, 30723, 29832, 28004, 26991, 24846, 23066, 21322, 19746, 19498, 18646, 17343, 15325, 14341, 13410, 11588, 10531, 9628, 8156, 7120, 5827, 4681, 3446, 2750, 1906, 1425, 1091, 856, 765, 716, 667, 
154359, 149673, 143236, 139569, 135276, 127235, 120252, 117496, 109875, 103727, 97813, 92991, 87196, 82422, 78335, 75359, 69893, 68128, 64754, 60774, 58268, 54716, 52202, 49488, 46155, 45354, 43544, 41580, 38581, 38171, 37087, 34342, 32082, 30612, 30241, 27524, 26852, 24647, 23534, 21914, 21157, 20077, 18071, 16668, 16109, 14784, 13824, 12438, 10757, 10164, 8945, 7787, 6460, 5310, 4295, 3287, 2288, 1683, 1340, 1050, 841, 707, 596, 569, 
148591, 142292, 135715, 129719, 126070, 119211, 110084, 107673, 99872, 95966, 92204, 87697, 83401, 76663, 73697, 70564, 65314, 62150, 60338, 57588, 53736, 51418, 47933, 45925, 43519, 43457, 40837, 39433, 38328, 35718, 33853, 32537, 30124, 29296, 28158, 27085, 24941, 24187, 22756, 20965, 19517, 18764, 17214, 16565, 15124, 13908, 13559, 11627, 10521, 9743, 8560, 7474, 5934, 5096, 3953, 2969, 2018, 1596, 1146, 907, 785, 598, 643, 558, 
139562, 132270, 128759, 122786, 118636, 111804, 105321, 101590, 95574, 90399, 84986, 82538, 78094, 73742, 69385, 65573, 61707, 59718, 56976, 53415, 51648, 47693, 45707, 44368, 42126, 40345, 37634, 37937, 36125, 33820, 33376, 31608, 28803, 28925, 26735, 25602, 25182, 22585, 21325, 20543, 18890, 18176, 16210, 16062, 14065, 13319, 12545, 11220, 10194, 9341, 8295, 7083, 5825, 4919, 3881, 2820, 1907, 1488, 1053, 797, 681, 593, 524, 540, 
131897, 124377, 118206, 115194, 109449, 105809, 97790, 92270, 89313, 85178, 80484, 75802, 73508, 68481, 66877, 61776, 59839, 56270, 53235, 51299, 47880, 45890, 43420, 42517, 39998, 38377, 36647, 35036, 34058, 32894, 31372, 30297, 28791, 26820, 26219, 24304, 22837, 21543, 20200, 19911, 18619, 17056, 15661, 14655, 13492, 12844, 11919, 10629, 9396, 9351, 7651, 6611, 5546, 4575, 3542, 2660, 1761, 1420, 959, 701, 593, 505, 467, 485, 
125520, 118538, 112896, 107776, 100920, 96766, 93244, 87940, 84902, 81372, 76906, 73627, 70022, 65641, 61948, 60413, 56287, 52229, 49993, 48574, 45517, 43889, 42021, 40760, 38302, 36323, 34264, 33148, 31669, 31043, 28890, 27945, 26088, 25913, 24459, 23504, 22216, 20269, 19763, 18742, 18207, 16748, 15161, 14626, 13563, 12743, 11229, 10369, 9097, 8462, 7493, 6021, 5089, 4139, 3210, 2370, 1555, 1208, 840, 666, 610, 477, 462, 427, 
120134, 113990, 110073, 104402, 99198, 93356, 88328, 84738, 81805, 76875, 72197, 70026, 65518, 60968, 58089, 55780, 51950, 50063, 46791, 46110, 43015, 40774, 39191, 37293, 34753, 34755, 32652, 31924, 29928, 29219, 27764, 25955, 24393, 24282, 22673, 21376, 20655, 20371, 18984, 16814, 16327, 15246, 15191, 13668, 12350, 11830, 10966, 10098, 8821, 8041, 6744, 5867, 4922, 4015, 2994, 2148, 1342, 1057, 789, 637, 521, 435, 374, 379, 
117013, 109607, 102391, 98533, 92365, 89106, 84609, 81632, 76983, 74130, 68212, 67222, 62849, 58289, 56524, 52936, 50063, 47207, 44685, 42787, 40737, 38890, 37052, 35537, 33688, 32735, 30781, 30254, 28911, 28115, 26821, 25296, 23101, 22539, 22588, 21382, 19625, 18981, 17435, 17181, 15756, 15566, 13796, 13171, 11890, 11270, 10333, 9152, 8124, 7496, 6498, 5369, 4712, 3800, 2864, 2021, 1358, 1001, 742, 571, 454, 467, 392, 376, 
111210, 103391, 98715, 94509, 87998, 85685, 81348, 77496, 74594, 68540, 65795, 63215, 59318, 55953, 53222, 49986, 47257, 44039, 42987, 41390, 39003, 37761, 35736, 34573, 32036, 31258, 30733, 28494, 28198, 25850, 25592, 24647, 22182, 22048, 20833, 20138, 19554, 18167, 17397, 16039, 15304, 14827, 13177, 12578, 11686, 10627, 9927, 8950, 7912, 7418, 6214, 5386, 4433, 3421, 2666, 1887, 1240, 922, 677, 523, 448, 392, 337, 353, 
106055, 98612, 93110, 88060, 84123, 79054, 75943, 71769, 67863, 64521, 63532, 58492, 56432, 53532, 50274, 47235, 45555, 41639, 39686, 38299, 37289, 35309, 32621, 31997, 30558, 28956, 27157, 27452, 25108, 24254, 24245, 23959, 21675, 21964, 20518, 19048, 18926, 17181, 16867, 15810, 15036, 13934, 12810, 11743, 11287, 10275, 9220, 7999, 7423, 6799, 5740, 5062, 4239, 3221, 2356, 1747, 1130, 862, 618, 435, 387, 354, 318, 315, 
101046, 94769, 89236, 83367, 78653, 75309, 71050, 68453, 65127, 61588, 58817, 57043, 54421, 51734, 48621, 44815, 42193, 40110, 38287, 36923, 35381, 32135, 31253, 29900, 29787, 27954, 27827, 25603, 24159, 24167, 23008, 22672, 20893, 20621, 19369, 18565, 17519, 16367, 15807, 14899, 14080, 12824, 12684, 11088, 10433, 9638, 9201, 8039, 7309, 6707, 5417, 4692, 3819, 3063, 2309, 1624, 1103, 812, 566, 415, 402, 316, 322, 336, 
95176, 89398, 85619, 79841, 74007, 70552, 67910, 64365, 61762, 58937, 55095, 53884, 50070, 47875, 45590, 43395, 41035, 38628, 36361, 34821, 33094, 31352, 31032, 28813, 27573, 26127, 25140, 24901, 23236, 21970, 21386, 21740, 19509, 19534, 18763, 17459, 17168, 16134, 15362, 14295, 13321, 12338, 11652, 10843, 10127, 9426, 8407, 7883, 6967, 6250, 5343, 4452, 3768, 2845, 2145, 1557, 943, 762, 553, 469, 360, 334, 273, 288, 
91179, 84603, 80041, 74706, 69926, 66565, 62620, 60209, 58071, 55276, 52499, 50122, 46967, 45818, 43604, 40633, 37983, 35809, 34210, 33350, 30891, 30038, 27939, 27147, 26307, 24871, 24384, 23456, 22819, 21225, 20845, 19868, 18904, 18517, 17426, 17073, 15292, 15326, 14309, 13747, 12845, 12070, 11069, 10336, 9560, 8933, 8422, 7282, 6385, 5870, 5014, 4185, 3420, 2783, 2076, 1535, 864, 734, 459, 391, 374, 285, 240, 275, 
86531, 79860, 75206, 70392, 65398, 62935, 60293, 57345, 55446, 52763, 49984, 48222, 44853, 42384, 40365, 38345, 35810, 33946, 32683, 30982, 29523, 28764, 27083, 25627, 24933, 23627, 22236, 21745, 21849, 20720, 19454, 19674, 17416, 17175, 17674, 15646, 14953, 14222, 13688, 13370, 12305, 11864, 11310, 10279, 9238, 8766, 8084, 6997, 6330, 5620, 4802, 3926, 3269, 2667, 1983, 1421, 870, 712, 448, 379, 331, 279, 270, 242, 
81669, 75295, 72401, 67949, 63187, 58655, 57435, 54474, 52620, 50668, 46168, 44016, 42366, 39864, 38948, 36842, 33594, 32237, 31487, 29488, 27912, 26784, 25218, 23979, 23692, 22637, 21713, 20865, 20195, 19615, 19004, 18174, 16994, 17005, 16303, 15664, 14698, 14072, 13190, 12117, 11810, 10960, 10533, 9729, 8676, 8030, 7467, 6509, 6034, 5315, 4787, 3841, 3040, 2551, 1848, 1283, 795, 608, 436, 375, 300, 262, 252, 248, 
77863, 72261, 68426, 65433, 60263, 56735, 54799, 51374, 49490, 47767, 43778, 40843, 39726, 38752, 36390, 35592, 32429, 31172, 28618, 27689, 26395, 24954, 23634, 22511, 21957, 21138, 21077, 20708, 19412, 18873, 18246, 17515, 16581, 16803, 15279, 14576, 14320, 13357, 12616, 11871, 11180, 10392, 10297, 9397, 8448, 7954, 7288, 6704, 5893, 5110, 4555, 3603, 3061, 2392, 1894, 1218, 794, 599, 437, 378, 286, 265, 268, 241, 
73698, 69946, 66758, 61808, 57581, 53980, 51907, 49183, 47182, 44803, 41673, 39846, 37742, 35816, 34662, 33105, 30970, 29465, 27108, 26085, 25562, 23504, 23001, 21578, 20962, 20483, 20003, 19092, 18913, 17970, 17666, 17246, 15265, 15450, 14631, 14390, 13743, 12629, 12095, 11409, 10789, 10223, 9576, 8950, 8367, 7644, 7107, 6142, 5798, 4995, 4341, 3736, 2886, 2423, 1608, 1154, 716, 538, 418, 336, 245, 242, 244, 232, 
72228, 65912, 62057, 58716, 54894, 51236, 48557, 46467, 44667, 42010, 38961, 37187, 35171, 34322, 33081, 30524, 28640, 27779, 26450, 25216, 24174, 23255, 20781, 20858, 20571, 20138, 19098, 18670, 17572, 17073, 17014, 15983, 15292, 15429, 14169, 13543, 12896, 11970, 11446, 10952, 10579, 9821, 9123, 8412, 7702, 7347, 6730, 6083, 5449, 5010, 4029, 3467, 2719, 2130, 1574, 1065, 627, 470, 386, 297, 264, 234, 215, 206, 
68156, 62742, 58018, 54529, 50678, 46866, 46178, 43579, 42218, 40336, 37813, 35328, 33643, 31867, 30242, 28841, 27746, 25773, 25010, 23637, 22744, 21487, 21042, 19524, 18980, 17883, 17576, 17109, 16845, 16746, 16021, 16016, 14546, 14301, 13627, 13280, 12728, 12391, 11753, 10571, 9719, 9350, 8639, 8097, 7754, 6934, 6454, 5908, 5496, 5030, 4108, 3334, 2642, 2151, 1527, 1023, 656, 500, 354, 324, 247, 224, 185, 211, 
63180, 57326, 53939, 50748, 47673, 44882, 43459, 40616, 38289, 36980, 34967, 33642, 31536, 30327, 28697, 27647, 26467, 24949, 23800, 22203, 21774, 20818, 19703, 18561, 18052, 18624, 16977, 16952, 16498, 15788, 15017, 14587, 14140, 13807, 13418, 12365, 12176, 11711, 10837, 9910, 9464, 9019, 8589, 7776, 7362, 6735, 6443, 5807, 5322, 4556, 4136, 3246, 2605, 1932, 1482, 947, 643, 502, 358, 291, 241, 211, 176, 194, 
59446, 54538, 51947, 48505, 45527, 41662, 40088, 38654, 36440, 34898, 32692, 30485, 29262, 27750, 27100, 25742, 24806, 23134, 22260, 21863, 20134, 19644, 19245, 18733, 17628, 16474, 16546, 16098, 15306, 15664, 14992, 14895, 13448, 13256, 12995, 11923, 11583, 11061, 10172, 9720, 9288, 8489, 8157, 7386, 7116, 6696, 5998, 5803, 4855, 4597, 3721, 3096, 2341, 1735, 1248, 896, 566, 402, 251, 227, 221, 193, 178, 174, 
56038, 51285, 48633, 45353, 42147, 39095, 37207, 35076, 33114, 31726, 29735, 29037, 26973, 26801, 25193, 24098, 22520, 21976, 21310, 20364, 19868, 18360, 18341, 17170, 16896, 16103, 15722, 15370, 14895, 15021, 14760, 13947, 13172, 13275, 12991, 11467, 11094, 10688, 10086, 9263, 8950, 8230, 7910, 7622, 7076, 6607, 5790, 5464, 4582, 4562, 3816, 3081, 2332, 1810, 1348, 948, 497, 430, 311, 236, 226, 232, 182, 171, 
52690, 48813, 46131, 43001, 39092, 37263, 34975, 33014, 31575, 31059, 29035, 26783, 26255, 24724, 23503, 22283, 22183, 21248, 20156, 19399, 18878, 17580, 17079, 16431, 16245, 15699, 14984, 14629, 14285, 14779, 13386, 13191, 12436, 12880, 11976, 11656, 11494, 10277, 9849, 9199, 8614, 8004, 7593, 7495, 6854, 6261, 5806, 5018, 4569, 4013, 3654, 2971, 2261, 1679, 1291, 896, 491, 396, 291, 215, 210, 206, 189, 195, 
50201, 47546, 43916, 41176, 37131, 34441, 32849, 31494, 30091, 28876, 27088, 25505, 24584, 23293, 22150, 21081, 20705, 20409, 18758, 18463, 17846, 16756, 16323, 16062, 15614, 14979, 14555, 14267, 14097, 13867, 13292, 13024, 11954, 12329, 11745, 11579, 10804, 10072, 9853, 9210, 8572, 7859, 7469, 7108, 6462, 5904, 5538, 5007, 4375, 3997, 3496, 2720, 2306, 1607, 1116, 857, 491, 407, 287, 271, 200, 193, 163, 172, 
47980, 44500, 42491, 39025, 35216, 32586, 30855, 30639, 28574, 26289, 26050, 23926, 23495, 21793, 21313, 20063, 20191, 18403, 18265, 17645, 17290, 16195, 15433, 15230, 14983, 14016, 14041, 13721, 13148, 13094, 13455, 12900, 11704, 11692, 11298, 10667, 10115, 9706, 8741, 8469, 8200, 7640, 7148, 6493, 6371, 5790, 5411, 5044, 4264, 4039, 3384, 2902, 2175, 1811, 1177, 808, 481, 386, 268, 218, 206, 180, 171, 186, 
46842, 41925, 39495, 36341, 33088, 30553, 29032, 27983, 26271, 26120, 24173, 23065, 21979, 20846, 20462, 18833, 18342, 18283, 17488, 16239, 15960, 15775, 14835, 14604, 14087, 14026, 13477, 13438, 13852, 12684, 12702, 12703, 11562, 11419, 11277, 10579, 10197, 9275, 9056, 8322, 7755, 7697, 6930, 6422, 6242, 5538, 5294, 4865, 4233, 3825, 3267, 2559, 2182, 1628, 1222, 781, 441, 365, 273, 193, 171, 191, 170, 165, 
43982, 40194, 37394, 34376, 31312, 29178, 27715, 26372, 25112, 23836, 23055, 21454, 20900, 20379, 19944, 18350, 17753, 17709, 16527, 16228, 15596, 14812, 14598, 14237, 13654, 14026, 13118, 12899, 12537, 12568, 12261, 12151, 11220, 11502, 10709, 10276, 9612, 9287, 8510, 8213, 7740, 7085, 6760, 6449, 5950, 5508, 5074, 4424, 3966, 3712, 3056, 2557, 2026, 1482, 1044, 698, 372, 320, 267, 184, 183, 173, 156, 163, 
42578, 37911, 35131, 32351, 30142, 27874, 26205, 24803, 23287, 22364, 21396, 20562, 20120, 18778, 17871, 17305, 16793, 16205, 15604, 14821, 14753, 14261, 13624, 13802, 13931, 13118, 12808, 12542, 12121, 12292, 11672, 11817, 10566, 10971, 10385, 9956, 9685, 9014, 8267, 7952, 7438, 7029, 6641, 6334, 5633, 5382, 5074, 4576, 3962, 3568, 3020, 2363, 1834, 1433, 1050, 703, 439, 282, 276, 201, 219, 168, 147, 135, 
40507, 35866, 34131, 31143, 28708, 26762, 24872, 24087, 22234, 21334, 20538, 19624, 19160, 17845, 17484, 16792, 16362, 15674, 15201, 14368, 14146, 13721, 13211, 13006, 13064, 12754, 12637, 12092, 12234, 11608, 11489, 11270, 10361, 10358, 10026, 9769, 8867, 8534, 7955, 7811, 7299, 6862, 6261, 6105, 5557, 4960, 4840, 4492, 3876, 3359, 2973, 2367, 1836, 1349, 991, 694, 407, 281, 228, 261, 171, 144, 145, 153, 
38707, 34256, 32982, 28670, 26123, 24720, 23507, 22128, 21397, 20576, 19324, 19117, 17709, 17471, 16716, 16013, 15437, 15503, 14645, 14250, 13709, 13262, 12664, 12322, 12309, 12369, 11974, 11502, 11451, 11358, 11092, 10875, 10276, 10184, 9961, 9393, 9095, 8421, 7954, 7482, 6978, 7009, 6170, 5893, 5588, 4937, 4883, 4338, 3804, 3369, 2909, 2340, 1791, 1326, 904, 611, 364, 282, 202, 214, 182, 121, 143, 154, 
36684, 32804, 30710, 27891, 25288, 23289, 21959, 21690, 20496, 19041, 18718, 17913, 17152, 16668, 16155, 15416, 14885, 14295, 14035, 13600, 13308, 12753, 12213, 11874, 12326, 11995, 11415, 11543, 11302, 10911, 10653, 11017, 9858, 10119, 9862, 9193, 8796, 8369, 7959, 7347, 6997, 6747, 6239, 5951, 5331, 5237, 4603, 4248, 3699, 3357, 2778, 2391, 1740, 1315, 866, 583, 369, 259, 206, 246, 226, 186, 131, 136, 
35723, 31175, 29509, 26660, 23957, 22532, 21398, 20442, 19428, 18431, 17724, 17208, 16477, 15955, 15440, 14997, 14095, 13715, 13883, 13028, 12860, 12749, 11991, 11793, 11661, 11513, 11443, 11034, 11402, 11307, 10594, 10408, 9763, 9968, 9348, 8843, 8381, 8129, 7775, 7148, 6765, 6353, 5982, 5761, 5455, 4928, 4505, 4033, 3545, 3119, 2726, 2248, 1769, 1226, 791, 599, 353, 275, 210, 162, 158, 197, 144, 160, 
34020, 31067, 28268, 25833, 23836, 21766, 20510, 20103, 18819, 17614, 17625, 16899, 15634, 14841, 14416, 14456, 13378, 13411, 13035, 12795, 12735, 11846, 11569, 11776, 11325, 11109, 10838, 11039, 10734, 11097, 10241, 10116, 9318, 9597, 9406, 9007, 8257, 7968, 7531, 7217, 6680, 6205, 5834, 5685, 5230, 4724, 4508, 3928, 3478, 3229, 2656, 2187, 1705, 1261, 863, 554, 367, 261, 191, 157, 152, 147, 184, 126, 
33454, 30410, 28138, 25447, 22570, 20845, 19936, 18915, 17861, 16952, 16640, 15774, 15436, 14534, 14215, 13507, 13194, 12914, 12606, 12574, 12296, 11523, 11460, 11438, 11119, 10742, 10809, 10838, 10511, 10378, 10199, 9918, 9146, 9372, 8874, 8492, 8147, 7738, 7235, 7236, 6306, 5994, 5896, 5365, 5070, 4691, 4160, 3908, 3280, 3185, 2568, 2179, 1626, 1172, 816, 540, 305, 222, 194, 183, 154, 141, 131, 133, 
31092, 28588, 26514, 23652, 21756, 20443, 18713, 18398, 17015, 16245, 15933, 15277, 14947, 14426, 13618, 13394, 13002, 12382, 11706, 11580, 11580, 11113, 11119, 10961, 10614, 10695, 10433, 10556, 10453, 10181, 9967, 9833, 9210, 9182, 8846, 8309, 7982, 7628, 7256, 6759, 6491, 6101, 5482, 5286, 5153, 4637, 4272, 3959, 3414, 3115, 2594, 1965, 1581, 1173, 753, 513, 342, 219, 157, 149, 150, 144, 148, 142, 
31507, 27277, 25526, 22668, 21045, 19380, 18278, 17765, 16428, 15860, 15778, 14793, 14271, 13970, 13705, 12744, 12416, 12120, 11418, 11791, 11564, 11443, 10736, 10618, 10508, 10550, 10342, 10344, 10242, 9971, 9816, 9613, 8693, 8980, 8819, 8282, 7849, 7379, 7005, 6691, 6167, 5953, 5727, 5244, 4851, 4503, 4212, 3820, 3286, 3017, 2453, 2008, 1566, 1162, 836, 510, 318, 215, 169, 123, 127, 114, 116, 141, 
29992, 26603, 24894, 22427, 20471, 18895, 17917, 17418, 16102, 15552, 15563, 14773, 14420, 13969, 13334, 12700, 12261, 11730, 11701, 11219, 10951, 11100, 10905, 10280, 10302, 10027, 9865, 10021, 10051, 9847, 9608, 9328, 8863, 8622, 8506, 7939, 7410, 7205, 6634, 6446, 5886, 5517, 5351, 5021, 4757, 4335, 4043, 3634, 3164, 2869, 2449, 1970, 1508, 1058, 697, 481, 277, 197, 145, 136, 124, 114, 109, 117, 
29961, 25129, 23164, 21366, 19943, 18342, 17283, 16607, 15899, 15108, 14850, 14386, 13785, 13547, 12933, 12370, 12358, 11632, 11592, 11292, 11050, 10567, 10407, 10219, 9947, 10310, 9920, 10153, 10054, 9881, 9474, 9494, 8839, 8880, 8413, 8017, 7817, 7267, 6981, 6421, 5937, 5738, 5429, 5177, 4637, 4463, 4095, 3679, 3233, 2912, 2475, 2068, 1517, 1106, 741, 490, 272, 200, 161, 137, 129, 123, 117, 119, 
27099, 23886, 22523, 20631, 19275, 17606, 16473, 15781, 15702, 14989, 14398, 14072, 13082, 12971, 12724, 12307, 11616, 11324, 11268, 11127, 10694, 10389, 10204, 9942, 10269, 9905, 9823, 9560, 9639, 9478, 9311, 9479, 8478, 8684, 8498, 8050, 7790, 7256, 6761, 6230, 5995, 5647, 5260, 4929, 4536, 4442, 4060, 3688, 3312, 2802, 2400, 1951, 1466, 1058, 723, 467, 291, 185, 180, 145, 109, 145, 121, 115, 
28573, 23165, 22160, 20037, 18413, 17254, 16227, 15723, 15230, 14739, 14266, 13327, 13156, 12489, 12108, 12016, 11611, 11420, 11157, 11002, 11387, 10632, 10089, 9996, 9844, 9876, 9447, 9485, 9612, 9383, 9447, 9023, 8332, 8723, 8350, 7866, 7497, 7284, 6625, 6181, 5785, 5720, 5218, 5013, 4553, 4438, 3968, 3681, 3191, 2856, 2340, 1942, 1419, 1022, 747, 486, 258, 192, 159, 143, 125, 115, 109, 120, 
24897, 22508, 21788, 19942, 18231, 16825, 16000, 15439, 14911, 14058, 13823, 13585, 12803, 12574, 12367, 11903, 11400, 11051, 11098, 11630, 12790, 11580, 9939, 9838, 9890, 9722, 9723, 9370, 9702, 9216, 9183, 9032, 8324, 8590, 8471, 7745, 7385, 7030, 6600, 6292, 5885, 5582, 5209, 5136, 4781, 4232, 4032, 3635, 3138, 2763, 2292, 1886, 1496, 1035, 722, 511, 247, 195, 141, 129, 119, 120, 114, 110, 
31866, 22438, 21494, 19442, 17572, 16476, 15855, 15045, 14405, 13871, 13720, 13214, 12754, 12344, 11651, 11761, 11438, 11248, 10672, 10778, 11328, 10786, 9840, 9705, 9503, 9889, 9423, 9751, 9480, 9300, 9187, 9063, 8523, 8404, 8132, 7830, 7487, 6922, 6549, 6252, 5973, 5396, 5275, 4976, 4761, 4427, 3960, 3703, 3080, 2819, 2426, 1920, 1464, 1062, 699, 427, 266, 212, 152, 120, 117, 124, 113, 116, 
24616, 22447, 21972, 19449, 17845, 16338, 15302, 15117, 14236, 13854, 13580, 12869, 12500, 11978, 11756, 11559, 11269, 10757, 10425, 10276, 10007, 9805, 9759, 9783, 9727, 9727, 9541, 9560, 9342, 9333, 9237, 9113, 8278, 8406, 8107, 7903, 7382, 7027, 6606, 6301, 5721, 5500, 5205, 4836, 4674, 4276, 3894, 3561, 3048, 2843, 2398, 1902, 1468, 1030, 672, 479, 260, 171, 146, 121, 114, 107, 134, 127, 
80381, 21886, 21009, 19239, 17571, 16114, 15126, 14700, 14271, 13664, 13061, 12880, 12415, 12003, 11802, 11703, 11231, 11078, 10689, 10454, 10183, 9895, 9559, 9911, 9583, 9632, 9422, 9183, 9300, 9150, 9162, 9098, 8125, 8667, 8390, 7729, 7461, 7004, 6491, 6449, 5729, 5619, 5194, 4908, 4642, 4157, 3921, 3565, 3308, 2804, 2353, 1923, 1452, 1011, 661, 483, 269, 197, 154, 134, 118, 128, 99, 131, },
{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
923, 529, 391, 296, 240, 204, 162, 139, 121, 95, 99, 79, 64, 49, 45, 41, 45, 36, 29, 32, 30, 21, 22, 24, 20, 18, 16, 14, 17, 14, 12, 12, 12, 13, 9, 12, 15, 11, 7, 11, 11, 7, 9, 8, 8, 6, 7, 6, 5, 4, 4, 5, 3, 3, 2, 6, 1, 4, 3, 13, 2, 48, 7, 188, 
532, 410, 347, 246, 211, 168, 143, 127, 123, 95, 76, 67, 67, 50, 50, 42, 35, 31, 25, 34, 30, 25, 23, 20, 20, 18, 13, 19, 13, 11, 9, 15, 11, 12, 7, 14, 10, 13, 8, 9, 6, 9, 6, 4, 8, 11, 5, 7, 2, 1, 2, 3, 4, 1, 2, 0, 2, 0, 0, 0, 2, 6, 1, 8, 
422, 356, 325, 252, 217, 164, 139, 131, 108, 91, 78, 81, 69, 53, 48, 40, 37, 40, 23, 24, 18, 21, 19, 21, 17, 20, 17, 15, 9, 11, 10, 9, 11, 11, 11, 9, 10, 11, 13, 11, 9, 9, 7, 3, 8, 9, 5, 4, 5, 5, 2, 5, 2, 1, 3, 2, 0, 4, 0, 0, 1, 1, 3, 4, 
351, 324, 289, 267, 228, 172, 143, 127, 104, 88, 82, 73, 62, 44, 41, 44, 37, 25, 32, 29, 20, 23, 21, 23, 16, 20, 16, 18, 14, 16, 14, 8, 11, 14, 9, 12, 12, 11, 4, 6, 9, 7, 8, 3, 8, 6, 6, 5, 3, 3, 3, 2, 4, 3, 2, 2, 3, 4, 2, 2, 0, 0, 3, 2, 
312, 264, 267, 222, 188, 172, 133, 115, 92, 100, 71, 70, 60, 53, 39, 36, 27, 31, 29, 29, 21, 22, 25, 19, 18, 19, 19, 12, 11, 12, 9, 15, 10, 13, 9, 10, 3, 6, 8, 5, 8, 5, 5, 5, 8, 6, 6, 4, 3, 2, 4, 4, 2, 3, 2, 5, 1, 0, 0, 0, 1, 2, 2, 1, 
247, 245, 216, 190, 167, 168, 134, 111, 88, 77, 62, 55, 60, 51, 44, 33, 31, 26, 21, 21, 26, 16, 17, 17, 21, 20, 16, 13, 12, 15, 11, 10, 12, 9, 10, 7, 11, 5, 6, 9, 9, 6, 4, 6, 7, 7, 4, 5, 4, 3, 2, 3, 5, 1, 3, 2, 0, 2, 0, 0, 3, 1, 0, 1, 
223, 193, 203, 179, 158, 150, 154, 123, 102, 77, 61, 70, 44, 38, 39, 34, 31, 24, 26, 23, 24, 25, 20, 11, 19, 21, 17, 14, 15, 13, 13, 10, 9, 7, 9, 7, 8, 7, 8, 6, 6, 7, 3, 4, 4, 6, 5, 7, 5, 6, 6, 3, 3, 3, 2, 3, 1, 0, 0, 0, 0, 2, 0, 3, 
198, 186, 179, 170, 145, 143, 116, 101, 89, 72, 63, 60, 42, 43, 29, 34, 31, 20, 26, 26, 19, 21, 14, 12, 19, 20, 17, 12, 15, 7, 10, 10, 8, 6, 10, 8, 11, 8, 6, 7, 6, 5, 7, 3, 5, 4, 6, 2, 1, 3, 3, 1, 3, 2, 1, 3, 1, 0, 0, 0, 0, 1, 0, 1, 
169, 137, 152, 139, 128, 113, 99, 96, 88, 58, 61, 45, 38, 39, 33, 31, 28, 21, 23, 27, 18, 18, 19, 15, 11, 18, 16, 17, 15, 9, 12, 12, 5, 7, 9, 10, 4, 5, 6, 9, 8, 5, 7, 5, 6, 0, 8, 6, 3, 4, 6, 4, 2, 3, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 
147, 132, 108, 112, 110, 103, 77, 83, 67, 77, 55, 40, 40, 34, 27, 33, 29, 25, 22, 21, 22, 18, 13, 15, 23, 9, 14, 16, 18, 11, 12, 9, 9, 3, 12, 10, 7, 5, 7, 5, 6, 9, 8, 3, 6, 2, 4, 2, 4, 4, 4, 4, 3, 4, 4, 1, 0, 2, 0, 0, 0, 0, 0, 0, 
121, 114, 109, 104, 95, 83, 67, 70, 56, 55, 63, 44, 39, 30, 31, 24, 21, 24, 22, 18, 20, 18, 15, 19, 8, 18, 15, 15, 16, 13, 8, 11, 5, 3, 6, 8, 6, 10, 7, 6, 9, 6, 8, 4, 7, 4, 5, 2, 5, 6, 6, 4, 4, 4, 3, 3, 2, 0, 0, 0, 0, 0, 0, 0, 
109, 107, 88, 89, 73, 69, 75, 65, 56, 52, 47, 40, 46, 30, 28, 25, 21, 25, 25, 20, 19, 15, 18, 15, 17, 14, 11, 15, 15, 17, 9, 9, 11, 9, 6, 10, 3, 7, 9, 5, 3, 9, 8, 4, 3, 6, 3, 4, 3, 3, 5, 5, 0, 3, 3, 1, 4, 1, 0, 0, 0, 0, 0, 0, 
93, 89, 82, 76, 70, 65, 67, 56, 47, 46, 41, 41, 42, 47, 25, 21, 25, 26, 28, 17, 18, 15, 14, 14, 14, 15, 13, 13, 14, 14, 13, 6, 7, 7, 3, 8, 10, 7, 3, 9, 3, 6, 3, 4, 6, 6, 5, 6, 7, 3, 1, 3, 3, 1, 3, 2, 0, 2, 1, 0, 0, 0, 0, 0, 
79, 70, 71, 61, 62, 58, 52, 42, 41, 30, 38, 34, 33, 39, 39, 18, 26, 29, 27, 14, 18, 24, 17, 12, 12, 11, 12, 8, 12, 15, 13, 13, 7, 5, 7, 11, 11, 7, 5, 7, 7, 6, 5, 6, 4, 5, 3, 7, 4, 4, 2, 2, 1, 4, 0, 3, 2, 1, 1, 0, 0, 0, 0, 0, 
74, 53, 64, 58, 50, 58, 46, 47, 36, 35, 32, 36, 29, 30, 34, 29, 33, 24, 24, 15, 18, 15, 13, 13, 13, 14, 13, 8, 10, 12, 12, 10, 8, 10, 8, 5, 9, 8, 7, 8, 6, 5, 7, 6, 3, 5, 5, 4, 6, 3, 3, 3, 5, 3, 4, 2, 1, 1, 0, 1, 0, 0, 0, 0, 
68, 54, 55, 54, 46, 51, 44, 33, 29, 32, 36, 32, 28, 31, 36, 35, 27, 22, 19, 14, 12, 13, 11, 11, 11, 15, 17, 6, 12, 13, 13, 13, 9, 6, 6, 4, 10, 9, 7, 8, 7, 5, 6, 1, 6, 5, 2, 2, 5, 4, 3, 6, 2, 2, 0, 3, 2, 0, 0, 0, 0, 0, 0, 0, 
60, 44, 43, 51, 45, 43, 40, 38, 32, 29, 26, 30, 29, 24, 26, 25, 34, 22, 18, 21, 10, 16, 13, 10, 10, 9, 14, 12, 13, 11, 9, 9, 12, 4, 7, 4, 7, 4, 6, 4, 4, 4, 5, 5, 4, 6, 4, 5, 4, 5, 3, 5, 2, 3, 1, 3, 0, 1, 0, 0, 1, 0, 0, 0, 
45, 46, 40, 32, 37, 39, 44, 30, 34, 36, 25, 23, 31, 23, 24, 27, 18, 27, 25, 20, 14, 16, 9, 10, 9, 11, 11, 13, 13, 12, 8, 12, 13, 10, 7, 6, 6, 7, 5, 4, 8, 6, 6, 7, 3, 3, 4, 6, 2, 3, 6, 1, 1, 3, 3, 2, 3, 0, 0, 0, 0, 0, 0, 0, 
48, 40, 38, 35, 36, 42, 33, 27, 28, 29, 23, 27, 22, 24, 22, 26, 24, 25, 26, 26, 20, 18, 13, 8, 11, 11, 13, 10, 10, 14, 12, 8, 12, 8, 6, 8, 6, 6, 8, 5, 3, 3, 6, 7, 4, 3, 2, 2, 4, 3, 2, 2, 2, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
39, 42, 37, 31, 29, 35, 28, 27, 23, 25, 23, 26, 18, 27, 21, 21, 19, 9, 26, 22, 17, 15, 10, 9, 13, 7, 11, 8, 8, 8, 6, 6, 12, 12, 8, 8, 4, 6, 6, 6, 8, 6, 3, 4, 3, 5, 3, 3, 3, 2, 4, 5, 4, 3, 2, 2, 4, 0, 0, 0, 0, 0, 0, 0, 
42, 38, 38, 34, 33, 31, 28, 30, 31, 27, 23, 16, 14, 19, 19, 20, 20, 11, 17, 13, 18, 14, 9, 10, 13, 8, 12, 10, 7, 6, 11, 11, 4, 10, 10, 7, 6, 8, 9, 5, 6, 3, 4, 2, 4, 1, 3, 2, 5, 3, 3, 2, 2, 2, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 
40, 31, 45, 30, 29, 24, 35, 28, 25, 17, 22, 21, 17, 21, 15, 21, 18, 14, 17, 13, 13, 16, 15, 12, 10, 12, 4, 11, 7, 13, 8, 11, 10, 9, 10, 6, 5, 8, 3, 3, 6, 4, 6, 4, 3, 5, 1, 3, 4, 3, 6, 3, 2, 0, 3, 1, 1, 0, 0, 0, 0, 0, 0, 0, 
29, 25, 31, 31, 36, 28, 24, 18, 26, 17, 22, 15, 22, 14, 15, 18, 10, 15, 12, 11, 14, 9, 9, 16, 13, 7, 11, 6, 7, 4, 9, 7, 5, 7, 7, 7, 6, 4, 3, 3, 3, 3, 3, 5, 3, 4, 2, 4, 2, 3, 3, 4, 2, 3, 2, 1, 0, 1, 0, 0, 0, 0, 0, 0, 
35, 23, 27, 33, 25, 20, 26, 19, 25, 19, 18, 20, 14, 17, 10, 12, 12, 10, 20, 13, 12, 11, 12, 12, 16, 6, 8, 11, 8, 8, 8, 7, 9, 5, 8, 8, 7, 9, 5, 4, 6, 4, 5, 4, 6, 5, 5, 2, 1, 1, 3, 4, 1, 2, 1, 0, 2, 0, 1, 0, 0, 0, 0, 0, 
34, 25, 32, 23, 18, 18, 14, 18, 15, 21, 14, 25, 18, 14, 11, 11, 10, 15, 12, 12, 11, 9, 10, 12, 16, 12, 9, 5, 5, 7, 6, 10, 8, 3, 7, 9, 8, 3, 5, 4, 5, 4, 3, 3, 3, 4, 1, 5, 4, 4, 3, 1, 2, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 
31, 25, 27, 18, 20, 13, 16, 18, 17, 18, 19, 18, 18, 12, 23, 13, 7, 11, 10, 11, 8, 11, 9, 7, 10, 11, 7, 9, 10, 6, 4, 5, 4, 4, 4, 7, 6, 7, 2, 3, 4, 5, 6, 5, 5, 3, 3, 3, 5, 3, 3, 5, 0, 4, 3, 1, 0, 1, 0, 0, 0, 0, 0, 0, 
31, 27, 17, 30, 16, 23, 12, 17, 17, 22, 21, 14, 15, 14, 16, 10, 10, 11, 13, 10, 9, 12, 9, 9, 9, 12, 11, 14, 7, 6, 5, 6, 5, 8, 10, 6, 7, 8, 7, 6, 4, 4, 7, 1, 4, 3, 3, 4, 3, 3, 3, 2, 2, 2, 2, 3, 0, 1, 0, 0, 0, 0, 0, 0, 
32, 23, 21, 17, 19, 18, 13, 16, 14, 15, 20, 14, 15, 10, 18, 13, 9, 9, 9, 7, 8, 12, 6, 8, 9, 8, 15, 9, 11, 6, 2, 3, 7, 6, 4, 5, 6, 5, 2, 3, 6, 5, 5, 5, 3, 5, 0, 4, 4, 2, 3, 1, 1, 2, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 
29, 25, 20, 18, 15, 18, 14, 14, 11, 19, 16, 20, 10, 8, 16, 11, 8, 8, 7, 6, 8, 11, 10, 9, 6, 6, 12, 14, 11, 9, 7, 4, 9, 6, 7, 4, 6, 3, 2, 5, 3, 3, 4, 3, 4, 3, 2, 3, 2, 4, 1, 0, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
26, 19, 21, 16, 13, 15, 14, 14, 16, 11, 17, 17, 16, 10, 9, 11, 11, 8, 7, 7, 9, 7, 7, 7, 8, 9, 8, 8, 8, 10, 7, 9, 5, 3, 5, 5, 6, 8, 4, 4, 3, 4, 5, 2, 3, 3, 3, 1, 2, 1, 2, 2, 0, 2, 2, 1, 2, 1, 0, 0, 0, 0, 0, 0, 
17, 11, 26, 14, 12, 11, 14, 11, 15, 13, 19, 16, 18, 9, 8, 12, 6, 6, 8, 6, 9, 8, 8, 9, 6, 9, 3, 4, 5, 7, 10, 7, 6, 6, 6, 4, 4, 6, 4, 4, 2, 4, 4, 2, 5, 4, 2, 3, 3, 1, 5, 2, 1, 2, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 
19, 19, 22, 15, 12, 9, 4, 14, 11, 13, 11, 13, 19, 12, 7, 10, 9, 10, 7, 6, 6, 7, 5, 7, 7, 9, 4, 10, 7, 6, 9, 5, 8, 6, 4, 7, 9, 5, 5, 3, 2, 2, 1, 3, 2, 5, 1, 1, 2, 2, 2, 3, 3, 2, 1, 2, 1, 0, 0, 1, 0, 0, 0, 0, 
17, 14, 15, 12, 13, 11, 10, 11, 10, 9, 8, 12, 16, 15, 7, 11, 6, 11, 4, 6, 5, 7, 6, 3, 3, 6, 5, 6, 6, 5, 6, 6, 3, 5, 7, 6, 6, 6, 5, 2, 3, 2, 4, 2, 3, 4, 1, 5, 2, 1, 4, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 
17, 18, 9, 9, 13, 12, 11, 10, 10, 8, 9, 9, 14, 11, 7, 10, 9, 10, 8, 7, 8, 5, 7, 6, 6, 3, 7, 5, 4, 6, 5, 2, 5, 5, 6, 8, 2, 6, 1, 4, 6, 2, 6, 4, 4, 0, 6, 4, 3, 2, 1, 1, 4, 3, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 
19, 18, 13, 15, 12, 12, 12, 11, 9, 12, 6, 10, 12, 10, 10, 10, 5, 8, 8, 5, 6, 10, 5, 5, 3, 5, 7, 6, 5, 6, 4, 4, 2, 5, 5, 5, 3, 4, 4, 4, 1, 4, 5, 5, 0, 2, 5, 2, 3, 2, 1, 2, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 
13, 12, 14, 8, 7, 10, 7, 8, 11, 8, 7, 10, 7, 9, 8, 6, 8, 5, 11, 6, 6, 7, 6, 6, 3, 6, 8, 5, 6, 4, 5, 4, 5, 3, 8, 3, 8, 4, 3, 2, 4, 6, 3, 2, 4, 2, 5, 3, 3, 2, 2, 2, 1, 1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 
15, 12, 13, 10, 8, 10, 8, 9, 6, 11, 13, 5, 13, 4, 5, 5, 7, 4, 6, 6, 5, 4, 6, 6, 3, 5, 7, 4, 3, 4, 3, 4, 3, 1, 4, 5, 7, 3, 1, 6, 2, 3, 5, 5, 3, 5, 0, 2, 2, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
14, 12, 13, 12, 11, 7, 8, 10, 11, 9, 8, 6, 5, 9, 7, 7, 7, 5, 3, 7, 4, 5, 4, 5, 5, 5, 5, 2, 5, 3, 5, 5, 5, 3, 4, 7, 6, 3, 4, 2, 3, 3, 3, 2, 2, 2, 3, 5, 0, 3, 1, 1, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 
13, 11, 7, 8, 12, 10, 8, 7, 8, 4, 5, 7, 7, 8, 6, 8, 7, 6, 4, 4, 4, 5, 5, 3, 4, 6, 6, 5, 4, 4, 6, 6, 6, 2, 2, 5, 5, 4, 5, 3, 6, 5, 2, 2, 0, 2, 1, 1, 3, 1, 2, 1, 1, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 
7, 9, 8, 5, 8, 9, 11, 6, 4, 10, 4, 8, 9, 7, 3, 7, 5, 8, 4, 6, 5, 3, 4, 4, 6, 5, 3, 2, 3, 4, 5, 4, 1, 4, 3, 2, 5, 1, 5, 2, 3, 2, 2, 0, 3, 1, 3, 0, 0, 3, 3, 3, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
16, 14, 15, 8, 3, 7, 7, 7, 6, 8, 7, 6, 3, 8, 6, 6, 5, 4, 6, 7, 4, 2, 5, 3, 4, 5, 3, 7, 3, 5, 2, 5, 3, 3, 5, 3, 4, 2, 2, 4, 2, 3, 3, 4, 1, 0, 4, 0, 2, 0, 3, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
10, 10, 15, 6, 4, 8, 6, 2, 6, 7, 6, 8, 6, 5, 5, 8, 7, 4, 4, 5, 5, 3, 4, 8, 3, 4, 2, 2, 5, 5, 4, 8, 6, 3, 5, 2, 2, 4, 2, 1, 4, 4, 2, 3, 3, 2, 1, 1, 0, 2, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
13, 11, 9, 7, 6, 8, 4, 6, 8, 6, 7, 3, 6, 6, 4, 5, 8, 7, 4, 5, 4, 5, 2, 4, 3, 5, 4, 4, 7, 3, 3, 3, 5, 2, 4, 5, 4, 2, 3, 2, 0, 2, 2, 4, 2, 3, 1, 1, 2, 0, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 11, 7, 9, 7, 7, 3, 4, 8, 8, 5, 2, 6, 2, 3, 6, 9, 6, 8, 4, 6, 4, 4, 6, 3, 4, 3, 5, 2, 2, 2, 4, 3, 2, 5, 4, 3, 3, 1, 2, 2, 2, 2, 3, 4, 4, 2, 1, 2, 1, 0, 3, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 11, 11, 8, 5, 7, 5, 5, 7, 4, 4, 4, 8, 6, 7, 7, 6, 6, 7, 5, 4, 3, 6, 6, 5, 5, 5, 1, 5, 2, 2, 4, 4, 2, 2, 4, 3, 1, 3, 6, 2, 0, 1, 1, 1, 2, 0, 1, 0, 1, 2, 1, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 12, 10, 8, 6, 5, 5, 7, 3, 6, 6, 6, 6, 5, 8, 7, 11, 10, 8, 4, 5, 7, 2, 4, 3, 3, 4, 3, 5, 4, 3, 4, 3, 3, 2, 2, 4, 2, 5, 3, 3, 3, 3, 1, 1, 0, 2, 1, 0, 0, 2, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
11, 13, 9, 8, 3, 5, 5, 7, 6, 7, 2, 3, 5, 6, 4, 4, 6, 7, 9, 5, 4, 6, 7, 4, 3, 5, 5, 4, 2, 5, 2, 3, 2, 3, 4, 1, 3, 2, 1, 3, 2, 0, 2, 0, 2, 4, 2, 1, 2, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 
7, 10, 7, 7, 5, 7, 3, 3, 7, 2, 6, 6, 4, 4, 4, 7, 2, 9, 9, 5, 5, 5, 4, 2, 3, 5, 0, 1, 1, 2, 3, 2, 2, 3, 5, 1, 3, 3, 4, 2, 3, 2, 1, 1, 2, 2, 0, 2, 1, 2, 1, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
9, 10, 8, 3, 6, 6, 5, 3, 4, 5, 4, 2, 6, 2, 5, 4, 2, 4, 9, 4, 4, 3, 5, 4, 4, 2, 2, 6, 2, 3, 4, 5, 0, 1, 2, 4, 4, 2, 4, 4, 1, 1, 1, 2, 0, 0, 2, 3, 2, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 
7, 6, 8, 7, 3, 4, 6, 6, 5, 8, 6, 4, 1, 3, 4, 6, 4, 5, 3, 4, 4, 4, 3, 5, 2, 1, 3, 4, 3, 2, 5, 3, 2, 1, 4, 2, 3, 4, 2, 1, 1, 1, 0, 2, 2, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 9, 5, 4, 6, 7, 5, 4, 5, 4, 7, 5, 4, 4, 1, 2, 3, 4, 5, 6, 3, 4, 3, 2, 4, 3, 2, 4, 3, 5, 5, 4, 1, 2, 2, 2, 3, 0, 3, 2, 1, 0, 1, 0, 1, 0, 0, 1, 2, 1, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
11, 6, 9, 4, 5, 4, 3, 3, 5, 7, 3, 4, 3, 2, 5, 4, 5, 4, 6, 5, 4, 2, 4, 1, 1, 5, 2, 3, 3, 1, 2, 2, 2, 1, 3, 3, 0, 1, 3, 2, 2, 1, 1, 4, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
9, 6, 7, 6, 4, 5, 3, 3, 7, 4, 6, 4, 5, 2, 5, 4, 4, 5, 7, 3, 2, 4, 3, 4, 2, 1, 1, 6, 3, 3, 2, 0, 1, 3, 1, 3, 1, 5, 0, 1, 1, 2, 1, 1, 0, 1, 1, 1, 2, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
8, 3, 8, 7, 3, 4, 3, 4, 8, 4, 3, 2, 5, 3, 4, 1, 5, 4, 3, 3, 4, 3, 3, 3, 0, 3, 2, 1, 3, 2, 2, 1, 1, 1, 2, 4, 2, 1, 3, 1, 1, 3, 0, 0, 0, 1, 1, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
9, 3, 5, 2, 1, 6, 3, 5, 3, 4, 4, 4, 3, 4, 4, 6, 4, 4, 2, 1, 2, 3, 3, 4, 2, 7, 1, 3, 1, 2, 1, 3, 1, 0, 1, 0, 2, 0, 2, 1, 1, 0, 1, 0, 2, 1, 1, 0, 1, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 6, 5, 3, 3, 7, 2, 1, 5, 3, 3, 4, 1, 5, 2, 4, 1, 1, 2, 2, 2, 3, 2, 3, 5, 4, 3, 1, 3, 3, 1, 3, 1, 2, 1, 2, 1, 2, 2, 3, 0, 0, 2, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 
6, 5, 6, 5, 6, 4, 5, 2, 6, 3, 6, 1, 4, 4, 3, 3, 1, 5, 2, 1, 3, 1, 3, 3, 2, 4, 2, 2, 1, 2, 1, 1, 1, 4, 1, 4, 2, 0, 1, 3, 2, 0, 0, 2, 0, 0, 1, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 2, 4, 4, 3, 5, 6, 0, 2, 3, 3, 6, 5, 2, 5, 3, 4, 3, 6, 3, 1, 3, 1, 4, 4, 2, 3, 1, 0, 3, 0, 2, 0, 1, 0, 1, 2, 1, 1, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
9, 3, 1, 2, 3, 3, 3, 2, 2, 6, 3, 4, 3, 4, 2, 2, 3, 4, 0, 2, 3, 1, 1, 3, 1, 1, 2, 1, 2, 0, 2, 4, 1, 1, 3, 1, 1, 0, 0, 1, 3, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 4, 4, 4, 3, 4, 4, 2, 3, 4, 3, 4, 2, 3, 1, 3, 4, 2, 3, 3, 2, 2, 4, 4, 4, 1, 3, 0, 2, 3, 1, 1, 1, 1, 2, 2, 0, 1, 0, 1, 1, 1, 0, 1, 0, 2, 2, 1, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
4, 6, 3, 4, 4, 2, 4, 2, 4, 5, 5, 4, 3, 3, 3, 2, 2, 2, 0, 2, 4, 2, 3, 5, 3, 2, 1, 3, 1, 2, 2, 0, 1, 0, 2, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
6, 3, 2, 4, 1, 1, 4, 2, 2, 3, 5, 5, 5, 2, 2, 5, 2, 2, 1, 4, 2, 3, 2, 2, 1, 2, 1, 3, 2, 0, 2, 2, 0, 0, 0, 1, 1, 2, 2, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
5, 6, 3, 3, 1, 2, 4, 3, 5, 3, 5, 1, 4, 3, 3, 2, 3, 1, 2, 2, 5, 0, 2, 3, 5, 3, 2, 1, 0, 0, 0, 1, 3, 1, 2, 3, 0, 2, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
11, 4, 3, 3, 3, 2, 5, 1, 2, 4, 2, 2, 4, 3, 6, 1, 0, 2, 3, 0, 1, 2, 1, 2, 1, 4, 3, 3, 3, 1, 3, 2, 0, 0, 1, 2, 0, 1, 2, 1, 1, 1, 1, 2, 0, 0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
{
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

};

static UInt FIX_SCANSTATS32x32[9][1024] =
{
	{504836, 282871, 167433, 110988, 64358, 49213, 36307, 26366, 22536, 19811, 16598, 12471, 11241, 9863, 8570, 8002, 7769, 6544, 7214, 6647, 4991, 4503, 3408, 3412, 2031, 2525, 1348, 3263, 790, 8164, 507, 43221, 349395, 187122, 121082, 80527, 47751, 33055, 25349, 18309, 13027, 11352, 9519, 7087, 6280, 5583, 4982, 4567, 4512, 4172, 4443, 3820, 3429, 2761, 2150, 1783, 1319, 1088, 827, 663, 536, 503, 342, 838, 283710, 146470, 97604, 65690, 36828, 27634, 21518, 15473, 10921, 9261, 7384, 6032, 5159, 4933, 4261, 4040, 4129, 3722, 3706, 3468, 3216, 2288, 1912, 1425, 1099, 867, 755, 628, 457, 374, 412, 455, 221021, 125440, 82743, 54838, 31702, 23274, 16747, 11933, 8609, 7384, 6432, 5214, 4377, 4105, 3870, 3673, 3274, 3389, 2974, 2919, 2700, 2109, 1528, 1194, 982, 769, 703, 514, 373, 349, 265, 330, 176507, 94513, 65759, 41877, 22730, 16790, 12027, 8975, 7208, 6408, 5459, 4387, 3628, 3414, 3171, 3124, 2866, 2863, 2559, 2572, 2084, 1775, 1347, 1163, 882, 743, 656, 499, 382, 273, 192, 241, 142900, 79918, 54051, 33610, 18072, 13420, 9356, 7658, 6294, 5296, 4514, 4041, 3391, 3095, 3075, 2989, 2816, 2931, 2685, 2441, 2028, 1660, 1317, 1051, 878, 736, 655, 471, 360, 283, 254, 190, 127230, 67895, 47028, 27909, 15427, 10853, 8490, 6296, 4947, 4608, 4145, 3532, 3158, 2931, 2826, 2795, 2490, 2639, 2466, 2161, 1806, 1479, 1178, 980, 799, 640, 540, 404, 314, 251, 232, 159, 100925, 54604, 38667, 23289, 11405, 8793, 6534, 5133, 4310, 3966, 3502, 3105, 2708, 2611, 2584, 2562, 2091, 2352, 2326, 1879, 1630, 1386, 1110, 898, 717, 603, 492, 348, 240, 197, 159, 140, 79735, 47023, 33435, 19294, 10316, 7425, 5831, 4520, 3622, 3448, 3291, 2990, 2628, 2376, 2412, 2214, 1915, 2041, 2332, 1980, 1602, 1256, 1103, 816, 690, 556, 428, 325, 181, 206, 159, 155, 69608, 40371, 28488, 17835, 8895, 6395, 5030, 3989, 3318, 2991, 2829, 2705, 2453, 2415, 2219, 2044, 1683, 1839, 2103, 1937, 1373, 1168, 1058, 827, 614, 510, 383, 315, 189, 165, 168, 118, 62025, 33331, 25218, 16995, 8982, 5719, 4443, 3721, 3060, 2899, 2499, 2563, 2324, 2341, 2136, 1907, 1743, 1596, 2010, 1932, 1331, 1091, 1007, 826, 583, 553, 376, 281, 174, 165, 144, 107, 54606, 29901, 21780, 15687, 8393, 4923, 3682, 3150, 2754, 2667, 2400, 2221, 2233, 2180, 1908, 1777, 1660, 1498, 1741, 1677, 1297, 1117, 862, 728, 591, 464, 370, 276, 188, 203, 120, 112, 53229, 27160, 20343, 14582, 8052, 4370, 3304, 2744, 2464, 2211, 2244, 2122, 2164, 1980, 1728, 1682, 1573, 1439, 1629, 1683, 1365, 1104, 869, 727, 547, 461, 307, 247, 152, 137, 110, 90, 49556, 25548, 18116, 13089, 8030, 4365, 3109, 2647, 2268, 2035, 1932, 1931, 1975, 1696, 1795, 1574, 1440, 1391, 1431, 1469, 1320, 1132, 876, 706, 531, 382, 322, 226, 129, 134, 93, 91, 48192, 23782, 15988, 12670, 8165, 4567, 2940, 2578, 2375, 2025, 2132, 2051, 1970, 1806, 1607, 1533, 1411, 1353, 1293, 1417, 1241, 962, 1002, 672, 486, 379, 299, 205, 124, 105, 74, 77, 43624, 22327, 14069, 11344, 7751, 4364, 3103, 2432, 2422, 2020, 1885, 1923, 2025, 1629, 1511, 1380, 1380, 1322, 1162, 1385, 1258, 930, 716, 681, 416, 310, 242, 183, 105, 106, 61, 72, 39040, 20163, 12943, 10067, 7601, 4758, 2690, 2165, 2095, 1862, 1710, 1629, 1731, 1536, 1547, 1358, 1336, 1247, 1428, 1228, 1097, 874, 608, 546, 407, 290, 228, 185, 113, 73, 53, 63, 34283, 18677, 11283, 8577, 6837, 4496, 2719, 2180, 2017, 1737, 1632, 1546, 1559, 1443, 1318, 1328, 1300, 1270, 1203, 1277, 1145, 749, 655, 526, 358, 280, 218, 152, 89, 66, 61, 45, 32035, 17118, 10998, 7933, 6407, 4652, 2622, 1958, 1750, 1618, 1546, 1470, 1493, 1449, 1323, 1285, 1241, 1171, 1109, 1066, 1015, 800, 616, 489, 341, 315, 204, 158, 46, 66, 55, 40, 29682, 15570, 9679, 6844, 6048, 4697, 2676, 1969, 1751, 1434, 1454, 1412, 1390, 1299, 1326, 1241, 1253, 1047, 1111, 977, 949, 747, 625, 464, 356, 259, 186, 118, 45, 49, 41, 50, 26548, 14428, 9047, 6369, 5317, 4607, 2796, 1833, 1575, 1417, 1468, 1400, 1346, 1304, 1323, 1257, 1099, 1115, 971, 910, 891, 768, 562, 422, 280, 218, 163, 106, 56, 56, 33, 47, 24703, 12632, 8270, 5186, 4152, 4115, 2868, 1821, 1596, 1416, 1392, 1350, 1398, 1245, 1261, 1245, 1143, 1096, 1023, 861, 833, 687, 512, 432, 299, 234, 163, 125, 45, 54, 39, 35, 22624, 11423, 7772, 4681, 3542, 3707, 2821, 1804, 1528, 1385, 1486, 1373, 1331, 1279, 1234, 1188, 1128, 1025, 967, 814, 692, 617, 462, 433, 221, 206, 157, 116, 42, 45, 34, 27, 21753, 10866, 6964, 4227, 3181, 3169, 2889, 1823, 1469, 1495, 1454, 1310, 1254, 1208, 1240, 1120, 1052, 1047, 922, 793, 687, 600, 458, 401, 230, 192, 149, 110, 44, 34, 32, 21, 20758, 10370, 6654, 3925, 2900, 2856, 2697, 1833, 1516, 1468, 1380, 1329, 1221, 1283, 1182, 1100, 1070, 995, 862, 789, 670, 537, 425, 314, 197, 170, 127, 99, 49, 38, 29, 33, 18829, 9496, 6083, 3743, 2630, 2565, 2503, 1935, 1486, 1474, 1383, 1316, 1238, 1172, 1167, 1098, 1053, 958, 871, 811, 616, 529, 406, 287, 195, 142, 104, 72, 30, 40, 25, 25, 17409, 8720, 5957, 3450, 2454, 2186, 2367, 2029, 1569, 1437, 1328, 1231, 1284, 1198, 1165, 1102, 997, 872, 805, 680, 604, 465, 340, 280, 174, 156, 83, 67, 24, 20, 13, 20, 16421, 8124, 5430, 3333, 2236, 2023, 2208, 1995, 1581, 1481, 1313, 1278, 1263, 1172, 1132, 1055, 973, 874, 770, 707, 556, 415, 354, 290, 177, 144, 89, 67, 20, 24, 24, 15, 15637, 6912, 4802, 2988, 2266, 1901, 1971, 1889, 1569, 1437, 1400, 1211, 1199, 1148, 1011, 932, 869, 784, 674, 564, 420, 348, 298, 199, 168, 121, 70, 46, 16, 22, 17, 18, 13947, 6109, 4339, 2934, 2105, 1784, 1859, 1830, 1745, 1464, 1288, 1321, 1217, 1143, 1037, 996, 899, 816, 763, 628, 538, 410, 319, 252, 123, 113, 85, 37, 23, 25, 20, 12, 13038, 5701, 4023, 2856, 1968, 1746, 1712, 1715, 1651, 1441, 1355, 1223, 1182, 1140, 1063, 973, 880, 709, 684, 586, 465, 347, 296, 221, 142, 98, 71, 46, 15, 15, 9, 10, 12738, 5308, 3869, 2514, 2101, 1627, 1559, 1881, 2004, 1600, 1261, 1265, 1249, 1129, 1045, 983, 882, 755, 643, 585, 512, 406, 292, 244, 136, 136, 69, 53, 16, 19, 16, 26, },
	{725111, 437795, 331324, 266157, 194488, 167186, 139631, 127128, 107034, 100368, 95818, 90445, 83805, 81087, 75737, 68026, 61604, 59028, 53773, 52628, 45686, 43797, 38534, 34659, 29814, 25816, 20735, 20603, 14497, 17188, 11377, 32052, 392272, 215757, 167901, 120075, 93759, 72791, 61297, 50392, 43923, 40076, 39393, 36495, 35517, 34169, 34420, 29642, 26734, 23347, 24002, 24656, 22929, 18019, 14023, 10954, 9349, 7657, 5934, 4568, 3916, 3307, 2721, 2552, 224792, 146191, 117803, 83487, 57267, 45804, 37936, 27329, 22582, 19813, 21415, 17886, 17937, 18639, 15994, 13576, 13186, 12750, 11474, 12229, 10141, 8632, 7295, 5756, 4937, 4128, 3557, 2903, 1896, 1613, 1253, 1279, 159675, 109935, 89336, 64160, 44984, 35142, 25580, 18929, 15949, 15036, 13136, 11017, 12719, 9647, 10474, 8787, 8600, 9031, 8729, 7687, 7724, 5519, 6258, 5246, 3553, 2651, 2888, 2661, 1975, 1362, 535, 536, 107492, 74169, 57723, 43874, 28717, 23209, 18109, 13482, 12502, 9816, 8775, 8328, 7123, 6924, 6913, 7111, 5961, 6366, 4950, 4951, 3963, 3989, 3066, 3323, 2577, 2744, 2296, 2081, 1481, 556, 302, 369, 87939, 58936, 49607, 34820, 22949, 18218, 13986, 11860, 9587, 8251, 7085, 6227, 5429, 5086, 4680, 4064, 3606, 3768, 4523, 3525, 3038, 2694, 2204, 2145, 1699, 1335, 1149, 894, 543, 340, 273, 257, 68597, 45377, 34546, 25696, 16339, 12845, 11152, 8629, 7615, 6852, 5651, 4967, 4157, 3659, 3655, 3306, 3249, 3018, 3099, 3032, 2792, 2464, 1807, 1665, 1391, 1217, 797, 582, 390, 311, 270, 189, 51269, 35198, 26736, 18815, 13877, 10012, 8274, 6690, 6270, 5378, 4781, 4437, 3644, 3533, 3218, 2711, 2571, 2601, 2560, 2306, 1950, 1723, 1508, 1227, 1080, 903, 641, 482, 254, 228, 156, 134, 40640, 25066, 19560, 14185, 9971, 7790, 6488, 5844, 5037, 4555, 3887, 3419, 3037, 2735, 2599, 2427, 2232, 2226, 2089, 1939, 1670, 1566, 1506, 1178, 1037, 845, 598, 421, 252, 196, 152, 139, 34462, 22693, 17684, 12356, 8923, 7037, 5870, 5098, 4330, 4006, 3633, 3300, 2860, 2435, 2380, 2347, 1970, 1910, 1800, 1714, 1589, 1303, 1244, 1097, 835, 731, 578, 514, 311, 252, 188, 172, 30642, 20508, 15867, 10939, 8484, 6750, 5106, 4418, 3697, 3480, 3338, 3242, 2909, 2508, 2238, 2118, 1925, 1774, 1689, 1590, 1393, 1351, 1206, 1032, 829, 756, 636, 432, 325, 288, 197, 194, 26196, 17652, 13615, 9656, 6999, 5421, 4407, 3728, 3229, 2996, 2663, 2815, 2862, 2315, 2030, 1886, 1723, 1767, 1537, 1442, 1285, 1112, 987, 832, 771, 632, 441, 373, 280, 222, 187, 152, 23230, 14318, 10496, 7123, 5091, 4195, 3383, 2956, 2714, 2355, 2244, 2377, 2293, 2057, 1938, 1730, 1582, 1517, 1445, 1253, 1227, 1115, 942, 753, 619, 497, 430, 289, 177, 118, 102, 123, 21678, 13164, 9718, 6459, 4682, 3727, 3096, 3035, 2478, 2261, 2166, 2066, 1929, 1858, 1782, 1639, 1452, 1385, 1299, 1180, 1169, 984, 848, 700, 545, 511, 389, 255, 135, 95, 62, 78, 19722, 12510, 9252, 5916, 4588, 3389, 2901, 2603, 2481, 2087, 1949, 1858, 1812, 1623, 1704, 1619, 1490, 1290, 1229, 1155, 979, 901, 799, 679, 511, 453, 319, 225, 112, 76, 61, 68, 18087, 11349, 8413, 5792, 4313, 3268, 2664, 2521, 2283, 2064, 2016, 1843, 1856, 1812, 1709, 1630, 1362, 1315, 1217, 1171, 1019, 893, 756, 670, 578, 479, 319, 183, 99, 75, 92, 70, 15649, 10492, 7368, 5093, 4020, 2962, 2493, 2260, 2116, 1824, 1857, 1713, 1597, 1580, 1421, 1442, 1314, 1094, 1177, 1045, 873, 815, 727, 572, 484, 368, 238, 159, 91, 60, 42, 36, 13944, 9649, 6733, 4402, 3478, 2873, 2382, 2199, 2164, 1842, 1697, 1738, 1547, 1519, 1334, 1314, 1208, 1098, 1213, 1009, 961, 842, 674, 517, 523, 325, 234, 125, 71, 60, 43, 37, 12577, 8638, 6311, 4303, 3123, 2615, 2165, 2006, 1811, 1783, 1644, 1625, 1623, 1461, 1299, 1349, 1106, 1125, 975, 1006, 812, 711, 646, 542, 402, 304, 225, 138, 61, 64, 47, 26, 11798, 7799, 5942, 3998, 3179, 2438, 2031, 1960, 1726, 1651, 1578, 1479, 1519, 1387, 1292, 1279, 1075, 1102, 1023, 954, 836, 717, 612, 474, 402, 321, 249, 119, 66, 49, 33, 45, 10468, 7044, 5311, 3550, 2528, 2308, 1917, 1706, 1665, 1530, 1625, 1505, 1435, 1326, 1287, 1181, 1160, 1012, 1018, 932, 789, 678, 585, 468, 370, 280, 208, 134, 76, 58, 58, 46, 9050, 6379, 4588, 3359, 2406, 2023, 1697, 1583, 1619, 1496, 1494, 1479, 1357, 1286, 1251, 1185, 1016, 1044, 968, 912, 805, 654, 534, 442, 381, 229, 175, 103, 49, 39, 30, 34, 8200, 5852, 4298, 3007, 2243, 1873, 1620, 1561, 1491, 1430, 1432, 1497, 1390, 1322, 1210, 1164, 984, 1047, 904, 797, 787, 724, 540, 473, 319, 247, 164, 122, 37, 41, 25, 31, 7421, 5216, 4173, 2981, 2098, 1770, 1617, 1428, 1453, 1483, 1424, 1273, 1334, 1250, 1173, 1111, 953, 1023, 933, 789, 750, 607, 516, 418, 295, 237, 150, 102, 34, 30, 26, 19, 6712, 4911, 3584, 2618, 1997, 1832, 1603, 1473, 1478, 1381, 1336, 1341, 1268, 1298, 1204, 1167, 975, 960, 833, 786, 633, 598, 481, 380, 285, 214, 127, 80, 35, 28, 16, 32, 5967, 4230, 3052, 2387, 2011, 1639, 1521, 1353, 1388, 1317, 1394, 1328, 1289, 1273, 1184, 1076, 913, 880, 868, 759, 624, 594, 442, 382, 265, 245, 142, 74, 34, 33, 28, 11, 5527, 4047, 3267, 2463, 2135, 1742, 1560, 1470, 1403, 1374, 1377, 1300, 1257, 1245, 1236, 1072, 946, 898, 838, 769, 634, 482, 428, 340, 234, 199, 119, 76, 26, 42, 22, 21, 5116, 3859, 3050, 2489, 1942, 1644, 1481, 1402, 1446, 1354, 1250, 1278, 1270, 1124, 1103, 1063, 927, 870, 763, 688, 568, 561, 422, 358, 275, 185, 120, 73, 31, 24, 16, 14, 5012, 3593, 3019, 2320, 1943, 1691, 1529, 1380, 1377, 1249, 1251, 1265, 1253, 1112, 1081, 964, 900, 774, 677, 615, 496, 412, 365, 301, 211, 178, 100, 47, 14, 6, 10, 14, 4397, 3305, 2776, 2395, 1973, 1750, 1582, 1489, 1443, 1404, 1357, 1298, 1173, 1213, 1059, 1029, 903, 833, 770, 631, 550, 484, 365, 314, 255, 173, 97, 54, 14, 23, 16, 17, 3977, 3450, 2877, 2438, 1974, 1837, 1656, 1513, 1457, 1377, 1431, 1328, 1267, 1120, 1073, 970, 860, 897, 711, 707, 578, 471, 383, 305, 268, 160, 112, 48, 25, 19, 10, 17, 5747, 3474, 2805, 2458, 1942, 1739, 1438, 1394, 1491, 1336, 1236, 1220, 1211, 1174, 1119, 1009, 881, 818, 720, 692, 600, 471, 383, 301, 231, 163, 82, 52, 23, 17, 11, 20, },
	{1488462, 1167494, 893493, 711317, 532520, 455186, 376855, 314152, 261965, 226088, 203000, 183854, 166107, 152700, 136965, 131003, 131929, 107868, 100308, 92559, 79929, 74937, 61176, 61162, 45203, 47130, 29233, 42632, 18652, 64243, 12493, 226127, 1204797, 931221, 758131, 604582, 468658, 389365, 338592, 276450, 231087, 199647, 181174, 161001, 145467, 133558, 118856, 110057, 102189, 95020, 88413, 81391, 72261, 63364, 54329, 46514, 41720, 33269, 26463, 20881, 15124, 12176, 8676, 10591, 973847, 796063, 664408, 544458, 420789, 356122, 302339, 255491, 210750, 187380, 168413, 154217, 138901, 130706, 117284, 107271, 98497, 91284, 88589, 78673, 70247, 61683, 52655, 45367, 39636, 30807, 24306, 20264, 13546, 9731, 8289, 6760, 797447, 685120, 584646, 484050, 379173, 322900, 274277, 230545, 193334, 172613, 160036, 143517, 129622, 120725, 111058, 101906, 91812, 89088, 82031, 74206, 67915, 59639, 50189, 44663, 34741, 30365, 25308, 17529, 12238, 9081, 6366, 5907, 633187, 559329, 484208, 407575, 308599, 265331, 225744, 194972, 173829, 158082, 143670, 132718, 119079, 110719, 99305, 94020, 87544, 79695, 75973, 69079, 60906, 54309, 48005, 40865, 34394, 29349, 23259, 17875, 12480, 8126, 5919, 4916, 540550, 484643, 424112, 351276, 263530, 230525, 198387, 177019, 160345, 148293, 134603, 120967, 109189, 100663, 92915, 86856, 79811, 74921, 70660, 63870, 57966, 49572, 44759, 37860, 30881, 25748, 22331, 17078, 11105, 7873, 6438, 4920, 470919, 416887, 367351, 307635, 234417, 207702, 184975, 165233, 144928, 134334, 121754, 110095, 100540, 90465, 86619, 81422, 76320, 70136, 63799, 59386, 51888, 46515, 39783, 34148, 29719, 25480, 19748, 14262, 9486, 7235, 5619, 4386, 387915, 343536, 311366, 264920, 207530, 179456, 164175, 146192, 132468, 123087, 113120, 103773, 91272, 85100, 78879, 75478, 68722, 64965, 60639, 53902, 47750, 43242, 37448, 30737, 26166, 22386, 17249, 12611, 7818, 5901, 4131, 3621, 326293, 287286, 265475, 224071, 190644, 169016, 149646, 133839, 120050, 114229, 104120, 94583, 86169, 80277, 74879, 70562, 62882, 59577, 55779, 50121, 44825, 40105, 35519, 29563, 24650, 19894, 16229, 11758, 6953, 5299, 3787, 3558, 289213, 256552, 233630, 206716, 177411, 155558, 136715, 122314, 110365, 102536, 94355, 88329, 78986, 74608, 69799, 64366, 58034, 53709, 48967, 45559, 41161, 36699, 33210, 28045, 23160, 18026, 14580, 10080, 6112, 4558, 3406, 2783, 258956, 228707, 209383, 184997, 160651, 141094, 124657, 112958, 101765, 93387, 85558, 80377, 73903, 69560, 63706, 59829, 52821, 49472, 45986, 41506, 37942, 33293, 29586, 25022, 20997, 17699, 13368, 8803, 5538, 4029, 3051, 2540, 227326, 205556, 184328, 163079, 142757, 124079, 112237, 99965, 90521, 84079, 78138, 73190, 68358, 63588, 58698, 55418, 49042, 47129, 42756, 38925, 34900, 30911, 26930, 23018, 19676, 15803, 11493, 8286, 5224, 3640, 2777, 2541, 206959, 184640, 167545, 143974, 127282, 113311, 101320, 90889, 83300, 76374, 71431, 66794, 62561, 56745, 54274, 51394, 46581, 43710, 41079, 37332, 31578, 29780, 25543, 21444, 17329, 14291, 11020, 7482, 4609, 3141, 2782, 2279, 188640, 167648, 149966, 129834, 115661, 102896, 94096, 82472, 75318, 68460, 65757, 61465, 57390, 52415, 49687, 46662, 42228, 39930, 37685, 33788, 30540, 27653, 23947, 20356, 16326, 13113, 9668, 6892, 3675, 3182, 2152, 1899, 173725, 154288, 136815, 117987, 106322, 95393, 84964, 76350, 70268, 63748, 60165, 56387, 52744, 49439, 45997, 44050, 40265, 38012, 34744, 31218, 28716, 25561, 23309, 18661, 15250, 12286, 8988, 6213, 3248, 2568, 1958, 1659, 157725, 138234, 123705, 109705, 94792, 87295, 78486, 71605, 64922, 58250, 54914, 50692, 49843, 44389, 42878, 40206, 36859, 35339, 32712, 30606, 26926, 23516, 20734, 18352, 14229, 11410, 8064, 5571, 2939, 2288, 1650, 1503, 144397, 126636, 112119, 97397, 88339, 79243, 72119, 64147, 58129, 54957, 49480, 46070, 43720, 42090, 39086, 37512, 34760, 32247, 30501, 28168, 25226, 21429, 18938, 16311, 13027, 10201, 7271, 4775, 2744, 1978, 1423, 1342, 128624, 115610, 101314, 87644, 78582, 72608, 65047, 58840, 54334, 49672, 46341, 43667, 41170, 38839, 36511, 35189, 32330, 30781, 28768, 26465, 23856, 20119, 18115, 15190, 12351, 9289, 7070, 4678, 2746, 1866, 1365, 1201, 117841, 105966, 92965, 80557, 71145, 65357, 60139, 55199, 48948, 46768, 42480, 39836, 39330, 36288, 34048, 32842, 30297, 28874, 26693, 24379, 22182, 19611, 17087, 14419, 11568, 9176, 6410, 4666, 2370, 1690, 1222, 1056, 105934, 94933, 84478, 72782, 64164, 58403, 54221, 50515, 44944, 42264, 39175, 37224, 35158, 33598, 31730, 30396, 28871, 27379, 25149, 23078, 20627, 18174, 15666, 13683, 11155, 8645, 6070, 3904, 2350, 1529, 1107, 1075, 94919, 87525, 76732, 68170, 57099, 53274, 48173, 45277, 41086, 38103, 37090, 34598, 33044, 31853, 30931, 29278, 26696, 26127, 23810, 21217, 19510, 17297, 15168, 12659, 10062, 8035, 5562, 3678, 2045, 1424, 994, 952, 83215, 77221, 68140, 59155, 50281, 47794, 43940, 41255, 39182, 35815, 33852, 32182, 30762, 29493, 28271, 27722, 25636, 23980, 23018, 20682, 18481, 16312, 14344, 12042, 9772, 8000, 5411, 3573, 1697, 1357, 933, 1253, 75940, 69961, 62704, 54458, 46661, 43950, 39898, 37416, 36760, 33616, 32258, 30741, 29945, 28276, 27361, 26370, 24243, 23031, 22069, 19477, 17479, 15761, 13496, 11715, 9137, 7315, 4958, 3231, 1600, 1141, 819, 823, 68906, 64915, 57290, 49023, 43110, 39573, 37725, 34710, 32882, 31611, 30206, 29096, 27346, 27322, 26600, 25601, 23268, 22751, 20428, 18575, 16168, 14408, 13360, 11317, 8456, 6416, 4877, 2971, 1496, 1055, 776, 765, 64653, 58910, 54450, 46027, 39848, 37260, 35250, 32510, 31821, 29218, 28831, 27269, 26531, 26517, 25336, 23959, 22385, 21636, 19347, 17574, 15291, 13727, 11881, 9966, 7776, 6071, 4199, 2805, 1287, 940, 765, 712, 60822, 56316, 49778, 42474, 36658, 33080, 33308, 30881, 29159, 27802, 27552, 25761, 25279, 24923, 23973, 23629, 21130, 20430, 18733, 16890, 14704, 13353, 11665, 9256, 7385, 5669, 3959, 2293, 1150, 886, 681, 674, 57290, 52103, 47971, 40743, 35794, 32497, 30448, 29418, 28454, 26763, 26401, 24795, 24573, 23685, 23126, 21858, 20113, 19493, 18067, 15586, 14301, 12344, 10736, 9138, 7045, 5473, 3681, 2219, 1117, 870, 607, 612, 53030, 49740, 45410, 38997, 33037, 30388, 29306, 27431, 26887, 26047, 25002, 24278, 23383, 22591, 22473, 21358, 19128, 18820, 17088, 14924, 13424, 11907, 10180, 8573, 6849, 5045, 3625, 2414, 925, 700, 549, 604, 49056, 44637, 41321, 36128, 31520, 29517, 27445, 26478, 25058, 24476, 23763, 22399, 21927, 21187, 20804, 19688, 18131, 17092, 15373, 13748, 12237, 10531, 8950, 7703, 6242, 4716, 3299, 1766, 945, 641, 493, 465, 46372, 42494, 38290, 35860, 30258, 28619, 26922, 25826, 25609, 24611, 23932, 23020, 21729, 21731, 21018, 19935, 17864, 17568, 15971, 14167, 12615, 10727, 9374, 8057, 6306, 4636, 3072, 1797, 800, 631, 523, 494, 41703, 40166, 36759, 33853, 29532, 27746, 26161, 24420, 24639, 23301, 23759, 22679, 21923, 21054, 20371, 19698, 17682, 17018, 15454, 13680, 12183, 10606, 9157, 7862, 6137, 4270, 2941, 1665, 742, 612, 497, 465, 64970, 38041, 35827, 32545, 28912, 26648, 25382, 23517, 24274, 23131, 22291, 22118, 21724, 20711, 20062, 19580, 17345, 17028, 15254, 13631, 11957, 10512, 8940, 7524, 5967, 4544, 2870, 1769, 811, 639, 507, 573, },
	{73936, 61522, 50725, 38470, 23957, 18522, 12609, 8672, 6781, 4974, 3004, 2805, 2675, 2675, 2405, 2235, 2002, 1451, 1118, 1029, 1328, 994, 885, 996, 425, 601, 275, 801, 186, 1723, 460, 5161, 53312, 43351, 37370, 30913, 23749, 17729, 14262, 9640, 6479, 5208, 4218, 5004, 3158, 4313, 2493, 1559, 1786, 1352, 1486, 2034, 995, 872, 497, 965, 625, 658, 205, 210, 144, 158, 125, 201, 33326, 28047, 26097, 21437, 18778, 15380, 13708, 10356, 8790, 7191, 6241, 4973, 4246, 3349, 3675, 2479, 1985, 1741, 1395, 1663, 2009, 1424, 1024, 1122, 508, 330, 528, 224, 268, 163, 52, 56, 22889, 20117, 17232, 14576, 11231, 12396, 9825, 9099, 8020, 8004, 6196, 7090, 5002, 4907, 3508, 2108, 2369, 2584, 1820, 2602, 1516, 1439, 825, 1525, 511, 428, 428, 178, 108, 196, 52, 48, 14433, 12109, 10586, 9154, 6748, 6095, 5258, 5168, 4780, 5008, 6434, 4483, 6422, 4770, 5413, 3475, 2434, 2539, 1809, 1369, 2392, 730, 1537, 1123, 539, 429, 490, 138, 213, 120, 22, 41, 10845, 9118, 8037, 6141, 4568, 5481, 3668, 3815, 3534, 3475, 5123, 4983, 3535, 5175, 4728, 4178, 3541, 3008, 3742, 3252, 2576, 1447, 1057, 1450, 1008, 604, 212, 211, 75, 144, 61, 204, 8283, 6646, 5969, 5054, 4190, 3067, 2546, 2377, 2730, 2907, 2687, 2380, 2435, 2913, 4061, 2514, 3150, 3341, 3581, 3528, 3682, 1871, 2299, 1734, 823, 640, 912, 216, 298, 236, 35, 45, 5982, 5009, 4837, 3435, 2776, 2381, 2067, 1958, 1677, 1619, 1612, 2227, 1789, 3118, 1933, 1564, 2209, 2522, 2111, 3826, 3419, 4144, 3532, 2775, 2607, 1889, 994, 696, 117, 867, 48, 51, 4605, 3694, 3251, 2736, 2405, 2360, 1880, 1758, 1300, 1272, 1150, 1171, 1155, 1078, 2083, 1688, 1293, 1718, 1852, 1949, 3395, 1673, 3307, 2608, 3626, 2501, 1628, 802, 1625, 945, 83, 236, 3663, 2965, 2957, 2325, 1974, 1766, 1413, 1243, 1135, 946, 940, 1076, 982, 1358, 839, 918, 1025, 1043, 1234, 1790, 926, 1159, 1113, 1830, 1174, 2460, 2116, 3325, 1747, 2467, 1126, 879, 3035, 2490, 2295, 1951, 1886, 1374, 1412, 1103, 1154, 866, 917, 710, 752, 865, 1188, 663, 635, 686, 675, 712, 1383, 592, 1580, 1417, 642, 843, 1504, 727, 2527, 1639, 2756, 1102, 2525, 2118, 1975, 1549, 1377, 1140, 1011, 902, 806, 691, 682, 750, 683, 745, 575, 584, 508, 437, 471, 612, 593, 551, 516, 874, 779, 760, 402, 326, 369, 655, 631, 2277, 2393, 1878, 1572, 1324, 1097, 994, 909, 818, 729, 636, 638, 563, 519, 474, 632, 446, 445, 382, 427, 417, 557, 375, 758, 408, 336, 310, 461, 155, 394, 621, 205, 210, 2183, 1635, 1430, 1262, 1209, 848, 809, 754, 591, 560, 567, 480, 428, 422, 412, 431, 383, 348, 395, 356, 311, 312, 263, 456, 328, 354, 221, 134, 60, 384, 77, 160, 1963, 1489, 1263, 1120, 934, 804, 677, 637, 540, 457, 452, 437, 463, 393, 414, 348, 337, 333, 320, 259, 298, 225, 384, 206, 204, 139, 216, 166, 94, 126, 33, 36, 1789, 1456, 1143, 996, 893, 767, 628, 670, 529, 443, 533, 533, 531, 776, 582, 374, 304, 306, 303, 272, 225, 178, 249, 171, 126, 110, 126, 78, 58, 84, 78, 28, 1701, 1267, 1072, 932, 786, 813, 619, 925, 558, 501, 368, 411, 427, 369, 291, 262, 268, 295, 233, 241, 218, 196, 166, 136, 153, 110, 83, 53, 29, 34, 22, 37, 1503, 1208, 1345, 867, 1883, 791, 1564, 864, 498, 468, 394, 373, 347, 325, 299, 286, 261, 249, 210, 225, 252, 219, 154, 137, 146, 87, 73, 51, 25, 29, 18, 18, 1629, 1792, 1421, 3164, 896, 854, 593, 526, 447, 452, 397, 404, 319, 308, 292, 269, 228, 235, 236, 204, 171, 269, 142, 126, 127, 106, 71, 57, 21, 23, 11, 18, 2136, 1787, 1744, 1174, 1454, 911, 732, 545, 479, 395, 424, 305, 275, 409, 279, 248, 245, 247, 247, 216, 166, 174, 138, 134, 94, 82, 59, 41, 15, 14, 14, 10, 1325, 1019, 1005, 1110, 812, 689, 790, 1011, 463, 398, 331, 308, 307, 281, 247, 259, 248, 227, 206, 174, 175, 173, 135, 122, 93, 83, 51, 39, 21, 12, 5, 12, 1184, 966, 853, 699, 638, 543, 500, 465, 590, 341, 396, 500, 421, 315, 259, 258, 225, 200, 219, 162, 190, 174, 163, 118, 82, 83, 50, 31, 23, 11, 12, 12, 915, 806, 645, 624, 545, 489, 448, 386, 358, 310, 292, 283, 567, 516, 397, 252, 291, 189, 215, 169, 171, 144, 120, 101, 100, 75, 58, 38, 27, 17, 14, 10, 869, 659, 559, 515, 478, 409, 345, 390, 309, 304, 290, 309, 261, 308, 356, 254, 232, 195, 169, 225, 161, 136, 115, 104, 68, 61, 46, 33, 12, 19, 10, 8, 832, 683, 596, 583, 470, 372, 318, 287, 261, 254, 282, 232, 228, 270, 288, 253, 196, 250, 191, 174, 140, 113, 126, 94, 73, 84, 46, 31, 19, 9, 12, 12, 969, 711, 613, 488, 350, 328, 314, 271, 281, 256, 250, 256, 246, 253, 227, 227, 181, 188, 166, 182, 118, 141, 109, 97, 76, 86, 49, 22, 14, 11, 10, 10, 748, 640, 551, 454, 451, 341, 380, 264, 285, 233, 235, 238, 262, 245, 209, 215, 165, 213, 208, 161, 137, 126, 111, 99, 76, 76, 40, 29, 16, 11, 12, 10, 707, 618, 527, 479, 320, 315, 300, 270, 282, 294, 274, 262, 228, 220, 205, 210, 169, 194, 158, 137, 140, 134, 107, 110, 83, 59, 38, 18, 15, 3, 7, 10, 604, 500, 421, 380, 347, 325, 303, 273, 267, 294, 232, 239, 172, 201, 181, 199, 161, 158, 165, 129, 129, 119, 110, 88, 63, 63, 41, 17, 15, 6, 5, 11, 631, 519, 400, 370, 327, 333, 311, 266, 254, 235, 228, 213, 221, 199, 203, 199, 174, 167, 139, 144, 136, 119, 115, 82, 73, 65, 44, 26, 12, 16, 14, 10, 507, 490, 384, 367, 335, 316, 297, 273, 240, 236, 230, 211, 203, 207, 211, 195, 173, 194, 149, 153, 152, 102, 96, 101, 67, 51, 35, 27, 8, 6, 6, 8, 1250, 512, 390, 379, 312, 288, 282, 238, 259, 261, 209, 186, 218, 195, 208, 228, 148, 153, 156, 147, 121, 125, 107, 72, 75, 60, 31, 17, 11, 11, 5, 8, },
	{44984, 35493, 26570, 16000, 9953, 6226, 3863, 2484, 1871, 1251, 953, 914, 673, 711, 609, 521, 552, 373, 387, 396, 315, 294, 290, 283, 158, 240, 109, 220, 107, 567, 132, 2329, 31808, 25934, 21752, 15972, 9903, 6283, 4196, 2538, 2029, 1242, 852, 1288, 1227, 558, 887, 431, 447, 316, 356, 563, 381, 285, 489, 403, 446, 114, 103, 98, 60, 42, 50, 77, 19701, 18216, 17774, 15164, 11140, 8951, 3961, 3417, 1890, 1439, 1239, 741, 821, 685, 757, 634, 354, 324, 339, 607, 249, 225, 262, 163, 308, 109, 121, 106, 51, 53, 38, 19, 13018, 11752, 11748, 11133, 10546, 9364, 7554, 4509, 2731, 2213, 1266, 1349, 1181, 575, 616, 380, 809, 380, 344, 432, 701, 237, 340, 324, 179, 135, 98, 157, 55, 39, 56, 128, 6896, 5988, 5763, 5666, 6203, 7639, 6172, 5195, 2993, 2417, 2090, 1069, 1132, 932, 471, 518, 515, 407, 681, 448, 514, 365, 326, 248, 96, 107, 222, 116, 68, 105, 175, 56, 5076, 4247, 4264, 3582, 3467, 3685, 5410, 5429, 4838, 3542, 1875, 1988, 1428, 1207, 835, 984, 517, 469, 704, 625, 285, 385, 541, 304, 312, 182, 159, 82, 23, 28, 26, 71, 4202, 3386, 3131, 2418, 2519, 2138, 2157, 3574, 4481, 4066, 3390, 3128, 1676, 1288, 1412, 1084, 1156, 478, 476, 534, 405, 390, 176, 383, 204, 100, 192, 39, 53, 38, 19, 20, 2795, 2710, 2502, 2623, 1984, 1128, 1463, 2095, 2041, 3521, 5522, 3768, 2723, 1606, 1510, 1967, 916, 809, 807, 422, 460, 496, 207, 398, 107, 170, 192, 170, 41, 17, 15, 12, 1931, 1851, 1761, 1956, 1411, 1309, 1380, 1298, 1094, 2255, 4545, 4700, 4782, 4104, 2574, 1146, 1108, 967, 1214, 1402, 521, 862, 404, 354, 326, 163, 59, 53, 56, 32, 17, 10, 1583, 1508, 1675, 970, 1942, 1000, 858, 1174, 1171, 856, 1423, 4186, 5192, 5130, 3682, 1696, 1811, 1222, 1139, 960, 1293, 533, 522, 637, 252, 202, 340, 58, 19, 27, 17, 5, 1369, 1045, 1147, 756, 763, 776, 578, 695, 762, 707, 1275, 1741, 2750, 4368, 5866, 4310, 2418, 1150, 627, 815, 583, 1337, 483, 633, 370, 147, 111, 72, 68, 16, 13, 20, 1105, 886, 756, 658, 646, 588, 728, 524, 629, 468, 467, 1055, 1002, 1882, 3092, 4944, 4423, 1605, 1290, 1953, 924, 727, 843, 808, 311, 418, 89, 75, 74, 31, 24, 11, 926, 754, 641, 660, 507, 493, 463, 426, 346, 654, 373, 553, 1174, 794, 1448, 2667, 2992, 4600, 1695, 2275, 666, 1195, 617, 344, 732, 547, 668, 145, 120, 51, 25, 15, 918, 722, 567, 463, 402, 522, 390, 687, 773, 310, 387, 735, 461, 327, 691, 1221, 1908, 2226, 4497, 3724, 2761, 1874, 1370, 1043, 693, 613, 448, 146, 101, 114, 14, 20, 846, 573, 527, 438, 437, 429, 417, 447, 450, 333, 354, 680, 430, 523, 378, 677, 931, 1858, 2220, 4038, 4526, 3836, 1438, 870, 1100, 433, 465, 395, 315, 110, 72, 20, 883, 529, 457, 526, 781, 327, 568, 287, 574, 281, 216, 283, 267, 692, 468, 272, 1085, 854, 1280, 3409, 4209, 3690, 4210, 3585, 291, 790, 262, 147, 250, 87, 195, 28, 685, 486, 511, 420, 1199, 285, 388, 325, 206, 205, 177, 314, 148, 253, 582, 270, 341, 740, 573, 888, 2087, 3717, 4940, 3820, 3263, 1287, 538, 575, 35, 59, 68, 24, 694, 603, 500, 643, 413, 247, 208, 264, 380, 432, 154, 227, 316, 310, 404, 206, 185, 365, 512, 329, 1163, 1590, 2417, 4328, 4603, 2367, 987, 686, 100, 34, 30, 28, 993, 490, 421, 421, 305, 233, 177, 178, 221, 214, 215, 173, 208, 359, 233, 150, 106, 419, 304, 388, 621, 1506, 814, 2118, 3272, 2835, 2099, 1707, 391, 318, 118, 38, 557, 444, 335, 249, 227, 236, 171, 161, 231, 166, 255, 142, 187, 153, 210, 127, 130, 163, 267, 207, 636, 261, 1023, 1043, 712, 1845, 1899, 1901, 1134, 444, 56, 54, 523, 437, 510, 462, 225, 281, 196, 292, 161, 183, 243, 124, 251, 123, 164, 123, 142, 116, 141, 126, 354, 152, 466, 228, 843, 585, 505, 1399, 2803, 1575, 644, 106, 471, 381, 265, 352, 327, 218, 212, 153, 277, 159, 176, 154, 102, 101, 120, 148, 278, 102, 126, 275, 180, 210, 243, 412, 495, 570, 740, 584, 1482, 2822, 2105, 1240, 418, 324, 229, 182, 281, 282, 234, 157, 211, 250, 153, 139, 127, 96, 124, 107, 112, 166, 99, 217, 103, 86, 96, 129, 170, 441, 384, 303, 302, 2260, 1893, 1701, 363, 258, 225, 177, 186, 169, 307, 292, 141, 123, 161, 135, 122, 123, 128, 100, 84, 131, 95, 96, 80, 84, 75, 127, 54, 83, 90, 278, 283, 281, 774, 1934, 333, 263, 222, 183, 133, 123, 135, 147, 244, 118, 134, 113, 114, 104, 140, 111, 133, 77, 101, 73, 79, 75, 54, 67, 78, 99, 176, 72, 176, 117, 466, 560, 341, 261, 189, 177, 170, 160, 123, 118, 103, 124, 146, 152, 120, 128, 117, 86, 88, 92, 130, 87, 72, 52, 56, 50, 80, 40, 34, 16, 84, 34, 446, 23, 295, 236, 172, 142, 128, 110, 94, 102, 137, 127, 120, 142, 164, 104, 93, 96, 78, 69, 82, 61, 52, 48, 80, 64, 32, 53, 19, 25, 24, 28, 85, 17, 295, 210, 180, 139, 128, 132, 155, 95, 91, 108, 115, 139, 144, 179, 143, 85, 91, 68, 111, 66, 61, 43, 42, 55, 38, 24, 18, 8, 8, 9, 4, 9, 217, 174, 157, 114, 129, 105, 106, 92, 109, 82, 86, 93, 100, 103, 90, 72, 75, 72, 75, 65, 45, 50, 32, 30, 21, 21, 13, 7, 6, 4, 13, 4, 201, 171, 149, 134, 115, 128, 129, 104, 108, 93, 89, 106, 81, 153, 87, 93, 78, 81, 57, 46, 51, 46, 45, 32, 27, 20, 11, 13, 4, 9, 5, 5, 187, 187, 145, 109, 128, 95, 105, 106, 154, 83, 85, 75, 77, 86, 90, 74, 83, 68, 74, 61, 46, 45, 42, 42, 23, 16, 19, 14, 3, 3, 5, 2, 408, 157, 143, 112, 119, 134, 154, 87, 106, 82, 89, 86, 81, 73, 94, 67, 79, 80, 62, 70, 54, 54, 35, 33, 27, 20, 12, 16, 4, 4, 6, 2, },
	{32763, 22874, 15890, 9278, 5592, 4139, 2903, 1889, 1752, 1048, 762, 643, 613, 522, 503, 384, 457, 349, 345, 334, 244, 245, 162, 183, 119, 179, 83, 248, 54, 685, 48, 3252, 23518, 18548, 14100, 8972, 5173, 3647, 2497, 1671, 1213, 958, 814, 682, 628, 423, 476, 392, 305, 301, 293, 278, 279, 205, 211, 145, 120, 125, 82, 54, 36, 36, 39, 71, 16540, 13628, 12144, 9216, 5425, 3383, 2188, 2137, 1292, 895, 867, 605, 550, 454, 394, 360, 292, 385, 259, 300, 228, 208, 149, 132, 146, 80, 73, 56, 39, 27, 20, 15, 10944, 9899, 9801, 8539, 5818, 3632, 2160, 1573, 1177, 934, 815, 567, 535, 457, 424, 343, 471, 339, 288, 218, 217, 220, 150, 177, 106, 89, 74, 55, 29, 22, 11, 15, 7515, 6132, 5762, 5886, 5196, 3711, 2500, 1379, 1154, 857, 899, 556, 566, 419, 410, 401, 304, 272, 248, 238, 201, 176, 168, 150, 99, 83, 65, 48, 22, 26, 8, 10, 4887, 4283, 4093, 3479, 3715, 3820, 2801, 1716, 1077, 916, 749, 597, 440, 498, 371, 360, 376, 254, 256, 208, 229, 163, 136, 114, 97, 64, 68, 42, 40, 15, 6, 15, 3480, 3205, 3144, 2426, 2538, 3111, 3372, 2488, 1396, 1079, 998, 473, 512, 438, 396, 260, 314, 254, 220, 237, 153, 149, 157, 116, 133, 95, 79, 45, 25, 14, 6, 11, 2687, 2281, 2173, 2244, 1772, 1889, 2350, 2588, 2473, 1170, 689, 607, 718, 359, 311, 377, 284, 254, 203, 203, 177, 165, 199, 136, 120, 74, 45, 58, 16, 14, 7, 6, 1835, 1628, 1604, 1332, 1254, 1308, 2089, 2200, 2533, 2300, 1261, 649, 795, 395, 323, 247, 256, 218, 208, 234, 145, 150, 108, 105, 78, 71, 53, 38, 19, 12, 7, 4, 1569, 1445, 1308, 1280, 1084, 1338, 1103, 1564, 1621, 1916, 2067, 1391, 710, 421, 387, 288, 299, 239, 211, 236, 135, 140, 112, 88, 77, 53, 43, 37, 20, 20, 7, 7, 1166, 992, 1027, 994, 819, 850, 766, 1061, 1097, 1702, 2242, 1601, 1020, 744, 360, 288, 265, 309, 183, 151, 118, 117, 110, 77, 75, 50, 38, 21, 20, 16, 6, 8, 985, 957, 775, 604, 608, 606, 712, 606, 839, 1000, 1698, 2091, 1525, 1101, 590, 373, 316, 201, 186, 183, 158, 129, 432, 102, 85, 69, 45, 25, 11, 7, 7, 4, 836, 793, 790, 571, 588, 445, 444, 454, 583, 500, 878, 1239, 1782, 1178, 926, 405, 420, 217, 272, 205, 152, 212, 112, 90, 83, 64, 49, 31, 20, 6, 1, 10, 761, 684, 602, 520, 464, 402, 509, 620, 361, 436, 598, 663, 890, 1629, 1222, 770, 516, 353, 352, 172, 118, 146, 102, 92, 64, 59, 39, 24, 13, 9, 3, 3, 799, 638, 484, 368, 523, 356, 345, 549, 305, 362, 396, 366, 966, 1086, 1249, 1443, 1058, 601, 249, 338, 187, 137, 105, 107, 84, 56, 38, 23, 9, 9, 7, 7, 700, 555, 398, 376, 391, 300, 387, 288, 319, 333, 368, 255, 455, 499, 1404, 1617, 1431, 1061, 907, 275, 245, 128, 247, 129, 63, 48, 47, 28, 24, 9, 7, 1, 540, 403, 317, 397, 288, 262, 222, 212, 341, 229, 287, 299, 262, 302, 710, 1385, 1813, 1570, 757, 441, 356, 144, 151, 144, 109, 94, 29, 15, 16, 5, 4, 2, 380, 391, 298, 317, 235, 255, 184, 185, 201, 191, 219, 188, 226, 203, 411, 763, 1678, 1860, 1374, 938, 990, 237, 164, 96, 352, 46, 34, 27, 13, 8, 2, 4, 371, 336, 291, 217, 197, 199, 263, 270, 165, 190, 148, 160, 162, 195, 294, 236, 637, 966, 991, 1018, 921, 379, 638, 146, 80, 72, 34, 25, 13, 15, 1, 4, 403, 264, 288, 204, 181, 211, 161, 167, 169, 174, 155, 153, 155, 252, 180, 218, 308, 497, 870, 1050, 1725, 807, 885, 204, 156, 80, 61, 18, 21, 17, 4, 2, 334, 264, 240, 159, 134, 150, 213, 151, 189, 150, 133, 201, 166, 113, 138, 253, 176, 188, 337, 992, 1170, 1309, 977, 220, 178, 118, 25, 29, 12, 5, 5, 2, 292, 215, 226, 200, 153, 168, 175, 165, 188, 198, 127, 157, 130, 113, 179, 348, 567, 222, 194, 532, 1333, 1374, 1014, 786, 660, 123, 122, 32, 23, 13, 2, 5, 272, 223, 166, 156, 161, 130, 154, 119, 144, 141, 197, 114, 111, 117, 132, 113, 113, 158, 224, 184, 425, 1258, 935, 1044, 713, 481, 79, 51, 37, 19, 4, 2, 235, 185, 147, 145, 111, 123, 143, 125, 113, 109, 112, 114, 96, 128, 141, 112, 92, 98, 103, 138, 174, 190, 1038, 547, 1016, 264, 431, 38, 14, 3, 5, 6, 240, 162, 192, 135, 123, 124, 147, 108, 131, 97, 98, 115, 116, 138, 117, 107, 80, 117, 120, 120, 140, 130, 506, 621, 781, 1263, 513, 112, 10, 58, 7, 1, 191, 160, 140, 127, 128, 113, 108, 104, 119, 87, 125, 130, 135, 107, 86, 89, 142, 101, 124, 82, 107, 67, 129, 389, 855, 868, 690, 136, 332, 24, 13, 7, 213, 163, 139, 124, 129, 131, 109, 98, 99, 93, 92, 92, 104, 83, 92, 82, 106, 71, 83, 92, 84, 58, 60, 47, 151, 429, 270, 506, 245, 121, 15, 2, 183, 163, 137, 119, 116, 99, 118, 98, 99, 100, 92, 83, 101, 105, 94, 74, 80, 70, 70, 61, 72, 60, 152, 46, 41, 69, 600, 539, 280, 57, 59, 3, 169, 152, 146, 91, 116, 102, 100, 90, 89, 88, 95, 76, 77, 85, 86, 84, 72, 83, 78, 61, 67, 41, 47, 40, 28, 25, 45, 82, 224, 706, 99, 3, 161, 137, 114, 116, 96, 87, 114, 91, 105, 95, 87, 89, 90, 88, 79, 74, 84, 72, 58, 58, 65, 57, 42, 44, 36, 23, 19, 12, 358, 249, 473, 0, 143, 127, 118, 111, 101, 92, 91, 104, 85, 90, 87, 97, 91, 80, 78, 78, 68, 87, 61, 56, 63, 45, 39, 37, 27, 22, 9, 8, 6, 118, 150, 7, 277, 134, 133, 98, 98, 110, 91, 91, 102, 106, 89, 87, 76, 83, 65, 82, 65, 92, 59, 57, 52, 47, 46, 37, 23, 16, 14, 10, 4, 4, 70, 237, },
	{87840, 61462, 36834, 24364, 14122, 10673, 7825, 5566, 4604, 4374, 3075, 2592, 2277, 2175, 2080, 1746, 1800, 1807, 1472, 1599, 1085, 1036, 759, 965, 521, 907, 339, 1033, 195, 2899, 169, 13226, 65970, 48421, 34608, 20638, 13208, 9279, 6896, 5053, 3627, 3096, 3143, 2068, 2003, 2036, 1577, 1498, 1255, 1312, 1342, 1188, 1027, 811, 752, 615, 451, 456, 337, 233, 185, 142, 150, 246, 48963, 38333, 30505, 20304, 11952, 8647, 6799, 4984, 3954, 2772, 2629, 2949, 2226, 2034, 1680, 1555, 1125, 1209, 1254, 1077, 878, 824, 610, 512, 490, 380, 297, 265, 161, 125, 108, 101, 33340, 31055, 26467, 19925, 11915, 7946, 5727, 4600, 3046, 2760, 2503, 2600, 2003, 1582, 1347, 1141, 1230, 1035, 1222, 1004, 831, 692, 574, 535, 449, 356, 275, 236, 192, 136, 105, 83, 23178, 19810, 19785, 16345, 10512, 6810, 5571, 4053, 3279, 2850, 2382, 2401, 1821, 1492, 1329, 1210, 1263, 1034, 1088, 999, 928, 696, 623, 497, 379, 322, 237, 197, 183, 123, 94, 59, 17578, 14989, 14566, 13648, 10250, 7042, 5197, 4113, 3739, 2538, 2304, 2298, 2031, 1534, 1223, 1201, 1118, 1079, 1148, 932, 781, 604, 624, 446, 353, 298, 258, 203, 145, 117, 78, 50, 14315, 12599, 10708, 10910, 9282, 7920, 5566, 4097, 3296, 2962, 2015, 2010, 1945, 1868, 1120, 1093, 1493, 1012, 950, 751, 703, 655, 690, 475, 449, 318, 273, 170, 127, 115, 71, 54, 11274, 9381, 8255, 8654, 8062, 7173, 6124, 3990, 3720, 2237, 1832, 2384, 1582, 1284, 1022, 1028, 860, 894, 851, 775, 685, 593, 541, 426, 367, 330, 407, 178, 118, 88, 71, 43, 8039, 6322, 6756, 5994, 6663, 6983, 6260, 4822, 2998, 3036, 2056, 1733, 1704, 1722, 1429, 1091, 846, 839, 675, 642, 773, 749, 490, 465, 403, 271, 241, 147, 101, 80, 61, 50, 6637, 5571, 4917, 3972, 5232, 5598, 5854, 4939, 3905, 2295, 2398, 2356, 1744, 1425, 948, 847, 815, 809, 677, 601, 662, 556, 472, 442, 402, 306, 254, 148, 89, 59, 62, 46, 6337, 4559, 3755, 3812, 3974, 5498, 5286, 5263, 3581, 3000, 2152, 2172, 1949, 1507, 1298, 1657, 917, 676, 628, 881, 589, 474, 426, 399, 366, 294, 166, 126, 78, 64, 43, 47, 5575, 4122, 3678, 2834, 3048, 3602, 4913, 4401, 4071, 3699, 2013, 2072, 2401, 1574, 962, 1195, 1103, 931, 603, 659, 737, 799, 386, 360, 343, 319, 329, 97, 79, 66, 36, 50, 4449, 3591, 3206, 2296, 3352, 2487, 3428, 4624, 3935, 2464, 2648, 1675, 1351, 2125, 943, 1379, 1169, 760, 674, 714, 548, 411, 403, 345, 334, 291, 296, 117, 58, 49, 37, 30, 4610, 3667, 2833, 2073, 2057, 2234, 2139, 3319, 3843, 3290, 2355, 2982, 1559, 1736, 1621, 1222, 1016, 1370, 661, 691, 428, 717, 474, 394, 289, 193, 180, 124, 71, 40, 38, 30, 4121, 3424, 2674, 1841, 1656, 1499, 1379, 2209, 3272, 2978, 2223, 2383, 1806, 1296, 1428, 695, 1159, 1160, 673, 765, 428, 406, 606, 537, 467, 435, 124, 103, 68, 45, 36, 31, 3425, 2828, 2228, 1532, 1784, 1474, 1397, 1375, 2409, 2546, 2353, 1961, 1724, 1095, 1059, 1292, 882, 768, 967, 465, 700, 419, 305, 445, 249, 179, 129, 95, 70, 49, 27, 20, 2938, 2283, 1900, 1339, 1248, 1115, 1095, 1050, 1263, 2497, 1695, 2046, 1798, 1106, 1071, 1023, 1234, 519, 839, 760, 909, 387, 412, 250, 200, 176, 117, 86, 61, 40, 38, 25, 2664, 2163, 1714, 1254, 1049, 1047, 920, 899, 1014, 1845, 2052, 1309, 1327, 1359, 1099, 824, 825, 645, 683, 463, 437, 745, 557, 429, 387, 452, 129, 90, 51, 40, 25, 31, 2338, 2002, 1469, 1073, 1060, 872, 802, 793, 744, 1093, 1985, 1519, 1247, 1380, 1220, 921, 825, 670, 467, 449, 391, 585, 306, 372, 236, 161, 99, 73, 40, 37, 30, 32, 2178, 1803, 1390, 1029, 842, 726, 768, 708, 669, 729, 1096, 2462, 1359, 1092, 1116, 866, 708, 615, 489, 496, 352, 327, 487, 527, 202, 644, 125, 68, 48, 36, 20, 33, 1877, 1747, 1628, 1030, 784, 852, 785, 602, 881, 661, 1333, 1624, 2449, 1398, 1044, 869, 908, 1006, 617, 476, 433, 467, 365, 213, 699, 156, 186, 76, 45, 37, 28, 19, 1685, 1636, 1111, 804, 742, 663, 619, 579, 579, 598, 939, 1885, 1621, 2245, 958, 894, 1254, 1158, 692, 658, 660, 439, 247, 235, 168, 158, 115, 73, 32, 32, 13, 32, 1494, 1621, 1095, 808, 1151, 605, 616, 545, 658, 579, 633, 700, 2041, 2149, 1039, 688, 1066, 1027, 1086, 638, 481, 502, 377, 357, 184, 170, 82, 62, 296, 27, 24, 27, 1374, 1175, 1000, 681, 756, 613, 526, 536, 514, 491, 497, 517, 1129, 1655, 1865, 705, 844, 1280, 1106, 700, 593, 500, 359, 416, 166, 150, 115, 144, 59, 291, 24, 15, 1271, 1163, 993, 681, 597, 527, 496, 524, 487, 427, 459, 470, 630, 1293, 837, 1135, 592, 680, 1167, 683, 487, 414, 378, 196, 156, 131, 309, 65, 95, 40, 21, 15, 1142, 1050, 883, 690, 515, 479, 482, 415, 462, 507, 456, 616, 474, 485, 1048, 773, 600, 737, 701, 670, 550, 325, 361, 270, 185, 109, 77, 142, 22, 35, 21, 18, 1110, 1016, 898, 659, 517, 465, 422, 431, 428, 467, 422, 431, 482, 455, 480, 780, 640, 565, 467, 438, 529, 334, 296, 216, 173, 95, 129, 67, 193, 37, 55, 16, 967, 881, 992, 604, 435, 426, 427, 377, 389, 438, 387, 375, 436, 433, 422, 619, 985, 551, 395, 362, 368, 445, 290, 218, 158, 92, 79, 45, 19, 16, 17, 13, 835, 875, 729, 561, 434, 440, 418, 371, 374, 394, 387, 365, 389, 563, 334, 864, 767, 895, 305, 348, 315, 330, 604, 242, 134, 111, 64, 56, 25, 26, 12, 9, 825, 713, 692, 525, 565, 414, 392, 378, 424, 388, 390, 399, 388, 403, 350, 364, 1065, 775, 629, 330, 293, 343, 359, 317, 159, 126, 76, 51, 30, 14, 10, 6, 643, 666, 644, 498, 422, 384, 403, 361, 421, 411, 368, 387, 382, 344, 359, 313, 367, 1237, 470, 335, 272, 225, 251, 456, 212, 72, 66, 32, 20, 20, 15, 7, 1073, 605, 560, 506, 397, 404, 401, 387, 394, 362, 399, 359, 343, 325, 346, 312, 306, 450, 1371, 487, 290, 234, 205, 186, 245, 117, 60, 44, 22, 16, 12, 10, },
	{99623, 60682, 36074, 24816, 15186, 12307, 8972, 6979, 5622, 5227, 4284, 3580, 2993, 2847, 2401, 2352, 2480, 2316, 1898, 1812, 1331, 1364, 933, 1298, 621, 1128, 343, 1714, 278, 5085, 191, 26092, 79616, 51205, 32818, 21644, 13388, 10089, 8121, 6101, 4605, 4043, 3425, 2742, 2613, 2269, 1990, 1984, 2068, 1939, 1533, 1310, 1128, 994, 828, 676, 505, 418, 321, 271, 198, 201, 185, 398, 65325, 44636, 29599, 18148, 12286, 8900, 6897, 5338, 4233, 3698, 3140, 2807, 2309, 2118, 1905, 1856, 1878, 1746, 1450, 1216, 980, 890, 737, 591, 492, 379, 313, 276, 213, 188, 161, 232, 48782, 40373, 27743, 18042, 10645, 7658, 5806, 4612, 3837, 3474, 3112, 2427, 2217, 1943, 1754, 1793, 1750, 1774, 1361, 1111, 944, 834, 633, 566, 396, 354, 310, 232, 208, 168, 164, 128, 34150, 31481, 24637, 15435, 8487, 6852, 4923, 4019, 3661, 3354, 2733, 2421, 2094, 1852, 1691, 1676, 1706, 1664, 1318, 1055, 914, 757, 627, 510, 369, 344, 298, 235, 184, 176, 160, 107, 26268, 27005, 22309, 14513, 7562, 5802, 4362, 3672, 3294, 2928, 2385, 2107, 1866, 1736, 1740, 1738, 1706, 1676, 1402, 1066, 892, 724, 627, 487, 386, 343, 246, 223, 181, 184, 147, 98, 21257, 23467, 20809, 14510, 7390, 5331, 4180, 3383, 2735, 2419, 2167, 1917, 1722, 1607, 1701, 1602, 1515, 1536, 1416, 1065, 899, 744, 588, 485, 422, 327, 259, 197, 198, 134, 117, 98, 16099, 17811, 17843, 12981, 7245, 4842, 3681, 3097, 2413, 2227, 1990, 1854, 1706, 1569, 1456, 1502, 1363, 1388, 1324, 1031, 855, 691, 527, 444, 376, 325, 237, 206, 186, 136, 98, 78, 10894, 13401, 14645, 11343, 7050, 4942, 3523, 2929, 2376, 2154, 1997, 1903, 1626, 1543, 1444, 1442, 1140, 1164, 1251, 1038, 791, 668, 541, 420, 362, 277, 225, 214, 171, 110, 93, 97, 8396, 10393, 11958, 10581, 7097, 4926, 3359, 2651, 2231, 1937, 1830, 1647, 1478, 1416, 1468, 1285, 1063, 1030, 1115, 1025, 771, 632, 512, 458, 346, 274, 212, 220, 125, 111, 94, 87, 6689, 7528, 10176, 9530, 6690, 4704, 3394, 2468, 2167, 1942, 1728, 1520, 1349, 1348, 1319, 1225, 981, 947, 1054, 978, 739, 620, 539, 438, 350, 272, 231, 202, 109, 97, 69, 61, 5663, 5946, 8247, 8569, 6020, 4399, 3554, 2446, 2058, 1853, 1570, 1379, 1327, 1271, 1147, 1104, 929, 863, 957, 930, 675, 562, 446, 383, 326, 247, 247, 200, 89, 93, 85, 68, 4892, 4834, 6362, 7533, 5570, 3932, 3401, 2426, 1771, 1739, 1492, 1334, 1284, 1178, 1047, 992, 958, 849, 911, 898, 753, 605, 468, 378, 326, 293, 223, 159, 92, 68, 62, 44, 4197, 4014, 4796, 6584, 5353, 3500, 3201, 2412, 1769, 1555, 1335, 1291, 1232, 1015, 932, 865, 828, 788, 765, 843, 657, 560, 454, 311, 238, 256, 188, 132, 76, 67, 60, 51, 3775, 3397, 3806, 5376, 5306, 3446, 2645, 2431, 1842, 1421, 1264, 1185, 1158, 995, 956, 903, 805, 766, 702, 734, 661, 538, 388, 279, 268, 244, 153, 98, 49, 51, 57, 37, 3108, 2936, 2880, 4454, 4823, 3350, 2233, 2093, 1801, 1320, 1042, 1109, 1024, 1025, 859, 771, 746, 750, 618, 662, 606, 501, 386, 290, 255, 189, 163, 112, 52, 48, 37, 44, 2662, 2641, 2287, 3475, 4398, 3731, 2310, 1903, 1711, 1308, 1048, 1043, 987, 918, 790, 742, 755, 677, 614, 592, 552, 459, 392, 282, 217, 157, 90, 82, 52, 46, 35, 26, 2386, 2320, 2035, 2696, 3900, 3731, 2339, 1664, 1455, 1315, 1057, 898, 846, 780, 697, 685, 658, 622, 608, 569, 549, 439, 331, 290, 196, 146, 107, 68, 42, 46, 33, 25, 2167, 2113, 1859, 2097, 3392, 3736, 2480, 1593, 1399, 1164, 995, 827, 816, 792, 739, 646, 642, 622, 573, 514, 485, 421, 333, 249, 183, 125, 108, 73, 26, 40, 27, 27, 1834, 1837, 1588, 1829, 2936, 3514, 2444, 1473, 1151, 1076, 969, 854, 728, 711, 654, 633, 641, 581, 533, 495, 443, 418, 332, 221, 159, 164, 86, 63, 33, 33, 30, 23, 1535, 1506, 1339, 1502, 2207, 3233, 2461, 1521, 1132, 979, 918, 878, 754, 689, 624, 657, 592, 606, 539, 501, 457, 385, 285, 228, 154, 123, 98, 70, 23, 26, 19, 21, 1314, 1273, 1173, 1154, 1690, 2693, 2401, 1524, 1035, 842, 871, 862, 750, 689, 654, 659, 607, 575, 517, 438, 408, 303, 286, 190, 153, 123, 78, 55, 20, 24, 22, 13, 1219, 1105, 1051, 1014, 1297, 2069, 2286, 1444, 950, 920, 810, 805, 766, 647, 613, 612, 577, 501, 490, 425, 380, 376, 270, 182, 127, 107, 63, 52, 19, 16, 15, 16, 1039, 964, 947, 876, 1121, 1637, 1919, 1410, 929, 851, 745, 792, 739, 675, 624, 573, 572, 533, 495, 420, 357, 301, 232, 176, 111, 91, 72, 42, 22, 17, 19, 9, 976, 960, 914, 901, 886, 1323, 1711, 1443, 1001, 891, 851, 758, 726, 673, 554, 572, 541, 525, 440, 401, 319, 258, 229, 166, 107, 90, 43, 47, 17, 17, 14, 19, 930, 880, 838, 776, 758, 1035, 1514, 1414, 975, 966, 823, 710, 745, 672, 640, 602, 520, 481, 461, 369, 304, 240, 223, 155, 108, 98, 61, 37, 11, 17, 12, 20, 920, 851, 825, 744, 660, 892, 1230, 1273, 1042, 906, 855, 661, 710, 649, 650, 558, 550, 473, 417, 390, 292, 200, 172, 130, 95, 81, 49, 33, 5, 19, 6, 8, 808, 773, 792, 724, 668, 725, 1036, 1139, 1066, 957, 811, 662, 655, 550, 586, 529, 500, 478, 429, 360, 291, 223, 172, 144, 82, 69, 51, 28, 10, 15, 9, 7, 811, 651, 720, 653, 621, 647, 891, 1123, 1006, 878, 762, 742, 630, 605, 547, 507, 462, 431, 367, 348, 264, 212, 150, 117, 82, 71, 41, 23, 8, 8, 9, 7, 721, 676, 717, 680, 557, 624, 825, 1032, 1087, 952, 798, 723, 651, 586, 566, 475, 465, 437, 357, 328, 255, 224, 154, 118, 99, 49, 33, 25, 8, 6, 13, 8, 669, 666, 599, 589, 554, 580, 706, 894, 1103, 837, 739, 690, 667, 586, 533, 540, 493, 410, 367, 302, 253, 219, 171, 127, 71, 52, 50, 24, 9, 9, 7, 6, 1031, 673, 588, 598, 555, 565, 665, 954, 1188, 1019, 765, 661, 644, 582, 569, 517, 482, 424, 345, 300, 277, 208, 157, 129, 87, 69, 26, 20, 15, 14, 7, 5, },
	{3263, 2590, 1945, 1269, 854, 652, 554, 451, 343, 283, 242, 218, 169, 176, 165, 145, 161, 149, 108, 113, 78, 90, 67, 71, 49, 57, 28, 85, 18, 191, 17, 821, 2630, 2180, 1697, 1192, 768, 605, 459, 358, 284, 249, 200, 183, 164, 153, 140, 125, 89, 109, 92, 78, 80, 62, 59, 41, 42, 39, 28, 24, 13, 6, 11, 16, 2193, 1880, 1603, 1117, 726, 551, 435, 347, 265, 238, 193, 183, 164, 142, 131, 117, 105, 92, 98, 70, 62, 58, 61, 51, 31, 28, 22, 17, 14, 12, 5, 9, 1768, 1631, 1438, 1163, 770, 555, 407, 332, 244, 222, 200, 159, 154, 137, 116, 122, 103, 113, 83, 82, 65, 60, 38, 52, 43, 40, 25, 13, 11, 7, 11, 7, 1332, 1230, 1120, 959, 657, 492, 355, 278, 253, 197, 170, 165, 123, 140, 138, 127, 103, 91, 91, 76, 67, 68, 44, 51, 41, 37, 21, 20, 14, 5, 8, 8, 1226, 1057, 891, 786, 655, 539, 379, 275, 229, 203, 182, 154, 134, 144, 114, 111, 88, 101, 84, 80, 64, 69, 53, 38, 40, 29, 28, 14, 16, 8, 10, 5, 1036, 845, 756, 643, 557, 522, 434, 326, 221, 214, 173, 140, 132, 110, 99, 93, 95, 87, 76, 76, 67, 51, 58, 44, 40, 34, 27, 21, 9, 11, 5, 4, 890, 723, 639, 507, 432, 433, 429, 292, 212, 174, 150, 149, 113, 114, 90, 99, 88, 87, 78, 82, 58, 42, 43, 46, 32, 42, 24, 11, 10, 6, 1, 2, 743, 581, 502, 438, 373, 358, 362, 290, 238, 182, 154, 128, 112, 109, 102, 92, 88, 81, 82, 60, 68, 49, 45, 44, 29, 24, 25, 25, 10, 9, 1, 5, 619, 529, 451, 355, 314, 296, 309, 294, 308, 240, 168, 126, 126, 109, 102, 91, 87, 79, 58, 66, 59, 55, 37, 31, 28, 31, 22, 18, 9, 5, 7, 5, 542, 440, 370, 309, 272, 233, 240, 270, 263, 258, 179, 133, 116, 108, 90, 104, 95, 79, 77, 64, 57, 48, 43, 34, 35, 29, 21, 22, 9, 9, 6, 3, 520, 414, 332, 305, 223, 209, 215, 226, 225, 216, 178, 145, 117, 88, 85, 82, 77, 83, 72, 62, 60, 40, 39, 31, 32, 25, 21, 12, 4, 5, 8, 2, 435, 343, 312, 247, 222, 159, 166, 165, 183, 182, 156, 164, 142, 103, 90, 81, 79, 66, 66, 80, 41, 50, 45, 28, 26, 24, 20, 16, 9, 6, 3, 5, 433, 379, 307, 203, 167, 149, 142, 139, 154, 158, 160, 186, 184, 129, 73, 80, 67, 66, 71, 58, 46, 44, 39, 35, 36, 19, 14, 14, 5, 2, 2, 3, 434, 358, 287, 200, 153, 138, 109, 118, 130, 148, 136, 154, 189, 135, 98, 70, 83, 69, 66, 64, 46, 43, 38, 35, 19, 22, 16, 7, 7, 8, 5, 2, 362, 327, 219, 172, 143, 134, 135, 115, 112, 109, 123, 114, 122, 121, 94, 72, 73, 67, 35, 57, 48, 47, 30, 24, 27, 14, 12, 10, 5, 3, 2, 2, 298, 203, 198, 155, 123, 111, 99, 103, 97, 82, 91, 104, 95, 95, 112, 93, 75, 69, 51, 52, 47, 43, 30, 30, 26, 11, 8, 11, 5, 5, 2, 1, 254, 212, 176, 154, 115, 120, 99, 87, 87, 76, 85, 72, 105, 85, 94, 109, 101, 68, 62, 50, 40, 39, 29, 22, 19, 18, 11, 12, 5, 2, 5, 3, 239, 184, 154, 118, 112, 107, 107, 92, 72, 73, 83, 78, 78, 74, 70, 107, 105, 66, 50, 44, 49, 27, 32, 33, 22, 17, 11, 7, 5, 3, 2, 3, 230, 172, 153, 120, 110, 87, 87, 74, 84, 71, 73, 70, 90, 83, 80, 66, 74, 64, 54, 55, 42, 32, 36, 22, 19, 15, 7, 4, 4, 5, 1, 3, 213, 151, 134, 109, 101, 95, 83, 60, 72, 62, 74, 63, 68, 74, 72, 65, 53, 49, 59, 47, 52, 29, 24, 25, 11, 12, 9, 7, 3, 3, 2, 0, 173, 169, 127, 111, 80, 75, 75, 65, 64, 63, 62, 65, 54, 58, 64, 71, 70, 56, 55, 65, 38, 29, 28, 22, 13, 13, 12, 5, 3, 3, 3, 1, 185, 130, 106, 105, 91, 68, 61, 63, 61, 46, 61, 62, 62, 57, 64, 74, 53, 58, 49, 57, 31, 26, 30, 22, 22, 12, 8, 7, 4, 4, 0, 2, 176, 113, 106, 85, 81, 71, 61, 53, 64, 71, 48, 68, 45, 57, 47, 58, 59, 68, 42, 35, 41, 33, 24, 23, 18, 16, 5, 9, 2, 1, 0, 0, 164, 138, 87, 83, 69, 72, 65, 64, 52, 57, 44, 52, 53, 48, 44, 37, 44, 49, 45, 35, 33, 37, 18, 16, 18, 8, 2, 5, 3, 1, 3, 1, 160, 129, 102, 67, 74, 56, 49, 50, 57, 52, 49, 39, 44, 54, 38, 47, 37, 40, 40, 32, 28, 21, 21, 14, 13, 6, 7, 3, 4, 2, 2, 0, 144, 106, 85, 81, 65, 37, 57, 52, 51, 52, 42, 57, 46, 42, 35, 46, 47, 38, 31, 32, 31, 30, 21, 15, 14, 6, 6, 2, 2, 0, 1, 0, 125, 102, 84, 86, 60, 56, 53, 39, 50, 44, 38, 58, 44, 39, 41, 41, 44, 32, 37, 38, 24, 19, 20, 9, 14, 10, 5, 4, 0, 0, 1, 0, 109, 109, 96, 70, 47, 59, 45, 39, 40, 47, 32, 36, 44, 22, 46, 33, 49, 34, 38, 34, 23, 21, 14, 15, 13, 4, 5, 3, 1, 1, 0, 0, 119, 91, 65, 54, 46, 51, 40, 39, 42, 53, 39, 50, 44, 46, 34, 40, 38, 28, 37, 34, 19, 23, 13, 14, 9, 11, 7, 2, 0, 2, 1, 0, 99, 79, 61, 52, 57, 47, 49, 32, 41, 43, 40, 44, 43, 34, 31, 43, 35, 41, 31, 25, 19, 16, 18, 12, 5, 10, 6, 3, 3, 1, 3, 0, 114, 82, 69, 55, 51, 37, 42, 40, 41, 44, 50, 41, 41, 34, 42, 37, 29, 31, 36, 29, 29, 22, 17, 15, 13, 7, 7, 2, 2, 0, 0, 2, },
};

static UInt FIX_SCANSTATS16x16[9][256] = 
{
	{3688037, 1843518, 884188, 564825, 331873, 260809, 192028, 154095, 130378, 112001, 88189, 69509, 40676, 40076, 16041, 131731, 2760331, 1337865, 649221, 412823, 234418, 180279, 131193, 105485, 87533, 76572, 61063, 41610, 26042, 16791, 10014, 8072, 2130261, 924793, 412764, 269708, 181031, 141405, 105302, 82389, 74215, 63628, 51493, 34777, 23603, 14238, 8532, 5935, 1800053, 771778, 331535, 213183, 148357, 113040, 85644, 69910, 61454, 52558, 42411, 29228, 19775, 11570, 6445, 4474, 1463114, 566717, 265377, 174718, 123554, 97077, 74580, 59950, 52926, 44924, 37839, 27038, 19135, 10682, 6567, 4418, 1262202, 484124, 209854, 131977, 96728, 74132, 59404, 48751, 42937, 37755, 31762, 22561, 15187, 8661, 4756, 3137, 1230906, 431583, 188469, 110997, 79245, 63076, 51576, 43364, 39428, 33787, 30112, 21204, 15182, 8347, 4527, 2999, 1092407, 384689, 157505, 88889, 65212, 51353, 42686, 36920, 33334, 28609, 24954, 18043, 11845, 6561, 2985, 2133, 980531, 387946, 148695, 78163, 53547, 42319, 39590, 33970, 31440, 27452, 23759, 16001, 11824, 6202, 3437, 2162, 829629, 332296, 122068, 64809, 44464, 36561, 33628, 29166, 27134, 23222, 20276, 14175, 9278, 4821, 2109, 1484, 719653, 272718, 102496, 54537, 42151, 33954, 32245, 28406, 26413, 21986, 20612, 13831, 9619, 4934, 2381, 1514, 616937, 227055, 86812, 48779, 36489, 30775, 29540, 26130, 23234, 20005, 16897, 11714, 7251, 3801, 1551, 1020, 553811, 196776, 80005, 46078, 35961, 29852, 29847, 25086, 23934, 18622, 16712, 10679, 7345, 3640, 1579, 1086, 479674, 170585, 66640, 40814, 31830, 27502, 26546, 23299, 20229, 16696, 13615, 8837, 5436, 2653, 1056, 695, 417268, 139018, 61566, 38502, 33034, 27232, 26605, 21242, 20617, 15039, 12596, 7852, 5238, 2279, 789, 610, 354595, 117010, 53681, 36473, 30177, 26564, 24650, 21043, 18889, 14901, 11741, 7370, 4908, 2068, 596, 598, },
	{4378702, 2818177, 1883569, 1580227, 1239446, 1149356, 1023376, 935803, 832662, 748668, 626251, 531043, 385636, 309665, 198536, 238349, 2059485, 1287638, 816811, 628284, 493889, 427265, 378609, 328399, 309085, 255747, 203129, 146051, 91746, 58484, 34465, 23233, 968634, 598342, 349536, 258667, 210041, 174237, 148282, 131252, 127190, 105723, 81014, 58048, 39729, 24689, 14148, 9450, 681413, 428805, 252190, 182518, 144768, 119371, 99711, 84260, 72811, 62101, 47575, 34561, 24270, 14820, 7593, 4751, 430828, 253449, 172570, 130313, 105027, 86887, 73075, 62071, 55122, 44566, 36060, 26595, 19861, 11563, 6713, 4258, 324173, 205742, 135937, 100948, 83820, 67600, 58536, 48993, 41562, 35095, 29406, 20477, 15183, 9326, 5209, 3471, 261345, 153044, 102550, 74232, 63069, 54797, 48309, 42721, 37322, 30158, 25555, 17887, 13580, 7735, 4342, 2835, 203285, 121243, 79383, 58176, 49141, 42920, 39621, 33135, 30472, 25342, 20923, 14946, 10260, 5643, 2547, 1671, 176726, 99640, 66607, 47667, 41017, 35234, 33239, 30294, 27072, 22891, 19094, 12559, 9345, 4784, 2520, 1715, 133691, 79750, 53013, 38468, 32421, 29645, 27918, 25206, 23174, 18769, 15829, 10677, 7411, 3953, 1703, 1097, 113355, 65144, 42797, 32383, 30279, 26790, 27062, 24089, 22430, 18276, 15934, 10521, 7346, 3813, 1819, 1247, 87544, 56827, 36595, 28466, 26338, 24387, 24090, 21881, 19809, 16383, 13612, 9161, 5887, 3035, 1237, 890, 75687, 47376, 34296, 27291, 26635, 23826, 24659, 21260, 20055, 15797, 13674, 8708, 6418, 3066, 1460, 874, 60795, 40975, 30130, 25537, 23639, 22323, 22114, 19622, 17727, 14219, 11639, 7510, 5038, 2525, 994, 590, 54164, 36868, 30650, 24983, 24761, 21781, 22047, 18295, 17681, 12942, 11095, 7046, 4998, 2188, 798, 488, 53013, 34857, 28058, 24036, 23181, 21637, 20744, 18129, 16249, 12602, 10178, 6759, 4410, 2020, 593, 425, },
	{4794756, 3556290, 2170736, 1625167, 1081936, 877215, 705207, 591984, 507350, 423929, 314336, 272234, 156605, 177289, 62003, 556810, 3584877, 2573048, 1657585, 1223867, 835387, 662773, 551614, 450944, 374870, 312581, 246292, 183244, 119107, 75282, 44239, 31342, 2312345, 1724589, 1134446, 860890, 682869, 547460, 454894, 374914, 320557, 264431, 201568, 148030, 102737, 64868, 37244, 24266, 1756568, 1362299, 914648, 713684, 569396, 461283, 384243, 312890, 271646, 227543, 171838, 124428, 87513, 53391, 28306, 18797, 1182108, 926809, 739001, 588739, 480516, 401052, 332182, 277422, 235351, 193340, 152434, 113821, 81484, 47254, 27746, 18048, 938406, 750834, 594816, 475158, 392923, 326640, 281665, 237595, 201252, 165559, 133132, 96494, 66643, 39634, 20728, 13977, 776797, 604751, 490699, 387094, 319713, 270727, 241965, 207671, 181748, 150416, 121073, 88074, 62672, 35505, 18860, 12473, 631844, 490054, 391208, 319188, 261685, 227116, 200621, 175152, 156035, 129233, 105184, 76659, 51666, 29152, 12922, 8941, 535085, 408763, 333865, 265875, 216478, 185462, 173339, 155304, 139965, 118980, 97576, 68790, 49604, 26037, 13576, 9198, 431851, 326960, 263790, 219340, 178068, 160069, 149493, 131589, 120662, 101419, 84160, 61585, 41214, 22081, 9503, 6392, 348431, 265114, 208797, 175634, 155847, 138273, 132641, 122861, 115502, 95483, 81200, 56592, 40119, 20081, 9309, 6437, 281561, 220415, 173364, 150136, 136193, 123585, 119827, 112549, 100888, 86842, 70454, 49637, 32084, 17486, 6721, 4640, 245147, 190391, 155525, 133965, 128818, 115791, 115783, 105115, 98412, 80407, 67744, 45759, 32231, 15917, 7285, 4825, 209281, 168392, 135441, 119246, 113193, 106190, 104416, 98221, 87370, 73622, 58506, 40818, 26739, 13132, 4927, 3489, 180566, 144613, 128068, 112342, 110421, 101027, 101245, 89634, 85216, 66000, 54292, 37524, 25731, 12052, 4437, 3233, 190509, 133630, 117401, 106398, 102312, 98438, 94126, 87444, 79796, 64047, 51384, 35815, 24006, 11204, 3670, 3459, },
	{623213, 524182, 378662, 305368, 201577, 153073, 111166, 83624, 58085, 43258, 29207, 22246, 13311, 16521, 5708, 47774, 395205, 338124, 257486, 222045, 171545, 151425, 122183, 95405, 75017, 52329, 36557, 25467, 16086, 8623, 3697, 1537, 196695, 161642, 118231, 104610, 102053, 101259, 99024, 84715, 78928, 62465, 39465, 25624, 16184, 9302, 4199, 1795, 137425, 111479, 81920, 67567, 61123, 60223, 56822, 59704, 55891, 53361, 42002, 30455, 21106, 10660, 3470, 1540, 81914, 64387, 56589, 46906, 42072, 36378, 35030, 33642, 34120, 32324, 29724, 25816, 18427, 12241, 8777, 3652, 66092, 55265, 44599, 35094, 30295, 25394, 22488, 20636, 19045, 17103, 15420, 15602, 13051, 10465, 7146, 5353, 50475, 38338, 32682, 26593, 22806, 19212, 16640, 15124, 14161, 13100, 12055, 8964, 7434, 5244, 5118, 3178, 40081, 31530, 26080, 20858, 17507, 16772, 13727, 13340, 11145, 9718, 8182, 6796, 5621, 3340, 1588, 1379, 35183, 28792, 25011, 20704, 17629, 14509, 13285, 10894, 9434, 8067, 7231, 5300, 3954, 2483, 1301, 823, 44805, 41333, 29308, 22120, 14866, 11922, 10473, 9018, 8224, 7195, 6064, 4835, 3565, 1907, 806, 485, 24494, 20058, 17176, 14638, 13265, 11588, 10770, 9161, 7925, 7000, 5736, 4399, 3337, 1676, 591, 392, 21263, 17477, 13975, 12076, 10610, 9269, 8934, 8128, 7100, 6360, 5431, 4147, 2847, 1452, 493, 290, 18309, 14926, 12551, 11315, 10759, 9835, 9217, 8949, 7543, 6708, 5210, 3971, 2905, 1409, 530, 278, 19256, 15743, 12001, 10066, 9033, 8212, 7972, 7610, 6509, 5783, 4735, 3656, 2739, 1319, 381, 241, 14989, 12147, 11037, 9610, 9108, 8276, 7697, 7180, 6484, 5516, 4544, 3620, 2655, 1263, 346, 176, 18100, 10855, 9699, 8488, 8097, 7632, 7108, 7009, 6168, 5333, 4402, 3503, 2474, 1253, 313, 215, },
	{672813, 524923, 329076, 220976, 120582, 84135, 58077, 45347, 35676, 26304, 19790, 16706, 10094, 10697, 3728, 37915, 453765, 383838, 285272, 211745, 123536, 85414, 58899, 43919, 31784, 26083, 18921, 14493, 8752, 5552, 3403, 1886, 242033, 212077, 173183, 159575, 132902, 97127, 68608, 47678, 35559, 27741, 19653, 13243, 8106, 5015, 2525, 1350, 164659, 139340, 114862, 110319, 109571, 101955, 84234, 60685, 42599, 32012, 18871, 13751, 8648, 4681, 2093, 1200, 90043, 77113, 72985, 67741, 67634, 75435, 76001, 69385, 50511, 33803, 25775, 16329, 9321, 4696, 1983, 1040, 68623, 57101, 51664, 44390, 40538, 42080, 52708, 55354, 51441, 39610, 28562, 23036, 12569, 6094, 2613, 930, 51491, 41542, 37394, 32352, 28671, 26886, 28300, 37816, 39398, 37325, 32489, 22652, 16263, 9857, 4142, 1192, 38038, 31521, 28932, 26126, 20951, 20978, 18406, 19247, 25480, 31197, 29678, 23603, 18832, 10512, 5375, 2258, 31899, 26767, 23401, 18950, 15644, 13709, 13906, 12809, 14640, 16767, 21777, 24457, 18329, 11398, 5417, 3031, 29286, 21988, 17831, 14366, 11956, 10761, 10270, 9826, 9594, 10471, 13309, 15974, 15258, 10792, 6775, 2784, 20582, 16311, 13775, 12036, 10588, 9258, 8642, 8230, 7719, 7878, 8269, 6955, 9305, 8998, 8344, 5403, 16037, 12791, 11491, 9867, 8711, 7562, 7408, 6959, 6452, 5891, 5724, 4912, 4750, 5472, 4800, 4593, 14201, 11344, 10039, 9145, 8683, 7502, 7221, 6543, 6199, 5289, 4521, 3507, 2959, 2394, 2249, 3061, 13397, 10234, 8590, 7162, 7182, 7248, 6784, 6141, 5307, 4576, 3739, 2969, 2081, 1241, 1191, 721, 11008, 9010, 7989, 7142, 6857, 6632, 6563, 5821, 5222, 4236, 3559, 2670, 1974, 971, 514, 381, 12702, 7994, 7507, 7094, 6584, 6152, 5632, 5440, 4680, 4253, 3423, 2484, 1831, 896, 321, 285, },
	{485634, 348141, 201360, 130950, 74320, 51847, 39934, 30284, 24506, 19335, 13491, 11787, 6568, 9058, 2464, 38605, 350342, 282434, 182424, 121277, 68384, 47824, 33699, 25402, 20619, 15645, 11442, 9086, 5666, 3267, 1866, 1395, 212218, 182732, 142644, 109346, 71336, 47980, 32198, 24026, 19393, 13957, 10697, 7550, 4673, 3011, 1503, 873, 146577, 130513, 115659, 102870, 79302, 51215, 33435, 23639, 18706, 12882, 9717, 6412, 4488, 2523, 1249, 750, 90447, 77354, 77370, 82076, 73696, 61009, 35832, 21726, 17852, 13159, 8895, 6130, 4252, 2248, 1195, 735, 65881, 56178, 51953, 50322, 57475, 63951, 50439, 27682, 18668, 11647, 8309, 6262, 3940, 2058, 1089, 603, 48894, 40590, 37588, 33727, 35868, 49698, 54696, 39939, 23450, 13148, 8636, 5627, 3664, 1935, 895, 564, 36420, 29815, 26827, 25805, 22750, 27883, 39334, 41785, 32299, 16392, 10311, 6980, 4004, 1893, 641, 408, 27727, 23036, 20772, 18375, 17832, 18096, 22253, 30216, 34076, 30602, 14976, 8723, 5384, 1990, 802, 447, 20093, 16013, 14463, 13799, 12275, 13316, 12882, 15269, 28213, 34560, 27579, 15621, 5914, 2005, 612, 375, 15994, 12849, 10412, 10039, 9559, 8981, 9846, 11905, 12355, 25792, 30876, 24889, 10215, 3014, 849, 333, 11987, 10138, 8304, 7980, 7848, 7285, 7500, 8936, 10029, 12117, 24111, 23070, 17713, 6052, 1329, 402, 10043, 8504, 7193, 6827, 6251, 5963, 6045, 6415, 7291, 7743, 9177, 16538, 17218, 15485, 4780, 574, 8725, 6829, 6072, 5598, 5521, 5275, 5382, 5143, 5135, 4600, 5028, 4601, 12413, 15298, 9924, 1060, 7808, 6503, 5569, 5387, 5185, 4937, 5096, 4617, 4305, 3630, 3133, 3043, 2031, 6748, 8719, 4359, 9045, 5710, 4989, 4785, 4732, 4713, 4572, 4410, 4038, 3413, 2688, 2057, 1462, 790, 1557, 3514, },
	{653947, 458399, 255178, 159478, 97181, 70267, 52055, 44361, 40116, 30166, 20671, 19100, 10445, 15734, 3982, 65668, 497779, 385254, 233214, 147255, 85544, 62049, 46941, 37768, 29225, 26500, 18921, 13956, 9091, 5713, 3710, 3115, 329779, 282826, 193954, 130919, 83363, 59280, 47805, 34897, 26823, 22064, 16780, 12720, 8292, 5194, 3292, 2144, 231609, 230118, 179779, 137192, 84387, 55066, 41809, 31952, 24574, 20168, 15355, 11526, 7331, 4426, 2605, 1721, 142192, 135937, 150161, 130482, 90018, 54110, 36855, 28063, 21615, 16766, 14403, 10379, 7481, 4058, 2344, 1791, 102879, 95128, 106534, 114986, 92229, 58770, 37980, 28733, 21661, 14843, 12344, 8939, 6922, 3578, 1828, 1452, 79386, 72800, 71631, 90701, 88327, 65773, 39597, 25149, 19424, 16280, 11018, 8016, 5995, 3464, 1842, 1274, 58846, 53014, 51146, 58244, 71723, 65286, 44563, 24980, 19883, 14159, 10379, 8022, 4929, 3009, 1445, 961, 44926, 40883, 38090, 36777, 45778, 54268, 45241, 31017, 19205, 13805, 10241, 7317, 4928, 2886, 1536, 982, 34829, 29158, 28101, 25800, 27825, 35173, 39166, 29809, 21265, 15554, 9410, 6804, 4926, 2503, 1095, 753, 28459, 23292, 20600, 19559, 20447, 28179, 31509, 30955, 26530, 16555, 11573, 6710, 4669, 2758, 1225, 759, 22362, 20484, 16916, 15589, 16654, 18092, 26958, 29901, 27589, 19378, 14012, 7495, 4626, 2143, 897, 569, 18767, 16340, 15426, 13940, 14343, 13651, 19101, 22847, 25043, 21214, 14191, 7941, 4454, 2371, 1065, 739, 16757, 14301, 12430, 11720, 10977, 11275, 12632, 15204, 18887, 17851, 14059, 7782, 3781, 2079, 779, 454, 15417, 12454, 11066, 10904, 10690, 10292, 10380, 11335, 13446, 12788, 12632, 8028, 4095, 1727, 611, 435, 16744, 11567, 10454, 9688, 10265, 9898, 8908, 8942, 10818, 12159, 10080, 7664, 4706, 1687, 539, 429, },
	{1680661, 1027359, 527463, 355422, 220912, 178652, 142262, 128560, 128930, 103147, 66143, 62915, 35909, 55374, 14380, 255573, 1406244, 895823, 459345, 292436, 181979, 143849, 117583, 109535, 113059, 81830, 56629, 41430, 29550, 18364, 12468, 13001, 1108669, 719777, 371018, 232819, 160372, 126258, 107012, 99762, 104741, 76965, 48757, 34360, 25882, 15666, 10835, 9502, 908256, 687943, 357505, 210916, 140340, 108978, 96316, 91647, 94489, 77887, 46438, 30058, 20858, 13749, 9903, 8600, 662876, 548850, 362945, 200313, 129090, 98325, 88571, 83074, 76671, 68389, 45727, 27945, 19745, 13056, 10829, 9348, 505236, 487591, 352215, 196296, 116513, 86411, 77989, 69693, 62378, 61110, 43200, 24610, 17236, 11866, 9434, 7672, 396280, 413901, 342783, 201935, 115257, 79903, 66581, 60108, 51871, 53451, 40152, 23877, 16923, 11115, 8077, 5868, 286602, 320063, 293128, 199224, 110084, 72685, 56582, 49531, 43051, 42585, 34945, 21796, 14339, 9054, 5733, 3979, 201790, 254111, 251988, 183965, 105591, 66968, 49596, 42058, 38550, 36296, 31506, 19605, 13689, 7938, 4952, 3376, 149640, 188886, 202095, 163863, 92386, 63882, 46103, 37224, 34052, 29201, 26072, 17218, 10901, 6413, 2993, 2232, 112573, 140699, 161314, 138859, 87487, 63796, 47199, 36033, 33405, 27212, 24664, 16057, 10334, 5627, 2587, 2018, 84794, 102751, 127671, 121348, 79148, 60831, 46066, 35497, 30277, 24810, 21033, 13908, 7874, 4397, 1640, 1345, 71045, 75409, 106420, 102921, 75686, 54904, 46500, 35496, 30726, 23367, 19534, 12081, 8089, 4088, 1737, 1445, 58242, 57645, 77737, 87511, 72552, 50436, 41895, 34074, 28822, 21484, 16603, 11055, 6296, 3323, 1114, 1004, 50168, 44213, 61775, 77389, 70162, 45583, 39143, 31840, 28708, 19714, 15396, 9495, 5956, 2953, 1169, 888, 47086, 37843, 45280, 64534, 62476, 44090, 34846, 31366, 27608, 20774, 14537, 9416, 5339, 2789, 877, 1012, },
	{6042, 4296, 2715, 1995, 1287, 1079, 850, 666, 608, 544, 413, 402, 200, 403, 96, 1706, 4612, 3371, 2047, 1598, 1051, 880, 662, 551, 457, 420, 307, 215, 150, 107, 69, 57, 3229, 2354, 1423, 1074, 840, 696, 557, 457, 401, 309, 257, 182, 151, 81, 56, 33, 2612, 1844, 1170, 858, 679, 547, 467, 372, 334, 290, 253, 159, 110, 72, 45, 29, 1905, 1260, 971, 738, 600, 468, 376, 341, 309, 285, 224, 142, 119, 73, 59, 40, 1527, 1115, 816, 589, 510, 375, 376, 289, 296, 237, 195, 140, 117, 61, 44, 25, 1316, 946, 693, 501, 373, 313, 311, 282, 279, 198, 212, 124, 88, 54, 37, 24, 1170, 792, 569, 422, 385, 310, 294, 259, 225, 217, 176, 142, 62, 43, 24, 19, 1030, 699, 513, 366, 323, 268, 285, 230, 226, 173, 150, 109, 75, 34, 20, 26, 806, 570, 426, 306, 262, 218, 226, 202, 203, 175, 162, 102, 67, 33, 17, 14, 676, 460, 348, 252, 235, 225, 228, 205, 216, 172, 127, 85, 60, 28, 15, 11, 586, 391, 293, 230, 257, 171, 221, 166, 190, 160, 133, 75, 51, 18, 7, 13, 512, 377, 236, 219, 215, 180, 202, 178, 153, 147, 127, 60, 51, 16, 5, 7, 450, 300, 221, 197, 184, 183, 199, 180, 145, 126, 107, 76, 55, 20, 2, 4, 393, 278, 227, 207, 175, 178, 178, 147, 156, 118, 98, 69, 47, 17, 4, 2, 382, 263, 213, 178, 196, 154, 171, 139, 164, 99, 86, 59, 40, 10, 3, 3}
};


// adopted from calcScanOrder
void sort(UInt *stats1D, UInt *order1D, int size)
{
	int i, j;
	UInt cStats, cOrder;

	for (i=1; i < size; i++)
	{
		cStats = stats1D[i];
		cOrder = order1D[i];
		j = i;
		while ((j > 0) && (stats1D[j-1] < cStats) )
		{
			stats1D[j] = stats1D[j-1];
			order1D[j] = order1D[j-1];
			j = j - 1;
		}
		stats1D[j] = cStats;
		order1D[j] = cOrder;
	}
}

Bool combineScanMap(UInt *inStats, UInt *outStats, UInt *outScan, UInt width, UInt category)
{
	const UInt threshold = 1;
	UInt height = width;
	UInt size = width*height;
	int i, j, k, kmax, l, lmax;
	UInt order1[4096], stats1[4096];  // for 32x32 and below
	UInt order2[4096], stats2[4096];
	UInt tmpStats[4096], tmpScan[4096];
	Bool orderChanged = false;

	// record positions of both significant and non-significant stats in scanning order
	k = l = 0;
    for(i = 0; i < size; i++)
    {
      if(inStats[i] > threshold)//[i] is just index does not necessarily mean the raster scan
      {
        stats1[k] = inStats[i];
        order1[k++] = i;
      }      
      else
        order2[l++] = i;

      tmpScan[i] = outScan[i]; //i*width+j is just index does not mean the raster scan; tmpScan convert index to raster scan
      tmpStats[i] = outStats[i];
    }

    kmax = k;
	lmax = l;

	//assert(kmax + lmax == size);

	sort(stats1, order1, kmax);

    for(k = 0; k < kmax; k++)
    {
      outScan[k] = tmpScan[order1[k]];
      outStats[k] = tmpStats[order1[k]];
    }

    for(l = 0; l < lmax; l++)
    {
      if(width == 16)
			stats2[l] = FIX_SCANSTATS16x16[category][tmpScan[order2[l]]];
		else if(width == 32)
			stats2[l] = FIX_SCANSTATS32x32[category][tmpScan[order2[l]]];
		else if(width == 64)
			stats2[l] = FIX_SCANSTATS64x64[category][tmpScan[order2[l]]];
		else
			printf("Unsupported block size!\n");
	}

	sort(stats2, order2, lmax);

	j = 0;
	for(l = 0; l < lmax; l++) 
    {
		outScan[kmax+l] = tmpScan[order2[l]];
		outStats[kmax+l] = tmpStats[order2[l]];
	}

	for(i = 0; i < size; i++)
		if(outScan[i] != tmpScan[i]) {
			orderChanged = true;
			break;
		}

	return orderChanged;

}

#endif

static int order1D[4096];

static int  calcScanOrder(UInt *stats, UInt *orderX, UInt *orderY, int size, int width)
{
  int i, j, cOrder, cStats;
  UInt *stats1D = stats; 
  int orderChanged = 0;

  for(i = 0; i < size; i++)
  {
    order1D[i] = orderX[i]+orderY[i]*width;
  }

  for (i=1; i < size; i++)
  {
    cStats = stats1D[i];
    cOrder = order1D[i];
    j = i;
    while ((j > 0) && (stats1D[j-1] < cStats) )
    {
      stats1D[j] = stats1D[j-1];
      order1D[j] = order1D[j-1];
      j = j - 1;
      orderChanged = 1;
    }
    stats1D[j] = cStats;
    order1D[j] = cOrder;
  }

  for(i = 0; i < size; i++)
  {
    orderX[i] = order1D[i]%width;
    orderY[i] = order1D[i]/width;
  }

  return orderChanged;
}


void InitScanOrderForSlice()
{
  int ipredmode;
  UInt stats4x4[9][16];
  UInt stats8x8[9][64];
  UInt stats16x16[NUM_SCANS_16x16][256];
  UInt stats32x32[NUM_SCANS_32x32][1024];
  UInt stats64x64[NUM_SCANS_64x64][4096];


  // 4x4 and 8x8
  for(ipredmode = 0; ipredmode < 9; ipredmode++)
  {
    int k;

    update4x4[ipredmode] = update8x8[ipredmode] = 1;
    for(k = 0; k < 16; k++)
    {
      scanStats4x4[ipredmode][k] = stats4x4[ipredmode][k] = SCANSTATS4x4[ipredmode][k]/2;
    }

    for(k = 0; k < 64; k++)
    {
      scanStats8x8[ipredmode][k] = stats8x8[ipredmode][k] = SCANSTATS8x8[ipredmode][k]/2;
    }
    update4x4Count[ipredmode] = update8x8Count[ipredmode] = 0;
    update4x4Thres[ipredmode] = 4;
    update8x8Thres[ipredmode] = 2;
  }

  for(ipredmode = 0; ipredmode < 9; ipredmode ++)
  {
    int i,dummy;

    {
      for(i = 0; i < 16; i++)
      {
        scanOrder4x4X[ipredmode][i] = i%4;//raster scanning
        scanOrder4x4Y[ipredmode][i] = i/4;
      }
      dummy = calcScanOrder(stats4x4[ipredmode], scanOrder4x4X[ipredmode], scanOrder4x4Y[ipredmode], 16, 4); //re-order scanning order of scanOrder4x4X scanOrder4x4Y stats4x4
      for(i = 0; i < 16; i++)
      {
        scanOrder4x4[ipredmode][i] = scanOrder4x4Y[ipredmode][i] * 4 + scanOrder4x4X[ipredmode][i];
        scanStats4x4[ipredmode][i]    = stats4x4[ipredmode][i];

        //if(scanOrder4x4[ipredmode][i] != g_auiFrameScanXY[0][i])
        //  printf("differ\n");

      }
    }

    {
      for(i = 0; i < 64; i++)
      {
        scanOrder8x8X[ipredmode][i] = i%8;
        scanOrder8x8Y[ipredmode][i] = i/8;
      }
      dummy = calcScanOrder(stats8x8[ipredmode], scanOrder8x8X[ipredmode], scanOrder8x8Y[ipredmode], 64, 8);    
      for(i = 0; i < 64; i++)
      {
        scanOrder8x8[ipredmode][i] = scanOrder8x8Y[ipredmode][i] * 8 + scanOrder8x8X[ipredmode][i];
        scanStats8x8[ipredmode][i]    = stats8x8[ipredmode][i];

        //if(scanOrder8x8[ipredmode][i] != g_auiFrameScanXY[1][i])
        //  printf("differ\n");
      }
    }
  }

  // 16x16
  // assign initialization scanning table
  for (int z=0; z < NUM_SCANS_16x16; z++)
  {
    int k;
    
    for(k = 0; k < 256; k++)
    {
      scanStats16x16[z][k] = stats16x16[z][k] = SCANSTATS16x16[z][k];
    }

  }

    // 16x16
  for (int z=0; z < NUM_SCANS_16x16; z++)
  {
	  int i;

#ifdef COMBINED_MAP
	  for(i = 0; i < 256; i++)
		scanOrder16x16[z][i] = i;//raster scan

	// keep in mind that scanOrder and scanStat need to be synchronized
	  combineScanMap(FIX_SCANSTATS16x16[z], scanStats16x16[z], scanOrder16x16[z], 16, z);  // use good scan for initlaization

	  for (i=0; i < 256; i++)
	  {
		  scanOrder16x16X[z][i] = scanOrder16x16[z][i] % 16;
		  scanOrder16x16Y[z][i] = scanOrder16x16[z][i] / 16;
	  }
#else
	  int dummy;

      for (i=0; i < 256; i++)
      {
        scanOrder16x16X[z][i] = i%16; //raster scanning
        scanOrder16x16Y[z][i] = i/16;
      }

      dummy = calcScanOrder(stats16x16[z], scanOrder16x16X[z], scanOrder16x16Y[z], 256, 16);
            //re-order scanning order of scanOrder16x16X, scanOrder16x16Y stats16x16
      for (i = 0; i < 256; i++)
      {
        scanOrder16x16[z][i] = scanOrder16x16Y[z][i] * 16 + scanOrder16x16X[z][i];
        scanStats16x16[z][i] = stats16x16[z][i];
      }
#endif
  }

  // 32x32
  // assign initialization scanning table
  for (int z=0; z < NUM_SCANS_32x32; z++)
  {
    int k;

    for(k = 0; k < 1024; k++)
    {
      scanStats32x32[z][k] = stats32x32[z][k] = SCANSTATS32x32[z][k];
    }
  }

    // 32x32
  for (int z=0; z < NUM_SCANS_32x32; z++)
  {
    int i;

#ifdef COMBINED_MAP
	  for(i = 0; i < 1024; i++)
		scanOrder32x32[z][i] = i;

	  combineScanMap(FIX_SCANSTATS32x32[z], scanStats32x32[z], scanOrder32x32[z], 32, z);  // use good scan for initialization
	  for(i = 0; i < 1024; i++)
	  {
		scanOrder32x32X[z][i] = scanOrder32x32[z][i] % 32;
		scanOrder32x32Y[z][i] = scanOrder32x32[z][i] / 32;
	  }
#else
	  int dummy;

      for (i=0; i < 1024; i++)
      {
        scanOrder32x32X[z][i] = i%32; //raster scanning
        scanOrder32x32Y[z][i] = i/32;
      }

      dummy = calcScanOrder(stats32x32[z], scanOrder32x32X[z], scanOrder32x32Y[z], 1024, 32);
            //re-order scanning order of scanOrder32x32X, scanOrder32x32Y stats32x32
      for (i = 0; i < 1024; i++)
      {
        scanOrder32x32[z][i] = scanOrder32x32Y[z][i] * 32 + scanOrder32x32X[z][i];
        scanStats32x32[z][i] = stats32x32[z][i];
      }
#endif

  }

  // 64x64
  for (int z=0; z < NUM_SCANS_64x64; z++)
  {
    int k;

    for(k = 0; k < 4096; k++)
    {
      scanStats64x64[z][k] = stats64x64[z][k] = SCANSTATS64x64[z][k];
    }

  }

  // 64x64
  for (int z=0; z < NUM_SCANS_64x64; z++)
  {
    int i;

#ifdef COMBINED_MAP
	  for(i = 0; i < 4096; i++)
		scanOrder64x64[z][i] = i;

	  combineScanMap(FIX_SCANSTATS64x64[z], scanStats64x64[z], scanOrder64x64[z], 64, z);  // use good scan for initialization

	  for(i = 0; i < 4096; i++)
	  {
		scanOrder64x64X[z][i] = scanOrder64x64[z][i] % 64;
		scanOrder64x64Y[z][i] = scanOrder64x64[z][i] / 64;
	  }
#else
      for (i=0; i < 4096; i++)
      {
        scanOrder64x64X[z][i] = i%64; //raster scanning
        scanOrder64x64Y[z][i] = i/64;
      }

      dummy = calcScanOrder(stats64x64[z], scanOrder64x64X[z], scanOrder64x64Y[z], 4096, 64);
      //re-order scanning order of scanOrder64x64X, scanOrder64x64Y stats64x64
      for (i = 0; i < 4096; i++)
      {
        scanOrder64x64[z][i] = scanOrder64x64Y[z][i] * 64 + scanOrder64x64X[z][i];
        scanStats64x64[z][i] = stats64x64[z][i];
      }
#endif
  }
}

void updateScanOrder(int first)
{
  int ipredmode;

  int orderChanged;
  UInt stats16x16[NUM_SCANS_16x16][256];
  UInt stats32x32[NUM_SCANS_32x32][1024];

  UInt stats64x64[NUM_SCANS_64x64][4096];


  for(ipredmode = 0; ipredmode < 9; ipredmode++)
  {
    if(update4x4Count[ipredmode] >= update4x4Thres[ipredmode])
    {
      update4x4[ipredmode] = 1;
      update4x4Count[ipredmode] = 0;
    }
    else update4x4[ipredmode] = 0;

    if(update8x8Count[ipredmode] >= update8x8Thres[ipredmode])
    {
      update8x8[ipredmode] = 1;
      update8x8Count[ipredmode] = 0;
    }
    else update8x8[ipredmode] = 0;
  }

  for(ipredmode = 0; ipredmode < 9; ipredmode ++)
  {
    int i;

    if(update4x4[ipredmode])
    {
      for(i = 0; i < 16; i++)
      {
        scanOrder4x4X[ipredmode][i] = scanOrder4x4[ipredmode][i]%4;
        scanOrder4x4Y[ipredmode][i] = scanOrder4x4[ipredmode][i]/4;
      }

      orderChanged = calcScanOrder(scanStats4x4[ipredmode], scanOrder4x4X[ipredmode], scanOrder4x4Y[ipredmode], 16, 4);
      if(!orderChanged)
      {
        update4x4Thres[ipredmode] <<= 1;
      }
      else if(update4x4Thres[ipredmode] > 4) 
        update4x4Thres[ipredmode] >>= 1;

      if(orderChanged)
      {
        for(i = 0; i < 16; i++)        
        {           
          scanOrder4x4[ipredmode][i] = scanOrder4x4Y[ipredmode][i] * 4 + scanOrder4x4X[ipredmode][i];
        }
      }
    }



    if(update8x8[ipredmode])
    {
      for(i = 0; i < 64; i++)
      {
        scanOrder8x8X[ipredmode][i] = scanOrder8x8[ipredmode][i]%8;
        scanOrder8x8Y[ipredmode][i] = scanOrder8x8[ipredmode][i]/8;
      }
      orderChanged = calcScanOrder(scanStats8x8[ipredmode], scanOrder8x8X[ipredmode], scanOrder8x8Y[ipredmode], 64, 8);  
      if(!orderChanged)
      {
        update8x8Thres[ipredmode] <<= 1;
      }
      else if(update8x8Thres[ipredmode] > 2) 
        update8x8Thres[ipredmode] >>= 1;
      if(orderChanged)
      {
        for(i = 0; i < 64; i++)
        {
          scanOrder8x8[ipredmode][i] = scanOrder8x8Y[ipredmode][i] * 8 + scanOrder8x8X[ipredmode][i];
        }
      }
    }
  }
  // 16x16
  for (int z=0; z < NUM_SCANS_16x16; z++)
  {
    int i;

#ifdef COMBINED_MAP
	for(i = 0; i < 256; i++)
		stats16x16[z][i] = scanStats16x16[z][i];

	orderChanged = combineScanMap(stats16x16[z], scanStats16x16[z], scanOrder16x16[z], 16, z);  // use combined scan for update

	for(i = 0; i < 256; i++)
    {
      scanOrder16x16X[z][i] = scanOrder16x16[z][i]%16;
      scanOrder16x16Y[z][i] = scanOrder16x16[z][i]/16;
    }
#else

    for(i = 0; i < 256; i++)
    {
      scanOrder16x16X[z][i] = scanOrder16x16[z][i]%16;
      scanOrder16x16Y[z][i] = scanOrder16x16[z][i]/16;
    }

    orderChanged = calcScanOrder(scanStats16x16[z], scanOrder16x16X[z], scanOrder16x16Y[z], 256, 16);

    if( orderChanged)
    {
      for(i = 0; i < 256; i++)        
      {           
        scanOrder16x16[z][i] = scanOrder16x16Y[z][i] * 16 + scanOrder16x16X[z][i];
      }
    }

#endif

  }

  // 32x32
  for (int z=0; z < NUM_SCANS_32x32; z++)
  {
    int i;

#ifdef COMBINED_MAP
	for(i = 0; i < 1024; i++)
		stats32x32[z][i] = scanStats32x32[z][i];

	combineScanMap(stats32x32[z], scanStats32x32[z], scanOrder32x32[z], 32, z);	// use mixed scan for update

    for(i = 0; i < 1024; i++)
    {
      scanOrder32x32X[z][i] = scanOrder32x32[z][i]%32;
      scanOrder32x32Y[z][i] = scanOrder32x32[z][i]/32;
    }
#else

    for(i = 0; i < 1024; i++)
    {
      scanOrder32x32X[z][i] = scanOrder32x32[z][i]%32;
      scanOrder32x32Y[z][i] = scanOrder32x32[z][i]/32;
    }

    orderChanged = calcScanOrder(scanStats32x32[z], scanOrder32x32X[z], scanOrder32x32Y[z], 1024, 32);

    if (orderChanged)
    {
      for(i = 0; i < 1024; i++)        
      {           
        scanOrder32x32[z][i] = scanOrder32x32Y[z][i] * 32 + scanOrder32x32X[z][i];
      }
    }

#endif

  }

  for (int z=0; z < NUM_SCANS_64x64; z++)
  {
    int i;

#ifdef COMBINED_MAP
	for(i = 0; i < 4096; i++)
		stats64x64[z][i] = scanStats64x64[z][i];

	orderChanged = combineScanMap(stats64x64[z], scanStats64x64[z], scanOrder64x64[z], 64, z);  // use combined scan for update

	for(i = 0; i < 4096; i++)
    {
      scanOrder64x64X[z][i] = scanOrder64x64[z][i]%64;
      scanOrder64x64Y[z][i] = scanOrder64x64[z][i]/64;
    }
#else

    for(i = 0; i < 4096; i++)
    {
      scanOrder64x64X[z][i] = scanOrder64x64[z][i]%64;
      scanOrder64x64Y[z][i] = scanOrder64x64[z][i]/64;
    }

    orderChanged = calcScanOrder(scanStats64x64[z], scanOrder64x64X[z], scanOrder64x64Y[z], 4096, 64);

    if (orderChanged)
    {
      for(i = 0; i < 4096; i++)        
      {           
        scanOrder64x64[z][i] = scanOrder64x64Y[z][i] * 64 + scanOrder64x64X[z][i];
      }
    }
#endif
  }


  //for(ipredmode = 0; ipredmode < 9; ipredmode ++)
  //{
  //  int i;
  //  printf("4x4 ipredmode = %d\n", ipredmode);
  //  for(i = 0; i < 16; i++)        
  //  {           
  //    printf("%d ", scanOrder4x4[ipredmode][i]);
  //  }
  //  printf("\n");


  //  printf("8x8 ipredmode = %d\n", ipredmode);
  //  for(i = 0; i < 64; i++)
  //  {
  //    printf("%d ", scanOrder8x8[ipredmode][i]);
  //  }
  //  printf("\n");

  //}
  //for (int ipredmode=0; ipredmode < NUM_SCANS_16x16; ipredmode++)
  //{
  //  int i;
  //  printf("16x16 ipredmode = %d\n", ipredmode);
  //  for(i = 0; i < 256; i++)
  //  {
  //    printf("%d ", scanOrder16x16[ipredmode][i]);
  //  }
  //  printf("\n");
  //}
}

static void scaleScanStats(UInt *stats, int size)
{
  int i;

  for(i = 0; i < size*size; i++)
    stats[i] >>= 1;
}

void normalizeScanStats()
{
  int ipredmode;

    for(ipredmode = 0; ipredmode < 9; ipredmode++)
    {
      if(scanStats4x4[ipredmode][0] >= 256)
      {
		  count4x4[ipredmode]++;
        scaleScanStats(scanStats4x4[ipredmode], 4);
      }
    }

    for(ipredmode = 0; ipredmode < 9; ipredmode++)
    {
      if(scanStats8x8[ipredmode][0] >= 256)
      {
		  count8x8[ipredmode]++;
        scaleScanStats(scanStats8x8[ipredmode], 8);
      }
    }

  // 16x16
  for (int z=0; z < NUM_SCANS_16x16; z++)
  {
    if (scanStats16x16[z][0] >= 256)
    {
		count16x16[z]++;
      scaleScanStats(scanStats16x16[z], 16);
    }
  }

  // 32x32
  for (int z=0; z < NUM_SCANS_32x32; z++)
  {
    if (scanStats32x32[z][0] >= 256)
    {
	  count32x32[z]++;
      scaleScanStats(scanStats32x32[z], 32);
    }
  }

  // 64x64
  for (int z=0; z < NUM_SCANS_64x64; z++)
  {
    if (scanStats64x64[z][0] >= 256)
    {
      scaleScanStats(scanStats64x64[z], 64);
    }
  }

}
#endif
>>>>>>> upstream/master
