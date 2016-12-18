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

/** \file     TComPattern.cpp
    \brief    neighbouring pixel access classes
*/

#include "TComPic.h"
#include "TComPattern.h"
#include "TComDataCU.h"
#include "TComTU.h"
#include "Debug.h"
#include "TComPrediction.h"

//! \ingroup TLibCommon
//! \{

// Forward declarations

/// padding of unavailable reference samples for intra prediction
Void fillReferenceSamples( const Int bitDepth, 
#if O0043_BEST_EFFORT_DECODING
                           const Int bitDepthDelta, 
#endif
                           const Pel* piRoiOrigin, 
                                 Pel* piIntraTemp,
                           const Bool* bNeighborFlags,
                           const Int iNumIntraNeighbor, 
                           const Int unitWidth, 
                           const Int unitHeight, 
                           const Int iAboveUnits, 
                           const Int iLeftUnits,
                           const UInt uiWidth, 
                           const UInt uiHeight, 
                           const Int iPicStride );

/// constrained intra prediction
Bool  isAboveLeftAvailable  ( TComDataCU* pcCU, UInt uiPartIdxLT );
Int   isAboveAvailable      ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool* bValidFlags );
Int   isLeftAvailable       ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool* bValidFlags );
Int   isAboveRightAvailable ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool* bValidFlags );
Int   isBelowLeftAvailable  ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool* bValidFlags );


// ====================================================================================================================
// Public member functions (TComPatternParam)
// ====================================================================================================================

/** 
 \param  piTexture     pixel data
 \param  iRoiWidth     pattern width
 \param  iRoiHeight    pattern height
 \param  iStride       buffer stride
 \param  bitDepth      bit depth
 */
Void TComPatternParam::setPatternParamPel ( Pel* piTexture,
                                           Int iRoiWidth,
                                           Int iRoiHeight,
                                           Int iStride,
                                           Int bitDepth
                                           )
{
  m_piROIOrigin    = piTexture;
  m_iROIWidth      = iRoiWidth;
  m_iROIHeight     = iRoiHeight;
  m_iPatternStride = iStride;
  m_bitDepth       = bitDepth;
}

// ====================================================================================================================
// Public member functions (TComPattern)
// ====================================================================================================================

Void TComPattern::initPattern (Pel* piY,
                               Int iRoiWidth,
                               Int iRoiHeight,
                               Int iStride,
                               Int bitDepthLuma)
{
  m_cPatternY. setPatternParamPel( piY,  iRoiWidth, iRoiHeight, iStride, bitDepthLuma);
}


// TODO: move this function to TComPrediction.cpp.
Void TComPrediction::initIntraPatternChType( TComTU &rTu, Bool& bAbove, Bool& bLeft, const ComponentID compID, const Bool bFilterRefSamples DEBUG_STRING_FN_DECLARE(sDebug))
{
  const ChannelType chType    = toChannelType(compID);

  TComDataCU *pcCU=rTu.getCU();
  const TComSPS &sps = *(pcCU->getSlice()->getSPS());
  const UInt uiZorderIdxInPart=rTu.GetAbsPartIdxTU();
  const UInt uiTuWidth        = rTu.getRect(compID).width;
  const UInt uiTuHeight       = rTu.getRect(compID).height;
  const UInt uiTuWidth2       = uiTuWidth  << 1;
  const UInt uiTuHeight2      = uiTuHeight << 1;

  const Int  iBaseUnitSize    = sps.getMaxCUWidth() >> sps.getMaxTotalCUDepth();
  const Int  iUnitWidth       = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(compID);
  const Int  iUnitHeight      = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(compID);
  const Int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const Int  iAboveUnits      = iTUWidthInUnits  << 1;
  const Int  iLeftUnits       = iTUHeightInUnits << 1;
  const Int  bitDepthForChannel = sps.getBitDepth(chType);

  assert(iTUHeightInUnits > 0 && iTUWidthInUnits > 0);

  const Int  iPartIdxStride   = pcCU->getPic()->getNumPartInCtuWidth();
  const UInt uiPartIdxLT      = pcCU->getZorderIdxInCtu() + uiZorderIdxInPart;
  const UInt uiPartIdxRT      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] +   iTUWidthInUnits  - 1                   ];
  const UInt uiPartIdxLB      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] + ((iTUHeightInUnits - 1) * iPartIdxStride)];

  Int   iPicStride = pcCU->getPic()->getStride(compID);
  Bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  Int   iNumIntraNeighbor = 0;

  bNeighborFlags[iLeftUnits] = isAboveLeftAvailable( pcCU, uiPartIdxLT );
  iNumIntraNeighbor += bNeighborFlags[iLeftUnits] ? 1 : 0;
  iNumIntraNeighbor  += isAboveAvailable     ( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1)                    );
  iNumIntraNeighbor  += isAboveRightAvailable( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1 + iTUWidthInUnits ) );
  iNumIntraNeighbor  += isLeftAvailable      ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1)                    );
  iNumIntraNeighbor  += isBelowLeftAvailable ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1 - iTUHeightInUnits) );

  bAbove = true;
  bLeft  = true;

  const UInt         uiROIWidth  = uiTuWidth2+1;
  const UInt         uiROIHeight = uiTuHeight2+1;

  assert(uiROIWidth*uiROIHeight <= m_iYuvExtSize);

#if DEBUG_STRING
  std::stringstream ss(stringstream::out);
#endif

  {
    Pel *piIntraTemp   = m_piYuvExt[compID][PRED_BUF_UNFILTERED];
    Pel *piRoiOrigin = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiZorderIdxInPart);
#if O0043_BEST_EFFORT_DECODING
    const Int  bitDepthForChannelInStream = sps.getStreamBitDepth(chType);
    fillReferenceSamples (bitDepthForChannelInStream, bitDepthForChannelInStream - bitDepthForChannel,
#else
    fillReferenceSamples (bitDepthForChannel,
#endif
                          piRoiOrigin, piIntraTemp, bNeighborFlags, iNumIntraNeighbor,  iUnitWidth, iUnitHeight, iAboveUnits, iLeftUnits,
                          uiROIWidth, uiROIHeight, iPicStride);


#if DEBUG_STRING
    if (DebugOptionList::DebugString_Pred.getInt()&DebugStringGetPredModeMask(MODE_INTRA))
    {
      ss << "###: generating Ref Samples for channel " << compID << " and " << rTu.getRect(compID).width << " x " << rTu.getRect(compID).height << "\n";
      for (UInt y=0; y<uiROIHeight; y++)
      {
        ss << "###: - ";
        for (UInt x=0; x<uiROIWidth; x++)
        {
          if (x==0 || y==0)
          {
            ss << piIntraTemp[y*uiROIWidth + x] << ", ";
//          if (x%16==15) ss << "\nPart size: ~ ";
          }
        }
        ss << "\n";
      }
    }
#endif

    if (bFilterRefSamples)
    {
      // generate filtered intra prediction samples

            Int          stride    = uiROIWidth;
      const Pel         *piSrcPtr  = piIntraTemp                           + (stride * uiTuHeight2); // bottom left
            Pel         *piDestPtr = m_piYuvExt[compID][PRED_BUF_FILTERED] + (stride * uiTuHeight2); // bottom left

      //------------------------------------------------

      Bool useStrongIntraSmoothing = isLuma(chType) && sps.getUseStrongIntraSmoothing();

      const Pel bottomLeft = piIntraTemp[stride * uiTuHeight2];
      const Pel topLeft    = piIntraTemp[0];
      const Pel topRight   = piIntraTemp[uiTuWidth2];

      if (useStrongIntraSmoothing)
      {
#if O0043_BEST_EFFORT_DECODING
        const Int  threshold     = 1 << (bitDepthForChannelInStream - 5);
#else
        const Int  threshold     = 1 << (bitDepthForChannel - 5);
#endif
        const Bool bilinearLeft  = abs((bottomLeft + topLeft ) - (2 * piIntraTemp[stride * uiTuHeight])) < threshold; //difference between the
        const Bool bilinearAbove = abs((topLeft    + topRight) - (2 * piIntraTemp[         uiTuWidth ])) < threshold; //ends and the middle
        if ((uiTuWidth < 32) || (!bilinearLeft) || (!bilinearAbove))
        {
          useStrongIntraSmoothing = false;
        }
      }

      *piDestPtr = *piSrcPtr; // bottom left is not filtered
      piDestPtr -= stride;
      piSrcPtr  -= stride;

      //------------------------------------------------

      //left column (bottom to top)

      if (useStrongIntraSmoothing)
      {
        const Int shift = g_aucConvertToBit[uiTuHeight] + 3; //log2(uiTuHeight2)

        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride)
        {
          *piDestPtr = (((uiTuHeight2 - i) * bottomLeft) + (i * topLeft) + uiTuHeight) >> shift;
        }

        piSrcPtr -= stride * (uiTuHeight2 - 1);
      }
      else
      {
        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride, piSrcPtr-=stride)
        {
          *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[-stride] + 2 ) >> 2;
        }
      }

      //------------------------------------------------

      //top-left

      if (useStrongIntraSmoothing)
      {
        *piDestPtr = piSrcPtr[0];
      }
      else
      {
        *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[1] + 2 ) >> 2;
      }
      piDestPtr += 1;
      piSrcPtr  += 1;

      //------------------------------------------------

      //top row (left-to-right)

      if (useStrongIntraSmoothing)
      {
        const Int shift = g_aucConvertToBit[uiTuWidth] + 3; //log2(uiTuWidth2)

        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++)
        {
          *piDestPtr = (((uiTuWidth2 - i) * topLeft) + (i * topRight) + uiTuWidth) >> shift;
        }

        piSrcPtr += uiTuWidth2 - 1;
      }
      else
      {
        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++, piSrcPtr++)
        {
          *piDestPtr = ( piSrcPtr[1] + 2*piSrcPtr[0] + piSrcPtr[-1] + 2 ) >> 2;
        }
      }

      //------------------------------------------------

      *piDestPtr=*piSrcPtr; // far right is not filtered

#if DEBUG_STRING
    if (DebugOptionList::DebugString_Pred.getInt()&DebugStringGetPredModeMask(MODE_INTRA))
    {
      ss << "###: filtered result for channel " << compID <<"\n";
      for (UInt y=0; y<uiROIHeight; y++)
      {
        ss << "###: - ";
        for (UInt x=0; x<uiROIWidth; x++)
        {
          if (x==0 || y==0)
          {
            ss << m_piYuvExt[compID][PRED_BUF_FILTERED][y*uiROIWidth + x] << ", ";
//          if (x%16==15) ss << "\nPart size: ~ ";
          }
        }
        ss << "\n";
      }
    }
#endif


    }
  }
  DEBUG_STRING_APPEND(sDebug, ss.str())
}

Void fillReferenceSamples( const Int bitDepth, 
#if O0043_BEST_EFFORT_DECODING
                           const Int bitDepthDelta, 
#endif
                           const Pel* piRoiOrigin, 
                                 Pel* piIntraTemp,
                           const Bool* bNeighborFlags,
                           const Int iNumIntraNeighbor, 
                           const Int unitWidth, 
                           const Int unitHeight, 
                           const Int iAboveUnits, 
                           const Int iLeftUnits,
                           const UInt uiWidth, 
                           const UInt uiHeight, 
                           const Int iPicStride )
{
  const Pel* piRoiTemp;
  Int  i, j;
  Int  iDCValue = 1 << (bitDepth - 1);
  const Int iTotalUnits = iAboveUnits + iLeftUnits + 1; //+1 for top-left

  if (iNumIntraNeighbor == 0)
  {
    // Fill border with DC value
    for (i=0; i<uiWidth; i++)
    {
      piIntraTemp[i] = iDCValue;
    }
    for (i=1; i<uiHeight; i++)
    {
      piIntraTemp[i*uiWidth] = iDCValue;
    }
  }
  else if (iNumIntraNeighbor == iTotalUnits)
  {
    // Fill top-left border and top and top right with rec. samples
    piRoiTemp = piRoiOrigin - iPicStride - 1;

    for (i=0; i<uiWidth; i++)
    {
#if O0043_BEST_EFFORT_DECODING
      piIntraTemp[i] = piRoiTemp[i] << bitDepthDelta;
#else
      piIntraTemp[i] = piRoiTemp[i];
#endif
    }

    // Fill left and below left border with rec. samples
    piRoiTemp = piRoiOrigin - 1;

    for (i=1; i<uiHeight; i++)
    {
#if O0043_BEST_EFFORT_DECODING
      piIntraTemp[i*uiWidth] = (*(piRoiTemp)) << bitDepthDelta;
#else
      piIntraTemp[i*uiWidth] = *(piRoiTemp);
#endif
      piRoiTemp += iPicStride;
    }
  }
  else // reference samples are partially available
  {
    // all above units have "unitWidth" samples each, all left/below-left units have "unitHeight" samples each
    const Int  iTotalSamples = (iLeftUnits * unitHeight) + ((iAboveUnits + 1) * unitWidth);
    Pel  piIntraLine[5 * MAX_CU_SIZE];
    Pel  *piIntraLineTemp;
    const Bool *pbNeighborFlags;


    // Initialize
    for (i=0; i<iTotalSamples; i++)
    {
      piIntraLine[i] = iDCValue;
    }

    // Fill top-left sample
    piRoiTemp = piRoiOrigin - iPicStride - 1;
    piIntraLineTemp = piIntraLine + (iLeftUnits * unitHeight);
    pbNeighborFlags = bNeighborFlags + iLeftUnits;
    if (*pbNeighborFlags)
    {
#if O0043_BEST_EFFORT_DECODING
      Pel topLeftVal=piRoiTemp[0] << bitDepthDelta;
#else
      Pel topLeftVal=piRoiTemp[0];
#endif
      for (i=0; i<unitWidth; i++)
      {
        piIntraLineTemp[i] = topLeftVal;
      }
    }

    // Fill left & below-left samples (downwards)
    piRoiTemp += iPicStride;
    piIntraLineTemp--;
    pbNeighborFlags--;

    for (j=0; j<iLeftUnits; j++)
    {
      if (*pbNeighborFlags)
      {
        for (i=0; i<unitHeight; i++)
        {
#if O0043_BEST_EFFORT_DECODING
          piIntraLineTemp[-i] = piRoiTemp[i*iPicStride] << bitDepthDelta;
#else
          piIntraLineTemp[-i] = piRoiTemp[i*iPicStride];
#endif
        }
      }
      piRoiTemp += unitHeight*iPicStride;
      piIntraLineTemp -= unitHeight;
      pbNeighborFlags--;
    }

    // Fill above & above-right samples (left-to-right) (each unit has "unitWidth" samples)
    piRoiTemp = piRoiOrigin - iPicStride;
    // offset line buffer by iNumUints2*unitHeight (for left/below-left) + unitWidth (for above-left)
    piIntraLineTemp = piIntraLine + (iLeftUnits * unitHeight) + unitWidth;
    pbNeighborFlags = bNeighborFlags + iLeftUnits + 1;
    for (j=0; j<iAboveUnits; j++)
    {
      if (*pbNeighborFlags)
      {
        for (i=0; i<unitWidth; i++)
        {
#if O0043_BEST_EFFORT_DECODING
          piIntraLineTemp[i] = piRoiTemp[i] << bitDepthDelta;
#else
          piIntraLineTemp[i] = piRoiTemp[i];
#endif
        }
      }
      piRoiTemp += unitWidth;
      piIntraLineTemp += unitWidth;
      pbNeighborFlags++;
    }

    // Pad reference samples when necessary
    Int iCurrJnit = 0;
    Pel  *piIntraLineCur   = piIntraLine;
    const UInt piIntraLineTopRowOffset = iLeftUnits * (unitHeight - unitWidth);

    if (!bNeighborFlags[0])
    {
      // very bottom unit of bottom-left; at least one unit will be valid.
      {
        Int   iNext = 1;
        while (iNext < iTotalUnits && !bNeighborFlags[iNext])
        {
          iNext++;
        }
        Pel *piIntraLineNext = piIntraLine + ((iNext < iLeftUnits) ? (iNext * unitHeight) : (piIntraLineTopRowOffset + (iNext * unitWidth)));
        const Pel refSample = *piIntraLineNext;
        // Pad unavailable samples with new value
        Int iNextOrTop = std::min<Int>(iNext, iLeftUnits);
        // fill left column
        while (iCurrJnit < iNextOrTop)
        {
          for (i=0; i<unitHeight; i++)
          {
            piIntraLineCur[i] = refSample;
          }
          piIntraLineCur += unitHeight;
          iCurrJnit++;
        }
        // fill top row
        while (iCurrJnit < iNext)
        {
          for (i=0; i<unitWidth; i++)
          {
            piIntraLineCur[i] = refSample;
          }
          piIntraLineCur += unitWidth;
          iCurrJnit++;
        }
      }
    }

    // pad all other reference samples.
    while (iCurrJnit < iTotalUnits)
    {
      if (!bNeighborFlags[iCurrJnit]) // samples not available
      {
        {
          const Int numSamplesInCurrUnit = (iCurrJnit >= iLeftUnits) ? unitWidth : unitHeight;
          const Pel refSample = *(piIntraLineCur-1);
          for (i=0; i<numSamplesInCurrUnit; i++)
          {
            piIntraLineCur[i] = refSample;
          }
          piIntraLineCur += numSamplesInCurrUnit;
          iCurrJnit++;
        }
      }
      else
      {
        piIntraLineCur += (iCurrJnit >= iLeftUnits) ? unitWidth : unitHeight;
        iCurrJnit++;
      }
    }

    // Copy processed samples

    piIntraLineTemp = piIntraLine + uiHeight + unitWidth - 2;
    // top left, top and top right samples
    for (i=0; i<uiWidth; i++)
    {
      piIntraTemp[i] = piIntraLineTemp[i];
    }

    piIntraLineTemp = piIntraLine + uiHeight - 1;
    for (i=1; i<uiHeight; i++)
    {
      piIntraTemp[i*uiWidth] = piIntraLineTemp[-i];
    }
  }
}

Bool TComPrediction::filteringIntraReferenceSamples(const ComponentID compID, UInt uiDirMode, UInt uiTuChWidth, UInt uiTuChHeight, const ChromaFormat chFmt, const Bool intraReferenceSmoothingDisabled)
{
  Bool bFilter;

  if (!filterIntraReferenceSamples(toChannelType(compID), chFmt, intraReferenceSmoothingDisabled))
  {
    bFilter=false;
  }
  else
  {
    assert(uiTuChWidth>=4 && uiTuChHeight>=4 && uiTuChWidth<128 && uiTuChHeight<128);

    if (uiDirMode == DC_IDX)
    {
      bFilter=false; //no smoothing for DC or LM chroma
    }
    else
    {
      Int diff = min<Int>(abs((Int) uiDirMode - HOR_IDX), abs((Int)uiDirMode - VER_IDX));
      UInt sizeIndex=g_aucConvertToBit[uiTuChWidth];
      assert(sizeIndex < MAX_INTRA_FILTER_DEPTHS);
      bFilter = diff > m_aucIntraFilter[toChannelType(compID)][sizeIndex];
    }
  }
  return bFilter;
}

Bool isAboveLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT )
{
  Bool bAboveLeftFlag;
  UInt uiPartAboveLeft;
  TComDataCU* pcCUAboveLeft = pcCU->getPUAboveLeft( uiPartAboveLeft, uiPartIdxLT );
  if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
  {
    bAboveLeftFlag = ( pcCUAboveLeft && pcCUAboveLeft->isIntra( uiPartAboveLeft ) );
  }
  else
  {
    bAboveLeftFlag = (pcCUAboveLeft ? true : false);
  }
  return bAboveLeftFlag;
}

Int isAboveAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool *bValidFlags )
{
  const UInt uiRasterPartBegin = g_auiZscanToRaster[uiPartIdxLT];
  const UInt uiRasterPartEnd = g_auiZscanToRaster[uiPartIdxRT]+1;
  const UInt uiIdxStep = 1;
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiRasterPart = uiRasterPartBegin; uiRasterPart < uiRasterPartEnd; uiRasterPart += uiIdxStep )
  {
    UInt uiPartAbove;
    TComDataCU* pcCUAbove = pcCU->getPUAbove( uiPartAbove, g_auiRasterToZscan[uiRasterPart] );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUAbove && pcCUAbove->isIntra( uiPartAbove ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if (pcCUAbove)
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags++;
  }
  return iNumIntra;
}

Int isLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool *bValidFlags )
{
  const UInt uiRasterPartBegin = g_auiZscanToRaster[uiPartIdxLT];
  const UInt uiRasterPartEnd = g_auiZscanToRaster[uiPartIdxLB]+1;
  const UInt uiIdxStep = pcCU->getPic()->getNumPartInCtuWidth();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiRasterPart = uiRasterPartBegin; uiRasterPart < uiRasterPartEnd; uiRasterPart += uiIdxStep )
  {
    UInt uiPartLeft;
    TComDataCU* pcCULeft = pcCU->getPULeft( uiPartLeft, g_auiRasterToZscan[uiRasterPart] );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCULeft && pcCULeft->isIntra( uiPartLeft ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCULeft )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}

Int isAboveRightAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool *bValidFlags )
{
  const UInt uiNumUnitsInPU = g_auiZscanToRaster[uiPartIdxRT] - g_auiZscanToRaster[uiPartIdxLT] + 1;
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiOffset = 1; uiOffset <= uiNumUnitsInPU; uiOffset++ )
  {
    UInt uiPartAboveRight;
    TComDataCU* pcCUAboveRight = pcCU->getPUAboveRight( uiPartAboveRight, uiPartIdxRT, uiOffset );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUAboveRight && pcCUAboveRight->isIntra( uiPartAboveRight ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCUAboveRight )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags++;
  }

  return iNumIntra;
}

Int isBelowLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool *bValidFlags )
{
  const UInt uiNumUnitsInPU = (g_auiZscanToRaster[uiPartIdxLB] - g_auiZscanToRaster[uiPartIdxLT]) / pcCU->getPic()->getNumPartInCtuWidth() + 1;
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiOffset = 1; uiOffset <= uiNumUnitsInPU; uiOffset++ )
  {
    UInt uiPartBelowLeft;
    TComDataCU* pcCUBelowLeft = pcCU->getPUBelowLeft( uiPartBelowLeft, uiPartIdxLB, uiOffset );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUBelowLeft && pcCUBelowLeft->isIntra( uiPartBelowLeft ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCUBelowLeft )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}
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

/** \file     TComPattern.cpp
    \brief    neighbouring pixel access classes
*/

#include "TComPic.h"
#include "TComPattern.h"
#include "TComDataCU.h"

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar g_aaucAvailableBlkMask[16][8] =
{
  // 4x4 block neighbor availability      // MB neighbor availability
  {0x0,0x0,0x0,0x0,  0x0,0x8,0x0,0x08  }, // L, A, AL, AR   <== WTF (blkIdx < 8)
  {0x1,0x0,0x0,0x0,  0x1,0x8,0x0,0x08  }, //    A, AL, AR
  {0xA,0xE,0xE,0x6,  0x0,0x8,0x0,0x08  }, // L,    AL, AR
  {0xB,0xE,0xE,0x6,  0x1,0x8,0x0,0x08  }, //       AL, AR

  {0x4,0x0,0x0,0x0,  0x0,0x8,0x0,0x08  }, // L, A,     AR
  {0x5,0x0,0x0,0x0,  0x1,0x8,0x0,0x08  }, //    A,     AR
  {0xE,0xE,0xE,0x6,  0x0,0x8,0x0,0x08  }, // L,        AR
  {0xF,0xE,0xE,0x6,  0x1,0x8,0x0,0x08  }, //           AR

  {0x0,0x0,0x0,0x8,  0x0,0x8,0x0,0x08  }, // L, A, AL       <== WTF (blkIdx < 8 || blkIdx >= 8)
  {0x1,0x0,0x0,0x8,  0x1,0x8,0x0,0x08  }, //    A, AL
  {0xA,0xE,0xE,0xE,  0x0,0x8,0x0,0x08  }, // L,    AL
  {0xB,0xE,0xE,0xE,  0x1,0x8,0x0,0x08  }, //       AL

  {0x4,0x0,0x0,0x8,  0x0,0x8,0x0,0x08  }, // L, A,
  {0x5,0x0,0x0,0x8,  0x1,0x8,0x0,0x08  }, //    A,
  {0xE,0xE,0xE,0xE,  0x0,0x8,0x0,0x08  }, // L,
  {0xF,0xE,0xE,0xE,  0x1,0x8,0x0,0x08  }  //
};

// ====================================================================================================================
// Public member functions (TComPatternParam)
// ====================================================================================================================

/** \param  piTexture     pixel data
    \param  iRoiWidth     pattern width
    \param  iRoiHeight    pattern height
    \param  iStride       buffer stride
    \param  iOffsetLeft   neighbour offset (left)
    \param  iOffsetRight  neighbour offset (right)
    \param  iOffsetAbove  neighbour offset (above)
    \param  iOffsetBottom neighbour offset (bottom)
 */
Void TComPatternParam::setPatternParamPel ( Pel* piTexture,
                                            Int iRoiWidth,
                                            Int iRoiHeight,
                                            Int iStride,
                                            Int iOffsetLeft,
                                            Int iOffsetRight,
                                            Int iOffsetAbove,
                                            Int iOffsetBottom )
{
  m_piPatternOrigin = piTexture;
  m_iROIWidth       = iRoiWidth;
  m_iROIHeight      = iRoiHeight;
  m_iPatternStride  = iStride;
  m_iOffsetLeft     = iOffsetLeft;
  m_iOffsetAbove    = iOffsetAbove;
  m_iOffsetRight    = iOffsetRight;
  m_iOffsetBottom   = iOffsetBottom;
}

/** \param  pcCU          CU data structure
    \param  iComp         component index (0=Y, 1=Cb, 2=Cr)
    \param  iRoiWidth     pattern width
    \param  iRoiHeight    pattern height
    \param  iStride       buffer stride
    \param  iOffsetLeft   neighbour offset (left)
    \param  iOffsetRight  neighbour offset (right)
    \param  iOffsetAbove  neighbour offset (above)
    \param  iOffsetBottom neighbour offset (bottom)
    \param  uiPartDepth   CU depth
    \param  uiAbsPartIdx  part index
 */
Void TComPatternParam::setPatternParamCU( TComDataCU* pcCU,
                                          UChar       iComp,
                                          UChar       iRoiWidth,
                                          UChar       iRoiHeight,
                                          Int         iOffsetLeft,
                                          Int         iOffsetRight,
                                          Int         iOffsetAbove,
                                          Int         iOffsetBottom,
                                          UInt        uiPartDepth,
                                          UInt        uiAbsPartIdx )
{
  m_iOffsetLeft   = iOffsetLeft;
  m_iOffsetRight  = iOffsetRight;
  m_iOffsetAbove  = iOffsetAbove;
  m_iOffsetBottom = iOffsetBottom;

  m_iROIWidth     = iRoiWidth;
  m_iROIHeight    = iRoiHeight;

  UInt uiAbsZorderIdx = pcCU->getZorderIdxInCU() + uiAbsPartIdx;

  if ( iComp == 0 )
  {
    m_iPatternStride  = pcCU->getPic()->getStride();
    m_piPatternOrigin = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), uiAbsZorderIdx) - m_iOffsetAbove * m_iPatternStride - m_iOffsetLeft;
  }
  else
  {
    m_iPatternStride = pcCU->getPic()->getCStride();
    if ( iComp == 1 )
      m_piPatternOrigin = pcCU->getPic()->getPicYuvRec()->getCbAddr(pcCU->getAddr(), uiAbsZorderIdx) - m_iOffsetAbove * m_iPatternStride - m_iOffsetLeft;
    else
      m_piPatternOrigin = pcCU->getPic()->getPicYuvRec()->getCrAddr(pcCU->getAddr(), uiAbsZorderIdx) - m_iOffsetAbove * m_iPatternStride - m_iOffsetLeft;
  }
}

// ====================================================================================================================
// Public member functions (TComPattern)
// ====================================================================================================================

Void TComPattern::initPattern ( Pel* piY,
                                Pel* piCb,
                                Pel* piCr,
                                Int iRoiWidth,
                                Int iRoiHeight,
                                Int iStride,
                                Int iOffsetLeft,
                                Int iOffsetRight,
                                Int iOffsetAbove,
                                Int iOffsetBottom )
{
  m_cPatternY. setPatternParamPel( piY,  iRoiWidth,      iRoiHeight,      iStride,      iOffsetLeft,      iOffsetRight,      iOffsetAbove,      iOffsetBottom );
  m_cPatternCb.setPatternParamPel( piCb, iRoiWidth >> 1, iRoiHeight >> 1, iStride >> 1, iOffsetLeft >> 1, iOffsetRight >> 1, iOffsetAbove >> 1, iOffsetBottom >> 1 );
  m_cPatternCr.setPatternParamPel( piCr, iRoiWidth >> 1, iRoiHeight >> 1, iStride >> 1, iOffsetLeft >> 1, iOffsetRight >> 1, iOffsetAbove >> 1, iOffsetBottom >> 1 );

  return;
}

Void TComPattern::initPattern( TComDataCU* pcCU, UInt uiPartDepth, UInt uiAbsPartIdx )
{
  Int   uiOffsetLeft  = 0;
  Int   uiOffsetRight = 0;
  Int   uiOffsetAbove = 0;

  TComPic* pcPic         = pcCU->getPic();
  UChar uiWidth          = pcCU->getWidth (0)>>uiPartDepth;
  UChar uiHeight         = pcCU->getHeight(0)>>uiPartDepth;

  UInt  uiAbsZorderIdx   = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
  UInt  uiCurrPicPelX    = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsZorderIdx] ];
  UInt  uiCurrPicPelY    = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsZorderIdx] ];

  if( uiCurrPicPelX != 0 )
    uiOffsetLeft = 1;
  if( uiCurrPicPelY != 0 )
  {
    UInt uiNumPartInWidth = ( uiWidth/pcPic->getMinCUWidth() );
    uiOffsetAbove = 1;

    if( uiCurrPicPelX + uiWidth < pcPic->getSlice()->getSPS()->getWidth() )
    {
      if( ( g_auiZscanToRaster[uiAbsZorderIdx] + uiNumPartInWidth ) % pcPic->getNumPartInWidth() ) // Not CU boundary
      {
        if( g_auiRasterToZscan[ (Int)g_auiZscanToRaster[uiAbsZorderIdx] - (Int)pcPic->getNumPartInWidth() + (Int)uiNumPartInWidth ] < uiAbsZorderIdx )
          uiOffsetRight = 1;
      }
      else // if it is CU boundary
      {
        if( g_auiZscanToRaster[uiAbsZorderIdx] < pcPic->getNumPartInWidth() && (uiCurrPicPelX+uiWidth) < pcPic->getPicYuvRec()->getWidth() ) // first line
        {
          uiOffsetRight = 1;
        }
      }
    }
  }

  m_cPatternY .setPatternParamCU( pcCU, 0, uiWidth,      uiHeight,      uiOffsetLeft, uiOffsetRight, uiOffsetAbove, 0, uiPartDepth, uiAbsPartIdx );
  m_cPatternCb.setPatternParamCU( pcCU, 1, uiWidth >> 1, uiHeight >> 1, uiOffsetLeft, uiOffsetRight, uiOffsetAbove, 0, uiPartDepth, uiAbsPartIdx );
  m_cPatternCr.setPatternParamCU( pcCU, 2, uiWidth >> 1, uiHeight >> 1, uiOffsetLeft, uiOffsetRight, uiOffsetAbove, 0, uiPartDepth, uiAbsPartIdx );
}

Void TComPattern::initAdiPattern( TComDataCU* pcCU, UInt uiZorderIdxInPart, UInt uiPartDepth, Int* piAdiBuf, Int iOrgBufStride, Int iOrgBufHeight, Bool& bAbove, Bool& bLeft )
{
  Pel*  piRoiOrigin;
  Pel*  piRoiTemp;
  Int*  piAdiTemp;
  UInt  uiCuWidth   = pcCU->getWidth(0) >> uiPartDepth;
  UInt  uiCuHeight  = pcCU->getHeight(0)>> uiPartDepth;
  UInt  uiCuWidth2  = uiCuWidth<<1;
  UInt  uiCuHeight2 = uiCuHeight<<1;
  UInt  uiWidth;
  UInt  uiHeight;
  Int   iPicStride = pcCU->getPic()->getStride();
  Int   i;
  Int   iCuAddr;
  Bool  bAboveFlag      = false;
  Bool  bAboveRightFlag = false;
  Bool  bLeftFlag       = false;
  Bool  bBelowLeftFlag  = false;

  iCuAddr = pcCU->getAddr();

  UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB, uiPartDum;

  pcCU->deriveLeftRightTopIdxAdi( uiPartIdxLT, uiPartIdxRT, uiZorderIdxInPart, uiPartDepth );
  pcCU->deriveLeftBottomIdxAdi  ( uiPartIdxLB,              uiZorderIdxInPart, uiPartDepth );

  if( pcCU->getPUAbove        ( uiPartDum,             uiPartIdxLT ) ) bAboveFlag      = true;
  if( pcCU->getPUAboveRightAdi( uiPartDum, uiCuWidth,  uiPartIdxRT ) ) bAboveRightFlag = true;
  if( pcCU->getPULeft         ( uiPartDum,             uiPartIdxLT ) ) bLeftFlag       = true;
  if( pcCU->getPUBelowLeftAdi ( uiPartDum, uiCuHeight, uiPartIdxLB ) ) bBelowLeftFlag  = true;

  bAbove = bAboveFlag;
  bLeft  = bLeftFlag;

  if (uiCuWidth<=8)
    bBelowLeftFlag=false;

  uiWidth=uiCuWidth2+1;
  uiHeight=uiCuHeight2+1;

  if (((uiWidth<<2)>iOrgBufStride)||((uiHeight<<2)>iOrgBufHeight))  return;

  piRoiOrigin = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiZorderIdxInPart);

  piAdiTemp   = piAdiBuf;
  Int iDCValue = ( 1<<( g_uiBitDepth + g_uiBitIncrement - 1) );

  //BB: fill border with DC value - needed if( bAboveFlag=false || bLeftFlag=false )
  for (i=0;i<uiWidth;i++)
    piAdiTemp[i]=iDCValue;
  for (i=0;i<uiHeight;i++) //BB: can start from 1?
    piAdiTemp[i*uiWidth]=iDCValue;

  piRoiTemp=piRoiOrigin; //BB: not needed?

  if (bAboveFlag)
  {
    // BB: fill top border with rec. samples
    piRoiTemp=piRoiOrigin-iPicStride;
    for (i=0;i<uiCuWidth;i++)
      piAdiTemp[1+i]=piRoiTemp[i];
    // BB: fill top right border with rec. samples
    if (bAboveRightFlag)
    {
      piRoiTemp=piRoiOrigin-iPicStride+uiCuWidth;
      for (i=0;i<uiCuWidth;i++)
        piAdiTemp[1+uiCuWidth+i]=piRoiTemp[i];
    }
    // BB: fill top right border with last sample from top
    else
    {
      for (i=0;i<uiCuWidth;i++)
        piAdiTemp[1+uiCuWidth+i]=piAdiTemp[uiCuWidth];
    }
    // BB: fill top left border corner with rec. sample
    if (bLeftFlag)//BB: why left not top left?
    {
      piRoiTemp=piRoiOrigin-iPicStride-1;
      piAdiTemp[0]=piRoiTemp[0];
    }
  }

  if (bLeftFlag)
  {
    // BB: fill left border with rec. samples
    piRoiTemp=piRoiOrigin-1;
    for (i=0;i<uiCuHeight;i++)
    {
      piAdiTemp[(1+i)*uiWidth]=piRoiTemp[0];
      piRoiTemp+=iPicStride;
    }
    // BB: fill below left border with rec. samples
    if (bBelowLeftFlag)
    {
      for (i=0;i<uiCuHeight;i++)
      {
        piAdiTemp[(1+uiCuHeight+i)*uiWidth]=piRoiTemp[0];
        piRoiTemp+=iPicStride;
      }
    }
    // BB: fill below left border with last sample from left
    else
    {
      for (i=0;i<uiCuHeight;i++)
        piAdiTemp[(1+uiCuHeight+i)*uiWidth]=piAdiTemp[(uiCuHeight)*uiWidth];
    }
  }

  Int iBufSize=uiCuHeight2+uiCuWidth2+1;    // BB: left and left above border + above and above right border + top left corner = length of 3. filter buffer

  UInt uiWH = uiWidth*uiHeight;             // BB: number of elements in one buffer

  Int* piFilteredBuf1=piAdiBuf+uiWH;        // BB: 1. filter buffer
  Int* piFilteredBuf2=piFilteredBuf1+uiWH;  // BB: 2. filter buffer
  Int* piFilterBuf=piFilteredBuf2+uiWH;     // BB: buffer for 2. filtering (sequential)
  Int* piFilterBufN=piFilterBuf+iBufSize;   // BB: buffer for 1. filtering (sequential)

  Int l=0;
  // BB: left border from bottom to top
  for (i=0;i<uiCuHeight2;i++)
    piFilterBuf[l++]=piAdiTemp[uiWidth*(uiCuHeight2-i)];
  // BB: top left corner
  piFilterBuf[l++]=piAdiTemp[0];
  // BB: above border from left to right
  for (i=0;i<uiCuWidth2;i++)
    piFilterBuf[l++]=piAdiTemp[1+i];

  // BB: 1. filtering with [1 2 1]
  piFilterBufN[0]=piFilterBuf[0];
  piFilterBufN[iBufSize-1]=piFilterBuf[iBufSize-1];
  for (i=1;i<iBufSize-1;i++)
    piFilterBufN[i]=(piFilterBuf[i-1]+2*piFilterBuf[i]+piFilterBuf[i+1]+2)>>2;

  // BB: fill 1. filter buffer with filtered values
  l=0;
  for (i=0;i<uiCuHeight2;i++)
    piFilteredBuf1[uiWidth*(uiCuHeight2-i)]=piFilterBufN[l++];
  piFilteredBuf1[0]=piFilterBufN[l++];
  for (i=0;i<uiCuWidth2;i++)
    piFilteredBuf1[1+i]=piFilterBufN[l++];

  // BB: 2. filtering with [1 2 1]
  piFilterBuf[0]=piFilterBufN[0];                   //BB: needed? value not modified
  piFilterBuf[iBufSize-1]=piFilterBufN[iBufSize-1]; //BB: needed? value not modified
  for (i=1;i<iBufSize-1;i++)
    piFilterBuf[i]=(piFilterBufN[i-1]+2*piFilterBufN[i]+piFilterBufN[i+1]+2)>>2;

  // BB: fill 2. filter buffer with filtered values
  l=0;
  for (i=0;i<uiCuHeight2;i++)
    piFilteredBuf2[uiWidth*(uiCuHeight2-i)]=piFilterBuf[l++];
  piFilteredBuf2[0]=piFilterBuf[l++];
  for (i=0;i<uiCuWidth2;i++)
    piFilteredBuf2[1+i]=piFilterBuf[l++];
}

#if HHI_RQT_INTRA
Void TComPattern::initAdiPatternChroma( TComDataCU* pcCU, UInt uiZorderIdxInPart, UInt uiPartDepth, Int* piAdiBuf, Int iOrgBufStride, Int iOrgBufHeight, Bool& bAbove, Bool& bLeft )
#else
Void TComPattern::initAdiPatternChroma( TComDataCU* pcCU, UInt uiZorderIdxInPart, Int* piAdiBuf, Int iOrgBufStride, Int iOrgBufHeight, Bool& bAbove, Bool& bLeft )
#endif
{
  Pel*  piRoiOrigin;
  Pel*  piRoiTemp;
  Int*  piAdiTemp;
#if HHI_RQT_INTRA
  UInt  uiCuWidth  = pcCU->getWidth (0) >> uiPartDepth;
  UInt  uiCuHeight = pcCU->getHeight(0) >> uiPartDepth;
#else
  UInt  uiCuWidth  = pcCU->getWidth(0);
  UInt  uiCuHeight = pcCU->getHeight(0);
#endif
  UInt  uiWidth;
  UInt  uiHeight;
  Int   iPicStride = pcCU->getPic()->getCStride();
  Int   i;
  Int   iCuAddr;
  Bool  bAboveFlag=false;
  Bool  bAboveRightFlag=false;
  Bool  bLeftFlag=false;
  Bool  bBelowLeftFlag=false;

  iCuAddr = pcCU->getAddr();

  UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB,uiPartDum;

#if HHI_RQT_INTRA
  pcCU->deriveLeftRightTopIdxAdi( uiPartIdxLT, uiPartIdxRT, uiZorderIdxInPart, uiPartDepth );
  pcCU->deriveLeftBottomIdxAdi  ( uiPartIdxLB,              uiZorderIdxInPart, uiPartDepth );
#else
  pcCU->deriveLeftRightTopIdxAdi( uiPartIdxLT, uiPartIdxRT,uiZorderIdxInPart,0 );
  pcCU->deriveLeftBottomIdxAdi  ( uiPartIdxLB ,uiZorderIdxInPart,0);
#endif

  if( pcCU->getPUAbove     ( uiPartDum, uiPartIdxLT ) ) bAboveFlag      = true;
  if( pcCU->getPUAboveRightAdi( uiPartDum,uiCuWidth, uiPartIdxRT ) ) bAboveRightFlag = true;
  if( pcCU->getPULeft      ( uiPartDum, uiPartIdxLT ) ) bLeftFlag       = true;
  if( pcCU->getPUBelowLeftAdi (uiPartDum, uiCuHeight, uiPartIdxLB ) ) bBelowLeftFlag  = true;

  bAbove = bAboveFlag;
  bLeft  = bLeftFlag;

  uiCuWidth=uiCuWidth>>1;  // for chroma
  uiCuHeight=uiCuHeight>>1;  // for chroma

  uiWidth=uiCuWidth*2+1;
  uiHeight=uiCuHeight*2+1;

  if ((4*uiWidth>iOrgBufStride)||(4*uiHeight>iOrgBufHeight))  return;

  Int iDCValue = ( 1<<( g_uiBitDepth + g_uiBitIncrement - 1) );

  // get Cb pattern
  piRoiOrigin = pcCU->getPic()->getPicYuvRec()->getCbAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiZorderIdxInPart);
  piAdiTemp   = piAdiBuf;


  for (i=0;i<uiWidth;i++)
    piAdiTemp[i]=iDCValue;
  for (i=0;i<uiHeight;i++)
    piAdiTemp[i*uiWidth]=iDCValue;


  piRoiTemp=piRoiOrigin;

  if (bAboveFlag){
    piRoiTemp=piRoiOrigin-iPicStride;
    for (i=0;i<uiCuWidth;i++)
      piAdiTemp[1+i]=piRoiTemp[i];
    if (bAboveRightFlag){
      piRoiTemp=piRoiOrigin-iPicStride+uiCuWidth;
      for (i=0;i<uiCuWidth;i++)
        piAdiTemp[1+uiCuWidth+i]=piRoiTemp[i];
    }
    else {
      for (i=0;i<uiCuWidth;i++)
        piAdiTemp[1+uiCuWidth+i]=piAdiTemp[uiCuWidth];
    }
    if (bLeftFlag){
      piRoiTemp=piRoiOrigin-iPicStride-1;
      piAdiTemp[0]=piRoiTemp[0];
    }
  }

  if (bLeftFlag){
    piRoiTemp=piRoiOrigin-1;
    for (i=0;i<uiCuHeight;i++){
      piAdiTemp[(1+i)*uiWidth]=piRoiTemp[0];
      piRoiTemp+=iPicStride;
    }
    if (bBelowLeftFlag){
      for (i=0;i<uiCuHeight;i++){
        piAdiTemp[(1+uiCuHeight+i)*uiWidth]=piRoiTemp[0];
        piRoiTemp+=iPicStride;
      }
    }
    else {
      for (i=0;i<uiCuHeight;i++)
        piAdiTemp[(1+uiCuHeight+i)*uiWidth]=piAdiTemp[(uiCuHeight)*uiWidth];
    }
  }

  // get Cr pattern
  piRoiOrigin = pcCU->getPic()->getPicYuvRec()->getCrAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiZorderIdxInPart);
  piAdiTemp   = piAdiBuf+uiWidth*uiHeight;

  for (i=0;i<uiWidth;i++)
    piAdiTemp[i]=iDCValue;
  for (i=0;i<uiHeight;i++)
    piAdiTemp[i*uiWidth]=iDCValue;

  piRoiTemp=piRoiOrigin;

  if (bAboveFlag){
    piRoiTemp=piRoiOrigin-iPicStride;
    for (i=0;i<uiCuWidth;i++)
      piAdiTemp[1+i]=piRoiTemp[i];
    if (bAboveRightFlag){
      piRoiTemp=piRoiOrigin-iPicStride+uiCuWidth;
      for (i=0;i<uiCuWidth;i++)
        piAdiTemp[1+uiCuWidth+i]=piRoiTemp[i];
    }
    else {
      for (i=0;i<uiCuWidth;i++)
        piAdiTemp[1+uiCuWidth+i]=piAdiTemp[uiCuWidth];
    }
    if (bLeftFlag){
      piRoiTemp=piRoiOrigin-iPicStride-1;
      piAdiTemp[0]=piRoiTemp[0];
    }
  }

  if (bLeftFlag){
    piRoiTemp=piRoiOrigin-1;
    for (i=0;i<uiCuHeight;i++){
      piAdiTemp[(1+i)*uiWidth]=piRoiTemp[0];
      piRoiTemp+=iPicStride;
    }
    if (bBelowLeftFlag){
      for (i=0;i<uiCuHeight;i++){
        piAdiTemp[(1+uiCuHeight+i)*uiWidth]=piRoiTemp[0];
        piRoiTemp+=iPicStride;
      }
    }
    else {
      for (i=0;i<uiCuHeight;i++)
        piAdiTemp[(1+uiCuHeight+i)*uiWidth]=piAdiTemp[(uiCuHeight)*uiWidth];
    }
  }
}

Pel* TComPattern::getROIYBlk( Int iLumaBlkIdx )
{
  // iLumaBlkIdx : raster scan order
  Int iBlkX = (iLumaBlkIdx % 4) << 2;
  Int iBlkY = (iLumaBlkIdx / 4) << 2;

  return ( m_cPatternY.getROIOrigin() + m_cPatternY.m_iPatternStride * iBlkY + iBlkX );
}

Int* TComPattern::getAdiOrgBuf( Int iCuWidth, Int iCuHeight, Int* piAdiBuf)
{
  return piAdiBuf;
}

Int* TComPattern::getAdiFilteredBuf1(Int iCuWidth,Int iCuHeight, Int* piAdiBuf)
{
  return piAdiBuf+((iCuWidth<<1)+1)*((iCuHeight<<1)+1);
}

Int* TComPattern::getAdiFilteredBuf2(Int iCuWidth,Int iCuHeight, Int* piAdiBuf)
{
  return piAdiBuf+(((iCuWidth<<1)+1)*((iCuHeight<<1)+1)<<1);
}

Int* TComPattern::getAdiCbBuf( Int iCuWidth, Int iCuHeight, Int* piAdiBuf)
{
  return piAdiBuf;
}
Int* TComPattern::getAdiCrBuf(Int iCuWidth,Int iCuHeight, Int* piAdiBuf)
{
  return piAdiBuf+(iCuWidth*2+1)*(iCuHeight*2+1);
}

>>>>>>> upstream/master
