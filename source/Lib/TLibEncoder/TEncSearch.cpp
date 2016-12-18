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

/** \file     TEncSearch.cpp
 \brief    encoder search class
 */

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComRom.h"
#include "TLibCommon/TComMotionInfo.h"
#include "TEncSearch.h"
#include "TLibCommon/TComTU.h"
#include "TLibCommon/Debug.h"
#include <math.h>
#include <limits>


//! \ingroup TLibEncoder
//! \{

static const TComMv s_acMvRefineH[9] =
{
  TComMv(  0,  0 ), // 0
  TComMv(  0, -1 ), // 1
  TComMv(  0,  1 ), // 2
  TComMv( -1,  0 ), // 3
  TComMv(  1,  0 ), // 4
  TComMv( -1, -1 ), // 5
  TComMv(  1, -1 ), // 6
  TComMv( -1,  1 ), // 7
  TComMv(  1,  1 )  // 8
};

static const TComMv s_acMvRefineQ[9] =
{
  TComMv(  0,  0 ), // 0
  TComMv(  0, -1 ), // 1
  TComMv(  0,  1 ), // 2
  TComMv( -1, -1 ), // 5
  TComMv(  1, -1 ), // 6
  TComMv( -1,  0 ), // 3
  TComMv(  1,  0 ), // 4
  TComMv( -1,  1 ), // 7
  TComMv(  1,  1 )  // 8
};

static const UInt s_auiDFilter[9] =
{
  0, 1, 0,
  2, 3, 2,
  0, 1, 0
};

static Void offsetSubTUCBFs(TComTU &rTu, const ComponentID compID)
{
        TComDataCU *pcCU              = rTu.getCU();
  const UInt        uiTrDepth         = rTu.GetTransformDepthRel();
  const UInt        uiAbsPartIdx      = rTu.GetAbsPartIdxTU(compID);
  const UInt        partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID) >> 1;

  //move the CBFs down a level and set the parent CBF

  UChar subTUCBF[2];
  UChar combinedSubTUCBF = 0;

  for (UInt subTU = 0; subTU < 2; subTU++)
  {
    const UInt subTUAbsPartIdx = uiAbsPartIdx + (subTU * partIdxesPerSubTU);

    subTUCBF[subTU]   = pcCU->getCbf(subTUAbsPartIdx, compID, uiTrDepth);
    combinedSubTUCBF |= subTUCBF[subTU];
  }

  for (UInt subTU = 0; subTU < 2; subTU++)
  {
    const UInt subTUAbsPartIdx = uiAbsPartIdx + (subTU * partIdxesPerSubTU);
    const UChar compositeCBF = (subTUCBF[subTU] << 1) | combinedSubTUCBF;

    pcCU->setCbfPartRange((compositeCBF << uiTrDepth), compID, subTUAbsPartIdx, partIdxesPerSubTU);
  }
}


TEncSearch::TEncSearch()
{
  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    m_ppcQTTempCoeff[ch]                           = NULL;
    m_pcQTTempCoeff[ch]                            = NULL;
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempArlCoeff[ch]                        = NULL;
    m_pcQTTempArlCoeff[ch]                         = NULL;
#endif
    m_puhQTTempCbf[ch]                             = NULL;
    m_phQTTempCrossComponentPredictionAlpha[ch]    = NULL;
    m_pSharedPredTransformSkip[ch]                 = NULL;
    m_pcQTTempTUCoeff[ch]                          = NULL;
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempTUArlCoeff[ch]                      = NULL;
#endif
    m_puhQTTempTransformSkipFlag[ch]               = NULL;
  }
  m_puhQTTempTrIdx                                 = NULL;
  m_pcQTTempTComYuv                                = NULL;
  m_pcEncCfg                                       = NULL;
  m_pcEntropyCoder                                 = NULL;
  m_pTempPel                                       = NULL;
  setWpScalingDistParam( NULL, -1, REF_PIC_LIST_X );
}




TEncSearch::~TEncSearch()
{
  if ( m_pTempPel )
  {
    delete [] m_pTempPel;
    m_pTempPel = NULL;
  }

  if ( m_pcEncCfg )
  {
    const UInt uiNumLayersAllocated = m_pcEncCfg->getQuadtreeTULog2MaxSize()-m_pcEncCfg->getQuadtreeTULog2MinSize()+1;

    for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
    {
      for (UInt layer = 0; layer < uiNumLayersAllocated; layer++)
      {
        delete[] m_ppcQTTempCoeff[ch][layer];
#if ADAPTIVE_QP_SELECTION
        delete[] m_ppcQTTempArlCoeff[ch][layer];
#endif
      }
      delete[] m_ppcQTTempCoeff[ch];
      delete[] m_pcQTTempCoeff[ch];
      delete[] m_puhQTTempCbf[ch];
#if ADAPTIVE_QP_SELECTION
      delete[] m_ppcQTTempArlCoeff[ch];
      delete[] m_pcQTTempArlCoeff[ch];
#endif
    }

    for( UInt layer = 0; layer < uiNumLayersAllocated; layer++ )
    {
      m_pcQTTempTComYuv[layer].destroy();
    }
  }

  delete[] m_puhQTTempTrIdx;
  delete[] m_pcQTTempTComYuv;

  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    delete[] m_pSharedPredTransformSkip[ch];
    delete[] m_pcQTTempTUCoeff[ch];
#if ADAPTIVE_QP_SELECTION
    delete[] m_ppcQTTempTUArlCoeff[ch];
#endif
    delete[] m_phQTTempCrossComponentPredictionAlpha[ch];
    delete[] m_puhQTTempTransformSkipFlag[ch];
  }
  m_pcQTTempTransformSkipTComYuv.destroy();

  m_tmpYuvPred.destroy();
}




Void TEncSearch::init(TEncCfg*      pcEncCfg,
                      TComTrQuant*  pcTrQuant,
                      Int           iSearchRange,
                      Int           bipredSearchRange,
                      Int           iFastSearch,
                      const UInt    maxCUWidth,
                      const UInt    maxCUHeight,
                      const UInt    maxTotalCUDepth,
                      TEncEntropy*  pcEntropyCoder,
                      TComRdCost*   pcRdCost,
                      TEncSbac*** pppcRDSbacCoder,
                      TEncSbac*   pcRDGoOnSbacCoder
                      )
{
  m_pcEncCfg             = pcEncCfg;
  m_pcTrQuant            = pcTrQuant;
  m_iSearchRange         = iSearchRange;
  m_bipredSearchRange    = bipredSearchRange;
  m_iFastSearch          = iFastSearch;
  m_pcEntropyCoder       = pcEntropyCoder;
  m_pcRdCost             = pcRdCost;

  m_pppcRDSbacCoder     = pppcRDSbacCoder;
  m_pcRDGoOnSbacCoder   = pcRDGoOnSbacCoder;

  for (UInt iDir = 0; iDir < MAX_NUM_REF_LIST_ADAPT_SR; iDir++)
  {
    for (UInt iRefIdx = 0; iRefIdx < MAX_IDX_ADAPT_SR; iRefIdx++)
    {
      m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange;
    }
  }

  m_puiDFilter = s_auiDFilter + 4;

  // initialize motion cost
  for( Int iNum = 0; iNum < AMVP_MAX_NUM_CANDS+1; iNum++)
  {
    for( Int iIdx = 0; iIdx < AMVP_MAX_NUM_CANDS; iIdx++)
    {
      if (iIdx < iNum)
      {
        m_auiMVPIdxCost[iIdx][iNum] = xGetMvpIdxBits(iIdx, iNum);
      }
      else
      {
        m_auiMVPIdxCost[iIdx][iNum] = MAX_INT;
      }
    }
  }

  const ChromaFormat cform=pcEncCfg->getChromaFormatIdc();
  initTempBuff(cform);

  m_pTempPel = new Pel[maxCUWidth*maxCUHeight];

  const UInt uiNumLayersToAllocate = pcEncCfg->getQuadtreeTULog2MaxSize()-pcEncCfg->getQuadtreeTULog2MinSize()+1;
  const UInt uiNumPartitions = 1<<(maxTotalCUDepth<<1);
  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    const UInt csx=::getComponentScaleX(ComponentID(ch), cform);
    const UInt csy=::getComponentScaleY(ComponentID(ch), cform);
    m_ppcQTTempCoeff[ch] = new TCoeff* [uiNumLayersToAllocate];
    m_pcQTTempCoeff[ch]   = new TCoeff [(maxCUWidth*maxCUHeight)>>(csx+csy)   ];
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempArlCoeff[ch]  = new TCoeff*[uiNumLayersToAllocate];
    m_pcQTTempArlCoeff[ch]   = new TCoeff [(maxCUWidth*maxCUHeight)>>(csx+csy)   ];
#endif
    m_puhQTTempCbf[ch] = new UChar  [uiNumPartitions];

    for (UInt layer = 0; layer < uiNumLayersToAllocate; layer++)
    {
      m_ppcQTTempCoeff[ch][layer] = new TCoeff[(maxCUWidth*maxCUHeight)>>(csx+csy)];
#if ADAPTIVE_QP_SELECTION
      m_ppcQTTempArlCoeff[ch][layer]  = new TCoeff[(maxCUWidth*maxCUHeight)>>(csx+csy) ];
#endif
    }

    m_phQTTempCrossComponentPredictionAlpha[ch]    = new Char  [uiNumPartitions];
    m_pSharedPredTransformSkip[ch]                 = new Pel   [MAX_CU_SIZE*MAX_CU_SIZE];
    m_pcQTTempTUCoeff[ch]                          = new TCoeff[MAX_CU_SIZE*MAX_CU_SIZE];
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempTUArlCoeff[ch]                      = new TCoeff[MAX_CU_SIZE*MAX_CU_SIZE];
#endif
    m_puhQTTempTransformSkipFlag[ch]               = new UChar [uiNumPartitions];
  }
  m_puhQTTempTrIdx   = new UChar  [uiNumPartitions];
  m_pcQTTempTComYuv  = new TComYuv[uiNumLayersToAllocate];
  for( UInt ui = 0; ui < uiNumLayersToAllocate; ++ui )
  {
    m_pcQTTempTComYuv[ui].create( maxCUWidth, maxCUHeight, pcEncCfg->getChromaFormatIdc() );
  }
  m_pcQTTempTransformSkipTComYuv.create( maxCUWidth, maxCUHeight, pcEncCfg->getChromaFormatIdc() );
  m_tmpYuvPred.create(MAX_CU_SIZE, MAX_CU_SIZE, pcEncCfg->getChromaFormatIdc());
}

#define TZ_SEARCH_CONFIGURATION                                                                                 \
const Int  iRaster                  = 5;  /* TZ soll von aussen ?ergeben werden */                            \
const Bool bTestOtherPredictedMV    = 0;                                                                      \
const Bool bTestZeroVector          = 1;                                                                      \
const Bool bTestZeroVectorStart     = 0;                                                                      \
const Bool bTestZeroVectorStop      = 0;                                                                      \
const Bool bFirstSearchDiamond      = 1;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bFirstSearchStop         = m_pcEncCfg->getFastMEAssumingSmootherMVEnabled();                       \
const UInt uiFirstSearchRounds      = 3;  /* first search stop X rounds after best match (must be >=1) */     \
const Bool bEnableRasterSearch      = 1;                                                                      \
const Bool bAlwaysRasterSearch      = 0;  /* ===== 1: BETTER but factor 2 slower ===== */                     \
const Bool bRasterRefinementEnable  = 0;  /* enable either raster refinement or star refinement */            \
const Bool bRasterRefinementDiamond = 0;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bStarRefinementEnable    = 1;  /* enable either star refinement or raster refinement */            \
const Bool bStarRefinementDiamond   = 1;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bStarRefinementStop      = 0;                                                                      \
const UInt uiStarRefinementRounds   = 2;  /* star refinement stop X rounds after best match (must be >=1) */  \


#define SEL_SEARCH_CONFIGURATION                                                                                 \
  const Bool bTestOtherPredictedMV    = 1;                                                                       \
  const Bool bTestZeroVector          = 1;                                                                       \
  const Bool bEnableRasterSearch      = 1;                                                                       \
  const Bool bAlwaysRasterSearch      = 0;  /* ===== 1: BETTER but factor 15x slower ===== */                    \
  const Bool bStarRefinementEnable    = 1;  /* enable either star refinement or raster refinement */             \
  const Bool bStarRefinementDiamond   = 1;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */         \
  const Bool bStarRefinementStop      = 0;                                                                       \
  const UInt uiStarRefinementRounds   = 2;  /* star refinement stop X rounds after best match (must be >=1) */   \
  const UInt uiSearchRange            = m_iSearchRange;                                                          \
  const Int  uiSearchRangeInitial     = m_iSearchRange >> 2;                                                     \
  const Int  uiSearchStep             = 4;                                                                       \
  const Int  iMVDistThresh            = 8;                                                                       \



__inline Void TEncSearch::xTZSearchHelp( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, const Int iSearchX, const Int iSearchY, const UChar ucPointNr, const UInt uiDistance )
{
  Distortion  uiSad = 0;

  Pel*  piRefSrch;

  piRefSrch = rcStruct.piRefY + iSearchY * rcStruct.iYStride + iSearchX;

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( pcPatternKey, piRefSrch, rcStruct.iYStride,  m_cDistParam );

  if(m_pcEncCfg->getFastSearch() != SELECTIVE)
  {
    // fast encoder decision: use subsampled SAD when rows > 8 for integer ME
    if ( m_pcEncCfg->getUseFastEnc() )
    {
      if ( m_cDistParam.iRows > 8 )
      {
        m_cDistParam.iSubShift = 1;
      }
    }
  }

  setDistParamComp(COMPONENT_Y);

  // distortion
  m_cDistParam.bitDepth = pcPatternKey->getBitDepthY();
  if(m_pcEncCfg->getFastSearch() == SELECTIVE)
  {
    Int isubShift = 0;
    // motion cost
    Distortion uiBitCost = m_pcRdCost->getCost( iSearchX, iSearchY );

    if ( m_cDistParam.iRows > 32 )
    {
      m_cDistParam.iSubShift = 4;
    }
    else if ( m_cDistParam.iRows > 16 )
    {
      m_cDistParam.iSubShift = 3;
    }
    else if ( m_cDistParam.iRows > 8 )
    {
      m_cDistParam.iSubShift = 2;
    }
    else
    {
      m_cDistParam.iSubShift = 1;
    }

    Distortion uiTempSad = m_cDistParam.DistFunc( &m_cDistParam );
    if((uiTempSad + uiBitCost) < rcStruct.uiBestSad)
    {
      uiSad += uiTempSad >>  m_cDistParam.iSubShift;
      while(m_cDistParam.iSubShift > 0)
      {
        isubShift         = m_cDistParam.iSubShift -1;
        m_cDistParam.pOrg = pcPatternKey->getROIY() + (pcPatternKey->getPatternLStride() << isubShift);
        m_cDistParam.pCur = piRefSrch + (rcStruct.iYStride << isubShift);
        uiTempSad = m_cDistParam.DistFunc( &m_cDistParam );
        uiSad += uiTempSad >>  m_cDistParam.iSubShift;
        if(((uiSad << isubShift) + uiBitCost) > rcStruct.uiBestSad)
        {
          break;
        }

        m_cDistParam.iSubShift--;
      }

      if(m_cDistParam.iSubShift == 0)
      {
        uiSad += uiBitCost;
        if( uiSad < rcStruct.uiBestSad )
        {
          rcStruct.uiBestSad      = uiSad;
          rcStruct.iBestX         = iSearchX;
          rcStruct.iBestY         = iSearchY;
          rcStruct.uiBestDistance = uiDistance;
          rcStruct.uiBestRound    = 0;
          rcStruct.ucPointNr      = ucPointNr;
        }
      }
    }
  }
  else
  {
    uiSad = m_cDistParam.DistFunc( &m_cDistParam );

    // motion cost
    uiSad += m_pcRdCost->getCost( iSearchX, iSearchY );

    if( uiSad < rcStruct.uiBestSad )
    {
      rcStruct.uiBestSad      = uiSad;
      rcStruct.iBestX         = iSearchX;
      rcStruct.iBestY         = iSearchY;
      rcStruct.uiBestDistance = uiDistance;
      rcStruct.uiBestRound    = 0;
      rcStruct.ucPointNr      = ucPointNr;
    }
  }
}




__inline Void TEncSearch::xTZ2PointSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  // 2 point search,                   //   1 2 3
  // check only the 2 untested points  //   4 0 5
  // around the start point            //   6 7 8
  Int iStartX = rcStruct.iBestX;
  Int iStartY = rcStruct.iBestY;
  switch( rcStruct.ucPointNr )
  {
    case 1:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY, 0, 2 );
      }
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY - 1, 0, 2 );
      }
    }
      break;
    case 2:
    {
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        if ( (iStartX - 1) >= iSrchRngHorLeft )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
        }
        if ( (iStartX + 1) <= iSrchRngHorRight )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
        }
      }
    }
      break;
    case 3:
    {
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY - 1, 0, 2 );
      }
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY, 0, 2 );
      }
    }
      break;
    case 4:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        if ( (iStartY + 1) <= iSrchRngVerBottom )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
        }
        if ( (iStartY - 1) >= iSrchRngVerTop )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
        }
      }
    }
      break;
    case 5:
    {
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        if ( (iStartY - 1) >= iSrchRngVerTop )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
        }
        if ( (iStartY + 1) <= iSrchRngVerBottom )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
        }
      }
    }
      break;
    case 6:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY , 0, 2 );
      }
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY + 1, 0, 2 );
      }
    }
      break;
    case 7:
    {
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        if ( (iStartX - 1) >= iSrchRngHorLeft )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
        }
        if ( (iStartX + 1) <= iSrchRngHorRight )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
        }
      }
    }
      break;
    case 8:
    {
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY, 0, 2 );
      }
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY + 1, 0, 2 );
      }
    }
      break;
    default:
    {
      assert( false );
    }
      break;
  } // switch( rcStruct.ucPointNr )
}




__inline Void TEncSearch::xTZ8PointSquareSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  assert( iDist != 0 );
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iTop >= iSrchRngVerTop ) // check top
  {
    if ( iLeft >= iSrchRngHorLeft ) // check top left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iTop, 1, iDist );
    }
    // top middle
    xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );

    if ( iRight <= iSrchRngHorRight ) // check top right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iTop, 3, iDist );
    }
  } // check top
  if ( iLeft >= iSrchRngHorLeft ) // check middle left
  {
    xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
  }
  if ( iRight <= iSrchRngHorRight ) // check middle right
  {
    xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
  }
  if ( iBottom <= iSrchRngVerBottom ) // check bottom
  {
    if ( iLeft >= iSrchRngHorLeft ) // check bottom left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iBottom, 6, iDist );
    }
    // check bottom middle
    xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );

    if ( iRight <= iSrchRngHorRight ) // check bottom right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iBottom, 8, iDist );
    }
  } // check bottom
}




__inline Void TEncSearch::xTZ8PointDiamondSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  assert ( iDist != 0 );
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iDist == 1 ) // iDist == 1
  {
    if ( iTop >= iSrchRngVerTop ) // check top
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
    }
    if ( iLeft >= iSrchRngHorLeft ) // check middle left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
    }
    if ( iRight <= iSrchRngHorRight ) // check middle right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
    }
    if ( iBottom <= iSrchRngVerBottom ) // check bottom
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
    }
  }
  else // if (iDist != 1)
  {
    if ( iDist <= 8 )
    {
      const Int iTop_2      = iStartY - (iDist>>1);
      const Int iBottom_2   = iStartY + (iDist>>1);
      const Int iLeft_2     = iStartX - (iDist>>1);
      const Int iRight_2    = iStartX + (iDist>>1);

      if (  iTop >= iSrchRngVerTop && iLeft >= iSrchRngHorLeft &&
          iRight <= iSrchRngHorRight && iBottom <= iSrchRngVerBottom ) // check border
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX,  iTop,      2, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2,  iTop_2,    1, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iTop_2,    3, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft,    iStartY,   4, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight,   iStartY,   5, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2,  iBottom_2, 6, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iBottom_2, 8, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX,  iBottom,   7, iDist    );
      }
      else // check border
      {
        if ( iTop >= iSrchRngVerTop ) // check top
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
        }
        if ( iTop_2 >= iSrchRngVerTop ) // check half top
        {
          if ( iLeft_2 >= iSrchRngHorLeft ) // check half left
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2, iTop_2, 1, (iDist>>1) );
          }
          if ( iRight_2 <= iSrchRngHorRight ) // check half right
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iTop_2, 3, (iDist>>1) );
          }
        } // check half top
        if ( iLeft >= iSrchRngHorLeft ) // check left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
        }
        if ( iRight <= iSrchRngHorRight ) // check right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
        }
        if ( iBottom_2 <= iSrchRngVerBottom ) // check half bottom
        {
          if ( iLeft_2 >= iSrchRngHorLeft ) // check half left
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2, iBottom_2, 6, (iDist>>1) );
          }
          if ( iRight_2 <= iSrchRngHorRight ) // check half right
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iBottom_2, 8, (iDist>>1) );
          }
        } // check half bottom
        if ( iBottom <= iSrchRngVerBottom ) // check bottom
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
        }
      } // check border
    }
    else // iDist > 8
    {
      if ( iTop >= iSrchRngVerTop && iLeft >= iSrchRngHorLeft &&
          iRight <= iSrchRngHorRight && iBottom <= iSrchRngVerBottom ) // check border
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop,    0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft,   iStartY, 0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight,  iStartY, 0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 0, iDist );
        for ( Int index = 1; index < 4; index++ )
        {
          Int iPosYT = iTop    + ((iDist>>2) * index);
          Int iPosYB = iBottom - ((iDist>>2) * index);
          Int iPosXL = iStartX - ((iDist>>2) * index);
          Int iPosXR = iStartX + ((iDist>>2) * index);
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYT, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYT, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYB, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYB, 0, iDist );
        }
      }
      else // check border
      {
        if ( iTop >= iSrchRngVerTop ) // check top
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 0, iDist );
        }
        if ( iLeft >= iSrchRngHorLeft ) // check left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 0, iDist );
        }
        if ( iRight <= iSrchRngHorRight ) // check right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 0, iDist );
        }
        if ( iBottom <= iSrchRngVerBottom ) // check bottom
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 0, iDist );
        }
        for ( Int index = 1; index < 4; index++ )
        {
          Int iPosYT = iTop    + ((iDist>>2) * index);
          Int iPosYB = iBottom - ((iDist>>2) * index);
          Int iPosXL = iStartX - ((iDist>>2) * index);
          Int iPosXR = iStartX + ((iDist>>2) * index);

          if ( iPosYT >= iSrchRngVerTop ) // check top
          {
            if ( iPosXL >= iSrchRngHorLeft ) // check left
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYT, 0, iDist );
            }
            if ( iPosXR <= iSrchRngHorRight ) // check right
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYT, 0, iDist );
            }
          } // check top
          if ( iPosYB <= iSrchRngVerBottom ) // check bottom
          {
            if ( iPosXL >= iSrchRngHorLeft ) // check left
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYB, 0, iDist );
            }
            if ( iPosXR <= iSrchRngHorRight ) // check right
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYB, 0, iDist );
            }
          } // check bottom
        } // for ...
      } // check border
    } // iDist <= 8
  } // iDist == 1
}





//<--

Distortion TEncSearch::xPatternRefinement( TComPattern* pcPatternKey,
                                           TComMv baseRefMv,
                                           Int iFrac, TComMv& rcMvFrac,
                                           Bool bAllowUseOfHadamard
                                         )
{
  Distortion  uiDist;
  Distortion  uiDistBest  = std::numeric_limits<Distortion>::max();
  UInt        uiDirecBest = 0;

  Pel*  piRefPos;
  Int iRefStride = m_filteredBlock[0][0].getStride(COMPONENT_Y);

  m_pcRdCost->setDistParam( pcPatternKey, m_filteredBlock[0][0].getAddr(COMPONENT_Y), iRefStride, 1, m_cDistParam, m_pcEncCfg->getUseHADME() && bAllowUseOfHadamard );

  const TComMv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);

  for (UInt i = 0; i < 9; i++)
  {
    TComMv cMvTest = pcMvRefine[i];
    cMvTest += baseRefMv;

    Int horVal = cMvTest.getHor() * iFrac;
    Int verVal = cMvTest.getVer() * iFrac;
    piRefPos = m_filteredBlock[ verVal & 3 ][ horVal & 3 ].getAddr(COMPONENT_Y);
    if ( horVal == 2 && ( verVal & 1 ) == 0 )
    {
      piRefPos += 1;
    }
    if ( ( horVal & 1 ) == 0 && verVal == 2 )
    {
      piRefPos += iRefStride;
    }
    cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;

    setDistParamComp(COMPONENT_Y);

    m_cDistParam.pCur = piRefPos;
    m_cDistParam.bitDepth = pcPatternKey->getBitDepthY();
    uiDist = m_cDistParam.DistFunc( &m_cDistParam );
    uiDist += m_pcRdCost->getCost( cMvTest.getHor(), cMvTest.getVer() );

    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
    }
  }

  rcMvFrac = pcMvRefine[uiDirecBest];

  return uiDistBest;
}



Void
TEncSearch::xEncSubdivCbfQT(TComTU      &rTu,
                            Bool         bLuma,
                            Bool         bChroma )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx         = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth            = rTu.GetTransformDepthRel();
  const UInt uiTrMode             = pcCU->getTransformIdx( uiAbsPartIdx );
  const UInt uiSubdiv             = ( uiTrMode > uiTrDepth ? 1 : 0 );
  const UInt uiLog2LumaTrafoSize  = rTu.GetLog2LumaTrSize();

  if( pcCU->isIntra(0) && pcCU->getPartitionSize(0) == SIZE_NxN && uiTrDepth == 0 )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2LumaTrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2LumaTrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
  {
    assert( !uiSubdiv );
  }
  else if( uiLog2LumaTrafoSize == pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
  {
    assert( !uiSubdiv );
  }
  else
  {
    assert( uiLog2LumaTrafoSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
    if( bLuma )
    {
      m_pcEntropyCoder->encodeTransformSubdivFlag( uiSubdiv, 5 - uiLog2LumaTrafoSize );
    }
  }

  if ( bChroma )
  {
    const UInt numberValidComponents = getNumberValidComponents(rTu.GetChromaFormat());
    for (UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
    {
      const ComponentID compID=ComponentID(ch);
      if( rTu.ProcessingAllQuadrants(compID) && (uiTrDepth==0 || pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepth-1 ) ))
      {
        m_pcEntropyCoder->encodeQtCbf(rTu, compID, (uiSubdiv == 0));
      }
    }
  }

  if( uiSubdiv )
  {
    TComTURecurse tuRecurse(rTu, false);
    do
    {
      xEncSubdivCbfQT( tuRecurse, bLuma, bChroma );
    } while (tuRecurse.nextSection(rTu));
  }
  else
  {
    //===== Cbfs =====
    if( bLuma )
    {
      m_pcEntropyCoder->encodeQtCbf( rTu, COMPONENT_Y, true );
    }
  }
}




Void
TEncSearch::xEncCoeffQT(TComTU &rTu,
                        const ComponentID  component,
                        Bool         bRealCoeff )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth=rTu.GetTransformDepthRel();

  const UInt  uiTrMode        = pcCU->getTransformIdx( uiAbsPartIdx );
  const UInt  uiSubdiv        = ( uiTrMode > uiTrDepth ? 1 : 0 );

  if( uiSubdiv )
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xEncCoeffQT( tuRecurseChild, component, bRealCoeff );
    } while (tuRecurseChild.nextSection(rTu) );
  }
  else if (rTu.ProcessComponentSection(component))
  {
    //===== coefficients =====
    const UInt  uiLog2TrafoSize = rTu.GetLog2LumaTrSize();
    UInt    uiCoeffOffset   = rTu.getCoefficientOffset(component);
    UInt    uiQTLayer       = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrafoSize;
    TCoeff* pcCoeff         = bRealCoeff ? pcCU->getCoeff(component) : m_ppcQTTempCoeff[component][uiQTLayer];

    if (isChroma(component) && (pcCU->getCbf( rTu.GetAbsPartIdxTU(), COMPONENT_Y, uiTrMode ) != 0) && pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() )
    {
      m_pcEntropyCoder->encodeCrossComponentPrediction( rTu, component );
    }

    m_pcEntropyCoder->encodeCoeffNxN( rTu, pcCoeff+uiCoeffOffset, component );
  }
}




Void
TEncSearch::xEncIntraHeader( TComDataCU*  pcCU,
                            UInt         uiTrDepth,
                            UInt         uiAbsPartIdx,
                            Bool         bLuma,
                            Bool         bChroma )
{
  if( bLuma )
  {
    // CU header
    if( uiAbsPartIdx == 0 )
    {
      if( !pcCU->getSlice()->isIntra() )
      {
        if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
        {
          m_pcEntropyCoder->encodeCUTransquantBypassFlag( pcCU, 0, true );
        }
        m_pcEntropyCoder->encodeSkipFlag( pcCU, 0, true );
        m_pcEntropyCoder->encodePredMode( pcCU, 0, true );
      }
      m_pcEntropyCoder  ->encodePartSize( pcCU, 0, pcCU->getDepth(0), true );

      if (pcCU->isIntra(0) && pcCU->getPartitionSize(0) == SIZE_2Nx2N )
      {
        m_pcEntropyCoder->encodeIPCMInfo( pcCU, 0, true );

        if ( pcCU->getIPCMFlag (0))
        {
          return;
        }
      }
    }
    // luma prediction mode
    if( pcCU->getPartitionSize(0) == SIZE_2Nx2N )
    {
      if (uiAbsPartIdx==0)
      {
        m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, 0 );
      }
    }
    else
    {
      UInt uiQNumParts = pcCU->getTotalNumPart() >> 2;
      if (uiTrDepth>0 && (uiAbsPartIdx%uiQNumParts)==0)
      {
        m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiAbsPartIdx );
      }
    }
  }

  if( bChroma )
  {
    if( pcCU->getPartitionSize(0) == SIZE_2Nx2N || !enable4ChromaPUsInIntraNxNCU(pcCU->getPic()->getChromaFormat()))
    {
      if(uiAbsPartIdx==0)
      {
         m_pcEntropyCoder->encodeIntraDirModeChroma ( pcCU, uiAbsPartIdx );
      }
    }
    else
    {
      UInt uiQNumParts = pcCU->getTotalNumPart() >> 2;
      assert(uiTrDepth>0);
      if ((uiAbsPartIdx%uiQNumParts)==0)
      {
        m_pcEntropyCoder->encodeIntraDirModeChroma ( pcCU, uiAbsPartIdx );
      }
    }
  }
}




UInt
TEncSearch::xGetIntraBitsQT(TComTU &rTu,
                            Bool         bLuma,
                            Bool         bChroma,
                            Bool         bRealCoeff /* just for test */ )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth=rTu.GetTransformDepthRel();
  m_pcEntropyCoder->resetBits();
  xEncIntraHeader ( pcCU, uiTrDepth, uiAbsPartIdx, bLuma, bChroma );
  xEncSubdivCbfQT ( rTu, bLuma, bChroma );

  if( bLuma )
  {
    xEncCoeffQT   ( rTu, COMPONENT_Y,      bRealCoeff );
  }
  if( bChroma )
  {
    xEncCoeffQT   ( rTu, COMPONENT_Cb,  bRealCoeff );
    xEncCoeffQT   ( rTu, COMPONENT_Cr,  bRealCoeff );
  }
  UInt   uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();

  return uiBits;
}

UInt TEncSearch::xGetIntraBitsQTChroma(TComTU &rTu,
                                       ComponentID compID,
                                       Bool         bRealCoeff /* just for test */ )
{
  m_pcEntropyCoder->resetBits();
  xEncCoeffQT   ( rTu, compID,  bRealCoeff );
  UInt   uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  return uiBits;
}

Void TEncSearch::xIntraCodingTUBlock(       TComYuv*    pcOrgYuv,
                                            TComYuv*    pcPredYuv,
                                            TComYuv*    pcResiYuv,
                                            Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                      const Bool        checkCrossCPrediction,
                                            Distortion& ruiDist,
                                      const ComponentID compID,
                                            TComTU&     rTu
                                      DEBUG_STRING_FN_DECLARE(sDebug)
                                           ,Int         default0Save1Load2
                                     )
{
  if (!rTu.ProcessComponentSection(compID))
  {
    return;
  }
  const Bool           bIsLuma          = isLuma(compID);
  const TComRectangle &rect             = rTu.getRect(compID);
        TComDataCU    *pcCU             = rTu.getCU();
  const UInt           uiAbsPartIdx     = rTu.GetAbsPartIdxTU();
  const TComSPS       &sps              = *(pcCU->getSlice()->getSPS());

  const UInt           uiTrDepth        = rTu.GetTransformDepthRelAdj(compID);
  const UInt           uiFullDepth      = rTu.GetTransformDepthTotal();
  const UInt           uiLog2TrSize     = rTu.GetLog2LumaTrSize();
  const ChromaFormat   chFmt            = pcOrgYuv->getChromaFormat();
  const ChannelType    chType           = toChannelType(compID);
  const Int            bitDepth         = sps.getBitDepth(chType);

  const UInt           uiWidth          = rect.width;
  const UInt           uiHeight         = rect.height;
  const UInt           uiStride         = pcOrgYuv ->getStride (compID);
        Pel           *piOrg            = pcOrgYuv ->getAddr( compID, uiAbsPartIdx );
        Pel           *piPred           = pcPredYuv->getAddr( compID, uiAbsPartIdx );
        Pel           *piResi           = pcResiYuv->getAddr( compID, uiAbsPartIdx );
        Pel           *piReco           = pcPredYuv->getAddr( compID, uiAbsPartIdx );
  const UInt           uiQTLayer        = sps.getQuadtreeTULog2MaxSize() - uiLog2TrSize;
        Pel           *piRecQt          = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
  const UInt           uiRecQtStride    = m_pcQTTempTComYuv[ uiQTLayer ].getStride(compID);
  const UInt           uiZOrder         = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;
        Pel           *piRecIPred       = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
        UInt           uiRecIPredStride = pcCU->getPic()->getPicYuvRec()->getStride  ( compID );
        TCoeff        *pcCoeff          = m_ppcQTTempCoeff[compID][uiQTLayer] + rTu.getCoefficientOffset(compID);
        Bool           useTransformSkip = pcCU->getTransformSkip(uiAbsPartIdx, compID);

#if ADAPTIVE_QP_SELECTION
        TCoeff        *pcArlCoeff       = m_ppcQTTempArlCoeff[compID][ uiQTLayer ] + rTu.getCoefficientOffset(compID);
#endif

  const UInt           uiChPredMode     = pcCU->getIntraDir( chType, uiAbsPartIdx );
  const UInt           partsPerMinCU    = 1<<(2*(sps.getMaxTotalCUDepth() - sps.getLog2DiffMaxMinCodingBlockSize()));
  const UInt           uiChCodedMode    = (uiChPredMode==DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, chFmt, partsPerMinCU)) : uiChPredMode;
  const UInt           uiChFinalMode    = ((chFmt == CHROMA_422)       && !bIsLuma) ? g_chroma422IntraAngleMappingTable[uiChCodedMode] : uiChCodedMode;

  const Int            blkX                                 = g_auiRasterToPelX[ g_auiZscanToRaster[ uiAbsPartIdx ] ];
  const Int            blkY                                 = g_auiRasterToPelY[ g_auiZscanToRaster[ uiAbsPartIdx ] ];
  const Int            bufferOffset                         = blkX + (blkY * MAX_CU_SIZE);
        Pel  *const    encoderLumaResidual                  = resiLuma[RESIDUAL_ENCODER_SIDE ] + bufferOffset;
        Pel  *const    reconstructedLumaResidual            = resiLuma[RESIDUAL_RECONSTRUCTED] + bufferOffset;
  const Bool           bUseCrossCPrediction                 = isChroma(compID) && (uiChPredMode == DM_CHROMA_IDX) && checkCrossCPrediction;
  const Bool           bUseReconstructedResidualForEstimate = m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate();
        Pel *const     lumaResidualForEstimate              = bUseReconstructedResidualForEstimate ? reconstructedLumaResidual : encoderLumaResidual;

#if DEBUG_STRING
  const Int debugPredModeMask=DebugStringGetPredModeMask(MODE_INTRA);
#endif

  //===== init availability pattern =====
  Bool  bAboveAvail = false;
  Bool  bLeftAvail  = false;

  DEBUG_STRING_NEW(sTemp)

#if !DEBUG_STRING
  if( default0Save1Load2 != 2 )
#endif
  {
    const Bool bUseFilteredPredictions=TComPrediction::filteringIntraReferenceSamples(compID, uiChFinalMode, uiWidth, uiHeight, chFmt, sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag());

    initIntraPatternChType( rTu, bAboveAvail, bLeftAvail, compID, bUseFilteredPredictions DEBUG_STRING_PASS_INTO(sDebug) );

    //===== get prediction signal =====
    predIntraAng( compID, uiChFinalMode, piOrg, uiStride, piPred, uiStride, rTu, bAboveAvail, bLeftAvail, bUseFilteredPredictions );

    // save prediction
    if( default0Save1Load2 == 1 )
    {
      Pel*  pPred   = piPred;
      Pel*  pPredBuf = m_pSharedPredTransformSkip[compID];
      Int k = 0;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pPredBuf[ k ++ ] = pPred[ uiX ];
        }
        pPred += uiStride;
      }
    }
  }
#if !DEBUG_STRING
  else
  {
    // load prediction
    Pel*  pPred   = piPred;
    Pel*  pPredBuf = m_pSharedPredTransformSkip[compID];
    Int k = 0;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pPred[ uiX ] = pPredBuf[ k ++ ];
      }
      pPred += uiStride;
    }
  }
#endif

  //===== get residual signal =====
  {
    // get residual
    Pel*  pOrg    = piOrg;
    Pel*  pPred   = piPred;
    Pel*  pResi   = piResi;

    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pResi[ uiX ] = pOrg[ uiX ] - pPred[ uiX ];
      }

      pOrg  += uiStride;
      pResi += uiStride;
      pPred += uiStride;
    }
  }

  if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
  {
    if (bUseCrossCPrediction)
    {
      if (xCalcCrossComponentPredictionAlpha( rTu, compID, lumaResidualForEstimate, piResi, uiWidth, uiHeight, MAX_CU_SIZE, uiStride ) == 0)
      {
        return;
      }
      TComTrQuant::crossComponentPrediction ( rTu, compID, reconstructedLumaResidual, piResi, piResi, uiWidth, uiHeight, MAX_CU_SIZE, uiStride, uiStride, false );
    }
    else if (isLuma(compID) && !bUseReconstructedResidualForEstimate)
    {
      xStoreCrossComponentPredictionResult( encoderLumaResidual, piResi, rTu, 0, 0, MAX_CU_SIZE, uiStride );
    }
  }

  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
  if( useTransformSkip ? m_pcEncCfg->getUseRDOQTS() : m_pcEncCfg->getUseRDOQ() )
  {
    m_pcEntropyCoder->estimateBit( m_pcTrQuant->m_pcEstBitsSbac, uiWidth, uiHeight, chType );
  }

  //--- transform and quantization ---
  TCoeff uiAbsSum = 0;
  if (bIsLuma)
  {
    pcCU       ->setTrIdxSubParts ( uiTrDepth, uiAbsPartIdx, uiFullDepth );
  }

  const QpParam cQP(*pcCU, compID);

#if RDOQ_CHROMA_LAMBDA
  m_pcTrQuant->selectLambda     (compID);
#endif

  m_pcTrQuant->transformNxN     ( rTu, compID, piResi, uiStride, pcCoeff,
#if ADAPTIVE_QP_SELECTION
    pcArlCoeff,
#endif
    uiAbsSum, cQP
    );

  //--- inverse transform ---

#if DEBUG_STRING
  if ( (uiAbsSum > 0) || (DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask) )
#else
  if ( uiAbsSum > 0 )
#endif
  {
    m_pcTrQuant->invTransformNxN ( rTu, compID, piResi, uiStride, pcCoeff, cQP DEBUG_STRING_PASS_INTO_OPTIONAL(&sDebug, (DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask)) );
  }
  else
  {
    Pel* pResi = piResi;
    memset( pcCoeff, 0, sizeof( TCoeff ) * uiWidth * uiHeight );
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      memset( pResi, 0, sizeof( Pel ) * uiWidth );
      pResi += uiStride;
    }
  }


  //===== reconstruction =====
  {
    Pel* pPred      = piPred;
    Pel* pResi      = piResi;
    Pel* pReco      = piReco;
    Pel* pRecQt     = piRecQt;
    Pel* pRecIPred  = piRecIPred;

    if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
    {
      if (bUseCrossCPrediction)
      {
        TComTrQuant::crossComponentPrediction( rTu, compID, reconstructedLumaResidual, piResi, piResi, uiWidth, uiHeight, MAX_CU_SIZE, uiStride, uiStride, true );
      }
      else if (isLuma(compID))
      {
        xStoreCrossComponentPredictionResult( reconstructedLumaResidual, piResi, rTu, 0, 0, MAX_CU_SIZE, uiStride );
      }
    }

 #if DEBUG_STRING
    std::stringstream ss(stringstream::out);
    const Bool bDebugPred=((DebugOptionList::DebugString_Pred.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));
    const Bool bDebugResi=((DebugOptionList::DebugString_Resi.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));
    const Bool bDebugReco=((DebugOptionList::DebugString_Reco.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));

    if (bDebugPred || bDebugResi || bDebugReco)
    {
      ss << "###: " << "CompID: " << compID << " pred mode (ch/fin): " << uiChPredMode << "/" << uiChFinalMode << " absPartIdx: " << rTu.GetAbsPartIdxTU() << "\n";
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        ss << "###: ";
        if (bDebugPred)
        {
          ss << " - pred: ";
          for( UInt uiX = 0; uiX < uiWidth; uiX++ )
          {
            ss << pPred[ uiX ] << ", ";
          }
        }
        if (bDebugResi)
        {
          ss << " - resi: ";
        }
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          if (bDebugResi)
          {
            ss << pResi[ uiX ] << ", ";
          }
          pReco    [ uiX ] = Pel(ClipBD<Int>( Int(pPred[uiX]) + Int(pResi[uiX]), bitDepth ));
          pRecQt   [ uiX ] = pReco[ uiX ];
          pRecIPred[ uiX ] = pReco[ uiX ];
        }
        if (bDebugReco)
        {
          ss << " - reco: ";
          for( UInt uiX = 0; uiX < uiWidth; uiX++ )
          {
            ss << pReco[ uiX ] << ", ";
          }
        }
        pPred     += uiStride;
        pResi     += uiStride;
        pReco     += uiStride;
        pRecQt    += uiRecQtStride;
        pRecIPred += uiRecIPredStride;
        ss << "\n";
      }
      DEBUG_STRING_APPEND(sDebug, ss.str())
    }
    else
#endif
    {

      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pReco    [ uiX ] = Pel(ClipBD<Int>( Int(pPred[uiX]) + Int(pResi[uiX]), bitDepth ));
          pRecQt   [ uiX ] = pReco[ uiX ];
          pRecIPred[ uiX ] = pReco[ uiX ];
        }
        pPred     += uiStride;
        pResi     += uiStride;
        pReco     += uiStride;
        pRecQt    += uiRecQtStride;
        pRecIPred += uiRecIPredStride;
      }
    }
  }

  //===== update distortion =====
  ruiDist += m_pcRdCost->getDistPart( bitDepth, piReco, uiStride, piOrg, uiStride, uiWidth, uiHeight, compID );
}




Void
TEncSearch::xRecurIntraCodingLumaQT(TComYuv*    pcOrgYuv,
                                    TComYuv*    pcPredYuv,
                                    TComYuv*    pcResiYuv,
                                    Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                    Distortion& ruiDistY,
#if HHI_RQT_INTRA_SPEEDUP
                                    Bool        bCheckFirst,
#endif
                                    Double&     dRDCost,
                                    TComTU&     rTu
                                    DEBUG_STRING_FN_DECLARE(sDebug))
{
  TComDataCU   *pcCU          = rTu.getCU();
  const UInt    uiAbsPartIdx  = rTu.GetAbsPartIdxTU();
  const UInt    uiFullDepth   = rTu.GetTransformDepthTotal();
  const UInt    uiTrDepth     = rTu.GetTransformDepthRel();
  const UInt    uiLog2TrSize  = rTu.GetLog2LumaTrSize();
        Bool    bCheckFull    = ( uiLog2TrSize  <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
        Bool    bCheckSplit   = ( uiLog2TrSize  >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );

        Pel     resiLumaSplit [NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE];
        Pel     resiLumaSingle[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE];

        Bool    bMaintainResidual[NUMBER_OF_STORED_RESIDUAL_TYPES];
        for (UInt residualTypeIndex = 0; residualTypeIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; residualTypeIndex++)
        {
          bMaintainResidual[residualTypeIndex] = true; //assume true unless specified otherwise
        }

        bMaintainResidual[RESIDUAL_ENCODER_SIDE] = !(m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate());

#if HHI_RQT_INTRA_SPEEDUP
  Int maxTuSize = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize();
  Int isIntraSlice = (pcCU->getSlice()->getSliceType() == I_SLICE);
  // don't check split if TU size is less or equal to max TU size
  Bool noSplitIntraMaxTuSize = bCheckFull;
  if(m_pcEncCfg->getRDpenalty() && ! isIntraSlice)
  {
    // in addition don't check split if TU size is less or equal to 16x16 TU size for non-intra slice
    noSplitIntraMaxTuSize = ( uiLog2TrSize  <= min(maxTuSize,4) );

    // if maximum RD-penalty don't check TU size 32x32
    if(m_pcEncCfg->getRDpenalty()==2)
    {
      bCheckFull    = ( uiLog2TrSize  <= min(maxTuSize,4));
    }
  }
  if( bCheckFirst && noSplitIntraMaxTuSize )

  {
    bCheckSplit = false;
  }
#else
  Int maxTuSize = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize();
  Int isIntraSlice = (pcCU->getSlice()->getSliceType() == I_SLICE);
  // if maximum RD-penalty don't check TU size 32x32
  if((m_pcEncCfg->getRDpenalty()==2)  && !isIntraSlice)
  {
    bCheckFull    = ( uiLog2TrSize  <= min(maxTuSize,4));
  }
#endif
  Double     dSingleCost                        = MAX_DOUBLE;
  Distortion uiSingleDistLuma                   = 0;
  UInt       uiSingleCbfLuma                    = 0;
  Bool       checkTransformSkip  = pcCU->getSlice()->getPPS()->getUseTransformSkip();
  Int        bestModeId[MAX_NUM_COMPONENT] = { 0, 0, 0};
  checkTransformSkip           &= TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(COMPONENT_Y), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize());
  checkTransformSkip           &= (!pcCU->getCUTransquantBypass(0));

  if ( m_pcEncCfg->getUseTransformSkipFast() )
  {
    checkTransformSkip       &= (pcCU->getPartitionSize(uiAbsPartIdx)==SIZE_NxN);
  }

  if( bCheckFull )
  {
    if(checkTransformSkip == true)
    {
      //----- store original entropy coding status -----
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );

      Distortion singleDistTmpLuma                    = 0;
      UInt       singleCbfTmpLuma                     = 0;
      Double     singleCostTmp                        = 0;
      Int        firstCheckId                         = 0;

      for(Int modeId = firstCheckId; modeId < 2; modeId ++)
      {
        DEBUG_STRING_NEW(sModeString)
        Int  default0Save1Load2 = 0;
        singleDistTmpLuma=0;
        if(modeId == firstCheckId)
        {
          default0Save1Load2 = 1;
        }
        else
        {
          default0Save1Load2 = 2;
        }

        if (rTu.ProcessComponentSection(COMPONENT_Y))
        {
          const UInt totalAdjustedDepthChan = rTu.GetTransformDepthTotalAdj(COMPONENT_Y);
          pcCU->setTransformSkipSubParts ( modeId, COMPONENT_Y, uiAbsPartIdx, totalAdjustedDepthChan );

          xIntraCodingTUBlock( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSingle, false, singleDistTmpLuma, COMPONENT_Y, rTu DEBUG_STRING_PASS_INTO(sModeString), default0Save1Load2 );
        }
        singleCbfTmpLuma = pcCU->getCbf( uiAbsPartIdx, COMPONENT_Y, uiTrDepth );

        //----- determine rate and r-d cost -----
        if(modeId == 1 && singleCbfTmpLuma == 0)
        {
          //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
          singleCostTmp = MAX_DOUBLE;
        }
        else
        {
          UInt uiSingleBits = xGetIntraBitsQT( rTu, true, false, false );
          singleCostTmp     = m_pcRdCost->calcRdCost( uiSingleBits, singleDistTmpLuma );
        }
        if(singleCostTmp < dSingleCost)
        {
          DEBUG_STRING_SWAP(sDebug, sModeString)
          dSingleCost   = singleCostTmp;
          uiSingleDistLuma = singleDistTmpLuma;
          uiSingleCbfLuma = singleCbfTmpLuma;

          bestModeId[COMPONENT_Y] = modeId;
          if(bestModeId[COMPONENT_Y] == firstCheckId)
          {
            xStoreIntraResultQT(COMPONENT_Y, rTu );
            m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
          }

          if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
          {
            const Int xOffset = rTu.getRect( COMPONENT_Y ).x0;
            const Int yOffset = rTu.getRect( COMPONENT_Y ).y0;
            for (UInt storedResidualIndex = 0; storedResidualIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; storedResidualIndex++)
            {
              if (bMaintainResidual[storedResidualIndex])
              {
                xStoreCrossComponentPredictionResult(resiLuma[storedResidualIndex], resiLumaSingle[storedResidualIndex], rTu, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE);
              }
            }
          }
        }
        if (modeId == firstCheckId)
        {
          m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
        }
      }

      if (rTu.ProcessComponentSection(COMPONENT_Y))
      {
        const UInt totalAdjustedDepthChan   = rTu.GetTransformDepthTotalAdj(COMPONENT_Y);
        pcCU ->setTransformSkipSubParts ( bestModeId[COMPONENT_Y], COMPONENT_Y, uiAbsPartIdx, totalAdjustedDepthChan );
      }

      if(bestModeId[COMPONENT_Y] == firstCheckId)
      {
        xLoadIntraResultQT(COMPONENT_Y, rTu );
        if (rTu.ProcessComponentSection(COMPONENT_Y))
        {
          pcCU->setCbfSubParts  ( uiSingleCbfLuma << uiTrDepth, COMPONENT_Y, uiAbsPartIdx, rTu.GetTransformDepthTotalAdj(COMPONENT_Y) );
        }

        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
      }
    }
    else
    {
      //----- store original entropy coding status -----
      if( bCheckSplit )
      {
        m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
      }
      //----- code luma/chroma block with given intra prediction mode and store Cbf-----
      dSingleCost   = 0.0;
      if (rTu.ProcessComponentSection(COMPONENT_Y))
      {
        const UInt totalAdjustedDepthChan   = rTu.GetTransformDepthTotalAdj(COMPONENT_Y);
        pcCU ->setTransformSkipSubParts ( 0, COMPONENT_Y, uiAbsPartIdx, totalAdjustedDepthChan );
      }

      xIntraCodingTUBlock( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSingle, false, uiSingleDistLuma, COMPONENT_Y, rTu DEBUG_STRING_PASS_INTO(sDebug));

      if( bCheckSplit )
      {
        uiSingleCbfLuma = pcCU->getCbf( uiAbsPartIdx, COMPONENT_Y, uiTrDepth );
      }
      //----- determine rate and r-d cost -----
      UInt uiSingleBits = xGetIntraBitsQT( rTu, true, false, false );

      if(m_pcEncCfg->getRDpenalty() && (uiLog2TrSize==5) && !isIntraSlice)
      {
        uiSingleBits=uiSingleBits*4;
      }

      dSingleCost       = m_pcRdCost->calcRdCost( uiSingleBits, uiSingleDistLuma );

      if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
      {
        const Int xOffset = rTu.getRect( COMPONENT_Y ).x0;
        const Int yOffset = rTu.getRect( COMPONENT_Y ).y0;
        for (UInt storedResidualIndex = 0; storedResidualIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; storedResidualIndex++)
        {
          if (bMaintainResidual[storedResidualIndex])
          {
            xStoreCrossComponentPredictionResult(resiLuma[storedResidualIndex], resiLumaSingle[storedResidualIndex], rTu, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE);
          }
        }
      }
    }
  }

  if( bCheckSplit )
  {
    //----- store full entropy coding status, load original entropy coding status -----
    if( bCheckFull )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    else
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    //----- code splitted block -----
    Double     dSplitCost      = 0.0;
    Distortion uiSplitDistLuma = 0;
    UInt       uiSplitCbfLuma  = 0;

    TComTURecurse tuRecurseChild(rTu, false);
    DEBUG_STRING_NEW(sSplit)
    do
    {
      DEBUG_STRING_NEW(sChild)
#if HHI_RQT_INTRA_SPEEDUP
      xRecurIntraCodingLumaQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSplit, uiSplitDistLuma, bCheckFirst, dSplitCost, tuRecurseChild DEBUG_STRING_PASS_INTO(sChild) );
#else
      xRecurIntraCodingLumaQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSplit, uiSplitDistLuma, dSplitCost, tuRecurseChild DEBUG_STRING_PASS_INTO(sChild) );
#endif
      DEBUG_STRING_APPEND(sSplit, sChild)
      uiSplitCbfLuma |= pcCU->getCbf( tuRecurseChild.GetAbsPartIdxTU(), COMPONENT_Y, tuRecurseChild.GetTransformDepthRel() );
    } while (tuRecurseChild.nextSection(rTu) );

    UInt    uiPartsDiv     = rTu.GetAbsPartIdxNumParts();
    {
      if (uiSplitCbfLuma)
      {
        const UInt flag=1<<uiTrDepth;
        UChar *pBase=pcCU->getCbf( COMPONENT_Y );
        for( UInt uiOffs = 0; uiOffs < uiPartsDiv; uiOffs++ )
        {
          pBase[ uiAbsPartIdx + uiOffs ] |= flag;
        }
      }
    }
    //----- restore context states -----
    m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    
    //----- determine rate and r-d cost -----
    UInt uiSplitBits = xGetIntraBitsQT( rTu, true, false, false );
    dSplitCost       = m_pcRdCost->calcRdCost( uiSplitBits, uiSplitDistLuma );

    //===== compare and set best =====
    if( dSplitCost < dSingleCost )
    {
      //--- update cost ---
      DEBUG_STRING_SWAP(sSplit, sDebug)
      ruiDistY += uiSplitDistLuma;
      dRDCost  += dSplitCost;

      if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
      {
        const Int xOffset = rTu.getRect( COMPONENT_Y ).x0;
        const Int yOffset = rTu.getRect( COMPONENT_Y ).y0;
        for (UInt storedResidualIndex = 0; storedResidualIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; storedResidualIndex++)
        {
          if (bMaintainResidual[storedResidualIndex])
          {
            xStoreCrossComponentPredictionResult(resiLuma[storedResidualIndex], resiLumaSplit[storedResidualIndex], rTu, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE);
          }
        }
      }

      return;
    }

    //----- set entropy coding status -----
    m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );

    //--- set transform index and Cbf values ---
    pcCU->setTrIdxSubParts( uiTrDepth, uiAbsPartIdx, uiFullDepth );
    const TComRectangle &tuRect=rTu.getRect(COMPONENT_Y);
    const UInt totalAdjustedDepthChan   = rTu.GetTransformDepthTotalAdj(COMPONENT_Y);
    pcCU->setCbfSubParts  ( uiSingleCbfLuma << uiTrDepth, COMPONENT_Y, uiAbsPartIdx, totalAdjustedDepthChan );
    pcCU ->setTransformSkipSubParts  ( bestModeId[COMPONENT_Y], COMPONENT_Y, uiAbsPartIdx, totalAdjustedDepthChan );

    //--- set reconstruction for next intra prediction blocks ---
    const UInt  uiQTLayer   = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    const UInt  uiZOrder    = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;
    const UInt  uiWidth     = tuRect.width;
    const UInt  uiHeight    = tuRect.height;
    Pel*  piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( COMPONENT_Y, uiAbsPartIdx );
    UInt  uiSrcStride = m_pcQTTempTComYuv[ uiQTLayer ].getStride  ( COMPONENT_Y );
    Pel*  piDes       = pcCU->getPic()->getPicYuvRec()->getAddr( COMPONENT_Y, pcCU->getCtuRsAddr(), uiZOrder );
    UInt  uiDesStride = pcCU->getPic()->getPicYuvRec()->getStride  ( COMPONENT_Y );

    for( UInt uiY = 0; uiY < uiHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        piDes[ uiX ] = piSrc[ uiX ];
      }
    }
  }
  ruiDistY += uiSingleDistLuma;
  dRDCost  += dSingleCost;
}


Void
TEncSearch::xSetIntraResultLumaQT(TComYuv* pcRecoYuv, TComTU &rTu)
{
  TComDataCU *pcCU        = rTu.getCU();
  const UInt uiTrDepth    = rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    //===== copy transform coefficients =====

    const TComRectangle &tuRect=rTu.getRect(COMPONENT_Y);
    const UInt coeffOffset = rTu.getCoefficientOffset(COMPONENT_Y);
    const UInt numCoeffInBlock = tuRect.width * tuRect.height;

    if (numCoeffInBlock!=0)
    {
      const TCoeff* srcCoeff = m_ppcQTTempCoeff[COMPONENT_Y][uiQTLayer] + coeffOffset;
      TCoeff* destCoeff      = pcCU->getCoeff(COMPONENT_Y) + coeffOffset;
      ::memcpy( destCoeff, srcCoeff, sizeof(TCoeff)*numCoeffInBlock );
#if ADAPTIVE_QP_SELECTION
      const TCoeff* srcArlCoeff = m_ppcQTTempArlCoeff[COMPONENT_Y][ uiQTLayer ] + coeffOffset;
      TCoeff* destArlCoeff      = pcCU->getArlCoeff (COMPONENT_Y)               + coeffOffset;
      ::memcpy( destArlCoeff, srcArlCoeff, sizeof( TCoeff ) * numCoeffInBlock );
#endif
      m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartComponent( COMPONENT_Y, pcRecoYuv, uiAbsPartIdx, tuRect.width, tuRect.height );
    }

  }
  else
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xSetIntraResultLumaQT( pcRecoYuv, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}


Void
TEncSearch::xStoreIntraResultQT(const ComponentID compID, TComTU &rTu )
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiTrDepth = rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if ( compID==COMPONENT_Y || uiTrMode == uiTrDepth )
  {
    assert(uiTrMode == uiTrDepth);
    const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    const UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    if (rTu.ProcessComponentSection(compID))
    {
      const TComRectangle &tuRect=rTu.getRect(compID);

      //===== copy transform coefficients =====
      const UInt uiNumCoeff    = tuRect.width * tuRect.height;
      TCoeff* pcCoeffSrc = m_ppcQTTempCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
      TCoeff* pcCoeffDst = m_pcQTTempTUCoeff[compID];

      ::memcpy( pcCoeffDst, pcCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#if ADAPTIVE_QP_SELECTION
      TCoeff* pcArlCoeffSrc = m_ppcQTTempArlCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
      TCoeff* pcArlCoeffDst = m_ppcQTTempTUArlCoeff[compID];
      ::memcpy( pcArlCoeffDst, pcArlCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#endif
      //===== copy reconstruction =====
      m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartComponent( compID, &m_pcQTTempTransformSkipTComYuv, uiAbsPartIdx, tuRect.width, tuRect.height );
    }
  }
}


Void
TEncSearch::xLoadIntraResultQT(const ComponentID compID, TComTU &rTu)
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiTrDepth = rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if ( compID==COMPONENT_Y || uiTrMode == uiTrDepth )
  {
    assert(uiTrMode == uiTrDepth);
    const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    const UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    const UInt uiZOrder     = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;

    if (rTu.ProcessComponentSection(compID))
    {
      const TComRectangle &tuRect=rTu.getRect(compID);

      //===== copy transform coefficients =====
      const UInt uiNumCoeff = tuRect.width * tuRect.height;
      TCoeff* pcCoeffDst = m_ppcQTTempCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
      TCoeff* pcCoeffSrc = m_pcQTTempTUCoeff[compID];

      ::memcpy( pcCoeffDst, pcCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#if ADAPTIVE_QP_SELECTION
      TCoeff* pcArlCoeffDst = m_ppcQTTempArlCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
      TCoeff* pcArlCoeffSrc = m_ppcQTTempTUArlCoeff[compID];
      ::memcpy( pcArlCoeffDst, pcArlCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#endif
      //===== copy reconstruction =====
      m_pcQTTempTransformSkipTComYuv.copyPartToPartComponent( compID, &m_pcQTTempTComYuv[ uiQTLayer ], uiAbsPartIdx, tuRect.width, tuRect.height );

      Pel*    piRecIPred        = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
      UInt    uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getStride (compID);
      Pel*    piRecQt           = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
      UInt    uiRecQtStride     = m_pcQTTempTComYuv[ uiQTLayer ].getStride  (compID);
      UInt    uiWidth           = tuRect.width;
      UInt    uiHeight          = tuRect.height;
      Pel* pRecQt               = piRecQt;
      Pel* pRecIPred            = piRecIPred;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pRecIPred[ uiX ] = pRecQt   [ uiX ];
        }
        pRecQt    += uiRecQtStride;
        pRecIPred += uiRecIPredStride;
      }
    }
  }
}

Void
TEncSearch::xStoreCrossComponentPredictionResult(       Pel    *pResiDst,
                                                  const Pel    *pResiSrc,
                                                        TComTU &rTu,
                                                  const Int     xOffset,
                                                  const Int     yOffset,
                                                  const Int     strideDst,
                                                  const Int     strideSrc )
{
  const Pel *pSrc = pResiSrc + yOffset * strideSrc + xOffset;
        Pel *pDst = pResiDst + yOffset * strideDst + xOffset;

  for( Int y = 0; y < rTu.getRect( COMPONENT_Y ).height; y++ )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel) * rTu.getRect( COMPONENT_Y ).width );
    pDst += strideDst;
    pSrc += strideSrc;
  }
}

Char
TEncSearch::xCalcCrossComponentPredictionAlpha(       TComTU &rTu,
                                                const ComponentID compID,
                                                const Pel*        piResiL,
                                                const Pel*        piResiC,
                                                const Int         width,
                                                const Int         height,
                                                const Int         strideL,
                                                const Int         strideC )
{
  const Pel *pResiL = piResiL;
  const Pel *pResiC = piResiC;

        TComDataCU *pCU = rTu.getCU();
  const Int  absPartIdx = rTu.GetAbsPartIdxTU( compID );
  const Int diffBitDepth = pCU->getSlice()->getSPS()->getDifferentialLumaChromaBitDepth();

  Char alpha = 0;
  Int SSxy  = 0;
  Int SSxx  = 0;

  for( UInt uiY = 0; uiY < height; uiY++ )
  {
    for( UInt uiX = 0; uiX < width; uiX++ )
    {
      const Pel scaledResiL = rightShift( pResiL[ uiX ], diffBitDepth );
      SSxy += ( scaledResiL * pResiC[ uiX ] );
      SSxx += ( scaledResiL * scaledResiL   );
    }

    pResiL += strideL;
    pResiC += strideC;
  }

  if( SSxx != 0 )
  {
    Double dAlpha = SSxy / Double( SSxx );
    alpha = Char(Clip3<Int>(-16, 16, (Int)(dAlpha * 16)));

    static const Char alphaQuant[17] = {0, 1, 1, 2, 2, 2, 4, 4, 4, 4, 4, 4, 8, 8, 8, 8, 8};

    alpha = (alpha < 0) ? -alphaQuant[Int(-alpha)] : alphaQuant[Int(alpha)];
  }
  pCU->setCrossComponentPredictionAlphaPartRange( alpha, compID, absPartIdx, rTu.GetAbsPartIdxNumParts( compID ) );

  return alpha;
}

Void
TEncSearch::xRecurIntraChromaCodingQT(TComYuv*    pcOrgYuv,
                                      TComYuv*    pcPredYuv,
                                      TComYuv*    pcResiYuv,
                                      Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                      Distortion& ruiDist,
                                      TComTU&     rTu
                                      DEBUG_STRING_FN_DECLARE(sDebug))
{
  TComDataCU         *pcCU                  = rTu.getCU();
  const UInt          uiTrDepth             = rTu.GetTransformDepthRel();
  const UInt          uiAbsPartIdx          = rTu.GetAbsPartIdxTU();
  const ChromaFormat  format                = rTu.GetChromaFormat();
  UInt                uiTrMode              = pcCU->getTransformIdx( uiAbsPartIdx );
  const UInt          numberValidComponents = getNumberValidComponents(format);

  if(  uiTrMode == uiTrDepth )
  {
    if (!rTu.ProcessChannelSection(CHANNEL_TYPE_CHROMA))
    {
      return;
    }

    const UInt uiFullDepth = rTu.GetTransformDepthTotal();

    Bool checkTransformSkip = pcCU->getSlice()->getPPS()->getUseTransformSkip();
    checkTransformSkip &= TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(COMPONENT_Cb), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize());

    if ( m_pcEncCfg->getUseTransformSkipFast() )
    {
      checkTransformSkip &= TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(COMPONENT_Y), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize());

      if (checkTransformSkip)
      {
        Int nbLumaSkip = 0;
        const UInt maxAbsPartIdxSub=uiAbsPartIdx + (rTu.ProcessingAllQuadrants(COMPONENT_Cb)?1:4);
        for(UInt absPartIdxSub = uiAbsPartIdx; absPartIdxSub < maxAbsPartIdxSub; absPartIdxSub ++)
        {
          nbLumaSkip += pcCU->getTransformSkip(absPartIdxSub, COMPONENT_Y);
        }
        checkTransformSkip &= (nbLumaSkip > 0);
      }
    }


    for (UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
    {
      const ComponentID compID = ComponentID(ch);
      DEBUG_STRING_NEW(sDebugBestMode)

      //use RDO to decide whether Cr/Cb takes TS
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiFullDepth][CI_QT_TRAFO_ROOT] );

      const Bool splitIntoSubTUs = rTu.getRect(compID).width != rTu.getRect(compID).height;

      TComTURecurse TUIterator(rTu, false, (splitIntoSubTUs ? TComTU::VERTICAL_SPLIT : TComTU::DONT_SPLIT), true, compID);

      const UInt partIdxesPerSubTU = TUIterator.GetAbsPartIdxNumParts(compID);

      do
      {
        const UInt subTUAbsPartIdx   = TUIterator.GetAbsPartIdxTU(compID);

        Double     dSingleCost               = MAX_DOUBLE;
        Int        bestModeId                = 0;
        Distortion singleDistC               = 0;
        UInt       singleCbfC                = 0;
        Distortion singleDistCTmp            = 0;
        Double     singleCostTmp             = 0;
        UInt       singleCbfCTmp             = 0;
        Char       bestCrossCPredictionAlpha = 0;
        Int        bestTransformSkipMode     = 0;

        const Bool checkCrossComponentPrediction =    (pcCU->getIntraDir(CHANNEL_TYPE_CHROMA, subTUAbsPartIdx) == DM_CHROMA_IDX)
                                                   &&  pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag()
                                                   && (pcCU->getCbf(subTUAbsPartIdx,  COMPONENT_Y, uiTrDepth) != 0);

        const Int  crossCPredictionModesToTest = checkCrossComponentPrediction ? 2 : 1;
        const Int  transformSkipModesToTest    = checkTransformSkip            ? 2 : 1;
        const Int  totalModesToTest            = crossCPredictionModesToTest * transformSkipModesToTest;
              Int  currModeId                  = 0;
              Int  default0Save1Load2          = 0;

        for(Int transformSkipModeId = 0; transformSkipModeId < transformSkipModesToTest; transformSkipModeId++)
        {
          for(Int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++)
          {
            pcCU->setCrossComponentPredictionAlphaPartRange(0, compID, subTUAbsPartIdx, partIdxesPerSubTU);
            DEBUG_STRING_NEW(sDebugMode)
            pcCU->setTransformSkipPartRange( transformSkipModeId, compID, subTUAbsPartIdx, partIdxesPerSubTU );
            currModeId++;

            const Bool isOneMode  = (totalModesToTest == 1);
            const Bool isLastMode = (currModeId == totalModesToTest); // currModeId is indexed from 1

            if (isOneMode)
            {
              default0Save1Load2 = 0;
            }
            else if (!isOneMode && (transformSkipModeId == 0) && (crossCPredictionModeId == 0))
            {
              default0Save1Load2 = 1; //save prediction on first mode
            }
            else
            {
              default0Save1Load2 = 2; //load it on subsequent modes
            }

            singleDistCTmp = 0;

            xIntraCodingTUBlock( pcOrgYuv, pcPredYuv, pcResiYuv, resiLuma, (crossCPredictionModeId != 0), singleDistCTmp, compID, TUIterator DEBUG_STRING_PASS_INTO(sDebugMode), default0Save1Load2);
            singleCbfCTmp = pcCU->getCbf( subTUAbsPartIdx, compID, uiTrDepth);

            if (  ((crossCPredictionModeId == 1) && (pcCU->getCrossComponentPredictionAlpha(subTUAbsPartIdx, compID) == 0))
               || ((transformSkipModeId    == 1) && (singleCbfCTmp == 0))) //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
            {
              singleCostTmp = MAX_DOUBLE;
            }
            else if (!isOneMode)
            {
              UInt bitsTmp = xGetIntraBitsQTChroma( TUIterator, compID, false );
              singleCostTmp  = m_pcRdCost->calcRdCost( bitsTmp, singleDistCTmp);
            }

            if(singleCostTmp < dSingleCost)
            {
              DEBUG_STRING_SWAP(sDebugBestMode, sDebugMode)
              dSingleCost               = singleCostTmp;
              singleDistC               = singleDistCTmp;
              bestCrossCPredictionAlpha = (crossCPredictionModeId != 0) ? pcCU->getCrossComponentPredictionAlpha(subTUAbsPartIdx, compID) : 0;
              bestTransformSkipMode     = transformSkipModeId;
              bestModeId                = currModeId;
              singleCbfC                = singleCbfCTmp;

              if (!isOneMode && !isLastMode)
              {
                xStoreIntraResultQT(compID, TUIterator);
                m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
              }
            }

            if (!isOneMode && !isLastMode)
            {
              m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
            }
          }
        }

        if(bestModeId < totalModesToTest)
        {
          xLoadIntraResultQT(compID, TUIterator);
          pcCU->setCbfPartRange( singleCbfC << uiTrDepth, compID, subTUAbsPartIdx, partIdxesPerSubTU );

          m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
        }

        DEBUG_STRING_APPEND(sDebug, sDebugBestMode)
        pcCU ->setTransformSkipPartRange                ( bestTransformSkipMode,     compID, subTUAbsPartIdx, partIdxesPerSubTU );
        pcCU ->setCrossComponentPredictionAlphaPartRange( bestCrossCPredictionAlpha, compID, subTUAbsPartIdx, partIdxesPerSubTU );
        ruiDist += singleDistC;
      } while (TUIterator.nextSection(rTu));

      if (splitIntoSubTUs)
      {
        offsetSubTUCBFs(rTu, compID);
      }
    }
  }
  else
  {
    UInt    uiSplitCbf[MAX_NUM_COMPONENT] = {0,0,0};

    TComTURecurse tuRecurseChild(rTu, false);
    const UInt uiTrDepthChild   = tuRecurseChild.GetTransformDepthRel();
    do
    {
      DEBUG_STRING_NEW(sChild)

      xRecurIntraChromaCodingQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLuma, ruiDist, tuRecurseChild DEBUG_STRING_PASS_INTO(sChild) );

      DEBUG_STRING_APPEND(sDebug, sChild)
      const UInt uiAbsPartIdxSub=tuRecurseChild.GetAbsPartIdxTU();

      for(UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
      {
        uiSplitCbf[ch] |= pcCU->getCbf( uiAbsPartIdxSub, ComponentID(ch), uiTrDepthChild );
      }
    } while ( tuRecurseChild.nextSection(rTu) );


    UInt uiPartsDiv = rTu.GetAbsPartIdxNumParts();
    for(UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
    {
      if (uiSplitCbf[ch])
      {
        const UInt flag=1<<uiTrDepth;
        ComponentID compID=ComponentID(ch);
        UChar *pBase=pcCU->getCbf( compID );
        for( UInt uiOffs = 0; uiOffs < uiPartsDiv; uiOffs++ )
        {
          pBase[ uiAbsPartIdx + uiOffs ] |= flag;
        }
      }
    }
  }
}




Void
TEncSearch::xSetIntraResultChromaQT(TComYuv*    pcRecoYuv, TComTU &rTu)
{
  if (!rTu.ProcessChannelSection(CHANNEL_TYPE_CHROMA))
  {
    return;
  }
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth   = rTu.GetTransformDepthRel();
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    //===== copy transform coefficients =====
    const TComRectangle &tuRectCb=rTu.getRect(COMPONENT_Cb);
    UInt uiNumCoeffC    = tuRectCb.width*tuRectCb.height;//( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( uiFullDepth << 1 );
    const UInt offset = rTu.getCoefficientOffset(COMPONENT_Cb);

    const UInt numberValidComponents = getNumberValidComponents(rTu.GetChromaFormat());
    for (UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
    {
      const ComponentID component = ComponentID(ch);
      const TCoeff* src           = m_ppcQTTempCoeff[component][uiQTLayer] + offset;//(uiNumCoeffIncC*uiAbsPartIdx);
      TCoeff* dest                = pcCU->getCoeff(component) + offset;//(uiNumCoeffIncC*uiAbsPartIdx);
      ::memcpy( dest, src, sizeof(TCoeff)*uiNumCoeffC );
#if ADAPTIVE_QP_SELECTION
      TCoeff* pcArlCoeffSrc = m_ppcQTTempArlCoeff[component][ uiQTLayer ] + offset;//( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcArlCoeffDst = pcCU->getArlCoeff(component)                + offset;//( uiNumCoeffIncC * uiAbsPartIdx );
      ::memcpy( pcArlCoeffDst, pcArlCoeffSrc, sizeof( TCoeff ) * uiNumCoeffC );
#endif
    }

    //===== copy reconstruction =====

    m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartComponent( COMPONENT_Cb, pcRecoYuv, uiAbsPartIdx, tuRectCb.width, tuRectCb.height );
    m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartComponent( COMPONENT_Cr, pcRecoYuv, uiAbsPartIdx, tuRectCb.width, tuRectCb.height );
  }
  else
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xSetIntraResultChromaQT( pcRecoYuv, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}



Void
TEncSearch::estIntraPredLumaQT(TComDataCU* pcCU,
                               TComYuv*    pcOrgYuv,
                               TComYuv*    pcPredYuv,
                               TComYuv*    pcResiYuv,
                               TComYuv*    pcRecoYuv,
                               Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE]
                               DEBUG_STRING_FN_DECLARE(sDebug))
{
  const UInt         uiDepth               = pcCU->getDepth(0);
  const UInt         uiInitTrDepth         = pcCU->getPartitionSize(0) == SIZE_2Nx2N ? 0 : 1;
  const UInt         uiNumPU               = 1<<(2*uiInitTrDepth);
  const UInt         uiQNumParts           = pcCU->getTotalNumPart() >> 2;
  const UInt         uiWidthBit            = pcCU->getIntraSizeIdx(0);
  const ChromaFormat chFmt                 = pcCU->getPic()->getChromaFormat();
  const UInt         numberValidComponents = getNumberValidComponents(chFmt);
  const TComSPS     &sps                   = *(pcCU->getSlice()->getSPS());
  const TComPPS     &pps                   = *(pcCU->getSlice()->getPPS());
        Distortion   uiOverallDistY        = 0;
        UInt         CandNum;
        Double       CandCostList[ FAST_UDI_MAX_RDMODE_NUM ];
        Pel          resiLumaPU[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE];

        Bool    bMaintainResidual[NUMBER_OF_STORED_RESIDUAL_TYPES];
        for (UInt residualTypeIndex = 0; residualTypeIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; residualTypeIndex++)
        {
          bMaintainResidual[residualTypeIndex] = true; //assume true unless specified otherwise
        }

        bMaintainResidual[RESIDUAL_ENCODER_SIDE] = !(m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate());

  // Lambda calculation at equivalent Qp of 4 is recommended because at that Qp, the quantisation divisor is 1.
#if FULL_NBIT
  const Double sqrtLambdaForFirstPass= (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING && pcCU->getCUTransquantBypass(0)) ?
                sqrt(0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12) / 3.0)))
              : m_pcRdCost->getSqrtLambda();
#else
  const Double sqrtLambdaForFirstPass= (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING && pcCU->getCUTransquantBypass(0)) ?
                sqrt(0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12 - 6 * (sps.getBitDepth(CHANNEL_TYPE_LUMA) - 8)) / 3.0)))
              : m_pcRdCost->getSqrtLambda();
#endif

  //===== set QP and clear Cbf =====
  if ( pps.getUseDQP() == true)
  {
    pcCU->setQPSubParts( pcCU->getQP(0), 0, uiDepth );
  }
  else
  {
    pcCU->setQPSubParts( pcCU->getSlice()->getSliceQp(), 0, uiDepth );
  }

  //===== loop over partitions =====
  TComTURecurse tuRecurseCU(pcCU, 0);
  TComTURecurse tuRecurseWithPU(tuRecurseCU, false, (uiInitTrDepth==0)?TComTU::DONT_SPLIT : TComTU::QUAD_SPLIT);

  do
  {
    const UInt uiPartOffset=tuRecurseWithPU.GetAbsPartIdxTU();
//  for( UInt uiPU = 0, uiPartOffset=0; uiPU < uiNumPU; uiPU++, uiPartOffset += uiQNumParts )
  //{
    //===== init pattern for luma prediction =====
    Bool bAboveAvail = false;
    Bool bLeftAvail  = false;
    DEBUG_STRING_NEW(sTemp2)

    //===== determine set of modes to be tested (using prediction signal only) =====
    Int numModesAvailable     = 35; //total number of Intra modes
    UInt uiRdModeList[FAST_UDI_MAX_RDMODE_NUM];
    Int numModesForFullRD = m_pcEncCfg->getFastUDIUseMPMEnabled()?g_aucIntraModeNumFast_UseMPM[ uiWidthBit ] : g_aucIntraModeNumFast_NotUseMPM[ uiWidthBit ];

    if (tuRecurseWithPU.ProcessComponentSection(COMPONENT_Y))
    {
      initIntraPatternChType( tuRecurseWithPU, bAboveAvail, bLeftAvail, COMPONENT_Y, true DEBUG_STRING_PASS_INTO(sTemp2) );
    }

    Bool doFastSearch = (numModesForFullRD != numModesAvailable);
    if (doFastSearch)
    {
      assert(numModesForFullRD < numModesAvailable);

      for( Int i=0; i < numModesForFullRD; i++ )
      {
        CandCostList[ i ] = MAX_DOUBLE;
      }
      CandNum = 0;

      const TComRectangle &puRect=tuRecurseWithPU.getRect(COMPONENT_Y);
      const UInt uiAbsPartIdx=tuRecurseWithPU.GetAbsPartIdxTU();

      Pel* piOrg         = pcOrgYuv ->getAddr( COMPONENT_Y, uiAbsPartIdx );
      Pel* piPred        = pcPredYuv->getAddr( COMPONENT_Y, uiAbsPartIdx );
      UInt uiStride      = pcPredYuv->getStride( COMPONENT_Y );
      DistParam distParam;
      const Bool bUseHadamard=pcCU->getCUTransquantBypass(0) == 0;
      m_pcRdCost->setDistParam(distParam, sps.getBitDepth(CHANNEL_TYPE_LUMA), piOrg, uiStride, piPred, uiStride, puRect.width, puRect.height, bUseHadamard);
      distParam.bApplyWeight = false;
      for( Int modeIdx = 0; modeIdx < numModesAvailable; modeIdx++ )
      {
        UInt       uiMode = modeIdx;
        Distortion uiSad  = 0;

        const Bool bUseFilter=TComPrediction::filteringIntraReferenceSamples(COMPONENT_Y, uiMode, puRect.width, puRect.height, chFmt, sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag());

        predIntraAng( COMPONENT_Y, uiMode, piOrg, uiStride, piPred, uiStride, tuRecurseWithPU, bAboveAvail, bLeftAvail, bUseFilter, TComPrediction::UseDPCMForFirstPassIntraEstimation(tuRecurseWithPU, uiMode) );

        // use hadamard transform here
        uiSad+=distParam.DistFunc(&distParam);

        UInt   iModeBits = 0;

        // NB xModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
        iModeBits+=xModeBitsIntra( pcCU, uiMode, uiPartOffset, uiDepth, CHANNEL_TYPE_LUMA );

        Double cost      = (Double)uiSad + (Double)iModeBits * sqrtLambdaForFirstPass;

#if DEBUG_INTRA_SEARCH_COSTS
        std::cout << "1st pass mode " << uiMode << " SAD = " << uiSad << ", mode bits = " << iModeBits << ", cost = " << cost << "\n";
#endif

        CandNum += xUpdateCandList( uiMode, cost, numModesForFullRD, uiRdModeList, CandCostList );
      }

      if (m_pcEncCfg->getFastUDIUseMPMEnabled())
      {
        Int uiPreds[NUM_MOST_PROBABLE_MODES] = {-1, -1, -1};

        Int iMode = -1;
        pcCU->getIntraDirPredictor( uiPartOffset, uiPreds, COMPONENT_Y, &iMode );

        const Int numCand = ( iMode >= 0 ) ? iMode : Int(NUM_MOST_PROBABLE_MODES);

        for( Int j=0; j < numCand; j++)
        {
          Bool mostProbableModeIncluded = false;
          Int mostProbableMode = uiPreds[j];

          for( Int i=0; i < numModesForFullRD; i++)
          {
            mostProbableModeIncluded |= (mostProbableMode == uiRdModeList[i]);
          }
          if (!mostProbableModeIncluded)
          {
            uiRdModeList[numModesForFullRD++] = mostProbableMode;
          }
        }
      }
    }
    else
    {
      for( Int i=0; i < numModesForFullRD; i++)
      {
        uiRdModeList[i] = i;
      }
    }

    //===== check modes (using r-d costs) =====
#if HHI_RQT_INTRA_SPEEDUP_MOD
    UInt   uiSecondBestMode  = MAX_UINT;
    Double dSecondBestPUCost = MAX_DOUBLE;
#endif
    DEBUG_STRING_NEW(sPU)
    UInt       uiBestPUMode  = 0;
    Distortion uiBestPUDistY = 0;
    Double     dBestPUCost   = MAX_DOUBLE;

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
    UInt max=numModesForFullRD;

    if (DebugOptionList::ForceLumaMode.isSet())
    {
      max=0;  // we are forcing a direction, so don't bother with mode check
    }
    for ( UInt uiMode = 0; uiMode < max; uiMode++)
#else
    for( UInt uiMode = 0; uiMode < numModesForFullRD; uiMode++ )
#endif
    {
      // set luma prediction mode
      UInt uiOrgMode = uiRdModeList[uiMode];

      pcCU->setIntraDirSubParts ( CHANNEL_TYPE_LUMA, uiOrgMode, uiPartOffset, uiDepth + uiInitTrDepth );

      DEBUG_STRING_NEW(sMode)
      // set context models
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );

      // determine residual for partition
      Distortion uiPUDistY = 0;
      Double     dPUCost   = 0.0;
#if HHI_RQT_INTRA_SPEEDUP
      xRecurIntraCodingLumaQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaPU, uiPUDistY, true, dPUCost, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sMode) );
#else
      xRecurIntraCodingLumaQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaPU, uiPUDistY, dPUCost, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sMode) );
#endif

#if DEBUG_INTRA_SEARCH_COSTS
      std::cout << "2nd pass [luma,chroma] mode [" << Int(pcCU->getIntraDir(CHANNEL_TYPE_LUMA, uiPartOffset)) << "," << Int(pcCU->getIntraDir(CHANNEL_TYPE_CHROMA, uiPartOffset)) << "] cost = " << dPUCost << "\n";
#endif

      // check r-d cost
      if( dPUCost < dBestPUCost )
      {
        DEBUG_STRING_SWAP(sPU, sMode)
#if HHI_RQT_INTRA_SPEEDUP_MOD
        uiSecondBestMode  = uiBestPUMode;
        dSecondBestPUCost = dBestPUCost;
#endif
        uiBestPUMode  = uiOrgMode;
        uiBestPUDistY = uiPUDistY;
        dBestPUCost   = dPUCost;

        xSetIntraResultLumaQT( pcRecoYuv, tuRecurseWithPU );

        if (pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
        {
          const Int xOffset = tuRecurseWithPU.getRect( COMPONENT_Y ).x0;
          const Int yOffset = tuRecurseWithPU.getRect( COMPONENT_Y ).y0;
          for (UInt storedResidualIndex = 0; storedResidualIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; storedResidualIndex++)
          {
            if (bMaintainResidual[storedResidualIndex])
            {
              xStoreCrossComponentPredictionResult(resiLuma[storedResidualIndex], resiLumaPU[storedResidualIndex], tuRecurseWithPU, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE );
            }
          }
        }

        UInt uiQPartNum = tuRecurseWithPU.GetAbsPartIdxNumParts();

        ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx()       + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        for (UInt component = 0; component < numberValidComponents; component++)
        {
          const ComponentID compID = ComponentID(component);
          ::memcpy( m_puhQTTempCbf[compID], pcCU->getCbf( compID  ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_puhQTTempTransformSkipFlag[compID],  pcCU->getTransformSkip(compID)  + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        }
      }
#if HHI_RQT_INTRA_SPEEDUP_MOD
      else if( dPUCost < dSecondBestPUCost )
      {
        uiSecondBestMode  = uiOrgMode;
        dSecondBestPUCost = dPUCost;
      }
#endif
    } // Mode loop

#if HHI_RQT_INTRA_SPEEDUP
#if HHI_RQT_INTRA_SPEEDUP_MOD
    for( UInt ui =0; ui < 2; ++ui )
#endif
    {
#if HHI_RQT_INTRA_SPEEDUP_MOD
      UInt uiOrgMode   = ui ? uiSecondBestMode  : uiBestPUMode;
      if( uiOrgMode == MAX_UINT )
      {
        break;
      }
#else
      UInt uiOrgMode = uiBestPUMode;
#endif

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      if (DebugOptionList::ForceLumaMode.isSet())
      {
        uiOrgMode = DebugOptionList::ForceLumaMode.getInt();
      }
#endif

      pcCU->setIntraDirSubParts ( CHANNEL_TYPE_LUMA, uiOrgMode, uiPartOffset, uiDepth + uiInitTrDepth );
      DEBUG_STRING_NEW(sModeTree)

      // set context models
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );

      // determine residual for partition
      Distortion uiPUDistY = 0;
      Double     dPUCost   = 0.0;

      xRecurIntraCodingLumaQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaPU, uiPUDistY, false, dPUCost, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sModeTree));

      // check r-d cost
      if( dPUCost < dBestPUCost )
      {
        DEBUG_STRING_SWAP(sPU, sModeTree)
        uiBestPUMode  = uiOrgMode;
        uiBestPUDistY = uiPUDistY;
        dBestPUCost   = dPUCost;

        xSetIntraResultLumaQT( pcRecoYuv, tuRecurseWithPU );

        if (pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
        {
          const Int xOffset = tuRecurseWithPU.getRect( COMPONENT_Y ).x0;
          const Int yOffset = tuRecurseWithPU.getRect( COMPONENT_Y ).y0;
          for (UInt storedResidualIndex = 0; storedResidualIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; storedResidualIndex++)
          {
            if (bMaintainResidual[storedResidualIndex])
            {
              xStoreCrossComponentPredictionResult(resiLuma[storedResidualIndex], resiLumaPU[storedResidualIndex], tuRecurseWithPU, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE );
            }
          }
        }

        const UInt uiQPartNum = tuRecurseWithPU.GetAbsPartIdxNumParts();
        ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx()       + uiPartOffset, uiQPartNum * sizeof( UChar ) );

        for (UInt component = 0; component < numberValidComponents; component++)
        {
          const ComponentID compID = ComponentID(component);
          ::memcpy( m_puhQTTempCbf[compID], pcCU->getCbf( compID  ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_puhQTTempTransformSkipFlag[compID],  pcCU->getTransformSkip(compID)  + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        }
      }
    } // Mode loop
#endif

    DEBUG_STRING_APPEND(sDebug, sPU)

    //--- update overall distortion ---
    uiOverallDistY += uiBestPUDistY;

    //--- update transform index and cbf ---
    const UInt uiQPartNum = tuRecurseWithPU.GetAbsPartIdxNumParts();
    ::memcpy( pcCU->getTransformIdx()       + uiPartOffset, m_puhQTTempTrIdx,  uiQPartNum * sizeof( UChar ) );
    for (UInt component = 0; component < numberValidComponents; component++)
    {
      const ComponentID compID = ComponentID(component);
      ::memcpy( pcCU->getCbf( compID  ) + uiPartOffset, m_puhQTTempCbf[compID], uiQPartNum * sizeof( UChar ) );
      ::memcpy( pcCU->getTransformSkip( compID  ) + uiPartOffset, m_puhQTTempTransformSkipFlag[compID ], uiQPartNum * sizeof( UChar ) );
    }

    //--- set reconstruction for next intra prediction blocks ---
    if( !tuRecurseWithPU.IsLastSection() )
    {
      const TComRectangle &puRect=tuRecurseWithPU.getRect(COMPONENT_Y);
      const UInt  uiCompWidth   = puRect.width;
      const UInt  uiCompHeight  = puRect.height;

      const UInt  uiZOrder      = pcCU->getZorderIdxInCtu() + uiPartOffset;
            Pel*  piDes         = pcCU->getPic()->getPicYuvRec()->getAddr( COMPONENT_Y, pcCU->getCtuRsAddr(), uiZOrder );
      const UInt  uiDesStride   = pcCU->getPic()->getPicYuvRec()->getStride( COMPONENT_Y);
      const Pel*  piSrc         = pcRecoYuv->getAddr( COMPONENT_Y, uiPartOffset );
      const UInt  uiSrcStride   = pcRecoYuv->getStride( COMPONENT_Y);

      for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
      {
        for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
        {
          piDes[ uiX ] = piSrc[ uiX ];
        }
      }
    }

    //=== update PU data ====
    pcCU->setIntraDirSubParts     ( CHANNEL_TYPE_LUMA, uiBestPUMode, uiPartOffset, uiDepth + uiInitTrDepth );
  } while (tuRecurseWithPU.nextSection(tuRecurseCU));


  if( uiNumPU > 1 )
  { // set Cbf for all blocks
    UInt uiCombCbfY = 0;
    UInt uiCombCbfU = 0;
    UInt uiCombCbfV = 0;
    UInt uiPartIdx  = 0;
    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiPartIdx += uiQNumParts )
    {
      uiCombCbfY |= pcCU->getCbf( uiPartIdx, COMPONENT_Y,  1 );
      uiCombCbfU |= pcCU->getCbf( uiPartIdx, COMPONENT_Cb, 1 );
      uiCombCbfV |= pcCU->getCbf( uiPartIdx, COMPONENT_Cr, 1 );
    }
    for( UInt uiOffs = 0; uiOffs < 4 * uiQNumParts; uiOffs++ )
    {
      pcCU->getCbf( COMPONENT_Y  )[ uiOffs ] |= uiCombCbfY;
      pcCU->getCbf( COMPONENT_Cb )[ uiOffs ] |= uiCombCbfU;
      pcCU->getCbf( COMPONENT_Cr )[ uiOffs ] |= uiCombCbfV;
    }
  }

  //===== reset context models =====
  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);

  //===== set distortion (rate and r-d costs are determined later) =====
  pcCU->getTotalDistortion() = uiOverallDistY;
}




Void
TEncSearch::estIntraPredChromaQT(TComDataCU* pcCU,
                                 TComYuv*    pcOrgYuv,
                                 TComYuv*    pcPredYuv,
                                 TComYuv*    pcResiYuv,
                                 TComYuv*    pcRecoYuv,
                                 Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE]
                                 DEBUG_STRING_FN_DECLARE(sDebug))
{
  const UInt    uiInitTrDepth  = pcCU->getPartitionSize(0) != SIZE_2Nx2N && enable4ChromaPUsInIntraNxNCU(pcOrgYuv->getChromaFormat()) ? 1 : 0;

  TComTURecurse tuRecurseCU(pcCU, 0);
  TComTURecurse tuRecurseWithPU(tuRecurseCU, false, (uiInitTrDepth==0)?TComTU::DONT_SPLIT : TComTU::QUAD_SPLIT);
  const UInt    uiQNumParts    = tuRecurseWithPU.GetAbsPartIdxNumParts();
  const UInt    uiDepthCU=tuRecurseWithPU.getCUDepth();
  const UInt    numberValidComponents = pcCU->getPic()->getNumberValidComponents();

  do
  {
    UInt       uiBestMode  = 0;
    Distortion uiBestDist  = 0;
    Double     dBestCost   = MAX_DOUBLE;

    //----- init mode list -----
    if (tuRecurseWithPU.ProcessChannelSection(CHANNEL_TYPE_CHROMA))
    {
      UInt uiModeList[FAST_UDI_MAX_RDMODE_NUM];
      const UInt  uiQPartNum     = uiQNumParts;
      const UInt  uiPartOffset   = tuRecurseWithPU.GetAbsPartIdxTU();
      {
        UInt  uiMinMode = 0;
        UInt  uiMaxMode = NUM_CHROMA_MODE;

        //----- check chroma modes -----
        pcCU->getAllowedChromaDir( uiPartOffset, uiModeList );

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
        if (DebugOptionList::ForceChromaMode.isSet())
        {
          uiMinMode=DebugOptionList::ForceChromaMode.getInt();
          if (uiModeList[uiMinMode]==34)
          {
            uiMinMode=4; // if the fixed mode has been renumbered because DM_CHROMA covers it, use DM_CHROMA.
          }
          uiMaxMode=uiMinMode+1;
        }
#endif

        DEBUG_STRING_NEW(sPU)

        for( UInt uiMode = uiMinMode; uiMode < uiMaxMode; uiMode++ )
        {
          //----- restore context models -----
          m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepthCU][CI_CURR_BEST] );
          
          DEBUG_STRING_NEW(sMode)
          //----- chroma coding -----
          Distortion uiDist = 0;
          pcCU->setIntraDirSubParts  ( CHANNEL_TYPE_CHROMA, uiModeList[uiMode], uiPartOffset, uiDepthCU+uiInitTrDepth );
          xRecurIntraChromaCodingQT       ( pcOrgYuv, pcPredYuv, pcResiYuv, resiLuma, uiDist, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sMode) );

          if( pcCU->getSlice()->getPPS()->getUseTransformSkip() )
          {
            m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepthCU][CI_CURR_BEST] );
          }

          UInt    uiBits = xGetIntraBitsQT( tuRecurseWithPU, false, true, false );
          Double  dCost  = m_pcRdCost->calcRdCost( uiBits, uiDist );

          //----- compare -----
          if( dCost < dBestCost )
          {
            DEBUG_STRING_SWAP(sPU, sMode);
            dBestCost   = dCost;
            uiBestDist  = uiDist;
            uiBestMode  = uiModeList[uiMode];

            xSetIntraResultChromaQT( pcRecoYuv, tuRecurseWithPU );
            for (UInt componentIndex = COMPONENT_Cb; componentIndex < numberValidComponents; componentIndex++)
            {
              const ComponentID compID = ComponentID(componentIndex);
              ::memcpy( m_puhQTTempCbf[compID], pcCU->getCbf( compID )+uiPartOffset, uiQPartNum * sizeof( UChar ) );
              ::memcpy( m_puhQTTempTransformSkipFlag[compID], pcCU->getTransformSkip( compID )+uiPartOffset, uiQPartNum * sizeof( UChar ) );
              ::memcpy( m_phQTTempCrossComponentPredictionAlpha[compID], pcCU->getCrossComponentPredictionAlpha(compID)+uiPartOffset, uiQPartNum * sizeof( Char ) );
            }
          }
        }

        DEBUG_STRING_APPEND(sDebug, sPU)

        //----- set data -----
        for (UInt componentIndex = COMPONENT_Cb; componentIndex < numberValidComponents; componentIndex++)
        {
          const ComponentID compID = ComponentID(componentIndex);
          ::memcpy( pcCU->getCbf( compID )+uiPartOffset, m_puhQTTempCbf[compID], uiQPartNum * sizeof( UChar ) );
          ::memcpy( pcCU->getTransformSkip( compID )+uiPartOffset, m_puhQTTempTransformSkipFlag[compID], uiQPartNum * sizeof( UChar ) );
          ::memcpy( pcCU->getCrossComponentPredictionAlpha(compID)+uiPartOffset, m_phQTTempCrossComponentPredictionAlpha[compID], uiQPartNum * sizeof( Char ) );
        }
      }

      if( ! tuRecurseWithPU.IsLastSection() )
      {
        for (UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
        {
          const ComponentID compID    = ComponentID(ch);
          const TComRectangle &tuRect = tuRecurseWithPU.getRect(compID);
          const UInt  uiCompWidth     = tuRect.width;
          const UInt  uiCompHeight    = tuRect.height;
          const UInt  uiZOrder        = pcCU->getZorderIdxInCtu() + tuRecurseWithPU.GetAbsPartIdxTU();
                Pel*  piDes           = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
          const UInt  uiDesStride     = pcCU->getPic()->getPicYuvRec()->getStride( compID);
          const Pel*  piSrc           = pcRecoYuv->getAddr( compID, uiPartOffset );
          const UInt  uiSrcStride     = pcRecoYuv->getStride( compID);

          for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
          {
            for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
            {
              piDes[ uiX ] = piSrc[ uiX ];
            }
          }
        }
      }

      pcCU->setIntraDirSubParts( CHANNEL_TYPE_CHROMA, uiBestMode, uiPartOffset, uiDepthCU+uiInitTrDepth );
      pcCU->getTotalDistortion      () += uiBestDist;
    }

  } while (tuRecurseWithPU.nextSection(tuRecurseCU));

  //----- restore context models -----

  if( uiInitTrDepth != 0 )
  { // set Cbf for all blocks
    UInt uiCombCbfU = 0;
    UInt uiCombCbfV = 0;
    UInt uiPartIdx  = 0;
    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiPartIdx += uiQNumParts )
    {
      uiCombCbfU |= pcCU->getCbf( uiPartIdx, COMPONENT_Cb, 1 );
      uiCombCbfV |= pcCU->getCbf( uiPartIdx, COMPONENT_Cr, 1 );
    }
    for( UInt uiOffs = 0; uiOffs < 4 * uiQNumParts; uiOffs++ )
    {
      pcCU->getCbf( COMPONENT_Cb )[ uiOffs ] |= uiCombCbfU;
      pcCU->getCbf( COMPONENT_Cr )[ uiOffs ] |= uiCombCbfV;
    }
  }

  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepthCU][CI_CURR_BEST] );
}




/** Function for encoding and reconstructing luma/chroma samples of a PCM mode CU.
 * \param pcCU pointer to current CU
 * \param uiAbsPartIdx part index
 * \param pOrg pointer to original sample arrays
 * \param pPCM pointer to PCM code arrays
 * \param pPred pointer to prediction signal arrays
 * \param pResi pointer to residual signal arrays
 * \param pReco pointer to reconstructed sample arrays
 * \param uiStride stride of the original/prediction/residual sample arrays
 * \param uiWidth block width
 * \param uiHeight block height
 * \param compID texture component type
 */
Void TEncSearch::xEncPCM (TComDataCU* pcCU, UInt uiAbsPartIdx, Pel* pOrg, Pel* pPCM, Pel* pPred, Pel* pResi, Pel* pReco, UInt uiStride, UInt uiWidth, UInt uiHeight, const ComponentID compID )
{
  const UInt uiReconStride   = pcCU->getPic()->getPicYuvRec()->getStride(compID);
  const UInt uiPCMBitDepth   = pcCU->getSlice()->getSPS()->getPCMBitDepth(toChannelType(compID));
  const Int  channelBitDepth = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
  Pel* pRecoPic = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiAbsPartIdx);

  const Int pcmShiftRight=(channelBitDepth - Int(uiPCMBitDepth));

  assert(pcmShiftRight >= 0);

  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      // Reset pred and residual
      pPred[uiX] = 0;
      pResi[uiX] = 0;
      // Encode
      pPCM[uiX] = (pOrg[uiX]>>pcmShiftRight);
      // Reconstruction
      pReco   [uiX] = (pPCM[uiX]<<(pcmShiftRight));
      pRecoPic[uiX] = pReco[uiX];
    }
    pPred += uiStride;
    pResi += uiStride;
    pPCM += uiWidth;
    pOrg += uiStride;
    pReco += uiStride;
    pRecoPic += uiReconStride;
  }
}


//!  Function for PCM mode estimation.
Void TEncSearch::IPCMSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv, TComYuv* pcRecoYuv )
{
  UInt        uiDepth      = pcCU->getDepth(0);
  const UInt  uiDistortion = 0;
  UInt        uiBits;

  Double dCost;

  for (UInt ch=0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
  {
    const ComponentID compID  = ComponentID(ch);
    const UInt width  = pcCU->getWidth(0)  >> pcCU->getPic()->getComponentScaleX(compID);
    const UInt height = pcCU->getHeight(0) >> pcCU->getPic()->getComponentScaleY(compID);
    const UInt stride = pcPredYuv->getStride(compID);

    Pel * pOrig    = pcOrgYuv->getAddr  (compID, 0, width);
    Pel * pResi    = pcResiYuv->getAddr(compID, 0, width);
    Pel * pPred    = pcPredYuv->getAddr(compID, 0, width);
    Pel * pReco    = pcRecoYuv->getAddr(compID, 0, width);
    Pel * pPCM     = pcCU->getPCMSample (compID);

    xEncPCM ( pcCU, 0, pOrig, pPCM, pPred, pResi, pReco, stride, width, height, compID );

  }

  m_pcEntropyCoder->resetBits();
  xEncIntraHeader ( pcCU, uiDepth, 0, true, false);
  uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();

  dCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );

  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);

  pcCU->getTotalBits()       = uiBits;
  pcCU->getTotalCost()       = dCost;
  pcCU->getTotalDistortion() = uiDistortion;

  pcCU->copyToPic(uiDepth);
}




Void TEncSearch::xGetInterPredictionError( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, Distortion& ruiErr, Bool /*bHadamard*/ )
{
  motionCompensation( pcCU, &m_tmpYuvPred, REF_PIC_LIST_X, iPartIdx );

  UInt uiAbsPartIdx = 0;
  Int iWidth = 0;
  Int iHeight = 0;
  pcCU->getPartIndexAndSize( iPartIdx, uiAbsPartIdx, iWidth, iHeight );

  DistParam cDistParam;

  cDistParam.bApplyWeight = false;


  m_pcRdCost->setDistParam( cDistParam, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA),
                            pcYuvOrg->getAddr( COMPONENT_Y, uiAbsPartIdx ), pcYuvOrg->getStride(COMPONENT_Y),
                            m_tmpYuvPred .getAddr( COMPONENT_Y, uiAbsPartIdx ), m_tmpYuvPred.getStride(COMPONENT_Y),
                            iWidth, iHeight, m_pcEncCfg->getUseHADME() && (pcCU->getCUTransquantBypass(iPartIdx) == 0) );

  ruiErr = cDistParam.DistFunc( &cDistParam );
}

//! estimation of best merge coding
Void TEncSearch::xMergeEstimation( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPUIdx, UInt& uiInterDir, TComMvField* pacMvField, UInt& uiMergeIndex, Distortion& ruiCost, TComMvField* cMvFieldNeighbours, UChar* uhInterDirNeighbours, Int& numValidMergeCand )
{
  UInt uiAbsPartIdx = 0;
  Int iWidth = 0;
  Int iHeight = 0;

  pcCU->getPartIndexAndSize( iPUIdx, uiAbsPartIdx, iWidth, iHeight );
  UInt uiDepth = pcCU->getDepth( uiAbsPartIdx );

  PartSize partSize = pcCU->getPartitionSize( 0 );
  if ( pcCU->getSlice()->getPPS()->getLog2ParallelMergeLevelMinus2() && partSize != SIZE_2Nx2N && pcCU->getWidth( 0 ) <= 8 )
  {
    if ( iPUIdx == 0 )
    {
      pcCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth ); // temporarily set
      pcCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand );
      pcCU->setPartSizeSubParts( partSize, 0, uiDepth ); // restore
    }
  }
  else
  {
    pcCU->getInterMergeCandidates( uiAbsPartIdx, iPUIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand );
  }

  xRestrictBipredMergeCand( pcCU, iPUIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand );

  ruiCost = std::numeric_limits<Distortion>::max();
  for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
  {
    Distortion uiCostCand = std::numeric_limits<Distortion>::max();
    UInt       uiBitsCand = 0;

    PartSize ePartSize = pcCU->getPartitionSize( 0 );

    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], ePartSize, uiAbsPartIdx, 0, iPUIdx );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], ePartSize, uiAbsPartIdx, 0, iPUIdx );

    xGetInterPredictionError( pcCU, pcYuvOrg, iPUIdx, uiCostCand, m_pcEncCfg->getUseHADME() );
    uiBitsCand = uiMergeCand + 1;
    if (uiMergeCand == m_pcEncCfg->getMaxNumMergeCand() -1)
    {
        uiBitsCand--;
    }
    uiCostCand = uiCostCand + m_pcRdCost->getCost( uiBitsCand );
    if ( uiCostCand < ruiCost )
    {
      ruiCost = uiCostCand;
      pacMvField[0] = cMvFieldNeighbours[0 + 2*uiMergeCand];
      pacMvField[1] = cMvFieldNeighbours[1 + 2*uiMergeCand];
      uiInterDir = uhInterDirNeighbours[uiMergeCand];
      uiMergeIndex = uiMergeCand;
    }
  }
}

/** convert bi-pred merge candidates to uni-pred
 * \param pcCU
 * \param puIdx
 * \param mvFieldNeighbours
 * \param interDirNeighbours
 * \param numValidMergeCand
 * \returns Void
 */
Void TEncSearch::xRestrictBipredMergeCand( TComDataCU* pcCU, UInt puIdx, TComMvField* mvFieldNeighbours, UChar* interDirNeighbours, Int numValidMergeCand )
{
  if ( pcCU->isBipredRestriction(puIdx) )
  {
    for( UInt mergeCand = 0; mergeCand < numValidMergeCand; ++mergeCand )
    {
      if ( interDirNeighbours[mergeCand] == 3 )
      {
        interDirNeighbours[mergeCand] = 1;
        mvFieldNeighbours[(mergeCand << 1) + 1].setMvField(TComMv(0,0), -1);
      }
    }
  }
}

//! search of the best candidate for inter prediction
#if AMP_MRG
Void TEncSearch::predInterSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv, TComYuv* pcRecoYuv DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseRes, Bool bUseMRG )
#else
Void TEncSearch::predInterSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv, TComYuv* pcRecoYuv, Bool bUseRes )
#endif
{
  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acYuvPred[i].clear();
  }
  m_cYuvPredTemp.clear();
  pcPredYuv->clear();

  if ( !bUseRes )
  {
    pcResiYuv->clear();
  }

  pcRecoYuv->clear();

  TComMv       cMvSrchRngLT;
  TComMv       cMvSrchRngRB;

  TComMv       cMvZero;
  TComMv       TempMv; //kolya

  TComMv       cMv[2];
  TComMv       cMvBi[2];
  TComMv       cMvTemp[2][33];

  Int          iNumPart    = pcCU->getNumPartitions();
  Int          iNumPredDir = pcCU->getSlice()->isInterP() ? 1 : 2;

  TComMv       cMvPred[2][33];

  TComMv       cMvPredBi[2][33];
  Int          aaiMvpIdxBi[2][33];

  Int          aaiMvpIdx[2][33];
  Int          aaiMvpNum[2][33];

  AMVPInfo     aacAMVPInfo[2][33];

  Int          iRefIdx[2]={0,0}; //If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  Int          iRefIdxBi[2];

  UInt         uiPartAddr;
  Int          iRoiWidth, iRoiHeight;

  UInt         uiMbBits[3] = {1, 1, 0};

  UInt         uiLastMode = 0;
  Int          iRefStart, iRefEnd;

  PartSize     ePartSize = pcCU->getPartitionSize( 0 );

  Int          bestBiPRefIdxL1 = 0;
  Int          bestBiPMvpL1 = 0;
  Distortion   biPDistTemp = std::numeric_limits<Distortion>::max();

  TComMvField cMvFieldNeighbours[MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
  UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
  Int numValidMergeCand = 0 ;

  for ( Int iPartIdx = 0; iPartIdx < iNumPart; iPartIdx++ )
  {
    Distortion   uiCost[2] = { std::numeric_limits<Distortion>::max(), std::numeric_limits<Distortion>::max() };
    Distortion   uiCostBi  =   std::numeric_limits<Distortion>::max();
    Distortion   uiCostTemp;

    UInt         uiBits[3];
    UInt         uiBitsTemp;
    Distortion   bestBiPDist = std::numeric_limits<Distortion>::max();

    Distortion   uiCostTempL0[MAX_NUM_REF];
    for (Int iNumRef=0; iNumRef < MAX_NUM_REF; iNumRef++)
    {
      uiCostTempL0[iNumRef] = std::numeric_limits<Distortion>::max();
    }
    UInt         uiBitsTempL0[MAX_NUM_REF];

    TComMv       mvValidList1;
    Int          refIdxValidList1 = 0;
    UInt         bitsValidList1 = MAX_UINT;
    Distortion   costValidList1 = std::numeric_limits<Distortion>::max();

    xGetBlkBits( ePartSize, pcCU->getSlice()->isInterP(), iPartIdx, uiLastMode, uiMbBits);

    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );

#if AMP_MRG
    Bool bTestNormalMC = true;

    if ( bUseMRG && pcCU->getWidth( 0 ) > 8 && iNumPart == 2 )
    {
      bTestNormalMC = false;
    }

    if (bTestNormalMC)
    {
#endif

    //  Uni-directional prediction
    for ( Int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
    {
      RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

      for ( Int iRefIdxTemp = 0; iRefIdxTemp < pcCU->getSlice()->getNumRefIdx(eRefPicList); iRefIdxTemp++ )
      {
        uiBitsTemp = uiMbBits[iRefList];
        if ( pcCU->getSlice()->getNumRefIdx(eRefPicList) > 1 )
        {
          uiBitsTemp += iRefIdxTemp+1;
          if ( iRefIdxTemp == pcCU->getSlice()->getNumRefIdx(eRefPicList)-1 )
          {
            uiBitsTemp--;
          }
        }
        xEstimateMvPredAMVP( pcCU, pcOrgYuv, iPartIdx, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], false, &biPDistTemp);
        aaiMvpIdx[iRefList][iRefIdxTemp] = pcCU->getMVPIdx(eRefPicList, uiPartAddr);
        aaiMvpNum[iRefList][iRefIdxTemp] = pcCU->getMVPNum(eRefPicList, uiPartAddr);

        if(pcCU->getSlice()->getMvdL1ZeroFlag() && iRefList==1 && biPDistTemp < bestBiPDist)
        {
          bestBiPDist = biPDistTemp;
          bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
          bestBiPRefIdxL1 = iRefIdxTemp;
        }

        uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

        if ( m_pcEncCfg->getFastMEForGenBLowDelayEnabled() && iRefList == 1 )    // list 1
        {
          if ( pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp ) >= 0 )
          {
            cMvTemp[1][iRefIdxTemp] = cMvTemp[0][pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp )];
            uiCostTemp = uiCostTempL0[pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp )];
            /*first subtract the bit-rate part of the cost of the other list*/
            uiCostTemp -= m_pcRdCost->getCost( uiBitsTempL0[pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp )] );
            /*correct the bit-rate part of the current ref*/
            m_pcRdCost->setPredictor  ( cMvPred[iRefList][iRefIdxTemp] );
            uiBitsTemp += m_pcRdCost->getBits( cMvTemp[1][iRefIdxTemp].getHor(), cMvTemp[1][iRefIdxTemp].getVer() );
            /*calculate the correct cost*/
            uiCostTemp += m_pcRdCost->getCost( uiBitsTemp );
          }
          else
          {
            xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
          }
        }
        else
        {
          xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
        }
        xCopyAMVPInfo(pcCU->getCUMvField(eRefPicList)->getAMVPInfo(), &aacAMVPInfo[iRefList][iRefIdxTemp]); // must always be done ( also when AMVP_MODE = AM_NONE )
        xCheckBestMVP(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);

        if ( iRefList == 0 )
        {
          uiCostTempL0[iRefIdxTemp] = uiCostTemp;
          uiBitsTempL0[iRefIdxTemp] = uiBitsTemp;
        }
        if ( uiCostTemp < uiCost[iRefList] )
        {
          uiCost[iRefList] = uiCostTemp;
          uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction

          // set motion
          cMv[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
          iRefIdx[iRefList] = iRefIdxTemp;
        }

        if ( iRefList == 1 && uiCostTemp < costValidList1 && pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp ) < 0 )
        {
          costValidList1 = uiCostTemp;
          bitsValidList1 = uiBitsTemp;

          // set motion
          mvValidList1     = cMvTemp[iRefList][iRefIdxTemp];
          refIdxValidList1 = iRefIdxTemp;
        }
      }
    }

    //  Bi-directional prediction
    if ( (pcCU->getSlice()->isInterB()) && (pcCU->isBipredRestriction(iPartIdx) == false) )
    {

      cMvBi[0] = cMv[0];            cMvBi[1] = cMv[1];
      iRefIdxBi[0] = iRefIdx[0];    iRefIdxBi[1] = iRefIdx[1];

      ::memcpy(cMvPredBi, cMvPred, sizeof(cMvPred));
      ::memcpy(aaiMvpIdxBi, aaiMvpIdx, sizeof(aaiMvpIdx));

      UInt uiMotBits[2];

      if(pcCU->getSlice()->getMvdL1ZeroFlag())
      {
        xCopyAMVPInfo(&aacAMVPInfo[1][bestBiPRefIdxL1], pcCU->getCUMvField(REF_PIC_LIST_1)->getAMVPInfo());
        pcCU->setMVPIdxSubParts( bestBiPMvpL1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;
        cMvPredBi[1][bestBiPRefIdxL1]   = pcCU->getCUMvField(REF_PIC_LIST_1)->getAMVPInfo()->m_acMvCand[bestBiPMvpL1];

        cMvBi[1] = cMvPredBi[1][bestBiPRefIdxL1];
        iRefIdxBi[1] = bestBiPRefIdxL1;
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMv( cMvBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( iRefIdxBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
        TComYuv* pcYuvPred = &m_acYuvPred[REF_PIC_LIST_1];
        motionCompensation( pcCU, pcYuvPred, REF_PIC_LIST_1, iPartIdx );

        uiMotBits[0] = uiBits[0] - uiMbBits[0];
        uiMotBits[1] = uiMbBits[1];

        if ( pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1) > 1 )
        {
          uiMotBits[1] += bestBiPRefIdxL1+1;
          if ( bestBiPRefIdxL1 == pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1)-1 )
          {
            uiMotBits[1]--;
          }
        }

        uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];

        uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];

        cMvTemp[1][bestBiPRefIdxL1] = cMvBi[1];
      }
      else
      {
        uiMotBits[0] = uiBits[0] - uiMbBits[0];
        uiMotBits[1] = uiBits[1] - uiMbBits[1];
        uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
      }

      // 4-times iteration (default)
      Int iNumIter = 4;

      // fast encoder setting: only one iteration
      if ( m_pcEncCfg->getUseFastEnc() || pcCU->getSlice()->getMvdL1ZeroFlag())
      {
        iNumIter = 1;
      }

      for ( Int iIter = 0; iIter < iNumIter; iIter++ )
      {
        Int         iRefList    = iIter % 2;

        if ( m_pcEncCfg->getUseFastEnc() )
        {
          if( uiCost[0] <= uiCost[1] )
          {
            iRefList = 1;
          }
          else
          {
            iRefList = 0;
          }
        }
        else if ( iIter == 0 )
        {
          iRefList = 0;
        }
        if ( iIter == 0 && !pcCU->getSlice()->getMvdL1ZeroFlag())
        {
          pcCU->getCUMvField(RefPicList(1-iRefList))->setAllMv( cMv[1-iRefList], ePartSize, uiPartAddr, 0, iPartIdx );
          pcCU->getCUMvField(RefPicList(1-iRefList))->setAllRefIdx( iRefIdx[1-iRefList], ePartSize, uiPartAddr, 0, iPartIdx );
          TComYuv*  pcYuvPred = &m_acYuvPred[1-iRefList];
          motionCompensation ( pcCU, pcYuvPred, RefPicList(1-iRefList), iPartIdx );
        }

        RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

        if(pcCU->getSlice()->getMvdL1ZeroFlag())
        {
          iRefList = 0;
          eRefPicList = REF_PIC_LIST_0;
        }

        Bool bChanged = false;

        iRefStart = 0;
        iRefEnd   = pcCU->getSlice()->getNumRefIdx(eRefPicList)-1;

        for ( Int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++ )
        {
          uiBitsTemp = uiMbBits[2] + uiMotBits[1-iRefList];
          if ( pcCU->getSlice()->getNumRefIdx(eRefPicList) > 1 )
          {
            uiBitsTemp += iRefIdxTemp+1;
            if ( iRefIdxTemp == pcCU->getSlice()->getNumRefIdx(eRefPicList)-1 )
            {
              uiBitsTemp--;
            }
          }
          uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];
          // call ME
          xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, true );

          xCopyAMVPInfo(&aacAMVPInfo[iRefList][iRefIdxTemp], pcCU->getCUMvField(eRefPicList)->getAMVPInfo());
          xCheckBestMVP(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);

          if ( uiCostTemp < uiCostBi )
          {
            bChanged = true;

            cMvBi[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
            iRefIdxBi[iRefList] = iRefIdxTemp;

            uiCostBi            = uiCostTemp;
            uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1-iRefList];
            uiBits[2]           = uiBitsTemp;

            if(iNumIter!=1)
            {
              //  Set motion
              pcCU->getCUMvField( eRefPicList )->setAllMv( cMvBi[iRefList], ePartSize, uiPartAddr, 0, iPartIdx );
              pcCU->getCUMvField( eRefPicList )->setAllRefIdx( iRefIdxBi[iRefList], ePartSize, uiPartAddr, 0, iPartIdx );

              TComYuv* pcYuvPred = &m_acYuvPred[iRefList];
              motionCompensation( pcCU, pcYuvPred, eRefPicList, iPartIdx );
            }
          }
        } // for loop-iRefIdxTemp

        if ( !bChanged )
        {
          if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] )
          {
            xCopyAMVPInfo(&aacAMVPInfo[0][iRefIdxBi[0]], pcCU->getCUMvField(REF_PIC_LIST_0)->getAMVPInfo());
            xCheckBestMVP(pcCU, REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], uiBits[2], uiCostBi);
            if(!pcCU->getSlice()->getMvdL1ZeroFlag())
            {
              xCopyAMVPInfo(&aacAMVPInfo[1][iRefIdxBi[1]], pcCU->getCUMvField(REF_PIC_LIST_1)->getAMVPInfo());
              xCheckBestMVP(pcCU, REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], uiBits[2], uiCostBi);
            }
          }
          break;
        }
      } // for loop-iter
    } // if (B_SLICE)

#if AMP_MRG
    } //end if bTestNormalMC
#endif
    //  Clear Motion Field
    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( TComMvField(), ePartSize, uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( TComMvField(), ePartSize, uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( cMvZero,       ePartSize, uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( cMvZero,       ePartSize, uiPartAddr, 0, iPartIdx );

    pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

    UInt uiMEBits = 0;
    // Set Motion Field_
    cMv[1] = mvValidList1;
    iRefIdx[1] = refIdxValidList1;
    uiBits[1] = bitsValidList1;
    uiCost[1] = costValidList1;

#if AMP_MRG
    if (bTestNormalMC)
    {
#endif
    if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1])
    {
      uiLastMode = 2;
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMv( cMvBi[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllRefIdx( iRefIdxBi[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMv( cMvBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllRefIdx( iRefIdxBi[1], ePartSize, uiPartAddr, 0, iPartIdx );

      TempMv = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );

      TempMv = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );

      pcCU->setInterDirSubParts( 3, uiPartAddr, iPartIdx, pcCU->getDepth(0) );

      pcCU->setMVPIdxSubParts( aaiMvpIdxBi[0][iRefIdxBi[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[0][iRefIdxBi[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPIdxSubParts( aaiMvpIdxBi[1][iRefIdxBi[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[1][iRefIdxBi[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

      uiMEBits = uiBits[2];
    }
    else if ( uiCost[0] <= uiCost[1] )
    {
      uiLastMode = 0;
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMv( cMv[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllRefIdx( iRefIdx[0], ePartSize, uiPartAddr, 0, iPartIdx );

      TempMv = cMv[0] - cMvPred[0][iRefIdx[0]];
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );

      pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, pcCU->getDepth(0) );

      pcCU->setMVPIdxSubParts( aaiMvpIdx[0][iRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[0][iRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

      uiMEBits = uiBits[0];
    }
    else
    {
      uiLastMode = 1;
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMv( cMv[1], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllRefIdx( iRefIdx[1], ePartSize, uiPartAddr, 0, iPartIdx );

      TempMv = cMv[1] - cMvPred[1][iRefIdx[1]];
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );

      pcCU->setInterDirSubParts( 2, uiPartAddr, iPartIdx, pcCU->getDepth(0) );

      pcCU->setMVPIdxSubParts( aaiMvpIdx[1][iRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[1][iRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

      uiMEBits = uiBits[1];
    }
#if AMP_MRG
    } // end if bTestNormalMC
#endif

    if ( pcCU->getPartitionSize( uiPartAddr ) != SIZE_2Nx2N )
    {
      UInt uiMRGInterDir = 0;
      TComMvField cMRGMvField[2];
      UInt uiMRGIndex = 0;

      UInt uiMEInterDir = 0;
      TComMvField cMEMvField[2];

      m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );

#if AMP_MRG
      // calculate ME cost
      Distortion uiMEError = std::numeric_limits<Distortion>::max();
      Distortion uiMECost  = std::numeric_limits<Distortion>::max();

      if (bTestNormalMC)
      {
        xGetInterPredictionError( pcCU, pcOrgYuv, iPartIdx, uiMEError, m_pcEncCfg->getUseHADME() );
        uiMECost = uiMEError + m_pcRdCost->getCost( uiMEBits );
      }
#else
      // calculate ME cost
      Distortion uiMEError = std::numeric_limits<Distortion>::max();
      xGetInterPredictionError( pcCU, pcOrgYuv, iPartIdx, uiMEError, m_pcEncCfg->getUseHADME() );
      Distortion uiMECost = uiMEError + m_pcRdCost->getCost( uiMEBits );
#endif
      // save ME result.
      uiMEInterDir = pcCU->getInterDir( uiPartAddr );
      pcCU->getMvField( pcCU, uiPartAddr, REF_PIC_LIST_0, cMEMvField[0] );
      pcCU->getMvField( pcCU, uiPartAddr, REF_PIC_LIST_1, cMEMvField[1] );

      // find Merge result
      Distortion uiMRGCost = std::numeric_limits<Distortion>::max();

      xMergeEstimation( pcCU, pcOrgYuv, iPartIdx, uiMRGInterDir, cMRGMvField, uiMRGIndex, uiMRGCost, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand);

      if ( uiMRGCost < uiMECost )
      {
        // set Merge result
        pcCU->setMergeFlagSubParts ( true,          uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->setMergeIndexSubParts( uiMRGIndex,    uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->setInterDirSubParts  ( uiMRGInterDir, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMRGMvField[0], ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMRGMvField[1], ePartSize, uiPartAddr, 0, iPartIdx );

        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( cMvZero,            ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( cMvZero,            ePartSize, uiPartAddr, 0, iPartIdx );

        pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      }
      else
      {
        // set ME result
        pcCU->setMergeFlagSubParts( false,        uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->setInterDirSubParts ( uiMEInterDir, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMEMvField[0], ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMEMvField[1], ePartSize, uiPartAddr, 0, iPartIdx );
      }
    }

    //  MC
    motionCompensation ( pcCU, pcPredYuv, REF_PIC_LIST_X, iPartIdx );

  } //  end of for ( Int iPartIdx = 0; iPartIdx < iNumPart; iPartIdx++ )

  setWpScalingDistParam( pcCU, -1, REF_PIC_LIST_X );

  return;
}


// AMVP
Void TEncSearch::xEstimateMvPredAMVP( TComDataCU* pcCU, TComYuv* pcOrgYuv, UInt uiPartIdx, RefPicList eRefPicList, Int iRefIdx, TComMv& rcMvPred, Bool bFilled, Distortion* puiDistBiP )
{
  AMVPInfo*  pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();

  TComMv     cBestMv;
  Int        iBestIdx   = 0;
  TComMv     cZeroMv;
  TComMv     cMvPred;
  Distortion uiBestCost = std::numeric_limits<Distortion>::max();
  UInt       uiPartAddr = 0;
  Int        iRoiWidth, iRoiHeight;
  Int        i;

  pcCU->getPartIndexAndSize( uiPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );
  // Fill the MV Candidates
  if (!bFilled)
  {
    pcCU->fillMvpCand( uiPartIdx, uiPartAddr, eRefPicList, iRefIdx, pcAMVPInfo );
  }

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  cBestMv  = pcAMVPInfo->m_acMvCand[0];
  if (pcAMVPInfo->iN <= 1)
  {
    rcMvPred = cBestMv;

    pcCU->setMVPIdxSubParts( iBestIdx, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));

    if(pcCU->getSlice()->getMvdL1ZeroFlag() && eRefPicList==REF_PIC_LIST_1)
    {
      (*puiDistBiP) = xGetTemplateCost( pcCU, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, rcMvPred, 0, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight);
    }
    return;
  }

  if (bFilled)
  {
    assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
    rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
    return;
  }

  m_cYuvPredTemp.clear();
  //-- Check Minimum Cost.
  for ( i = 0 ; i < pcAMVPInfo->iN; i++)
  {
    Distortion uiTmpCost;
    uiTmpCost = xGetTemplateCost( pcCU, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, pcAMVPInfo->m_acMvCand[i], i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight);
    if ( uiBestCost > uiTmpCost )
    {
      uiBestCost = uiTmpCost;
      cBestMv   = pcAMVPInfo->m_acMvCand[i];
      iBestIdx  = i;
      (*puiDistBiP) = uiTmpCost;
    }
  }

  m_cYuvPredTemp.clear();

  // Setting Best MVP
  rcMvPred = cBestMv;
  pcCU->setMVPIdxSubParts( iBestIdx, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
  pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
  return;
}

UInt TEncSearch::xGetMvpIdxBits(Int iIdx, Int iNum)
{
  assert(iIdx >= 0 && iNum >= 0 && iIdx < iNum);

  if (iNum == 1)
  {
    return 0;
  }

  UInt uiLength = 1;
  Int iTemp = iIdx;
  if ( iTemp == 0 )
  {
    return uiLength;
  }

  Bool bCodeLast = ( iNum-1 > iTemp );

  uiLength += (iTemp-1);

  if( bCodeLast )
  {
    uiLength++;
  }

  return uiLength;
}

Void TEncSearch::xGetBlkBits( PartSize eCUMode, Bool bPSlice, Int iPartIdx, UInt uiLastMode, UInt uiBlkBit[3])
{
  if ( eCUMode == SIZE_2Nx2N )
  {
    uiBlkBit[0] = (! bPSlice) ? 3 : 1;
    uiBlkBit[1] = 3;
    uiBlkBit[2] = 5;
  }
  else if ( (eCUMode == SIZE_2NxN || eCUMode == SIZE_2NxnU) || eCUMode == SIZE_2NxnD )
  {
    UInt aauiMbBits[2][3][3] = { { {0,0,3}, {0,0,0}, {0,0,0} } , { {5,7,7}, {7,5,7}, {9-3,9-3,9-3} } };
    if ( bPSlice )
    {
      uiBlkBit[0] = 3;
      uiBlkBit[1] = 0;
      uiBlkBit[2] = 0;
    }
    else
    {
      ::memcpy( uiBlkBit, aauiMbBits[iPartIdx][uiLastMode], 3*sizeof(UInt) );
    }
  }
  else if ( (eCUMode == SIZE_Nx2N || eCUMode == SIZE_nLx2N) || eCUMode == SIZE_nRx2N )
  {
    UInt aauiMbBits[2][3][3] = { { {0,2,3}, {0,0,0}, {0,0,0} } , { {5,7,7}, {7-2,7-2,9-2}, {9-3,9-3,9-3} } };
    if ( bPSlice )
    {
      uiBlkBit[0] = 3;
      uiBlkBit[1] = 0;
      uiBlkBit[2] = 0;
    }
    else
    {
      ::memcpy( uiBlkBit, aauiMbBits[iPartIdx][uiLastMode], 3*sizeof(UInt) );
    }
  }
  else if ( eCUMode == SIZE_NxN )
  {
    uiBlkBit[0] = (! bPSlice) ? 3 : 1;
    uiBlkBit[1] = 3;
    uiBlkBit[2] = 5;
  }
  else
  {
    printf("Wrong!\n");
    assert( 0 );
  }
}

Void TEncSearch::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->iN = pSrc->iN;
  for (Int i = 0; i < pSrc->iN; i++)
  {
    pDst->m_acMvCand[i] = pSrc->m_acMvCand[i];
  }
}

Void TEncSearch::xCheckBestMVP ( TComDataCU* pcCU, RefPicList eRefPicList, TComMv cMv, TComMv& rcMvPred, Int& riMVPIdx, UInt& ruiBits, Distortion& ruiCost )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();

  assert(pcAMVPInfo->m_acMvCand[riMVPIdx] == rcMvPred);

  if (pcAMVPInfo->iN < 2)
  {
    return;
  }

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(0) );
  m_pcRdCost->setCostScale ( 0    );

  Int iBestMVPIdx = riMVPIdx;

  m_pcRdCost->setPredictor( rcMvPred );
  Int iOrgMvBits  = m_pcRdCost->getBits(cMv.getHor(), cMv.getVer());
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];
  Int iBestMvBits = iOrgMvBits;

  for (Int iMVPIdx = 0; iMVPIdx < pcAMVPInfo->iN; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx)
    {
      continue;
    }

    m_pcRdCost->setPredictor( pcAMVPInfo->m_acMvCand[iMVPIdx] );

    Int iMvBits = m_pcRdCost->getBits(cMv.getHor(), cMv.getVer());
    iMvBits += m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];

    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)  //if changed
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[iBestMVPIdx];

    riMVPIdx = iBestMVPIdx;
    UInt uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits ))  + m_pcRdCost->getCost( ruiBits );
  }
}


Distortion TEncSearch::xGetTemplateCost( TComDataCU* pcCU,
                                         UInt        uiPartAddr,
                                         TComYuv*    pcOrgYuv,
                                         TComYuv*    pcTemplateCand,
                                         TComMv      cMvCand,
                                         Int         iMVPIdx,
                                         Int         iMVPNum,
                                         RefPicList  eRefPicList,
                                         Int         iRefIdx,
                                         Int         iSizeX,
                                         Int         iSizeY
                                         )
{
  Distortion uiCost = std::numeric_limits<Distortion>::max();

  TComPicYuv* pcPicYuvRef = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec();

  pcCU->clipMv( cMvCand );

  // prediction pattern
  if ( pcCU->getSlice()->testWeightPred() && pcCU->getSlice()->getSliceType()==P_SLICE )
  {
    xPredInterBlk( COMPONENT_Y, pcCU, pcPicYuvRef, uiPartAddr, &cMvCand, iSizeX, iSizeY, pcTemplateCand, true, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );
  }
  else
  {
    xPredInterBlk( COMPONENT_Y, pcCU, pcPicYuvRef, uiPartAddr, &cMvCand, iSizeX, iSizeY, pcTemplateCand, false, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );
  }

  if ( pcCU->getSlice()->testWeightPred() && pcCU->getSlice()->getSliceType()==P_SLICE )
  {
    xWeightedPredictionUni( pcCU, pcTemplateCand, uiPartAddr, iSizeX, iSizeY, eRefPicList, pcTemplateCand, iRefIdx );
  }

  // calc distortion

  uiCost = m_pcRdCost->getDistPart( pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA), pcTemplateCand->getAddr(COMPONENT_Y, uiPartAddr), pcTemplateCand->getStride(COMPONENT_Y), pcOrgYuv->getAddr(COMPONENT_Y, uiPartAddr), pcOrgYuv->getStride(COMPONENT_Y), iSizeX, iSizeY, COMPONENT_Y, DF_SAD );
  uiCost = (UInt) m_pcRdCost->calcRdCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum], uiCost, false, DF_SAD );
  return uiCost;
}




Void TEncSearch::xMotionEstimation( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, RefPicList eRefPicList, TComMv* pcMvPred, Int iRefIdxPred, TComMv& rcMv, UInt& ruiBits, Distortion& ruiCost, Bool bBi  )
{
  UInt          uiPartAddr;
  Int           iRoiWidth;
  Int           iRoiHeight;

  TComMv        cMvHalf, cMvQter;
  TComMv        cMvSrchRngLT;
  TComMv        cMvSrchRngRB;

  TComYuv*      pcYuv = pcYuvOrg;

  assert(eRefPicList < MAX_NUM_REF_LIST_ADAPT_SR && iRefIdxPred<Int(MAX_IDX_ADAPT_SR));
  m_iSearchRange = m_aaiAdaptSR[eRefPicList][iRefIdxPred];

  Int           iSrchRng      = ( bBi ? m_bipredSearchRange : m_iSearchRange );
  TComPattern   tmpPattern;
  TComPattern*  pcPatternKey  = &tmpPattern;

  Double        fWeight       = 1.0;

  pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );

  if ( bBi )
  {
    TComYuv*  pcYuvOther = &m_acYuvPred[1-(Int)eRefPicList];
    pcYuv                = &m_cYuvPredTemp;

    pcYuvOrg->copyPartToPartYuv( pcYuv, uiPartAddr, iRoiWidth, iRoiHeight );

    pcYuv->removeHighFreq( pcYuvOther, uiPartAddr, iRoiWidth, iRoiHeight, pcCU->getSlice()->getSPS()->getBitDepths().recon, m_pcEncCfg->getClipForBiPredMeEnabled() );

    fWeight = 0.5;
  }

  //  Search key pattern initialization
  pcPatternKey->initPattern( pcYuv->getAddr  ( COMPONENT_Y, uiPartAddr ),
                             iRoiWidth,
                             iRoiHeight,
                             pcYuv->getStride(COMPONENT_Y),
                             pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );

  Pel*        piRefY      = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRec()->getAddr( COMPONENT_Y, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiPartAddr );
  Int         iRefStride  = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRec()->getStride(COMPONENT_Y);

  TComMv      cMvPred = *pcMvPred;

  if ( bBi )
  {
    xSetSearchRange   ( pcCU, rcMv   , iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
  }
  else
  {
    xSetSearchRange   ( pcCU, cMvPred, iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
  }

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );

  m_pcRdCost->setPredictor  ( *pcMvPred );
  m_pcRdCost->setCostScale  ( 2 );

  setWpScalingDistParam( pcCU, iRefIdxPred, eRefPicList );
  //  Do integer search
  if ( !m_iFastSearch || bBi )
  {
    xPatternSearch      ( pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost );
  }
  else
  {
    rcMv = *pcMvPred;
    const TComMv *pIntegerMv2Nx2NPred=0;
    if (pcCU->getPartitionSize(0) != SIZE_2Nx2N || pcCU->getDepth(0) != 0)
    {
      pIntegerMv2Nx2NPred = &(m_integerMv2Nx2N[eRefPicList][iRefIdxPred]);
    }
    xPatternSearchFast  ( pcCU, pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost, pIntegerMv2Nx2NPred );
    if (pcCU->getPartitionSize(0) == SIZE_2Nx2N)
    {
      m_integerMv2Nx2N[eRefPicList][iRefIdxPred] = rcMv;
    }
  }

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );
  m_pcRdCost->setCostScale ( 1 );

  const Bool bIsLosslessCoded = pcCU->getCUTransquantBypass(uiPartAddr) != 0;
  xPatternSearchFracDIF( bIsLosslessCoded, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost );

  m_pcRdCost->setCostScale( 0 );
  rcMv <<= 2;
  rcMv += (cMvHalf <<= 1);
  rcMv +=  cMvQter;

  UInt uiMvBits = m_pcRdCost->getBits( rcMv.getHor(), rcMv.getVer() );

  ruiBits      += uiMvBits;
  ruiCost       = (Distortion)( floor( fWeight * ( (Double)ruiCost - (Double)m_pcRdCost->getCost( uiMvBits ) ) ) + (Double)m_pcRdCost->getCost( ruiBits ) );
}




Void TEncSearch::xSetSearchRange ( TComDataCU* pcCU, TComMv& cMvPred, Int iSrchRng, TComMv& rcMvSrchRngLT, TComMv& rcMvSrchRngRB )
{
  Int  iMvShift = 2;
  TComMv cTmpMvPred = cMvPred;
  pcCU->clipMv( cTmpMvPred );

  rcMvSrchRngLT.setHor( cTmpMvPred.getHor() - (iSrchRng << iMvShift) );
  rcMvSrchRngLT.setVer( cTmpMvPred.getVer() - (iSrchRng << iMvShift) );

  rcMvSrchRngRB.setHor( cTmpMvPred.getHor() + (iSrchRng << iMvShift) );
  rcMvSrchRngRB.setVer( cTmpMvPred.getVer() + (iSrchRng << iMvShift) );
  pcCU->clipMv        ( rcMvSrchRngLT );
  pcCU->clipMv        ( rcMvSrchRngRB );

  rcMvSrchRngLT >>= iMvShift;
  rcMvSrchRngRB >>= iMvShift;
}




Void TEncSearch::xPatternSearch( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, TComMv& rcMv, Distortion& ruiSAD )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  Distortion  uiSad;
  Distortion  uiSadBest = std::numeric_limits<Distortion>::max();
  Int         iBestX = 0;
  Int         iBestY = 0;

  Pel*  piRefSrch;

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( pcPatternKey, piRefY, iRefStride,  m_cDistParam );

  // fast encoder decision: use subsampled SAD for integer ME
  if ( m_pcEncCfg->getUseFastEnc() )
  {
    if ( m_cDistParam.iRows > 8 )
    {
      m_cDistParam.iSubShift = 1;
    }
  }

  piRefY += (iSrchRngVerTop * iRefStride);
  for ( Int y = iSrchRngVerTop; y <= iSrchRngVerBottom; y++ )
  {
    for ( Int x = iSrchRngHorLeft; x <= iSrchRngHorRight; x++ )
    {
      //  find min. distortion position
      piRefSrch = piRefY + x;
      m_cDistParam.pCur = piRefSrch;

      setDistParamComp(COMPONENT_Y);

      m_cDistParam.bitDepth = pcPatternKey->getBitDepthY();
      uiSad = m_cDistParam.DistFunc( &m_cDistParam );

      // motion cost
      uiSad += m_pcRdCost->getCost( x, y );

      if ( uiSad < uiSadBest )
      {
        uiSadBest = uiSad;
        iBestX    = x;
        iBestY    = y;
      }
    }
    piRefY += iRefStride;
  }

  rcMv.set( iBestX, iBestY );

  ruiSAD = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY );
  return;
}



Void TEncSearch::xPatternSearchFast( TComDataCU*   pcCU,
                                     TComPattern*  pcPatternKey,
                                     Pel*          piRefY,
                                     Int           iRefStride,
                                     TComMv*       pcMvSrchRngLT,
                                     TComMv*       pcMvSrchRngRB,
                                     TComMv       &rcMv,
                                     Distortion   &ruiSAD,
                                     const TComMv* pIntegerMv2Nx2NPred )
{
  assert (MD_LEFT < NUM_MV_PREDICTORS);
  pcCU->getMvPredLeft       ( m_acMvPredictors[MD_LEFT] );
  assert (MD_ABOVE < NUM_MV_PREDICTORS);
  pcCU->getMvPredAbove      ( m_acMvPredictors[MD_ABOVE] );
  assert (MD_ABOVE_RIGHT < NUM_MV_PREDICTORS);
  pcCU->getMvPredAboveRight ( m_acMvPredictors[MD_ABOVE_RIGHT] );

  switch ( m_iFastSearch )
  {
    case 1:
      xTZSearch( pcCU, pcPatternKey, piRefY, iRefStride, pcMvSrchRngLT, pcMvSrchRngRB, rcMv, ruiSAD, pIntegerMv2Nx2NPred );
      break;

    case 2:
      xTZSearchSelective( pcCU, pcPatternKey, piRefY, iRefStride, pcMvSrchRngLT, pcMvSrchRngRB, rcMv, ruiSAD, pIntegerMv2Nx2NPred );
      break;
    default:
      break;
  }
}




Void TEncSearch::xTZSearch( TComDataCU*  pcCU,
                            TComPattern* pcPatternKey,
                            Pel*         piRefY,
                            Int          iRefStride,
                            TComMv*      pcMvSrchRngLT,
                            TComMv*      pcMvSrchRngRB,
                            TComMv      &rcMv,
                            Distortion  &ruiSAD,
                            const TComMv* pIntegerMv2Nx2NPred )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  TZ_SEARCH_CONFIGURATION

  UInt uiSearchRange = m_iSearchRange;
  pcCU->clipMv( rcMv );
  rcMv >>= 2;
  // init TZSearchStruct
  IntTZSearchStruct cStruct;
  cStruct.iYStride    = iRefStride;
  cStruct.piRefY      = piRefY;
  cStruct.uiBestSad   = MAX_UINT;

  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( pcPatternKey, cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );

  // test whether one of PRED_A, PRED_B, PRED_C MV is better start point than Median predictor
  if ( bTestOtherPredictedMV )
  {
    for ( UInt index = 0; index < NUM_MV_PREDICTORS; index++ )
    {
      TComMv cMv = m_acMvPredictors[index];
      pcCU->clipMv( cMv );
      cMv >>= 2;
      xTZSearchHelp( pcPatternKey, cStruct, cMv.getHor(), cMv.getVer(), 0, 0 );
    }
  }

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    xTZSearchHelp( pcPatternKey, cStruct, 0, 0, 0, 0 );
  }

  if (pIntegerMv2Nx2NPred != 0)
  {
    TComMv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred <<= 2;
    pcCU->clipMv( integerMv2Nx2NPred );
    integerMv2Nx2NPred >>= 2;
    xTZSearchHelp(pcPatternKey, cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);

    // reset search range
    TComMv cMvSrchRngLT;
    TComMv cMvSrchRngRB;
    Int iSrchRng = m_iSearchRange;
    TComMv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= 2;
    xSetSearchRange( pcCU, currBestMv, iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
    iSrchRngHorLeft   = cMvSrchRngLT.getHor();
    iSrchRngHorRight  = cMvSrchRngRB.getHor();
    iSrchRngVerTop    = cMvSrchRngLT.getVer();
    iSrchRngVerBottom = cMvSrchRngRB.getVer();
  }

  // start search
  Int  iDist = 0;
  Int  iStartX = cStruct.iBestX;
  Int  iStartY = cStruct.iBestY;

  // first search
  for ( iDist = 1; iDist <= (Int)uiSearchRange; iDist*=2 )
  {
    if ( bFirstSearchDiamond == 1 )
    {
      xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
    }
    else
    {
      xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
    }

    if ( bFirstSearchStop && ( cStruct.uiBestRound >= uiFirstSearchRounds ) ) // stop criterion
    {
      break;
    }
  }

  // test whether zero Mv is a better start point than Median predictor
  if ( bTestZeroVectorStart && ((cStruct.iBestX != 0) || (cStruct.iBestY != 0)) )
  {
    xTZSearchHelp( pcPatternKey, cStruct, 0, 0, 0, 0 );
    if ( (cStruct.iBestX == 0) && (cStruct.iBestY == 0) )
    {
      // test its neighborhood
      for ( iDist = 1; iDist <= (Int)uiSearchRange; iDist*=2 )
      {
        xTZ8PointDiamondSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, 0, 0, iDist );
        if ( bTestZeroVectorStop && (cStruct.uiBestRound > 0) ) // stop criterion
        {
          break;
        }
      }
    }
  }

  // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
  if ( cStruct.uiBestDistance == 1 )
  {
    cStruct.uiBestDistance = 0;
    xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
  }

  // raster search if distance is too big
  if ( bEnableRasterSearch && ( ((Int)(cStruct.uiBestDistance) > iRaster) || bAlwaysRasterSearch ) )
  {
    cStruct.uiBestDistance = iRaster;
    for ( iStartY = iSrchRngVerTop; iStartY <= iSrchRngVerBottom; iStartY += iRaster )
    {
      for ( iStartX = iSrchRngHorLeft; iStartX <= iSrchRngHorRight; iStartX += iRaster )
      {
        xTZSearchHelp( pcPatternKey, cStruct, iStartX, iStartY, 0, iRaster );
      }
    }
  }

  // raster refinement
  if ( bRasterRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      if ( cStruct.uiBestDistance > 1 )
      {
        iDist = cStruct.uiBestDistance >>= 1;
        if ( bRasterRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        else
        {
          xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
      }

      // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
        }
      }
    }
  }

  // start refinement
  if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < (Int)uiSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        else
        {
          xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
        }
      }
    }
  }

  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCost( cStruct.iBestX, cStruct.iBestY );
}


Void TEncSearch::xTZSearchSelective( TComDataCU*   pcCU,
                                     TComPattern*  pcPatternKey,
                                     Pel*          piRefY,
                                     Int           iRefStride,
                                     TComMv*       pcMvSrchRngLT,
                                     TComMv*       pcMvSrchRngRB,
                                     TComMv       &rcMv,
                                     Distortion   &ruiSAD,
                                     const TComMv* pIntegerMv2Nx2NPred )
{
  SEL_SEARCH_CONFIGURATION

  Int   iSrchRngHorLeft         = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight        = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop          = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom       = pcMvSrchRngRB->getVer();
  Int   iFirstSrchRngHorLeft    = 0;
  Int   iFirstSrchRngHorRight   = 0;
  Int   iFirstSrchRngVerTop     = 0;
  Int   iFirstSrchRngVerBottom  = 0;
  Int   iStartX                 = 0;
  Int   iStartY                 = 0;
  Int   iBestX                  = 0;
  Int   iBestY                  = 0;
  Int   iDist                   = 0;

  pcCU->clipMv( rcMv );
  rcMv >>= 2;
  // init TZSearchStruct
  IntTZSearchStruct cStruct;
  cStruct.iYStride    = iRefStride;
  cStruct.piRefY      = piRefY;
  cStruct.uiBestSad   = MAX_UINT;
  cStruct.iBestX = 0;
  cStruct.iBestY = 0;


  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( pcPatternKey, cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );

  // test whether one of PRED_A, PRED_B, PRED_C MV is better start point than Median predictor
  if ( bTestOtherPredictedMV )
  {
    for ( UInt index = 0; index < NUM_MV_PREDICTORS; index++ )
    {
      TComMv cMv = m_acMvPredictors[index];
      pcCU->clipMv( cMv );
      cMv >>= 2;
      xTZSearchHelp( pcPatternKey, cStruct, cMv.getHor(), cMv.getVer(), 0, 0 );
    }
  }

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    xTZSearchHelp( pcPatternKey, cStruct, 0, 0, 0, 0 );
  }

  if ( pIntegerMv2Nx2NPred != 0 )
  {
    TComMv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred <<= 2;
    pcCU->clipMv( integerMv2Nx2NPred );
    integerMv2Nx2NPred >>= 2;
    xTZSearchHelp(pcPatternKey, cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);

    // reset search range
    TComMv cMvSrchRngLT;
    TComMv cMvSrchRngRB;
    Int iSrchRng = m_iSearchRange;
    TComMv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= 2;
    xSetSearchRange( pcCU, currBestMv, iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
    iSrchRngHorLeft   = cMvSrchRngLT.getHor();
    iSrchRngHorRight  = cMvSrchRngRB.getHor();
    iSrchRngVerTop    = cMvSrchRngLT.getVer();
    iSrchRngVerBottom = cMvSrchRngRB.getVer();
  }

  // Initial search
  iBestX = cStruct.iBestX;
  iBestY = cStruct.iBestY; 
  iFirstSrchRngHorLeft    = ((iBestX - uiSearchRangeInitial) > iSrchRngHorLeft)   ? (iBestX - uiSearchRangeInitial) : iSrchRngHorLeft;
  iFirstSrchRngVerTop     = ((iBestY - uiSearchRangeInitial) > iSrchRngVerTop)    ? (iBestY - uiSearchRangeInitial) : iSrchRngVerTop;
  iFirstSrchRngHorRight   = ((iBestX + uiSearchRangeInitial) < iSrchRngHorRight)  ? (iBestX + uiSearchRangeInitial) : iSrchRngHorRight;  
  iFirstSrchRngVerBottom  = ((iBestY + uiSearchRangeInitial) < iSrchRngVerBottom) ? (iBestY + uiSearchRangeInitial) : iSrchRngVerBottom;    

  for ( iStartY = iFirstSrchRngVerTop; iStartY <= iFirstSrchRngVerBottom; iStartY += uiSearchStep )
  {
    for ( iStartX = iFirstSrchRngHorLeft; iStartX <= iFirstSrchRngHorRight; iStartX += uiSearchStep )
    {
      xTZSearchHelp( pcPatternKey, cStruct, iStartX, iStartY, 0, 0 );
      xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, 1 );
      xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, 2 );
    }
  }

  Int iMaxMVDistToPred = (abs(cStruct.iBestX - iBestX) > iMVDistThresh || abs(cStruct.iBestY - iBestY) > iMVDistThresh);

  //full search with early exit if MV is distant from predictors
  if ( bEnableRasterSearch && (iMaxMVDistToPred || bAlwaysRasterSearch) )
  {
    for ( iStartY = iSrchRngVerTop; iStartY <= iSrchRngVerBottom; iStartY += 1 )
    {
      for ( iStartX = iSrchRngHorLeft; iStartX <= iSrchRngHorRight; iStartX += 1 )
      {
        xTZSearchHelp( pcPatternKey, cStruct, iStartX, iStartY, 0, 1 );
      }
    }
  }
  //Smaller MV, refine around predictor
  else if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    // start refinement
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < (Int)uiSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        else
        {
          xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
        }
      }
    }
  }

  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCost( cStruct.iBestX, cStruct.iBestY );

}


Void TEncSearch::xPatternSearchFracDIF(
                                       Bool         bIsLosslessCoded,
                                       TComPattern* pcPatternKey,
                                       Pel*         piRefY,
                                       Int          iRefStride,
                                       TComMv*      pcMvInt,
                                       TComMv&      rcMvHalf,
                                       TComMv&      rcMvQter,
                                       Distortion&  ruiCost
                                      )
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern(piRefY + iOffset,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride,
                          pcPatternKey->getBitDepthY());

  //  Half-pel refinement
  xExtDIFUpSamplingH ( &cPatternRoi );

  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  TComMv baseRefMv(0, 0);
  ruiCost = xPatternRefinement( pcPatternKey, baseRefMv, 2, rcMvHalf, !bIsLosslessCoded );

  m_pcRdCost->setCostScale( 0 );

  xExtDIFUpSamplingQ ( &cPatternRoi, rcMvHalf );
  baseRefMv = rcMvHalf;
  baseRefMv <<= 1;

  rcMvQter = *pcMvInt;   rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost = xPatternRefinement( pcPatternKey, baseRefMv, 1, rcMvQter, !bIsLosslessCoded );
}


//! encode residual and calculate rate-distortion for a CU block
Void TEncSearch::encodeResAndCalcRdInterCU( TComDataCU* pcCU, TComYuv* pcYuvOrg, TComYuv* pcYuvPred,
                                            TComYuv* pcYuvResi, TComYuv* pcYuvResiBest, TComYuv* pcYuvRec,
                                            Bool bSkipResidual DEBUG_STRING_FN_DECLARE(sDebug) )
{
  assert ( !pcCU->isIntra(0) );

  const UInt cuWidthPixels      = pcCU->getWidth ( 0 );
  const UInt cuHeightPixels     = pcCU->getHeight( 0 );
  const Int  numValidComponents = pcCU->getPic()->getNumberValidComponents();
  const TComSPS &sps=*(pcCU->getSlice()->getSPS());

  // The pcCU is not marked as skip-mode at this point, and its m_pcTrCoeff, m_pcArlCoeff, m_puhCbf, m_puhTrIdx will all be 0.
  // due to prior calls to TComDataCU::initEstData(  );

  if ( bSkipResidual ) //  No residual coding : SKIP mode
  {
    pcCU->setSkipFlagSubParts( true, 0, pcCU->getDepth(0) );

    pcYuvResi->clear();

    pcYuvPred->copyToPartYuv( pcYuvRec, 0 );
    Distortion distortion = 0;

    for (Int comp=0; comp < numValidComponents; comp++)
    {
      const ComponentID compID=ComponentID(comp);
      const UInt csx=pcYuvOrg->getComponentScaleX(compID);
      const UInt csy=pcYuvOrg->getComponentScaleY(compID);
      distortion += m_pcRdCost->getDistPart( sps.getBitDepth(toChannelType(compID)), pcYuvRec->getAddr(compID), pcYuvRec->getStride(compID), pcYuvOrg->getAddr(compID),
                                               pcYuvOrg->getStride(compID), cuWidthPixels >> csx, cuHeightPixels >> csy, compID);
    }

    m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST]);
    m_pcEntropyCoder->resetBits();

    if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
    }

    m_pcEntropyCoder->encodeSkipFlag(pcCU, 0, true);
    m_pcEntropyCoder->encodeMergeIndex( pcCU, 0, true );

    UInt uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    pcCU->getTotalBits()       = uiBits;
    pcCU->getTotalDistortion() = distortion;
    pcCU->getTotalCost()       = m_pcRdCost->calcRdCost( uiBits, distortion );

    m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_TEMP_BEST]);

#if DEBUG_STRING
    pcYuvResiBest->clear(); // Clear the residual image, if we didn't code it.
    for(UInt i=0; i<MAX_NUM_COMPONENT+1; i++)
    {
      sDebug+=debug_reorder_data_inter_token[i];
    }
#endif

    return;
  }

  //  Residual coding.

   pcYuvResi->subtract( pcYuvOrg, pcYuvPred, 0, cuWidthPixels );

  TComTURecurse tuLevel0(pcCU, 0);

  Double     nonZeroCost       = 0;
  UInt       nonZeroBits       = 0;
  Distortion nonZeroDistortion = 0;
  Distortion zeroDistortion    = 0;

  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_CURR_BEST ] );

  xEstimateInterResidualQT( pcYuvResi,  nonZeroCost, nonZeroBits, nonZeroDistortion, &zeroDistortion, tuLevel0 DEBUG_STRING_PASS_INTO(sDebug) );

  // -------------------------------------------------------
  // set the coefficients in the pcCU, and also calculates the residual data.
  // If a block full of 0's is efficient, then just use 0's.
  // The costs at this point do not include header bits.

  m_pcEntropyCoder->resetBits();
  m_pcEntropyCoder->encodeQtRootCbfZero( );
  const UInt   zeroResiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  const Double zeroCost     = (pcCU->isLosslessCoded( 0 )) ? (nonZeroCost+1) : (m_pcRdCost->calcRdCost( zeroResiBits, zeroDistortion ));

  if ( zeroCost < nonZeroCost || !pcCU->getQtRootCbf(0) )
  {
    const UInt uiQPartNum = tuLevel0.GetAbsPartIdxNumParts();
    ::memset( pcCU->getTransformIdx()     , 0, uiQPartNum * sizeof(UChar) );
    for (Int comp=0; comp < numValidComponents; comp++)
    {
      const ComponentID component = ComponentID(comp);
      ::memset( pcCU->getCbf( component ) , 0, uiQPartNum * sizeof(UChar) );
      ::memset( pcCU->getCrossComponentPredictionAlpha(component), 0, ( uiQPartNum * sizeof(Char) ) );
    }
    static const UInt useTS[MAX_NUM_COMPONENT]={0,0,0};
    pcCU->setTransformSkipSubParts ( useTS, 0, pcCU->getDepth(0) );
#if DEBUG_STRING
    sDebug.clear();
    for(UInt i=0; i<MAX_NUM_COMPONENT+1; i++)
    {
      sDebug+=debug_reorder_data_inter_token[i];
    }
#endif
  }
  else
  {
    xSetInterResidualQTData( NULL, false, tuLevel0); // Call first time to set coefficients.
  }

  // all decisions now made. Fully encode the CU, including the headers:
  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST] );

  UInt finalBits = 0;
  xAddSymbolBitsInter( pcCU, finalBits );
  // we've now encoded the pcCU, and so have a valid bit cost

  if ( !pcCU->getQtRootCbf( 0 ) )
  {
    pcYuvResiBest->clear(); // Clear the residual image, if we didn't code it.
  }
  else
  {
    xSetInterResidualQTData( pcYuvResiBest, true, tuLevel0 ); // else set the residual image data pcYUVResiBest from the various temp images.
  }
  m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_TEMP_BEST ] );

  pcYuvRec->addClip ( pcYuvPred, pcYuvResiBest, 0, cuWidthPixels, sps.getBitDepths() );

  // update with clipped distortion and cost (previously unclipped reconstruction values were used)

  Distortion finalDistortion = 0;
  for(Int comp=0; comp<numValidComponents; comp++)
  {
    const ComponentID compID=ComponentID(comp);
    finalDistortion += m_pcRdCost->getDistPart( sps.getBitDepth(toChannelType(compID)), pcYuvRec->getAddr(compID ), pcYuvRec->getStride(compID ), pcYuvOrg->getAddr(compID ), pcYuvOrg->getStride(compID), cuWidthPixels >> pcYuvOrg->getComponentScaleX(compID), cuHeightPixels >> pcYuvOrg->getComponentScaleY(compID), compID);
  }

  pcCU->getTotalBits()       = finalBits;
  pcCU->getTotalDistortion() = finalDistortion;
  pcCU->getTotalCost()       = m_pcRdCost->calcRdCost( finalBits, finalDistortion );
}



Void TEncSearch::xEstimateInterResidualQT( TComYuv    *pcResi,
                                           Double     &rdCost,
                                           UInt       &ruiBits,
                                           Distortion &ruiDist,
                                           Distortion *puiZeroDist,
                                           TComTU     &rTu
                                           DEBUG_STRING_FN_DECLARE(sDebug) )
{
  TComDataCU *pcCU        = rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiDepth      = rTu.GetTransformDepthTotal();
  const UInt uiTrMode     = rTu.GetTransformDepthRel();
  const UInt subTUDepth   = uiTrMode + 1;
  const UInt numValidComp = pcCU->getPic()->getNumberValidComponents();
  DEBUG_STRING_NEW(sSingleStringComp[MAX_NUM_COMPONENT])

  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();

  UInt SplitFlag = ((pcCU->getSlice()->getSPS()->getQuadtreeTUMaxDepthInter() == 1) && pcCU->isInter(uiAbsPartIdx) && ( pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N ));
#if DEBUG_STRING
  const Int debugPredModeMask = DebugStringGetPredModeMask(pcCU->getPredictionMode(uiAbsPartIdx));
#endif

  Bool bCheckFull;

  if ( SplitFlag && uiDepth == pcCU->getDepth(uiAbsPartIdx) && ( uiLog2TrSize >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) ) )
  {
    bCheckFull = false;
  }
  else
  {
    bCheckFull =  ( uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
  }

  const Bool bCheckSplit  = ( uiLog2TrSize >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );

  assert( bCheckFull || bCheckSplit );

  // code full block
  Double     dSingleCost = MAX_DOUBLE;
  UInt       uiSingleBits                                                                                                        = 0;
  Distortion uiSingleDistComp            [MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/] = {{0,0},{0,0},{0,0}};
  Distortion uiSingleDist                                                                                                        = 0;
  TCoeff     uiAbsSum                    [MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/] = {{0,0},{0,0},{0,0}};
  UInt       uiBestTransformMode         [MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/] = {{0,0},{0,0},{0,0}};
  //  Stores the best explicit RDPCM mode for a TU encoded without split
  UInt       bestExplicitRdpcmModeUnSplit[MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/] = {{3,3}, {3,3}, {3,3}};
  Char       bestCrossCPredictionAlpha   [MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/] = {{0,0},{0,0},{0,0}};

  m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );

  if( bCheckFull )
  {
    Double minCost[MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/];
    Bool checkTransformSkip[MAX_NUM_COMPONENT];
    pcCU->setTrIdxSubParts( uiTrMode, uiAbsPartIdx, uiDepth );

    m_pcEntropyCoder->resetBits();

    memset( m_pTempPel, 0, sizeof( Pel ) * rTu.getRect(COMPONENT_Y).width * rTu.getRect(COMPONENT_Y).height ); // not necessary needed for inside of recursion (only at the beginning)

    const UInt uiQTTempAccessLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    TCoeff *pcCoeffCurr[MAX_NUM_COMPONENT];
#if ADAPTIVE_QP_SELECTION
    TCoeff *pcArlCoeffCurr[MAX_NUM_COMPONENT];
#endif

    for(UInt i=0; i<numValidComp; i++)
    {
      minCost[i][0] = MAX_DOUBLE;
      minCost[i][1] = MAX_DOUBLE;
    }

    Pel crossCPredictedResidualBuffer[ MAX_TU_SIZE * MAX_TU_SIZE ];

    for(UInt i=0; i<numValidComp; i++)
    {
      checkTransformSkip[i]=false;
      const ComponentID compID=ComponentID(i);
      const Int channelBitDepth=pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
      pcCoeffCurr[compID]    = m_ppcQTTempCoeff[compID][uiQTTempAccessLayer] + rTu.getCoefficientOffset(compID);
#if ADAPTIVE_QP_SELECTION
      pcArlCoeffCurr[compID] = m_ppcQTTempArlCoeff[compID ][uiQTTempAccessLayer] +  rTu.getCoefficientOffset(compID);
#endif

      if(rTu.ProcessComponentSection(compID))
      {
        const QpParam cQP(*pcCU, compID);

        checkTransformSkip[compID] = pcCU->getSlice()->getPPS()->getUseTransformSkip() &&
                                     TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(compID), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize()) &&
                                     (!pcCU->isLosslessCoded(0));

        const Bool splitIntoSubTUs = rTu.getRect(compID).width != rTu.getRect(compID).height;

        TComTURecurse TUIterator(rTu, false, (splitIntoSubTUs ? TComTU::VERTICAL_SPLIT : TComTU::DONT_SPLIT), true, compID);

        const UInt partIdxesPerSubTU = TUIterator.GetAbsPartIdxNumParts(compID);

        do
        {
          const UInt           subTUIndex             = TUIterator.GetSectionNumber();
          const UInt           subTUAbsPartIdx        = TUIterator.GetAbsPartIdxTU(compID);
          const TComRectangle &tuCompRect             = TUIterator.getRect(compID);
          const UInt           subTUBufferOffset      = tuCompRect.width * tuCompRect.height * subTUIndex;

                TCoeff        *currentCoefficients    = pcCoeffCurr[compID] + subTUBufferOffset;
#if ADAPTIVE_QP_SELECTION
                TCoeff        *currentARLCoefficients = pcArlCoeffCurr[compID] + subTUBufferOffset;
#endif
          const Bool isCrossCPredictionAvailable      =    isChroma(compID)
                                                         && pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag()
                                                         && (pcCU->getCbf(subTUAbsPartIdx, COMPONENT_Y, uiTrMode) != 0);

          Char preCalcAlpha = 0;
          const Pel *pLumaResi = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( COMPONENT_Y, rTu.getRect( COMPONENT_Y ).x0, rTu.getRect( COMPONENT_Y ).y0 );

          if (isCrossCPredictionAvailable)
          {
            const Bool bUseReconstructedResidualForEstimate = m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate();
            const Pel  *const lumaResidualForEstimate       = bUseReconstructedResidualForEstimate ? pLumaResi                                                     : pcResi->getAddrPix(COMPONENT_Y, tuCompRect.x0, tuCompRect.y0);
            const UInt        lumaResidualStrideForEstimate = bUseReconstructedResidualForEstimate ? m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y) : pcResi->getStride(COMPONENT_Y);

            preCalcAlpha = xCalcCrossComponentPredictionAlpha(TUIterator,
                                                              compID,
                                                              lumaResidualForEstimate,
                                                              pcResi->getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                              tuCompRect.width,
                                                              tuCompRect.height,
                                                              lumaResidualStrideForEstimate,
                                                              pcResi->getStride(compID));
          }

          const Int transformSkipModesToTest    = checkTransformSkip[compID] ? 2 : 1;
          const Int crossCPredictionModesToTest = (preCalcAlpha != 0)        ? 2 : 1; // preCalcAlpha cannot be anything other than 0 if isCrossCPredictionAvailable is false

          const Bool isOneMode                  = (crossCPredictionModesToTest == 1) && (transformSkipModesToTest == 1);

          for (Int transformSkipModeId = 0; transformSkipModeId < transformSkipModesToTest; transformSkipModeId++)
          {
            pcCU->setTransformSkipPartRange(transformSkipModeId, compID, subTUAbsPartIdx, partIdxesPerSubTU);

            for (Int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++)
            {
              const Bool isFirstMode          = (transformSkipModeId == 0) && (crossCPredictionModeId == 0);
              const Bool bUseCrossCPrediction = crossCPredictionModeId != 0;

              m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
              m_pcEntropyCoder->resetBits();

              pcCU->setTransformSkipPartRange(transformSkipModeId, compID, subTUAbsPartIdx, partIdxesPerSubTU);
              pcCU->setCrossComponentPredictionAlphaPartRange((bUseCrossCPrediction ? preCalcAlpha : 0), compID, subTUAbsPartIdx, partIdxesPerSubTU );

              if ((compID != COMPONENT_Cr) && ((transformSkipModeId == 1) ? m_pcEncCfg->getUseRDOQTS() : m_pcEncCfg->getUseRDOQ()))
              {
                m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, tuCompRect.width, tuCompRect.height, toChannelType(compID));
              }

#if RDOQ_CHROMA_LAMBDA
              m_pcTrQuant->selectLambda(compID);
#endif

              Pel *pcResiCurrComp = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0);
              UInt resiStride     = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID);

              TCoeff bestCoeffComp   [MAX_TU_SIZE*MAX_TU_SIZE];
              Pel    bestResiComp    [MAX_TU_SIZE*MAX_TU_SIZE];

#if ADAPTIVE_QP_SELECTION
              TCoeff bestArlCoeffComp[MAX_TU_SIZE*MAX_TU_SIZE];
#endif
              TCoeff     currAbsSum   = 0;
              UInt       currCompBits = 0;
              Distortion currCompDist = 0;
              Double     currCompCost = 0;
              UInt       nonCoeffBits = 0;
              Distortion nonCoeffDist = 0;
              Double     nonCoeffCost = 0;

              if(!isOneMode && !isFirstMode)
              {
                memcpy(bestCoeffComp,    currentCoefficients,    (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#if ADAPTIVE_QP_SELECTION
                memcpy(bestArlCoeffComp, currentARLCoefficients, (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#endif
                for(Int y = 0; y < tuCompRect.height; y++)
                {
                  memcpy(&bestResiComp[y * tuCompRect.width], (pcResiCurrComp + (y * resiStride)), (sizeof(Pel) * tuCompRect.width));
                }
              }

              if (bUseCrossCPrediction)
              {
                TComTrQuant::crossComponentPrediction(TUIterator,
                                                      compID,
                                                      pLumaResi,
                                                      pcResi->getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                      crossCPredictedResidualBuffer,
                                                      tuCompRect.width,
                                                      tuCompRect.height,
                                                      m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                      pcResi->getStride(compID),
                                                      tuCompRect.width,
                                                      false);

                m_pcTrQuant->transformNxN(TUIterator, compID, crossCPredictedResidualBuffer, tuCompRect.width, currentCoefficients,
#if ADAPTIVE_QP_SELECTION
                                          currentARLCoefficients,
#endif
                                          currAbsSum, cQP);
              }
              else
              {
                m_pcTrQuant->transformNxN(TUIterator, compID, pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ), pcResi->getStride(compID), currentCoefficients,
#if ADAPTIVE_QP_SELECTION
                                          currentARLCoefficients,
#endif
                                          currAbsSum, cQP);
              }

              if(isFirstMode || (currAbsSum == 0))
              {
                if (bUseCrossCPrediction)
                {
                  TComTrQuant::crossComponentPrediction(TUIterator,
                                                        compID,
                                                        pLumaResi,
                                                        m_pTempPel,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                        tuCompRect.width,
                                                        tuCompRect.height,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                        tuCompRect.width,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID),
                                                        true);

                  nonCoeffDist = m_pcRdCost->getDistPart( channelBitDepth, m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride( compID ), pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                          pcResi->getStride(compID), tuCompRect.width, tuCompRect.height, compID); // initialized with zero residual destortion
                }
                else
                {
                  nonCoeffDist = m_pcRdCost->getDistPart( channelBitDepth, m_pTempPel, tuCompRect.width, pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                          pcResi->getStride(compID), tuCompRect.width, tuCompRect.height, compID); // initialized with zero residual destortion
                }

                m_pcEntropyCoder->encodeQtCbfZero( TUIterator, toChannelType(compID) );

                if ( isCrossCPredictionAvailable )
                {
                  m_pcEntropyCoder->encodeCrossComponentPrediction( TUIterator, compID );
                }

                nonCoeffBits = m_pcEntropyCoder->getNumberOfWrittenBits();
                nonCoeffCost = m_pcRdCost->calcRdCost( nonCoeffBits, nonCoeffDist );
              }

              if((puiZeroDist != NULL) && isFirstMode)
              {
                *puiZeroDist += nonCoeffDist; // initialized with zero residual destortion
              }

              DEBUG_STRING_NEW(sSingleStringTest)

              if( currAbsSum > 0 ) //if non-zero coefficients are present, a residual needs to be derived for further prediction
              {
                if (isFirstMode)
                {
                  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
                  m_pcEntropyCoder->resetBits();
                }

                m_pcEntropyCoder->encodeQtCbf( TUIterator, compID, true );

                if (isCrossCPredictionAvailable)
                {
                  m_pcEntropyCoder->encodeCrossComponentPrediction( TUIterator, compID );
                }

                m_pcEntropyCoder->encodeCoeffNxN( TUIterator, currentCoefficients, compID );
                currCompBits = m_pcEntropyCoder->getNumberOfWrittenBits();

                pcResiCurrComp = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 );

                m_pcTrQuant->invTransformNxN( TUIterator, compID, pcResiCurrComp, m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID), currentCoefficients, cQP DEBUG_STRING_PASS_INTO_OPTIONAL(&sSingleStringTest, (DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask)) );

                if (bUseCrossCPrediction)
                {
                  TComTrQuant::crossComponentPrediction(TUIterator,
                                                        compID,
                                                        pLumaResi,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                        tuCompRect.width,
                                                        tuCompRect.height,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID     ),
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID     ),
                                                        true);
                }

                currCompDist = m_pcRdCost->getDistPart( channelBitDepth, m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID),
                                                        pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                        pcResi->getStride(compID),
                                                        tuCompRect.width, tuCompRect.height, compID);

                currCompCost = m_pcRdCost->calcRdCost(currCompBits, currCompDist);
                  
                if (pcCU->isLosslessCoded(0))
                {
                  nonCoeffCost = MAX_DOUBLE;
                }
              }
              else if ((transformSkipModeId == 1) && !bUseCrossCPrediction)
              {
                currCompCost = MAX_DOUBLE;
              }
              else
              {
                currCompBits = nonCoeffBits;
                currCompDist = nonCoeffDist;
                currCompCost = nonCoeffCost;
              }

              // evaluate
              if ((currCompCost < minCost[compID][subTUIndex]) || ((transformSkipModeId == 1) && (currCompCost == minCost[compID][subTUIndex])))
              {
                bestExplicitRdpcmModeUnSplit[compID][subTUIndex] = pcCU->getExplicitRdpcmMode(compID, subTUAbsPartIdx);

                if(isFirstMode) //check for forced null
                {
                  if((nonCoeffCost < currCompCost) || (currAbsSum == 0))
                  {
                    memset(currentCoefficients, 0, (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));

                    currAbsSum   = 0;
                    currCompBits = nonCoeffBits;
                    currCompDist = nonCoeffDist;
                    currCompCost = nonCoeffCost;
                  }
                }

#if DEBUG_STRING
                if (currAbsSum > 0)
                {
                  DEBUG_STRING_SWAP(sSingleStringComp[compID], sSingleStringTest)
                }
                else
                {
                  sSingleStringComp[compID].clear();
                }
#endif

                uiAbsSum                 [compID][subTUIndex] = currAbsSum;
                uiSingleDistComp         [compID][subTUIndex] = currCompDist;
                minCost                  [compID][subTUIndex] = currCompCost;
                uiBestTransformMode      [compID][subTUIndex] = transformSkipModeId;
                bestCrossCPredictionAlpha[compID][subTUIndex] = (crossCPredictionModeId == 1) ? pcCU->getCrossComponentPredictionAlpha(subTUAbsPartIdx, compID) : 0;

                if (uiAbsSum[compID][subTUIndex] == 0)
                {
                  if (bUseCrossCPrediction)
                  {
                    TComTrQuant::crossComponentPrediction(TUIterator,
                                                          compID,
                                                          pLumaResi,
                                                          m_pTempPel,
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                          tuCompRect.width,
                                                          tuCompRect.height,
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                          tuCompRect.width,
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID),
                                                          true);
                  }
                  else
                  {
                    pcResiCurrComp = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0);
                    const UInt uiStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID);
                    for(UInt uiY = 0; uiY < tuCompRect.height; uiY++)
                    {
                      memset(pcResiCurrComp, 0, (sizeof(Pel) * tuCompRect.width));
                      pcResiCurrComp += uiStride;
                    }
                  }
                }
              }
              else
              {
                // reset
                memcpy(currentCoefficients,    bestCoeffComp,    (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#if ADAPTIVE_QP_SELECTION
                memcpy(currentARLCoefficients, bestArlCoeffComp, (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#endif
                for (Int y = 0; y < tuCompRect.height; y++)
                {
                  memcpy((pcResiCurrComp + (y * resiStride)), &bestResiComp[y * tuCompRect.width], (sizeof(Pel) * tuCompRect.width));
                }
              }
            }
          }

          pcCU->setExplicitRdpcmModePartRange            (   bestExplicitRdpcmModeUnSplit[compID][subTUIndex],                            compID, subTUAbsPartIdx, partIdxesPerSubTU);
          pcCU->setTransformSkipPartRange                (   uiBestTransformMode         [compID][subTUIndex],                            compID, subTUAbsPartIdx, partIdxesPerSubTU );
          pcCU->setCbfPartRange                          ((((uiAbsSum                    [compID][subTUIndex] > 0) ? 1 : 0) << uiTrMode), compID, subTUAbsPartIdx, partIdxesPerSubTU );
          pcCU->setCrossComponentPredictionAlphaPartRange(   bestCrossCPredictionAlpha   [compID][subTUIndex],                            compID, subTUAbsPartIdx, partIdxesPerSubTU );
        } while (TUIterator.nextSection(rTu)); //end of sub-TU loop
      } // processing section
    } // component loop

    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID = ComponentID(ch);
      if (rTu.ProcessComponentSection(compID) && (rTu.getRect(compID).width != rTu.getRect(compID).height))
      {
        offsetSubTUCBFs(rTu, compID); //the CBFs up to now have been defined for two sub-TUs - shift them down a level and replace with the parent level CBF
      }
    }

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    m_pcEntropyCoder->resetBits();

    if( uiLog2TrSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
    {
      m_pcEntropyCoder->encodeTransformSubdivFlag( 0, 5 - uiLog2TrSize );
    }

    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      const UInt chOrderChange = ((ch + 1) == numValidComp) ? 0 : (ch + 1);
      const ComponentID compID=ComponentID(chOrderChange);
      if( rTu.ProcessComponentSection(compID) )
      {
        m_pcEntropyCoder->encodeQtCbf( rTu, compID, true );
      }
    }

    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID=ComponentID(ch);
      if (rTu.ProcessComponentSection(compID))
      {
        if(isChroma(compID) && (uiAbsSum[COMPONENT_Y][0] != 0))
        {
          m_pcEntropyCoder->encodeCrossComponentPrediction( rTu, compID );
        }

        m_pcEntropyCoder->encodeCoeffNxN( rTu, pcCoeffCurr[compID], compID );
        for (UInt subTUIndex = 0; subTUIndex < 2; subTUIndex++)
        {
          uiSingleDist += uiSingleDistComp[compID][subTUIndex];
        }
      }
    }

    uiSingleBits = m_pcEntropyCoder->getNumberOfWrittenBits();

    dSingleCost = m_pcRdCost->calcRdCost( uiSingleBits, uiSingleDist );
  } // check full

  // code sub-blocks
  if( bCheckSplit )
  {
    if( bCheckFull )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_TEST ] );
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    Distortion uiSubdivDist = 0;
    UInt       uiSubdivBits = 0;
    Double     dSubdivCost = 0.0;

    //save the non-split CBFs in case we need to restore them later

    UInt bestCBF     [MAX_NUM_COMPONENT];
    UInt bestsubTUCBF[MAX_NUM_COMPONENT][2];
    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID=ComponentID(ch);

      if (rTu.ProcessComponentSection(compID))
      {
        bestCBF[compID] = pcCU->getCbf(uiAbsPartIdx, compID, uiTrMode);

        const TComRectangle &tuCompRect = rTu.getRect(compID);
        if (tuCompRect.width != tuCompRect.height)
        {
          const UInt partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID) >> 1;

          for (UInt subTU = 0; subTU < 2; subTU++)
          {
            bestsubTUCBF[compID][subTU] = pcCU->getCbf ((uiAbsPartIdx + (subTU * partIdxesPerSubTU)), compID, subTUDepth);
          }
        }
      }
    }


    TComTURecurse tuRecurseChild(rTu, false);
    const UInt uiQPartNumSubdiv = tuRecurseChild.GetAbsPartIdxNumParts();

    DEBUG_STRING_NEW(sSplitString[MAX_NUM_COMPONENT])

    do
    {
      DEBUG_STRING_NEW(childString)
      xEstimateInterResidualQT( pcResi, dSubdivCost, uiSubdivBits, uiSubdivDist, bCheckFull ? NULL : puiZeroDist,  tuRecurseChild DEBUG_STRING_PASS_INTO(childString));
#if DEBUG_STRING
      // split the string by component and append to the relevant output (because decoder decodes in channel order, whereas this search searches by TU-order)
      std::size_t lastPos=0;
      const std::size_t endStrng=childString.find(debug_reorder_data_inter_token[MAX_NUM_COMPONENT], lastPos);
      for(UInt ch = 0; ch < numValidComp; ch++)
      {
        if (lastPos!=std::string::npos && childString.find(debug_reorder_data_inter_token[ch], lastPos)==lastPos)
        {
          lastPos+=strlen(debug_reorder_data_inter_token[ch]); // skip leading string
        }
        std::size_t pos=childString.find(debug_reorder_data_inter_token[ch+1], lastPos);
        if (pos!=std::string::npos && pos>endStrng)
        {
          lastPos=endStrng;
        }
        sSplitString[ch]+=childString.substr(lastPos, (pos==std::string::npos)? std::string::npos : (pos-lastPos) );
        lastPos=pos;
      }
#endif
    } while ( tuRecurseChild.nextSection(rTu) ) ;

    UInt uiCbfAny=0;
    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      UInt uiYUVCbf = 0;
      for( UInt ui = 0; ui < 4; ++ui )
      {
        uiYUVCbf |= pcCU->getCbf( uiAbsPartIdx + ui * uiQPartNumSubdiv, ComponentID(ch),  uiTrMode + 1 );
      }
      UChar *pBase=pcCU->getCbf( ComponentID(ch) );
      const UInt flags=uiYUVCbf << uiTrMode;
      for( UInt ui = 0; ui < 4 * uiQPartNumSubdiv; ++ui )
      {
        pBase[uiAbsPartIdx + ui] |= flags;
      }
      uiCbfAny|=uiYUVCbf;
    }

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    m_pcEntropyCoder->resetBits();

    // when compID isn't a channel, code Cbfs:
    xEncodeInterResidualQT( MAX_NUM_COMPONENT, rTu );
    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      xEncodeInterResidualQT( ComponentID(ch), rTu );
    }

    uiSubdivBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    dSubdivCost  = m_pcRdCost->calcRdCost( uiSubdivBits, uiSubdivDist );

    if (!bCheckFull || (uiCbfAny && (dSubdivCost < dSingleCost)))
    {
      rdCost += dSubdivCost;
      ruiBits += uiSubdivBits;
      ruiDist += uiSubdivDist;
#if DEBUG_STRING
      for(UInt ch = 0; ch < numValidComp; ch++)
      {
        DEBUG_STRING_APPEND(sDebug, debug_reorder_data_inter_token[ch])
        DEBUG_STRING_APPEND(sDebug, sSplitString[ch])
      }
#endif
    }
    else
    {
      rdCost  += dSingleCost;
      ruiBits += uiSingleBits;
      ruiDist += uiSingleDist;

      //restore state to unsplit

      pcCU->setTrIdxSubParts( uiTrMode, uiAbsPartIdx, uiDepth );

      for(UInt ch = 0; ch < numValidComp; ch++)
      {
        const ComponentID compID=ComponentID(ch);

        DEBUG_STRING_APPEND(sDebug, debug_reorder_data_inter_token[ch])
        if (rTu.ProcessComponentSection(compID))
        {
          DEBUG_STRING_APPEND(sDebug, sSingleStringComp[compID])

          const Bool splitIntoSubTUs   = rTu.getRect(compID).width != rTu.getRect(compID).height;
          const UInt numberOfSections  = splitIntoSubTUs ? 2 : 1;
          const UInt partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID) >> (splitIntoSubTUs ? 1 : 0);

          for (UInt subTUIndex = 0; subTUIndex < numberOfSections; subTUIndex++)
          {
            const UInt  uisubTUPartIdx = uiAbsPartIdx + (subTUIndex * partIdxesPerSubTU);

            if (splitIntoSubTUs)
            {
              const UChar combinedCBF = (bestsubTUCBF[compID][subTUIndex] << subTUDepth) | (bestCBF[compID] << uiTrMode);
              pcCU->setCbfPartRange(combinedCBF, compID, uisubTUPartIdx, partIdxesPerSubTU);
            }
            else
            {
              pcCU->setCbfPartRange((bestCBF[compID] << uiTrMode), compID, uisubTUPartIdx, partIdxesPerSubTU);
            }

            pcCU->setCrossComponentPredictionAlphaPartRange(bestCrossCPredictionAlpha[compID][subTUIndex], compID, uisubTUPartIdx, partIdxesPerSubTU);
            pcCU->setTransformSkipPartRange(uiBestTransformMode[compID][subTUIndex], compID, uisubTUPartIdx, partIdxesPerSubTU);
            pcCU->setExplicitRdpcmModePartRange(bestExplicitRdpcmModeUnSplit[compID][subTUIndex], compID, uisubTUPartIdx, partIdxesPerSubTU);
          }
        }
      }

      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_TEST ] );
    }
  }
  else
  {
    rdCost  += dSingleCost;
    ruiBits += uiSingleBits;
    ruiDist += uiSingleDist;
#if DEBUG_STRING
    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID=ComponentID(ch);
      DEBUG_STRING_APPEND(sDebug, debug_reorder_data_inter_token[compID])

      if (rTu.ProcessComponentSection(compID))
      {
        DEBUG_STRING_APPEND(sDebug, sSingleStringComp[compID])
      }
    }
#endif
  }
  DEBUG_STRING_APPEND(sDebug, debug_reorder_data_inter_token[MAX_NUM_COMPONENT])
}



Void TEncSearch::xEncodeInterResidualQT( const ComponentID compID, TComTU &rTu )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();
  const UInt uiCurrTrMode = rTu.GetTransformDepthRel();
  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiTrMode = pcCU->getTransformIdx( uiAbsPartIdx );

  const Bool bSubdiv = uiCurrTrMode != uiTrMode;

  const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();

  if (compID==MAX_NUM_COMPONENT)  // we are not processing a channel, instead we always recurse and code the CBFs
  {
    if( uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() && uiLog2TrSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
    {
      if((pcCU->getSlice()->getSPS()->getQuadtreeTUMaxDepthInter() == 1) && (pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N))
      {
        assert(bSubdiv); // Inferred splitting rule - see derivation and use of interSplitFlag in the specification.
      }
      else
      {
        m_pcEntropyCoder->encodeTransformSubdivFlag( bSubdiv, 5 - uiLog2TrSize );
      }
    }

    assert( !pcCU->isIntra(uiAbsPartIdx) );

    const Bool bFirstCbfOfCU = uiCurrTrMode == 0;

    for (UInt ch=COMPONENT_Cb; ch<pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      const ComponentID compIdInner=ComponentID(ch);
      if( bFirstCbfOfCU || rTu.ProcessingAllQuadrants(compIdInner) )
      {
        if( bFirstCbfOfCU || pcCU->getCbf( uiAbsPartIdx, compIdInner, uiCurrTrMode - 1 ) )
        {
          m_pcEntropyCoder->encodeQtCbf( rTu, compIdInner, !bSubdiv );
        }
      }
      else
      {
        assert( pcCU->getCbf( uiAbsPartIdx, compIdInner, uiCurrTrMode ) == pcCU->getCbf( uiAbsPartIdx, compIdInner, uiCurrTrMode - 1 ) );
      }
    }

    if (!bSubdiv)
    {
      m_pcEntropyCoder->encodeQtCbf( rTu, COMPONENT_Y, true );
    }
  }

  if( !bSubdiv )
  {
    if (compID != MAX_NUM_COMPONENT) // we have already coded the CBFs, so now we code coefficients
    {
      if (rTu.ProcessComponentSection(compID))
      {
        if (isChroma(compID) && (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y, uiTrMode) != 0))
        {
          m_pcEntropyCoder->encodeCrossComponentPrediction(rTu, compID);
        }

        if (pcCU->getCbf(uiAbsPartIdx, compID, uiTrMode) != 0)
        {
          const UInt uiQTTempAccessLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
          TCoeff *pcCoeffCurr = m_ppcQTTempCoeff[compID][uiQTTempAccessLayer] + rTu.getCoefficientOffset(compID);
          m_pcEntropyCoder->encodeCoeffNxN( rTu, pcCoeffCurr, compID );
        }
      }
    }
  }
  else
  {
    if( compID==MAX_NUM_COMPONENT || pcCU->getCbf( uiAbsPartIdx, compID, uiCurrTrMode ) )
    {
      TComTURecurse tuRecurseChild(rTu, false);
      do
      {
        xEncodeInterResidualQT( compID, tuRecurseChild );
      } while (tuRecurseChild.nextSection(rTu));
    }
  }
}




Void TEncSearch::xSetInterResidualQTData( TComYuv* pcResi, Bool bSpatial, TComTU &rTu ) // TODO: turn this into two functions for bSpatial=true and false.
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiCurrTrMode=rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();
  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiTrMode = pcCU->getTransformIdx( uiAbsPartIdx );
  const TComSPS *sps=pcCU->getSlice()->getSPS();

  if( uiCurrTrMode == uiTrMode )
  {
    const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    const UInt uiQTTempAccessLayer = sps->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    if( bSpatial )
    {
      // Data to be copied is in the spatial domain, i.e., inverse-transformed.

      for(UInt i=0; i<pcResi->getNumberValidComponents(); i++)
      {
        const ComponentID compID=ComponentID(i);
        if (rTu.ProcessComponentSection(compID))
        {
          const TComRectangle &rectCompTU(rTu.getRect(compID));
          m_pcQTTempTComYuv[uiQTTempAccessLayer].copyPartToPartComponentMxN    ( compID, pcResi, rectCompTU );
        }
      }
    }
    else
    {
      for (UInt ch=0; ch < getNumberValidComponents(sps->getChromaFormatIdc()); ch++)
      {
        const ComponentID compID   = ComponentID(ch);
        if (rTu.ProcessComponentSection(compID))
        {
          const TComRectangle &rectCompTU(rTu.getRect(compID));
          const UInt numCoeffInBlock    = rectCompTU.width * rectCompTU.height;
          const UInt offset             = rTu.getCoefficientOffset(compID);
          TCoeff* dest                  = pcCU->getCoeff(compID)                        + offset;
          const TCoeff* src             = m_ppcQTTempCoeff[compID][uiQTTempAccessLayer] + offset;
          ::memcpy( dest, src, sizeof(TCoeff)*numCoeffInBlock );

#if ADAPTIVE_QP_SELECTION
          TCoeff* pcArlCoeffSrc            = m_ppcQTTempArlCoeff[compID][uiQTTempAccessLayer] + offset;
          TCoeff* pcArlCoeffDst            = pcCU->getArlCoeff(compID)                        + offset;
          ::memcpy( pcArlCoeffDst, pcArlCoeffSrc, sizeof( TCoeff ) * numCoeffInBlock );
#endif
        }
      }
    }
  }
  else
  {

    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xSetInterResidualQTData( pcResi, bSpatial, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}




UInt TEncSearch::xModeBitsIntra( TComDataCU* pcCU, UInt uiMode, UInt uiPartOffset, UInt uiDepth, const ChannelType chType )
{
  // Reload only contexts required for coding intra mode information
  m_pcRDGoOnSbacCoder->loadIntraDirMode( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST], chType );

  // Temporarily set the intra dir being tested, and only
  // for absPartIdx, since encodeIntraDirModeLuma/Chroma only use
  // the entry at absPartIdx.

  UChar &rIntraDirVal=pcCU->getIntraDir( chType )[uiPartOffset];
  UChar origVal=rIntraDirVal;
  rIntraDirVal = uiMode;
  //pcCU->setIntraDirSubParts ( chType, uiMode, uiPartOffset, uiDepth + uiInitTrDepth );

  m_pcEntropyCoder->resetBits();
  if (isLuma(chType))
  {
    m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiPartOffset);
  }
  else
  {
    m_pcEntropyCoder->encodeIntraDirModeChroma ( pcCU, uiPartOffset);
  }

  rIntraDirVal = origVal; // restore

  return m_pcEntropyCoder->getNumberOfWrittenBits();
}




UInt TEncSearch::xUpdateCandList( UInt uiMode, Double uiCost, UInt uiFastCandNum, UInt * CandModeList, Double * CandCostList )
{
  UInt i;
  UInt shift=0;

  while ( shift<uiFastCandNum && uiCost<CandCostList[ uiFastCandNum-1-shift ] )
  {
    shift++;
  }

  if( shift!=0 )
  {
    for(i=1; i<shift; i++)
    {
      CandModeList[ uiFastCandNum-i ] = CandModeList[ uiFastCandNum-1-i ];
      CandCostList[ uiFastCandNum-i ] = CandCostList[ uiFastCandNum-1-i ];
    }
    CandModeList[ uiFastCandNum-shift ] = uiMode;
    CandCostList[ uiFastCandNum-shift ] = uiCost;
    return 1;
  }

  return 0;
}





/** add inter-prediction syntax elements for a CU block
 * \param pcCU
 * \param uiQp
 * \param uiTrMode
 * \param ruiBits
 * \returns Void
 */
Void  TEncSearch::xAddSymbolBitsInter( TComDataCU* pcCU, UInt& ruiBits )
{
  if(pcCU->getMergeFlag( 0 ) && pcCU->getPartitionSize( 0 ) == SIZE_2Nx2N && !pcCU->getQtRootCbf( 0 ))
  {
    pcCU->setSkipFlagSubParts( true, 0, pcCU->getDepth(0) );

    m_pcEntropyCoder->resetBits();
    if(pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
    }
    m_pcEntropyCoder->encodeSkipFlag(pcCU, 0, true);
    m_pcEntropyCoder->encodeMergeIndex(pcCU, 0, true);

    ruiBits += m_pcEntropyCoder->getNumberOfWrittenBits();
  }
  else
  {
    m_pcEntropyCoder->resetBits();

    if(pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
    }

    m_pcEntropyCoder->encodeSkipFlag ( pcCU, 0, true );
    m_pcEntropyCoder->encodePredMode( pcCU, 0, true );
    m_pcEntropyCoder->encodePartSize( pcCU, 0, pcCU->getDepth(0), true );
    m_pcEntropyCoder->encodePredInfo( pcCU, 0 );

    Bool codeDeltaQp = false;
    Bool codeChromaQpAdj = false;
    m_pcEntropyCoder->encodeCoeff   ( pcCU, 0, pcCU->getDepth(0), codeDeltaQp, codeChromaQpAdj );

    ruiBits += m_pcEntropyCoder->getNumberOfWrittenBits();
  }
}





/**
 * \brief Generate half-sample interpolated block
 *
 * \param pattern Reference picture ROI
 * \param biPred    Flag indicating whether block is for biprediction
 */
Void TEncSearch::xExtDIFUpSamplingH( TComPattern* pattern )
{
  Int width      = pattern->getROIYWidth();
  Int height     = pattern->getROIYHeight();
  Int srcStride  = pattern->getPatternLStride();

  Int intStride = m_filteredBlockTmp[0].getStride(COMPONENT_Y);
  Int dstStride = m_filteredBlock[0][0].getStride(COMPONENT_Y);
  Pel *intPtr;
  Pel *dstPtr;
  Int filterSize = NTAPS_LUMA;
  Int halfFilterSize = (filterSize>>1);
  Pel *srcPtr = pattern->getROIY() - halfFilterSize*srcStride - 1;

  const ChromaFormat chFmt = m_filteredBlock[0][0].getChromaFormat();

  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[0].getAddr(COMPONENT_Y), intStride, width+1, height+filterSize, 0, false, chFmt, pattern->getBitDepthY());
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[2].getAddr(COMPONENT_Y), intStride, width+1, height+filterSize, 2, false, chFmt, pattern->getBitDepthY());

  intPtr = m_filteredBlockTmp[0].getAddr(COMPONENT_Y) + halfFilterSize * intStride + 1;
  dstPtr = m_filteredBlock[0][0].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+0, height+0, 0, false, true, chFmt, pattern->getBitDepthY());

  intPtr = m_filteredBlockTmp[0].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride + 1;
  dstPtr = m_filteredBlock[2][0].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+0, height+1, 2, false, true, chFmt, pattern->getBitDepthY());

  intPtr = m_filteredBlockTmp[2].getAddr(COMPONENT_Y) + halfFilterSize * intStride;
  dstPtr = m_filteredBlock[0][2].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+1, height+0, 0, false, true, chFmt, pattern->getBitDepthY());

  intPtr = m_filteredBlockTmp[2].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[2][2].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+1, height+1, 2, false, true, chFmt, pattern->getBitDepthY());
}





/**
 * \brief Generate quarter-sample interpolated blocks
 *
 * \param pattern    Reference picture ROI
 * \param halfPelRef Half-pel mv
 * \param biPred     Flag indicating whether block is for biprediction
 */
Void TEncSearch::xExtDIFUpSamplingQ( TComPattern* pattern, TComMv halfPelRef )
{
  Int width      = pattern->getROIYWidth();
  Int height     = pattern->getROIYHeight();
  Int srcStride  = pattern->getPatternLStride();

  Pel *srcPtr;
  Int intStride = m_filteredBlockTmp[0].getStride(COMPONENT_Y);
  Int dstStride = m_filteredBlock[0][0].getStride(COMPONENT_Y);
  Pel *intPtr;
  Pel *dstPtr;
  Int filterSize = NTAPS_LUMA;

  Int halfFilterSize = (filterSize>>1);

  Int extHeight = (halfPelRef.getVer() == 0) ? height + filterSize : height + filterSize-1;

  const ChromaFormat chFmt = m_filteredBlock[0][0].getChromaFormat();

  // Horizontal filter 1/4
  srcPtr = pattern->getROIY() - halfFilterSize * srcStride - 1;
  intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y);
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() >= 0)
  {
    srcPtr += 1;
  }
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 1, false, chFmt, pattern->getBitDepthY());

  // Horizontal filter 3/4
  srcPtr = pattern->getROIY() - halfFilterSize*srcStride - 1;
  intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y);
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() > 0)
  {
    srcPtr += 1;
  }
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 3, false, chFmt, pattern->getBitDepthY());

  // Generate @ 1,1
  intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[1][1].getAddr(COMPONENT_Y);
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, pattern->getBitDepthY());

  // Generate @ 3,1
  intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[3][1].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, pattern->getBitDepthY());

  if (halfPelRef.getVer() != 0)
  {
    // Generate @ 2,1
    intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[2][1].getAddr(COMPONENT_Y);
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true, chFmt, pattern->getBitDepthY());

    // Generate @ 2,3
    intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[2][3].getAddr(COMPONENT_Y);
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true, chFmt, pattern->getBitDepthY());
  }
  else
  {
    // Generate @ 0,1
    intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][1].getAddr(COMPONENT_Y);
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true, chFmt, pattern->getBitDepthY());

    // Generate @ 0,3
    intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][3].getAddr(COMPONENT_Y);
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true, chFmt, pattern->getBitDepthY());
  }

  if (halfPelRef.getHor() != 0)
  {
    // Generate @ 1,2
    intPtr = m_filteredBlockTmp[2].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[1][2].getAddr(COMPONENT_Y);
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, pattern->getBitDepthY());

    // Generate @ 3,2
    intPtr = m_filteredBlockTmp[2].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[3][2].getAddr(COMPONENT_Y);
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, pattern->getBitDepthY());
  }
  else
  {
    // Generate @ 1,0
    intPtr = m_filteredBlockTmp[0].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride + 1;
    dstPtr = m_filteredBlock[1][0].getAddr(COMPONENT_Y);
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, pattern->getBitDepthY());

    // Generate @ 3,0
    intPtr = m_filteredBlockTmp[0].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride + 1;
    dstPtr = m_filteredBlock[3][0].getAddr(COMPONENT_Y);
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, pattern->getBitDepthY());
  }

  // Generate @ 1,3
  intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[1][3].getAddr(COMPONENT_Y);
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, pattern->getBitDepthY());

  // Generate @ 3,3
  intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[3][3].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, pattern->getBitDepthY());
}





//! set wp tables
Void  TEncSearch::setWpScalingDistParam( TComDataCU* pcCU, Int iRefIdx, RefPicList eRefPicListCur )
{
  if ( iRefIdx<0 )
  {
    m_cDistParam.bApplyWeight = false;
    return;
  }

  TComSlice       *pcSlice  = pcCU->getSlice();
  WPScalingParam  *wp0 , *wp1;

  m_cDistParam.bApplyWeight = ( pcSlice->getSliceType()==P_SLICE && pcSlice->testWeightPred() ) || ( pcSlice->getSliceType()==B_SLICE && pcSlice->testWeightBiPred() ) ;

  if ( !m_cDistParam.bApplyWeight )
  {
    return;
  }

  Int iRefIdx0 = ( eRefPicListCur == REF_PIC_LIST_0 ) ? iRefIdx : (-1);
  Int iRefIdx1 = ( eRefPicListCur == REF_PIC_LIST_1 ) ? iRefIdx : (-1);

  getWpScaling( pcCU, iRefIdx0, iRefIdx1, wp0 , wp1 );

  if ( iRefIdx0 < 0 )
  {
    wp0 = NULL;
  }
  if ( iRefIdx1 < 0 )
  {
    wp1 = NULL;
  }

  m_cDistParam.wpCur  = NULL;

  if ( eRefPicListCur == REF_PIC_LIST_0 )
  {
    m_cDistParam.wpCur = wp0;
  }
  else
  {
    m_cDistParam.wpCur = wp1;
  }
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

/** \file     TEncSearch.cpp
 \brief    encoder search class
 */

#include "../TLibCommon/TypeDef.h"
#include "../TLibCommon/TComMotionInfo.h"
#include "TEncSearch.h"

#ifdef ROUNDING_CONTROL_BIPRED
#ifndef ROUNDING_CONTROL_BIPRED_FIX
__inline Pel  xClip  (Pel x )      { return ( (x < 0) ? 0 : (x > (Pel)g_uiIBDI_MAX) ? (Pel)g_uiIBDI_MAX : x ); }
#endif
#endif

#ifdef DCM_PBIC
extern Int entropyBits[128];
#endif

static TComMv s_acMvRefineH[9] =
{
  TComMv(  0,  0 ), // 0
  TComMv(  0, -1 ), // 1
  TComMv(  0,  1 ), // 2
  TComMv( -1,  0 ), // 3
  TComMv(  1,  0 ), // 4
  TComMv( -1, -1 ), // 5
  TComMv(  1, -1 ), // 6
  TComMv( -1,  1 ), // 7
  TComMv(  1,  1 )  // 8
};


#ifdef QC_SIFO
static TComMv s_acMvRefineQ[16] =
{
  TComMv(  0,  0 ), // 0
  TComMv(  0, -1 ), // 1
  TComMv(  0,  1 ), // 2
  TComMv( -1, -1 ), // 5
  TComMv(  1, -1 ), // 6
  TComMv( -1,  0 ), // 3
  TComMv(  1,  0 ), // 4
  TComMv( -1,  1 ), // 7
  TComMv(  1,  1 ), // 8
  TComMv( -1, -2 ), // 9
  TComMv(  0, -2 ), // 10
  TComMv(  1, -2 ), // 11
  TComMv( -2, -1 ), // 12
  TComMv( -2,  0 ), // 13
  TComMv( -2,  1 ), // 14
  TComMv( -2, -2 )  // 15
};
#else
static TComMv s_acMvRefineQ[9] =
{
  TComMv(  0,  0 ), // 0
  TComMv(  0, -1 ), // 1
  TComMv(  0,  1 ), // 2
  TComMv( -1, -1 ), // 5
  TComMv(  1, -1 ), // 6
  TComMv( -1,  0 ), // 3
  TComMv(  1,  0 ), // 4
  TComMv( -1,  1 ), // 7
  TComMv(  1,  1 )  // 8
};
#endif

static UInt s_auiDFilter[9] =
{
  0, 1, 0,
  2, 3, 2,
  0, 1, 0
};

TEncSearch::TEncSearch()
{
#if HHI_RQT
  m_ppcQTTempCoeffY  = NULL;
  m_ppcQTTempCoeffCb = NULL;
  m_ppcQTTempCoeffCr = NULL;
  m_pcQTTempCoeffY   = NULL;
  m_pcQTTempCoeffCb  = NULL;
  m_pcQTTempCoeffCr  = NULL;
  m_puhQTTempTrIdx   = NULL;
  m_puhQTTempCbf[0] = m_puhQTTempCbf[1] = m_puhQTTempCbf[2] = NULL;
  m_pcQTTempTComYuv  = NULL;
#endif
  m_pcEncCfg = NULL;
  m_pcEntropyCoder = NULL;
  m_pTempPel = NULL;
}

TEncSearch::~TEncSearch()
{
  if ( m_pTempPel )
  {
    delete [] m_pTempPel;
    m_pTempPel = NULL;
  }
#if HHI_RQT
  if( m_pcEncCfg && m_pcEncCfg->getQuadtreeTUFlag() )
  {
    const UInt uiNumLayersAllocated = m_pcEncCfg->getQuadtreeTULog2MaxSize()-m_pcEncCfg->getQuadtreeTULog2MinSize()+1;
    for( UInt ui = 0; ui < uiNumLayersAllocated; ++ui )
    {
      delete[] m_ppcQTTempCoeffY[ui];
      delete[] m_ppcQTTempCoeffCb[ui];
      delete[] m_ppcQTTempCoeffCr[ui];
      m_pcQTTempTComYuv[ui].destroy();
    }
    delete[] m_ppcQTTempCoeffY;
    delete[] m_ppcQTTempCoeffCb;
    delete[] m_ppcQTTempCoeffCr;
    delete[] m_pcQTTempCoeffY;
    delete[] m_pcQTTempCoeffCb;
    delete[] m_pcQTTempCoeffCr;
    delete[] m_puhQTTempTrIdx;
    delete[] m_puhQTTempCbf[0];
    delete[] m_puhQTTempCbf[1];
    delete[] m_puhQTTempCbf[2];
    delete[] m_pcQTTempTComYuv;
  }
#endif
}

void TEncSearch::init(  TEncCfg*      pcEncCfg,
                      TComTrQuant*  pcTrQuant,
                      Int           iSearchRange,
                      Int           iFastSearch,
                      Int           iMaxDeltaQP,
                      TEncEntropy*  pcEntropyCoder,
                      TComRdCost*   pcRdCost,
                      TEncSbac*** pppcRDSbacCoder,
                      TEncSbac*   pcRDGoOnSbacCoder
                      )
{
  m_pcEncCfg             = pcEncCfg;
  m_pcTrQuant            = pcTrQuant;
  m_iSearchRange         = iSearchRange;
  m_iFastSearch          = iFastSearch;
  m_iMaxDeltaQP          = iMaxDeltaQP;
  m_pcEntropyCoder       = pcEntropyCoder;
  m_pcRdCost             = pcRdCost;
  
  m_pppcRDSbacCoder     = pppcRDSbacCoder;
  m_pcRDGoOnSbacCoder   = pcRDGoOnSbacCoder;
  
  m_bUseSBACRD          = pppcRDSbacCoder ? true : false;
  
  for (Int iDir = 0; iDir < 2; iDir++)
    for (Int iRefIdx = 0; iRefIdx < 33; iRefIdx++)
    {
      m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange;
    }
  
  m_puiDFilter = s_auiDFilter + 4;
  
  // initialize motion cost
  m_pcRdCost->initRateDistortionModel( m_iSearchRange << 2 );
  
  for( Int iNum = 0; iNum < AMVP_MAX_NUM_CANDS+1; iNum++)
    for( Int iIdx = 0; iIdx < AMVP_MAX_NUM_CANDS; iIdx++)
    {
      if (iIdx < iNum)
        m_auiMVPIdxCost[iIdx][iNum] = xGetMvpIdxBits(iIdx, iNum);
      else
        m_auiMVPIdxCost[iIdx][iNum] = MAX_INT;
    }
  
#ifdef DCM_PBIC
    for( Int iNum = 0; iNum < AICP_MAX_NUM_CANDS+1; iNum++)
      for( Int iIdx = 0; iIdx < AICP_MAX_NUM_CANDS; iIdx++)
      {
        if (iIdx < iNum)
          m_auiICPIdxCost[iIdx][iNum] = xGetIcpIdxBits(iIdx, iNum);
        else
          m_auiICPIdxCost[iIdx][iNum] = MAX_INT;
      }
#endif

  initTempBuff();
  
  m_pTempPel = new Pel[g_uiMaxCUWidth*g_uiMaxCUHeight];
  
  m_iDIFTap2 = (m_iDIFTap << 1);
  
#if HHI_RQT
  if( pcEncCfg->getQuadtreeTUFlag() )
  {
    const UInt uiNumLayersToAllocate = pcEncCfg->getQuadtreeTULog2MaxSize()-pcEncCfg->getQuadtreeTULog2MinSize()+1;
    m_ppcQTTempCoeffY  = new TCoeff*[uiNumLayersToAllocate];
    m_ppcQTTempCoeffCb = new TCoeff*[uiNumLayersToAllocate];
    m_ppcQTTempCoeffCr = new TCoeff*[uiNumLayersToAllocate];
    m_pcQTTempCoeffY   = new TCoeff [g_uiMaxCUWidth*g_uiMaxCUHeight   ];
    m_pcQTTempCoeffCb  = new TCoeff [g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
    m_pcQTTempCoeffCr  = new TCoeff [g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
    
    const UInt uiNumPartitions = 1<<(g_uiMaxCUDepth<<1);
    m_puhQTTempTrIdx   = new UChar  [uiNumPartitions];
    m_puhQTTempCbf[0]  = new UChar  [uiNumPartitions];
    m_puhQTTempCbf[1]  = new UChar  [uiNumPartitions];
    m_puhQTTempCbf[2]  = new UChar  [uiNumPartitions];
    m_pcQTTempTComYuv  = new TComYuv[uiNumLayersToAllocate];
    for( UInt ui = 0; ui < uiNumLayersToAllocate; ++ui )
    {
      m_ppcQTTempCoeffY[ui]  = new TCoeff[g_uiMaxCUWidth*g_uiMaxCUHeight   ];
      m_ppcQTTempCoeffCb[ui] = new TCoeff[g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
      m_ppcQTTempCoeffCr[ui] = new TCoeff[g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
      m_pcQTTempTComYuv[ui].create( g_uiMaxCUWidth, g_uiMaxCUHeight );
    }
  }
#endif
}

#if FASTME_SMOOTHER_MV
#define FIRSTSEARCHSTOP     1
#else
#define FIRSTSEARCHSTOP     0
#endif

#define TZ_SEARCH_CONFIGURATION                                                                                 \
const Int  iRaster                  = 3;  /* TZ soll von aussen ?ergeben werden */                            \
const Bool bTestOtherPredictedMV    = 1;                                                                      \
const Bool bTestZeroVector          = 1;                                                                      \
const Bool bTestZeroVectorStart     = 0;                                                                      \
const Bool bTestZeroVectorStop      = 0;                                                                      \
const Bool bFirstSearchDiamond      = 1;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bFirstSearchStop         = FIRSTSEARCHSTOP;                                                        \
const UInt uiFirstSearchRounds      = 3;  /* first search stop X rounds after best match (must be >=1) */     \
const Bool bEnableRasterSearch      = 1;                                                                      \
const Bool bAlwaysRasterSearch      = 0;  /* ===== 1: BETTER but factor 2 slower ===== */                     \
const Bool bRasterRefinementEnable  = 0;  /* enable either raster refinement or star refinement */            \
const Bool bRasterRefinementDiamond = 0;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bStarRefinementEnable    = 1;  /* enable either star refinement or raster refinement */            \
const Bool bStarRefinementDiamond   = 1;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bStarRefinementStop      = 0;                                                                      \
const UInt uiStarRefinementRounds   = 2;  /* star refinement stop X rounds after best match (must be >=1) */  \


__inline Void TEncSearch::xTZSearchHelp( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, const Int iSearchX, const Int iSearchY, const UChar ucPointNr, const UInt uiDistance )
{
  UInt  uiSad;
  
  Pel*  piRefSrch;
  
  piRefSrch = rcStruct.piRefY + iSearchY * rcStruct.iYStride + iSearchX;
  
  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( pcPatternKey, piRefSrch, rcStruct.iYStride,  m_cDistParam );
  
  // fast encoder decision: use subsampled SAD when rows > 8 for integer ME
  if ( m_pcEncCfg->getUseFastEnc() )
  {
    if ( m_cDistParam.iRows > 8 )
    {
      m_cDistParam.iSubShift = 1;
    }
  }
  
  // distortion
  uiSad = m_cDistParam.DistFunc( &m_cDistParam );
  
#if HHI_IMVP
  if ( m_pcEncCfg->getUseIMP() )
  {
#ifdef QC_AMVRES
    TComMv cMvPred;
    if(m_pcEncCfg->getUseAMVRes())
    {
      cMvPred = m_cMvPredMeasure.getMVPred( (iSearchX<<3) , (iSearchY<<3) );
      cMvPred.scale_down();
    }
    else
      cMvPred = m_cMvPredMeasure.getMVPred( (iSearchX<<2) , (iSearchY<<2) );
    
    m_pcRdCost->setPredictor( cMvPred );
#else
    TComMv cMvPred = m_cMvPredMeasure.getMVPred( (iSearchX<<2) , (iSearchY<<2) );
    m_pcRdCost->setPredictor( cMvPred );
#endif
  }
#endif
  // motion cost
  uiSad += m_pcRdCost->getCost( iSearchX, iSearchY );
  
  if( uiSad < rcStruct.uiBestSad )
  {
    rcStruct.uiBestSad      = uiSad;
    rcStruct.iBestX         = iSearchX;
    rcStruct.iBestY         = iSearchY;
    rcStruct.uiBestDistance = uiDistance;
    rcStruct.uiBestRound    = 0;
    rcStruct.ucPointNr      = ucPointNr;
  }
}

__inline Void TEncSearch::xTZ2PointSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();
  
  // 2 point search,                   //   1 2 3
  // check only the 2 untested points  //   4 0 5
  // around the start point            //   6 7 8
  Int iStartX = rcStruct.iBestX;
  Int iStartY = rcStruct.iBestY;
  switch( rcStruct.ucPointNr )
  {
    case 1:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY, 0, 2 );
      }
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY - 1, 0, 2 );
      }
    }
      break;
    case 2:
    {
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        if ( (iStartX - 1) >= iSrchRngHorLeft )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
        }
        if ( (iStartX + 1) <= iSrchRngHorRight )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
        }
      }
    }
      break;
    case 3:
    {
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY - 1, 0, 2 );
      }
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY, 0, 2 );
      }
    }
      break;
    case 4:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        if ( (iStartY + 1) <= iSrchRngVerBottom )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
        }
        if ( (iStartY - 1) >= iSrchRngVerTop )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
        }
      }
    }
      break;
    case 5:
    {
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        if ( (iStartY - 1) >= iSrchRngVerTop )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
        }
        if ( (iStartY + 1) <= iSrchRngVerBottom )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
        }
      }
    }
      break;
    case 6:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY , 0, 2 );
      }
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY + 1, 0, 2 );
      }
    }
      break;
    case 7:
    {
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        if ( (iStartX - 1) >= iSrchRngHorLeft )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
        }
        if ( (iStartX + 1) <= iSrchRngHorRight )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
        }
      }
    }
      break;
    case 8:
    {
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY, 0, 2 );
      }
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY + 1, 0, 2 );
      }
    }
      break;
    default:
    {
      assert( false );
    }
      break;
  } // switch( rcStruct.ucPointNr )
}

__inline Void TEncSearch::xTZ8PointSquareSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();
  
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  assert( iDist != 0 );
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;
  
  if ( iTop >= iSrchRngVerTop ) // check top
  {
    if ( iLeft >= iSrchRngHorLeft ) // check top left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iTop, 1, iDist );
    }
    // top middle
    xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
    
    if ( iRight <= iSrchRngHorRight ) // check top right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iTop, 3, iDist );
    }
  } // check top
  if ( iLeft >= iSrchRngHorLeft ) // check middle left
  {
    xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
  }
  if ( iRight <= iSrchRngHorRight ) // check middle right
  {
    xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
  }
  if ( iBottom <= iSrchRngVerBottom ) // check bottom
  {
    if ( iLeft >= iSrchRngHorLeft ) // check bottom left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iBottom, 6, iDist );
    }
    // check bottom middle
    xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
    
    if ( iRight <= iSrchRngHorRight ) // check bottom right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iBottom, 8, iDist );
    }
  } // check bottom
}

__inline Void TEncSearch::xTZ8PointDiamondSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();
  
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  assert ( iDist != 0 );
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;
  
  if ( iDist == 1 ) // iDist == 1
  {
    if ( iTop >= iSrchRngVerTop ) // check top
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
    }
    if ( iLeft >= iSrchRngHorLeft ) // check middle left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
    }
    if ( iRight <= iSrchRngHorRight ) // check middle right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
    }
    if ( iBottom <= iSrchRngVerBottom ) // check bottom
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
    }
  }
  else // if (iDist != 1)
  {
    if ( iDist <= 8 )
    {
      const Int iTop_2      = iStartY - (iDist>>1);
      const Int iBottom_2   = iStartY + (iDist>>1);
      const Int iLeft_2     = iStartX - (iDist>>1);
      const Int iRight_2    = iStartX + (iDist>>1);
      
      if (  iTop >= iSrchRngVerTop && iLeft >= iSrchRngHorLeft &&
          iRight <= iSrchRngHorRight && iBottom <= iSrchRngVerBottom ) // check border
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX,  iTop,      2, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2,  iTop_2,    1, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iTop_2,    3, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft,    iStartY,   4, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight,   iStartY,   5, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2,  iBottom_2, 6, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iBottom_2, 8, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX,  iBottom,   7, iDist    );
      }
      else // check border
      {
        if ( iTop >= iSrchRngVerTop ) // check top
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
        }
        if ( iTop_2 >= iSrchRngVerTop ) // check half top
        {
          if ( iLeft_2 >= iSrchRngHorLeft ) // check half left
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2, iTop_2, 1, (iDist>>1) );
          }
          if ( iRight_2 <= iSrchRngHorRight ) // check half right
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iTop_2, 3, (iDist>>1) );
          }
        } // check half top
        if ( iLeft >= iSrchRngHorLeft ) // check left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
        }
        if ( iRight <= iSrchRngHorRight ) // check right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
        }
        if ( iBottom_2 <= iSrchRngVerBottom ) // check half bottom
        {
          if ( iLeft_2 >= iSrchRngHorLeft ) // check half left
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2, iBottom_2, 6, (iDist>>1) );
          }
          if ( iRight_2 <= iSrchRngHorRight ) // check half right
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iBottom_2, 8, (iDist>>1) );
          }
        } // check half bottom
        if ( iBottom <= iSrchRngVerBottom ) // check bottom
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
        }
      } // check border
    }
    else // iDist > 8
    {
      if ( iTop >= iSrchRngVerTop && iLeft >= iSrchRngHorLeft &&
          iRight <= iSrchRngHorRight && iBottom <= iSrchRngVerBottom ) // check border
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop,    0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft,   iStartY, 0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight,  iStartY, 0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 0, iDist );
        for ( Int index = 1; index < 4; index++ )
        {
          Int iPosYT = iTop    + ((iDist>>2) * index);
          Int iPosYB = iBottom - ((iDist>>2) * index);
          Int iPosXL = iStartX - ((iDist>>2) * index);
          Int iPosXR = iStartX + ((iDist>>2) * index);
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYT, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYT, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYB, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYB, 0, iDist );
        }
      }
      else // check border
      {
        if ( iTop >= iSrchRngVerTop ) // check top
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 0, iDist );
        }
        if ( iLeft >= iSrchRngHorLeft ) // check left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 0, iDist );
        }
        if ( iRight <= iSrchRngHorRight ) // check right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 0, iDist );
        }
        if ( iBottom <= iSrchRngVerBottom ) // check bottom
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 0, iDist );
        }
        for ( Int index = 1; index < 4; index++ )
        {
          Int iPosYT = iTop    + ((iDist>>2) * index);
          Int iPosYB = iBottom - ((iDist>>2) * index);
          Int iPosXL = iStartX - ((iDist>>2) * index);
          Int iPosXR = iStartX + ((iDist>>2) * index);
          
          if ( iPosYT >= iSrchRngVerTop ) // check top
          {
            if ( iPosXL >= iSrchRngHorLeft ) // check left
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYT, 0, iDist );
            }
            if ( iPosXR <= iSrchRngHorRight ) // check right
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYT, 0, iDist );
            }
          } // check top
          if ( iPosYB <= iSrchRngVerBottom ) // check bottom
          {
            if ( iPosXL >= iSrchRngHorLeft ) // check left
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYB, 0, iDist );
            }
            if ( iPosXR <= iSrchRngHorRight ) // check right
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYB, 0, iDist );
            }
          } // check bottom
        } // for ...
      } // check border
    } // iDist <= 8
  } // iDist == 1
}

#ifdef ROUNDING_CONTROL_BIPRED
UInt TEncSearch::xPatternRefinement_Bi    ( TComPattern* pcPatternKey, Pel* piRef, Int iRefStride, Int iIntStep, Int iFrac, TComMv& rcMvFrac, Pel* pcRef2, Bool bRound )
{
  UInt  uiDist;
  UInt  uiDistBest  = MAX_UINT;
  UInt  uiDirecBest = 0;
  
  Pel*  piRefPos;
  
  m_pcRdCost->setDistParam_Bi( pcPatternKey, piRef, iRefStride, iIntStep, m_cDistParam, m_pcEncCfg->getUseHADME() );
  
  TComMv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);
  
  for (UInt i = 0; i < 9; i++)
  {
    TComMv cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;
    piRefPos = piRef + (pcMvRefine[i].getHor() + iRefStride * pcMvRefine[i].getVer()) * iFrac;
    m_cDistParam.pCur = piRefPos;
    uiDist = m_cDistParam.DistFuncRnd( &m_cDistParam, pcRef2, bRound );
#if HHI_IMVP
    if ( m_pcEncCfg->getUseIMP() )
    {
#ifdef QC_AMVRES
      TComMv cMvPred;
      if(m_pcEncCfg->getUseAMVRes())
      {
        cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<iFrac, cMvTest.getVer()<<iFrac);
        cMvPred.scale_down();
      }
      else
        cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<(iFrac-1), cMvTest.getVer()<<(iFrac-1) );
      
      m_pcRdCost->setPredictor( cMvPred );
#else
      TComMv cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<(iFrac-1), cMvTest.getVer()<<(iFrac-1) );
      m_pcRdCost->setPredictor( cMvPred );
#endif
    }
#endif
    uiDist += m_pcRdCost->getCost( cMvTest.getHor(), cMvTest.getVer() );
    
    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
    }
  }
  
  rcMvFrac = pcMvRefine[uiDirecBest];
  
  return uiDistBest;
}
#ifdef QC_AMVRES
#if HHI_INTERP_FILTER
UInt TEncSearch::xPatternRefinementHAM_MOMS_Bi    ( TComPattern* pcPatternKey, Pel* piRef, Int iRefStride, Int iIntStep, TComMv& rcMvFrac ,UInt  uiDistBest_onefourth, InterpFilterType ePFilt,TComMv* PredMv, Pel* pcRef2, Bool bRound )
{
  UInt  uiDist;
  UInt  uiDistBest  = uiDistBest_onefourth;
  
  Pel* piLumaExt = m_cYuvExt.getLumaAddr();
  Int iYuvExtStride = m_cYuvExt.getStride();
  m_pcRdCost->setDistParam_Bi( pcPatternKey, piLumaExt, iYuvExtStride, 1, m_cDistParam, m_pcEncCfg->getUseHADME() );
  
  TComMv cMvQ;
  
  rcMvFrac <<=1;
  TComMv cMvHAM;
  
  for (Int dMVx = -1; dMVx <= 1; dMVx++)
  {
    for (Int dMVy = -1; dMVy <= 1; dMVy++)
    {
      if (dMVy!=0 || dMVx!=0)
      {
        TComMv cMvTest = rcMvFrac;
#if HHI_IMVP
        if ( m_pcEncCfg->getUseIMP() )
        {
          TComMv cMvPred;
          cMvPred = m_cMvPredMeasure.getMVPred(cMvTest.getHor()+dMVx, cMvTest.getVer()+dMVy);
          m_pcRdCost->setPredictor( cMvPred );
        }
        else
#endif
          m_pcRdCost->setPredictor(*PredMv);
        
        predInterLumaBlkHAM_ME_MOMS(piRef, iRefStride, piLumaExt, iYuvExtStride, &cMvTest, pcPatternKey->getROIYWidth(), 
                                    pcPatternKey->getROIYHeight(), ePFilt,  dMVx,  dMVy);
		uiDist = m_cDistParam.DistFuncRnd( &m_cDistParam, pcRef2, bRound );
        uiDist += m_pcRdCost->getCost( cMvTest.getHor(), cMvTest.getVer(),1 );
        if ( uiDist < uiDistBest )
        {
          cMvHAM.set(dMVx,dMVy);
          uiDistBest  = uiDist;
        }
      }
    }
  }
  rcMvFrac = cMvHAM;
  
  return uiDistBest;
}
#endif

UInt TEncSearch::xPatternRefinementHAM_DIF_Bi( TComPattern* pcPatternKey, Pel* piRef, Int iRefStride, Int iIntStep, TComMv& rcMvFrac ,UInt  uiDistBest_onefourth,TComMv* PredMv, Pel* pcRef2, Bool bRound )
{
  UInt  uiDist;
  UInt  uiDistBest  = uiDistBest_onefourth;
  
  Pel* piLumaExt = m_cYuvExt.getLumaAddr();
  Int iYuvExtStride = m_cYuvExt.getStride();
  
  m_pcRdCost->setDistParam_Bi( pcPatternKey, piLumaExt, iYuvExtStride, 1, m_cDistParam, m_pcEncCfg->getUseHADME());
  
  TComMv cMvQ;
  
  rcMvFrac <<=1;
  TComMv cMvHAM;
  for (Int dMVx = -1; dMVx <= 1; dMVx++)
  {
    for (Int dMVy = -1; dMVy <= 1; dMVy++)
    {
      if (dMVy!=0 || dMVx!=0)
      {
        TComMv cMvTest = rcMvFrac;
#if HHI_IMVP
        if ( m_pcEncCfg->getUseIMP() )
        {
          TComMv cMvPred;
          cMvPred = m_cMvPredMeasure.getMVPred(cMvTest.getHor()+dMVx, cMvTest.getVer()+dMVy);
          m_pcRdCost->setPredictor( cMvPred );
        }
        else
#endif
          m_pcRdCost->setPredictor(*PredMv);
        
        xPredInterLumaBlkHMVME(piRef, iRefStride, piLumaExt, iYuvExtStride, &cMvTest, pcPatternKey->getROIYWidth(), 
                               pcPatternKey->getROIYHeight(),   dMVx,  dMVy);
        uiDist = m_cDistParam.DistFuncRnd( &m_cDistParam,pcRef2 ,bRound );
        uiDist += m_pcRdCost->getCost( cMvTest.getHor(), cMvTest.getVer(),1 );
        if ( uiDist < uiDistBest )
        {
          cMvHAM.set(dMVx,dMVy);
          uiDistBest  = uiDist;
        }
      }
    }
  }
  rcMvFrac = cMvHAM;
  
  return uiDistBest;
}
#endif
#endif




#ifdef QC_AMVRES
#if HHI_INTERP_FILTER
UInt TEncSearch::xPatternRefinementHAM_MOMS    ( TComPattern* pcPatternKey, Pel* piRef, Int iRefStride, Int iIntStep, TComMv& rcMvFrac ,UInt  uiDistBest_onefourth, InterpFilterType ePFilt,TComMv* PredMv)
{
  UInt  uiDist;
  UInt  uiDistBest  = uiDistBest_onefourth;
  
  Pel* piLumaExt = m_cYuvExt.getLumaAddr();
  Int iYuvExtStride = m_cYuvExt.getStride();
  
  m_pcRdCost->setDistParam( pcPatternKey, piLumaExt, iYuvExtStride, 1, m_cDistParam, m_pcEncCfg->getUseHADME() );
  
  TComMv cMvQ;
  
  rcMvFrac <<=1;
  TComMv cMvHAM;
  
  for (Int dMVx = -1; dMVx <= 1; dMVx++)
  {
    for (Int dMVy = -1; dMVy <= 1; dMVy++)
    {
      if (dMVy!=0 || dMVx!=0)
      {
        TComMv cMvTest = rcMvFrac;
#if HHI_IMVP
        if ( m_pcEncCfg->getUseIMP() )
        {
          TComMv cMvPred;
          cMvPred = m_cMvPredMeasure.getMVPred(cMvTest.getHor()+dMVx, cMvTest.getVer()+dMVy);
          m_pcRdCost->setPredictor( cMvPred );
        }
        else
#endif
          m_pcRdCost->setPredictor(*PredMv);
        
        predInterLumaBlkHAM_ME_MOMS(piRef, iRefStride, piLumaExt, iYuvExtStride, &cMvTest, pcPatternKey->getROIYWidth(), 
                                    pcPatternKey->getROIYHeight(), ePFilt,  dMVx,  dMVy);
        uiDist = m_cDistParam.DistFunc( &m_cDistParam );
        uiDist += m_pcRdCost->getCost( cMvTest.getHor(), cMvTest.getVer(),1 );
        if ( uiDist < uiDistBest )
        {
          cMvHAM.set(dMVx,dMVy);
          uiDistBest  = uiDist;
        }
      }
    }
  }
  rcMvFrac = cMvHAM;
  
  return uiDistBest;
}
#endif

UInt TEncSearch::xPatternRefinementHAM_DIF( TComPattern* pcPatternKey, Pel* piRef, Int iRefStride, Int iIntStep, TComMv& rcMvFrac ,UInt  uiDistBest_onefourth,TComMv* PredMv)
{
  UInt  uiDist;
  UInt  uiDistBest  = uiDistBest_onefourth;
  
  Pel* piLumaExt = m_cYuvExt.getLumaAddr();
  Int iYuvExtStride = m_cYuvExt.getStride();
  
  m_pcRdCost->setDistParam( pcPatternKey, piLumaExt, iYuvExtStride, 1, m_cDistParam, m_pcEncCfg->getUseHADME() );
  
  TComMv cMvQ;
  
  rcMvFrac <<=1;
  TComMv cMvHAM;
  for (Int dMVx = -1; dMVx <= 1; dMVx++)
  {
    for (Int dMVy = -1; dMVy <= 1; dMVy++)
    {
      if (dMVy!=0 || dMVx!=0)
      {
        TComMv cMvTest = rcMvFrac;
#if HHI_IMVP
        if ( m_pcEncCfg->getUseIMP() )
        {
          TComMv cMvPred;
          cMvPred = m_cMvPredMeasure.getMVPred(cMvTest.getHor()+dMVx, cMvTest.getVer()+dMVy);
          m_pcRdCost->setPredictor( cMvPred );
        }
        else
#endif
          m_pcRdCost->setPredictor(*PredMv);
        
        xPredInterLumaBlkHMVME(piRef, iRefStride, piLumaExt, iYuvExtStride, &cMvTest, pcPatternKey->getROIYWidth(), 
                               pcPatternKey->getROIYHeight(),   dMVx,  dMVy);
        uiDist = m_cDistParam.DistFunc( &m_cDistParam );
        uiDist += m_pcRdCost->getCost( cMvTest.getHor(), cMvTest.getVer(),1 );
        if ( uiDist < uiDistBest )
        {
          cMvHAM.set(dMVx,dMVy);
          uiDistBest  = uiDist;
        }
      }
    }
  }
  rcMvFrac = cMvHAM;
  
  return uiDistBest;
}
#endif

//<--

UInt TEncSearch::xPatternRefinement    ( TComPattern* pcPatternKey, Pel* piRef, Int iRefStride, Int iIntStep, Int iFrac, TComMv& rcMvFrac )
{
  UInt  uiDist;
  UInt  uiDistBest  = MAX_UINT;
  UInt  uiDirecBest = 0;
  
  Pel*  piRefPos;
  m_pcRdCost->setDistParam( pcPatternKey, piRef, iRefStride, iIntStep, m_cDistParam, m_pcEncCfg->getUseHADME() );
  
  TComMv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);
  
  for (UInt i = 0; i < 9; i++)
  {
    TComMv cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;
    piRefPos = piRef + (pcMvRefine[i].getHor() + iRefStride * pcMvRefine[i].getVer()) * iFrac;
    m_cDistParam.pCur = piRefPos;
    uiDist = m_cDistParam.DistFunc( &m_cDistParam );
#if HHI_IMVP
    if ( m_pcEncCfg->getUseIMP() )
    {
#ifdef QC_AMVRES
      TComMv cMvPred;
      if(m_pcEncCfg->getUseAMVRes())
      {
        cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<iFrac, cMvTest.getVer()<<iFrac);
        cMvPred.scale_down();
      }
      else
        cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<(iFrac-1), cMvTest.getVer()<<(iFrac-1) );
      
      m_pcRdCost->setPredictor( cMvPred );
#else
      TComMv cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<(iFrac-1), cMvTest.getVer()<<(iFrac-1) );
      m_pcRdCost->setPredictor( cMvPred );
#endif
    }
#endif
    uiDist += m_pcRdCost->getCost( cMvTest.getHor(), cMvTest.getVer() );
    
    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
    }
  }
  
  rcMvFrac = pcMvRefine[uiDirecBest];
  
  return uiDistBest;
}


Void TEncSearch::xRecurIntraChromaSearchADI( TComDataCU* pcCU, UInt uiAbsPartIdx, Pel* piOrg, Pel* piPred, Pel* piResi, Pel* piReco, UInt uiStride, TCoeff* piCoeff, UInt uiMode, UInt uiWidth, UInt uiHeight, UInt uiMaxDepth, UInt uiCurrDepth, TextType eText )
{
  UInt uiCoeffOffset = uiWidth*uiHeight;
  if( uiMaxDepth == uiCurrDepth )
  {
    UInt uiX, uiY;
    Pel* pOrg  = piOrg;
    Pel* pPred = piPred;
    Pel* pResi = piResi;
    Pel* pReco = piReco;
    Pel* pRecoPic;
    if( eText == TEXT_CHROMA_U)
      pRecoPic= pcCU->getPic()->getPicYuvRec()->getCbAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);
    else
      pRecoPic= pcCU->getPic()->getPicYuvRec()->getCrAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);
    
    UInt uiReconStride = pcCU->getPic()->getPicYuvRec()->getCStride();
    UInt uiAbsSum = 0;
    
    if (m_pcEncCfg->getUseRDOQ())
      m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, uiWidth, eText );
    
    pcCU->getPattern()->initPattern( pcCU, uiCurrDepth, uiAbsPartIdx );
    
    Bool bAboveAvail = false;
    Bool bLeftAvail  = false;
    
#if HHI_RQT_INTRA
    pcCU->getPattern()->initAdiPatternChroma(pcCU,uiAbsPartIdx, uiCurrDepth, m_piYuvExt,m_iYuvExtStride,m_iYuvExtHeight,bAboveAvail,bLeftAvail);
#else
    pcCU->getPattern()->initAdiPatternChroma(pcCU,uiAbsPartIdx, m_piYuvExt,m_iYuvExtStride,m_iYuvExtHeight,bAboveAvail,bLeftAvail);
#endif
    
    Int*   pPatChr;
    
    if (eText==TEXT_CHROMA_U)
      pPatChr=  pcCU->getPattern()->getAdiCbBuf( uiWidth, uiHeight, m_piYuvExt );
    else // (eText==TEXT_CHROMA_V)
      pPatChr=  pcCU->getPattern()->getAdiCrBuf( uiWidth, uiHeight, m_piYuvExt );
    
#if ANG_INTRA
    if ( pcCU->angIntraEnabledPredPart( uiAbsPartIdx ) )
      predIntraChromaAng( pcCU->getPattern(),pPatChr,uiMode, pPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
    else
      predIntraChromaAdi( pcCU->getPattern(),pPatChr,uiMode, pPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
    predIntraChromaAdi( pcCU->getPattern(),pPatChr,uiMode, pPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
    
    // Make residual from prediction. (MUST BE FIXED FOR EACH TRANSFORM UNIT PREDICTION)
    UChar indexROT = pcCU->getROTindex(0);
    
    // Get Residual
    for( uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( uiX = 0; uiX < uiWidth; uiX++ )
      {
        pResi[uiX] = pOrg[uiX] - pPred[uiX];
      }
      pOrg  += uiStride;
      pResi += uiStride;
      pPred += uiStride;
    }
    
    pPred = piPred;
    pResi = piResi;
    
    m_pcTrQuant->transformNxN( pcCU, pResi, uiStride, piCoeff, uiWidth, uiHeight, uiAbsSum, eText, uiAbsPartIdx, indexROT );
    
    if ( uiAbsSum )
    {
#if QC_MDDT
      m_pcTrQuant->invtransformNxN( eText, REG_DCT, pResi, uiStride, piCoeff, uiWidth, uiHeight, indexROT );
#else
      m_pcTrQuant->invtransformNxN( pResi, uiStride, piCoeff, uiWidth, uiHeight, indexROT );
#endif
    }
    else
    {
      memset(piCoeff,  0, sizeof(TCoeff)*uiCoeffOffset);
      for( uiY = 0; uiY < uiHeight; uiY++ )
      {
        memset(pResi, 0, sizeof(Pel)*uiWidth);
        pResi += uiStride;
      }
    }
    
    m_pcEntropyCoder->encodeCoeffNxN( pcCU, piCoeff, uiAbsPartIdx, uiWidth, uiHeight, pcCU->getDepth(0)+uiCurrDepth, eText, true );
    
    pPred = piPred;
    pResi = piResi;
    
    // Reconstruction
    for( uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( uiX = 0; uiX < uiWidth; uiX++ )
      {
        pReco   [uiX] = Clip(pPred[uiX] + pResi[uiX]);
        pRecoPic[uiX] = pReco[uiX];
      }
      pReco    += uiStride;
      pResi    += uiStride;
      pPred    += uiStride;
      pRecoPic += uiReconStride;
    }
  }
  else
  {
    uiCurrDepth++;
    uiWidth  >>= 1;
    uiHeight >>= 1;
    UInt uiPartOffset  = pcCU->getTotalNumPart()>>(uiCurrDepth<<1);
    UInt uiPelOffset   = uiHeight* uiStride;
    uiCoeffOffset >>= 2;
    Pel* pOrg  = piOrg;
    Pel* pResi = piResi;
    Pel* pReco = piReco;
    Pel* pPred = piPred;
    
    xRecurIntraChromaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, eText );
    uiAbsPartIdx += uiPartOffset;
    pOrg = piOrg+uiWidth; pPred = piPred+uiWidth; pResi = piResi+uiWidth; pReco = piReco+uiWidth;
    piCoeff += uiCoeffOffset;
    xRecurIntraChromaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, eText );
    uiAbsPartIdx += uiPartOffset;
    pOrg = piOrg+uiPelOffset; pPred = piPred+uiPelOffset; pResi = piResi+uiPelOffset; pReco = piReco+uiPelOffset;
    piCoeff += uiCoeffOffset;
    xRecurIntraChromaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, eText );
    uiAbsPartIdx += uiPartOffset;
    pOrg = piOrg+uiPelOffset+uiWidth; pPred = piPred+uiPelOffset+uiWidth; pResi = piResi+uiPelOffset+uiWidth; pReco = piReco+uiPelOffset+uiWidth;
    piCoeff += uiCoeffOffset;
    xRecurIntraChromaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, eText );
  }
}

// temp buffer for CIP
static Pel iPredOL[ MAX_CU_SIZE*MAX_CU_SIZE ];

#if HHI_AIS
Void TEncSearch::xRecurIntraLumaSearchADI( TComDataCU* pcCU, UInt uiAbsPartIdx, Pel* piOrg, Pel* piPred, Pel* piResi, Pel* piReco, UInt uiStride, TCoeff* piCoeff, UInt uiMode, Bool bSmoothing, UInt uiWidth, UInt uiHeight, UInt uiMaxDepth, UInt uiCurrDepth, Bool bAbove, Bool bLeft, Bool bSmallTrs)
#else
Void TEncSearch::xRecurIntraLumaSearchADI( TComDataCU* pcCU, UInt uiAbsPartIdx, Pel* piOrg, Pel* piPred, Pel* piResi, Pel* piReco, UInt uiStride, TCoeff* piCoeff, UInt uiMode, UInt uiWidth, UInt uiHeight, UInt uiMaxDepth, UInt uiCurrDepth, Bool bAbove, Bool bLeft, Bool bSmallTrs)
#endif
{
  UInt uiCoeffOffset = uiWidth*uiHeight;
  if( uiMaxDepth == uiCurrDepth )
  {
    UInt uiX, uiY;
    UInt uiZorder = pcCU->getZorderIdxInCU()+uiAbsPartIdx;
    Pel* pOrg  = piOrg;
    Pel* pPred = piPred;
    Pel* pResi = piResi;
    Pel* pReco = piReco;
    Pel* pRecoPic = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), uiZorder);
    UInt uiReconStride = pcCU->getPic()->getPicYuvRec()->getStride();
    UInt uiAbsSum = 0;
    
    if (m_pcEncCfg->getUseRDOQ())
      m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, uiWidth, TEXT_LUMA );
    
    Bool bAboveAvail = false;
    Bool bLeftAvail  = false;
    
    if (bSmallTrs){
      pcCU->getPattern()->initPattern   ( pcCU, uiCurrDepth, uiAbsPartIdx );
      pcCU->getPattern()->initAdiPattern( pcCU, uiAbsPartIdx, uiCurrDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail);
#ifdef EDGE_BASED_PREDICTION
      if(getEdgeBasedPred()->get_edge_prediction_enable())
        getEdgeBasedPred()->initEdgeBasedBuffer(pcCU, uiAbsPartIdx, uiCurrDepth, m_piYExtEdgeBased);
#endif //EDGE_BASED_PREDICTION
    }
    else {
      bAboveAvail=  bAbove;
      bLeftAvail=bLeft;
    }
    
#if ANG_INTRA
    if ( pcCU->angIntraEnabledPredPart( uiAbsPartIdx ) )
#if HHI_AIS
      predIntraLumaAng( pcCU->getPattern(), uiMode, bSmoothing, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
    predIntraLumaAng( pcCU->getPattern(), uiMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
    else
#if HHI_AIS
      predIntraLumaAdi( pcCU->getPattern(), uiMode, bSmoothing, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
    predIntraLumaAdi( pcCU->getPattern(), uiMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
    
#else // ANG_INTRA
    
#if HHI_AIS
    predIntraLumaAdi( pcCU->getPattern(), uiMode, bSmoothing, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
    predIntraLumaAdi( pcCU->getPattern(), uiMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
#endif // ANG_INTRA
    
    // CIP
    if ( pcCU->getCIPflag( uiAbsPartIdx ) )
    {
      // Prediction
      xPredIntraLumaNxNCIPEnc( pcCU->getPattern(), piOrg, piPred, uiStride, iPredOL, uiWidth, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
      
      // Get Residual
      for( uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( uiX = 0; uiX < uiWidth; uiX++ )
        {
          pResi[uiX] = pOrg[uiX] - CIP_WSUM( pPred[uiX], iPredOL[ uiX+uiY*uiWidth ], CIP_WEIGHT );
        }
        pOrg  += uiStride;
        pResi += uiStride;
        pPred += uiStride;
      }
    }
    else
    {
      // Get Residual
      for( uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( uiX = 0; uiX < uiWidth; uiX++ )
        {
          pResi[uiX] = pOrg[uiX] - pPred[uiX];
        }
        pOrg  += uiStride;
        pResi += uiStride;
        pPred += uiStride;
      }
    }
    
    pPred = piPred;
    pResi = piResi;
    
    UChar indexROT = pcCU->getROTindex(0);
#if DISABLE_ROT_LUMA_4x4_8x8
    if (uiWidth < 16) indexROT = 0;
#endif
    m_pcTrQuant->transformNxN( pcCU, piResi, uiStride, piCoeff, uiWidth, uiHeight, uiAbsSum, TEXT_LUMA, uiAbsPartIdx, indexROT );
    
    if ( uiAbsSum )
    {
#if QC_MDDT
      m_pcTrQuant->m_bQT = (1 << (pcCU->getIntraSizeIdx( uiAbsPartIdx ) + 1)) != uiWidth;
      
      m_pcTrQuant->invtransformNxN( TEXT_LUMA, uiMode, pResi, uiStride, piCoeff, uiWidth, uiHeight, indexROT );
#else
#if DISABLE_ROT_LUMA_4x4_8x8
      if (uiWidth < 16) indexROT = 0;
#endif
      m_pcTrQuant->invtransformNxN( pResi, uiStride, piCoeff, uiWidth, uiHeight, indexROT );
#endif
    }
    else
    {
      memset(piCoeff,  0, sizeof(TCoeff)*uiCoeffOffset);
      for( uiY = 0; uiY < uiHeight; uiY++ )
      {
        memset(pResi, 0, sizeof(Pel)*uiWidth);
        pResi += uiStride;
      }
    }
#if QC_MDDT//ADAPTIVE_SCAN
    g_bUpdateStats = false;
#endif
    m_pcEntropyCoder->encodeCoeffNxN( pcCU, piCoeff, uiAbsPartIdx, uiWidth, uiHeight, pcCU->getDepth( 0 )+uiCurrDepth, TEXT_LUMA, true );
    
    pPred = piPred;
    pResi = piResi;
    
    // Reconstruction
    if ( pcCU->getCIPflag( uiAbsPartIdx ) )
    {
      recIntraLumaCIP( pcCU->getPattern(), piPred, piResi, piReco, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
      
      // update to picture
      for( uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( uiX = 0; uiX < uiWidth; uiX++ )
        {
          pRecoPic[uiX] = pReco[uiX];
        }
        pReco += uiStride;
        pRecoPic += uiReconStride;
      }
    }
    else
    {
      for( uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( uiX = 0; uiX < uiWidth; uiX++ )
        {
          pReco   [uiX] = Clip(pPred[uiX] + pResi[uiX]);
          pRecoPic[uiX] = pReco[uiX];
        }
        pReco += uiStride;
        pResi += uiStride;
        pPred += uiStride;
        pRecoPic += uiReconStride;
      }
    }
  }
  else
  {
    uiCurrDepth++;
    uiWidth  >>= 1;
    uiHeight >>= 1;
    uiCoeffOffset >>= 2;
    UInt uiPartOffset  = pcCU->getTotalNumPart()>>(uiCurrDepth<<1);
    UInt uiPelOffset   = uiHeight*uiStride;
    Pel* pOrg  = piOrg;
    Pel* pResi = piResi;
    Pel* pReco = piReco;
    Pel* pPred = piPred;
#if HHI_AIS
    xRecurIntraLumaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, bSmoothing, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, false, false,true);
#else
    xRecurIntraLumaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, false, false,true);
#endif
    uiAbsPartIdx += uiPartOffset;
    pOrg = piOrg+uiWidth; pPred = piPred+uiWidth; pResi = piResi+uiWidth; pReco = piReco+uiWidth;
    piCoeff += uiCoeffOffset;
#if HHI_AIS
    xRecurIntraLumaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, bSmoothing, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, false, false, true);
#else
    xRecurIntraLumaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, false, false,true);
#endif
    uiAbsPartIdx += uiPartOffset;
    pOrg = piOrg+uiPelOffset; pPred = piPred+uiPelOffset; pResi = piResi+uiPelOffset; pReco = piReco+uiPelOffset;
    piCoeff += uiCoeffOffset;
#if HHI_AIS
    xRecurIntraLumaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, bSmoothing, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, false, false,true);
#else
    xRecurIntraLumaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, false, false,true);
#endif
    uiAbsPartIdx += uiPartOffset;
    pOrg = piOrg+uiPelOffset+uiWidth; pPred = piPred+uiPelOffset+uiWidth; pResi = piResi+uiPelOffset+uiWidth; pReco = piReco+uiPelOffset+uiWidth;
    piCoeff += uiCoeffOffset;
#if HHI_AIS
    xRecurIntraLumaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, bSmoothing, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, false, false, true);
#else
    xRecurIntraLumaSearchADI( pcCU, uiAbsPartIdx, pOrg, pPred, pResi, pReco, uiStride, piCoeff, uiMode, uiWidth, uiHeight, uiMaxDepth, uiCurrDepth, false, false,true);
#endif
  }
}


#if HHI_RQT_INTRA

Void
TEncSearch::xEncSubdivCbfQT( TComDataCU*  pcCU,
                            UInt         uiTrDepth,
                            UInt         uiAbsPartIdx,
                            Bool         bLuma,
                            Bool         bChroma )
{
  UInt  uiFullDepth     = pcCU->getDepth(0) + uiTrDepth;
  UInt  uiTrMode        = pcCU->getTransformIdx( uiAbsPartIdx );
  UInt  uiSubdiv        = ( uiTrMode > uiTrDepth ? 1 : 0 );
  UInt  uiLog2TrafoSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()] + 2 - uiFullDepth;
  
  if( pcCU->getPredictionMode(0) == MODE_INTRA && pcCU->getPartitionSize(0) == SIZE_NxN && uiTrDepth == 0 )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2TrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2TrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
  {
    assert( !uiSubdiv );
  }
#if HHI_RQT_DEPTH || HHI_RQT_DISABLE_SUB
  else if( uiLog2TrafoSize == pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
  {
    assert( !uiSubdiv );
  }
#endif  
  else
  {
#if HHI_RQT_DEPTH || HHI_RQT_DISABLE_SUB
    assert( uiLog2TrafoSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
#else 
    assert( uiLog2TrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() );
#endif
    if( bLuma )
    {
      m_pcEntropyCoder->encodeTransformSubdivFlag( uiSubdiv, uiFullDepth );
    }
  }
  
  if( uiSubdiv )
  {
    UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
    for( UInt uiPart = 0; uiPart < 4; uiPart++ )
    {
      xEncSubdivCbfQT( pcCU, uiTrDepth + 1, uiAbsPartIdx + uiPart * uiQPartNum, bLuma, bChroma );
    }
    return;
  }
  
  //===== Cbfs =====
  if( bLuma )
  {
    m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_LUMA,     uiTrMode );
  }
  if( bChroma )
  {
    Bool bCodeChroma = true;
    if( uiLog2TrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
    {
      assert( uiTrDepth > 0 );
      UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth - 1 ) << 1 );
      bCodeChroma  = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
    }
    if( bCodeChroma )
    {
      m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiTrDepth );
      m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiTrDepth );
    }
  }
}


Void
TEncSearch::xEncCoeffQT( TComDataCU*  pcCU,
                        UInt         uiTrDepth,
                        UInt         uiAbsPartIdx,
                        TextType     eTextType,
                        Bool         bRealCoeff )
{
  UInt  uiFullDepth     = pcCU->getDepth(0) + uiTrDepth;
  UInt  uiTrMode        = pcCU->getTransformIdx( uiAbsPartIdx );
  UInt  uiSubdiv        = ( uiTrMode > uiTrDepth ? 1 : 0 );
  UInt  uiLog2TrafoSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()] + 2 - uiFullDepth;
  UInt  uiChroma        = ( eTextType != TEXT_LUMA ? 1 : 0 );
  
  if( uiSubdiv )
  {
    UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
    for( UInt uiPart = 0; uiPart < 4; uiPart++ )
    {
      xEncCoeffQT( pcCU, uiTrDepth + 1, uiAbsPartIdx + uiPart * uiQPartNum, eTextType, bRealCoeff );
    }
    return;
  }
  
  if( eTextType != TEXT_LUMA && uiLog2TrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
  {
    assert( uiTrDepth > 0 );
    uiTrDepth--;
    UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth ) << 1 );
    Bool bFirstQ = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
    if( !bFirstQ )
    {
      return;
    }
  }
  
  //===== coefficients =====
  UInt    uiWidth         = pcCU->getWidth  ( 0 ) >> ( uiTrDepth + uiChroma );
  UInt    uiHeight        = pcCU->getHeight ( 0 ) >> ( uiTrDepth + uiChroma );
  UInt    uiCoeffOffset   = ( pcCU->getPic()->getMinCUWidth() * pcCU->getPic()->getMinCUHeight() * uiAbsPartIdx ) >> ( uiChroma << 1 );
  UInt    uiQTLayer       = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrafoSize;
  TCoeff* pcCoeff         = 0;
  switch( eTextType )
  {
    case TEXT_LUMA:     pcCoeff = ( bRealCoeff ? pcCU->getCoeffY () : m_ppcQTTempCoeffY [uiQTLayer] );  break;
    case TEXT_CHROMA_U: pcCoeff = ( bRealCoeff ? pcCU->getCoeffCb() : m_ppcQTTempCoeffCb[uiQTLayer] );  break;
    case TEXT_CHROMA_V: pcCoeff = ( bRealCoeff ? pcCU->getCoeffCr() : m_ppcQTTempCoeffCr[uiQTLayer] );  break;
    default:            assert(0);
  }
  pcCoeff += uiCoeffOffset;
  
#if QC_MDDT//ADAPTIVE_SCAN
  g_bUpdateStats = false;
#endif
  m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeff, uiAbsPartIdx, uiWidth, uiHeight, uiFullDepth, eTextType, false );
}


Void
TEncSearch::xEncIntraHeader( TComDataCU*  pcCU,
                            UInt         uiTrDepth,
                            UInt         uiAbsPartIdx,
                            Bool         bLuma,
                            Bool         bChroma )
{
  if( bLuma )
  {
    // CU header
    if( uiAbsPartIdx == 0 )
    {
      if( !pcCU->getSlice()->isIntra() )
      {
        m_pcEntropyCoder->encodeSkipFlag( pcCU, 0, true );
#if HHI_MRG && !HHI_MRG_PU
        m_pcEntropyCoder->encodeMergeInfo( pcCU, 0, true );
#endif
        m_pcEntropyCoder->encodePredMode( pcCU, 0, true );
      }
      
#if PLANAR_INTRA
      m_pcEntropyCoder->encodePlanarInfo( pcCU, 0, true );
      
      if ( pcCU->getPlanarInfo(0, PLANAR_FLAG) )
        return;
#endif
      
      m_pcEntropyCoder  ->encodePartSize( pcCU, 0, pcCU->getDepth(0), true );
    }
    // luma prediction mode
    if( pcCU->getPartitionSize(0) == SIZE_2Nx2N )
    {
      if( uiAbsPartIdx == 0 )
      {
        m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, 0 );
#if HHI_AIS
        m_pcEntropyCoder->encodeIntraFiltFlagLuma( pcCU, 0 );
#endif
      }
    }
    else
    {
      UInt uiQNumParts = pcCU->getTotalNumPart() >> 2;
      if( uiTrDepth == 0 )
      {
        assert( uiAbsPartIdx == 0 );
        for( UInt uiPart = 0; uiPart < 4; uiPart++ )
        {
          m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiPart * uiQNumParts );
        }
#if HHI_AIS
        for( UInt uiPart = 0; uiPart < 4; uiPart++ )
        {
          m_pcEntropyCoder->encodeIntraFiltFlagLuma( pcCU, uiPart * uiQNumParts );
        }
#endif
      }
      else if( ( uiAbsPartIdx % uiQNumParts ) == 0 )
      {
        m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiAbsPartIdx );
#if HHI_AIS
        m_pcEntropyCoder->encodeIntraFiltFlagLuma( pcCU, uiAbsPartIdx );
#endif
      }
    }
  }
  if( bChroma )
  {
    // chroma prediction mode
    if( uiAbsPartIdx == 0 )
    {
      m_pcEntropyCoder->encodeIntraDirModeChroma( pcCU, 0, true );
    }
  }
}


UInt
TEncSearch::xGetIntraBitsQT( TComDataCU*  pcCU,
                            UInt         uiTrDepth,
                            UInt         uiAbsPartIdx,
                            Bool         bLuma,
                            Bool         bChroma,
                            Bool         bRealCoeff /* just for test */ )
{
  m_pcEntropyCoder->resetBits();
  xEncIntraHeader ( pcCU, uiTrDepth, uiAbsPartIdx, bLuma, bChroma );
  xEncSubdivCbfQT ( pcCU, uiTrDepth, uiAbsPartIdx, bLuma, bChroma );
  if( bLuma )
  {
    xEncCoeffQT   ( pcCU, uiTrDepth, uiAbsPartIdx, TEXT_LUMA,      bRealCoeff );
  }
  if( bChroma )
  {
    xEncCoeffQT   ( pcCU, uiTrDepth, uiAbsPartIdx, TEXT_CHROMA_U,  bRealCoeff );
    xEncCoeffQT   ( pcCU, uiTrDepth, uiAbsPartIdx, TEXT_CHROMA_V,  bRealCoeff );
  }
  UInt   uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  return uiBits;
}



Void
TEncSearch::xIntraCodingLumaBlk( TComDataCU* pcCU,
                                UInt        uiTrDepth,
                                UInt        uiAbsPartIdx,
                                TComYuv*    pcOrgYuv, 
                                TComYuv*    pcPredYuv, 
                                TComYuv*    pcResiYuv, 
                                UInt&       ruiDist )
{
#if HHI_AIS
  Bool    bIntraSmoothing   = pcCU     ->getLumaIntraFiltFlag( uiAbsPartIdx );
#endif
  UInt    uiLumaPredMode    = pcCU     ->getLumaIntraDir     ( uiAbsPartIdx );
  UInt    uiFullDepth       = pcCU     ->getDepth   ( 0 )  + uiTrDepth;
  UInt    uiWidth           = pcCU     ->getWidth   ( 0 ) >> uiTrDepth;
  UInt    uiHeight          = pcCU     ->getHeight  ( 0 ) >> uiTrDepth;
  UInt    uiStride          = pcOrgYuv ->getStride  ();
  Pel*    piOrg             = pcOrgYuv ->getLumaAddr( uiAbsPartIdx );
  Pel*    piPred            = pcPredYuv->getLumaAddr( uiAbsPartIdx );
  Pel*    piResi            = pcResiYuv->getLumaAddr( uiAbsPartIdx );
  Pel*    piReco            = pcPredYuv->getLumaAddr( uiAbsPartIdx );
  
  UInt    uiLog2TrSize      = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
  UInt    uiQTLayer         = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
  UInt    uiNumCoeffPerInc  = pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
  TCoeff* pcCoeff           = m_ppcQTTempCoeffY[ uiQTLayer ] + uiNumCoeffPerInc * uiAbsPartIdx;
  Pel*    piRecQt           = m_pcQTTempTComYuv[ uiQTLayer ].getLumaAddr( uiAbsPartIdx );
  UInt    uiRecQtStride     = m_pcQTTempTComYuv[ uiQTLayer ].getStride  ();
  
  UInt    uiZOrder          = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
  Pel*    piRecIPred        = pcCU->getPic()->getPicYuvRec()->getLumaAddr( pcCU->getAddr(), uiZOrder );
  UInt    uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getStride  ();
  
  //===== init availability pattern =====
  Bool  bAboveAvail = false;
  Bool  bLeftAvail  = false;
  pcCU->getPattern()->initPattern   ( pcCU, uiTrDepth, uiAbsPartIdx );
  pcCU->getPattern()->initAdiPattern( pcCU, uiAbsPartIdx, uiTrDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail );
#ifdef EDGE_BASED_PREDICTION
  if(getEdgeBasedPred()->get_edge_prediction_enable())
    getEdgeBasedPred()->initEdgeBasedBuffer(pcCU, uiAbsPartIdx, uiTrDepth, m_piYExtEdgeBased);
#endif //EDGE_BASED_PREDICTION
  
  //===== get prediction signal =====
#if ANG_INTRA
  if ( pcCU->angIntraEnabledPredPart( uiAbsPartIdx ) )
#if HHI_AIS
    predIntraLumaAng( pcCU->getPattern(), uiLumaPredMode, bIntraSmoothing, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
  predIntraLumaAng( pcCU->getPattern(), uiLumaPredMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
  else
#if HHI_AIS
    predIntraLumaAdi( pcCU->getPattern(), uiLumaPredMode, bIntraSmoothing, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
  predIntraLumaAdi( pcCU->getPattern(), uiLumaPredMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
  
#else // ANG_INTRA
  
#if HHI_AIS
  predIntraLumaAdi( pcCU->getPattern(), uiLumaPredMode, bIntraSmoothing, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
  predIntraLumaAdi( pcCU->getPattern(), uiLumaPredMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
#endif // ANG_INTRA
  
  //===== get residual signal =====
  if( pcCU->getCIPflag( uiAbsPartIdx ) )
  {
    // CIP
    Pel aiPredOL[ MAX_CU_SIZE*MAX_CU_SIZE ];
    xPredIntraLumaNxNCIPEnc( pcCU->getPattern(), piOrg, piPred, uiStride, aiPredOL, uiWidth, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
    
    // get residual
    Pel*  pOrg    = piOrg;
    Pel*  pPred   = piPred;
    Pel*  pResi   = piResi;
    Pel*  pPredOL = aiPredOL;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pResi[ uiX ] = pOrg[ uiX ] - CIP_WSUM( pPred[ uiX ], pPredOL[ uiX ], CIP_WEIGHT );
      }
      pOrg    += uiStride;
      pPred   += uiStride;
      pResi   += uiStride;
      pPredOL += uiWidth;
    }
  }
  else
  {
    // get residual
    Pel*  pOrg    = piOrg;
    Pel*  pPred   = piPred;
    Pel*  pResi   = piResi;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pResi[ uiX ] = pOrg[ uiX ] - pPred[ uiX ];
      }
      pOrg  += uiStride;
      pResi += uiStride;
      pPred += uiStride;
    }
  }
  
  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
  if( m_pcEncCfg->getUseRDOQ() )
  {
    m_pcEntropyCoder->estimateBit( m_pcTrQuant->m_pcEstBitsSbac, uiWidth, TEXT_LUMA );
  }
  //--- transform and quantization ---
  UInt uiAbsSum = 0;
  pcCU       ->setTrIdxSubParts ( uiTrDepth, uiAbsPartIdx, uiFullDepth );
  m_pcTrQuant->setQPforQuant    ( pcCU->getQP( 0 ), !pcCU->getSlice()->getDepth(), pcCU->getSlice()->getSliceType(), TEXT_LUMA );
#if DISABLE_ROT_LUMA_4x4_8x8
  m_pcTrQuant->transformNxN     ( pcCU, piResi, uiStride, pcCoeff, uiWidth, uiHeight, uiAbsSum, TEXT_LUMA, uiAbsPartIdx, uiWidth > 8 ? pcCU->getROTindex( 0 ): 0  );
#else
  m_pcTrQuant->transformNxN     ( pcCU, piResi, uiStride, pcCoeff, uiWidth, uiHeight, uiAbsSum, TEXT_LUMA, uiAbsPartIdx, pcCU->getROTindex( 0 ) );
#endif
  //--- set coded block flag ---
  pcCU->setCbfSubParts          ( ( uiAbsSum ? 1 : 0 ) << uiTrDepth, TEXT_LUMA, uiAbsPartIdx, uiFullDepth );
  //--- inverse transform ---
  if( uiAbsSum )
  {
#if QC_MDDT
    m_pcTrQuant->m_bQT = (1 << (pcCU->getIntraSizeIdx( uiAbsPartIdx ) + 1)) != uiWidth;
    
    m_pcTrQuant->invtransformNxN( TEXT_LUMA, pcCU->getLumaIntraDir( uiAbsPartIdx ), piResi, uiStride, pcCoeff, uiWidth, uiHeight, pcCU->getROTindex( 0 ) );
#else
#if DISABLE_ROT_LUMA_4x4_8x8
    m_pcTrQuant->invtransformNxN( piResi, uiStride, pcCoeff, uiWidth, uiHeight, uiWidth > 8 ? pcCU->getROTindex( 0 ): 0 );
#else
    m_pcTrQuant->invtransformNxN( piResi, uiStride, pcCoeff, uiWidth, uiHeight, pcCU->getROTindex( 0 ) );
#endif
#endif
  }
  else
  {
    Pel* pResi = piResi;
    memset( pcCoeff, 0, sizeof( TCoeff ) * uiWidth * uiHeight );
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      memset( pResi, 0, sizeof( Pel ) * uiWidth );
      pResi += uiStride;
    }
  }
  
  //===== reconstruction =====
  if( pcCU->getCIPflag( uiAbsPartIdx ) )
  {
    recIntraLumaCIP( pcCU->getPattern(), piPred, piResi, piReco, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
    
    Pel* pReco      = piReco;
    Pel* pRecQt     = piRecQt;
    Pel* pRecIPred  = piRecIPred;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pRecQt   [ uiX ] = pReco[ uiX ];
        pRecIPred[ uiX ] = pReco[ uiX ];
      }
      pReco     += uiStride;
      pRecQt    += uiRecQtStride;
      pRecIPred += uiRecIPredStride;
    }
  }
  else
  {
    Pel* pPred      = piPred;
    Pel* pResi      = piResi;
    Pel* pReco      = piReco;
    Pel* pRecQt     = piRecQt;
    Pel* pRecIPred  = piRecIPred;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pReco    [ uiX ] = Clip( pPred[ uiX ] + pResi[ uiX ] );
        pRecQt   [ uiX ] = pReco[ uiX ];
        pRecIPred[ uiX ] = pReco[ uiX ];
      }
      pPred     += uiStride;
      pResi     += uiStride;
      pReco     += uiStride;
      pRecQt    += uiRecQtStride;
      pRecIPred += uiRecIPredStride;
    }
  }
  
  //===== update distortion =====
  ruiDist += m_pcRdCost->getDistPart( piReco, uiStride, piOrg, uiStride, uiWidth, uiHeight );
}


Void
TEncSearch::xIntraCodingChromaBlk( TComDataCU* pcCU,
                                  UInt        uiTrDepth,
                                  UInt        uiAbsPartIdx,
                                  TComYuv*    pcOrgYuv, 
                                  TComYuv*    pcPredYuv, 
                                  TComYuv*    pcResiYuv, 
                                  UInt&       ruiDist,
                                  UInt        uiChromaId )
{
  UInt uiOrgTrDepth = uiTrDepth;
  UInt uiFullDepth  = pcCU->getDepth( 0 ) + uiTrDepth;
  UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
  if( uiLog2TrSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
  {
    assert( uiTrDepth > 0 );
    uiTrDepth--;
    UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth ) << 1 );
    Bool bFirstQ = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
    if( !bFirstQ )
    {
      return;
    }
  }
  
  TextType  eText             = ( uiChromaId > 0 ? TEXT_CHROMA_V : TEXT_CHROMA_U );
  UInt      uiChromaPredMode  = pcCU     ->getChromaIntraDir( uiAbsPartIdx );
  UInt      uiWidth           = pcCU     ->getWidth   ( 0 ) >> ( uiTrDepth + 1 );
  UInt      uiHeight          = pcCU     ->getHeight  ( 0 ) >> ( uiTrDepth + 1 );
  UInt      uiStride          = pcOrgYuv ->getCStride ();
  Pel*      piOrg             = ( uiChromaId > 0 ? pcOrgYuv ->getCrAddr( uiAbsPartIdx ) : pcOrgYuv ->getCbAddr( uiAbsPartIdx ) );
  Pel*      piPred            = ( uiChromaId > 0 ? pcPredYuv->getCrAddr( uiAbsPartIdx ) : pcPredYuv->getCbAddr( uiAbsPartIdx ) );
  Pel*      piResi            = ( uiChromaId > 0 ? pcResiYuv->getCrAddr( uiAbsPartIdx ) : pcResiYuv->getCbAddr( uiAbsPartIdx ) );
  Pel*      piReco            = ( uiChromaId > 0 ? pcPredYuv->getCrAddr( uiAbsPartIdx ) : pcPredYuv->getCbAddr( uiAbsPartIdx ) );
  
  UInt      uiQTLayer         = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
  UInt      uiNumCoeffPerInc  = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 ) ) >> 2;
  TCoeff*   pcCoeff           = ( uiChromaId > 0 ? m_ppcQTTempCoeffCr[ uiQTLayer ] : m_ppcQTTempCoeffCb[ uiQTLayer ] ) + uiNumCoeffPerInc * uiAbsPartIdx;
  Pel*      piRecQt           = ( uiChromaId > 0 ? m_pcQTTempTComYuv[ uiQTLayer ].getCrAddr( uiAbsPartIdx ) : m_pcQTTempTComYuv[ uiQTLayer ].getCbAddr( uiAbsPartIdx ) );
  UInt      uiRecQtStride     = m_pcQTTempTComYuv[ uiQTLayer ].getCStride();
  
  UInt      uiZOrder          = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
  Pel*      piRecIPred        = ( uiChromaId > 0 ? pcCU->getPic()->getPicYuvRec()->getCrAddr( pcCU->getAddr(), uiZOrder ) : pcCU->getPic()->getPicYuvRec()->getCbAddr( pcCU->getAddr(), uiZOrder ) );
  UInt      uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getCStride();
  
  //===== update chroma mode =====
  if( uiChromaPredMode == 4 )
#if ANG_INTRA
  {
    UInt    uiIntraIdx        = pcCU->getIntraSizeIdx( 0 );
    uiChromaPredMode          = pcCU->angIntraEnabledPredPart( 0 ) ? pcCU->getLumaIntraDir( 0 ) : g_aucIntraModeOrder[ uiIntraIdx ][ pcCU->getLumaIntraDir( 0 ) ];
  }
#else
  {
    UInt    uiIntraIdx        = pcCU->getIntraSizeIdx( 0 );
    uiChromaPredMode          = g_aucIntraModeOrder[ uiIntraIdx ][ pcCU->getLumaIntraDir( 0 ) ];
  }
#endif
  
  //===== init availability pattern =====
  Bool  bAboveAvail = false;
  Bool  bLeftAvail  = false;
  pcCU->getPattern()->initPattern         ( pcCU, uiTrDepth, uiAbsPartIdx );
  pcCU->getPattern()->initAdiPatternChroma( pcCU, uiAbsPartIdx, uiTrDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail );
  Int*  pPatChroma  = ( uiChromaId > 0 ? pcCU->getPattern()->getAdiCrBuf( uiWidth, uiHeight, m_piYuvExt ) : pcCU->getPattern()->getAdiCbBuf( uiWidth, uiHeight, m_piYuvExt ) );
  
  //===== get prediction signal =====
#if ANG_INTRA
  if ( pcCU->angIntraEnabledPredPart( 0 ) )
    predIntraChromaAng( pcCU->getPattern(), pPatChroma, uiChromaPredMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
  else
    predIntraChromaAdi( pcCU->getPattern(), pPatChroma, uiChromaPredMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
  predIntraChromaAdi( pcCU->getPattern(), pPatChroma, uiChromaPredMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
  
  //===== get residual signal =====
  {
    // get residual
    Pel*  pOrg    = piOrg;
    Pel*  pPred   = piPred;
    Pel*  pResi   = piResi;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pResi[ uiX ] = pOrg[ uiX ] - pPred[ uiX ];
      }
      pOrg  += uiStride;
      pResi += uiStride;
      pPred += uiStride;
    }
  }
  
  //===== transform and quantization =====
  {
    //--- init rate estimation arrays for RDOQ ---
    if( m_pcEncCfg->getUseRDOQ() )
    {
      m_pcEntropyCoder->estimateBit( m_pcTrQuant->m_pcEstBitsSbac, uiWidth, eText );
    }
    //--- transform and quantization ---
    UInt uiAbsSum = 0;
    m_pcTrQuant->setQPforQuant     ( pcCU->getQP( 0 ), !pcCU->getSlice()->getDepth(), pcCU->getSlice()->getSliceType(), TEXT_CHROMA );
    m_pcTrQuant->transformNxN      ( pcCU, piResi, uiStride, pcCoeff, uiWidth, uiHeight, uiAbsSum, eText, uiAbsPartIdx, pcCU->getROTindex( 0 ) );
    //--- set coded block flag ---
    pcCU->setCbfSubParts           ( ( uiAbsSum ? 1 : 0 ) << uiOrgTrDepth, eText, uiAbsPartIdx, pcCU->getDepth(0) + uiTrDepth );
    //--- inverse transform ---
    if( uiAbsSum )
    {
#if QC_MDDT
      m_pcTrQuant->invtransformNxN( TEXT_CHROMA, REG_DCT, piResi, uiStride, pcCoeff, uiWidth, uiHeight, pcCU->getROTindex( 0 ) );
#else
      m_pcTrQuant->invtransformNxN( piResi, uiStride, pcCoeff, uiWidth, uiHeight, pcCU->getROTindex( 0 ) );
#endif
    }
    else
    {
      Pel* pResi = piResi;
      memset( pcCoeff, 0, sizeof( TCoeff ) * uiWidth * uiHeight );
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        memset( pResi, 0, sizeof( Pel ) * uiWidth );
        pResi += uiStride;
      }
    }
  }
  
  //===== reconstruction =====
  {
    Pel* pPred      = piPred;
    Pel* pResi      = piResi;
    Pel* pReco      = piReco;
    Pel* pRecQt     = piRecQt;
    Pel* pRecIPred  = piRecIPred;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pReco    [ uiX ] = Clip( pPred[ uiX ] + pResi[ uiX ] );
        pRecQt   [ uiX ] = pReco[ uiX ];
        pRecIPred[ uiX ] = pReco[ uiX ];
      }
      pPred     += uiStride;
      pResi     += uiStride;
      pReco     += uiStride;
      pRecQt    += uiRecQtStride;
      pRecIPred += uiRecIPredStride;
    }
  }
  
  //===== update distortion =====
  ruiDist += m_pcRdCost->getDistPart( piReco, uiStride, piOrg, uiStride, uiWidth, uiHeight );
}



Void 
TEncSearch::xRecurIntraCodingQT( TComDataCU*  pcCU, 
                                UInt         uiTrDepth,
                                UInt         uiAbsPartIdx, 
                                Bool         bLumaOnly,
                                TComYuv*     pcOrgYuv, 
                                TComYuv*     pcPredYuv, 
                                TComYuv*     pcResiYuv, 
                                UInt&        ruiDistY,
                                UInt&        ruiDistC,
                                Double&      dRDCost )
{
  UInt    uiFullDepth   = pcCU->getDepth( 0 ) +  uiTrDepth;
  UInt    uiLog2TrSize  = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
  Bool    bCheckFull    = ( uiLog2TrSize  <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
#if HHI_RQT_DEPTH || HHI_RQT_DISABLE_SUB
  Bool    bCheckSplit   = ( uiLog2TrSize  >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
#else
  Bool    bCheckSplit   = ( uiLog2TrSize  >  pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() );
#endif
  
  Double  dSingleCost   = MAX_DOUBLE;
  UInt    uiSingleDistY = 0;
  UInt    uiSingleDistC = 0;
  UInt    uiSingleCbfY  = 0;
  UInt    uiSingleCbfU  = 0;
  UInt    uiSingleCbfV  = 0;
  
  if( bCheckFull )
  {
    //----- store original entropy coding status -----
    if( m_bUseSBACRD && bCheckSplit )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    //----- code luma block with given intra prediction mode and store Cbf-----
    dSingleCost   = 0.0;
    xIntraCodingLumaBlk( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, uiSingleDistY ); 
    if( bCheckSplit )
    {
      uiSingleCbfY = pcCU->getCbf( uiAbsPartIdx, TEXT_LUMA, uiTrDepth );
    }
    //----- code chroma blocks with given intra prediction mode and store Cbf-----
    if( !bLumaOnly )
    {
      xIntraCodingChromaBlk ( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, uiSingleDistC, 0 ); 
      xIntraCodingChromaBlk ( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, uiSingleDistC, 1 ); 
      if( bCheckSplit )
      {
        uiSingleCbfU = pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiTrDepth );
        uiSingleCbfV = pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiTrDepth );
      }
    }
    //----- determine rate and r-d cost -----
    UInt uiSingleBits = xGetIntraBitsQT( pcCU, uiTrDepth, uiAbsPartIdx, true, !bLumaOnly, false );
    dSingleCost       = m_pcRdCost->calcRdCost( uiSingleBits, uiSingleDistY + uiSingleDistC );
  }
  
  if( bCheckSplit )
  {
    //----- store full entropy coding status, load original entropy coding status -----
    if( m_bUseSBACRD )
    {
      if( bCheckFull )
      {
        m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );
        m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
      }
      else
      {
        m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
      }
    }
    //----- code splitted block -----
    Double  dSplitCost      = 0.0;
    UInt    uiSplitDistY    = 0;
    UInt    uiSplitDistC    = 0;
    UInt    uiQPartsDiv     = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
    UInt    uiAbsPartIdxSub = uiAbsPartIdx;
    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiAbsPartIdxSub += uiQPartsDiv )
    {
      xRecurIntraCodingQT( pcCU, uiTrDepth + 1, uiAbsPartIdxSub, bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, uiSplitDistY, uiSplitDistC, dSplitCost );
    }
    //----- restore context states -----
    if( m_bUseSBACRD )
    {
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    //----- determine rate and r-d cost -----
    UInt uiSplitBits = xGetIntraBitsQT( pcCU, uiTrDepth, uiAbsPartIdx, true, !bLumaOnly, false );
    dSplitCost       = m_pcRdCost->calcRdCost( uiSplitBits, uiSplitDistY + uiSplitDistC );
    
    //===== compare and set best =====
    if( dSplitCost < dSingleCost )
    {
      //--- set luma Cbf values ---
      UInt  uiSplitCbfY = 0;
      uiAbsPartIdxSub   = uiAbsPartIdx;
      for( UInt uiPart  = 0; uiPart < 4; uiPart++, uiAbsPartIdxSub += uiQPartsDiv )
      {
        uiSplitCbfY |= pcCU->getCbf( uiAbsPartIdxSub, TEXT_LUMA, uiTrDepth + 1 );
      }
      for( UInt uiOffs = 0; uiOffs < 4 * uiQPartsDiv; uiOffs++ )
      {
        pcCU->getCbf( TEXT_LUMA )[ uiAbsPartIdx + uiOffs ] |= ( uiSplitCbfY << uiTrDepth );
      }
      //--- set chroma Cbf values ---
      if( !bLumaOnly )
      {
        UInt  uiSplitCbfU = 0;
        UInt  uiSplitCbfV = 0;
        uiAbsPartIdxSub   = uiAbsPartIdx;
        for( UInt uiPart  = 0; uiPart < 4; uiPart++, uiAbsPartIdxSub += uiQPartsDiv )
        {
          uiSplitCbfU |= pcCU->getCbf( uiAbsPartIdxSub, TEXT_CHROMA_U, uiTrDepth + 1 );
          uiSplitCbfV |= pcCU->getCbf( uiAbsPartIdxSub, TEXT_CHROMA_V, uiTrDepth + 1 );
        }
        for( UInt uiOffs = 0; uiOffs < 4 * uiQPartsDiv; uiOffs++ )
        {
          pcCU->getCbf( TEXT_CHROMA_U )[ uiAbsPartIdx + uiOffs ] |= ( uiSplitCbfU << uiTrDepth );
          pcCU->getCbf( TEXT_CHROMA_V )[ uiAbsPartIdx + uiOffs ] |= ( uiSplitCbfV << uiTrDepth );
        }
      }
      //--- update cost ---
      ruiDistY += uiSplitDistY;
      ruiDistC += uiSplitDistC;
      dRDCost  += dSplitCost;
      return;
    }
    //----- set entropy coding status -----
    if( m_bUseSBACRD )
    {
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );
    }
  }
  
  if( bCheckSplit )
  {
    //--- set transform index and Cbf values ---
    pcCU->setTrIdxSubParts( uiTrDepth, uiAbsPartIdx, uiFullDepth );
    pcCU->setCbfSubParts  ( uiSingleCbfY << uiTrDepth, TEXT_LUMA, uiAbsPartIdx, uiFullDepth );
    if( !bLumaOnly )
    {
      pcCU->setCbfSubParts( uiSingleCbfU << uiTrDepth, TEXT_CHROMA_U, uiAbsPartIdx, uiFullDepth );
      pcCU->setCbfSubParts( uiSingleCbfV << uiTrDepth, TEXT_CHROMA_V, uiAbsPartIdx, uiFullDepth );
    }
    
    //--- set reconstruction for next intra prediction blocks ---
    UInt  uiWidth     = pcCU->getWidth ( 0 ) >> uiTrDepth;
    UInt  uiHeight    = pcCU->getHeight( 0 ) >> uiTrDepth;
    UInt  uiQTLayer   = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    UInt  uiZOrder    = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
    Pel*  piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getLumaAddr( uiAbsPartIdx );
    UInt  uiSrcStride = m_pcQTTempTComYuv[ uiQTLayer ].getStride  ();
    Pel*  piDes       = pcCU->getPic()->getPicYuvRec()->getLumaAddr( pcCU->getAddr(), uiZOrder );
    UInt  uiDesStride = pcCU->getPic()->getPicYuvRec()->getStride  ();
    for( UInt uiY = 0; uiY < uiHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        piDes[ uiX ] = piSrc[ uiX ];
      }
    }
    if( !bLumaOnly )
    {
      uiWidth   >>= 1;
      uiHeight  >>= 1;
      piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getCbAddr  ( uiAbsPartIdx );
      uiSrcStride = m_pcQTTempTComYuv[ uiQTLayer ].getCStride ();
      piDes       = pcCU->getPic()->getPicYuvRec()->getCbAddr ( pcCU->getAddr(), uiZOrder );
      uiDesStride = pcCU->getPic()->getPicYuvRec()->getCStride();
      for( UInt uiY = 0; uiY < uiHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          piDes[ uiX ] = piSrc[ uiX ];
        }
      }
      piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getCrAddr  ( uiAbsPartIdx );
      piDes       = pcCU->getPic()->getPicYuvRec()->getCrAddr ( pcCU->getAddr(), uiZOrder );
      for( UInt uiY = 0; uiY < uiHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          piDes[ uiX ] = piSrc[ uiX ];
        }
      }
    }
  }
  ruiDistY += uiSingleDistY;
  ruiDistC += uiSingleDistC;
  dRDCost  += dSingleCost;
}


Void
TEncSearch::xSetIntraResultQT( TComDataCU* pcCU,
                              UInt        uiTrDepth,
                              UInt        uiAbsPartIdx,
                              Bool        bLumaOnly,
                              TComYuv*    pcRecoYuv )
{
  UInt uiFullDepth  = pcCU->getDepth(0) + uiTrDepth;
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
    UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    
    Bool bSkipChroma  = false;
    Bool bChromaSame  = false;
    if( !bLumaOnly && uiLog2TrSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
    {
      assert( uiTrDepth > 0 );
      UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth - 1 ) << 1 );
      bSkipChroma  = ( ( uiAbsPartIdx % uiQPDiv ) != 0 );
      bChromaSame  = true;
    }
    
    //===== copy transform coefficients =====
    UInt uiNumCoeffY    = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( uiFullDepth << 1 );
    UInt uiNumCoeffIncY = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
    TCoeff* pcCoeffSrcY = m_ppcQTTempCoeffY [ uiQTLayer ] + ( uiNumCoeffIncY * uiAbsPartIdx );
    TCoeff* pcCoeffDstY = pcCU->getCoeffY ()              + ( uiNumCoeffIncY * uiAbsPartIdx );
    ::memcpy( pcCoeffDstY, pcCoeffSrcY, sizeof( TCoeff ) * uiNumCoeffY );
    if( !bLumaOnly && !bSkipChroma )
    {
      UInt uiNumCoeffC    = ( bChromaSame ? uiNumCoeffY    : uiNumCoeffY    >> 2 );
      UInt uiNumCoeffIncC = uiNumCoeffIncY >> 2;
      TCoeff* pcCoeffSrcU = m_ppcQTTempCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcCoeffSrcV = m_ppcQTTempCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcCoeffDstU = pcCU->getCoeffCb()              + ( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcCoeffDstV = pcCU->getCoeffCr()              + ( uiNumCoeffIncC * uiAbsPartIdx );
      ::memcpy( pcCoeffDstU, pcCoeffSrcU, sizeof( TCoeff ) * uiNumCoeffC );
      ::memcpy( pcCoeffDstV, pcCoeffSrcV, sizeof( TCoeff ) * uiNumCoeffC );
    }
    
    //===== copy reconstruction =====
    m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartLuma( pcRecoYuv, uiAbsPartIdx, 1 << uiLog2TrSize, 1 << uiLog2TrSize );
    if( !bLumaOnly && !bSkipChroma )
    {
      UInt uiLog2TrSizeChroma = ( bChromaSame ? uiLog2TrSize : uiLog2TrSize - 1 );
      m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartChroma( pcRecoYuv, uiAbsPartIdx, 1 << uiLog2TrSizeChroma, 1 << uiLog2TrSizeChroma );
    }
  }
  else
  {
    UInt uiNumQPart  = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
    for( UInt uiPart = 0; uiPart < 4; uiPart++ )
    {
      xSetIntraResultQT( pcCU, uiTrDepth + 1, uiAbsPartIdx + uiPart * uiNumQPart, bLumaOnly, pcRecoYuv );
    }
  }
}



Void 
TEncSearch::xRecurIntraChromaCodingQT( TComDataCU*  pcCU, 
                                      UInt         uiTrDepth,
                                      UInt         uiAbsPartIdx, 
                                      TComYuv*     pcOrgYuv, 
                                      TComYuv*     pcPredYuv, 
                                      TComYuv*     pcResiYuv, 
                                      UInt&        ruiDist )
{
  UInt uiFullDepth = pcCU->getDepth( 0 ) +  uiTrDepth;
  UInt uiTrMode    = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    xIntraCodingChromaBlk( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, ruiDist, 0 ); 
    xIntraCodingChromaBlk( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, ruiDist, 1 ); 
  }
  else
  {
    UInt uiSplitCbfU     = 0;
    UInt uiSplitCbfV     = 0;
    UInt uiQPartsDiv     = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
    UInt uiAbsPartIdxSub = uiAbsPartIdx;
    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiAbsPartIdxSub += uiQPartsDiv )
    {
      xRecurIntraChromaCodingQT( pcCU, uiTrDepth + 1, uiAbsPartIdxSub, pcOrgYuv, pcPredYuv, pcResiYuv, ruiDist );
      uiSplitCbfU |= pcCU->getCbf( uiAbsPartIdxSub, TEXT_CHROMA_U, uiTrDepth + 1 );
      uiSplitCbfV |= pcCU->getCbf( uiAbsPartIdxSub, TEXT_CHROMA_V, uiTrDepth + 1 );
    }
    for( UInt uiOffs = 0; uiOffs < 4 * uiQPartsDiv; uiOffs++ )
    {
      pcCU->getCbf( TEXT_CHROMA_U )[ uiAbsPartIdx + uiOffs ] |= ( uiSplitCbfU << uiTrDepth );
      pcCU->getCbf( TEXT_CHROMA_V )[ uiAbsPartIdx + uiOffs ] |= ( uiSplitCbfV << uiTrDepth );
    }
  }
}

Void
TEncSearch::xSetIntraResultChromaQT( TComDataCU* pcCU,
                                    UInt        uiTrDepth,
                                    UInt        uiAbsPartIdx,
                                    TComYuv*    pcRecoYuv )
{
  UInt uiFullDepth  = pcCU->getDepth(0) + uiTrDepth;
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
    UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    
    Bool bChromaSame  = false;
    if( uiLog2TrSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
    {
      assert( uiTrDepth > 0 );
      UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth - 1 ) << 1 );
      if( ( uiAbsPartIdx % uiQPDiv ) != 0 )
      {
        return;
      }
      bChromaSame     = true;
    }
    
    //===== copy transform coefficients =====
    UInt uiNumCoeffC    = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( uiFullDepth << 1 );
    if( !bChromaSame )
    {
      uiNumCoeffC     >>= 2;
    }
    UInt uiNumCoeffIncC = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 ) + 2 );
    TCoeff* pcCoeffSrcU = m_ppcQTTempCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    TCoeff* pcCoeffSrcV = m_ppcQTTempCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    TCoeff* pcCoeffDstU = pcCU->getCoeffCb()              + ( uiNumCoeffIncC * uiAbsPartIdx );
    TCoeff* pcCoeffDstV = pcCU->getCoeffCr()              + ( uiNumCoeffIncC * uiAbsPartIdx );
    ::memcpy( pcCoeffDstU, pcCoeffSrcU, sizeof( TCoeff ) * uiNumCoeffC );
    ::memcpy( pcCoeffDstV, pcCoeffSrcV, sizeof( TCoeff ) * uiNumCoeffC );
    
    //===== copy reconstruction =====
    UInt uiLog2TrSizeChroma = ( bChromaSame ? uiLog2TrSize : uiLog2TrSize - 1 );
    m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartChroma( pcRecoYuv, uiAbsPartIdx, 1 << uiLog2TrSizeChroma, 1 << uiLog2TrSizeChroma );
  }
  else
  {
    UInt uiNumQPart  = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
    for( UInt uiPart = 0; uiPart < 4; uiPart++ )
    {
      xSetIntraResultChromaQT( pcCU, uiTrDepth + 1, uiAbsPartIdx + uiPart * uiNumQPart, pcRecoYuv );
    }
  }
}


Void 
TEncSearch::preestChromaPredMode( TComDataCU* pcCU, 
                                 TComYuv*    pcOrgYuv, 
                                 TComYuv*    pcPredYuv )
{
  UInt  uiWidth     = pcCU->getWidth ( 0 ) >> 1;
  UInt  uiHeight    = pcCU->getHeight( 0 ) >> 1;
  UInt  uiStride    = pcOrgYuv ->getCStride();
  Pel*  piOrgU      = pcOrgYuv ->getCbAddr ( 0 );
  Pel*  piOrgV      = pcOrgYuv ->getCrAddr ( 0 );
  Pel*  piPredU     = pcPredYuv->getCbAddr ( 0 );
  Pel*  piPredV     = pcPredYuv->getCrAddr ( 0 );
  
  //===== init pattern =====
  Bool  bAboveAvail = false;
  Bool  bLeftAvail  = false;
  pcCU->getPattern()->initPattern         ( pcCU, 0, 0 );
  pcCU->getPattern()->initAdiPatternChroma( pcCU, 0, 0, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail );
  Int*  pPatChromaU = pcCU->getPattern()->getAdiCbBuf( uiWidth, uiHeight, m_piYuvExt );
  Int*  pPatChromaV = pcCU->getPattern()->getAdiCrBuf( uiWidth, uiHeight, m_piYuvExt );
  
  //===== get best prediction modes (using SAD) =====
  UInt  uiMinMode   = 0;
  UInt  uiMaxMode   = 4;
  UInt  uiBestMode  = MAX_UINT;
  UInt  uiMinSAD    = MAX_UINT;
  for( UInt uiMode  = uiMinMode; uiMode < uiMaxMode; uiMode++ )
  {
    //--- get prediction ---
#if ANG_INTRA
    if( pcCU->angIntraEnabledPredPart( 0 ) ){
      predIntraChromaAng( pcCU->getPattern(), pPatChromaU, uiMode, piPredU, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
      predIntraChromaAng( pcCU->getPattern(), pPatChromaV, uiMode, piPredV, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
    }
    else{
      predIntraChromaAdi( pcCU->getPattern(), pPatChromaU, uiMode, piPredU, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
      predIntraChromaAdi( pcCU->getPattern(), pPatChromaV, uiMode, piPredV, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
    }
#else
    predIntraChromaAdi( pcCU->getPattern(), pPatChromaU, uiMode, piPredU, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
    predIntraChromaAdi( pcCU->getPattern(), pPatChromaV, uiMode, piPredV, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
    
    //--- get SAD ---
    UInt  uiSAD  = m_pcRdCost->calcHAD( piOrgU, uiStride, piPredU, uiStride, uiWidth, uiHeight );
    uiSAD       += m_pcRdCost->calcHAD( piOrgV, uiStride, piPredV, uiStride, uiWidth, uiHeight );
    //--- check ---
    if( uiSAD < uiMinSAD )
    {
      uiMinSAD   = uiSAD;
      uiBestMode = uiMode;
    }
  }
  
  //===== set chroma pred mode =====
  pcCU->setChromIntraDirSubParts( uiBestMode, 0, pcCU->getDepth( 0 ) );
}

Void 
TEncSearch::estIntraPredQT( TComDataCU* pcCU, 
                           TComYuv*    pcOrgYuv, 
                           TComYuv*    pcPredYuv, 
                           TComYuv*    pcResiYuv, 
                           TComYuv*    pcRecoYuv,
                           UInt&       ruiDistC,
                           Bool        bLumaOnly )
{
  UInt    uiDepth        = pcCU->getDepth(0);
  UInt    uiNumPU        = pcCU->getNumPartInter();
  UInt    uiInitTrDepth  = pcCU->getPartitionSize(0) == SIZE_2Nx2N ? 0 : 1;
  UInt    uiWidth        = pcCU->getWidth (0) >> uiInitTrDepth;
  UInt    uiHeight       = pcCU->getHeight(0) >> uiInitTrDepth;
  UInt    uiQNumParts    = pcCU->getTotalNumPart() >> 2;
  UInt    uiWidthBit     = pcCU->getIntraSizeIdx(0);
  UInt    uiOverallDistY = 0;
  UInt    uiOverallDistC = 0;
#if HHI_AIS
  Bool    bAISEnabled    = pcCU->getSlice()->getSPS()->getUseAIS();
  Bool    bDefaultIS     = ( bAISEnabled ? true : DEFAULT_IS );
#endif
#if ANG_INTRA
  Bool    angIntraEnabled= pcCU->angIntraEnabledPredPart( 0 );
#endif
  
  //===== set QP and clear Cbf =====
  pcCU->setQPSubParts( pcCU->getSlice()->getSliceQp(), 0, uiDepth );
  
  //===== loop over partitions =====
  UInt uiPartOffset = 0;
  for( UInt uiPU = 0; uiPU < uiNumPU; uiPU++, uiPartOffset += uiQNumParts )
  {
    //===== init pattern for luma prediction =====
    Bool bAboveAvail = false;
    Bool bLeftAvail  = false;
    pcCU->getPattern()->initPattern   ( pcCU, uiInitTrDepth, uiPartOffset );
    pcCU->getPattern()->initAdiPattern( pcCU, uiPartOffset, uiInitTrDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail );
#ifdef EDGE_BASED_PREDICTION
    if(getEdgeBasedPred()->get_edge_prediction_enable())
      getEdgeBasedPred()->initEdgeBasedBuffer(pcCU, uiPartOffset, uiInitTrDepth, m_piYExtEdgeBased);
#endif //EDGE_BASED_PREDICTION
    
    //===== determine set of modes to be tested (using prediction signal only) =====
#if ANG_INTRA
#if UNIFIED_DIRECTIONAL_INTRA
    UInt uiMaxMode     = angIntraEnabled ? g_aucIntraModeNumAng[uiWidthBit] : g_aucIntraModeNum[uiWidthBit];
#else
    UInt uiMaxMode     = angIntraEnabled ? 34 : g_aucIntraModeNum[uiWidthBit];
#endif
#else
    UInt uiMaxMode     = g_aucIntraModeNum    [ uiWidthBit ];
#endif
    UInt uiMaxModeFast = g_aucIntraModeNumFast[ uiWidthBit ];
    Pel* piOrg         = pcOrgYuv ->getLumaAddr( uiPU, uiWidth );
    Pel* piPred        = pcPredYuv->getLumaAddr( uiPU, uiWidth );
    UInt uiStride      = pcPredYuv->getStride();
    UInt uiBestSad     = MAX_UINT;
    UInt iBestPreMode  = 0;
    for( UInt uiMode = uiMaxModeFast; uiMode < uiMaxMode; uiMode++ )
    {
#if ANG_INTRA
      if ( !predIntraLumaDirAvailable( uiMode, uiWidthBit, angIntraEnabled, bAboveAvail, bLeftAvail ) )
        continue;
      
      if ( angIntraEnabled ){
#if HHI_AIS
        predIntraLumaAng( pcCU->getPattern(), uiMode, bDefaultIS, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
        predIntraLumaAng( pcCU->getPattern(), uiMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
      }
      else{
#if HHI_AIS
        predIntraLumaAdi( pcCU->getPattern(), uiMode, bDefaultIS, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
        predIntraLumaAdi( pcCU->getPattern(), uiMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
      }
      
#else // ANG_INTRA
      
      UInt uiNewMode = g_aucIntraModeOrder[uiWidthBit][uiMode];
      if ( ( g_aucIntraAvail[uiNewMode][0] && (!bAboveAvail) ) || ( g_aucIntraAvail[uiNewMode][1] && (!bLeftAvail) ) )
      {
        continue;
      }
#if HHI_AIS
      predIntraLumaAdi( pcCU->getPattern(), uiMode, bDefaultIS, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
      predIntraLumaAdi( pcCU->getPattern(), uiMode, piPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
#endif // ANG_INTRA
      
      // use hadamard transform here
      UInt uiSad = m_pcRdCost->calcHAD( piOrg, uiStride, piPred, uiStride, uiWidth, uiHeight );
      if ( uiSad < uiBestSad )
      {
        uiBestSad    = uiSad;
        iBestPreMode = uiMode;
      }
    }
    UInt uiRdModeList[10];
    UInt uiNewMaxMode;
    UInt uiMinMode = 0;
    for( Int i = 0; i < 10; i++ ) 
    {
      uiRdModeList[ i ] = i;
    }
    if( uiMaxModeFast >= uiMaxMode )
    {
      uiNewMaxMode = uiMaxMode;
    }
    else
    {
      uiNewMaxMode = uiMaxModeFast + 1;
      uiRdModeList[uiMaxModeFast] = iBestPreMode;
    }
    
    //===== check modes (using r-d costs) =====
#if HHI_AIS
    Bool    bBestISMode   = bDefaultIS;
#endif
    UInt    uiBestPUMode  = 0;
    UInt    uiBestPUDistY = 0;
    UInt    uiBestPUDistC = 0;
    Double  dBestPUCost   = MAX_DOUBLE;
    for( UInt uiMode = uiMinMode; uiMode < uiNewMaxMode; uiMode++ )
    {
      // set luma prediction mode
      UInt uiOrgMode = uiRdModeList[uiMode];
      
#if ANG_INTRA
      if ( !predIntraLumaDirAvailable( uiOrgMode, uiWidthBit, angIntraEnabled, bAboveAvail, bLeftAvail ) )
        continue;
#else
      UInt uiNewMode = g_aucIntraModeOrder[uiWidthBit][uiOrgMode];
      if ( ( g_aucIntraAvail[uiNewMode][0] && (!bAboveAvail) ) || ( g_aucIntraAvail[uiNewMode][1] && (!bLeftAvail) ) )
      {
        continue;
      }
#endif
      
      pcCU->setLumaIntraDirSubParts ( uiOrgMode, uiPartOffset, uiDepth + uiInitTrDepth );
      
#if HHI_AIS
      // set intra smoothing mode
      pcCU->setLumaIntraFiltFlagSubParts( bBestISMode, uiPartOffset, uiDepth + uiInitTrDepth );
#endif
      
      // set context models
      if( m_bUseSBACRD )
      {
        if( uiPU )  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth+1][CI_NEXT_BEST] );
        else        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth  ][CI_CURR_BEST] );
      }
      
      // determine residual for partition
      UInt   uiPUDistY = 0;
      UInt   uiPUDistC = 0;
      Double dPUCost   = 0.0;
      xRecurIntraCodingQT( pcCU, uiInitTrDepth, uiPartOffset, bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost );
      
      // check r-d cost
      if( dPUCost < dBestPUCost )
      {
        uiBestPUMode  = uiOrgMode;
        uiBestPUDistY = uiPUDistY;
        uiBestPUDistC = uiPUDistC;
        dBestPUCost   = dPUCost;
        
        xSetIntraResultQT( pcCU, uiInitTrDepth, uiPartOffset, bLumaOnly, pcRecoYuv );
        
        UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth(0) + uiInitTrDepth ) << 1 );
        ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx()       + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempCbf[0], pcCU->getCbf( TEXT_LUMA     ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempCbf[1], pcCU->getCbf( TEXT_CHROMA_U ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempCbf[2], pcCU->getCbf( TEXT_CHROMA_V ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        
        if( m_bUseSBACRD )
        {
          m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiDepth+1][CI_NEXT_BEST] );
        }
      }
    } // Mode loop
    
    
#if HHI_AIS
    //===== test all or selected modes with modified intra smoothing (execpt intra DC) =====
    if( bAISEnabled )
    {
      Bool bTestISMode = !bBestISMode;
      for( UInt uiMode = uiMinMode; uiMode < uiNewMaxMode; uiMode++ )
      {
        if( uiRdModeList[uiMode] == 2 )
        {
          continue;
        }
#if AIS_TEST_BEST
        if( uiRdModeList[uiMode] != uiBestPUMode )
        {
          continue;
        }
#endif
        
        // set luma prediction mode
        UInt uiOrgMode = uiRdModeList[uiMode];
        
#if ANG_INTRA
        if ( !predIntraLumaDirAvailable( uiOrgMode, uiWidthBit, angIntraEnabled, bAboveAvail, bLeftAvail ) )
          continue;
#else
        UInt uiNewMode = g_aucIntraModeOrder[uiWidthBit][uiOrgMode];
        if ( ( g_aucIntraAvail[uiNewMode][0] && (!bAboveAvail) ) || ( g_aucIntraAvail[uiNewMode][1] && (!bLeftAvail) ) )
        {
          continue;
        }
#endif
        
        pcCU->setLumaIntraDirSubParts ( uiOrgMode, uiPartOffset, uiDepth + uiInitTrDepth );
        
        // set intra smoothing mode
        pcCU->setLumaIntraFiltFlagSubParts( bTestISMode, uiPartOffset, uiDepth + uiInitTrDepth );
        
        // set context models
        if( m_bUseSBACRD )
        {
          if( uiPU )  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth+1][CI_NEXT_BEST] );
          else        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth  ][CI_CURR_BEST] );
        }
        
        // determine residual for partition
        UInt   uiPUDistY = 0;
        UInt   uiPUDistC = 0;
        Double dPUCost   = 0.0;
        xRecurIntraCodingQT( pcCU, uiInitTrDepth, uiPartOffset, bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost );
        
        // check r-d cost
        if( dPUCost < dBestPUCost )
        {
          bBestISMode   = bTestISMode;
          uiBestPUMode  = uiOrgMode;
          uiBestPUDistY = uiPUDistY;
          uiBestPUDistC = uiPUDistC;
          dBestPUCost   = dPUCost;
          
          xSetIntraResultQT( pcCU, uiInitTrDepth, uiPartOffset, bLumaOnly, pcRecoYuv );
          
          UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth(0) + uiInitTrDepth ) << 1 );
          ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx()       + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_puhQTTempCbf[0], pcCU->getCbf( TEXT_LUMA     ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_puhQTTempCbf[1], pcCU->getCbf( TEXT_CHROMA_U ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_puhQTTempCbf[2], pcCU->getCbf( TEXT_CHROMA_V ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          
          if( m_bUseSBACRD )
          {
            m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiDepth+1][CI_NEXT_BEST] );
          }
        }
      } // Mode loop
    } // AIS enabled
#endif
    
    
    //--- update overall distortion ---
    uiOverallDistY += uiBestPUDistY;
    uiOverallDistC += uiBestPUDistC;
    
    //--- update transform index and cbf ---
    UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth(0) + uiInitTrDepth ) << 1 );
    ::memcpy( pcCU->getTransformIdx()       + uiPartOffset, m_puhQTTempTrIdx,  uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getCbf( TEXT_LUMA     ) + uiPartOffset, m_puhQTTempCbf[0], uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getCbf( TEXT_CHROMA_U ) + uiPartOffset, m_puhQTTempCbf[1], uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getCbf( TEXT_CHROMA_V ) + uiPartOffset, m_puhQTTempCbf[2], uiQPartNum * sizeof( UChar ) );
    
    //--- set reconstruction for next intra prediction blocks ---
    if( uiPU != uiNumPU - 1 )
    {
      Bool bSkipChroma  = false;
      Bool bChromaSame  = false;
      UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> ( pcCU->getDepth(0) + uiInitTrDepth ) ] + 2;
      if( !bLumaOnly && uiLog2TrSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
      {
        assert( uiInitTrDepth  > 0 );
        bSkipChroma  = ( uiPU != 0 );
        bChromaSame  = true;
      }
      
      UInt    uiCompWidth   = pcCU->getWidth ( 0 ) >> uiInitTrDepth;
      UInt    uiCompHeight  = pcCU->getHeight( 0 ) >> uiInitTrDepth;
      UInt    uiZOrder      = pcCU->getZorderIdxInCU() + uiPartOffset;
      Pel*    piDes         = pcCU->getPic()->getPicYuvRec()->getLumaAddr( pcCU->getAddr(), uiZOrder );
      UInt    uiDesStride   = pcCU->getPic()->getPicYuvRec()->getStride();
      Pel*    piSrc         = pcRecoYuv->getLumaAddr( uiPartOffset );
      UInt    uiSrcStride   = pcRecoYuv->getStride();
      for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
      {
        for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
        {
          piDes[ uiX ] = piSrc[ uiX ];
        }
      }
      if( !bLumaOnly && !bSkipChroma )
      {
        if( !bChromaSame )
        {
          uiCompWidth   >>= 1;
          uiCompHeight  >>= 1;
        }
        piDes         = pcCU->getPic()->getPicYuvRec()->getCbAddr( pcCU->getAddr(), uiZOrder );
        uiDesStride   = pcCU->getPic()->getPicYuvRec()->getCStride();
        piSrc         = pcRecoYuv->getCbAddr( uiPartOffset );
        uiSrcStride   = pcRecoYuv->getCStride();
        for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
        {
          for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
          {
            piDes[ uiX ] = piSrc[ uiX ];
          }
        }
        piDes         = pcCU->getPic()->getPicYuvRec()->getCrAddr( pcCU->getAddr(), uiZOrder );
        piSrc         = pcRecoYuv->getCrAddr( uiPartOffset );
        for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
        {
          for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
          {
            piDes[ uiX ] = piSrc[ uiX ];
          }
        }
      }
    }
    
    //=== update PU data ====
#if HHI_AIS
    pcCU->setLumaIntraFiltFlagSubParts( bBestISMode,  uiPartOffset, uiDepth + uiInitTrDepth );
#endif
    pcCU->setLumaIntraDirSubParts     ( uiBestPUMode, uiPartOffset, uiDepth + uiInitTrDepth );
    pcCU->copyToPic                   ( uiDepth, uiPU, uiInitTrDepth );
  } // PU loop
  
  
  if( uiNumPU > 1 )
  { // set Cbf for all blocks
    UInt uiCombCbfY = 0;
    UInt uiCombCbfU = 0;
    UInt uiCombCbfV = 0;
    UInt uiPartIdx  = 0;
    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiPartIdx += uiQNumParts )
    {
      uiCombCbfY |= pcCU->getCbf( uiPartIdx, TEXT_LUMA,     1 );
      uiCombCbfU |= pcCU->getCbf( uiPartIdx, TEXT_CHROMA_U, 1 );
      uiCombCbfV |= pcCU->getCbf( uiPartIdx, TEXT_CHROMA_V, 1 );
    }
    for( UInt uiOffs = 0; uiOffs < 4 * uiQNumParts; uiOffs++ )
    {
      pcCU->getCbf( TEXT_LUMA     )[ uiOffs ] |= uiCombCbfY;
      pcCU->getCbf( TEXT_CHROMA_U )[ uiOffs ] |= uiCombCbfU;
      pcCU->getCbf( TEXT_CHROMA_V )[ uiOffs ] |= uiCombCbfV;
    }
  }
  
  //===== reset context models =====
  if(m_bUseSBACRD)
  {
    m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
  }
  
  //===== set distortion (rate and r-d costs are determined later) =====
  ruiDistC                   = uiOverallDistC;
  pcCU->getTotalDistortion() = uiOverallDistY + uiOverallDistC;
}



Void 
TEncSearch::estIntraPredChromaQT( TComDataCU* pcCU, 
                                 TComYuv*    pcOrgYuv, 
                                 TComYuv*    pcPredYuv, 
                                 TComYuv*    pcResiYuv, 
                                 TComYuv*    pcRecoYuv,
                                 UInt        uiPreCalcDistC )
{
  UInt    uiDepth     = pcCU->getDepth(0);
  UInt    uiBestMode  = 0;
  UInt    uiBestDist  = 0;
  Double  dBestCost   = MAX_DOUBLE;
  
  //----- init mode list -----
  Int   iIntraIdx = pcCU->getIntraSizeIdx(0);
  UInt  uiModeList[5];
  for( Int i = 0; i < 4; i++ )
  {
    uiModeList[i] = i;
  }
  
#if ANG_INTRA
  uiModeList[4]   = pcCU->angIntraEnabledPredPart( 0 ) ? pcCU->getLumaIntraDir(0) : g_aucIntraModeOrder[iIntraIdx][pcCU->getLumaIntraDir(0)];
#else
  uiModeList[4]   = g_aucIntraModeOrder[iIntraIdx][pcCU->getLumaIntraDir(0)];
#endif
  
  UInt  uiMinMode = 0;
  UInt  uiMaxMode = ( uiModeList[4] >= 4 ? 5 : 4 );
  
  //----- check chroma modes -----
  for( UInt uiMode = uiMinMode; uiMode < uiMaxMode; uiMode++ )
  {
    //----- restore context models -----
    if( m_bUseSBACRD )
    {
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );
    }
    
    //----- chroma coding -----
    UInt    uiDist = 0;
    pcCU->setChromIntraDirSubParts  ( uiMode, 0, uiDepth );
    xRecurIntraChromaCodingQT       ( pcCU,   0, 0, pcOrgYuv, pcPredYuv, pcResiYuv, uiDist );
    UInt    uiBits = xGetIntraBitsQT( pcCU,   0, 0, false, true, false );
    Double  dCost  = m_pcRdCost->calcRdCost( uiBits, uiDist );
    
    //----- compare -----
    if( dCost < dBestCost )
    {
      dBestCost   = dCost;
      uiBestDist  = uiDist;
      uiBestMode  = uiMode;
      UInt  uiQPN = pcCU->getPic()->getNumPartInCU() >> ( uiDepth << 1 );
      xSetIntraResultChromaQT( pcCU, 0, 0, pcRecoYuv );
      ::memcpy( m_puhQTTempCbf[1], pcCU->getCbf( TEXT_CHROMA_U ), uiQPN * sizeof( UChar ) );
      ::memcpy( m_puhQTTempCbf[2], pcCU->getCbf( TEXT_CHROMA_V ), uiQPN * sizeof( UChar ) );
    }
  }
  
  //----- set data -----
  UInt  uiQPN = pcCU->getPic()->getNumPartInCU() >> ( uiDepth << 1 );
  ::memcpy( pcCU->getCbf( TEXT_CHROMA_U ), m_puhQTTempCbf[1], uiQPN * sizeof( UChar ) );
  ::memcpy( pcCU->getCbf( TEXT_CHROMA_V ), m_puhQTTempCbf[2], uiQPN * sizeof( UChar ) );
  pcCU->setChromIntraDirSubParts( uiBestMode, 0, uiDepth );
  pcCU->getTotalDistortion      () += uiBestDist - uiPreCalcDistC;
  
  //----- restore context models -----
  if( m_bUseSBACRD )
  {
    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );
  }
}


#endif

#if ANG_INTRA
Bool TEncSearch::predIntraLumaDirAvailable( UInt uiMode, UInt uiWidthBit, Bool angIntraEnabled, Bool bAboveAvail, Bool bLeftAvail)
{
  Bool bDirAvailable = true;
  UInt uiNewMode     = angIntraEnabled ? g_aucAngIntraModeOrder[uiMode] : g_aucIntraModeOrder[uiWidthBit][uiMode];
  
  if ( angIntraEnabled ){
    if ( uiNewMode > 0 && ( (!bAboveAvail) && uiNewMode < 18 ) || ( (!bLeftAvail) && uiNewMode > 17 ) )
      bDirAvailable = false;
  }
  else{
    if ( ( g_aucIntraAvail[uiNewMode][0] && (!bAboveAvail) ) || ( g_aucIntraAvail[uiNewMode][1] && (!bLeftAvail) ) )
      bDirAvailable = false;
  }
  
  return bDirAvailable;
}
#endif

#if PLANAR_INTRA
Void TEncSearch::xIntraPlanarRecon( TComDataCU* pcCU, UInt uiAbsPartIdx, Pel* piOrg, Pel* piPred, Pel* piResi, Pel* piReco, UInt uiStride, TCoeff* piCoeff, UInt uiWidth, UInt uiHeight, UInt uiCurrDepth, TextType eText )
{
  UInt uiX, uiY;
  UInt uiReconStride;
  Int  iSample;
  
  Pel* pOrg  = piOrg;
  Pel* pPred = piPred;
  Pel* pResi = piResi;
  Pel* pReco = piReco;
  Pel* pRecoPic;
  Int* pPat;
  
  pcCU->getPattern()->initPattern( pcCU, uiCurrDepth, uiAbsPartIdx );
  
  Bool bAboveAvail = false;
  Bool bLeftAvail  = false;
  
  if( eText == TEXT_LUMA)
  {
    pcCU->getPattern()->initAdiPattern( pcCU, uiAbsPartIdx, uiCurrDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail);
    
    pPat          = pcCU->getPattern()->getAdiOrgBuf( uiWidth, uiHeight, m_piYuvExt );
    uiReconStride = pcCU->getPic()->getPicYuvRec()->getStride();
    pRecoPic      = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);
  }
  else
  {
    
#if HHI_RQT_INTRA
    pcCU->getPattern()->initAdiPatternChroma(pcCU,uiAbsPartIdx, 0, m_piYuvExt,m_iYuvExtStride,m_iYuvExtHeight,bAboveAvail,bLeftAvail);
#else
    pcCU->getPattern()->initAdiPatternChroma(pcCU, uiAbsPartIdx, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail);
#endif
    
    uiReconStride = pcCU->getPic()->getPicYuvRec()->getCStride();
    
    if( eText == TEXT_CHROMA_U )
    {
      pRecoPic = pcCU->getPic()->getPicYuvRec()->getCbAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);
      pPat     = pcCU->getPattern()->getAdiCbBuf( uiWidth, uiHeight, m_piYuvExt );
    }
    else
    {
      pRecoPic = pcCU->getPic()->getPicYuvRec()->getCrAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);
      pPat     = pcCU->getPattern()->getAdiCrBuf( uiWidth, uiHeight, m_piYuvExt );
    }
  }
  
  // Get the sample value for the bottom-right corner
  iSample = ( piOrg[(uiHeight-1)*uiStride + uiWidth - 1] +
             piOrg[(uiHeight-2)*uiStride + uiWidth - 1] +
             piOrg[(uiHeight-1)*uiStride + uiWidth - 2] +
             piOrg[(uiHeight-2)*uiStride + uiWidth - 2] + 2 ) >> 2;
  
  // Get prediction for the bottom-right sample value
  Int iPredBufStride = ( uiWidth<<1 ) + 1;
  Int iSamplePred    = predIntraGetPredValDC(pPat+iPredBufStride+1, iPredBufStride, uiWidth, uiHeight, bAboveAvail, bLeftAvail );
  Int iDelta         = iSample - iSamplePred;
  Int iSign          = iDelta < 0 ? -1 : 1;
  iDelta             = abs(iDelta) >> g_uiBitIncrement;
  
  // Quantize the difference value
  if( iDelta < 4 )
    iDelta = iDelta;
  else if( iDelta < 16 )
    iDelta = (iDelta>>1)<<1;
  else if( iDelta < 64 )
    iDelta = ((iDelta>>2)<<2)+2;
  else
    iDelta = ((iDelta>>3)<<3)+4;
  
  iDelta *= iSign;
  
  // Intermediate value to be passed to entropy coding (would be better to have here a continuous index instead and avoid replicating quantization steps in the entropy coder)
  if( eText == TEXT_LUMA)
    pcCU->setPlanarInfo( uiAbsPartIdx, PLANAR_DELTAY, iDelta );
  else if( eText == TEXT_CHROMA_U)
    pcCU->setPlanarInfo( uiAbsPartIdx, PLANAR_DELTAU, iDelta );
  else
    pcCU->setPlanarInfo( uiAbsPartIdx, PLANAR_DELTAV, iDelta );
  
  // Reconstructed sample value
  iDelta  = iDelta << g_uiBitIncrement;
  iSample = iDelta + iSamplePred;
  
  predIntraPlanar( pPat, iSample, pPred, uiStride, uiWidth, uiHeight, bAboveAvail, bLeftAvail );
  
  // Make residual from prediction. (MUST BE FIXED FOR EACH TRANSFORM UNIT PREDICTION)
  for( uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( uiX = 0; uiX < uiWidth; uiX++ )
    {
      pResi[uiX] = pOrg[uiX] - pPred[uiX];
    }
    pOrg  += uiStride;
    pResi += uiStride;
    pPred += uiStride;
  }
  
  pPred = piPred;
  pResi = piResi;
  
  //  m_pcTrQuant->transformNxN( pcCU, pResi, uiStride, piCoeff, uiWidth, uiHeight, uiAbsSum, eText, uiAbsPartIdx, indexROT );
  
  for( uiY = 0; uiY < uiHeight; uiY++ )
  {
    memset(pResi, 0, sizeof(Pel)*uiWidth);
    pResi += uiStride;
  }
  
  pPred = piPred;
  pResi = piResi;
  
  // Reconstruction
  for( uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( uiX = 0; uiX < uiWidth; uiX++ )
    {
      pReco   [uiX] = Clip(pPred[uiX]);
      pRecoPic[uiX] = pReco[uiX];
    }
    pReco    += uiStride;
    pResi    += uiStride;
    pPred    += uiStride;
    pRecoPic += uiReconStride;
  }
}

/// encoder estimation - planar intra prediction (luma & chroma)
Void TEncSearch::predIntraPlanarSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv )
{
  UInt   uiDepth        = pcCU->getDepth(0);
  UInt   uiWidth        = pcCU->getWidth(0);
  UInt   uiHeight       = pcCU->getHeight(0);
  UInt   uiStride       = rpcPredYuv->getStride();
  UInt   uiStrideC      = rpcPredYuv->getCStride();
  UInt   uiWidthC       = uiWidth  >> 1;
  UInt   uiHeightC      = uiHeight >> 1;
  UInt   uiBits;
  UInt   uiDistortion;
  
  Double dCost;
  
  Pel*    pOrig;
  Pel*    pResi;
  Pel*    pReco;
  Pel*    pPred;
  TCoeff* pCoeff;
  
  // Planar for Luminance
  pOrig    = pcOrgYuv->getLumaAddr(0, uiWidth);
  pResi    = rpcResiYuv->getLumaAddr(0, uiWidth);
  pPred    = rpcPredYuv->getLumaAddr(0, uiWidth);
  pReco    = rpcRecoYuv->getLumaAddr(0, uiWidth);
  pCoeff   = pcCU->getCoeffY();
  
  xIntraPlanarRecon( pcCU, 0, pOrig, pPred, pResi, pReco, uiStride, pCoeff, uiWidth, uiHeight, 0, TEXT_LUMA );
  
  uiDistortion  = m_pcRdCost->getDistPart( pReco, uiStride, pOrig, uiStride, uiWidth, uiHeight );
  
  // Planar for U
  pOrig    = pcOrgYuv->getCbAddr();
  pResi    = rpcResiYuv->getCbAddr();
  pPred    = rpcPredYuv->getCbAddr();
  pReco    = rpcRecoYuv->getCbAddr();
  pCoeff   = pcCU->getCoeffCb();
  
  xIntraPlanarRecon( pcCU, 0, pOrig, pPred, pResi, pReco, uiStrideC, pCoeff, uiWidthC, uiHeightC, 0, TEXT_CHROMA_U );
  
  uiDistortion += m_pcRdCost->getDistPart( pReco, uiStrideC, pOrig, uiStrideC, uiWidthC, uiHeightC );
  
  // Planar for V
  pOrig    = pcOrgYuv->getCrAddr();
  pResi    = rpcResiYuv->getCrAddr();
  pPred    = rpcPredYuv->getCrAddr();
  pReco    = rpcRecoYuv->getCrAddr();
  pCoeff   = pcCU->getCoeffCr();
  
  xIntraPlanarRecon( pcCU, 0, pOrig, pPred, pResi, pReco, uiStrideC, pCoeff, uiWidthC, uiHeightC, 0, TEXT_CHROMA_V );
  
  uiDistortion += m_pcRdCost->getDistPart( pReco, uiStrideC, pOrig, uiStrideC, uiWidthC, uiHeightC );
  
  xAddSymbolBitsIntra( pcCU, pCoeff, 0, 0, 0, 1, 0, 0, uiWidth, uiHeight, uiBits );
  
  dCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );
  
  if(m_bUseSBACRD)
    m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
  
  pcCU->getTotalBits()       = uiBits;
  pcCU->getTotalCost()       = dCost;
  pcCU->getTotalDistortion() = uiDistortion;
  
  pcCU->copyToPic(uiDepth, 0, 0);
  
}
#endif




Void TEncSearch::predIntraLumaAdiSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv )
{
  UInt   uiDepth        = pcCU->getDepth(0);
  UInt   uiNumPU        = pcCU->getNumPartInter();
  UInt   uiPartDepth    = pcCU->getPartitionSize(0) == SIZE_2Nx2N ? 0 : 1;
  UInt   uiWidth        = pcCU->getWidth(0) >> uiPartDepth;
  UInt   uiHeight       = pcCU->getHeight(0)>> uiPartDepth;
  UInt   uiCoeffSize    = uiWidth*uiHeight;
  
  UInt   uiWidthBit;
  
  UInt   uiNextDepth    = uiDepth + 1;
  
  UInt   uiQNumParts    = pcCU->getTotalNumPart()>>2;
  UInt   uiPU;
  
  UInt uiBestBits       = 0;
  UInt uiPUBestBits     = 0;
  UInt uiBits;
  
  UInt uiBestDistortion   = 0;
  UInt uiPUBestDistortion = 0;
  UInt uiDistortion;
  
  Double dBestCost      = MAX_DOUBLE;
  Double dPUBestCost    = MAX_DOUBLE;
  Double dCost;
  
  UInt uiTrLevel = 0;
  
  UInt uiWidthInBit  = g_aucConvertToBit[pcCU->getWidth(0)]+2;
  UInt uiTrSizeInBit = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxTrSize()]+2;
  uiTrLevel          = uiWidthInBit >= uiTrSizeInBit ? uiWidthInBit - uiTrSizeInBit : 0;
  
  UInt uiMaxTrDepth;
  
  uiMaxTrDepth   = pcCU->getSlice()->getSPS()->getMinTrDepth() + uiTrLevel + (pcCU->getPartitionSize(0) == SIZE_NxN ? 1 : 0);
  
  UInt   uiMinMode      = 0;
  UInt   uiMaxMode      = 1;
  UInt   uiPUBestMode   = 2;
  UInt   uiMode;
  UInt   uiPUMode[4];
  UInt   uiBestMode[4];
  
#if HHI_AIS
  // BB: AIS adaptive intra smoothing (filtering)
  Bool   bUseAIS = pcCU->getSlice()->getSPS()->getUseAIS();
  Bool   bPUBestFilt = bUseAIS ? true : DEFAULT_IS;
  Bool   bPUCurrFilt = bPUBestFilt;
  Bool   bPUFilt[4];    // BB: best per PU
  Bool   bBestFilt[4];  // BB: best per PU in dQp loop (currently the same)
  Double dPUNoFiltCost;
  UInt   uiPUNoFiltBits;
  UInt   uiPUNoFiltDistortion;
#endif
  
  Int    iMindQp        = 0;
  Int    iMaxdQp        = 0;
  Int    idQp;
  Int    iBestdQp       = 0;
  
  UInt   uiPartOffset;
  UInt   uiCoeffOffset;
  
  Pel*    pOrg;
  Pel*    pResi;
  Pel*    pReco;
  Pel*    pPred;
  TCoeff* pCoeff;
  
  UInt    uiStride  = rpcPredYuv->getStride();
  
  TComPattern* pcPattern  = pcCU->getPattern();
  
#if ANG_INTRA
  Bool angIntraEnabled    = pcCU->angIntraEnabledPredPart( 0 );
#endif
  
  pcCU->setTrIdxSubParts( uiMaxTrDepth, 0, uiDepth );
  uiWidthBit = pcCU->getIntraSizeIdx(0);
  
  for ( idQp = iMindQp; idQp <= iMaxdQp; idQp++ )
  {
    UInt uiPUBits = 0;
    UInt uiPUDistortion = 0;
    Double dPUCost = 0;
    
    pcCU->clearCbf(0, TEXT_LUMA, pcCU->getTotalNumPart());
    
    // Set Qp for quantization.
    m_pcTrQuant->setQPforQuant( pcCU->getSlice()->getSliceQp()+idQp,  !pcCU->getSlice()->getDepth() , pcCU->getSlice()->getSliceType(), TEXT_LUMA );
    pcCU->setQPSubParts( pcCU->getSlice()->getSliceQp()+idQp, 0, uiDepth );
    
    uiPartOffset  = 0;
    uiCoeffOffset = 0;
    for( uiPU = 0; uiPU < uiNumPU; uiPU++ )
    {
      pcPattern->initPattern( pcCU, uiPartDepth, uiPartOffset );
      
      // ADI ADDED
      Bool bAboveAvail = false;
      Bool bLeftAvail  = false;
      
      pcPattern->initAdiPattern(pcCU, uiPartOffset, uiPartDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail);
#ifdef EDGE_BASED_PREDICTION
      if(getEdgeBasedPred()->get_edge_prediction_enable())
        getEdgeBasedPred()->initEdgeBasedBuffer(pcCU, uiPartOffset, uiPartDepth, m_piYExtEdgeBased);
#endif //EDGE_BASED_PREDICTION
      
#if ANG_INTRA
#if UNIFIED_DIRECTIONAL_INTRA
      uiMaxMode          = angIntraEnabled ? g_aucIntraModeNumAng[uiWidthBit] : g_aucIntraModeNum[uiWidthBit];
#else
      uiMaxMode          = angIntraEnabled ? 34 : g_aucIntraModeNum[uiWidthBit];
#endif
#else
      uiMaxMode          = g_aucIntraModeNum    [uiWidthBit];
#endif
      UInt uiMaxModeFast = g_aucIntraModeNumFast[uiWidthBit];
      
      pOrg     = pcOrgYuv->getLumaAddr  (uiPU, uiWidth);
      pResi    = rpcResiYuv->getLumaAddr(uiPU, uiWidth);
      pReco    = rpcRecoYuv->getLumaAddr(uiPU, uiWidth);
      pPred    = rpcPredYuv->getLumaAddr(uiPU, uiWidth);
      pCoeff   = pcCU->getCoeffY()  + uiCoeffOffset;
      
      dPUBestCost = MAX_DOUBLE;
      
      UInt uiBestSad    = MAX_UINT;
      UInt iBestPreMode = 0;
      
      for ( uiMode = uiMaxModeFast; uiMode < uiMaxMode; uiMode++ )
      {
#if ANG_INTRA
        if ( !predIntraLumaDirAvailable( uiMode, uiWidthBit, angIntraEnabled, bAboveAvail, bLeftAvail ) )
          continue;
        
        if ( angIntraEnabled ){
#if HHI_AIS
          predIntraLumaAng( pcPattern, uiMode, bPUCurrFilt, pPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
          predIntraLumaAng( pcPattern, uiMode, pPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
        }
        else{
#if HHI_AIS
          predIntraLumaAdi( pcPattern, uiMode, bPUCurrFilt, pPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
          predIntraLumaAdi( pcPattern, uiMode, pPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
        }
        
#else // ANG_INTRA
        UInt uiNewMode = g_aucIntraModeOrder[uiWidthBit][uiMode];
        
        if ( ( g_aucIntraAvail[uiNewMode][0] && (!bAboveAvail) ) || ( g_aucIntraAvail[uiNewMode][1] && (!bLeftAvail) ) )
          continue;
        
#if HHI_AIS
        predIntraLumaAdi( pcPattern, uiMode, bPUCurrFilt, pPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#else
        predIntraLumaAdi( pcPattern, uiMode, pPred, uiStride, uiWidth, uiHeight, pcCU, bAboveAvail, bLeftAvail );
#endif
#endif // ANG_INTRA
        
        Pel* piOrgY  = pcOrgYuv  ->getLumaAddr(uiPU, uiWidth);
        Pel* piPreY  = rpcPredYuv->getLumaAddr(uiPU, uiWidth);
        UInt iStride = pcOrgYuv  ->getStride  ();
        
        // use hadamard transform here
        UInt uiSad   = m_pcRdCost->calcHAD( piOrgY, iStride, piPreY, iStride, uiWidth, uiHeight );
        if ( uiSad < uiBestSad )
        {
          uiBestSad    = uiSad;
          iBestPreMode = uiMode;
        }
      }
      
      UInt uiRdModeList[10];
      UInt uiNewMaxMode;
      for (Int i=0;i<10;i++) uiRdModeList[i]=i;
      
      if (uiMaxModeFast>=uiMaxMode)
      {
        uiNewMaxMode=uiMaxMode;
      }
      else
      {
        uiNewMaxMode=uiMaxModeFast+1;
        uiRdModeList[uiMaxModeFast]=iBestPreMode;
      }
      
      bAboveAvail = false;
      bLeftAvail  = false;
      pcPattern->initPattern( pcCU, uiPartDepth, uiPartOffset );
      pcPattern->initAdiPattern(pcCU, uiPartOffset, uiPartDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail);
#ifdef EDGE_BASED_PREDICTION
      if(getEdgeBasedPred()->get_edge_prediction_enable())
        getEdgeBasedPred()->initEdgeBasedBuffer(pcCU, uiPartOffset, uiPartDepth, m_piYExtEdgeBased);
#endif //EDGE_BASED_PREDICTION
      
      for ( uiMode = uiMinMode; uiMode < uiNewMaxMode; uiMode++ )
      {
        UInt uiOrgMode = uiRdModeList[uiMode];
        
#if ANG_INTRA
        if ( !predIntraLumaDirAvailable( uiOrgMode, uiWidthBit, angIntraEnabled, bAboveAvail, bLeftAvail ) )
          continue;
#else
        UInt uiNewMode = g_aucIntraModeOrder[uiWidthBit][uiOrgMode];
        
        if ( ( g_aucIntraAvail[uiNewMode][0] && (!bAboveAvail) ) || ( g_aucIntraAvail[uiNewMode][1] && (!bLeftAvail) ) )
          continue;
#endif
        
        uiBits = 0;
        pcCU->setLumaIntraDirSubParts     ( uiOrgMode,   uiPartOffset, uiPartDepth+uiDepth );
        
#if HHI_AIS
        bPUCurrFilt    = bUseAIS ? true : DEFAULT_IS;
        pcCU->setLumaIntraFiltFlagSubParts( bPUCurrFilt, uiPartOffset, uiPartDepth+uiDepth );
#endif
        
        if(m_bUseSBACRD)
        {
          if( uiPU )
            m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiNextDepth][CI_NEXT_BEST]);
          else
            m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
        }
        
#if HHI_AIS
        xRecurIntraLumaSearchADI( pcCU, uiPartOffset, pOrg, pPred, pResi, pReco, uiStride, pCoeff, uiOrgMode, bPUCurrFilt, uiWidth, uiHeight, uiMaxTrDepth, uiPartDepth, bAboveAvail,bLeftAvail, (uiMaxTrDepth>uiPartDepth)? 1:0);
#else
        xRecurIntraLumaSearchADI( pcCU, uiPartOffset, pOrg, pPred, pResi, pReco, uiStride, pCoeff, uiOrgMode, uiWidth, uiHeight, uiMaxTrDepth, uiPartDepth, bAboveAvail,bLeftAvail, (uiMaxTrDepth>uiPartDepth)? 1:0);
#endif
        pcCU->setCuCbfLuma( uiPartOffset, uiMaxTrDepth, uiPartDepth );
        
        
        uiDistortion = m_pcRdCost->getDistPart( pReco, uiStride, pcOrgYuv->getLumaAddr(uiPU, uiWidth), uiStride, uiWidth, uiHeight );
        
        if(m_bUseSBACRD)
        {
          if( uiPU )
            m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiNextDepth][CI_NEXT_BEST]);
          else
            m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
        }
        
        xAddSymbolBitsIntra( pcCU, pCoeff, uiPU, uiQNumParts, uiPartDepth, 1, uiMaxTrDepth, uiPartDepth, uiWidth, uiHeight, uiBits );
        
        dCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );
        
        if( dCost < dPUBestCost )
        {
          uiPUBestMode       = uiOrgMode;
#if HHI_AIS
          bPUBestFilt        = bPUCurrFilt;
#endif
          uiPUBestBits       = uiBits;
          uiPUBestDistortion = uiDistortion;
          dPUBestCost        = dCost;
          
          if( m_bUseSBACRD )
            m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiNextDepth][CI_TEMP_BEST] );
        }
        
#if HHI_AIS
#if !AIS_TEST_BEST
        // BB: test every mode without filtering (execpt intra DC)
        ////////////////////////////////////////////////////////////////////////////
        if ( bUseAIS && (uiOrgMode != 2) )
        {
          dPUNoFiltCost        = MAX_DOUBLE;
          uiPUNoFiltBits       = 0;
          uiPUNoFiltDistortion = 0;
          bPUCurrFilt          = false;
          
          pcCU->setLumaIntraDirSubParts     ( uiOrgMode,   uiPartOffset, uiPartDepth+uiDepth );
          pcCU->setLumaIntraFiltFlagSubParts( bPUCurrFilt, uiPartOffset, uiPartDepth+uiDepth );
          
          if(m_bUseSBACRD)
          {
            if( uiPU )
              m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiNextDepth][CI_NEXT_BEST]);
            else
              m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
          }
          
          xRecurIntraLumaSearchADI( pcCU, uiPartOffset, pOrg, pPred, pResi, pReco, uiStride, pCoeff, uiOrgMode, bPUCurrFilt, uiWidth, uiHeight, uiMaxTrDepth, uiPartDepth, bAboveAvail,bLeftAvail, (uiMaxTrDepth>uiPartDepth)? 1:0);
          pcCU->setCuCbfLuma( uiPartOffset, uiMaxTrDepth, uiPartDepth );
          
          
          uiPUNoFiltDistortion = m_pcRdCost->getDistPart( pReco, uiStride, pcOrgYuv->getLumaAddr(uiPU, uiWidth), uiStride, uiWidth, uiHeight );
          
          if(m_bUseSBACRD)
          {
            if( uiPU )
              m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiNextDepth][CI_NEXT_BEST]);
            else
              m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
          }
          
          xAddSymbolBitsIntra( pcCU, pCoeff, uiPU, uiQNumParts, uiPartDepth, 1, uiMaxTrDepth, uiPartDepth, uiWidth, uiHeight, uiPUNoFiltBits );
          
          dPUNoFiltCost = m_pcRdCost->calcRdCost( uiPUNoFiltBits, uiPUNoFiltDistortion );
          
          if( dPUNoFiltCost < dPUBestCost )
          {
            uiPUBestMode       = uiOrgMode;
            bPUBestFilt        = bPUCurrFilt;
            uiPUBestBits       = uiPUNoFiltBits;
            uiPUBestDistortion = uiPUNoFiltDistortion;
            dPUBestCost        = dPUNoFiltCost;
            
            if( m_bUseSBACRD )
              m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiNextDepth][CI_TEMP_BEST] );
          }
        }
        ////////////////////////////////////////////////////////////////////////////
#endif
        
#endif
      } // Mode loop
      
#if HHI_AIS
#if AIS_TEST_BEST
      // BB: test best mode without filtering (execpt intra DC)
      ////////////////////////////////////////////////////////////////////////////
      if ( bUseAIS && (uiPUBestMode != 2) )
      {
        dPUNoFiltCost        = MAX_DOUBLE;
        uiPUNoFiltBits       = 0;
        uiPUNoFiltDistortion = 0;
        bPUCurrFilt          = false;
        
        pcCU->setLumaIntraDirSubParts     ( uiPUBestMode,   uiPartOffset, uiPartDepth+uiDepth );
        pcCU->setLumaIntraFiltFlagSubParts( bPUCurrFilt, uiPartOffset, uiPartDepth+uiDepth );
        
        if(m_bUseSBACRD)
        {
          if( uiPU )
            m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiNextDepth][CI_NEXT_BEST]);
          else
            m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
        }
        
        xRecurIntraLumaSearchADI( pcCU, uiPartOffset, pOrg, pPred, pResi, pReco, uiStride, pCoeff, uiPUBestMode, bPUCurrFilt, uiWidth, uiHeight, uiMaxTrDepth, uiPartDepth, bAboveAvail,bLeftAvail, (uiMaxTrDepth>uiPartDepth)? 1:0);
        pcCU->setCuCbfLuma( uiPartOffset, uiMaxTrDepth, uiPartDepth );
        
        
        uiPUNoFiltDistortion = m_pcRdCost->getDistPart( pReco, uiStride, pcOrgYuv->getLumaAddr(uiPU, uiWidth), uiStride, uiWidth, uiHeight );
        
        if(m_bUseSBACRD)
        {
          if( uiPU )
            m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiNextDepth][CI_NEXT_BEST]);
          else
            m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
        }
        
        xAddSymbolBitsIntra( pcCU, pCoeff, uiPU, uiQNumParts, uiPartDepth, 1, uiMaxTrDepth, uiPartDepth, uiWidth, uiHeight, uiPUNoFiltBits );
        
        dPUNoFiltCost = m_pcRdCost->calcRdCost( uiPUNoFiltBits, uiPUNoFiltDistortion );
        
        if( dPUNoFiltCost < dPUBestCost )
        {
          bPUBestFilt        = bPUCurrFilt;
          uiPUBestBits       = uiPUNoFiltBits;
          uiPUBestDistortion = uiPUNoFiltDistortion;
          dPUBestCost        = dPUNoFiltCost;
          
          if( m_bUseSBACRD )
            m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiNextDepth][CI_TEMP_BEST] );
        }
        pcCU->setLumaIntraFiltFlagSubParts( bPUBestFilt,     uiPartOffset, uiPartDepth+uiDepth );
      }
      ////////////////////////////////////////////////////////////////////////////
#endif
      
      bPUFilt[uiPU]       = bPUBestFilt;
#endif
      uiPUMode[uiPU]      = uiPUBestMode;
      uiPUBits           += uiPUBestBits;
      uiPUDistortion     += uiPUBestDistortion;
      dPUCost            += dPUBestCost;
      
      pOrg   = pcOrgYuv->getLumaAddr(uiPU, uiWidth);
      pResi  = rpcResiYuv->getLumaAddr(uiPU, uiWidth);
      pPred  = rpcPredYuv->getLumaAddr(uiPU, uiWidth);
      pReco  = rpcRecoYuv->getLumaAddr(uiPU, uiWidth);
      pCoeff = pcCU->getCoeffY()  + uiCoeffOffset;
      
      pcCU->setLumaIntraDirSubParts     ( uiPUBestMode,    uiPartOffset, uiPartDepth+uiDepth );
#if HHI_AIS
      pcCU->setLumaIntraFiltFlagSubParts( bPUBestFilt,     uiPartOffset, uiPartDepth+uiDepth );
#endif
      
      if(m_bUseSBACRD)
      {
        if( uiPU )
          m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiNextDepth][CI_NEXT_BEST]);
        else
          m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
      }
      
#if HHI_AIS
      xRecurIntraLumaSearchADI( pcCU, uiPartOffset, pOrg, pPred, pResi, pReco, uiStride, pCoeff, uiPUBestMode, bPUBestFilt, uiWidth, uiHeight, uiMaxTrDepth, uiPartDepth, bAboveAvail,bLeftAvail,(uiMaxTrDepth>uiPartDepth)? 1:0  );
#else
      xRecurIntraLumaSearchADI( pcCU, uiPartOffset, pOrg, pPred, pResi, pReco, uiStride, pCoeff, uiPUBestMode, uiWidth, uiHeight, uiMaxTrDepth, uiPartDepth, bAboveAvail,bLeftAvail,(uiMaxTrDepth>uiPartDepth)? 1:0  );
#endif
      pcCU->setCuCbfLuma( uiPartOffset, uiMaxTrDepth, uiPartDepth );
      
      pcCU->copyToPic(uiDepth, uiPU, uiPartDepth);
      
      if( m_bUseSBACRD )
        m_pppcRDSbacCoder[uiNextDepth][CI_TEMP_BEST]->store( m_pppcRDSbacCoder[uiNextDepth][CI_NEXT_BEST] );
      
      uiPartOffset  += uiQNumParts;
      uiCoeffOffset += uiCoeffSize;
    } // PU loop
    
    
    pcCU->setCuCbfLuma( 0, uiMaxTrDepth, 0);
    
    if( dPUCost < dBestCost )
    {
      uiBestMode[0] = uiPUMode[0]; uiBestMode[1] = uiPUMode[1]; uiBestMode[2] = uiPUMode[2]; uiBestMode[3] = uiPUMode[3];
#if HHI_AIS
      bBestFilt[0]  =  bPUFilt[0];  bBestFilt[1] =  bPUFilt[1];  bBestFilt[2] =  bPUFilt[2];  bBestFilt[3] =  bPUFilt[3];
#endif
      uiBestDistortion = uiPUDistortion;
      iBestdQp   = idQp;
      dBestCost  = dPUCost;
      uiBestBits = uiPUBits;
      
      if(m_bUseSBACRD)
      {
        if ( uiMaxTrDepth < uiPartDepth )
        {
          m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
        }
        else
        {
          m_pppcRDSbacCoder[uiNextDepth][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
        }
      }
    }
  } // dQp loop
  
  
  //Finalize Intra Coding
  pcCU->getTotalBits()       = uiBestBits;
  pcCU->getTotalCost()       = dBestCost;
  pcCU->getTotalDistortion() = uiBestDistortion;
  
  if(m_bUseSBACRD)
    m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
  
  pcCU->setQPSubParts( pcCU->getSlice()->getSliceQp()+iBestdQp, 0, uiDepth );
  pcCU->setTrIdxSubParts( uiMaxTrDepth, 0, uiDepth );
  uiWidthBit = pcCU->getIntraSizeIdx(0);
  
  uiPartOffset  = 0;
  uiCoeffOffset = 0;
  
  for( uiPU = 0; uiPU < uiNumPU; uiPU++ )
  {
    pcCU->setLumaIntraDirSubParts     ( uiBestMode[uiPU],     uiPartOffset, uiPartDepth+uiDepth );
#if HHI_AIS
    pcCU->setLumaIntraFiltFlagSubParts( bBestFilt[uiPU],      uiPartOffset, uiPartDepth+uiDepth );
#endif
    
    pOrg     = pcOrgYuv->getLumaAddr(uiPU, uiWidth);
    pResi    = rpcResiYuv->getLumaAddr(uiPU, uiWidth);
    pPred    = rpcPredYuv->getLumaAddr(uiPU, uiWidth);
    pReco    = rpcRecoYuv->getLumaAddr(uiPU, uiWidth);
    uiStride = rpcPredYuv->getStride();
    pCoeff   = pcCU->getCoeffY()  + uiCoeffOffset;
    
    pcPattern->initPattern( pcCU, uiPartDepth, uiPartOffset );
    
    // ADI ADDED
    Bool bAboveAvail = false;
    Bool bLeftAvail  = false;
    
    pcPattern->initAdiPattern(pcCU, uiPartOffset, uiPartDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail);
#ifdef EDGE_BASED_PREDICTION
    if(getEdgeBasedPred()->get_edge_prediction_enable())
      getEdgeBasedPred()->initEdgeBasedBuffer(pcCU, uiPartOffset, uiPartDepth, m_piYExtEdgeBased);
#endif //EDGE_BASED_PREDICTION
    
    
#if HHI_AIS
    xRecurIntraLumaSearchADI( pcCU, uiPartOffset, pOrg, pPred, pResi, pReco, uiStride, pCoeff, uiBestMode[uiPU], bBestFilt[uiPU], uiWidth, uiHeight, uiMaxTrDepth, uiPartDepth, bAboveAvail,bLeftAvail,(uiMaxTrDepth>uiPartDepth)? 1:0);
#else
    xRecurIntraLumaSearchADI( pcCU, uiPartOffset, pOrg, pPred, pResi, pReco, uiStride, pCoeff, uiBestMode[uiPU], uiWidth, uiHeight, uiMaxTrDepth, uiPartDepth, bAboveAvail,bLeftAvail,(uiMaxTrDepth>uiPartDepth)? 1:0);
#endif
    pcCU->setCuCbfLuma( uiPartOffset, uiMaxTrDepth, uiPartDepth );
    
    pcCU->copyToPic(uiDepth, uiPU, uiPartDepth);
    
    uiPartOffset  += uiQNumParts;
    uiCoeffOffset += uiCoeffSize;
  }
}

Void TEncSearch::predIntraChromaAdiSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv, UInt uiChromaTrMode )
{
  UInt    uiBestBits       = 0;
  UInt    uiBestDistortion = 0;
  UInt    uiDistortion     = 0;
  
  Double  fBestCost        = MAX_DOUBLE;
  Double  fCost            = 0;
  
  UInt    uiWidth          = pcCU->getWidth(0) >>1;
  UInt    uiHeight         = pcCU->getHeight(0)>>1;
  
  // Mode
  UInt    uiMinMode        = 0;
  UInt    uiMaxMode        = 4;
  UInt    uiBestMode       = 0;
  UInt    uiMode;
  
  TComPattern* pcPattern = pcCU->getPattern();
  
  // Buffer pointers for each transform unit
  Pel*    pOrigCb = pcOrgYuv  ->getCbAddr();
  Pel*    pResiCb = rpcResiYuv->getCbAddr();
  Pel*    pRecoCb = rpcRecoYuv->getCbAddr();
  Pel*    pPredCb = rpcPredYuv->getCbAddr();
  TCoeff* pCoefCb = pcCU      ->getCoeffCb();
  
  Pel*    pOrigCr = pcOrgYuv  ->getCrAddr();
  Pel*    pResiCr = rpcResiYuv->getCrAddr();
  Pel*    pRecoCr = rpcRecoYuv->getCrAddr();
  Pel*    pPredCr = rpcPredYuv->getCrAddr();
  TCoeff* pCoefCr = pcCU      ->getCoeffCr();
  UInt    uiStride = pcOrgYuv ->getCStride();
  
  // Set Qp for quantization.
  m_pcTrQuant->setQPforQuant( pcCU->getQP(0),  !pcCU->getSlice()->getDepth() , pcCU->getSlice()->getSliceType(), TEXT_CHROMA );
  
  if (m_pcEncCfg->getUseRDOQ())
  {
    if( m_bUseSBACRD )
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CHROMA_INTRA] );
    m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, uiWidth, TEXT_CHROMA);
  }
  
  pcPattern->initPattern( pcCU, 0, 0 );
  
  Int  iIntraIdx      = pcCU->getIntraSizeIdx(0);
  
  UInt    uiModeList[5];
  for (Int i=0;i<4;i++)
    uiModeList[i]=i;
  
#if ANG_INTRA
  uiModeList[4] = pcCU->angIntraEnabledPredPart( 0 ) ? pcCU->getLumaIntraDir( 0 ) : g_aucIntraModeOrder[iIntraIdx][pcCU->getLumaIntraDir( 0 )];
#else
  uiModeList[4] = g_aucIntraModeOrder[iIntraIdx][pcCU->getLumaIntraDir(0)];
#endif
  
  if (uiModeList[4]>=4) uiMaxMode=5;
  
  // Prediction Mode loop
  for ( uiMode = uiMinMode; uiMode < uiMaxMode; uiMode++ )
  {
    pcCU->setChromIntraDirSubParts(  uiMode, 0, pcCU->getDepth(0) ); // for RD cost
    
    pcCU->clearCbf(0, TEXT_CHROMA_U, pcCU->getTotalNumPart());
    pcCU->clearCbf(0, TEXT_CHROMA_V, pcCU->getTotalNumPart());
    
    xRecurIntraChromaSearchADI( pcCU, 0,  pOrigCb, pPredCb, pResiCb, pRecoCb, uiStride, pCoefCb, uiModeList[uiMode], uiWidth, uiHeight, uiChromaTrMode, 0, TEXT_CHROMA_U);
    xRecurIntraChromaSearchADI( pcCU, 0,  pOrigCr, pPredCr, pResiCr, pRecoCr, uiStride, pCoefCr, uiModeList[uiMode], uiWidth, uiHeight, uiChromaTrMode, 0, TEXT_CHROMA_V);
    
    pcCU->setCuCbfChroma( 0, uiChromaTrMode );
    
    // For RD compare
    uiDistortion  = m_pcRdCost->getDistPart( pRecoCb, uiStride, pOrigCb, uiStride, uiWidth, uiHeight );
    uiDistortion += m_pcRdCost->getDistPart( pRecoCr, uiStride, pOrigCr, uiStride, uiWidth, uiHeight );
    
    // load Going on CI from the CI_CHROMA
    if( m_bUseSBACRD )
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CHROMA_INTRA] );
    
    // Entropy coding
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeIntraDirModeChroma( pcCU, 0, true );
    
    m_pcEntropyCoder->encodeCbf( pcCU, 0, TEXT_CHROMA_U, 0 );
    m_pcEntropyCoder->encodeCoeff(pcCU, pCoefCb, 0, pcCU->getDepth(0), uiWidth, uiHeight, uiChromaTrMode, 0, TEXT_CHROMA_U);
    
    m_pcEntropyCoder->encodeCbf( pcCU, 0, TEXT_CHROMA_V, 0 );
    m_pcEntropyCoder->encodeCoeff(pcCU, pCoefCr, 0, pcCU->getDepth(0), uiWidth, uiHeight, uiChromaTrMode, 0, TEXT_CHROMA_V);
    UInt uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    
    // Calculate RD cost
    fCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );
    
    // Choose Best RD mode
    if( fCost < fBestCost )
    {
      // Keep best_dir, best Qp, best transform index, best_distortion, best_bit, best_RDcost
      uiBestBits       = uiBits;
      uiBestDistortion = uiDistortion;
      uiBestMode       = uiMode;
      fBestCost        = fCost;
      
      // store Going on CI from the Temp best buffer
      if( m_bUseSBACRD )
        m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_TEMP_BEST] );
    }
  } // End of Mode loop
  
  // Set bests
  pcCU->getTotalBits()       += uiBestBits;
  pcCU->getTotalCost()       += fBestCost;
  pcCU->getTotalDistortion() += uiBestDistortion;
  pcCU->setChromIntraDirSubParts( uiBestMode, 0, pcCU->getDepth( 0 ) );
  
  pcCU->clearCbf(0, TEXT_CHROMA_U, pcCU->getTotalNumPart());
  pcCU->clearCbf(0, TEXT_CHROMA_V, pcCU->getTotalNumPart());
  
  // Best prediction (MUST BE FIXED FOR EACH TRANSFORM UNIT PREDICTION)
  xRecurIntraChromaSearchADI( pcCU, 0,  pOrigCb, pPredCb, pResiCb, pRecoCb, uiStride, pCoefCb, uiModeList[uiBestMode], uiWidth, uiHeight, uiChromaTrMode, 0, TEXT_CHROMA_U);
  xRecurIntraChromaSearchADI( pcCU, 0,  pOrigCr, pPredCr, pResiCr, pRecoCr, uiStride, pCoefCr, uiModeList[uiBestMode], uiWidth, uiHeight, uiChromaTrMode, 0, TEXT_CHROMA_V);
  
  pcCU->setCuCbfChroma( 0, uiChromaTrMode );
}

#if HHI_MRG_PU
#ifdef DCM_PBIC
Void TEncSearch::xMergeEstimation( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, Int* piRefIdxPred, TComMv* pcMvTemp, TComIc& rcIcTemp, UInt& uiInterDir, UInt& uiMergeIndex, UInt& ruiCost )
#else
Void TEncSearch::xMergeEstimation( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, Int* piRefIdxPred, TComMv* pcMvTemp, UInt& uiInterDir, UInt& uiMergeIndex, UInt& ruiCost )
#endif
{
  UInt        uiNeighbourInfos = 0;
  TComMvField  cMFieldNeighbours[4]; // 0: above ref_list 0, above ref list 1, left ref list 0, left ref list 1
#ifdef DCM_PBIC
  TComIc cIcNeighbours[2]; //above, left
#endif
  UChar uhInterDirNeighbours[2];
  uhInterDirNeighbours[0] = 0;
  uhInterDirNeighbours[1] = 0;
  
  UInt uiCostTemp = MAX_UINT;
  UInt uiBitsTemp = 0;
  ruiCost = MAX_UINT;

  Int iNumPredDir = pcCU->getSlice()->isInterP() ? 1 : 2;

  UInt uiAbsPartIdx = 0;
  Int iWidth = 0;
  Int iHeight = 0;
  pcCU->getPartIndexAndSize( iPartIdx, uiAbsPartIdx, iWidth, iHeight );
#ifdef DCM_PBIC
  pcCU->getInterMergeCandidates( uiAbsPartIdx, cMFieldNeighbours, cIcNeighbours, uhInterDirNeighbours, uiNeighbourInfos );
#else
  pcCU->getInterMergeCandidates( uiAbsPartIdx, cMFieldNeighbours, uhInterDirNeighbours, uiNeighbourInfos );
#endif

  UInt uiNeighbourIdx = 0; // 0: top, 1: left
  if ( uiNeighbourInfos == 2 )
  {
    // test left parameters first
    uiNeighbourIdx = 1;
  }

  TComYuv cYuvPred;
  cYuvPred.create( pcYuvOrg->getWidth(), pcYuvOrg->getHeight() );

  while ( uiNeighbourInfos > 0 && uiNeighbourIdx < 2 )
  {
    uiCostTemp = MAX_UINT;    
    uiBitsTemp = 0;

    PartSize ePartSize = pcCU->getPartitionSize( 0 );

#ifdef QC_AMVRES
    if (pcCU->getSlice()->getSPS()->getUseAMVRes())
    {
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField_AMVRes( cMFieldNeighbours[0 + 2*uiNeighbourIdx].getMv(), cMFieldNeighbours[0 + 2*uiNeighbourIdx].getRefIdx(), ePartSize, uiAbsPartIdx, iPartIdx, 0 );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField_AMVRes( cMFieldNeighbours[1 + 2*uiNeighbourIdx].getMv(), cMFieldNeighbours[1 + 2*uiNeighbourIdx].getRefIdx(), ePartSize, uiAbsPartIdx, iPartIdx, 0 );
    }
    else
    {
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( cMFieldNeighbours[0 + 2*uiNeighbourIdx].getMv(), cMFieldNeighbours[0 + 2*uiNeighbourIdx].getRefIdx(), ePartSize, uiAbsPartIdx, iPartIdx, 0 );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( cMFieldNeighbours[1 + 2*uiNeighbourIdx].getMv(), cMFieldNeighbours[1 + 2*uiNeighbourIdx].getRefIdx(), ePartSize, uiAbsPartIdx, iPartIdx, 0 );
    }
#else
    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( cMFieldNeighbours[0 + 2*uiNeighbourIdx].getMv(), cMFieldNeighbours[0 + 2*uiNeighbourIdx].getRefIdx(), ePartSize, uiAbsPartIdx, iPartIdx, 0 );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( cMFieldNeighbours[1 + 2*uiNeighbourIdx].getMv(), cMFieldNeighbours[1 + 2*uiNeighbourIdx].getRefIdx(), ePartSize, uiAbsPartIdx, iPartIdx, 0 );
#endif

#ifdef DCM_PBIC
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      RefPicList eRefList = (uhInterDirNeighbours[uiNeighbourIdx] == 3) ? REF_PIC_LIST_X : ( (uhInterDirNeighbours[uiNeighbourIdx] == 2) ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
      cIcNeighbours[uiNeighbourIdx].computeScaleOffset( eRefList );
      pcCU->getCUIcField()->setAllIcField( cIcNeighbours[uiNeighbourIdx], ePartSize, uiAbsPartIdx, iPartIdx, 0 );
    }
#endif

    motionCompensation( pcCU, &cYuvPred, REF_PIC_LIST_X, iPartIdx );

    // both blocks have the size of the CU but we only cycle over the current PU.
    Pel* pOrgY = pcYuvOrg->getLumaAddr( uiAbsPartIdx );
    Pel* pPredY = cYuvPred.getLumaAddr( uiAbsPartIdx );
    UInt uiSAD = 0;
    for ( UInt uiY = 0; uiY < iHeight; uiY++ )
    {
      for ( UInt uiX = 0; uiX < iWidth; uiX++ )
      {
        uiSAD += abs( pOrgY[uiX] - pPredY[uiX] );
      }
      pOrgY += pcYuvOrg->getStride();
      pPredY += cYuvPred.getStride();
    }

    uiSAD >>= g_uiBitIncrement;

    // Merge signalization
    if ( uiNeighbourInfos == 3 )
    {
      uiBitsTemp = 2;
    }
    else
    {
      uiBitsTemp = 1;
    }

    m_pcRdCost->getMotionCost( 0, 0 ); // choose lambda;

    uiCostTemp = (UInt)( floor(  (Double)uiSAD + (Double)m_pcRdCost->getCost( uiBitsTemp ) ) );


    if ( iNumPredDir == 1 ) // P slice
    {
      if ( uiCostTemp < ruiCost )
      {
        ruiCost = uiCostTemp;
        pcMvTemp[0]    = cMFieldNeighbours[0 + 2*uiNeighbourIdx].getMv();
#ifdef DCM_PBIC
        rcIcTemp       = cIcNeighbours[uiNeighbourIdx];
#endif
        piRefIdxPred[0] = cMFieldNeighbours[0 + 2*uiNeighbourIdx].getRefIdx();
        uiInterDir = 1;
        uiMergeIndex = uiNeighbourIdx;
      }
    }
    else // B slice
    {
      // Ref List 0
      if ( uhInterDirNeighbours[uiNeighbourIdx] == 1 )
      {
        if ( uiCostTemp < ruiCost )
        {
          ruiCost = uiCostTemp;
          pcMvTemp[0]    = cMFieldNeighbours[0 + 2*uiNeighbourIdx].getMv();
#ifdef DCM_PBIC
          rcIcTemp       = cIcNeighbours[uiNeighbourIdx];
#endif
          piRefIdxPred[0] = cMFieldNeighbours[0 + 2*uiNeighbourIdx].getRefIdx();
          uiInterDir = 1;
          uiMergeIndex = uiNeighbourIdx;
        }
      }
      // Ref List 1
      else if ( uhInterDirNeighbours[uiNeighbourIdx] == 2 )
      {
        if ( uiCostTemp < ruiCost )
        {
          ruiCost = uiCostTemp;
          pcMvTemp[1]    = cMFieldNeighbours[1 + 2*uiNeighbourIdx].getMv();
#ifdef DCM_PBIC
          rcIcTemp       = cIcNeighbours[uiNeighbourIdx];
#endif
          piRefIdxPred[1] = cMFieldNeighbours[1 + 2*uiNeighbourIdx].getRefIdx();
          uiInterDir = 2;
          uiMergeIndex = uiNeighbourIdx;
        }
      } 
      // Both of the List 0&1
      else 
      {
        if ( uiCostTemp < ruiCost )
        {
          ruiCost = uiCostTemp;
          pcMvTemp[0]    = cMFieldNeighbours[0 + 2*uiNeighbourIdx].getMv();
          pcMvTemp[1]    = cMFieldNeighbours[1 + 2*uiNeighbourIdx].getMv();
#ifdef DCM_PBIC
          rcIcTemp       = cIcNeighbours[uiNeighbourIdx];
#endif
          piRefIdxPred[0] = cMFieldNeighbours[0 + 2*uiNeighbourIdx].getRefIdx();
          piRefIdxPred[1] = cMFieldNeighbours[1 + 2*uiNeighbourIdx].getRefIdx();
          uiInterDir = 3;
          uiMergeIndex = uiNeighbourIdx;
        }
      }
    }
    if ( uiNeighbourInfos != 3 )
    {
      // there is only one merge candidate
      // only one candidate has to be tested.
      break;
    }
    else
    {
      uiNeighbourIdx++;
    }
  }

  cYuvPred.destroy();
}
#endif

Void TEncSearch::predInterSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv, Bool bUseRes )
{
  m_acYuvPred[0].clear();
  m_acYuvPred[1].clear();
  m_cYuvPredTemp.clear();
  rpcPredYuv->clear();
  
  if ( !bUseRes )
  {
    rpcResiYuv->clear();
  }
  
  rpcRecoYuv->clear();
  
  TComMv        cMvSrchRngLT;
  TComMv        cMvSrchRngRB;
  
  TComMv        cMvZero;
  TComMv        TempMv; //kolya
  
  TComMv        cMv[2];
  TComMv        cMvBi[2];
  TComMv        cMvTemp[2][33];
  
  Int           iNumPart    = pcCU->getNumPartInter();
  Int           iNumPredDir = pcCU->getSlice()->isInterP() ? 1 : 2;
  
  TComMv        cMvPred[2][33];
  
  TComMv        cMvPredBi[2][33];
  Int           aaiMvpIdxBi[2][33];
#if HHI_IMVP
  TComMv        cMvPredBiTemp[2][33];
  MvPredMeasure cMvPredMeasure[2][33];
#endif
  
  Int           aaiMvpIdx[2][33];
  Int           aaiMvpNum[2][33];
  
  AMVPInfo aacAMVPInfo[2][33];
  
  Int           iRefIdx[2];
  Int           iRefIdxBi[2];
  
  UInt          uiPartAddr;
  Int           iRoiWidth, iRoiHeight;
  
  UInt          uiMbBits[3] = {1, 1, 0};
  
  UInt          uiLastMode = 0;
  Int           iRefStart, iRefEnd;
  
  PartSize      ePartSize = pcCU->getPartitionSize( 0 );
  
#if HHI_MRG_PU
  // Merge tools
  UInt uiMergeInterDir = 0;
  Int iMergeRefIdx[2];
  TComMv cMergeMv[2];
#ifdef DCM_PBIC
  TComIc cMergeIc;
#endif
  Int iRefBestList= 0;
  UInt uiMergeCost = MAX_UINT;
  UInt uiMergeIndex = 0;
#endif

  for ( Int iPartIdx = 0; iPartIdx < iNumPart; iPartIdx++ )
  {
    UInt          uiCost[2] = { MAX_UINT, MAX_UINT };
    UInt          uiCostBi  =   MAX_UINT;
    UInt          uiCostTemp;
    
    UInt          uiBits[3];
    UInt          uiBitsTemp;
    
    xGetBlkBits( ePartSize, pcCU->getSlice()->isInterP(), iPartIdx, uiLastMode, uiMbBits);
    
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );
    
    //  Uni-directional prediction
    for ( Int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
    {
      RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
      
      for ( Int iRefIdxTemp = 0; iRefIdxTemp < pcCU->getSlice()->getNumRefIdx(eRefPicList); iRefIdxTemp++ )
      {
#ifdef QC_SIFO
        setCurrList(iRefList);
        setCurrRefFrame(iRefIdxTemp);
#endif
        uiBitsTemp = uiMbBits[iRefList];
        if ( pcCU->getSlice()->getNumRefIdx(eRefPicList) > 1 )
        {
          uiBitsTemp += iRefIdxTemp+1;
          if ( iRefIdxTemp == pcCU->getSlice()->getNumRefIdx(eRefPicList)-1 ) uiBitsTemp--;
        }
        xEstimateMvPredAMVP( pcCU, pcOrgYuv, iPartIdx, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp]);
#if HHI_IMVP
        if ( pcCU->getSlice()->getSPS()->getUseIMP() )
        {
          xEstimateMvPredIMVP( pcCU, iPartIdx, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], cMvPredMeasure[iRefList][iRefIdxTemp] );
          m_cMvPredMeasure = cMvPredMeasure[iRefList][iRefIdxTemp];
        }
#endif
        aaiMvpIdx[iRefList][iRefIdxTemp] = pcCU->getMVPIdx(eRefPicList, uiPartAddr);
        aaiMvpNum[iRefList][iRefIdxTemp] = pcCU->getMVPNum(eRefPicList, uiPartAddr);
        
        uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][aaiMvpNum[iRefList][iRefIdxTemp]];
        
#if GPB_SIMPLE_UNI
        if ( pcCU->getSlice()->getSPS()->getUseLDC() )
        {
          if ( iRefList && iRefIdxTemp != iRefIdx[0] )
          {
            uiCostTemp = MAX_UINT;
#ifdef QC_AMVRES
#if HHI_IMVP
            //if ( pcCU->getSlice()->getSPS()->getUseIMP() && pcCU->getSlice()->getSPS()->getUseAMVRes())
            if ( pcCU->getSlice()->getSPS()->getUseIMP())
              cMvPred[iRefList][iRefIdxTemp] = m_cMvPredMeasure.getMVPred( cMvTemp[iRefList][iRefIdxTemp].getHor(), cMvTemp[iRefList][iRefIdxTemp].getVer() );
#endif
#endif
          }
          else
          {
            xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
          }
        }
        else
        {
          xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
        }
#else
        xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
#endif
        xCopyAMVPInfo(pcCU->getCUMvField(eRefPicList)->getAMVPInfo(), &aacAMVPInfo[iRefList][iRefIdxTemp]); // must always be done ( also when AMVP_MODE = AM_NONE )
        if ( pcCU->getAMVPMode(uiPartAddr) == AM_EXPL )
        {          
#ifdef QC_AMVRES
          if( !cMvTemp[iRefList][iRefIdxTemp].isHAM()&& pcCU->getSlice()->getSPS()->getUseAMVRes())
            xCheckBestMVP_onefourth(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);
          else
#endif
            xCheckBestMVP(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);
#if HHI_IMVP
          if ( pcCU->getSlice()->getSPS()->getUseIMP() )
          {
            cMvPredMeasure[iRefList][iRefIdxTemp] = m_cMvPredMeasure;
          }
#endif
        }
        if ( uiCostTemp < uiCost[iRefList] )
        {
          uiCost[iRefList] = uiCostTemp;
          uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction
          
          // set motion
          cMv[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
          iRefIdx[iRefList] = iRefIdxTemp;
#ifdef QC_AMVRES
          if (pcCU->getSlice()->getSPS()->getUseAMVRes())
            pcCU->getCUMvField(eRefPicList)->setAllMvField_AMVRes( cMv[iRefList], iRefIdx[iRefList], ePartSize, uiPartAddr, iPartIdx, 0 );
          else
#endif
            pcCU->getCUMvField(eRefPicList)->setAllMvField( cMv[iRefList], iRefIdx[iRefList], ePartSize, uiPartAddr, iPartIdx, 0 );
          
          // storing list 1 prediction signal for iterative bi-directional prediction
          if ( eRefPicList == REF_PIC_LIST_1 )
          {
            TComYuv*  pcYuvPred = &m_acYuvPred[iRefList];
            motionCompensation ( pcCU, pcYuvPred, eRefPicList, iPartIdx );
          }
        }
      }
    }
    
    //  Bi-directional prediction
    if ( pcCU->getSlice()->isInterB() )
    {
      cMvBi[0] = cMv[0];            cMvBi[1] = cMv[1];
      iRefIdxBi[0] = iRefIdx[0];    iRefIdxBi[1] = iRefIdx[1];
      
      ::memcpy(cMvPredBi, cMvPred, sizeof(cMvPred));
#if HHI_IMVP
      ::memcpy(cMvPredBiTemp, cMvPred, sizeof(cMvPred));
#endif
      ::memcpy(aaiMvpIdxBi, aaiMvpIdx, sizeof(aaiMvpIdx));
      
      UInt uiMotBits[2] = { uiBits[0] - uiMbBits[0], uiBits[1] - uiMbBits[1] };
      uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
      
      // 4-times iteration (default)
      Int iNumIter = 4;
      
      // fast encoder setting: only one iteration
      if ( m_pcEncCfg->getUseFastEnc() )
      {
        iNumIter = 1;
      }
      
      for ( Int iIter = 0; iIter < iNumIter; iIter++ )
      {
        Int         iRefList    = iIter % 2;
        RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
        
        Bool bChanged = false;
        
#if GPB_SIMPLE
        if ( pcCU->getSlice()->getSPS()->getUseLDC() && iRefList )
        {
          iRefStart = iRefIdxBi[1-iRefList];
          iRefEnd   = iRefIdxBi[1-iRefList];
        }
        else
        {
          iRefStart = 0;
          iRefEnd   = pcCU->getSlice()->getNumRefIdx(eRefPicList)-1;
        }
#else
        iRefStart = 0;
        iRefEnd   = pcCU->getSlice()->getNumRefIdx(eRefPicList)-1;
#endif
        
        for ( Int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++ )
        {
          uiBitsTemp = uiMbBits[2] + uiMotBits[1-iRefList];
          if ( pcCU->getSlice()->getNumRefIdx(eRefPicList) > 1 )
          {
            uiBitsTemp += iRefIdxTemp+1;
            if ( iRefIdxTemp == pcCU->getSlice()->getNumRefIdx(eRefPicList)-1 ) uiBitsTemp--;
          }
          
          uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][aaiMvpNum[iRefList][iRefIdxTemp]];
#ifdef QC_SIFO
          setCurrList(iRefList);
          setCurrRefFrame(iRefIdxTemp);
#endif
          // call ME
#if HHI_IMVP
          if ( pcCU->getSlice()->getSPS()->getUseIMP() )
          {
            m_cMvPredMeasure = cMvPredMeasure[iRefList][iRefIdxTemp];
          }
          xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPredBiTemp[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, true );
#else
          xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, true );
#endif
          if ( pcCU->getAMVPMode(uiPartAddr) == AM_EXPL )
          {
            xCopyAMVPInfo(&aacAMVPInfo[iRefList][iRefIdxTemp], pcCU->getCUMvField(eRefPicList)->getAMVPInfo());
#if HHI_IMVP
#ifdef QC_AMVRES
            if( !cMvTemp[iRefList][iRefIdxTemp].isHAM()&& pcCU->getSlice()->getSPS()->getUseAMVRes())
              xCheckBestMVP_onefourth(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBiTemp[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);
            else
#endif 
              xCheckBestMVP(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBiTemp[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);
            if ( pcCU->getSlice()->getSPS()->getUseIMP() )
            {
              cMvPredMeasure[iRefList][iRefIdxTemp] = m_cMvPredMeasure;
              cMvPredBi[iRefList][iRefIdxTemp].setVer( cMvPredBiTemp[iRefList][iRefIdxTemp].getVer());
            }
            else
            {
              cMvPredBi[iRefList][iRefIdxTemp] =  cMvPredBiTemp[iRefList][iRefIdxTemp];
            }
#else
#ifdef QC_AMVRES
            if( !cMvTemp[iRefList][iRefIdxTemp].isHAM()&& pcCU->getSlice()->getSPS()->getUseAMVRes())
              xCheckBestMVP_onefourth(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);
            else
#endif  
              xCheckBestMVP(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);
#endif
          }
          
          if ( uiCostTemp < uiCostBi )
          {
            bChanged = true;
            
            cMvBi[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
#if HHI_IMVP
            if ( pcCU->getSlice()->getSPS()->getUseIMP() )
            {
              cMvPredBi[iRefList][iRefIdxTemp].setHor( cMvPredBiTemp[iRefList][iRefIdxTemp].getHor() ); 
            }
#endif          
            iRefIdxBi[iRefList] = iRefIdxTemp;
            
            uiCostBi            = uiCostTemp;
            uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1-iRefList];
            uiBits[2]           = uiBitsTemp;
            
            //  Set motion
#ifdef QC_AMVRES
            if (pcCU->getSlice()->getSPS()->getUseAMVRes())
              pcCU->getCUMvField(eRefPicList)->setAllMvField_AMVRes( cMvBi[iRefList], iRefIdxBi[iRefList], ePartSize, uiPartAddr, iPartIdx, 0 );
            else
#endif
              pcCU->getCUMvField( eRefPicList )->setAllMvField( cMvBi[iRefList], iRefIdxBi[iRefList], ePartSize, uiPartAddr, iPartIdx, 0 );
            
            TComYuv* pcYuvPred = &m_acYuvPred[iRefList];
            motionCompensation( pcCU, pcYuvPred, eRefPicList, iPartIdx );
          }
        } // for loop-iRefIdxTemp
        
        if ( !bChanged )
        {
          if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] && pcCU->getAMVPMode(uiPartAddr) == AM_EXPL )
          {
#if HHI_IMVP
            if ( pcCU->getSlice()->getSPS()->getUseIMP() )
            {
              m_cMvPredMeasure = cMvPredMeasure[0][iRefIdxBi[0]];
            }
#endif
            xCopyAMVPInfo(&aacAMVPInfo[0][iRefIdxBi[0]], pcCU->getCUMvField(REF_PIC_LIST_0)->getAMVPInfo());
#ifdef QC_AMVRES
            if( !cMvBi[0].isHAM()&& pcCU->getSlice()->getSPS()->getUseAMVRes())
              xCheckBestMVP_onefourth(pcCU, REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], uiBits[2], uiCostBi);
            else
#endif             
              xCheckBestMVP(pcCU, REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], uiBits[2], uiCostBi);
#if HHI_IMVP
            if ( pcCU->getSlice()->getSPS()->getUseIMP() )
            {
              cMvPredMeasure[0][iRefIdxBi[0]] = m_cMvPredMeasure;
              m_cMvPredMeasure = cMvPredMeasure[1][iRefIdxBi[1]];
            }  
#endif
            xCopyAMVPInfo(&aacAMVPInfo[1][iRefIdxBi[1]], pcCU->getCUMvField(REF_PIC_LIST_1)->getAMVPInfo());
#ifdef QC_AMVRES
            if( !cMvBi[1].isHAM()&& pcCU->getSlice()->getSPS()->getUseAMVRes())
              xCheckBestMVP_onefourth(pcCU, REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], uiBits[2], uiCostBi);
            else
#endif             
              xCheckBestMVP(pcCU, REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], uiBits[2], uiCostBi);
#if HHI_IMVP
            if ( pcCU->getSlice()->getSPS()->getUseIMP() )
            {
              cMvPredMeasure[1][iRefIdxBi[1]] = m_cMvPredMeasure;
            }
#endif
          }
          break;
        }
      } // for loop-iter
    } // if (B_SLICE)
    
#if HHI_MRG_PU
    pcCU->setMergeFlagSubParts( false, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));  
    pcCU->setMergeIndexSubParts( 0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

    UInt uiBestCost = MAX_INT;
    UInt uiMergeCost = MAX_INT;
    if ( pcCU->getSlice()->getSPS()->getUseMRG() )
    {
#ifdef DCM_PBIC
      xMergeEstimation( pcCU, pcOrgYuv, iPartIdx, iMergeRefIdx, cMergeMv , cMergeIc, uiMergeInterDir, uiMergeIndex, uiMergeCost );
#else
      xMergeEstimation( pcCU, pcOrgYuv, iPartIdx, iMergeRefIdx, cMergeMv , uiMergeInterDir, uiMergeIndex, uiMergeCost );
#endif

      if( pcCU->getSlice()->isInterP() )
      {
        if ( uiMergeCost < uiCost[0] )
        {
          uiCost[ 0 ] = uiMergeCost;
          // set motion
          cMv[0] = cMergeMv[0];
          cMvPred[0][iMergeRefIdx[0]] = cMergeMv[0];
          iRefIdx[0] = iMergeRefIdx[0];
          
          aaiMvpIdx[0][iRefIdx[0]] = 0;
          aaiMvpNum[0][iRefIdx[0]] = 0;
          
          pcCU->setMergeFlagSubParts( true, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
          pcCU->setMergeIndexSubParts( uiMergeIndex, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));   
        }
      }
      else // Slice B
      {
        if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] )
        {
          uiBestCost = uiCostBi;
        }
        else if ( uiCost[0] <= uiCost[1] )
        {
          uiBestCost = uiCost[0];
        }
        else
        {
          uiBestCost = uiCost[1];
        }
        if ( uiMergeCost < uiBestCost )
        {
          if ( uiMergeInterDir == 3 )
          {
            uiCostBi = uiMergeCost;
            // set motion
            cMvBi[0]      = cMergeMv[0];
            cMvPredBi[0][iMergeRefIdx[0]] = cMergeMv[0];
            cMvBi[1]      = cMergeMv[1];
            cMvPredBi[1][iMergeRefIdx[1]]  = cMergeMv[1];
            iRefIdxBi[0] = iMergeRefIdx[0];
            iRefIdxBi[1] = iMergeRefIdx[1];

            aaiMvpIdxBi[0][iRefIdxBi[0]] = 0;
            aaiMvpNum[0][iRefIdxBi[0]] = 0; 
            aaiMvpIdxBi[1][iRefIdxBi[1]] = 0;
            aaiMvpNum[1][iRefIdxBi[1]] = 0; 

            pcCU->setMergeFlagSubParts( true, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
            pcCU->setMergeIndexSubParts( uiMergeIndex, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));   
          } 
          else
          {
            uiCost[ uiMergeInterDir-1 ] = uiMergeCost;
            // set motion
            cMv[uiMergeInterDir-1] = cMergeMv[uiMergeInterDir-1];
            cMvPred[uiMergeInterDir-1][iMergeRefIdx[uiMergeInterDir-1]] = cMergeMv[uiMergeInterDir-1];
            iRefIdx[uiMergeInterDir-1] = iMergeRefIdx[ uiMergeInterDir-1 ];
            
            aaiMvpIdx[uiMergeInterDir-1][iRefIdx[uiMergeInterDir-1]] = 0;
            aaiMvpNum[uiMergeInterDir-1][iRefIdx[uiMergeInterDir-1]] = 0;

            pcCU->setMergeFlagSubParts( true, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
            pcCU->setMergeIndexSubParts( uiMergeIndex, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));   
          }
        }
      }
    }
#endif

    //  Clear Motion Field
#ifdef QC_AMVRES
    if (pcCU->getSlice()->getSPS()->getUseAMVRes())
    {
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField_AMVRes( cMvZero, NOT_VALID, ePartSize, uiPartAddr, iPartIdx, 0 );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField_AMVRes( cMvZero, NOT_VALID, ePartSize, uiPartAddr, iPartIdx, 0 );
    }
    else
#endif
    {
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( cMvZero, NOT_VALID, ePartSize, uiPartAddr, iPartIdx, 0 );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( cMvZero, NOT_VALID, ePartSize, uiPartAddr, iPartIdx, 0 );
    }
    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( cMvZero,            ePartSize, uiPartAddr, iPartIdx, 0 );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( cMvZero,            ePartSize, uiPartAddr, iPartIdx, 0 );
    
    pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    
    // Set Motion Field_
    if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1])
    {
      uiLastMode = 2;
#ifdef QC_AMVRES
      if (pcCU->getSlice()->getSPS()->getUseAMVRes())
      {
        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField_AMVRes( cMvBi[0], iRefIdxBi[0], ePartSize, uiPartAddr, iPartIdx, 0 );
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField_AMVRes( cMvBi[1], iRefIdxBi[1], ePartSize, uiPartAddr, iPartIdx, 0 );
      }
      else
#endif
      {
        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( cMvBi[0], iRefIdxBi[0], ePartSize, uiPartAddr, iPartIdx, 0 );
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( cMvBi[1], iRefIdxBi[1], ePartSize, uiPartAddr, iPartIdx, 0 );
      }
#ifdef QC_AMVRES
      if( !cMvBi[0].isHAM()&& pcCU->getSlice()->getSPS()->getUseAMVRes() )
      {
        TempMv.set((cMvBi[0].getHor()/2-cMvPredBi[0][iRefIdxBi[0]].getHor()/2)*2,(cMvBi[0].getVer()/2-cMvPredBi[0][iRefIdxBi[0]].getVer()/2)*2 ) ;
        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, iPartIdx, 0 );
#if HHI_IMVP 
        if (pcCU->clearMVPCand_one_fourth(TempMv, &aacAMVPInfo[0][iRefIdxBi[0]], REF_PIC_LIST_0, uiPartAddr, iRefIdxBi[0] )) 
#else			
          if (pcCU->clearMVPCand_one_fourth(TempMv, &aacAMVPInfo[0][iRefIdxBi[0]]))
#endif
          {
            aaiMvpIdxBi[0][iRefIdxBi[0]] = pcCU->searchMVPIdx_one_fourth(cMvPredBi[0][iRefIdxBi[0]], &aacAMVPInfo[0][iRefIdxBi[0]]);
            aaiMvpNum[0][iRefIdxBi[0]] = aacAMVPInfo[0][iRefIdxBi[0]].iN;
          }
      }
      else
#endif
      {
        TempMv = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, iPartIdx, 0 );
#if HHI_IMVP
        if ( pcCU->clearMVPCand(TempMv, &aacAMVPInfo[0][iRefIdxBi[0]], REF_PIC_LIST_0, uiPartAddr, iRefIdxBi[0] )) 
#else
          if (pcCU->clearMVPCand(TempMv, &aacAMVPInfo[0][iRefIdxBi[0]]))
#endif
          {
            aaiMvpIdxBi[0][iRefIdxBi[0]] = pcCU->searchMVPIdx(cMvPredBi[0][iRefIdxBi[0]], &aacAMVPInfo[0][iRefIdxBi[0]]);
            aaiMvpNum[0][iRefIdxBi[0]] = aacAMVPInfo[0][iRefIdxBi[0]].iN;
          }
      }
#ifdef QC_AMVRES
      if( !cMvBi[1].isHAM()&& pcCU->getSlice()->getSPS()->getUseAMVRes())
      {
        TempMv.set((cMvBi[1].getHor()/2-cMvPredBi[1][iRefIdxBi[1]].getHor()/2)*2,(cMvBi[1].getVer()/2-cMvPredBi[1][iRefIdxBi[1]].getVer()/2) *2 ) ;
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, iPartIdx, 0 );
#if HHI_IMVP
        if (pcCU->clearMVPCand_one_fourth(TempMv, &aacAMVPInfo[1][iRefIdxBi[1]], REF_PIC_LIST_1, uiPartAddr, iRefIdxBi[1] ) )
#else			
          if (pcCU->clearMVPCand_one_fourth(TempMv, &aacAMVPInfo[1][iRefIdxBi[1]]))
#endif
          {
            aaiMvpIdxBi[1][iRefIdxBi[1]] = pcCU->searchMVPIdx_one_fourth(cMvPredBi[1][iRefIdxBi[1]], &aacAMVPInfo[1][iRefIdxBi[1]]);
            aaiMvpNum[1][iRefIdxBi[1]] = aacAMVPInfo[1][iRefIdxBi[1]].iN;
          }
      }
      else
#endif
      {
        TempMv = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, iPartIdx, 0 );
#if HHI_IMVP
        if ( pcCU->clearMVPCand(TempMv, &aacAMVPInfo[1][iRefIdxBi[1]], REF_PIC_LIST_1, uiPartAddr, iRefIdxBi[1] ) )
#else
          if (pcCU->clearMVPCand(TempMv, &aacAMVPInfo[1][iRefIdxBi[1]]))
#endif
          {
            aaiMvpIdxBi[1][iRefIdxBi[1]] = pcCU->searchMVPIdx(cMvPredBi[1][iRefIdxBi[1]], &aacAMVPInfo[1][iRefIdxBi[1]]);
            aaiMvpNum[1][iRefIdxBi[1]] = aacAMVPInfo[1][iRefIdxBi[1]].iN;
          }
      }
      
      pcCU->setInterDirSubParts( 3, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
      
      pcCU->setMVPIdxSubParts( aaiMvpIdxBi[0][iRefIdxBi[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[0][iRefIdxBi[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPIdxSubParts( aaiMvpIdxBi[1][iRefIdxBi[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[1][iRefIdxBi[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    }
    else if ( uiCost[0] <= uiCost[1] )
    {
      uiLastMode = 0;
#ifdef QC_AMVRES 
      if (pcCU->getSlice()->getSPS()->getUseAMVRes())
      {
        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField_AMVRes( cMv[0], iRefIdx[0], ePartSize, uiPartAddr, iPartIdx, 0 );
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMVRes      ( false , ePartSize, uiPartAddr, iPartIdx, 0 );
      }
      else
#endif
        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( cMv[0],   iRefIdx[0],   ePartSize, uiPartAddr, iPartIdx, 0 );
      
#ifdef QC_AMVRES
      if( !cMv[0].isHAM()&& pcCU->getSlice()->getSPS()->getUseAMVRes())
      {
        TempMv.set(((cMv[0].getHor()/2)-(cMvPred[0][iRefIdx[0]].getHor()/2))*2,((cMv[0].getVer()/2)-(cMvPred[0][iRefIdx[0]].getVer()/2))*2 ) ;
        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, iPartIdx, 0 );
#if HHI_IMVP
        if (pcCU->clearMVPCand_one_fourth(TempMv, &aacAMVPInfo[0][iRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iRefIdx[0] ) )
#else		  
          if (pcCU->clearMVPCand_one_fourth(TempMv, &aacAMVPInfo[0][iRefIdx[0]]))
#endif
          {
            aaiMvpIdx[0][iRefIdx[0]] = pcCU->searchMVPIdx_one_fourth(cMvPred[0][iRefIdx[0]], &aacAMVPInfo[0][iRefIdx[0]]);
            aaiMvpNum[0][iRefIdx[0]] = aacAMVPInfo[0][iRefIdx[0]].iN;
          }
      }
      else
#endif
      {
        TempMv = cMv[0] - cMvPred[0][iRefIdx[0]];
        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, iPartIdx, 0 );
#if HHI_IMVP
        if ( pcCU->clearMVPCand(TempMv, &aacAMVPInfo[0][iRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iRefIdx[0] ) )
#else
          if (pcCU->clearMVPCand(TempMv, &aacAMVPInfo[0][iRefIdx[0]]))
#endif
          {
            aaiMvpIdx[0][iRefIdx[0]] = pcCU->searchMVPIdx(cMvPred[0][iRefIdx[0]], &aacAMVPInfo[0][iRefIdx[0]]);
            aaiMvpNum[0][iRefIdx[0]] = aacAMVPInfo[0][iRefIdx[0]].iN;
          }
      }
      pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
      
      pcCU->setMVPIdxSubParts( aaiMvpIdx[0][iRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[0][iRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    }
    else
    {
      uiLastMode = 1;
#ifdef QC_AMVRES
      if (pcCU->getSlice()->getSPS()->getUseAMVRes())
      {
        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMVRes      ( false , ePartSize, uiPartAddr, iPartIdx, 0 );
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField_AMVRes( cMv[1], iRefIdx[1], ePartSize, uiPartAddr, iPartIdx, 0 );
      }
      else
#endif
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( cMv[1],   iRefIdx[1],   ePartSize, uiPartAddr, iPartIdx, 0 );
      
#ifdef QC_AMVRES
      if( !cMv[1].isHAM() && pcCU->getSlice()->getSPS()->getUseAMVRes())
      {
        TempMv.set(((cMv[1].getHor()/2)-(cMvPred[1][iRefIdx[1]].getHor()/2))*2,((cMv[1].getVer()/2)-(cMvPred[1][iRefIdx[1]].getVer()/2))*2 ) ;
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, iPartIdx, 0 );
#if HHI_IMVP
        if (pcCU->clearMVPCand_one_fourth(TempMv, &aacAMVPInfo[1][iRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iRefIdx[1] ) )
#else			
          if (pcCU->clearMVPCand_one_fourth(TempMv, &aacAMVPInfo[1][iRefIdx[1]]))
#endif
          {
            aaiMvpIdx[1][iRefIdx[1]] = pcCU->searchMVPIdx_one_fourth(cMvPred[1][iRefIdx[1]], &aacAMVPInfo[1][iRefIdx[1]]);
            aaiMvpNum[1][iRefIdx[1]] = aacAMVPInfo[1][iRefIdx[1]].iN;
          }
      }
      else
#endif
      {
        TempMv = cMv[1] - cMvPred[1][iRefIdx[1]];
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, iPartIdx, 0 );
#if HHI_IMVP
        if ( pcCU->clearMVPCand(TempMv, &aacAMVPInfo[1][iRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iRefIdx[1] ) )
#else
          if (pcCU->clearMVPCand(TempMv, &aacAMVPInfo[1][iRefIdx[1]]))
#endif
          {
            aaiMvpIdx[1][iRefIdx[1]] = pcCU->searchMVPIdx(cMvPred[1][iRefIdx[1]], &aacAMVPInfo[1][iRefIdx[1]]);
            aaiMvpNum[1][iRefIdx[1]] = aacAMVPInfo[1][iRefIdx[1]].iN;
          }
      }
      pcCU->setInterDirSubParts( 2, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
      
      pcCU->setMVPIdxSubParts( aaiMvpIdx[1][iRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[1][iRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    }
    
#if HHI_MRG_PU && defined(DCM_PBIC)
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      if (pcCU->getMergeFlag(uiPartAddr) == false)
        cMergeIc.reset();
      pcCU->getCUIcField()->setAllIc( cMergeIc, ePartSize, uiPartAddr, iPartIdx, 0 );
    }
#endif

    //  MC
    motionCompensation ( pcCU, rpcPredYuv, REF_PIC_LIST_X, iPartIdx );

#ifdef DCM_PBIC
#if HHI_MRG_PU
    if (pcCU->getSlice()->getSPS()->getUseIC() && (pcCU->getMergeFlag(uiPartAddr) == false))
#else
    if (pcCU->getSlice()->getSPS()->getUseIC())
#endif
    {
      predICompSearch(pcCU, pcOrgYuv, rpcPredYuv, iPartIdx, uiLastMode);
    }
#endif

  } //  end of for ( Int iPartIdx = 0; iPartIdx < iNumPart; iPartIdx++ )
  
  return;
}

#if HHI_IMVP
Void TEncSearch::xEstimateMvPredIMVP( TComDataCU* pcCU, UInt uiPartIdx, RefPicList eRefPicList, Int iRefIdx, TComMv& rcMvPred, MvPredMeasure&  rcMvPredMeasure )
{  
  Int iMvHor = 0;
  
  pcCU->getYThresXPredLists( uiPartIdx, eRefPicList, iRefIdx, rcMvPredMeasure.m_iYOrigin, rcMvPredMeasure.m_aiYThreshold, rcMvPredMeasure.m_aiXOrigin ) ;   
  rcMvPredMeasure.m_iYOrigin = rcMvPred.getVer();
  pcCU->getMvPredXDep( eRefPicList, uiPartIdx, iRefIdx, rcMvPredMeasure.m_iYOrigin, iMvHor );
  rcMvPred.setHor( iMvHor );
}
#endif

// AMVP
Void TEncSearch::xEstimateMvPredAMVP( TComDataCU* pcCU, TComYuv* pcOrgYuv, UInt uiPartIdx, RefPicList eRefPicList, Int iRefIdx, TComMv& rcMvPred, Bool bFilled )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();
  
  TComMv  cBestMv;
  Int     iBestIdx = 0;
  TComMv  cZeroMv;
  TComMv  cMvPred;
  UInt    uiBestCost = MAX_INT;
  UInt    uiPartAddr = 0;
  Int     iRoiWidth, iRoiHeight;
  Int     i;
  
  pcCU->getPartIndexAndSize( uiPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );
  // Fill the MV Candidates
  if (!bFilled)
  {
    pcCU->fillMvpCand( uiPartIdx, uiPartAddr, eRefPicList, iRefIdx, pcAMVPInfo );
  }
  
  // initialize Mvp index & Mvp
  iBestIdx = 0;
  cBestMv  = pcAMVPInfo->m_acMvCand[0];
  
  if( pcCU->getAMVPMode(uiPartAddr) == AM_NONE || (pcAMVPInfo->iN <= 1 && pcCU->getAMVPMode(uiPartAddr) == AM_EXPL) )
  {
    rcMvPred = cBestMv;
    
    pcCU->setMVPIdxSubParts( iBestIdx, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    return;
  }
  
  if (pcCU->getAMVPMode(uiPartAddr) == AM_EXPL && bFilled)
  {
    assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
    rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
    return;
  }
  
  if (pcCU->getAMVPMode(uiPartAddr) == AM_EXPL)
  {
    m_cYuvPredTemp.clear();
    
    //-- Check Minimum Cost.
    for ( i = 0 ; i < pcAMVPInfo->iN; i++)
    {
      UInt uiTmpCost;
      
      uiTmpCost = xGetTemplateCost( pcCU, uiPartIdx, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, pcAMVPInfo->m_acMvCand[i], i, pcAMVPInfo->iN, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight);
      
      if ( uiBestCost > uiTmpCost )
      {
        uiBestCost = uiTmpCost;
        cBestMv   = pcAMVPInfo->m_acMvCand[i];
        iBestIdx  = i;
      }
    }
    
    m_cYuvPredTemp.clear();
  }
  
  // Setting Best MVP
  rcMvPred = cBestMv;
  pcCU->setMVPIdxSubParts( iBestIdx, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
  pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
  return;
}

UInt TEncSearch::xGetMvpIdxBits(Int iIdx, Int iNum)
{
  assert(iIdx >= 0 && iNum >= 0 && iIdx < iNum);
  
  if (iNum == 1)
    return 0;
  
  UInt uiLength = 1;
  Int iTemp = iIdx;
  if ( iTemp == 0 )
  {
    return uiLength;
  }
  
  Bool bCodeLast = ( iNum-1 > iTemp );
  
  uiLength += (iTemp-1);
  
  if( bCodeLast )
  {
    uiLength++;
  }
  
  return uiLength;
}

Void TEncSearch::xGetBlkBits( PartSize eCUMode, Bool bPSlice, Int iPartIdx, UInt uiLastMode, UInt uiBlkBit[3])
{
  if ( eCUMode == SIZE_2Nx2N )
  {
    uiBlkBit[0] = (! bPSlice) ? 3 : 1;
    uiBlkBit[1] = 3;
    uiBlkBit[2] = 5;
  }
  else if ( (eCUMode == SIZE_2NxN || eCUMode == SIZE_2NxnU) || eCUMode == SIZE_2NxnD )
  {
    UInt aauiMbBits[2][3][3] = { { {0,0,3}, {0,0,0}, {0,0,0} } , { {5,7,7}, {7,5,7}, {9-3,9-3,9-3} } };
    if ( bPSlice )
    {
      uiBlkBit[0] = 3;
      uiBlkBit[1] = 0;
      uiBlkBit[2] = 0;
    }
    else
    {
      ::memcpy( uiBlkBit, aauiMbBits[iPartIdx][uiLastMode], 3*sizeof(UInt) );
    }
  }
  else if ( (eCUMode == SIZE_Nx2N || eCUMode == SIZE_nLx2N) || eCUMode == SIZE_nRx2N )
  {
    UInt aauiMbBits[2][3][3] = { { {0,2,3}, {0,0,0}, {0,0,0} } , { {5,7,7}, {7-2,7-2,9-2}, {9-3,9-3,9-3} } };
    if ( bPSlice )
    {
      uiBlkBit[0] = 3;
      uiBlkBit[1] = 0;
      uiBlkBit[2] = 0;
    }
    else
    {
      ::memcpy( uiBlkBit, aauiMbBits[iPartIdx][uiLastMode], 3*sizeof(UInt) );
    }
  }
  else if ( eCUMode == SIZE_NxN )
  {
    uiBlkBit[0] = (! bPSlice) ? 3 : 1;
    uiBlkBit[1] = 3;
    uiBlkBit[2] = 5;
  }
  else
  {
    printf("Wrong!\n");
    assert( 0 );
  }
}

Void TEncSearch::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->iN = pSrc->iN;
  for (Int i = 0; i < pSrc->iN; i++)
  {
    pDst->m_acMvCand[i] = pSrc->m_acMvCand[i];
  }
}
#ifdef QC_AMVRES 
Void TEncSearch::xCheckBestMVP_onefourth ( TComDataCU* pcCU, RefPicList eRefPicList, TComMv cMv, TComMv& rcMvPred, Int& riMVPIdx, UInt& ruiBits, UInt& ruiCost )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();
  
#if HHI_IMVP
  TComMv cMvPredIMP;
  if ( pcCU->getSlice()->getSPS()->getUseIMP() )
  {
    cMvPredIMP = m_cMvPredMeasure.getMVPred( cMv.getHor(), cMv.getVer() );
    assert( pcAMVPInfo->m_acMvCand[riMVPIdx].getVer() == rcMvPred.getVer() );
    assert( cMvPredIMP.getHor() == rcMvPred.getHor() );
  }
  else
  {
    assert(pcAMVPInfo->m_acMvCand[riMVPIdx] == rcMvPred);
  }
#else
  assert(pcAMVPInfo->m_acMvCand[riMVPIdx] == rcMvPred);
#endif
  if (pcAMVPInfo->iN < 2) return;
  
  m_pcRdCost->getMotionCost( 1, 0 );
  m_pcRdCost->setCostScale ( 0    );
  
  Int iBestMVPIdx = riMVPIdx;
  
  TComMv rcMvPred_round=rcMvPred,cMv_round=cMv;
  
  rcMvPred_round.scale_down();
  cMv_round.scale_down();
  m_pcRdCost->setPredictor( rcMvPred_round );
  
  
  Int iOrgMvBits  = m_pcRdCost->getBits(cMv_round.getHor(), cMv_round.getVer());
  Int iBestMvBits = iOrgMvBits;
  
  for (Int iMVPIdx = 0; iMVPIdx < pcAMVPInfo->iN; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx) continue;
    
#if HHI_IMVP
    if ( pcCU->getSlice()->getSPS()->getUseIMP() )
    {
      TComMv rcMvPred_round_curr= rcMvPred;
      rcMvPred_round_curr.setVer( pcAMVPInfo->m_acMvCand[iMVPIdx].getVer() );
      rcMvPred_round_curr.scale_down();
      m_pcRdCost->setPredictor( rcMvPred_round_curr);
    }
    else
    {
      TComMv rcMvPred_round_curr=pcAMVPInfo->m_acMvCand[iMVPIdx];
      rcMvPred_round_curr.scale_down();
      m_pcRdCost->setPredictor( rcMvPred_round_curr);
    }
#else
    TComMv rcMvPred_round_curr=pcAMVPInfo->m_acMvCand[iMVPIdx];
    rcMvPred_round_curr.scale_down();
    m_pcRdCost->setPredictor( rcMvPred_round_curr);
#endif
    Int iMvBits = m_pcRdCost->getBits(cMv_round.getHor(), cMv_round.getVer());
    
    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }
  
  if (iBestMVPIdx != riMVPIdx)	//if changed
  {
#if HHI_IMVP
    if ( pcCU->getSlice()->getSPS()->getUseIMP() )
    {
      m_cMvPredMeasure.m_iYOrigin = pcAMVPInfo->m_acMvCand[iBestMVPIdx].getVer();
      rcMvPred.setVer( pcAMVPInfo->m_acMvCand[iBestMVPIdx].getVer() );
      rcMvPred.setHor( cMvPredIMP.getHor() );
    }
    else
    {
      rcMvPred = pcAMVPInfo->m_acMvCand[iBestMVPIdx];
    }
#else
    rcMvPred = pcAMVPInfo->m_acMvCand[iBestMVPIdx];
#endif
    iOrgMvBits  += m_auiMVPIdxCost[riMVPIdx][pcAMVPInfo->iN];
    iBestMvBits += m_auiMVPIdxCost[iBestMVPIdx][pcAMVPInfo->iN];
    
    riMVPIdx = iBestMVPIdx;
    UInt uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits ))	+ m_pcRdCost->getCost( ruiBits );
  }
}
#endif 
Void TEncSearch::xCheckBestMVP ( TComDataCU* pcCU, RefPicList eRefPicList, TComMv cMv, TComMv& rcMvPred, Int& riMVPIdx, UInt& ruiBits, UInt& ruiCost )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();
  
#if HHI_IMVP
  TComMv cMvPredIMP;
  if ( pcCU->getSlice()->getSPS()->getUseIMP() )
  {
    cMvPredIMP = m_cMvPredMeasure.getMVPred( cMv.getHor(), cMv.getVer() );
    assert( pcAMVPInfo->m_acMvCand[riMVPIdx].getVer() == rcMvPred.getVer() );
    assert( cMvPredIMP.getHor() == rcMvPred.getHor() );
  }
  else
  {
    assert(pcAMVPInfo->m_acMvCand[riMVPIdx] == rcMvPred);
  }
#else
  assert(pcAMVPInfo->m_acMvCand[riMVPIdx] == rcMvPred);
#endif
  
  if (pcAMVPInfo->iN < 2) return;
  
  m_pcRdCost->getMotionCost( 1, 0 );
  m_pcRdCost->setCostScale ( 0    );
  
  Int iBestMVPIdx = riMVPIdx;
  
  m_pcRdCost->setPredictor( rcMvPred );
  Int iOrgMvBits  = m_pcRdCost->getBits(cMv.getHor(), cMv.getVer());
  Int iBestMvBits = iOrgMvBits;
  
  for (Int iMVPIdx = 0; iMVPIdx < pcAMVPInfo->iN; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx) continue;
    
#if HHI_IMVP
    if ( pcCU->getSlice()->getSPS()->getUseIMP() )
    {
      cMvPredIMP.setVer( pcAMVPInfo->m_acMvCand[iMVPIdx].getVer() );
      m_pcRdCost->setPredictor( cMvPredIMP );
    }
    else
    {
      m_pcRdCost->setPredictor( pcAMVPInfo->m_acMvCand[iMVPIdx] );
    }
#else
    m_pcRdCost->setPredictor( pcAMVPInfo->m_acMvCand[iMVPIdx] );
#endif
    
    Int iMvBits = m_pcRdCost->getBits(cMv.getHor(), cMv.getVer());
    
    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }
  
  if (iBestMVPIdx != riMVPIdx)  //if changed
  {
#if HHI_IMVP
    if ( pcCU->getSlice()->getSPS()->getUseIMP() )
    {
      m_cMvPredMeasure.m_iYOrigin = pcAMVPInfo->m_acMvCand[iBestMVPIdx].getVer();
      rcMvPred.setVer( pcAMVPInfo->m_acMvCand[iBestMVPIdx].getVer() );
      rcMvPred.setHor( cMvPredIMP.getHor() );
    }
    else
    {
      rcMvPred = pcAMVPInfo->m_acMvCand[iBestMVPIdx];
    }
#else
    rcMvPred = pcAMVPInfo->m_acMvCand[iBestMVPIdx];
#endif
    
    iOrgMvBits  += m_auiMVPIdxCost[riMVPIdx][pcAMVPInfo->iN];
    iBestMvBits += m_auiMVPIdxCost[iBestMVPIdx][pcAMVPInfo->iN];
    
    riMVPIdx = iBestMVPIdx;
    UInt uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits ))  + m_pcRdCost->getCost( ruiBits );
  }
}

UInt TEncSearch::xGetTemplateCost( TComDataCU* pcCU,
                                  UInt        uiPartIdx,
                                  UInt      uiPartAddr,
                                  TComYuv*    pcOrgYuv,
                                  TComYuv*    pcTemplateCand,
                                  TComMv      cMvCand,
                                  Int         iMVPIdx,
                                  Int     iMVPNum,
                                  RefPicList  eRefPicList,
                                  Int         iRefIdx,
                                  Int         iSizeX,
                                  Int         iSizeY)
{
  UInt uiCost  = MAX_INT;
  
  TComPicYuv* pcPicYuvRef = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec();
  
  // prediction pattern
#ifdef QC_AMVRES
  if (pcCU->getSlice()->getSPS()->getUseAMVRes())
    xPredInterLumaBlkHMV( pcCU, pcPicYuvRef, uiPartAddr, &cMvCand, iSizeX, iSizeY, pcTemplateCand );
  else
#endif
    xPredInterLumaBlk ( pcCU, pcPicYuvRef, uiPartAddr, &cMvCand, iSizeX, iSizeY, pcTemplateCand );
  
  // calc distortion
  uiCost = m_pcRdCost->getDistPart( pcTemplateCand->getLumaAddr(uiPartAddr), pcTemplateCand->getStride(), pcOrgYuv->getLumaAddr(uiPartAddr), pcOrgYuv->getStride(), iSizeX, iSizeY, DF_SAD );
  uiCost = (UInt) m_pcRdCost->calcRdCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum], uiCost, false, DF_SAD );
  return uiCost;
}

#ifdef DCM_PBIC
UInt TEncSearch::xGetTemplateCostIC   ( TComDataCU* pcCU,
                                        UInt        uiPartIdx,
                                        UInt        uiPartAddr,
                                        TComYuv*    pcOrgYuv,
                                        TComIc      cIcCand,
                                        Int         iICPIdx,
                                        Int         iICPNum,
                                        RefPicList  eRefPicList,
                                        Int         iSizeX,
                                        Int         iSizeY)
{
  UInt uiCost  = MAX_INT;

  //Apply IC
  if (eRefPicList == REF_PIC_LIST_X )
  {
    Int iRefIdx[2];
    iRefIdx[0] = pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr );
    iRefIdx[1] = pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr );

    xPredICompLumaBlk  ( &cIcCand, iSizeX   , iSizeY   , m_acYuvTempIC[0].getStride() , 1, m_acYuvTempIC[0].getLumaAddr(uiPartAddr), m_acYuvPred[0].getStride() , 1, m_acYuvPred[0].getLumaAddr(uiPartAddr), REF_PIC_LIST_0 );
    xPredICompChromaBlk( &cIcCand, iSizeX>>1, iSizeY>>1, m_acYuvTempIC[0].getCStride(), 1, m_acYuvTempIC[0].getCbAddr(uiPartAddr)  , m_acYuvPred[0].getCStride(), 1, m_acYuvPred[0].getCbAddr(uiPartAddr)  , REF_PIC_LIST_0 );
    xPredICompChromaBlk( &cIcCand, iSizeX>>1, iSizeY>>1, m_acYuvTempIC[0].getCStride(), 1, m_acYuvTempIC[0].getCrAddr(uiPartAddr)  , m_acYuvPred[0].getCStride(), 1, m_acYuvPred[0].getCrAddr(uiPartAddr)  , REF_PIC_LIST_0 );

    xPredICompLumaBlk  ( &cIcCand, iSizeX   , iSizeY   , m_acYuvTempIC[1].getStride() , 1, m_acYuvTempIC[1].getLumaAddr(uiPartAddr), m_acYuvPred[1].getStride() , 1, m_acYuvPred[1].getLumaAddr(uiPartAddr), REF_PIC_LIST_1 );
    xPredICompChromaBlk( &cIcCand, iSizeX>>1, iSizeY>>1, m_acYuvTempIC[1].getCStride(), 1, m_acYuvTempIC[1].getCbAddr(uiPartAddr)  , m_acYuvPred[1].getCStride(), 1, m_acYuvPred[1].getCbAddr(uiPartAddr)  , REF_PIC_LIST_1 );
    xPredICompChromaBlk( &cIcCand, iSizeX>>1, iSizeY>>1, m_acYuvTempIC[1].getCStride(), 1, m_acYuvTempIC[1].getCrAddr(uiPartAddr)  , m_acYuvPred[1].getCStride(), 1, m_acYuvPred[1].getCrAddr(uiPartAddr)  , REF_PIC_LIST_1 );

    TComYuv* pcYuvPred = m_acYuvTempIC;
    xWeightedAverage( pcCU, m_acYuvTempIC, m_acYuvTempIC+1, iRefIdx[0], iRefIdx[1], uiPartAddr, iSizeX, iSizeY, pcYuvPred);
  }
  else
  {
    xPredICompLumaBlk  ( &cIcCand, iSizeX   , iSizeY   , m_acYuvTempIC->getStride() , 1, m_acYuvTempIC->getLumaAddr(uiPartAddr), m_acYuvPred[eRefPicList].getStride() , 1, m_acYuvPred[eRefPicList].getLumaAddr(uiPartAddr), eRefPicList );
    xPredICompChromaBlk( &cIcCand, iSizeX>>1, iSizeY>>1, m_acYuvTempIC->getCStride(), 1, m_acYuvTempIC->getCbAddr(uiPartAddr)  , m_acYuvPred[eRefPicList].getCStride(), 1, m_acYuvPred[eRefPicList].getCbAddr(uiPartAddr)  , eRefPicList );
    xPredICompChromaBlk( &cIcCand, iSizeX>>1, iSizeY>>1, m_acYuvTempIC->getCStride(), 1, m_acYuvTempIC->getCrAddr(uiPartAddr)  , m_acYuvPred[eRefPicList].getCStride(), 1, m_acYuvPred[eRefPicList].getCrAddr(uiPartAddr)  , eRefPicList );
  }

  // calc distortion
  DFunc eDFunc = (m_pcEncCfg->getUseHADME() == true) ? DF_HADS : DF_SADS;
  uiCost = m_pcRdCost->getDistPart( m_acYuvTempIC->getLumaAddr(uiPartAddr), m_acYuvTempIC->getStride(), pcOrgYuv->getLumaAddr(uiPartAddr), pcOrgYuv->getStride(), iSizeX, iSizeY, eDFunc );
  uiCost += UInt( m_pcRdCost->getDistPart( m_acYuvTempIC->getCbAddr(uiPartAddr), m_acYuvTempIC->getCStride(),pcOrgYuv->getCbAddr(uiPartAddr),pcOrgYuv->getCStride(), iSizeX>>1, iSizeY>>1, eDFunc)
                + m_pcRdCost->getDistPart( m_acYuvTempIC->getCrAddr(uiPartAddr), m_acYuvTempIC->getCStride(),pcOrgYuv->getCrAddr(uiPartAddr),pcOrgYuv->getCStride(), iSizeX>>1, iSizeY>>1, eDFunc));
  uiCost =  (UInt) ( uiCost / 1.5 );

  uiCost = (UInt) m_pcRdCost->calcRdCost( m_auiICPIdxCost[iICPIdx][iICPNum], uiCost, false, DF_SAD );

	return uiCost;
}
#endif

Void TEncSearch::xMotionEstimation( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, RefPicList eRefPicList, TComMv* pcMvPred, Int iRefIdxPred, TComMv& rcMv, UInt& ruiBits, UInt& ruiCost, Bool bBi  )
{
  UInt          uiPartAddr;
  Int           iRoiWidth;
  Int           iRoiHeight;
  
  TComMv        cMvHalf, cMvQter;
  TComMv        cMvSrchRngLT;
  TComMv        cMvSrchRngRB;
  
  TComYuv*  pcYuv = pcYuvOrg;
#ifdef ROUNDING_CONTROL_BIPRED
  Pel			pRefBufY[16384];  // 128x128
#endif
#ifdef QC_SIFO
  Int iList = (eRefPicList==REF_PIC_LIST_0)? 0 : 1;
  TComMv cMv_start = rcMv;
  Int FullpelOffset;
#endif
  m_iSearchRange = m_aaiAdaptSR[eRefPicList][iRefIdxPred];
  
  Int           iSrchRng      = ( bBi ? 8 : m_iSearchRange );
  TComPattern*  pcPatternKey  = pcCU->getPattern        ();
  
  Double        fWeight       = 1.0;
  
  pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );
  
  if ( bBi )
  {
    TComYuv*  pcYuvOther = &m_acYuvPred[1-(Int)eRefPicList];
    pcYuv                = &m_cYuvPredTemp;
    
    pcYuvOrg->copyPartToPartYuv( pcYuv, uiPartAddr, iRoiWidth, iRoiHeight );
    
#ifdef ROUNDING_CONTROL_BIPRED
	Int y;
	//Int x;
	Pel *pRefY = pcYuvOther->getLumaAddr(uiPartAddr);
	Int iRefStride = pcYuvOther->getStride();
    
	// copy the MC block into pRefBufY
	for( y = 0; y < iRoiHeight; y++)
	{
      memcpy(pRefBufY+y*iRoiWidth,pRefY,sizeof(Pel)*iRoiWidth);
      pRefY += iRefStride;
	}
#else
    pcYuv->removeHighFreq( pcYuvOther, uiPartAddr, iRoiWidth, iRoiHeight );
#endif
    
    fWeight = 0.5;
  }
  
  //  Search key pattern initialization
  pcPatternKey->initPattern( pcYuv->getLumaAddr( uiPartAddr ),
                            pcYuv->getCbAddr  ( uiPartAddr ),
                            pcYuv->getCrAddr  ( uiPartAddr ),
                            iRoiWidth,
                            iRoiHeight,
                            pcYuv->getStride(),
                            0, 0, 0, 0 );
  
  Pel*        piRefY      = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRec()->getLumaAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr );
  Int         iRefStride  = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRec()->getStride();
  
  TComMv      cMvPred = *pcMvPred;
  
#ifdef QC_AMVRES
  TComMv      cMvPred_onefourth;
#endif
  
  if ( bBi )  xSetSearchRange   ( pcCU, rcMv   , iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
  else        xSetSearchRange   ( pcCU, cMvPred, iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
  
  m_pcRdCost->getMotionCost ( 1, 0 );
  
#ifdef QC_AMVRES
  if (pcCU->getSlice()->getSPS()->getUseAMVRes())
  {
    cMvPred_onefourth = *pcMvPred;
    cMvPred_onefourth.scale_down();
    m_pcRdCost->setPredictor	( cMvPred_onefourth );
  }
  else
#endif
    m_pcRdCost->setPredictor  ( *pcMvPred );
  m_pcRdCost->setCostScale  ( 2 );
  
  //  Do integer search
#ifdef ROUNDING_CONTROL_BIPRED
  if( bBi ) 
  {
	xPatternSearch_Bi      ( pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost, pRefBufY, pcCU->getSlice()->isRounding() );
  } 
  else
  {
    if ( !m_iFastSearch)
    {
      xPatternSearch      ( pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost );
    }
    else
    {
      rcMv = *pcMvPred;
      xPatternSearchFast  ( pcCU, pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost );
    }
  }
#else
  if ( !m_iFastSearch || bBi )
  {
    xPatternSearch      ( pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost );
  }
  else
  {
    rcMv = *pcMvPred;
    xPatternSearchFast  ( pcCU, pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost );
  }
#endif
  
#ifdef QC_SIFO
  UInt NumMEOffsets = getNum_Offset_FullpelME(iList); 
  if(pcCU->getSlice()->getUseSIFO() && NumMEOffsets>0 && iRefIdxPred==0)
  {
    for(UInt MEloop = 0; MEloop < NumMEOffsets; MEloop++)
    {
      UInt uiCostTemp = MAX_UINT;
      TComMv cMv_temp = cMv_start;  
      FullpelOffset = getOffset_FullpelME(iList,MEloop);
      xAddSubFullPelOffset(pcPatternKey, FullpelOffset, 1);
      //  Do integer search
#ifdef ROUNDING_CONTROL_BIPRED
	  if( bBi ) 
	  {
#if BUGFIX48
		xPatternSearch_Bi      ( pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, cMv_temp, uiCostTemp, pRefBufY, pcCU->getSlice()->isRounding() );
#else
		xPatternSearch_Bi      ( pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost, pRefBufY, pcCU->getSlice()->isRounding() );
#endif
	  }
	  else
	  {
        if (!m_iFastSearch)
        {
          xPatternSearch      ( pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, cMv_temp, uiCostTemp );
        }
        else
        {
          cMv_temp = *pcMvPred;
          xPatternSearchFast  ( pcCU, pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, cMv_temp, uiCostTemp );
        }
	  }
#else
      if ( !m_iFastSearch || bBi )
      {
        xPatternSearch      ( pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, cMv_temp, uiCostTemp );
      }
      else
      {
        cMv_temp = *pcMvPred;
        xPatternSearchFast  ( pcCU, pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, cMv_temp, uiCostTemp );
      }
#endif
      xAddSubFullPelOffset(pcPatternKey, FullpelOffset, 0);
      
      if(uiCostTemp < ruiCost)
      {
        rcMv = cMv_temp;
        ruiCost = uiCostTemp;
      }
    }
  }
#endif
  
  
  m_pcRdCost->getMotionCost( 1, 0 );
  m_pcRdCost->setCostScale ( 1 );
  
#ifdef ROUNDING_CONTROL_BIPRED
  if( bBi ) 
  {
	Bool bRound =  pcCU->getSlice()->isRounding() ;
#if HHI_INTERP_FILTER
    InterpFilterType ePFilt = (InterpFilterType)pcCU->getSlice()->getInterpFilterType();
    switch ( ePFilt )
    {
#ifdef QC_AMVRES
#ifdef QC_SIFO
      case IPF_QC_SIFO:
        xPatternSearchFracDIF_QC_Bi( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost,pcMvPred,iRefIdxPred, pRefBufY, bRound);
        break;
#endif
#if TEN_DIRECTIONAL_INTERP
      case IPF_TEN_DIF:
        xPatternSearchFracDIF_TEN_Bi( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost , pRefBufY, bRound);
        break;
#endif
      case IPF_HHI_4TAP_MOMS:
      case IPF_HHI_6TAP_MOMS:
        piRefY = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRecFilt()->getLumaAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr );
        xPatternSearchFracMOMS_Bi( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost, ePFilt ,pcMvPred,iRefIdxPred, pRefBufY, bRound);
        break;
      default:
        xPatternSearchFracDIF_Bi( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost,pcMvPred,iRefIdxPred, pRefBufY, bRound);  
#else
#ifdef QC_SIFO
      case IPF_QC_SIFO:
        xPatternSearchFracDIF_QC_Bi( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost , pRefBufY, bRound);
        break;
#endif
#if TEN_DIRECTIONAL_INTERP
      case IPF_TEN_DIF:
        xPatternSearchFracDIF_TEN_Bi( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost , pRefBufY, bRound);
        break;
#endif
      case IPF_HHI_4TAP_MOMS:
      case IPF_HHI_6TAP_MOMS:
        piRefY = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRecFilt()->getLumaAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr );
        xPatternSearchFracMOMS_Bi( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost, ePFilt , pRefBufY, bRound);
        break;
      default:
        xPatternSearchFracDIF_Bi( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost, pRefBufY, bRound );
#endif
    }
#else
#ifdef QC_AMVRES
    xPatternSearchFracDIF_Bi( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost,pcMvPred,iRefIdxPred, pRefBufY, bRound);
#else
    xPatternSearchFracDIF_Bi( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost, pRefBufY, bRound );
#endif
#endif
  }
  else
#endif
  {
#if HHI_INTERP_FILTER
    InterpFilterType ePFilt = (InterpFilterType)pcCU->getSlice()->getInterpFilterType();
    switch ( ePFilt )
    {
#ifdef QC_AMVRES
#ifdef QC_SIFO
      case IPF_QC_SIFO:
        xPatternSearchFracDIF_QC( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost,pcMvPred,iRefIdxPred);
        break;
#endif
#if TEN_DIRECTIONAL_INTERP
      case IPF_TEN_DIF:
        xPatternSearchFracDIF_TEN( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost );
        break;
#endif
      case IPF_HHI_4TAP_MOMS:
      case IPF_HHI_6TAP_MOMS:
        piRefY = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRecFilt()->getLumaAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr );
        xPatternSearchFracMOMS( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost, ePFilt ,pcMvPred,iRefIdxPred);
        break;
      default:
        xPatternSearchFracDIF( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost,pcMvPred,iRefIdxPred);
#else
#ifdef QC_SIFO
      case IPF_QC_SIFO:
        xPatternSearchFracDIF_QC( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost );
        break;
#endif
#if TEN_DIRECTIONAL_INTERP
      case IPF_TEN_DIF:
        xPatternSearchFracDIF_TEN( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost );
        break;
#endif
      case IPF_HHI_4TAP_MOMS:
      case IPF_HHI_6TAP_MOMS:
        piRefY = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRecFilt()->getLumaAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr );
        xPatternSearchFracMOMS( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost, ePFilt );
        break;
      default:
        xPatternSearchFracDIF( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost );
#endif
    }
#else
#ifdef QC_AMVRES
    xPatternSearchFracDIF( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost,pcMvPred,iRefIdxPred);
#else
    xPatternSearchFracDIF( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost );
#endif
#endif
  }
  
  
  
  m_pcRdCost->setCostScale( 0 );
#ifdef QC_AMVRES 
  rcMv <<= (2+pcCU->getSlice()->getSPS()->getUseAMVRes());
  rcMv += (cMvHalf <<= (1+pcCU->getSlice()->getSPS()->getUseAMVRes()));
  rcMv +=  cMvQter;
  UInt uiMvBits;
  
#if HHI_IMVP
  if ( pcCU->getSlice()->getSPS()->getUseIMP() )
  {
    *pcMvPred = m_cMvPredMeasure.getMVPred( rcMv.getHor(), rcMv.getVer() ) ;
  }
#endif
  
  if (!rcMv.isHAM()&& pcCU->getSlice()->getSPS()->getUseAMVRes())
  {
    cMvPred_onefourth=*pcMvPred;
    cMvPred_onefourth.scale_down();
    m_pcRdCost->setPredictor	( cMvPred_onefourth );
    uiMvBits = m_pcRdCost->getBits( rcMv.getHor()/2, rcMv.getVer()/2 );
  }
  else
  {
    m_pcRdCost->setPredictor	( *pcMvPred );
    uiMvBits = m_pcRdCost->getBits( rcMv.getHor(), rcMv.getVer())+pcCU->getSlice()->getSPS()->getUseAMVRes();
  }
#else
  rcMv <<= 2;
  rcMv += (cMvHalf <<= 1);
  rcMv +=  cMvQter;
  
#if HHI_IMVP
  if ( pcCU->getSlice()->getSPS()->getUseIMP() )
  {
    *pcMvPred = m_cMvPredMeasure.getMVPred( rcMv.getHor(), rcMv.getVer() ) ;
    m_pcRdCost->setPredictor(*pcMvPred); 
  }
#endif
  
  UInt uiMvBits = m_pcRdCost->getBits( rcMv.getHor(), rcMv.getVer() );
#endif
  
  ruiBits      += uiMvBits;
  ruiCost       = (UInt)( floor( fWeight * ( (Double)ruiCost - (Double)m_pcRdCost->getCost( uiMvBits ) ) ) + (Double)m_pcRdCost->getCost( ruiBits ) );
}


Void TEncSearch::xSetSearchRange ( TComDataCU* pcCU, TComMv& cMvPred, Int iSrchRng, TComMv& rcMvSrchRngLT, TComMv& rcMvSrchRngRB )
{
#ifdef QC_AMVRES
  Int  iMvShift = (pcCU->getSlice()->getSPS()->getUseAMVRes())?3:2;
#else
  Int  iMvShift = 2;
#endif
  pcCU->clipMv( cMvPred );
  
  rcMvSrchRngLT.setHor( cMvPred.getHor() - (iSrchRng << iMvShift) );
  rcMvSrchRngLT.setVer( cMvPred.getVer() - (iSrchRng << iMvShift) );
  
  rcMvSrchRngRB.setHor( cMvPred.getHor() + (iSrchRng << iMvShift) );
  rcMvSrchRngRB.setVer( cMvPred.getVer() + (iSrchRng << iMvShift) );
  
  pcCU->clipMv        ( rcMvSrchRngLT );
  pcCU->clipMv        ( rcMvSrchRngRB );
  
  rcMvSrchRngLT >>= iMvShift;
  rcMvSrchRngRB >>= iMvShift;
}



#ifdef ROUNDING_CONTROL_BIPRED
Void TEncSearch::xPatternSearch_Bi( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, TComMv& rcMv, UInt& ruiSAD, Pel* pcRefY2, Bool bRound )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();
  
  UInt  uiSad;
  UInt  uiSadBest         = MAX_UINT;
  Int   iBestX = 0;
  Int   iBestY = 0;
  
  Pel*  piRefSrch;
  
  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam_Bi( pcPatternKey, piRefY, iRefStride,  m_cDistParam);
  
  // fast encoder decision: use subsampled SAD for integer ME
  if ( m_pcEncCfg->getUseFastEnc() )
  {
    if ( m_cDistParam.iRows > 8 )
    {
      m_cDistParam.iSubShift = 1;
    }
  }
  
  piRefY += (iSrchRngVerTop * iRefStride);
  for ( Int y = iSrchRngVerTop; y <= iSrchRngVerBottom; y++ )
  {
    for ( Int x = iSrchRngHorLeft; x <= iSrchRngHorRight; x++ )
    {
      //  find min. distortion position
      piRefSrch = piRefY + x;
      m_cDistParam.pCur = piRefSrch;
      uiSad = m_cDistParam.DistFuncRnd( &m_cDistParam, pcRefY2, bRound );
      
      // motion cost
#if HHI_IMVP
      if ( m_pcEncCfg->getUseIMP() )
      {
#ifdef QC_AMVRES
        if (m_pcEncCfg->getUseAMVRes() )
        {
          TComMv cMvPred = m_cMvPredMeasure.getMVPred( (x<<3) , (y<<3) );
          cMvPred.scale_down();
          m_pcRdCost->setPredictor( cMvPred );
        }
        else
        {
          TComMv cMvPred = m_cMvPredMeasure.getMVPred( (x<<2) , (y<<2) );
          m_pcRdCost->setPredictor( cMvPred );
        }
#else
        TComMv cMvPred = m_cMvPredMeasure.getMVPred( (x<<2) , (y<<2) );
        m_pcRdCost->setPredictor( cMvPred );
#endif
      }
#endif
      uiSad += m_pcRdCost->getCost( x, y );
      
      if ( uiSad < uiSadBest )
      {
        uiSadBest = uiSad;
        iBestX    = x;
        iBestY    = y;
      }
    }
    piRefY += iRefStride;
  }
  
  rcMv.set( iBestX, iBestY );
#if HHI_IMVP
  if ( m_pcEncCfg->getUseIMP() )
  {
#ifdef QC_AMVRES
    if (m_pcEncCfg->getUseAMVRes() )
    {
      TComMv cMvPred = m_cMvPredMeasure.getMVPred( (iBestX<<3) , (iBestY<<3) );
      cMvPred.scale_down();
      m_pcRdCost->setPredictor( cMvPred );
    }
    else
    {
      TComMv cMvPred = m_cMvPredMeasure.getMVPred( (iBestX<<2) , (iBestY<<2) );
      m_pcRdCost->setPredictor( cMvPred );
    }
#else
    TComMv cMvPred = m_cMvPredMeasure.getMVPred( (iBestX<<2) , (iBestY<<2) );
    m_pcRdCost->setPredictor( cMvPred );
#endif
  }
#endif
  ruiSAD = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY );
  return;
}
#endif

Void TEncSearch::xPatternSearch( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, TComMv& rcMv, UInt& ruiSAD )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();
  
  UInt  uiSad;
  UInt  uiSadBest         = MAX_UINT;
  Int   iBestX = 0;
  Int   iBestY = 0;
  
  Pel*  piRefSrch;
  
  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( pcPatternKey, piRefY, iRefStride,  m_cDistParam );
  
  // fast encoder decision: use subsampled SAD for integer ME
  if ( m_pcEncCfg->getUseFastEnc() )
  {
    if ( m_cDistParam.iRows > 8 )
    {
      m_cDistParam.iSubShift = 1;
    }
  }
  
  piRefY += (iSrchRngVerTop * iRefStride);
  for ( Int y = iSrchRngVerTop; y <= iSrchRngVerBottom; y++ )
  {
    for ( Int x = iSrchRngHorLeft; x <= iSrchRngHorRight; x++ )
    {
      //  find min. distortion position
      piRefSrch = piRefY + x;
      m_cDistParam.pCur = piRefSrch;
      uiSad = m_cDistParam.DistFunc( &m_cDistParam );
      
      // motion cost
#if HHI_IMVP
      if ( m_pcEncCfg->getUseIMP() )
      {
#ifdef QC_AMVRES
        if (m_pcEncCfg->getUseAMVRes() )
        {
          TComMv cMvPred = m_cMvPredMeasure.getMVPred( (x<<3) , (y<<3) );
          cMvPred.scale_down();
          m_pcRdCost->setPredictor( cMvPred );
        }
        else
        {
          TComMv cMvPred = m_cMvPredMeasure.getMVPred( (x<<2) , (y<<2) );
          m_pcRdCost->setPredictor( cMvPred );
        }
#else
        TComMv cMvPred = m_cMvPredMeasure.getMVPred( (x<<2) , (y<<2) );
        m_pcRdCost->setPredictor( cMvPred );
#endif
      }
#endif
      
      uiSad += m_pcRdCost->getCost( x, y );
      
      if ( uiSad < uiSadBest )
      {
        uiSadBest = uiSad;
        iBestX    = x;
        iBestY    = y;
      }
    }
    piRefY += iRefStride;
  }
  
  rcMv.set( iBestX, iBestY );
  
#if HHI_IMVP
  if ( m_pcEncCfg->getUseIMP() )
  {
#ifdef QC_AMVRES
    if (m_pcEncCfg->getUseAMVRes() )
    {
      TComMv cMvPred = m_cMvPredMeasure.getMVPred( (iBestX<<3) , (iBestY<<3) );
      cMvPred.scale_down();
      m_pcRdCost->setPredictor( cMvPred );
    }
    else
    {
      TComMv cMvPred = m_cMvPredMeasure.getMVPred( (iBestX<<2) , (iBestY<<2) );
      m_pcRdCost->setPredictor( cMvPred );
    }
#else
    TComMv cMvPred = m_cMvPredMeasure.getMVPred( (iBestX<<2) , (iBestY<<2) );
    m_pcRdCost->setPredictor( cMvPred );
#endif
  }
#endif
  ruiSAD = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY );
  return;
}

Void TEncSearch::xPatternSearchFast( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, TComMv& rcMv, UInt& ruiSAD )
{
  pcCU->getMvPredLeft       ( m_acMvPredictors[0] );
  pcCU->getMvPredAbove      ( m_acMvPredictors[1] );
  pcCU->getMvPredAboveRight ( m_acMvPredictors[2] );
  
  switch ( m_iFastSearch )
  {
    case 1:
      xTZSearch( pcCU, pcPatternKey, piRefY, iRefStride, pcMvSrchRngLT, pcMvSrchRngRB, rcMv, ruiSAD );
      break;
      
    default:
      break;
  }
}

Void TEncSearch::xTZSearch( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, TComMv& rcMv, UInt& ruiSAD )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();
  
  TZ_SEARCH_CONFIGURATION
  
  UInt uiSearchRange = m_iSearchRange;
  pcCU->clipMv( rcMv );
#ifdef QC_AMVRES
  rcMv >>= (2+pcCU->getSlice()->getSPS()->getUseAMVRes());
#else
  rcMv >>= 2;
#endif
  // init TZSearchStruct
  IntTZSearchStruct cStruct;
  cStruct.iYStride    = iRefStride;
  cStruct.piRefY      = piRefY;
  cStruct.uiBestSad   = MAX_UINT;
  
  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( pcPatternKey, cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );
  
  // test whether one of PRED_A, PRED_B, PRED_C MV is better start point than Median predictor
  if ( bTestOtherPredictedMV )
  {
    for ( UInt index = 0; index < 3; index++ )
    {
      TComMv cMv = m_acMvPredictors[index];
      pcCU->clipMv( cMv );
#ifdef QC_AMVRES
      cMv >>= (2+pcCU->getSlice()->getSPS()->getUseAMVRes());
#else
      cMv >>= 2;
#endif
      xTZSearchHelp( pcPatternKey, cStruct, cMv.getHor(), cMv.getVer(), 0, 0 );
    }
  }
  
  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    xTZSearchHelp( pcPatternKey, cStruct, 0, 0, 0, 0 );
  }
  
  // start search
  Int  iDist = 0;
  Int  iStartX = cStruct.iBestX;
  Int  iStartY = cStruct.iBestY;
  
  // first search
  for ( iDist = 1; iDist <= (Int)uiSearchRange; iDist*=2 )
  {
    if ( bFirstSearchDiamond == 1 )
    {
      xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
    }
    else
    {
      xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
    }
    
    if ( bFirstSearchStop && ( cStruct.uiBestRound >= uiFirstSearchRounds ) ) // stop criterion
    {
      break;
    }
  }
  
  // test whether zero Mv is a better start point than Median predictor
  if ( bTestZeroVectorStart && ((cStruct.iBestX != 0) || (cStruct.iBestY != 0)) )
  {
    xTZSearchHelp( pcPatternKey, cStruct, 0, 0, 0, 0 );
    if ( (cStruct.iBestX == 0) && (cStruct.iBestY == 0) )
    {
      // test its neighborhood
      for ( iDist = 1; iDist <= (Int)uiSearchRange; iDist*=2 )
      {
        xTZ8PointDiamondSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, 0, 0, iDist );
        if ( bTestZeroVectorStop && (cStruct.uiBestRound > 0) ) // stop criterion
        {
          break;
        }
      }
    }
  }
  
  // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
  if ( cStruct.uiBestDistance == 1 )
  {
    cStruct.uiBestDistance = 0;
    xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
  }
  
  // raster search if distance is too big
  if ( bEnableRasterSearch && ( ((Int)(cStruct.uiBestDistance) > iRaster) || bAlwaysRasterSearch ) )
  {
    cStruct.uiBestDistance = iRaster;
    for ( iStartY = iSrchRngVerTop; iStartY <= iSrchRngVerBottom; iStartY += iRaster )
    {
      for ( iStartX = iSrchRngHorLeft; iStartX <= iSrchRngHorRight; iStartX += iRaster )
      {
        xTZSearchHelp( pcPatternKey, cStruct, iStartX, iStartY, 0, iRaster );
      }
    }
  }
  
  // raster refinement
  if ( bRasterRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      if ( cStruct.uiBestDistance > 1 )
      {
        iDist = cStruct.uiBestDistance >>= 1;
        if ( bRasterRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        else
        {
          xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
      }
      
      // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
        }
      }
    }
  }
  
  // start refinement
  if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < (Int)uiSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        else
        {
          xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }
      
      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
        }
      }
    }
  }
  
  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCost( cStruct.iBestX, cStruct.iBestY );
}


#ifdef ROUNDING_CONTROL_BIPRED
#ifdef QC_AMVRES
Void TEncSearch::xPatternSearchFracDIF_Bi( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost,TComMv *PredMv, Int iRefIdxPred , Pel* piRefY2, Bool bRound )
#else
Void TEncSearch::xPatternSearchFracDIF_Bi( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost, Pel* piRefY2, Bool bRound )
#endif
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern( piRefY +  iOffset,
                          NULL,
                          NULL,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride,
                          0, 0, 0, 0 );
  Pel*  piRef;
#ifdef QC_AMVRES
  Int iRefStride_HAM	= iRefStride;
#endif
  iRefStride  = m_cYuvExt.getStride();
  
  //  Half-pel refinement
  xExtDIFUpSamplingH ( &cPatternRoi, &m_cYuvExt );
  piRef = m_cYuvExt.getLumaAddr() + ((iRefStride + m_iDIFHalfTap) << 2);
  
  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  ruiCost = xPatternRefinement_Bi( pcPatternKey, piRef, iRefStride, 4, 2, rcMvHalf, piRefY2, bRound );
  
  m_pcRdCost->setCostScale( 0 );
  
  //  Quater-pel refinement
  Pel*  piSrcPel = cPatternRoi.getROIY() + (rcMvHalf.getHor() >> 1) + cPatternRoi.getPatternLStride() * (rcMvHalf.getVer() >> 1);
  Int*  piSrc    = m_piYuvExt  + ((m_iYuvExtStride + m_iDIFHalfTap) << 2) + (rcMvHalf.getHor() << 1) + m_iYuvExtStride * (rcMvHalf.getVer() << 1);
  piRef += (rcMvHalf.getHor() << 1) + iRefStride * (rcMvHalf.getVer() << 1);
  xExtDIFUpSamplingQ ( pcPatternKey, piRef, iRefStride, piSrcPel, cPatternRoi.getPatternLStride(), piSrc, m_iYuvExtStride, m_puiDFilter[rcMvHalf.getHor()+rcMvHalf.getVer()*3] );
  
  rcMvQter = *pcMvInt;   rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost = xPatternRefinement_Bi( pcPatternKey, piRef, iRefStride, 4, 1, rcMvQter, piRefY2, bRound );
  
#ifdef QC_AMVRES
  if (pcCU->getSlice()->getSPS()->getUseAMVRes())
  {
    if (pcCU->getSlice()->getSymbolMode()==0 && iRefIdxPred !=0 )
    {
      rcMvQter <<=1;
    }
    else
    {
      piRef = piRefY; 
      TComMv rcMv_HAM=*pcMvInt; rcMv_HAM<<= 1;  
      rcMv_HAM += rcMvHalf;  rcMv_HAM <<= 1;
      rcMv_HAM += rcMvQter; 
      ruiCost   = xPatternRefinementHAM_DIF_Bi( pcPatternKey, piRef, iRefStride_HAM, 4, rcMv_HAM,ruiCost ,PredMv, piRefY2, bRound );
      rcMvQter <<=1;
      rcMvQter+=rcMv_HAM;
    }
  }
#endif
}

#ifdef QC_SIFO
#ifdef QC_AMVRES
Void TEncSearch::xPatternSearchFracDIF_QC_Bi( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost,TComMv *PredMv, Int iRefIdxPred , Pel* piRefY2, Bool bRound )
#else
Void TEncSearch::xPatternSearchFracDIF_QC_Bi( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost , Pel* piRefY2, Bool bRound )
#endif
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern( piRefY +  iOffset,
                          NULL,
                          NULL,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride,
                          0, 0, 0, 0 );
  Pel*  piRef;
  Pel* piRef_temp = piRefY; 
  Int iRefStride_temp = iRefStride;
#ifdef QC_AMVRES
  Int iRefStride_HAM	= iRefStride;
#endif
  
  iRefStride  = m_cYuvExt.getStride();
  
  //  Half-pel refinement
  xExtDIFUpSamplingH_QC ( &cPatternRoi, &m_cYuvExt );
  piRef = m_cYuvExt.getLumaAddr() + ((iRefStride + m_iDIFHalfTap) << 2);
  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    
  ruiCost = xPatternRefinement_Bi( pcPatternKey, piRef, iRefStride, 4, 2, rcMvHalf, piRefY2, bRound );
  //  Quater-pel refinement on the fly
  m_pcRdCost->setCostScale( 0 ); 
  rcMvQter  = *pcMvInt;  rcMvQter <<= 1;    
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost  = xPatternRefinementMC_Bi( pcPatternKey, piRef_temp, iRefStride_temp, 4, 1, rcMvQter, piRefY2, bRound );
  
#ifdef QC_AMVRES
  if (pcCU->getSlice()->getSPS()->getUseAMVRes())
  {
    if (pcCU->getSlice()->getSymbolMode()==0 && iRefIdxPred !=0 )
    {
      rcMvQter <<=1;
    }
    else
    {
      piRef = piRefY; 
      TComMv rcMv_HAM=*pcMvInt; rcMv_HAM<<= 1;  
      rcMv_HAM += rcMvHalf;  rcMv_HAM <<= 1;
      rcMv_HAM += rcMvQter; 
      ruiCost   = xPatternRefinementHAM_DIF_Bi( pcPatternKey, piRef, iRefStride_HAM, 4, rcMv_HAM,ruiCost ,PredMv, piRefY2, bRound  );
      rcMvQter <<=1;
      rcMvQter+=rcMv_HAM;
    }
  }
#endif
}
UInt TEncSearch::xPatternRefinementMC_Bi  ( TComPattern* pcPatternKey, Pel* piRef, Int iRefStride, Int iIntStep, Int iFrac, TComMv& rcMvFrac, Pel* piRefY2, Bool bRound  )
{
  UInt  uiDist;
  UInt  uiDistBest  = MAX_UINT;
  UInt  uiDirecBest = 0;
  
  Pel* piLumaExt = m_cYuvExt.getLumaAddr();
  Int iYuvExtStride = m_cYuvExt.getStride();
  
  m_pcRdCost->setDistParam_Bi( pcPatternKey, piLumaExt, iYuvExtStride, 1, m_cDistParam, m_pcEncCfg->getUseHADME() );
  
  TComMv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);
  
  UInt i = 0;
  Int search_pos=16;
  
  for ( i = 0; i < search_pos; i++)
  {
    TComMv cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;
    
    TComMv cMvQter = cMvTest;
    cMvQter <<= (iFrac-1);
    Pel* piOrg   = m_cDistParam.pOrg;
    Int iStrideOrg = m_cDistParam.iStrideOrg;
    
    xPredInterLumaBlk_SIFOApplyME(piRef, iRefStride, piLumaExt, iYuvExtStride, &cMvQter, pcPatternKey->getROIYWidth(), 
                                  pcPatternKey->getROIYHeight(), piOrg, iStrideOrg,  0,  0);
    
    uiDist = m_cDistParam.DistFuncRnd( &m_cDistParam,piRefY2, bRound );
    
#if HHI_IMVP
    if ( m_pcEncCfg->getUseIMP() )
    {
#ifdef QC_AMVRES
      TComMv cMvPred;
      if(m_pcEncCfg->getUseAMVRes())
      {
        cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<iFrac, cMvTest.getVer()<<iFrac);
        cMvPred.scale_down();
      }
      else
        cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<(iFrac-1), cMvTest.getVer()<<(iFrac-1) );
      
      m_pcRdCost->setPredictor( cMvPred );
#else
      TComMv cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<(iFrac-1), cMvTest.getVer()<<(iFrac-1) );
      m_pcRdCost->setPredictor( cMvPred );
#endif
    }
#endif
    
    uiDist += m_pcRdCost->getCost( cMvQter.getHor(), cMvQter.getVer() );
    
    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
    }
  }
  
  rcMvFrac = pcMvRefine[uiDirecBest];
  return uiDistBest;
}
#endif


#if TEN_DIRECTIONAL_INTERP
Void TEncSearch::xPatternSearchFracDIF_TEN_Bi( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost , Pel* piRefY2, Bool bRound  )
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern( piRefY +  iOffset,
                          NULL,
                          NULL,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride,
                          0, 0, 0, 0 );
  Pel*  piRef;
  iRefStride  = m_cYuvExt.getStride();
  
  //  Half-pel refinement
  xExtDIFUpSamplingH_TEN ( &cPatternRoi, &m_cYuvExt );
  piRef = m_cYuvExt.getLumaAddr() + ((iRefStride + 0) << 2);
  
  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  ruiCost = xPatternRefinement_Bi( pcPatternKey, piRef, iRefStride, 4, 2, rcMvHalf  , piRefY2,  bRound   );
  
  m_pcRdCost->setCostScale( 0 );
  
  //  Quater-pel refinement
  Pel*  piSrcPel = cPatternRoi.getROIY() + (rcMvHalf.getHor() >> 1) + cPatternRoi.getPatternLStride() * (rcMvHalf.getVer() >> 1);
  Int*  piSrc    = m_piYuvExt  + ((m_iYuvExtStride + m_iDIFHalfTap) << 2) + (rcMvHalf.getHor() << 1) + m_iYuvExtStride * (rcMvHalf.getVer() << 1);
  piRef += (rcMvHalf.getHor() << 1) + iRefStride * (rcMvHalf.getVer() << 1);
  xExtDIFUpSamplingQ_TEN ( pcPatternKey, piRef, iRefStride, piSrcPel, cPatternRoi.getPatternLStride(), piSrc, m_iYuvExtStride, m_puiDFilter[rcMvHalf.getHor()+rcMvHalf.getVer()*3] );
  
  rcMvQter = *pcMvInt;   rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost = xPatternRefinement_Bi( pcPatternKey, piRef, iRefStride, 4, 1, rcMvQter , piRefY2, bRound  );
}
#endif

#if HHI_INTERP_FILTER
#ifdef QC_AMVRES
Void TEncSearch::xPatternSearchFracMOMS_Bi( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost, InterpFilterType ePFilt ,TComMv *PredMv , Int iRefIdxPred , Pel* piRefY2, Bool bRound)
#else
Void TEncSearch::xPatternSearchFracMOMS_Bi( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost, InterpFilterType ePFilt , Pel* piRefY2, Bool bRound)
#endif
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern( piRefY +  iOffset,
                          NULL,
                          NULL,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride,
                          0, 0, 0, 0 );
  
  Int   iBufStride  = TComPredFilterMOMS::getTmpStride();
  Pel*  piBuf       = TComPredFilterMOMS::getTmpLumaAddr() + ((iBufStride + 1) << 2);
  
  //  Half-pel refinement
  m_pcRdCost->setCostScale( 1 );
  TComPredFilterMOMS::setFiltType( ePFilt );
  TComPredFilterMOMS::extMOMSUpSamplingH ( &cPatternRoi );
  
  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  ruiCost = xPatternRefinement_Bi( pcPatternKey, piBuf, iBufStride, 4, 2, rcMvHalf  , piRefY2, bRound );
  
  //  Quarter-pel refinement
  m_pcRdCost->setCostScale( 0 );
  TComPredFilterMOMS::extMOMSUpSamplingQ ( &cPatternRoi, rcMvHalf, piBuf, iBufStride );
  
  rcMvQter = *pcMvInt;   rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  piBuf += (rcMvHalf.getHor() << 1) + iBufStride * (rcMvHalf.getVer() << 1);
  ruiCost = xPatternRefinement_Bi( pcPatternKey, piBuf, iBufStride, 4, 1, rcMvQter , piRefY2, bRound);
  
#ifdef QC_AMVRES
  if (pcCU->getSlice()->getSPS()->getUseAMVRes())
  {
    if (pcCU->getSlice()->getSymbolMode()==0 && iRefIdxPred  !=0 )
    {
      rcMvQter <<=1;
    }
    else
    {
      Pel* piRef = piRefY; 
      TComMv rcMv_HAM=*pcMvInt; rcMv_HAM<<= 1;  
      rcMv_HAM += rcMvHalf;  rcMv_HAM <<= 1;
      rcMv_HAM += rcMvQter; 
      ruiCost   = xPatternRefinementHAM_MOMS_Bi( pcPatternKey, piRef, iRefStride, 4, rcMv_HAM,ruiCost,ePFilt ,PredMv ,  piRefY2,  bRound);
      rcMvQter <<=1;
      rcMvQter+=rcMv_HAM;
    }
  }
#endif
}
#endif
#endif

#ifdef QC_AMVRES
Void TEncSearch::xPatternSearchFracDIF( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost,TComMv *PredMv, Int iRefIdxPred )
#else
Void TEncSearch::xPatternSearchFracDIF( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost )
#endif
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern( piRefY +  iOffset,
                          NULL,
                          NULL,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride,
                          0, 0, 0, 0 );
  Pel*  piRef;
#ifdef QC_AMVRES
  Int iRefStride_HAM	= iRefStride;
#endif
  
  iRefStride  = m_cYuvExt.getStride();
  
  //  Half-pel refinement
  xExtDIFUpSamplingH ( &cPatternRoi, &m_cYuvExt );
  piRef = m_cYuvExt.getLumaAddr() + ((iRefStride + m_iDIFHalfTap) << 2);
  
  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  ruiCost = xPatternRefinement( pcPatternKey, piRef, iRefStride, 4, 2, rcMvHalf   );
  
  m_pcRdCost->setCostScale( 0 );
  
  //  Quater-pel refinement
  Pel*  piSrcPel = cPatternRoi.getROIY() + (rcMvHalf.getHor() >> 1) + cPatternRoi.getPatternLStride() * (rcMvHalf.getVer() >> 1);
  Int*  piSrc    = m_piYuvExt  + ((m_iYuvExtStride + m_iDIFHalfTap) << 2) + (rcMvHalf.getHor() << 1) + m_iYuvExtStride * (rcMvHalf.getVer() << 1);
  piRef += (rcMvHalf.getHor() << 1) + iRefStride * (rcMvHalf.getVer() << 1);
  xExtDIFUpSamplingQ ( pcPatternKey, piRef, iRefStride, piSrcPel, cPatternRoi.getPatternLStride(), piSrc, m_iYuvExtStride, m_puiDFilter[rcMvHalf.getHor()+rcMvHalf.getVer()*3] );
  
  rcMvQter = *pcMvInt;   rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost = xPatternRefinement( pcPatternKey, piRef, iRefStride, 4, 1, rcMvQter );
  
#ifdef QC_AMVRES
  if (pcCU->getSlice()->getSPS()->getUseAMVRes())
  {
    if (pcCU->getSlice()->getSymbolMode()==0 && iRefIdxPred !=0 )
    {
      rcMvQter <<=1;
    }
    else
    {
      piRef = piRefY; 
      TComMv rcMv_HAM=*pcMvInt; rcMv_HAM<<= 1;  
      rcMv_HAM += rcMvHalf;  rcMv_HAM <<= 1;
      rcMv_HAM += rcMvQter; 
      ruiCost   = xPatternRefinementHAM_DIF( pcPatternKey, piRef, iRefStride_HAM, 4, rcMv_HAM,ruiCost ,PredMv );
      rcMvQter <<=1;
      rcMvQter+=rcMv_HAM;
    }
  }
#endif
}


#ifdef QC_SIFO
#ifdef QC_AMVRES
Void TEncSearch::xPatternSearchFracDIF_QC( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost,TComMv *PredMv, Int iRefIdxPred )
#else
Void TEncSearch::xPatternSearchFracDIF_QC( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost )
#endif
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern( piRefY +  iOffset,
                          NULL,
                          NULL,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride,
                          0, 0, 0, 0 );
  Pel*  piRef;
  Pel* piRef_temp = piRefY; 
  Int iRefStride_temp = iRefStride;
#ifdef QC_AMVRES
  Int iRefStride_HAM	= iRefStride;
#endif
  
  iRefStride  = m_cYuvExt.getStride();
  
  //  Half-pel refinement
  xExtDIFUpSamplingH_QC ( &cPatternRoi, &m_cYuvExt );
  piRef = m_cYuvExt.getLumaAddr() + ((iRefStride + m_iDIFHalfTap) << 2);
  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    
  ruiCost = xPatternRefinement( pcPatternKey, piRef, iRefStride, 4, 2, rcMvHalf   );
  //  Quater-pel refinement on the fly
  m_pcRdCost->setCostScale( 0 ); 
  rcMvQter  = *pcMvInt;  rcMvQter <<= 1;    
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost  = xPatternRefinementMC( pcPatternKey, piRef_temp, iRefStride_temp, 4, 1, rcMvQter);
  
#ifdef QC_AMVRES
  if (pcCU->getSlice()->getSPS()->getUseAMVRes())
  {
    if (pcCU->getSlice()->getSymbolMode()==0 && iRefIdxPred !=0 )
    {
      rcMvQter <<=1;
    }
    else
    {
      piRef = piRefY; 
      TComMv rcMv_HAM=*pcMvInt; rcMv_HAM<<= 1;  
      rcMv_HAM += rcMvHalf;  rcMv_HAM <<= 1;
      rcMv_HAM += rcMvQter; 
      ruiCost   = xPatternRefinementHAM_DIF( pcPatternKey, piRef, iRefStride_HAM, 4, rcMv_HAM,ruiCost ,PredMv );
      rcMvQter <<=1;
      rcMvQter+=rcMv_HAM;
    }
  }
#endif
}

Void TEncSearch::xExtDIFUpSamplingH_QC ( TComPattern* pcPattern, TComYuv* pcYuvExt)
{
  Int   x, y;
  Int   iWidth      = pcPattern->getROIYWidth();
  Int   iHeight     = pcPattern->getROIYHeight();
  Int   iPatStride  = pcPattern->getPatternLStride();
  Int   iExtStride  = pcYuvExt ->getStride();
  Int*  piSrcY;
  Int*  piDstY;
  Pel*  piDstYPel;
  Pel*  piSrcYPel;
  
  Int List = getCurrList();
  Int RefFrame = getCurrRefFrame();
  Int Offset=0; Offset = getSIFOOffset(List,0,RefFrame);
  
  //  Copy integer-pel
  piSrcYPel = pcPattern->getROIY() - m_iDIFHalfTap - iPatStride;
  piDstY    = m_piYuvExt;//pcYuvExt->getLumaAddr();
  piDstYPel = pcYuvExt->getLumaAddr();
  for ( y = 0; y < iHeight + 2; y++ )
  {
    for ( x = 0; x < iWidth + m_iDIFTap; x++ )
    {
      piDstYPel[x << 2] = Clip(piSrcYPel[x] + Offset);
    }
    piSrcYPel +=  iPatStride;
    piDstY    += (m_iYuvExtStride << 2);
    piDstYPel += (iExtStride      << 2);
  }
  
  Int i,filterH,filterV,filterC;
  i=2;	filterH = getTabFilters(i,getSIFOFilter(i),0);
  i=8;	filterV = getTabFilters(i,getSIFOFilter(i),0);
  i=10;	filterC = getSIFOFilter(i);
  
  Int OffsetH=0;  OffsetH = getSIFOOffset(List, 2,RefFrame);
  Int OffsetV=0;  OffsetV = getSIFOOffset(List, 8,RefFrame);
  Int OffsetC=0;  OffsetC = getSIFOOffset(List,10,RefFrame);
  
  //vertical
  piSrcYPel = pcPattern->getROIY()    - iPatStride - m_iDIFHalfTap;
  piDstYPel = pcYuvExt->getLumaAddr() + (iExtStride<<1);
  xCTI_FilterHalfVer     (piSrcYPel, iPatStride,     1, iWidth + m_iDIFTap, iHeight + 1, iExtStride<<2, 4, piDstYPel, filterV, OffsetV);
  
  //horizontal
  piSrcYPel = pcPattern->getROIY()   -  iPatStride - 1;
  piDstYPel = pcYuvExt->getLumaAddr() + m_iDIFTap2 - 2;
  xCTI_FilterHalfHor (piSrcYPel, iPatStride,     1,  iWidth + 1, iHeight + 1,  iExtStride<<2, 4, piDstYPel, filterH, OffsetH);
  
  //center
#if USE_DIAGONAL_FILT==1
#if SIFO_DIF_COMPATIBILITY==1
  UInt DIF_filter_position = getNum_SIFOFilters() - getNum_AvailableFilters();
  if(filterC>=DIF_filter_position && m_iDIFTap==6)
  {
    piSrcYPel = pcPattern->getROIY()   -  iPatStride - 1;
    piDstYPel = pcYuvExt->getLumaAddr() + (iExtStride<<1) - 2;
    xCTI_FilterDIF_TEN (piSrcYPel, iPatStride, 1,  iWidth + 1, iHeight + 1, iExtStride<<2, 4, piDstYPel, 2, 2, filterC-DIF_filter_position);
    if(OffsetC)
    {
      for (Int y = 0; y < iHeight + 1; y++)
      {
        for ( Int x = 0; x < iWidth + 1; x++)
          piDstYPel[x*4] = Clip( piDstYPel[x*4] + OffsetC);
        piDstYPel += (iExtStride<<2);
      }
    }
    return;
  }
#else
  if(filterC==5 && m_iDIFTap==6)
  {
    piSrcYPel = pcPattern->getROIY()   -  iPatStride - 1;
    piDstYPel = pcYuvExt->getLumaAddr() + (iExtStride<<1) - 2;
    xCTI_FilterDIF_TEN (piSrcYPel, iPatStride, 1,  iWidth + 1, iHeight + 1, iExtStride<<2, 4, piDstYPel, 2, 2);
    if(OffsetC)
    {
      for (Int y = 0; y < iHeight + 1; y++)
      {
        for ( Int x = 0; x < iWidth + 1; x++)
          piDstYPel[x*4] = Clip( piDstYPel[x*4] + OffsetC);
        piDstYPel += (iExtStride<<2);
      }
    }
    return;
  }
#endif
#endif
  i=10;
  Int filterC_0 = getTabFilters(i,getSIFOFilter(i),0);
  Int filterC_1 = getTabFilters(i,getSIFOFilter(i),1);
  piSrcYPel = pcPattern->getROIY()    - iPatStride - m_iDIFHalfTap;
  piDstY    = m_piYuvExt              + (m_iYuvExtStride<<1);
  xCTI_FilterHalfVer     (piSrcYPel, iPatStride,     1, iWidth + m_iDIFTap, iHeight + 1, m_iYuvExtStride<<2, 4, piDstY, filterC_0, 0);
  piSrcY    = m_piYuvExt              + (m_iYuvExtStride<<1) + ((m_iDIFHalfTap-1) << 2);
  piDstYPel = pcYuvExt->getLumaAddr() + (iExtStride<<1)      + m_iDIFTap2 - 2;
  xCTI_FilterHalfHor       (piSrcY, m_iYuvExtStride<<2, 4, iWidth + 1, iHeight + 1,iExtStride<<2, 4, piDstYPel, filterC_1, OffsetC);
  
}

UInt TEncSearch::xPatternRefinementMC  ( TComPattern* pcPatternKey, Pel* piRef, Int iRefStride, Int iIntStep, Int iFrac, TComMv& rcMvFrac )
{
  UInt  uiDist;
  UInt  uiDistBest  = MAX_UINT;
  UInt  uiDirecBest = 0;
  
  Pel* piLumaExt = m_cYuvExt.getLumaAddr();
  Int iYuvExtStride = m_cYuvExt.getStride();
  
  m_pcRdCost->setDistParam( pcPatternKey, piLumaExt, iYuvExtStride, 1, m_cDistParam, m_pcEncCfg->getUseHADME() );
  
  TComMv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);
  
  UInt i = 0;
  Int search_pos=16;
  
  for ( i = 0; i < search_pos; i++)
  {
    TComMv cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;
    
    TComMv cMvQter = cMvTest;
    cMvQter <<= (iFrac-1);
    Pel* piOrg   = m_cDistParam.pOrg;
    Int iStrideOrg = m_cDistParam.iStrideOrg;
    
    xPredInterLumaBlk_SIFOApplyME(piRef, iRefStride, piLumaExt, iYuvExtStride, &cMvQter, pcPatternKey->getROIYWidth(), 
                                  pcPatternKey->getROIYHeight(), piOrg, iStrideOrg,  0,  0);
    
    uiDist = m_cDistParam.DistFunc( &m_cDistParam );
    
#if HHI_IMVP
    if ( m_pcEncCfg->getUseIMP() )
    {
#ifdef QC_AMVRES
      TComMv cMvPred;
      if(m_pcEncCfg->getUseAMVRes())
      {
        cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<iFrac, cMvTest.getVer()<<iFrac);
        cMvPred.scale_down();
      }
      else
        cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<(iFrac-1), cMvTest.getVer()<<(iFrac-1) );
      
      m_pcRdCost->setPredictor( cMvPred );
#else
      TComMv cMvPred = m_cMvPredMeasure.getMVPred( cMvTest.getHor()<<(iFrac-1), cMvTest.getVer()<<(iFrac-1) );
      m_pcRdCost->setPredictor( cMvPred );
#endif
    }
#endif
    
    uiDist += m_pcRdCost->getCost( cMvQter.getHor(), cMvQter.getVer() );
    
    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
    }
  }
  
  rcMvFrac = pcMvRefine[uiDirecBest];
  return uiDistBest;
}

Void TEncSearch::xAddSubFullPelOffset( TComPattern* pcPatternKey, Int iOffset, Bool Add)
{//SAD = [Org - (Ref+Offset)] = (Org - Offset - Ref);  ........(Org - Offset) is done here...
  if(Add) 
    iOffset = -iOffset; 
  
  Pel *piOrgY = pcPatternKey->getROIY();
  for(Int h = 0; h < pcPatternKey->getROIYHeight(); h++)
  {
    for(Int w = 0; w < pcPatternKey->getROIYWidth(); w++)
    {
      piOrgY[w] = piOrgY[w] + iOffset;      
    }
    piOrgY += pcPatternKey->getPatternLStride();
  }
}

#endif  //QC_SIFO


#if TEN_DIRECTIONAL_INTERP
Void TEncSearch::xPatternSearchFracDIF_TEN( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost )
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern( piRefY +  iOffset,
                          NULL,
                          NULL,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride,
                          0, 0, 0, 0 );
  Pel*  piRef;
  iRefStride  = m_cYuvExt.getStride();
  
  //  Half-pel refinement
  xExtDIFUpSamplingH_TEN ( &cPatternRoi, &m_cYuvExt );
  piRef = m_cYuvExt.getLumaAddr() + ((iRefStride + 0) << 2);
  
  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  ruiCost = xPatternRefinement( pcPatternKey, piRef, iRefStride, 4, 2, rcMvHalf   );
  
  m_pcRdCost->setCostScale( 0 );
  
  //  Quater-pel refinement
  Pel*  piSrcPel = cPatternRoi.getROIY() + (rcMvHalf.getHor() >> 1) + cPatternRoi.getPatternLStride() * (rcMvHalf.getVer() >> 1);
  Int*  piSrc    = m_piYuvExt  + ((m_iYuvExtStride + m_iDIFHalfTap) << 2) + (rcMvHalf.getHor() << 1) + m_iYuvExtStride * (rcMvHalf.getVer() << 1);
  piRef += (rcMvHalf.getHor() << 1) + iRefStride * (rcMvHalf.getVer() << 1);
  xExtDIFUpSamplingQ_TEN ( pcPatternKey, piRef, iRefStride, piSrcPel, cPatternRoi.getPatternLStride(), piSrc, m_iYuvExtStride, m_puiDFilter[rcMvHalf.getHor()+rcMvHalf.getVer()*3] );
  
  rcMvQter = *pcMvInt;   rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost = xPatternRefinement( pcPatternKey, piRef, iRefStride, 4, 1, rcMvQter );
}
#endif

#if HHI_INTERP_FILTER
#ifdef QC_AMVRES
Void TEncSearch::xPatternSearchFracMOMS( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost, InterpFilterType ePFilt ,TComMv *PredMv , Int iRefIdxPred )
#else
Void TEncSearch::xPatternSearchFracMOMS( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvInt, TComMv& rcMvHalf, TComMv& rcMvQter, UInt& ruiCost, InterpFilterType ePFilt )
#endif
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern( piRefY +  iOffset,
                          NULL,
                          NULL,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride,
                          0, 0, 0, 0 );
  
  Int   iBufStride  = TComPredFilterMOMS::getTmpStride();
  Pel*  piBuf       = TComPredFilterMOMS::getTmpLumaAddr() + ((iBufStride + 1) << 2);
  
  //  Half-pel refinement
  m_pcRdCost->setCostScale( 1 );
  TComPredFilterMOMS::setFiltType( ePFilt );
  TComPredFilterMOMS::extMOMSUpSamplingH ( &cPatternRoi );
  
  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  ruiCost = xPatternRefinement( pcPatternKey, piBuf, iBufStride, 4, 2, rcMvHalf   );
  
  //  Quarter-pel refinement
  m_pcRdCost->setCostScale( 0 );
  TComPredFilterMOMS::extMOMSUpSamplingQ ( &cPatternRoi, rcMvHalf, piBuf, iBufStride );
  
  rcMvQter = *pcMvInt;   rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  piBuf += (rcMvHalf.getHor() << 1) + iBufStride * (rcMvHalf.getVer() << 1);
  ruiCost = xPatternRefinement( pcPatternKey, piBuf, iBufStride, 4, 1, rcMvQter );
  
#ifdef QC_AMVRES
  if (pcCU->getSlice()->getSPS()->getUseAMVRes())
  {
    if (pcCU->getSlice()->getSymbolMode()==0 && iRefIdxPred  !=0 )
    {
      rcMvQter <<=1;
    }
    else
    {
      Pel* piRef = piRefY; 
      TComMv rcMv_HAM=*pcMvInt; rcMv_HAM<<= 1;  
      rcMv_HAM += rcMvHalf;  rcMv_HAM <<= 1;
      rcMv_HAM += rcMvQter; 
      ruiCost   = xPatternRefinementHAM_MOMS( pcPatternKey, piRef, iRefStride, 4, rcMv_HAM,ruiCost,ePFilt ,PredMv );
      rcMvQter <<=1;
      rcMvQter+=rcMv_HAM;
    }
  }
#endif
}
#endif

Void TEncSearch::predInterSkipSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv )
{
  SliceType eSliceType = pcCU->getSlice()->getSliceType();
  if ( eSliceType == I_SLICE )
    return;
  
  if ( eSliceType == P_SLICE && pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) > 0 )
  {
    pcCU->setInterDirSubParts( 1, 0, 0, pcCU->getDepth( 0 ) );
    
    TComMv cMv;
    TComMv cZeroMv;
    xEstimateMvPredAMVP( pcCU, pcOrgYuv, 0, REF_PIC_LIST_0, 0, cMv, (pcCU->getCUMvField( REF_PIC_LIST_0 )->getAMVPInfo()->iN > 0?  true:false) );
#ifdef QC_AMVRES
    if (pcCU->getSlice()->getSPS()->getUseAMVRes())
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField_AMVRes( cMv, 0, SIZE_2Nx2N, 0, 0, 0 );
    else
#endif
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMv, 0, SIZE_2Nx2N, 0, 0, 0 );
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd    ( cZeroMv, SIZE_2Nx2N, 0, 0, 0 );  //unnecessary
#ifdef QC_AMVRES
    if (pcCU->getSlice()->getSPS()->getUseAMVRes())
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField_AMVRes( cZeroMv, NOT_VALID, SIZE_2Nx2N, 0, 0, 0 );  //unnecessary
    else
#endif
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cZeroMv, NOT_VALID, SIZE_2Nx2N, 0, 0, 0 );  //unnecessary
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd    ( cZeroMv, SIZE_2Nx2N, 0, 0, 0 );  //unnecessary
    
    pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, 0, 0, pcCU->getDepth(0));
    pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_1, 0, 0, pcCU->getDepth(0));
    
    //  Motion compensation
#ifdef DCM_PBIC
    motionCompensation ( pcCU, rpcPredYuv );
#else
    motionCompensation ( pcCU, rpcPredYuv, REF_PIC_LIST_0 );
#endif

#ifdef DCM_PBIC 
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      TComIc cIc;

      xEstimateIcPredAICP( pcCU, pcOrgYuv, 0, REF_PIC_LIST_0, cIc, false );

      cIc.computeScaleOffset( REF_PIC_LIST_0 );
      pcCU->getCUIcField()->setAllIcField( cIc, SIZE_2Nx2N, 0, 0, 0 );

      motionCompensation ( pcCU, rpcPredYuv );
    }
#endif 
  }
  else if ( eSliceType == B_SLICE &&
           pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) > 0 &&
           pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) > 0  )
  {
    TComMv cMv;
    TComMv cZeroMv;
    
    if (pcCU->getInterDir(0)!=2)
    {
      xEstimateMvPredAMVP( pcCU, pcOrgYuv, 0, REF_PIC_LIST_0, 0, cMv, (pcCU->getCUMvField( REF_PIC_LIST_0 )->getAMVPInfo()->iN > 0?  true:false) );
#ifdef QC_AMVRES
      if (pcCU->getSlice()->getSPS()->getUseAMVRes())
        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField_AMVRes( cMv, 0, SIZE_2Nx2N, 0, 0, 0 );  
      else
#endif      
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMv, 0, SIZE_2Nx2N, 0, 0, 0 );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd    ( cZeroMv, SIZE_2Nx2N, 0, 0, 0 ); //unnecessary
    }
    
    if (pcCU->getInterDir(0)!=1)
    {
      xEstimateMvPredAMVP( pcCU, pcOrgYuv, 0, REF_PIC_LIST_1, 0, cMv, (pcCU->getCUMvField( REF_PIC_LIST_1 )->getAMVPInfo()->iN > 0?  true:false) );
#ifdef QC_AMVRES
      if (pcCU->getSlice()->getSPS()->getUseAMVRes())
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField_AMVRes( cMv, 0, SIZE_2Nx2N, 0, 0, 0 );
      else
#endif      
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMv, 0, SIZE_2Nx2N, 0, 0, 0 );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd    ( cZeroMv, SIZE_2Nx2N, 0, 0, 0 );  //unnecessary
    }
    
    motionCompensation ( pcCU, rpcPredYuv );

#ifdef DCM_PBIC 
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      TComIc cIc;
      UChar uhInterDir = pcCU->getInterDir(0);
      RefPicList eRefList = (uhInterDir == 3) ? REF_PIC_LIST_X : ( (uhInterDir == 2) ? REF_PIC_LIST_1 : REF_PIC_LIST_0);;

      xEstimateIcPredAICP( pcCU, pcOrgYuv, 0, eRefList, cIc, false );

      cIc.computeScaleOffset( eRefList );
      pcCU->getCUIcField()->setAllIcField( cIc,SIZE_2Nx2N, 0, 0, 0 );
      
      motionCompensation ( pcCU, rpcPredYuv );
    }
#endif
  }
  else
  {
    assert( 0 );
  }
  
  return;
}

#if HHI_MRG
#ifdef DCM_PBIC
Void TEncSearch::predInterMergeSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv,  TComMvField cMvFieldNeighbourToTest[2], TComIc cIcNeighbourToTest, UChar uhInterDirNeighbourToTest )
#else
Void TEncSearch::predInterMergeSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv,  TComMvField cMvFieldNeighbourToTest[2], UChar uhInterDirNeighbourToTest )
#endif
{
  SliceType eSliceType = pcCU->getSlice()->getSliceType();
  if ( eSliceType == I_SLICE )
    return;
  
  UChar uhDepth = pcCU->getDepth( 0 );
  pcCU->setInterDirSubParts( uhInterDirNeighbourToTest, 0, 0, uhDepth );
#ifdef QC_AMVRES
  if (pcCU->getSlice()->getSPS()->getUseAMVRes())
  {
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField_AMVRes( cMvFieldNeighbourToTest[ 0 ].getMv(), cMvFieldNeighbourToTest[ 0 ].getRefIdx(), SIZE_2Nx2N, 0, 0, 0 );
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField_AMVRes( cMvFieldNeighbourToTest[ 1 ].getMv(), cMvFieldNeighbourToTest[ 1 ].getRefIdx(), SIZE_2Nx2N, 0, 0, 0 );
  }
  else
  {
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbourToTest[ 0 ].getMv(), cMvFieldNeighbourToTest[ 0 ].getRefIdx(), SIZE_2Nx2N, 0, 0, 0 );
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbourToTest[ 1 ].getMv(), cMvFieldNeighbourToTest[ 1 ].getRefIdx(), SIZE_2Nx2N, 0, 0, 0 );
  }
#else
  pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbourToTest[ 0 ].getMv(), cMvFieldNeighbourToTest[ 0 ].getRefIdx(), SIZE_2Nx2N, 0, 0, 0 );
  pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbourToTest[ 1 ].getMv(), cMvFieldNeighbourToTest[ 1 ].getRefIdx(), SIZE_2Nx2N, 0, 0, 0 );
#endif

#ifdef DCM_PBIC
  if (pcCU->getSlice()->getSPS()->getUseIC())
  {
    RefPicList eRefList = (uhInterDirNeighbourToTest == 3) ? REF_PIC_LIST_X : ( (uhInterDirNeighbourToTest == 2) ? REF_PIC_LIST_1 : REF_PIC_LIST_0);;
    cIcNeighbourToTest.computeScaleOffset( eRefList ); 
    pcCU->getCUIcField()->setAllIcField( cIcNeighbourToTest, SIZE_2Nx2N, 0, 0, 0 );
  }
#endif

  motionCompensation ( pcCU, rpcPredYuv );
  
  return;
}
#endif

#if HHI_RQT
Void TEncSearch::encodeResAndCalcRdInterCU( TComDataCU* pcCU, TComYuv* pcYuvOrg, TComYuv* pcYuvPred, TComYuv*& rpcYuvResi, TComYuv*& rpcYuvResiBest, TComYuv*& rpcYuvRec, Bool bSkipRes )
#else
Void TEncSearch::encodeResAndCalcRdInterCU( TComDataCU* pcCU, TComYuv* pcYuvOrg, TComYuv* pcYuvPred, TComYuv*& rpcYuvResi, TComYuv*& rpcYuvRec, Bool bSkipRes )
#endif
{
  if ( pcCU->isIntra(0) )
  {
    return;
  }
  
  PredMode  ePredMode    = pcCU->getPredictionMode( 0 );
  Bool      bHighPass    = pcCU->getSlice()->getDepth() ? true : false;
  UInt      uiBits       = 0, uiBitsBest = 0;
  UInt      uiDistortion = 0, uiDistortionBest = 0;
  
  UInt      uiWidth      = pcCU->getWidth ( 0 );
  UInt      uiHeight     = pcCU->getHeight( 0 );
  
  //  No residual coding : SKIP mode
  if ( ePredMode == MODE_SKIP && bSkipRes )
  {
    rpcYuvResi->clear();
    
    pcYuvPred->copyToPartYuv( rpcYuvRec, 0 );
    
    uiDistortion = m_pcRdCost->getDistPart( rpcYuvRec->getLumaAddr(), rpcYuvRec->getStride(),  pcYuvOrg->getLumaAddr(), pcYuvOrg->getStride(),  uiWidth,      uiHeight      )
    + m_pcRdCost->getDistPart( rpcYuvRec->getCbAddr(),   rpcYuvRec->getCStride(), pcYuvOrg->getCbAddr(),   pcYuvOrg->getCStride(), uiWidth >> 1, uiHeight >> 1 )
    + m_pcRdCost->getDistPart( rpcYuvRec->getCrAddr(),   rpcYuvRec->getCStride(), pcYuvOrg->getCrAddr(),   pcYuvOrg->getCStride(), uiWidth >> 1, uiHeight >> 1 );
    
    if( m_bUseSBACRD )
      m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST]);
    
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeSkipFlag(pcCU, 0, true);
    
    if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) > 0 ) //if ( ref. frame list0 has at least 1 entry )
    {
      m_pcEntropyCoder->encodeMVPIdx( pcCU, 0, REF_PIC_LIST_0);
    }
    if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) > 0 ) //if ( ref. frame list1 has at least 1 entry )
    {
      m_pcEntropyCoder->encodeMVPIdx( pcCU, 0, REF_PIC_LIST_1);
    }
    
#ifdef DCM_PBIC
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      m_pcEntropyCoder->encodeICPIdx( pcCU, 0 );
    }
#endif

    uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    pcCU->getTotalBits()       = uiBits;
    pcCU->getTotalDistortion() = uiDistortion;
    pcCU->getTotalCost()       = m_pcRdCost->calcRdCost( uiBits, uiDistortion );
    
    if( m_bUseSBACRD )
      m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_TEMP_BEST]);
    
    pcCU->setCbfSubParts( 0, 0, 0, 0, pcCU->getDepth( 0 ) );
    pcCU->setTrIdxSubParts( 0, 0, pcCU->getDepth(0) );
    
    return;
  }
  
  //  Residual coding.
  UInt    uiQp, uiQpBest = 0, uiQpMin, uiQpMax;
  Double  dCost, dCostBest = MAX_DOUBLE;
  
  UInt uiTrMode, uiBestTrMode = 0;
  
  UInt uiTrLevel = 0;
  if( (pcCU->getWidth(0) > pcCU->getSlice()->getSPS()->getMaxTrSize()) )
  {
    while( pcCU->getWidth(0) > (pcCU->getSlice()->getSPS()->getMaxTrSize()<<uiTrLevel) ) uiTrLevel++;
  }
  UInt uiMinTrMode = pcCU->getSlice()->getSPS()->getMinTrDepth() + uiTrLevel;
  UInt uiMaxTrMode = pcCU->getSlice()->getSPS()->getMaxTrDepth() + uiTrLevel;
  
  Bool bSpecial = false;
  
  if (pcCU->getPartitionSize(0) >= SIZE_2NxnU && pcCU->getPartitionSize(0) <= SIZE_nRx2N && uiMinTrMode == 0 && uiMaxTrMode == 1)
  {
    uiMaxTrMode++;
    bSpecial = true;
  }
  
  while((uiWidth>>uiMaxTrMode) < (g_uiMaxCUWidth>>g_uiMaxCUDepth)) uiMaxTrMode--;
  
  uiQpMin      = bHighPass ? Min( MAX_QP, Max( MIN_QP, pcCU->getQP(0) - m_iMaxDeltaQP ) ) : pcCU->getQP( 0 );
  uiQpMax      = bHighPass ? Min( MAX_QP, Max( MIN_QP, pcCU->getQP(0) + m_iMaxDeltaQP ) ) : pcCU->getQP( 0 );
  
#if HHI_RQT
  if( pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() )
  {
    rpcYuvResi->subtract( pcYuvOrg, pcYuvPred, 0, uiWidth );
  }
#endif
  for ( uiQp = uiQpMin; uiQp <= uiQpMax; uiQp++ )
  {
#if HHI_RQT
    if( pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() )
    {
      pcCU->setQPSubParts( uiQp, 0, pcCU->getDepth(0) );
      dCost = 0.;
      uiBits = 0;
      uiDistortion = 0;
      if( m_bUseSBACRD )
      {
        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_CURR_BEST ] );
      }
      
#if HHI_RQT_ROOT
      UInt uiZeroDistortion = 0;
      xEstimateResidualQT( pcCU, 0, 0, rpcYuvResi,  pcCU->getDepth(0), dCost, uiBits, uiDistortion, &uiZeroDistortion );

      double dZeroCost = m_pcRdCost->calcRdCost( 0, uiZeroDistortion );
      if ( dZeroCost < dCost )
      {
        dCost        = dZeroCost;
        uiBits       = 0;
        uiDistortion = uiZeroDistortion;

        const UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> (pcCU->getDepth(0) << 1);
        ::memset( pcCU->getTransformIdx()      , 0, uiQPartNum * sizeof(UChar) );
        ::memset( pcCU->getCbf( TEXT_LUMA )    , 0, uiQPartNum * sizeof(UChar) );
        ::memset( pcCU->getCbf( TEXT_CHROMA_U ), 0, uiQPartNum * sizeof(UChar) );
        ::memset( pcCU->getCbf( TEXT_CHROMA_V ), 0, uiQPartNum * sizeof(UChar) );
        ::memset( pcCU->getCoeffY()            , 0, uiWidth * uiHeight * sizeof( TCoeff )      );
        ::memset( pcCU->getCoeffCb()           , 0, uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
        ::memset( pcCU->getCoeffCr()           , 0, uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
      }
      else
      {
        xSetResidualQTData( pcCU, 0, NULL, pcCU->getDepth(0), false );
      }
#else
#if HHI_RQT_FORCE_SPLIT_ACC2_PU
      xEstimateResidualQT( pcCU, 0, 0, rpcYuvResi,  pcCU->getDepth(0), dCost, uiBits, uiDistortion );
#else
      xEstimateResidualQT( pcCU, 0, rpcYuvResi,  pcCU->getDepth(0), dCost, uiBits, uiDistortion );
#endif
      xSetResidualQTData( pcCU, 0, NULL, pcCU->getDepth(0), false );
#endif

      if( m_bUseSBACRD )
      {
        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST] );
      }
#if 0 // check
      {
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeCoeff( pcCU, 0, pcCU->getDepth(0), pcCU->getWidth(0), pcCU->getHeight(0) );
        const UInt uiBitsForCoeff = m_pcEntropyCoder->getNumberOfWrittenBits();
        if( m_bUseSBACRD )
        {
          m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST] );
        }
        if( uiBitsForCoeff != uiBits )
          assert( 0 );
      }
#endif
      uiBits = 0;
      {
        TComYuv *pDummy = NULL;
        xAddSymbolBitsInter( pcCU, 0, 0, uiBits, pDummy, NULL, pDummy );
      }
      
      
      Double dExactCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );
      dCost = dExactCost;
      
      if ( dCost < dCostBest )
      {
#if HHI_RQT_ROOT
        if ( !pcCU->getQtRootCbf( 0 ) )
        {
          rpcYuvResiBest->clear();
        }
        else
        {
          xSetResidualQTData( pcCU, 0, rpcYuvResiBest, pcCU->getDepth(0), true );
        }
#else
        xSetResidualQTData( pcCU, 0, rpcYuvResiBest, pcCU->getDepth(0), true );
#endif
        
        if( uiQpMin != uiQpMax && uiQp != uiQpMax )
        {
          const UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> (pcCU->getDepth(0) << 1);
          ::memcpy( m_puhQTTempTrIdx, pcCU->getTransformIdx(),        uiQPartNum * sizeof(UChar) );
          ::memcpy( m_puhQTTempCbf[0], pcCU->getCbf( TEXT_LUMA ),     uiQPartNum * sizeof(UChar) );
          ::memcpy( m_puhQTTempCbf[1], pcCU->getCbf( TEXT_CHROMA_U ), uiQPartNum * sizeof(UChar) );
          ::memcpy( m_puhQTTempCbf[2], pcCU->getCbf( TEXT_CHROMA_V ), uiQPartNum * sizeof(UChar) );
          ::memcpy( m_pcQTTempCoeffY,  pcCU->getCoeffY(),  uiWidth * uiHeight * sizeof( TCoeff )      );
          ::memcpy( m_pcQTTempCoeffCb, pcCU->getCoeffCb(), uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
          ::memcpy( m_pcQTTempCoeffCr, pcCU->getCoeffCr(), uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
        }
        uiBitsBest       = uiBits;
        uiDistortionBest = uiDistortion;
        dCostBest        = dCost;
        uiQpBest         = uiQp;
        
        if( m_bUseSBACRD )
        {
          m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_TEMP_BEST ] );
        }
      }
    }
    else
    {
#endif
      for ( uiTrMode = uiMinTrMode; uiTrMode <= uiMaxTrMode; uiTrMode++ )
      {
        if (bSpecial && uiMaxTrMode == 2 && uiTrMode == 1)
          continue;
        
        pcCU->setTrIdxSubParts( uiTrMode, 0, pcCU->getDepth(0) );
        
        // coefficient clearing is needed in the loop
        ::memset( pcCU->getCoeffY(), 0, sizeof(TCoeff) * uiWidth * uiHeight );
        
        rpcYuvResi->subtract      ( pcYuvOrg, pcYuvPred, 0, uiWidth );
        
        if( m_bUseSBACRD )
          m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST] );
        
        xEncodeInterTexture( pcCU, uiQp, bHighPass,  rpcYuvResi, uiTrMode );
        rpcYuvRec->addClip ( pcYuvPred,              rpcYuvResi, 0,      uiWidth  );
        
        uiDistortion = m_pcRdCost->getDistPart( rpcYuvRec->getLumaAddr(), rpcYuvRec->getStride(),  pcYuvOrg->getLumaAddr(), pcYuvOrg->getStride(),  uiWidth,      uiHeight      )
        + m_pcRdCost->getDistPart( rpcYuvRec->getCbAddr(),   rpcYuvRec->getCStride(), pcYuvOrg->getCbAddr(),   pcYuvOrg->getCStride(), uiWidth >> 1, uiHeight >> 1 )
        + m_pcRdCost->getDistPart( rpcYuvRec->getCrAddr(),   rpcYuvRec->getCStride(), pcYuvOrg->getCrAddr(),   pcYuvOrg->getCStride(), uiWidth >> 1, uiHeight >> 1 );
        
        // here is code for subtract bcbp if cbp == 0
        
        uiBits = 0;
        
        if( m_bUseSBACRD )
        {
          m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST] );
        }
        
        xAddSymbolBitsInter( pcCU, uiQp, uiTrMode, uiBits, rpcYuvRec, pcYuvPred, rpcYuvResi );
        dCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );
        
        if ( dCost < dCostBest )
        {
          uiBitsBest       = uiBits;
          uiDistortionBest = uiDistortion;
          dCostBest        = dCost;
          uiQpBest         = uiQp;
          uiBestTrMode     = uiTrMode;
          
          if( m_bUseSBACRD )
            m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_TEMP_BEST]);
        }
      }
#if HHI_RQT
    }
#endif
  }
  
  assert ( dCostBest != MAX_DOUBLE );
  
#if HHI_RQT
  if( ! pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() )
  {
#endif
    rpcYuvResi->subtract      ( pcYuvOrg, pcYuvPred, 0, uiWidth );
    
    if( m_bUseSBACRD )
    {
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST] );
    }
    
#if HHI_RQT
    pcCU->setTrIdxSubParts( uiBestTrMode, 0, pcCU->getDepth(0) );
#endif
    xEncodeInterTexture( pcCU, uiQpBest, bHighPass, rpcYuvResi, uiBestTrMode );
    
    if( (pcCU->getCbf(0, TEXT_LUMA, 0) == 0) &&
       (pcCU->getCbf(0, TEXT_CHROMA_U, 0) == 0) &&
       (pcCU->getCbf(0, TEXT_CHROMA_V, 0) == 0) )
#if HHI_RQT
    {
      if( pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() )
      {
        assert( 0 ); // obsolete
        const UInt uiLog2CUSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()]+2 - pcCU->getDepth(0);
        const UInt uiLog2MaxTUSize = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize();
        uiBestTrMode = uiLog2MaxTUSize >= uiLog2CUSize ? 0 : uiLog2CUSize - uiLog2MaxTUSize;
      }
      else
      {
#endif
        uiBestTrMode = 0;
#if HHI_RQT
      }
    }
  }
  else if( uiQpMin != uiQpMax && uiQpBest != uiQpMax )
  {
    if( m_bUseSBACRD )
    {
      assert( 0 ); // check
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_TEMP_BEST ] );
    }
    // copy best cbf and trIdx to pcCU
    const UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> (pcCU->getDepth(0) << 1);
    ::memcpy( pcCU->getTransformIdx(),       m_puhQTTempTrIdx,  uiQPartNum * sizeof(UChar) );
    ::memcpy( pcCU->getCbf( TEXT_LUMA ),     m_puhQTTempCbf[0], uiQPartNum * sizeof(UChar) );
    ::memcpy( pcCU->getCbf( TEXT_CHROMA_U ), m_puhQTTempCbf[1], uiQPartNum * sizeof(UChar) );
    ::memcpy( pcCU->getCbf( TEXT_CHROMA_V ), m_puhQTTempCbf[2], uiQPartNum * sizeof(UChar) );
    ::memcpy( pcCU->getCoeffY(),  m_pcQTTempCoeffY,  uiWidth * uiHeight * sizeof( TCoeff )      );
    ::memcpy( pcCU->getCoeffCb(), m_pcQTTempCoeffCb, uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
    ::memcpy( pcCU->getCoeffCr(), m_pcQTTempCoeffCr, uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
  }
  rpcYuvRec->addClip ( pcYuvPred, pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() ? rpcYuvResiBest : rpcYuvResi, 0, uiWidth );
#else
  rpcYuvRec->addClip ( pcYuvPred,                 rpcYuvResi, 0,      uiWidth  );
#endif
  
#if HHI_RQT
  if( pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() )
  {
    // update with clipped distortion and cost (qp estimation loop uses unclipped values)
    uiDistortionBest = m_pcRdCost->getDistPart( rpcYuvRec->getLumaAddr(), rpcYuvRec->getStride(),  pcYuvOrg->getLumaAddr(), pcYuvOrg->getStride(),  uiWidth,      uiHeight      )
    + m_pcRdCost->getDistPart( rpcYuvRec->getCbAddr(),   rpcYuvRec->getCStride(), pcYuvOrg->getCbAddr(),   pcYuvOrg->getCStride(), uiWidth >> 1, uiHeight >> 1 )
    + m_pcRdCost->getDistPart( rpcYuvRec->getCrAddr(),   rpcYuvRec->getCStride(), pcYuvOrg->getCrAddr(),   pcYuvOrg->getCStride(), uiWidth >> 1, uiHeight >> 1 );
    dCostBest = m_pcRdCost->calcRdCost( uiBitsBest, uiDistortionBest );
  }
#endif
  
  pcCU->getTotalBits()       = uiBitsBest;
  pcCU->getTotalDistortion() = uiDistortionBest;
  pcCU->getTotalCost()       = dCostBest;
  
  if ( pcCU->isSkipped(0) )
  {
    uiBestTrMode = 0;
    pcCU->setCbfSubParts( 0, 0, 0, 0, pcCU->getDepth( 0 ) );
  }
  
#if HHI_RQT
  if( ! pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() )
#endif
    pcCU->setTrIdxSubParts( uiBestTrMode, 0, pcCU->getDepth(0) );
  pcCU->setQPSubParts( uiQpBest, 0, pcCU->getDepth(0) );
}

#if HHI_RQT
#if HHI_RQT_ROOT
      Void TEncSearch::xEstimateResidualQT( TComDataCU* pcCU, UInt uiQuadrant, UInt uiAbsPartIdx, TComYuv* pcResi, const UInt uiDepth, Double &rdCost, UInt &ruiBits, UInt &ruiDist, UInt *puiZeroDist )
#elif HHI_RQT_FORCE_SPLIT_ACC2_PU
      Void TEncSearch::xEstimateResidualQT( TComDataCU* pcCU, UInt uiQuadrant, UInt uiAbsPartIdx, TComYuv* pcResi, const UInt uiDepth, Double &rdCost, UInt &ruiBits, UInt &ruiDist )
#else
      Void TEncSearch::xEstimateResidualQT( TComDataCU* pcCU, UInt uiAbsPartIdx, TComYuv* pcResi, const UInt uiDepth, Double &rdCost, UInt &ruiBits, UInt &ruiDist )
#endif
{
  const UInt uiTrMode = uiDepth - pcCU->getDepth( 0 );
  
  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiLog2TrSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiDepth]+2;

#if HHI_RQT_FORCE_SPLIT_ACC2_PU
#if HHI_RQT_FORCE_SPLIT_NxN
  const Bool bNxNOK = pcCU->getPartitionSize( 0 ) == SIZE_NxN && uiTrMode > 0;
#else
  const Bool bNxNOK = pcCU->getPartitionSize( 0 ) == SIZE_NxN;
#endif
#if HHI_RQT_FORCE_SPLIT_RECT
  const Bool bSymmetricOK  = pcCU->getPartitionSize( 0 ) >= SIZE_2NxN  && pcCU->getPartitionSize( 0 ) < SIZE_NxN   && uiTrMode > 0;
#else
  const Bool bSymmetricOK  = pcCU->getPartitionSize( 0 ) >= SIZE_2NxN  && pcCU->getPartitionSize( 0 ) < SIZE_NxN;
#endif
#if HHI_RQT_FORCE_SPLIT_ASYM
  const Bool bAsymmetricOK = pcCU->getPartitionSize( 0 ) >= SIZE_2NxnU && pcCU->getPartitionSize( 0 ) <= SIZE_nRx2N && uiTrMode > 1;
  const Bool b2NxnUOK      = pcCU->getPartitionSize( 0 ) == SIZE_2NxnU && uiQuadrant > 1      && uiTrMode == 1;
  const Bool b2NxnDOK      = pcCU->getPartitionSize( 0 ) == SIZE_2NxnD && uiQuadrant < 2      && uiTrMode == 1;
  const Bool bnLx2NOK      = pcCU->getPartitionSize( 0 ) == SIZE_nLx2N &&  ( uiQuadrant & 1 ) && uiTrMode == 1;
  const Bool bnRx2NOK      = pcCU->getPartitionSize( 0 ) == SIZE_nRx2N && !( uiQuadrant & 1 ) && uiTrMode == 1;
  const Bool bNoForceSplit = pcCU->getPartitionSize( 0 ) == SIZE_2Nx2N || bSymmetricOK || bAsymmetricOK ||
                                                            b2NxnUOK || b2NxnDOK || bnLx2NOK || bnRx2NOK || bNxNOK;
#else
  const Bool bAsymmetricOK = pcCU->getPartitionSize( 0 ) >= SIZE_2NxnU && pcCU->getPartitionSize( 0 ) <= SIZE_nRx2N;
  const Bool bNoForceSplit = pcCU->getPartitionSize( 0 ) == SIZE_2Nx2N || bNxNOK || bSymmetricOK || bAsymmetricOK;
#endif
  const Bool bCheckFull    = bNoForceSplit && ( uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
#else
  const Bool bCheckFull    = ( uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
#endif

#if HHI_RQT_DEPTH || HHI_RQT_DISABLE_SUB
  const Bool bCheckSplit  = ( uiLog2TrSize >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
#else
  const Bool bCheckSplit   = ( uiLog2TrSize >  pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() );
#endif
  
  assert( bCheckFull || bCheckSplit );
  
  Bool  bCodeChroma   = true;
  UInt  uiTrModeC     = uiTrMode;
  UInt  uiLog2TrSizeC = uiLog2TrSize-1;
  if( uiLog2TrSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
  {
    uiLog2TrSizeC++;
    uiTrModeC    --;
    UInt  uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrModeC ) << 1 );
    bCodeChroma   = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
  }
  
  const UInt uiSetCbf = 1 << uiTrMode;
  // code full block
  Double dSingleCost = MAX_DOUBLE;
  UInt uiSingleBits = 0;
  UInt uiSingleDist = 0;
  UInt uiAbsSumY = 0, uiAbsSumU = 0, uiAbsSumV = 0;
  
  if( m_bUseSBACRD )
  {
    m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
  }
  
  if( bCheckFull )
  {
    assert( pcCU->getROTindex( 0 ) == pcCU->getROTindex( uiAbsPartIdx ) );
    const UInt uiNumCoeffPerAbsPartIdxIncrement = pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
    const UInt uiQTTempAccessLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    TCoeff *pcCoeffCurrY = m_ppcQTTempCoeffY [uiQTTempAccessLayer] +  uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx;
    TCoeff *pcCoeffCurrU = m_ppcQTTempCoeffCb[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
    TCoeff *pcCoeffCurrV = m_ppcQTTempCoeffCr[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
    
    pcCU->setTrIdxSubParts( uiDepth - pcCU->getDepth( 0 ), uiAbsPartIdx, uiDepth );
    if (m_pcEncCfg->getUseRDOQ())
    {
      m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, 1<< uiLog2TrSize, TEXT_LUMA );
    }
    m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), false, pcCU->getSlice()->getSliceType(), TEXT_LUMA );
#if DISABLE_ROT_LUMA_4x4_8x8
    m_pcTrQuant->transformNxN( pcCU, pcResi->getLumaAddr( uiAbsPartIdx ), pcResi->getStride (), pcCoeffCurrY, 1<< uiLog2TrSize,    1<< uiLog2TrSize,    uiAbsSumY, TEXT_LUMA,     uiAbsPartIdx, (1<< uiLog2TrSize) > 8 ? pcCU->getROTindex( 0 ) : 0 );
#else
    m_pcTrQuant->transformNxN( pcCU, pcResi->getLumaAddr( uiAbsPartIdx ), pcResi->getStride (), pcCoeffCurrY, 1<< uiLog2TrSize,    1<< uiLog2TrSize,    uiAbsSumY, TEXT_LUMA,     uiAbsPartIdx, pcCU->getROTindex( 0 ) );
#endif
    
    pcCU->setCbfSubParts( uiAbsSumY ? uiSetCbf : 0, TEXT_LUMA, uiAbsPartIdx, uiDepth );
    
    if( bCodeChroma )
    {
      if (m_pcEncCfg->getUseRDOQ())
      {
        m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, 1<<uiLog2TrSizeC, TEXT_CHROMA );
      }
      m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), false, pcCU->getSlice()->getSliceType(), TEXT_CHROMA );
      m_pcTrQuant->transformNxN( pcCU, pcResi->getCbAddr( uiAbsPartIdx ), pcResi->getCStride(), pcCoeffCurrU, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, uiAbsSumU, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getROTindex( 0 ) );
      m_pcTrQuant->transformNxN( pcCU, pcResi->getCrAddr( uiAbsPartIdx ), pcResi->getCStride(), pcCoeffCurrV, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, uiAbsSumV, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getROTindex( 0 ) );
      pcCU->setCbfSubParts( uiAbsSumU ? uiSetCbf : 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
      pcCU->setCbfSubParts( uiAbsSumV ? uiSetCbf : 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
    }
    
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_LUMA,     uiTrMode );
    m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrY, uiAbsPartIdx, 1<< uiLog2TrSize,    1<< uiLog2TrSize,    uiDepth, TEXT_LUMA,     false );
    const UInt uiSingleBitsY = m_pcEntropyCoder->getNumberOfWrittenBits();
    
    UInt uiSingleBitsU = 0;
    UInt uiSingleBitsV = 0;
    if( bCodeChroma )
    {
      m_pcEntropyCoder->encodeQtCbf   ( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiTrMode );
      m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrU, uiAbsPartIdx, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, uiDepth, TEXT_CHROMA_U, false );
      uiSingleBitsU = m_pcEntropyCoder->getNumberOfWrittenBits() - uiSingleBitsY;
      
      m_pcEntropyCoder->encodeQtCbf   ( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiTrMode );
      m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrV, uiAbsPartIdx, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, uiDepth, TEXT_CHROMA_V, false );
      uiSingleBitsV = m_pcEntropyCoder->getNumberOfWrittenBits() - ( uiSingleBitsY + uiSingleBitsU );
    }
    
    const UInt uiNumSamplesLuma = 1 << (uiLog2TrSize<<1);
    const UInt uiNumSamplesChro = 1 << (uiLog2TrSizeC<<1);
    
    ::memset( m_pTempPel, 0, sizeof( Pel ) * uiNumSamplesLuma ); // not necessary needed for inside of recursion (only at the beginning)
    
    UInt uiDistY = m_pcRdCost->getDistPart( m_pTempPel, 1<< uiLog2TrSize, pcResi->getLumaAddr( uiAbsPartIdx ), pcResi->getStride(), 1<< uiLog2TrSize, 1<< uiLog2TrSize ); // initialized with zero residual destortion
#if HHI_RQT_ROOT
    if ( puiZeroDist )
    {
      *puiZeroDist += uiDistY;
    }
#endif
    if( uiAbsSumY )
    {
      Pel *pcResiCurrY = m_pcQTTempTComYuv[uiQTTempAccessLayer].getLumaAddr( uiAbsPartIdx );
      m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), false, pcCU->getSlice()->getSliceType(), TEXT_LUMA );
#if QC_MDDT
      m_pcTrQuant->invtransformNxN( TEXT_LUMA, REG_DCT, pcResiCurrY, m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(),  pcCoeffCurrY, 1<< uiLog2TrSize,    1<< uiLog2TrSize,    pcCU->getROTindex( 0 ) );//this is for inter mode only
#else
#if DISABLE_ROT_LUMA_4x4_8x8
      m_pcTrQuant->invtransformNxN( pcResiCurrY, m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(),  pcCoeffCurrY, 1<< uiLog2TrSize,    1<< uiLog2TrSize,    (1<< uiLog2TrSize) > 8 ? pcCU->getROTindex( 0 ) : 0 );
#else
      m_pcTrQuant->invtransformNxN( pcResiCurrY, m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(),  pcCoeffCurrY, 1<< uiLog2TrSize,    1<< uiLog2TrSize,    pcCU->getROTindex( 0 ) );
#endif
#endif
      const UInt uiNonzeroDistY = m_pcRdCost->getDistPart( m_pcQTTempTComYuv[uiQTTempAccessLayer].getLumaAddr( uiAbsPartIdx ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(),
                                                          pcResi->getLumaAddr( uiAbsPartIdx ), pcResi->getStride(), 1<< uiLog2TrSize,    1<< uiLog2TrSize );
      const Double dSingleCostY = m_pcRdCost->calcRdCost( uiSingleBitsY, uiNonzeroDistY );
      const Double dNullCostY   = m_pcRdCost->calcRdCost( 0, uiDistY );
      if( dNullCostY < dSingleCostY )
      {
        uiAbsSumY = 0;
        ::memset( pcCoeffCurrY, 0, sizeof( TCoeff ) * uiNumSamplesLuma );
      }
      else
      {
        uiDistY = uiNonzeroDistY;
      }
    }
    
    if( !uiAbsSumY )
    {
      Pel *pcPtr =  m_pcQTTempTComYuv[uiQTTempAccessLayer].getLumaAddr( uiAbsPartIdx );
      const UInt uiStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride();
      for( UInt uiY = 0; uiY < 1<< uiLog2TrSize; ++uiY )
      {
        ::memset( pcPtr, 0, sizeof(Pel) << uiLog2TrSize );
        pcPtr += uiStride;
      }
    }
    
    UInt uiDistU = 0;
    UInt uiDistV = 0;
    if( bCodeChroma )
    {
      uiDistU = m_pcRdCost->getDistPart( m_pTempPel, 1<<uiLog2TrSizeC, pcResi->getCbAddr( uiAbsPartIdx ), pcResi->getCStride(), 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC ); // initialized with zero residual destortion
#if HHI_RQT_ROOT
      if ( puiZeroDist )
      {
        *puiZeroDist += uiDistU;
      }
#endif
      if( uiAbsSumU )
      {
        Pel *pcResiCurrU = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCbAddr( uiAbsPartIdx );
        m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), false, pcCU->getSlice()->getSliceType(), TEXT_CHROMA );
#if QC_MDDT
        m_pcTrQuant->invtransformNxN( TEXT_CHROMA, REG_DCT, pcResiCurrU, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(), pcCoeffCurrU, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, pcCU->getROTindex( 0 ) );
#else
        m_pcTrQuant->invtransformNxN( pcResiCurrU, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(), pcCoeffCurrU, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, pcCU->getROTindex( 0 ) );
#endif
        const UInt uiNonzeroDistU = m_pcRdCost->getDistPart( m_pcQTTempTComYuv[uiQTTempAccessLayer].getCbAddr( uiAbsPartIdx ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(),
                                                            pcResi->getCbAddr( uiAbsPartIdx ), pcResi->getCStride(), 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC );
        const Double dSingleCostU = m_pcRdCost->calcRdCost( uiSingleBitsU, uiNonzeroDistU );
        const Double dNullCostU   = m_pcRdCost->calcRdCost( 0, uiDistU );
        if( dNullCostU < dSingleCostU )
        {
          uiAbsSumU = 0;
          ::memset( pcCoeffCurrU, 0, sizeof( TCoeff ) * uiNumSamplesChro );
        }
        else
        {
          uiDistU = uiNonzeroDistU;
        }
      }
      if( !uiAbsSumU )
      {
        Pel *pcPtr =  m_pcQTTempTComYuv[uiQTTempAccessLayer].getCbAddr( uiAbsPartIdx );
        const UInt uiStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride();
        for( UInt uiY = 0; uiY < 1<<uiLog2TrSizeC; ++uiY )
        {
          ::memset( pcPtr, 0, sizeof(Pel) << uiLog2TrSizeC );
          pcPtr += uiStride;
        }
      }
      
      uiDistV = m_pcRdCost->getDistPart( m_pTempPel, 1<<uiLog2TrSizeC, pcResi->getCrAddr( uiAbsPartIdx ), pcResi->getCStride(), 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC ); // initialized with zero residual destortion
#if HHI_RQT_ROOT
      if ( puiZeroDist )
      {
        *puiZeroDist += uiDistV;
      }
#endif
      if( uiAbsSumV )
      {
        Pel *pcResiCurrV = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCrAddr  ( uiAbsPartIdx );
        if( !uiAbsSumU )
        {
          m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), false, pcCU->getSlice()->getSliceType(), TEXT_CHROMA );
        }
#if QC_MDDT
        m_pcTrQuant->invtransformNxN( TEXT_CHROMA, REG_DCT, pcResiCurrV, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(), pcCoeffCurrV, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, pcCU->getROTindex( 0 ) );
#else
        m_pcTrQuant->invtransformNxN( pcResiCurrV, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(), pcCoeffCurrV, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, pcCU->getROTindex( 0 ) );
#endif
        const UInt uiNonzeroDistV = m_pcRdCost->getDistPart( m_pcQTTempTComYuv[uiQTTempAccessLayer].getCrAddr( uiAbsPartIdx ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(),
                                                            pcResi->getCrAddr( uiAbsPartIdx ), pcResi->getCStride(), 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC );
        const Double dSingleCostV = m_pcRdCost->calcRdCost( uiSingleBitsV, uiNonzeroDistV );
        const Double dNullCostV   = m_pcRdCost->calcRdCost( 0, uiDistV );
        if( dNullCostV < dSingleCostV )
        {
          uiAbsSumV = 0;
          ::memset( pcCoeffCurrV, 0, sizeof( TCoeff ) * uiNumSamplesChro );
        }
        else
        {
          uiDistV = uiNonzeroDistV;
        }
      }
      if( !uiAbsSumV )
      {
        Pel *pcPtr =  m_pcQTTempTComYuv[uiQTTempAccessLayer].getCrAddr( uiAbsPartIdx );
        const UInt uiStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride();
        for( UInt uiY = 0; uiY < 1<<uiLog2TrSizeC; ++uiY )
        {
          ::memset( pcPtr, 0, sizeof(Pel) << uiLog2TrSizeC );
          pcPtr += uiStride;
        }
      }
    }
    pcCU->setCbfSubParts( uiAbsSumY ? uiSetCbf : 0, TEXT_LUMA, uiAbsPartIdx, uiDepth );
    if( bCodeChroma )
    {
      pcCU->setCbfSubParts( uiAbsSumU ? uiSetCbf : 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
      pcCU->setCbfSubParts( uiAbsSumV ? uiSetCbf : 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
    }
    
    if( m_bUseSBACRD )
    {
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    
    m_pcEntropyCoder->resetBits();
#if HHI_RQT_DEPTH || HHI_RQT_DISABLE_SUB
    if( uiLog2TrSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
#else	
    if( uiLog2TrSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
#endif
    {
      m_pcEntropyCoder->encodeTransformSubdivFlag( 0, uiDepth );
    }
    
#if HHI_RQT_CHROMA_CBF_MOD
    if( bCodeChroma )
    {
      m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiTrMode );
      m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiTrMode );
    }
#endif
    
    m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_LUMA,     uiTrMode );
    
#if HHI_RQT_CHROMA_CBF_MOD
    if( 0 )
#endif
      if( bCodeChroma )
      {
        m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiTrMode );
        m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiTrMode );
      }
    
    m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrY, uiAbsPartIdx, 1<< uiLog2TrSize,    1<< uiLog2TrSize,    uiDepth, TEXT_LUMA,     false );
    
    if( bCodeChroma )
    {
      m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrU, uiAbsPartIdx, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, uiDepth, TEXT_CHROMA_U, false );
      m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrV, uiAbsPartIdx, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, uiDepth, TEXT_CHROMA_V, false );
    }
    
    uiSingleBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    
    uiSingleDist = uiDistY + uiDistU + uiDistV;
    dSingleCost = m_pcRdCost->calcRdCost( uiSingleBits, uiSingleDist );
  }
  
  // code sub-blocks
  if( bCheckSplit )
  {
    if( m_bUseSBACRD && bCheckFull )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_TEST ] );
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    UInt uiSubdivDist = 0;
    UInt uiSubdivBits = 0;
    Double dSubdivCost = 0.0;
    
    const UInt uiQPartNumSubdiv = pcCU->getPic()->getNumPartInCU() >> ((uiDepth + 1 ) << 1);
    for( UInt ui = 0; ui < 4; ++ui )
    {
#if HHI_RQT_ROOT
      xEstimateResidualQT( pcCU, ui, uiAbsPartIdx + ui * uiQPartNumSubdiv, pcResi, uiDepth + 1, dSubdivCost, uiSubdivBits, uiSubdivDist, bCheckFull ? NULL : puiZeroDist );
#elif HHI_RQT_FORCE_SPLIT_ACC2_PU
      xEstimateResidualQT( pcCU, ui, uiAbsPartIdx + ui * uiQPartNumSubdiv, pcResi, uiDepth + 1, dSubdivCost, uiSubdivBits, uiSubdivDist );
#else
      xEstimateResidualQT( pcCU, uiAbsPartIdx + ui * uiQPartNumSubdiv, pcResi, uiDepth + 1, dSubdivCost, uiSubdivBits, uiSubdivDist );
#endif
    }
    
    UInt uiYCbf = 0;
    UInt uiUCbf = 0;
    UInt uiVCbf = 0;
    for( UInt ui = 0; ui < 4; ++ui )
    {
      uiYCbf |= pcCU->getCbf( uiAbsPartIdx + ui * uiQPartNumSubdiv, TEXT_LUMA,     uiTrMode + 1 );
      uiUCbf |= pcCU->getCbf( uiAbsPartIdx + ui * uiQPartNumSubdiv, TEXT_CHROMA_U, uiTrMode + 1 );
      uiVCbf |= pcCU->getCbf( uiAbsPartIdx + ui * uiQPartNumSubdiv, TEXT_CHROMA_V, uiTrMode + 1 );
    }
    for( UInt ui = 0; ui < 4 * uiQPartNumSubdiv; ++ui )
    {
      pcCU->getCbf( TEXT_LUMA     )[uiAbsPartIdx + ui] |= uiYCbf << uiTrMode;
      pcCU->getCbf( TEXT_CHROMA_U )[uiAbsPartIdx + ui] |= uiUCbf << uiTrMode;
      pcCU->getCbf( TEXT_CHROMA_V )[uiAbsPartIdx + ui] |= uiVCbf << uiTrMode;
    }
    
    if( m_bUseSBACRD )
    {
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    m_pcEntropyCoder->resetBits();
    
    xEncodeResidualQT( pcCU, uiAbsPartIdx, uiDepth, true,  TEXT_LUMA );
    xEncodeResidualQT( pcCU, uiAbsPartIdx, uiDepth, false, TEXT_LUMA );
    xEncodeResidualQT( pcCU, uiAbsPartIdx, uiDepth, false, TEXT_CHROMA_U );
    xEncodeResidualQT( pcCU, uiAbsPartIdx, uiDepth, false, TEXT_CHROMA_V );
    
    uiSubdivBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    dSubdivCost  = m_pcRdCost->calcRdCost( uiSubdivBits, uiSubdivDist );
    
    if( dSubdivCost < dSingleCost )
    {
      rdCost += dSubdivCost;
      ruiBits += uiSubdivBits;
      ruiDist += uiSubdivDist;
      return;
    }
    assert( bCheckFull );
    if( m_bUseSBACRD )
    {
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_TEST ] );
    }
  }
  rdCost += dSingleCost;
  ruiBits += uiSingleBits;
  ruiDist += uiSingleDist;
  
  pcCU->setTrIdxSubParts( uiTrMode, uiAbsPartIdx, uiDepth );
  
  pcCU->setCbfSubParts( uiAbsSumY ? uiSetCbf : 0, TEXT_LUMA, uiAbsPartIdx, uiDepth );
  if( bCodeChroma )
  {
    pcCU->setCbfSubParts( uiAbsSumU ? uiSetCbf : 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
    pcCU->setCbfSubParts( uiAbsSumV ? uiSetCbf : 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
  }
}

Void TEncSearch::xEncodeResidualQT( TComDataCU* pcCU, UInt uiAbsPartIdx, const UInt uiDepth, Bool bSubdivAndCbf, TextType eType )
{
  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiCurrTrMode = uiDepth - pcCU->getDepth( 0 );
  const UInt uiTrMode = pcCU->getTransformIdx( uiAbsPartIdx );
  
  const Bool bSubdiv = uiCurrTrMode != uiTrMode;
  
  const UInt uiLog2TrSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiDepth]+2;
#if HHI_RQT_DEPTH || HHI_RQT_DISABLE_SUB
  if( bSubdivAndCbf && uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() && uiLog2TrSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
#else
  if( bSubdivAndCbf && uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() && uiLog2TrSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
#endif
  {
    m_pcEntropyCoder->encodeTransformSubdivFlag( bSubdiv, uiDepth );
  }
  
#if HHI_RQT_CHROMA_CBF_MOD
  assert( pcCU->getPredictionMode(uiAbsPartIdx) != MODE_INTRA );
  if( bSubdivAndCbf && uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() )
  {
    const Bool bFirstCbfOfCU = uiLog2TrSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() || uiCurrTrMode == 0;
    if( bFirstCbfOfCU || uiLog2TrSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
    {
      if( bFirstCbfOfCU || pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiCurrTrMode - 1 ) )
      {
        m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiCurrTrMode );
      }
      if( bFirstCbfOfCU || pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiCurrTrMode - 1 ) )
      {
        m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiCurrTrMode );
      }
    }
    else if( uiLog2TrSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
    {
      assert( pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiCurrTrMode ) == pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiCurrTrMode - 1 ) );
      assert( pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiCurrTrMode ) == pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiCurrTrMode - 1 ) );
    }
  }
#endif
  
  if( !bSubdiv )
  {
    const UInt uiNumCoeffPerAbsPartIdxIncrement = pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
    assert( 16 == uiNumCoeffPerAbsPartIdxIncrement ); // check
    const UInt uiQTTempAccessLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    TCoeff *pcCoeffCurrY = m_ppcQTTempCoeffY [uiQTTempAccessLayer] +  uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx;
    TCoeff *pcCoeffCurrU = m_ppcQTTempCoeffCb[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
    TCoeff *pcCoeffCurrV = m_ppcQTTempCoeffCr[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
    
    Bool  bCodeChroma   = true;
    UInt  uiTrModeC     = uiTrMode;
    UInt  uiLog2TrSizeC = uiLog2TrSize-1;
    if( uiLog2TrSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
    {
      uiLog2TrSizeC++;
      uiTrModeC    --;
      UInt  uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrModeC ) << 1 );
      bCodeChroma   = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
    }
    
    if( bSubdivAndCbf )
    {
      m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_LUMA,     uiTrMode );
      
#if HHI_RQT_CHROMA_CBF_MOD
      if( 0 ) // do only if intra -> never
#endif
        if( bCodeChroma )
        {
          m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiTrMode );
          m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiTrMode );
        }
    }
    else
    {
      if( eType == TEXT_LUMA     && pcCU->getCbf( uiAbsPartIdx, TEXT_LUMA,     uiTrMode ) )
      {
        m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrY, uiAbsPartIdx, 1<< uiLog2TrSize,    1<< uiLog2TrSize,    uiDepth, TEXT_LUMA,     false );
      }
      if( bCodeChroma )
      {
        if( eType == TEXT_CHROMA_U && pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiTrMode ) )
        {
          m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrU, uiAbsPartIdx, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, uiDepth, TEXT_CHROMA_U, false );
        }
        if( eType == TEXT_CHROMA_V && pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiTrMode ) )
        {
          m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrV, uiAbsPartIdx, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC, uiDepth, TEXT_CHROMA_V, false );
        }
      }
    }
  }
  else
  {
    if( bSubdivAndCbf || pcCU->getCbf( uiAbsPartIdx, eType, uiCurrTrMode ) )
    {
      const UInt uiQPartNumSubdiv = pcCU->getPic()->getNumPartInCU() >> ((uiDepth + 1 ) << 1);
      for( UInt ui = 0; ui < 4; ++ui )
      {
        xEncodeResidualQT( pcCU, uiAbsPartIdx + ui * uiQPartNumSubdiv, uiDepth + 1, bSubdivAndCbf, eType );
      }
    }
  }
}

Void TEncSearch::xSetResidualQTData( TComDataCU* pcCU, UInt uiAbsPartIdx, TComYuv* pcResi, UInt uiDepth, Bool bSpatial )
{
  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiCurrTrMode = uiDepth - pcCU->getDepth( 0 );
  const UInt uiTrMode = pcCU->getTransformIdx( uiAbsPartIdx );
  
  if( uiCurrTrMode == uiTrMode )
  {
    const UInt uiLog2TrSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiDepth]+2;
    const UInt uiQTTempAccessLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    
    Bool  bCodeChroma   = true;
    UInt  uiTrModeC     = uiTrMode;
    UInt  uiLog2TrSizeC = uiLog2TrSize-1;
    if( uiLog2TrSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
    {
      uiLog2TrSizeC++;
      uiTrModeC    --;
      UInt  uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrModeC ) << 1 );
      bCodeChroma   = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
    }
    
    if( bSpatial )
    {
      m_pcQTTempTComYuv[uiQTTempAccessLayer].copyPartToPartLuma    ( pcResi, uiAbsPartIdx, 1<<uiLog2TrSize , 1<<uiLog2TrSize  );
      if( bCodeChroma )
      {
        m_pcQTTempTComYuv[uiQTTempAccessLayer].copyPartToPartChroma( pcResi, uiAbsPartIdx, 1<<uiLog2TrSizeC, 1<<uiLog2TrSizeC );
      }
    }
    else
    {
      UInt    uiNumCoeffPerAbsPartIdxIncrement = pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
      UInt    uiNumCoeffY = ( 1 << ( uiLog2TrSize << 1 ) );
      TCoeff* pcCoeffSrcY = m_ppcQTTempCoeffY [uiQTTempAccessLayer] +  uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx;
      TCoeff* pcCoeffDstY = pcCU->getCoeffY() + uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx;
      ::memcpy( pcCoeffDstY, pcCoeffSrcY, sizeof( TCoeff ) * uiNumCoeffY );
      if( bCodeChroma )
      {
        UInt    uiNumCoeffC = ( 1 << ( uiLog2TrSizeC << 1 ) );
        TCoeff* pcCoeffSrcU = m_ppcQTTempCoeffCb[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        TCoeff* pcCoeffSrcV = m_ppcQTTempCoeffCr[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        TCoeff* pcCoeffDstU = pcCU->getCoeffCb() + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        TCoeff* pcCoeffDstV = pcCU->getCoeffCr() + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        ::memcpy( pcCoeffDstU, pcCoeffSrcU, sizeof( TCoeff ) * uiNumCoeffC );
        ::memcpy( pcCoeffDstV, pcCoeffSrcV, sizeof( TCoeff ) * uiNumCoeffC );
      }
    }
  }
  else
  {
    const UInt uiQPartNumSubdiv = pcCU->getPic()->getNumPartInCU() >> ((uiDepth + 1 ) << 1);
    for( UInt ui = 0; ui < 4; ++ui )
    {
      xSetResidualQTData( pcCU, uiAbsPartIdx + ui * uiQPartNumSubdiv, pcResi, uiDepth + 1, bSpatial );
    }
  }
}
#endif

Void TEncSearch::xEncodeInterTexture ( TComDataCU*& rpcCU, UInt uiQp, Bool bHighPass, TComYuv*& rpcYuv, UInt uiTrMode )
{
  UInt    uiWidth, uiHeight, uiCWidth, uiCHeight, uiLumaTrMode, uiChromaTrMode;
  TCoeff* piCoeff = rpcCU->getCoeffY();
  Pel*    pResi;
  
  uiWidth    = rpcCU->getWidth ( 0 );
  uiHeight   = rpcCU->getHeight( 0 );
  uiCWidth   = uiWidth >>1;
  uiCHeight  = uiHeight>>1;
  
  rpcCU->convertTransIdx( 0, uiTrMode, uiLumaTrMode, uiChromaTrMode );
  
  m_pcTrQuant->setQPforQuant( uiQp, !bHighPass, rpcCU->getSlice()->getSliceType(), TEXT_LUMA );
  
  rpcCU->clearCbf(0, TEXT_LUMA,     rpcCU->getTotalNumPart());
  rpcCU->clearCbf(0, TEXT_CHROMA_U, rpcCU->getTotalNumPart());
  rpcCU->clearCbf(0, TEXT_CHROMA_V, rpcCU->getTotalNumPart());
  
  UChar indexROT =     rpcCU->getROTindex(0);
  
  // Luma   Y
  piCoeff = rpcCU->getCoeffY();
  
  xRecurTransformNxN( rpcCU, 0, rpcYuv->getLumaAddr(), 0, rpcYuv->getStride(), uiWidth, uiHeight, uiLumaTrMode, 0, piCoeff, TEXT_LUMA, indexROT );
  rpcCU->setCuCbfLuma( 0, uiLumaTrMode );
  piCoeff = rpcCU->getCoeffY(); pResi = rpcYuv->getLumaAddr();
  rpcYuv->clearLuma();
  m_pcTrQuant->invRecurTransformNxN( rpcCU, 0, TEXT_LUMA, pResi, 0, rpcYuv->getStride(), uiWidth, uiHeight, uiLumaTrMode, 0, piCoeff, indexROT );
  
  m_pcTrQuant->setQPforQuant( uiQp, !bHighPass, rpcCU->getSlice()->getSliceType(), TEXT_CHROMA );
  
  // Chroma Cb
  piCoeff = rpcCU->getCoeffCb();
  xRecurTransformNxN( rpcCU, 0, rpcYuv->getCbAddr(), 0, rpcYuv->getCStride(), uiCWidth, uiCHeight, uiChromaTrMode, 0, piCoeff, TEXT_CHROMA_U, indexROT );
  rpcCU->setCuCbfChromaUV( 0, uiChromaTrMode, TEXT_CHROMA_U );
  piCoeff = rpcCU->getCoeffCb(); pResi = rpcYuv->getCbAddr();
  rpcYuv->clearChromaUV(0);
  m_pcTrQuant->invRecurTransformNxN( rpcCU, 0, TEXT_CHROMA_U, pResi, 0, rpcYuv->getCStride(), uiCWidth, uiCHeight, uiChromaTrMode, 0, piCoeff, indexROT );
  
  // Chroma Cr
  piCoeff = rpcCU->getCoeffCr();
  xRecurTransformNxN( rpcCU, 0, rpcYuv->getCrAddr(), 0, rpcYuv->getCStride(), uiCWidth, uiCHeight, uiChromaTrMode, 0, piCoeff, TEXT_CHROMA_V, indexROT );
  rpcCU->setCuCbfChromaUV( 0, uiChromaTrMode, TEXT_CHROMA_V );
  piCoeff = rpcCU->getCoeffCr(); pResi = rpcYuv->getCrAddr();
  rpcYuv->clearChromaUV(1);
  m_pcTrQuant->invRecurTransformNxN( rpcCU, 0, TEXT_CHROMA_V, pResi, 0, rpcYuv->getCStride(), uiCWidth, uiCHeight, uiChromaTrMode, 0, piCoeff, indexROT );
}

Void TEncSearch::xRecurTransformNxN( TComDataCU* rpcCU, UInt uiAbsPartIdx, Pel* pcResidual, UInt uiAddr, UInt uiStride, UInt uiWidth, UInt uiHeight, UInt uiMaxTrMode, UInt uiTrMode, TCoeff*& rpcCoeff, TextType eType, Int indexROT )
{
  if ( uiTrMode == uiMaxTrMode )
  {
    UInt uiAbsSum;
    UInt uiCoeffOffset = uiWidth*uiHeight;
    
    if (m_pcEncCfg->getUseRDOQ())
      m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, uiWidth, eType );
    
    m_pcTrQuant->transformNxN( rpcCU, pcResidual + uiAddr, uiStride, rpcCoeff, uiWidth, uiHeight, uiAbsSum, eType, uiAbsPartIdx, indexROT );
    
    if ( !rpcCU->getSlice()->isIntra() )
    {
      if ( uiAbsSum )
      {
        UInt uiBits, uiDist, uiDistCC;
        Double fCost, fCostCC;
        
        Pel* pcResidualRec = m_pTempPel;
        
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeCoeffNxN( rpcCU, rpcCoeff, uiAbsPartIdx, uiWidth, uiHeight, rpcCU->getDepth( 0 ) + uiTrMode, eType, true );
        uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
#if QC_MDDT
        m_pcTrQuant->invtransformNxN( eType, REG_DCT, pcResidualRec, uiStride, rpcCoeff, uiWidth, uiHeight, indexROT);
#else
        m_pcTrQuant->invtransformNxN( pcResidualRec, uiStride, rpcCoeff, uiWidth, uiHeight, indexROT);
#endif
        //-- distortion when the coefficients are not cleared
        uiDist   = m_pcRdCost->getDistPart(pcResidualRec, uiStride, pcResidual+uiAddr, uiStride, uiWidth, uiHeight);
        
        //-- distortion when the coefficients are cleared
        memset(pcResidualRec, 0, sizeof(Pel)*uiHeight*uiStride);
        uiDistCC = m_pcRdCost->getDistPart(pcResidualRec, uiStride, pcResidual+uiAddr, uiStride, uiWidth, uiHeight);
        
        fCost   = m_pcRdCost->calcRdCost(uiBits, uiDist);
        fCostCC = m_pcRdCost->calcRdCost(0, uiDistCC);
        
        if ( fCostCC < fCost )
        {
          uiAbsSum = 0;
          memset(rpcCoeff, 0, sizeof(TCoeff)*uiCoeffOffset);
          rpcCU->setCbfSubParts( 0x00, eType, uiAbsPartIdx, rpcCU->getDepth( 0 ) + uiTrMode );
        }
      }
      else
      {
        rpcCU->setCbfSubParts( 0x00, eType, uiAbsPartIdx, rpcCU->getDepth( 0 ) + uiTrMode );
        memset(rpcCoeff, 0, sizeof(TCoeff)*uiCoeffOffset);
      }
    }
    else
    {
      if ( uiAbsSum )
      {
        rpcCU->setCbfSubParts( 1 << (uiTrMode), eType, uiAbsPartIdx, rpcCU->getDepth( 0 ) + uiTrMode );
      }
      else
      {
        rpcCU->setCbfSubParts( 0x00, eType, uiAbsPartIdx, rpcCU->getDepth( 0 ) + uiTrMode );
        memset(rpcCoeff, 0, sizeof(TCoeff)*uiCoeffOffset);
      }
    }
    
    rpcCoeff += uiCoeffOffset;
  }
  else
  {
    uiTrMode++;
    uiWidth  = uiWidth  >> 1;
    uiHeight = uiHeight >> 1;
    UInt uiQPartNum = rpcCU->getPic()->getNumPartInCU() >> ( ( rpcCU->getDepth(0)+uiTrMode ) << 1 );
    UInt uiAddrOffset = uiHeight * uiStride;
    xRecurTransformNxN( rpcCU, uiAbsPartIdx, pcResidual, uiAddr                         , uiStride, uiWidth, uiHeight, uiMaxTrMode, uiTrMode, rpcCoeff, eType, indexROT );
    uiAbsPartIdx += uiQPartNum;
    xRecurTransformNxN( rpcCU, uiAbsPartIdx, pcResidual, uiAddr + uiWidth               , uiStride, uiWidth, uiHeight, uiMaxTrMode, uiTrMode, rpcCoeff, eType, indexROT );
    uiAbsPartIdx += uiQPartNum;
    xRecurTransformNxN( rpcCU, uiAbsPartIdx, pcResidual, uiAddr + uiAddrOffset          , uiStride, uiWidth, uiHeight, uiMaxTrMode, uiTrMode, rpcCoeff, eType, indexROT );
    uiAbsPartIdx += uiQPartNum;
    xRecurTransformNxN( rpcCU, uiAbsPartIdx, pcResidual, uiAddr + uiAddrOffset + uiWidth, uiStride, uiWidth, uiHeight, uiMaxTrMode, uiTrMode, rpcCoeff, eType, indexROT );
  }
}

Void  TEncSearch::xAddSymbolBitsIntra( TComDataCU* pcCU, TCoeff* pCoeff, UInt uiPU, UInt uiQNumPart, UInt uiPartDepth, UInt uiNumPart, UInt uiMaxTrDepth, UInt uiTrDepth, UInt uiWidth, UInt uiHeight, UInt& ruiBits )
{
  UInt uiPartOffset = uiPU*uiQNumPart;
  m_pcEntropyCoder->resetBits();
  
  if( uiPU==0 )
  {
    if( !pcCU->getSlice()->isIntra() )
    {
      m_pcEntropyCoder->encodeSkipFlag( pcCU, 0, true );
#if HHI_MRG && !HHI_MRG_PU
      m_pcEntropyCoder->encodeMergeInfo( pcCU, 0, true );      
#endif
      m_pcEntropyCoder->encodePredMode( pcCU, 0, true );
    }
    
#if PLANAR_INTRA
    if( pcCU->isIntra( 0 ) )
    {
      m_pcEntropyCoder->encodePlanarInfo( pcCU, 0, true );
      
      if ( pcCU->getPlanarInfo(0, PLANAR_FLAG) )
      {
        ruiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
        return;
      }
    }
#endif
    
    //BugFix for 1603
    m_pcEntropyCoder->encodePartSize( pcCU, 0, pcCU->getDepth(0), true );
    
    for( UInt ui = 0; ui < uiNumPart; ui++ )
    {
      m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, (uiPU+ui)*uiQNumPart );
#if HHI_AIS
      //BB: intra ref. samples filtering flag
      m_pcEntropyCoder->encodeIntraFiltFlagLuma( pcCU, (uiPU+ui)*uiQNumPart );
#endif
    }
  }
  else
  {
    for( UInt ui = 0; ui < uiNumPart; ui++ )
    {
      m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, (uiPU+ui)*uiQNumPart );
#if HHI_AIS
      //BB: intra ref. samples filtering flag
      m_pcEntropyCoder->encodeIntraFiltFlagLuma( pcCU, (uiPU+ui)*uiQNumPart );
#endif
    }
  }
  
  m_pcEntropyCoder->encodeCbf( pcCU, uiPartOffset, TEXT_LUMA, uiTrDepth );
#if QC_MDDT//ADAPTIVE_SCAN//USE_INTRA_MDDT_bRD //setting bRD to be true changes the results
  g_bUpdateStats = false;
  m_pcEntropyCoder->encodeCoeff( pcCU, pCoeff, uiPartOffset, pcCU->getDepth(0)+uiPartDepth, uiWidth, uiHeight, uiMaxTrDepth, uiTrDepth, TEXT_LUMA );
#else
  m_pcEntropyCoder->encodeCoeff( pcCU, pCoeff, uiPartOffset, pcCU->getDepth(0)+uiPartDepth, uiWidth, uiHeight, uiMaxTrDepth, uiTrDepth, TEXT_LUMA );
#endif
  ruiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
}

Void  TEncSearch::xAddSymbolBitsInter( TComDataCU* pcCU, UInt uiQp, UInt uiTrMode, UInt& ruiBits, TComYuv*& rpcYuvRec, TComYuv*pcYuvPred, TComYuv*& rpcYuvResi )
{
  if ( pcCU->isSkipped( 0 ) )
  {
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeSkipFlag(pcCU, 0, true);
    ruiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    
    m_pcEntropyCoder->resetBits();
    if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) > 0 ) //if ( ref. frame list0 has at least 1 entry )
    {
      m_pcEntropyCoder->encodeMVPIdx( pcCU, 0, REF_PIC_LIST_0);
    }
    if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) > 0 ) //if ( ref. frame list1 has at least 1 entry )
    {
      m_pcEntropyCoder->encodeMVPIdx( pcCU, 0, REF_PIC_LIST_1);
    }
#ifdef DCM_PBIC
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      m_pcEntropyCoder->encodeICPIdx( pcCU, 0 );
    }
#endif
    ruiBits += m_pcEntropyCoder->getNumberOfWrittenBits();
  }
  else
  {
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeSkipFlag ( pcCU, 0, true );
#if HHI_MRG && !HHI_MRG_PU
    m_pcEntropyCoder->encodeMergeInfo( pcCU, 0, true );
#endif
    m_pcEntropyCoder->encodePredMode( pcCU, 0, true );
    m_pcEntropyCoder->encodePartSize( pcCU, 0, pcCU->getDepth(0), true );
    m_pcEntropyCoder->encodePredInfo( pcCU, 0, true );
    m_pcEntropyCoder->encodeCoeff   ( pcCU, 0, pcCU->getDepth(0), pcCU->getWidth(0), pcCU->getHeight(0) );
    
#if !USLESS_TR_CODE
    // ROT index
    if ( pcCU->getSlice()->getSPS()->getUseROT() )
    {
      m_pcEntropyCoder->encodeROTindex( pcCU, 0, pcCU->getDepth(0) );
    }
#endif
    ruiBits += m_pcEntropyCoder->getNumberOfWrittenBits();
  }
}


#ifdef DCM_PBIC
Int TEncSearch::xEstMvdIcdBits( TComDataCU *pcCU, TComMv* pcMvd, TComIc* pcIcd, RefPicList eRefList )
{
  Int iZeroPatt = 0;
  Int iIcParam[3];
  Int iCost;
  ContextModel* pcCtxModel = NULL;
  TComZeroTree* pcZTree;

  if( m_bUseSBACRD )
    m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST]);

  // AMVRES flag is not counted here as it would be a common cost to all candidates. Mvd is already expected to scaled accordingly.

  // Identify the non-zero components
  if (eRefList == REF_PIC_LIST_X)
  {
    iZeroPatt |= ( pcMvd[REF_PIC_LIST_0].getHor() == 0 ) ? 0 : 1;
    iZeroPatt |= ( pcMvd[REF_PIC_LIST_0].getVer() == 0 ) ? 0 : 2;
    iZeroPatt |= ( pcMvd[REF_PIC_LIST_1].getHor() == 0 ) ? 0 : 4;
    iZeroPatt |= ( pcMvd[REF_PIC_LIST_1].getVer() == 0 ) ? 0 : 8;

    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      pcIcd->getIcParam( iIcParam[0], iIcParam[1], iIcParam[2] );
      iZeroPatt |= ( iIcParam[0] == 0 ) ? 0 : 16;
      iZeroPatt |= ( iIcParam[1] == 0 ) ? 0 : 32;
      iZeroPatt |= ( iIcParam[2] == 0 ) ? 0 : 64;

      pcZTree    = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDICDBI);
      pcCtxModel = m_pcEntropyCoder->m_pcEntropyCoderIf->getZTreeCtx(IDX_ZTREE_MVDICDBI);
    }
    else
    {
      pcZTree    = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDBI);
      pcCtxModel = m_pcEntropyCoder->m_pcEntropyCoderIf->getZTreeCtx(IDX_ZTREE_MVDBI);
    }
  } 
  else
  {
    iZeroPatt |= ( pcMvd[eRefList].getHor() == 0 ) ? 0 : 1;
    iZeroPatt |= ( pcMvd[eRefList].getVer() == 0 ) ? 0 : 2;

    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      pcIcd->getIcParam( iIcParam[0], iIcParam[1], iIcParam[2] );
      iZeroPatt |= ( iIcParam[0] == 0 ) ? 0 : 4;
      assert ( iIcParam[1] == 0 );
      iZeroPatt |= ( iIcParam[2] == 0 ) ? 0 : 8;

      pcZTree    = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDICDUNI);
      pcCtxModel = m_pcEntropyCoder->m_pcEntropyCoderIf->getZTreeCtx(IDX_ZTREE_MVDICDUNI);
    }
    else
    {
      pcZTree    = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDUNI);
      pcCtxModel = m_pcEntropyCoder->m_pcEntropyCoderIf->getZTreeCtx(IDX_ZTREE_MVDUNI);
    }
  }

  // Cost to encode zeroflag and zerotree (if necessary)
  iCost = xcostBin( (iZeroPatt==0), m_pcEntropyCoder->m_pcEntropyCoderIf->getZTreeCtx(IDX_ZEROFLAG) );

  if (iZeroPatt != 0)
  {
    pcZTree->updateVal(iZeroPatt);
    iCost += xcostZTree( pcZTree, pcZTree->m_pcRoot, pcCtxModel );
  }
  iCost = ((iCost + (1<<14)) >> 15);

  //Cost to encode the non-zero components
  if ( (eRefList == REF_PIC_LIST_X) || (eRefList == REF_PIC_LIST_0) )
  {
    iCost += ( xGetMvdBits( pcMvd[REF_PIC_LIST_0] ) - 2 );
  }
  if ( (eRefList == REF_PIC_LIST_X) || (eRefList == REF_PIC_LIST_1) )
  {
    iCost += ( xGetMvdBits( pcMvd[REF_PIC_LIST_1] ) - 2 );
  }

  if (pcCU->getSlice()->getSPS()->getUseIC())
  {
    iCost += ( xGetIcdBits( *pcIcd ) - 3 );
  }

  return iCost;
}

Int TEncSearch::xcostZTree( TComZeroTree* pcZTree, TComZTNode* pcZTNode, ContextModel *pcCtxModel )
{
  Int iVal, iLval, iRval;
  Int iCost;

  iCost = 0;
  if (pcZTNode->IsLeaf() == false)
  {
    iLval = pcZTNode->m_pcLeft->m_iVal;
    iRval = pcZTNode->m_pcRight->m_iVal;

    iVal = iLval & iRval;
    iCost += xcostBin( iVal, (pcCtxModel == NULL) ? NULL : (pcCtxModel + 2*pcZTNode->m_id) );

    if (iVal == 0)
      iCost += xcostBin( iLval, (pcCtxModel == NULL) ? NULL : (pcCtxModel + 2*pcZTNode->m_id + 1) );

    if (iLval != 0)
      iCost += xcostZTree( pcZTree,  pcZTNode->m_pcLeft, pcCtxModel);
    if (iRval != 0)
      iCost += xcostZTree( pcZTree, pcZTNode->m_pcRight, pcCtxModel);
  }
  return iCost;
}

Int TEncSearch::xcostBin( UInt uiBin, ContextModel *pcCtxModel )
{
  if (pcCtxModel == NULL)
    return 32768;

  uiBin = (uiBin != 0);
  Int iState = (uiBin == pcCtxModel->getMps() ) ? 64 + pcCtxModel->getState() : 63 - pcCtxModel->getState();

  return entropyBits[127-iState];
}
#endif

Void TEncSearch::xExtDIFUpSamplingH ( TComPattern* pcPattern, TComYuv* pcYuvExt  )
{
  Int   x, y;
  
  Int   iWidth      = pcPattern->getROIYWidth();
  Int   iHeight     = pcPattern->getROIYHeight();
  
  Int   iPatStride  = pcPattern->getPatternLStride();
  Int   iExtStride  = pcYuvExt ->getStride();
  
  Int*  piSrcY;
  Int*  piDstY;
  Pel*  piDstYPel;
  Pel*  piSrcYPel;
  
  //  Copy integer-pel
  piSrcYPel = pcPattern->getROIY() - m_iDIFHalfTap - iPatStride;
  piDstY    = m_piYuvExt;//pcYuvExt->getLumaAddr();
  piDstYPel = pcYuvExt->getLumaAddr();
  for ( y = 0; y < iHeight + 2; y++ )
  {
    for ( x = 0; x < iWidth + m_iDIFTap; x++ )
    {
      piDstYPel[x << 2] = piSrcYPel[x];
    }
    piSrcYPel +=  iPatStride;
    piDstY    += (m_iYuvExtStride << 2);
    piDstYPel += (iExtStride      << 2);
  }
  
  //  Half-pel NORM. : vertical
  piSrcYPel = pcPattern->getROIY()    - iPatStride - m_iDIFHalfTap;
  piDstY    = m_piYuvExt              + (m_iYuvExtStride<<1);
  piDstYPel = pcYuvExt->getLumaAddr() + (iExtStride<<1);
  xCTI_FilterHalfVer     (piSrcYPel, iPatStride,     1, iWidth + m_iDIFTap, iHeight + 1, m_iYuvExtStride<<2, 4, piDstY, iExtStride<<2, piDstYPel);
  
  //  Half-pel interpolation : horizontal
  piSrcYPel = pcPattern->getROIY()   -  iPatStride - 1;
  piDstYPel = pcYuvExt->getLumaAddr() + m_iDIFTap2 - 2;
  xCTI_FilterHalfHor (piSrcYPel, iPatStride,     1,  iWidth + 1, iHeight + 1,  iExtStride<<2, 4, piDstYPel);
  
  //  Half-pel interpolation : center
  piSrcY    = m_piYuvExt              + (m_iYuvExtStride<<1) + ((m_iDIFHalfTap-1) << 2);
  piDstYPel = pcYuvExt->getLumaAddr() + (iExtStride<<1)      + m_iDIFTap2 - 2;
  xCTI_FilterHalfHor       (piSrcY, m_iYuvExtStride<<2, 4, iWidth + 1, iHeight + 1,iExtStride<<2, 4, piDstYPel);
  
}

Void TEncSearch::xExtDIFUpSamplingQ   ( TComPattern* pcPatternKey, Pel* piDst, Int iDstStride, Pel* piSrcPel, Int iSrcPelStride, Int* piSrc, Int iSrcStride, UInt uiFilter )
{
  Int   x, y;
  
  Int   iWidth      = pcPatternKey->getROIYWidth();
  Int   iHeight     = pcPatternKey->getROIYHeight();
  
  Int*  piSrcY;
  Int*  piDstY;
  Pel*  piDstYPel;
  Pel*  piSrcYPel;
  
  Int iSrcStride4 = (iSrcStride<<2);
  Int iDstStride4 = (iDstStride<<2);
  
  switch (uiFilter)
  {
    case 0:
    {
      //  Quater-pel interpolation : vertical
      piSrcYPel = piSrcPel - m_iDIFHalfTap + 1;
      piDstY    = piSrc - m_iDIFTap2 + 2 - iSrcStride;
      xCTI_FilterQuarter0Ver(piSrcYPel, iSrcPelStride, 1, iWidth + m_iDIFTap - 1, iHeight, iSrcStride4, 4, piDstY);
      
      piSrcYPel = piSrcPel - m_iDIFHalfTap + 1;
      piDstY    = piSrc - m_iDIFTap2 + 2 + iSrcStride;
      xCTI_FilterQuarter1Ver(piSrcYPel, iSrcPelStride, 1, iWidth + m_iDIFTap - 1, iHeight, iSrcStride4, 4, piDstY);
      
      // Above three pixels
      piSrcY    = piSrc-2 - iSrcStride;
      piDstYPel = piDst-1 - iDstStride;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-2 - iSrcStride;
      piDstYPel = piDst   - iDstStride;;
      xCTI_FilterHalfHor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-2 - iSrcStride;
      piDstYPel = piDst+1 - iDstStride;
      xCTI_FilterQuarter1Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      // Middle two pixels
      piSrcY    = piSrc-2;
      piDstYPel = piDst-1;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-2;
      piDstYPel = piDst+1;
      xCTI_FilterQuarter1Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      // Below three pixels
      piSrcY    = piSrc-2 + iSrcStride;
      piDstYPel = piDst-1 + iDstStride;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-2 + iSrcStride;
      piDstYPel = piDst   + iDstStride;;
      xCTI_FilterHalfHor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-2 + iSrcStride;
      piDstYPel = piDst+1 + iDstStride;
      xCTI_FilterQuarter1Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      break;
    }
    case 1:
    {
      //  Quater-pel interpolation : vertical
      piSrcYPel = piSrcPel - m_iDIFHalfTap;
      piDstY    = piSrc-m_iDIFTap2 - iSrcStride;
      xCTI_FilterQuarter0Ver(piSrcYPel, iSrcPelStride, 1, iWidth + m_iDIFTap, iHeight, iSrcStride4, 4, piDstY);
      
      piSrcYPel = piSrcPel - m_iDIFHalfTap;
      piDstY    = piSrc-m_iDIFTap2 + iSrcStride;
      xCTI_FilterQuarter1Ver(piSrcYPel, iSrcPelStride, 1, iWidth + m_iDIFTap, iHeight, iSrcStride4, 4, piDstY);
      
      // Left three pixels
      piSrcY    = piSrc-4 - iSrcStride;
      piDstYPel = piDst-1 - iDstStride;
      xCTI_FilterQuarter1Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-4;
      piDstYPel = piDst-1;
      xCTI_FilterQuarter1Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-4 + iSrcStride;
      piDstYPel = piDst-1 + iDstStride;
      xCTI_FilterQuarter1Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      // Middle two pixels
      piSrcY    = piSrc - iSrcStride;
      piDstYPel = piDst - iDstStride;
      Int iSrcStride2 = (iSrcStride<<1);
      Int iDstStride2 = (iDstStride<<1);
      
      for (y=0; y < iHeight*2; y++)
      {
        for (x=0; x < iWidth; x++)
        {
          piDstYPel[x*4] = Clip( (piSrcY[x*4] +  128) >>  8 );
        }
        piSrcY+=iSrcStride2;
        piDstYPel+=iDstStride2;
      }
      
      // Right three pixels
      piSrcY    = piSrc   - iSrcStride;
      piDstYPel = piDst+1 - iDstStride;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc;
      piDstYPel = piDst+1;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc   + iSrcStride;
      piDstYPel = piDst+1 + iDstStride;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      
      // Right three pixels
      piSrcY    = piSrc   - iSrcStride;
      piDstYPel = piDst+1 - iDstStride;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc;
      piDstYPel = piDst+1;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc   + iSrcStride;
      piDstYPel = piDst+1 + iDstStride;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      // Middle two pixels
      piSrcYPel = piSrcPel;
      piDstYPel = piDst - iDstStride;
      xCTI_FilterQuarter0Ver(piSrcYPel, iSrcPelStride, 1, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcYPel = piSrcPel;
      piDstYPel = piDst + iDstStride;
      xCTI_FilterQuarter1Ver(piSrcYPel, iSrcPelStride, 1, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      break;
    }
    case 2:
    {
      //  Quater-pel interpolation : vertical
      piSrcYPel = piSrcPel - m_iDIFHalfTap + 1 - iSrcPelStride;;
      piDstY    = piSrc - m_iDIFTap2 + 2 - iSrcStride;
      xCTI_FilterQuarter1Ver(piSrcYPel, iSrcPelStride, 1, iWidth + m_iDIFTap - 1, iHeight, iSrcStride4, 4, piDstY);
      
      piSrcYPel = piSrcPel - m_iDIFHalfTap + 1;
      piDstY    = piSrc - m_iDIFTap2 + 2 + iSrcStride;
      xCTI_FilterQuarter0Ver(piSrcYPel, iSrcPelStride, 1, iWidth + m_iDIFTap - 1, iHeight, iSrcStride4, 4, piDstY);
      
      // Above three pixels
      piSrcY    = piSrc-2 - iSrcStride;
      piDstYPel = piDst-1 - iDstStride;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-2 - iSrcStride;
      piDstYPel = piDst   - iDstStride;;
      xCTI_FilterHalfHor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-2 - iSrcStride;
      piDstYPel = piDst+1 - iDstStride;
      xCTI_FilterQuarter1Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      // Middle two pixels
      piDstYPel = piDst - 1;
      xCTI_FilterQuarter0Hor(piSrcPel, iSrcPelStride, 1, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piDstYPel = piDst + 1;
      xCTI_FilterQuarter1Hor(piSrcPel, iSrcPelStride, 1, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      // Below three pixels
      piSrcY    = piSrc-2 + iSrcStride;
      piDstYPel = piDst-1 + iDstStride;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-2 + iSrcStride;
      piDstYPel = piDst   + iDstStride;;
      xCTI_FilterHalfHor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-2 + iSrcStride;
      piDstYPel = piDst+1 + iDstStride;
      xCTI_FilterQuarter1Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      break;
    }
    case 3:
    {
      //  Quater-pel interpolation : vertical
      piSrcYPel = piSrcPel-m_iDIFHalfTap - iSrcPelStride;
      piDstY    = piSrc-m_iDIFTap2 - iSrcStride;
      xCTI_FilterQuarter1Ver(piSrcYPel, iSrcPelStride, 1, iWidth + m_iDIFTap, iHeight, iSrcStride4, 4, piDstY);
      
      piSrcYPel = piSrcPel-m_iDIFHalfTap;
      piDstY    = piSrc-m_iDIFTap2 + iSrcStride;
      xCTI_FilterQuarter0Ver(piSrcYPel, iSrcPelStride, 1, iWidth + m_iDIFTap, iHeight, iSrcStride4, 4, piDstY);
      
      // Left three pixels
      piSrcY    = piSrc-4 - iSrcStride;
      piDstYPel = piDst-1 - iDstStride;
      xCTI_FilterQuarter1Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcYPel = piSrcPel-1;
      piDstYPel = piDst-1;
      xCTI_FilterQuarter1Hor(piSrcYPel, iSrcPelStride, 1, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc-4 + iSrcStride;
      piDstYPel = piDst-1 + iDstStride;
      xCTI_FilterQuarter1Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      // Middle two pixels
      piSrcY    = piSrc - iSrcStride;
      piDstYPel = piDst - iDstStride;
      Int iSrcStride2 = (iSrcStride<<1);
      Int iDstStride2 = (iDstStride<<1);
      
      for (y=0; y < iHeight*2; y++)
      {
        for (x=0; x < iWidth; x++)
        {
          piDstYPel[x*4] = Clip( (piSrcY[x*4] + 128) >>  8 );
        }
        piSrcY+=iSrcStride2;
        piDstYPel+=iDstStride2;
      }
      
      // Right three pixels
      piSrcY    = piSrc   - iSrcStride;
      piDstYPel = piDst+1 - iDstStride;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piDstYPel = piDst+1;
      xCTI_FilterQuarter0Hor(piSrcPel, iSrcPelStride, 1, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      piSrcY    = piSrc   + iSrcStride;
      piDstYPel = piDst+1 + iDstStride;
      xCTI_FilterQuarter0Hor(piSrcY, iSrcStride4, 4, iWidth, iHeight, iDstStride4, 4, piDstYPel);
      
      break;
    }
    default:
    {
      assert(0);
    }
  }
}

#if TEN_DIRECTIONAL_INTERP
Void TEncSearch::xExtDIFUpSamplingH_TEN ( TComPattern* pcPattern, TComYuv* pcYuvExt  )
{
  Int   x, y;
  
  Int   iWidth      = pcPattern->getROIYWidth();
  Int   iHeight     = pcPattern->getROIYHeight();
  
  Int   iPatStride  = pcPattern->getPatternLStride();
  Int   iExtStride  = pcYuvExt ->getStride();
  
  Pel*  piDstYPel;
  Pel*  piSrcYPel;
  
  //  Copy integer-pel
  piSrcYPel = pcPattern->getROIY();
  piDstYPel = pcYuvExt->getLumaAddr() + (iExtStride<<2);
  for ( y = 0; y < iHeight + 1; y++ )
  {
    for ( x = 0; x < iWidth; x++ )
    {
      piDstYPel[x << 2] = piSrcYPel[x];
    }
    piSrcYPel +=  iPatStride;
    piDstYPel += (iExtStride      << 2);
  }
  
  //  Half-pel NORM. : vertical
  piSrcYPel = pcPattern->getROIY()    - iPatStride;
  piDstYPel = pcYuvExt->getLumaAddr() + (iExtStride<<1);
  xCTI_FilterDIF_TEN (piSrcYPel, iPatStride, 1, iWidth, iHeight + 1, iExtStride<<2, 4, piDstYPel, 2, 0);
  
  //  Half-pel interpolation : horizontal
  piSrcYPel = pcPattern->getROIY() - 1;
  piDstYPel = pcYuvExt->getLumaAddr() + (iExtStride<<2) - 2;
  xCTI_FilterDIF_TEN (piSrcYPel, iPatStride, 1, iWidth + 1, iHeight, iExtStride<<2, 4, piDstYPel, 0, 2);
  
  
  //  Half-pel interpolation : center
  piSrcYPel = pcPattern->getROIY()   -  iPatStride - 1;
  piDstYPel = pcYuvExt->getLumaAddr() + (iExtStride<<1) - 2;
  xCTI_FilterDIF_TEN (piSrcYPel, iPatStride, 1,  iWidth + 1, iHeight + 1, iExtStride<<2, 4, piDstYPel, 2, 2);
}

Void TEncSearch::xExtDIFUpSamplingQ_TEN   ( TComPattern* pcPatternKey, Pel* piDst, Int iDstStride, Pel* piSrcPel, Int iSrcPelStride, Int* piSrc, Int iSrcStride, UInt uiFilter )
{
  Int   iWidth      = pcPatternKey->getROIYWidth();
  Int   iHeight     = pcPatternKey->getROIYHeight();
  
  Pel*  piDstYPel;
  Pel*  piSrcYPel;
  
  Int iDstStride4 = (iDstStride<<2);
  
  Int i,j,i0,j0;
  
  /* Position of quarter pixel search center relative to integer pixel grid in units of quarter pixels */
  Int ioff = 2 - (uiFilter&2);
  Int joff = 2 - 2*(uiFilter&1);
  
  /* Loop over quarter pixel search candidates,
   (i0,j0) is relative to quarter pixel search center
   (i,j) is relative to the closest upper left integer pixel */
  for (i0 = -1; i0 < 2; i0++){
    i = (i0 + ioff)&3;
    for (j0 = -1; j0 < 2; j0++){
      j = (j0 + joff)&3;
      piSrcYPel = piSrcPel - ((j-j0)>>2) - ((i-i0)>>2)*iSrcPelStride;
      piDstYPel = piDst + j0 + i0*iDstStride;
      if (i0 || j0)
        xCTI_FilterDIF_TEN (piSrcYPel, iSrcPelStride, 1, iWidth, iHeight, iDstStride4, 4, piDstYPel, i, j);
    }
  }
}
#endif

Void TEncSearch::xPredIntraLumaNxNCIPEnc( TComPattern* pcTComPattern, Pel* pOrig, Pel* pPredCL, UInt uiStride, Pel* pPred, UInt uiPredStride, Int iWidth, Int iHeight, TComDataCU* pcCU, Bool bAboveAvail, Bool bLeftAvail )
{
  Int*  ptrSrc;
  Int   sw, iWidth2;
  Int   x, y;
  
  // obtain source
  ptrSrc  = pcTComPattern->getAdiOrgBuf( iWidth, iHeight, m_piYuvExt );
  
  // obtain source stride
  iWidth2 = iWidth<<1;
  sw = iWidth2+1;
  ptrSrc += sw+1;
  
  // compute DC value for non-availability
  Int  iDC  = 0;
  Int  iCnt = 0;
  
  if ( !bAboveAvail && !bLeftAvail )
  {
    iDC = 128 << g_uiBitIncrement;
  }
  else
    if ( !bAboveAvail )
    {
      for ( y=0; y<iHeight; y++ )
      {
        iDC  += ptrSrc[-1+y*sw];
        iCnt ++;
      }
      iDC = ( iDC + iCnt/2 ) / iCnt;
    }
    else
      if ( !bLeftAvail )
      {
        for ( x=0; x<iWidth; x++ )
        {
          iDC  += ptrSrc[x+(-1)*sw];
          iCnt ++;
        }
        iDC = ( iDC + iCnt/2 ) / iCnt;
      }
      else
      {
        for ( y=0; y<iHeight; y++ )
        {
          iDC  += ptrSrc[-1+y*sw];
          iCnt ++;
        }
        for ( x=0; x<iWidth; x++ )
        {
          iDC  += ptrSrc[x+(-1)*sw];
          iCnt ++;
        }
        iDC = ( iDC + iCnt/2 ) / iCnt;
      }
  
  // update prediction for left-top corner
  if ( bAboveAvail && bLeftAvail )
  {
    pPred[ 0 ] = CIP_PRED( ptrSrc[-1], ptrSrc[-sw], ptrSrc[-1-sw] );
  }
  else
    if ( bAboveAvail )
    {
      pPred[ 0 ] = CIP_PRED( iDC, ptrSrc[-sw], iDC );
    }
    else
      if ( bLeftAvail )
      {
        pPred[ 0 ] = CIP_PRED( ptrSrc[-1], iDC, iDC );
      }
      else
      {
        pPred[ 0 ] = iDC;
      }
  
  // update prediction for top side
  if ( bAboveAvail )
  {
    for ( x=1; x<iWidth; x++ )
    {
      pPred[ x ] = CIP_PRED( pOrig[x-1], ptrSrc[x-sw], ptrSrc[x-1-sw] );
    }
  }
  else
  {
    for ( x=1; x<iWidth; x++ )
    {
      pPred[ x ] = CIP_PRED( pOrig[x-1], iDC, iDC );
    }
  }
  
  // update prediction for left side
  if ( bLeftAvail )
  {
    for ( y=1; y<iHeight; y++ )
    {
      pPred[ y*uiPredStride ] = CIP_PRED( ptrSrc[-1+y*sw], pOrig[(y-1)*uiStride], ptrSrc[-1+(y-1)*sw] );
    }
  }
  else
  {
    for ( y=1; y<iHeight; y++ )
    {
      pPred[ y*uiPredStride ] = CIP_PRED( iDC, pOrig[(y-1)*uiStride], iDC );
    }
  }
  
  // update prediction for inner region
  for ( y=1; y<iHeight; y++ )
  {
    for ( x=1; x<iWidth; x++ )
    {
      pPred[ x+y*uiPredStride ] = CIP_PRED( pOrig[ (x-1)+y*uiStride], pOrig[ x+(y-1)*uiStride ], pOrig[ (x-1)+(y-1)*uiStride ] );
    }
  }
}

#ifdef DCM_PBIC
Void TEncSearch::xEstimateIcPredAICP( TComDataCU* pcCU, TComYuv* pcOrgYuv, UInt uiPartIdx, RefPicList  eRefPicList, TComIc& rcIcPred, Bool bFilled )
{
  AICPInfo* pcAICPInfo = pcCU->getCUIcField()->getAICPInfo();

  TComIc  cBestIc;
  Int     iBestIdx = 0;
  TComIc  cZeroIc;
  TComIc  cIcTemp;
  UInt    uiBestCost = MAX_INT;
  UInt    uiPartAddr = 0;
  Int     iRoiWidth, iRoiHeight;
  Int     i;
  
  pcCU->getPartIndexAndSize( uiPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );

  // Fill the IC Candidates
  if (!bFilled)
  {
    pcCU->fillICPCand( uiPartIdx, uiPartAddr, pcAICPInfo );
  }

  // initialize Icp index & Icp
  iBestIdx = 0;
	cBestIc=pcAICPInfo->m_acIcCand[0];

	if( pcAICPInfo->iN <= 1 )
  {
    rcIcPred = cBestIc;

	  pcCU->setICPIdxSubParts( iBestIdx, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr) );
    pcCU->setICPNumSubParts( pcAICPInfo->iN, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    return;
  }

  if ( bFilled )
  {
    assert(pcCU->getICPIdx(uiPartAddr) >= 0);
    rcIcPred = pcAICPInfo->m_acIcCand[pcCU->getICPIdx(uiPartAddr)];
    return;
  }

  //-- Check Minimum Cost.
  for ( i = 0 ; i < pcAICPInfo->iN; i++)
  {
    UInt uiTmpCost;
    cIcTemp = pcAICPInfo->m_acIcCand[i];
		cIcTemp.computeScaleOffset( eRefPicList);

    uiTmpCost = xGetTemplateCostIC( pcCU, uiPartIdx, uiPartAddr, pcOrgYuv, cIcTemp, i, pcAICPInfo->iN, eRefPicList, iRoiWidth, iRoiHeight);
    if ( uiBestCost > uiTmpCost )
    {
      uiBestCost = uiTmpCost;
      cBestIc    = pcAICPInfo->m_acIcCand[i];
      iBestIdx   = i;
    }
  }

  // Setting Best ICP
	rcIcPred=cBestIc;

  pcCU->setICPIdxSubParts( iBestIdx, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr) );
  pcCU->setICPNumSubParts( pcAICPInfo->iN, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
 
  return;
	
}

UInt TEncSearch::xGetIcpIdxBits(Int iIdx, Int iNum)
{
  assert(iIdx >= 0 && iNum >= 0 && iIdx < iNum);

  if (iNum == 1)
    return 0;

  UInt uiLength = 1;
  Int iTemp = iIdx;
  if ( iTemp == 0 )
  {
    return uiLength;
  }

  Bool bCodeLast = ( iNum-1 > iTemp );

  uiLength += (iTemp-1);

  if( bCodeLast )
  {
    uiLength++;
  }

  return uiLength;
}

Void  TEncSearch::predICompSearch( TComDataCU* pcCU, TComYuv* pcYuvOrg, TComYuv* pcYuvPred, Int iPartIdx, Int uiLastMode )
{
  Int   iRoiWidth;
  Int   iRoiHeight;
  Int   iIcpIdxCur;
  Int   iIcpIdxBest;

  UInt  uiPartAddr;
  UInt  uiCost;
  UInt  uiCostBest = MAX_UINT;
  
  TComIc cIc; 
  TComIc cIcd;
  TComIc cIcLMSlist[2];
  TComIc cIcPred;
  TComIc ZeroIc;
  TComIc cIc_test;

  RefPicList eRefPicList;

  PartSize ePartSize = pcCU->getPartitionSize( 0 );
  pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );

  m_acYuvTempIC[0].clear();
  m_acYuvTempIC[1].clear();

  if( uiLastMode==0 || uiLastMode==1 )
  {
    eRefPicList = (uiLastMode ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    Calc_ICParam_LMS_Uni( pcYuvOrg, &m_acYuvPred[eRefPicList], uiPartAddr, iRoiHeight, iRoiWidth, cIcLMSlist, eRefPicList);
  }
  else
  {
    eRefPicList = REF_PIC_LIST_X;
    Calc_ICParam_LMS_Bi( pcYuvOrg, &m_acYuvPred[0], &m_acYuvPred[1], uiPartAddr, iRoiHeight, iRoiWidth, cIcLMSlist );
  }

  AICPInfo* pcAICPInfo = pcCU->getCUIcField()->getAICPInfo();
  pcCU->fillICPCand( iPartIdx, uiPartAddr, pcAICPInfo );

  // Test LMS candidates
  for(Int iLMSIdx = 0; iLMSIdx < 2; iLMSIdx++)
  {
    cIc_test = cIcLMSlist[iLMSIdx];
    uiCost = MAX_UINT;
    xSearchBestICParam(&cIc_test, iIcpIdxCur,  0,  0,  0, iPartIdx, pcCU, pcYuvOrg, eRefPicList, uiCost); //Just test the candidate - no search
    if ( uiCost < uiCostBest )
    {
      uiCostBest  = uiCost;
      cIc         = cIc_test;
      iIcpIdxBest = iIcpIdxCur;
    }
  }

  // Test every predictor as a candidate
  for(Int iIcpIdxTemp = 0; iIcpIdxTemp < pcAICPInfo->iN; iIcpIdxTemp++)
  {
    cIc_test = pcAICPInfo->m_acIcCand[iIcpIdxTemp];
    uiCost = MAX_UINT;
    xSearchBestICParam(&cIc_test, iIcpIdxCur,  0,  0,  0, iPartIdx, pcCU, pcYuvOrg, eRefPicList, uiCost); //Just test the candidate - no search
    if ( uiCost < uiCostBest )
    {
      uiCostBest  = uiCost;
      cIc         = cIc_test;
      iIcpIdxBest = iIcpIdxCur;
    }
  }

  if (uiCostBest == MAX_UINT)
  {
    cIc.reset();
    iIcpIdxBest = 0;
  }

  cIc.computeScaleOffset( eRefPicList );
  pcCU->getCUIcField()->setAllIcField( cIc, ePartSize, uiPartAddr, iPartIdx, 0);

  cIcPred = pcAICPInfo->m_acIcCand[iIcpIdxBest];

  cIcd.copyIcParam(cIc);
  cIcd.subIcParamPred(cIcPred);
  pcCU->getCUIcField()->setAllIcd    ( cIcd, ePartSize, uiPartAddr, iPartIdx, 0);

  pcCU->setICPIdxSubParts( iIcpIdxBest, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
  pcCU->setICPNumSubParts( pcAICPInfo->iN, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

  //Apply the best IC
  if (eRefPicList != REF_PIC_LIST_X)
  {
    xPredICompLumaBlk  ( &cIc, iRoiWidth   , iRoiHeight   , pcYuvPred->getStride() , 1, pcYuvPred->getLumaAddr(uiPartAddr), m_acYuvPred[eRefPicList].getStride() , 1, m_acYuvPred[eRefPicList].getLumaAddr(uiPartAddr), eRefPicList );
    xPredICompChromaBlk( &cIc, iRoiWidth>>1, iRoiHeight>>1, pcYuvPred->getCStride(), 1, pcYuvPred->getCbAddr(uiPartAddr)  , m_acYuvPred[eRefPicList].getCStride(), 1, m_acYuvPred[eRefPicList].getCbAddr(uiPartAddr)  , eRefPicList );
    xPredICompChromaBlk( &cIc, iRoiWidth>>1, iRoiHeight>>1, pcYuvPred->getCStride(), 1, pcYuvPred->getCrAddr(uiPartAddr)  , m_acYuvPred[eRefPicList].getCStride(), 1, m_acYuvPred[eRefPicList].getCrAddr(uiPartAddr)  , eRefPicList );
  }
  else
  {
    Int iRefIdx[2];
    iRefIdx[0] = pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr );
    iRefIdx[1] = pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr );

    xPredICompLumaBlk  ( &cIc, iRoiWidth   , iRoiHeight   , m_acYuvTempIC[0].getStride() , 1, m_acYuvTempIC[0].getLumaAddr(uiPartAddr), m_acYuvPred[0].getStride() , 1, m_acYuvPred[0].getLumaAddr(uiPartAddr), REF_PIC_LIST_0 );
    xPredICompChromaBlk( &cIc, iRoiWidth>>1, iRoiHeight>>1, m_acYuvTempIC[0].getCStride(), 1, m_acYuvTempIC[0].getCbAddr(uiPartAddr)  , m_acYuvPred[0].getCStride(), 1, m_acYuvPred[0].getCbAddr(uiPartAddr)  , REF_PIC_LIST_0 );
    xPredICompChromaBlk( &cIc, iRoiWidth>>1, iRoiHeight>>1, m_acYuvTempIC[0].getCStride(), 1, m_acYuvTempIC[0].getCrAddr(uiPartAddr)  , m_acYuvPred[0].getCStride(), 1, m_acYuvPred[0].getCrAddr(uiPartAddr)  , REF_PIC_LIST_0 );

    xPredICompLumaBlk  ( &cIc, iRoiWidth   , iRoiHeight   , m_acYuvTempIC[1].getStride() , 1, m_acYuvTempIC[1].getLumaAddr(uiPartAddr), m_acYuvPred[1].getStride() , 1, m_acYuvPred[1].getLumaAddr(uiPartAddr), REF_PIC_LIST_1 );
    xPredICompChromaBlk( &cIc, iRoiWidth>>1, iRoiHeight>>1, m_acYuvTempIC[1].getCStride(), 1, m_acYuvTempIC[1].getCbAddr(uiPartAddr)  , m_acYuvPred[1].getCStride(), 1, m_acYuvPred[1].getCbAddr(uiPartAddr)  , REF_PIC_LIST_1 );
    xPredICompChromaBlk( &cIc, iRoiWidth>>1, iRoiHeight>>1, m_acYuvTempIC[1].getCStride(), 1, m_acYuvTempIC[1].getCrAddr(uiPartAddr)  , m_acYuvPred[1].getCStride(), 1, m_acYuvPred[1].getCrAddr(uiPartAddr)  , REF_PIC_LIST_1 );

    xWeightedAverage ( pcCU, m_acYuvTempIC, m_acYuvTempIC+1, iRefIdx[0], iRefIdx[1], uiPartAddr, iRoiWidth, iRoiHeight, pcYuvPred);
  }

}

Bool TEncSearch::xSearchBestICParam( TComIc* cIcSearchCenter, Int& riIcpIdx, Int SearchRng0, Int SearchRng1, Int SearchRng2, UInt uiPartIdx, TComDataCU* pcCU, TComYuv* pcYuvOrg, RefPicList eRefPicList, UInt& ruiCostBest, Bool bFlagReset)
{ 
  Int i,j,k;
  UInt  uiCost;

  TComIc cIcRefine;
  TComIc cIcTest;
  Int iPram0RefineBest  = 0; 
  Int iPram1RefineBest  = 0;
  Int iPram2RefineBest  = 0;
  Int iIcpIdx           = 0;
  Int iIcpIdxBest       = 0;
  Bool bNewBest = false;

  ruiCostBest = MAX_UINT;
  Int aiCurParam[3], aiCenParam[3];

  assert (IC_SCALE_PREC >= 1);

  if (bFlagReset == true)
  {
    return false;
  }

  cIcSearchCenter->getIcParam(aiCenParam[0], aiCenParam[1], aiCenParam[2]);

  if (eRefPicList != REF_PIC_LIST_X) { SearchRng1 = 0; }

  for( i = -SearchRng0; i <= SearchRng0; i++)
  {
    for( j = -SearchRng1; j <= SearchRng1; j++)
    {
      for( k = -SearchRng2; k <= SearchRng2; k++)
      {
        aiCurParam[0] = aiCenParam[0] + i;
        aiCurParam[1] = aiCenParam[1] + j;
        aiCurParam[2] = aiCenParam[2] + k;

        cIcTest.setIcParam(aiCurParam[0], aiCurParam[1], aiCurParam[2]);
        cIcTest.computeScaleOffset( eRefPicList );
        uiCost = xCalculateCostAfterICPred(cIcTest, iIcpIdx, pcCU, pcYuvOrg, uiPartIdx, eRefPicList);

        if ( uiCost < ruiCostBest )
        {
          ruiCostBest      = uiCost;
          iPram0RefineBest = i;
          iPram1RefineBest = j;
          iPram2RefineBest = k;
          iIcpIdxBest      = iIcpIdx;
          bNewBest         = true;
        }
      }
    }
  }
  //Update cIcSearchCenter
  if (bNewBest == true)
  {
    cIcRefine.setIcParam(iPram0RefineBest,iPram1RefineBest,iPram2RefineBest);
    cIcSearchCenter->addIcParamDiff(cIcRefine);
    cIcSearchCenter->computeScaleOffset(eRefPicList);
    riIcpIdx = iIcpIdxBest;
  }

  return bNewBest;
}

UInt TEncSearch::xCalculateCostAfterICPred( TComIc cIcCand, Int& riIcpIdx, TComDataCU* pcCU, TComYuv* pcYuvOrg, UInt uiPartIdx, RefPicList eRefPicList )
{
  Int    iRoiWidth;
  Int    iRoiHeight;
  UInt   uiPartAddr;
  TComIc cIcPred;
  TComIc cIcd;

  UInt   uiCost;
  UInt   IcpBits;
  UInt   IcBits;

  TComMv acMvd[2];

  pcCU->getPartIndexAndSize( uiPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );

  AICPInfo* pcAICPInfo = pcCU->getCUIcField()->getAICPInfo();
  Int iIcpIdx;
  Int iIcpNum = pcAICPInfo->iN;
  UInt IcBitsBest = MAX_UINT;
  for (iIcpIdx = 0; iIcpIdx < pcAICPInfo->iN; iIcpIdx++)
  {

    cIcPred = pcAICPInfo->m_acIcCand[iIcpIdx];

    IcpBits = m_auiICPIdxCost[iIcpIdx][iIcpNum];

    // compute ICD
    cIcd.copyIcParam(cIcCand);
    cIcd.subIcParamPred(cIcPred);

    // add the estimated cost of coding MVD & ICD
    if (eRefPicList != REF_PIC_LIST_X)
    {
      acMvd[eRefPicList] = pcCU->getCUMvField( eRefPicList )->getMvd   ( uiPartAddr );
    }
    else
    {
      acMvd[0]   = pcCU->getCUMvField( REF_PIC_LIST_0 )->getMvd   ( uiPartAddr );
      acMvd[1]   = pcCU->getCUMvField( REF_PIC_LIST_1 )->getMvd   ( uiPartAddr );
    }
#ifdef QC_AMVRES
    if ( pcCU->getSlice()->getSPS()->getUseAMVRes() )
    {
      if (( (eRefPicList == REF_PIC_LIST_0) || (eRefPicList == REF_PIC_LIST_X) ) && ( !(pcCU->getCUMvField( REF_PIC_LIST_0 )->getMv( uiPartAddr ).isHAM()) ))
        acMvd[REF_PIC_LIST_0].scale_down();
      if (( (eRefPicList == REF_PIC_LIST_1) || (eRefPicList == REF_PIC_LIST_X) ) && ( !(pcCU->getCUMvField( REF_PIC_LIST_1 )->getMv( uiPartAddr ).isHAM()) ))
        acMvd[REF_PIC_LIST_1].scale_down();
    }
#endif
    IcBits = IcpBits + (UInt)xEstMvdIcdBits( pcCU, acMvd, &cIcd, eRefPicList );

    if (cIcCand.isequalIcParam(cIcPred))
    {
      riIcpIdx = iIcpIdx;
      IcBitsBest = IcBits;
      break;
    }
    if (IcBits < IcBitsBest)
    {
      riIcpIdx = iIcpIdx;
      IcBitsBest = IcBits;
    }
  }
  iIcpIdx = riIcpIdx;
  IcBits = IcBitsBest;

  if (eRefPicList != REF_PIC_LIST_X)
  {
    xPredICompLumaBlk  ( &cIcCand, iRoiWidth   , iRoiHeight   , m_acYuvTempIC->getStride() , 1, m_acYuvTempIC->getLumaAddr(uiPartAddr), m_acYuvPred[eRefPicList].getStride() , 1, m_acYuvPred[eRefPicList].getLumaAddr(uiPartAddr), eRefPicList );
    xPredICompChromaBlk( &cIcCand, iRoiWidth>>1, iRoiHeight>>1, m_acYuvTempIC->getCStride(), 1, m_acYuvTempIC->getCbAddr(uiPartAddr)  , m_acYuvPred[eRefPicList].getCStride(), 1, m_acYuvPred[eRefPicList].getCbAddr(uiPartAddr)  , eRefPicList );
    xPredICompChromaBlk( &cIcCand, iRoiWidth>>1, iRoiHeight>>1, m_acYuvTempIC->getCStride(), 1, m_acYuvTempIC->getCrAddr(uiPartAddr)  , m_acYuvPred[eRefPicList].getCStride(), 1, m_acYuvPred[eRefPicList].getCrAddr(uiPartAddr)  , eRefPicList );
  }
  else
  {
    Int iRefIdx[2];
    iRefIdx[0] = pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr );
    iRefIdx[1] = pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr );

    xPredICompLumaBlk  ( &cIcCand, iRoiWidth   , iRoiHeight   , m_acYuvTempIC[0].getStride() , 1, m_acYuvTempIC[0].getLumaAddr(uiPartAddr), m_acYuvPred[0].getStride() , 1, m_acYuvPred[0].getLumaAddr(uiPartAddr), REF_PIC_LIST_0 );
    xPredICompChromaBlk( &cIcCand, iRoiWidth>>1, iRoiHeight>>1, m_acYuvTempIC[0].getCStride(), 1, m_acYuvTempIC[0].getCbAddr(uiPartAddr)  , m_acYuvPred[0].getCStride(), 1, m_acYuvPred[0].getCbAddr(uiPartAddr)  , REF_PIC_LIST_0 );
    xPredICompChromaBlk( &cIcCand, iRoiWidth>>1, iRoiHeight>>1, m_acYuvTempIC[0].getCStride(), 1, m_acYuvTempIC[0].getCrAddr(uiPartAddr)  , m_acYuvPred[0].getCStride(), 1, m_acYuvPred[0].getCrAddr(uiPartAddr)  , REF_PIC_LIST_0 );

    xPredICompLumaBlk  ( &cIcCand, iRoiWidth   , iRoiHeight   , m_acYuvTempIC[1].getStride() , 1, m_acYuvTempIC[1].getLumaAddr(uiPartAddr), m_acYuvPred[1].getStride() , 1, m_acYuvPred[1].getLumaAddr(uiPartAddr), REF_PIC_LIST_1 );
    xPredICompChromaBlk( &cIcCand, iRoiWidth>>1, iRoiHeight>>1, m_acYuvTempIC[1].getCStride(), 1, m_acYuvTempIC[1].getCbAddr(uiPartAddr)  , m_acYuvPred[1].getCStride(), 1, m_acYuvPred[1].getCbAddr(uiPartAddr)  , REF_PIC_LIST_1 );
    xPredICompChromaBlk( &cIcCand, iRoiWidth>>1, iRoiHeight>>1, m_acYuvTempIC[1].getCStride(), 1, m_acYuvTempIC[1].getCrAddr(uiPartAddr)  , m_acYuvPred[1].getCStride(), 1, m_acYuvPred[1].getCrAddr(uiPartAddr)  , REF_PIC_LIST_1 );

    TComYuv* pcYuvPred = m_acYuvTempIC;
    xWeightedAverage( pcCU, m_acYuvTempIC, m_acYuvTempIC+1, iRefIdx[0], iRefIdx[1], uiPartAddr, iRoiWidth, iRoiHeight, pcYuvPred);
  }

  // calc distortion ( HADSAD + bitcost )
  DFunc eDFunc = (m_pcEncCfg->getUseHADME() == true) ? DF_HADS : DF_SADS;
  uiCost = m_pcRdCost->getDistPart(  m_acYuvTempIC->getLumaAddr(uiPartAddr),  m_acYuvTempIC->getStride(), pcYuvOrg->getLumaAddr(uiPartAddr), pcYuvOrg->getStride(), iRoiWidth, iRoiHeight, eDFunc );
  uiCost += UInt(m_pcRdCost->getDistPart( m_acYuvTempIC->getCbAddr(uiPartAddr), m_acYuvTempIC->getCStride(),pcYuvOrg->getCbAddr(uiPartAddr),pcYuvOrg->getCStride(), iRoiWidth>>1, iRoiHeight>>1, eDFunc)
              +  m_pcRdCost->getDistPart( m_acYuvTempIC->getCrAddr(uiPartAddr), m_acYuvTempIC->getCStride(),pcYuvOrg->getCrAddr(uiPartAddr),pcYuvOrg->getCStride(), iRoiWidth>>1, iRoiHeight>>1, eDFunc));
  uiCost = (UInt) ( uiCost / 1.5 );

  UInt uiCostTmp = uiCost;
  uiCost = (UInt) m_pcRdCost->calcRdCost( IcBits, uiCost, false, DF_SAD );
  assert(uiCost >= uiCostTmp);

  return uiCost;
}

Double TEncSearch::xComputeSum( TComYuv* pcYuvSrc, UInt uiPartAddr, Int iWidth, Int iHeight, Int YCbCr )
{
  Int x, y;
  Double dbSum=0.0;
  Pel* pSrc; 
  Int iStride;

  if(YCbCr==0){
	  pSrc   = pcYuvSrc->getLumaAddr(uiPartAddr);
	  iStride = pcYuvSrc->getStride();
 }
  if(YCbCr==1){
		pSrc   = pcYuvSrc->getCbAddr(uiPartAddr);
		iStride = pcYuvSrc->getCStride();
	}
	if(YCbCr==2){
		pSrc   = pcYuvSrc->getCrAddr(uiPartAddr);
		iStride = pcYuvSrc->getCStride();
	}

  for (y=0; y<iHeight; y++)
  {
    for (x=0; x<iWidth; x++)
    {
      dbSum += (Double)(pSrc[x]);
    }
    pSrc += iStride;  
	}

  return dbSum;
}

Double TEncSearch::xComputeSquareSum( TComYuv* pcYuvSrc, UInt uiPartAddr, Int iWidth, Int iHeight, Int YCbCr )
{
  Int x, y;
  Double dbSquareSum=0.0;
  Pel* pSrc; 
  Int iStride;

  if(YCbCr==0){
	  pSrc    = pcYuvSrc->getLumaAddr(uiPartAddr);
	  iStride = pcYuvSrc->getStride();
 }
  if(YCbCr==1){
		pSrc    = pcYuvSrc->getCbAddr(uiPartAddr);
		iStride = pcYuvSrc->getCStride();
	}
	if(YCbCr==2){
		pSrc    = pcYuvSrc->getCrAddr(uiPartAddr);
		iStride = pcYuvSrc->getCStride();
	}

  for (y=0; y<iHeight; y++)
  {
    for (x=0; x<iWidth; x++)
    {
			dbSquareSum += (((Double)pSrc[x])*pSrc[x]);

    }
		pSrc += iStride;
  }
  return dbSquareSum;
}

Double TEncSearch::xComputeSumMult( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiPartAddr, Int iWidth, Int iHeight, Int YCbCr )
{
  Int x, y;
  Double dbSumMult=0.0;
	Pel* pSrc0;
	Pel* pSrc1;
	Int iStride0,iStride1;

 if(YCbCr==0){
    pSrc0    = pcYuvSrc0->getLumaAddr(uiPartAddr);
    iStride0 = pcYuvSrc0->getStride();
 
	  pSrc1    = pcYuvSrc1->getLumaAddr(uiPartAddr);
	  iStride1 = pcYuvSrc1->getStride();
 }
 if(YCbCr==1){
    pSrc0   = pcYuvSrc0->getCbAddr(uiPartAddr);
    iStride0 = pcYuvSrc0->getCStride();

    pSrc1   = pcYuvSrc1->getCbAddr(uiPartAddr);
    iStride1 = pcYuvSrc1->getCStride();
 }
 if(YCbCr==2){
		pSrc0   = pcYuvSrc0->getCrAddr(uiPartAddr);
		iStride0 = pcYuvSrc0->getCStride();

		pSrc1   = pcYuvSrc1->getCrAddr(uiPartAddr);
		iStride1 = pcYuvSrc1->getCStride();
	}

  for (y=0; y<iHeight; y++)
  {
    for (x=0; x<iWidth; x++)
    {
			dbSumMult += (((Double)pSrc0[x])*pSrc1[x]);
    }
    pSrc0 += iStride0;
	  pSrc1 += iStride1;
		
  }
  return dbSumMult;
}

Void TEncSearch::Calc_ICParam_LMS_Uni( TComYuv* pcYuvOrg, TComYuv* pcYuvPred, UInt uiPartAddr, Int iHeight, Int iWidth, TComIc* rcIc, RefPicList eRefPicList )
{
  Double dbScale0;
  Double dbOffset;

  Int iParam0;
  Int iParam2;
  Int iDCoffset = (128 << g_uiBitIncrement);
  Int iRound = (g_uiBitIncrement>0) ? (1<<(g_uiBitIncrement-1)) : 0;

  Double dbOrgSum          = xComputeSum(pcYuvOrg ,uiPartAddr,iWidth,iHeight,0);          // sum(OrgY)
  Double dbPred0Sum        = xComputeSum(pcYuvPred,uiPartAddr,iWidth,iHeight,0);          // sum(PredY)
  Double dbPred0SquareSum  = xComputeSquareSum(pcYuvPred ,uiPartAddr,iWidth,iHeight,0); 
  Double dbPred0OrgSum     = xComputeSumMult(pcYuvPred ,pcYuvOrg ,uiPartAddr,iWidth,iHeight,0);
                         
  Double dbN                 = iWidth * iHeight; //N : d

  Double dbNchroma     = (iWidth>>1) * (iHeight>>1);
  Double dbOrgCbSum    = xComputeSum(pcYuvOrg ,uiPartAddr,iWidth>>1,iHeight>>1,1);
  Double dbOrgCrSum    = xComputeSum(pcYuvOrg ,uiPartAddr,iWidth>>1,iHeight>>1,2);
  Double dbPred0CbSum  = xComputeSum(pcYuvPred,uiPartAddr,iWidth>>1,iHeight>>1,1);    
  Double dbPred0CrSum  = xComputeSum(pcYuvPred,uiPartAddr,iWidth>>1,iHeight>>1,2); 

  Double dbPred0CbSquareSum = xComputeSquareSum(pcYuvPred ,uiPartAddr,iWidth>>1,iHeight>>1,1); 
  Double dbPred0CrSquareSum = xComputeSquareSum(pcYuvPred ,uiPartAddr,iWidth>>1,iHeight>>1,2); 

  Double dbPred0CbOrgCbSum  = xComputeSumMult(pcYuvPred ,pcYuvOrg ,uiPartAddr,iWidth>>1,iHeight>>1,1);
  Double dbPred0CrOrgCrSum  = xComputeSumMult(pcYuvPred ,pcYuvOrg ,uiPartAddr,iWidth>>1,iHeight>>1,2);

  dbPred0CbSquareSum -= iDCoffset * ( dbPred0CbSum + dbPred0CbSum - (dbNchroma * iDCoffset) );
  dbPred0CrSquareSum -= iDCoffset * ( dbPred0CrSum + dbPred0CrSum - (dbNchroma * iDCoffset) );
  dbPred0CbOrgCbSum  -= iDCoffset * ( dbPred0CbSum + dbOrgCbSum   - (dbNchroma * iDCoffset) );
  dbPred0CrOrgCrSum  -= iDCoffset * ( dbPred0CrSum + dbOrgCrSum   - (dbNchroma * iDCoffset) );

  dbPred0SquareSum += ( dbPred0CbSquareSum + dbPred0CrSquareSum);  // update to sum(predY^2)+ sum( predCb^2) + sum ( predCr^2) :a
  dbPred0OrgSum    += ( dbPred0CbOrgCbSum  + dbPred0CrOrgCrSum);  // update to sum(predY * OrgY) + sum(predCb * OrgCb) + sum(predCr * OrgCr)

  Double dbDet =  (dbPred0SquareSum * dbN) - (dbPred0Sum * dbPred0Sum) ;

  if( dbDet > 0 )
  {
    dbScale0 =((dbN * dbPred0OrgSum) - (dbPred0Sum  * dbOrgSum)) / dbDet;
    iParam0 = ICROUND( (dbScale0 - 1.0) * (1 << IC_SCALE_PREC) );

    dbScale0 = 1 + Double(iParam0) / (1 << IC_SCALE_PREC);
    dbOffset = ( dbOrgSum - dbPred0Sum*dbScale0 ) / dbN;
    iParam2 = ( ICROUND(dbOffset) + iRound ) >> g_uiBitIncrement;
  }
  else
  {
	  iParam0=0;
	  iParam2=0;
  }

  rcIc->setIcParam(iParam0,0,iParam2); 
  rcIc->computeScaleOffset( eRefPicList );

  // Scale only
  rcIc++;
  if (dbPred0SquareSum > 0)
  {
    dbScale0 = dbPred0OrgSum / dbPred0SquareSum;
    iParam0 = ICROUND( (dbScale0 - 1.0) * (1 << IC_SCALE_PREC) );
  }
  else
  {
    iParam0 = 0;
  }
  rcIc->setIcParam(iParam0, 0, 0);
  rcIc->computeScaleOffset( eRefPicList );

  return;
}

Void TEncSearch::Calc_ICParam_LMS_Bi( TComYuv* pcYuvOrg, TComYuv* pcYuvPred0, TComYuv* pcYuvPred1, UInt uiPartAddr, Int iHeight, Int iWidth, TComIc* rcIc )
{
  Double dbScale0;
  Double dbScale1;
  Double dbOffset;

  Int iParam0;
  Int iParam1;
  Int iParam2;
  Int iDCoffset = (128 << g_uiBitIncrement);
  Int iRound = (g_uiBitIncrement>0) ? (1<<(g_uiBitIncrement-1)) : 0;

  Double dbOrgSum   = xComputeSum(pcYuvOrg   ,uiPartAddr,iWidth,iHeight,0); // sum(OrgY)
  Double dbPred0Sum = xComputeSum(pcYuvPred0 ,uiPartAddr,iWidth,iHeight,0); //sum(pred0Y)
  Double dbPred1Sum = xComputeSum(pcYuvPred1 ,uiPartAddr,iWidth,iHeight,0); //sum(pred1Y)

  Double dbPred0SquareSum = xComputeSquareSum(pcYuvPred0,uiPartAddr,iWidth,iHeight,0);
  Double dbPred1SquareSum = xComputeSquareSum(pcYuvPred1,uiPartAddr,iWidth,iHeight,0);
  Double dbN             = iWidth * iHeight; //N

  Double dbPred0Pred1Sum  = xComputeSumMult(pcYuvPred0,pcYuvPred1,uiPartAddr,iWidth,iHeight,0);
  Double dbPred0OrgSum    = xComputeSumMult(pcYuvPred0,pcYuvOrg  ,uiPartAddr,iWidth,iHeight,0);
  Double dbPred1OrgSum    = xComputeSumMult(pcYuvPred1,pcYuvOrg  ,uiPartAddr,iWidth,iHeight,0);

  Double dbNchroma     = (iWidth>>1) * (iHeight>>1);
  Double dbOrgCbSum    = xComputeSum(pcYuvOrg  ,uiPartAddr,iWidth>>1,iHeight>>1,1);
  Double dbOrgCrSum    = xComputeSum(pcYuvOrg  ,uiPartAddr,iWidth>>1,iHeight>>1,2);
  Double dbPred0CbSum  = xComputeSum(pcYuvPred0,uiPartAddr,iWidth>>1,iHeight>>1,1);    
  Double dbPred0CrSum  = xComputeSum(pcYuvPred0,uiPartAddr,iWidth>>1,iHeight>>1,2); 
  Double dbPred1CbSum  = xComputeSum(pcYuvPred1,uiPartAddr,iWidth>>1,iHeight>>1,1);    
  Double dbPred1CrSum  = xComputeSum(pcYuvPred1,uiPartAddr,iWidth>>1,iHeight>>1,2); 

  Double dbPred0CbSquareSum = xComputeSquareSum(pcYuvPred0,uiPartAddr,iWidth>>1,iHeight>>1,1); 
  Double dbPred0CrSquareSum = xComputeSquareSum(pcYuvPred0,uiPartAddr,iWidth>>1,iHeight>>1,2); 
  Double dbPred1CbSquareSum = xComputeSquareSum(pcYuvPred1,uiPartAddr,iWidth>>1,iHeight>>1,1); 
  Double dbPred1CrSquareSum = xComputeSquareSum(pcYuvPred1,uiPartAddr,iWidth>>1,iHeight>>1,2); 

  Double dbPred0CbOrgCbSum  = xComputeSumMult(pcYuvPred0 ,pcYuvOrg ,uiPartAddr,iWidth>>1,iHeight>>1,1);
  Double dbPred0CrOrgCrSum  = xComputeSumMult(pcYuvPred0 ,pcYuvOrg ,uiPartAddr,iWidth>>1,iHeight>>1,2);
  Double dbPred1CbOrgCbSum  = xComputeSumMult(pcYuvPred1 ,pcYuvOrg ,uiPartAddr,iWidth>>1,iHeight>>1,1);
  Double dbPred1CrOrgCrSum  = xComputeSumMult(pcYuvPred1 ,pcYuvOrg ,uiPartAddr,iWidth>>1,iHeight>>1,2);

  Double dbPred0CbPred1CbSum  = xComputeSumMult(pcYuvPred0 ,pcYuvPred1 ,uiPartAddr,iWidth>>1,iHeight>>1,1);
  Double dbPred0CrPred1CrSum  = xComputeSumMult(pcYuvPred0 ,pcYuvPred1 ,uiPartAddr,iWidth>>1,iHeight>>1,2);

  dbPred0CbSquareSum -= iDCoffset * ( dbPred0CbSum + dbPred0CbSum - (dbNchroma * iDCoffset) );
  dbPred0CrSquareSum -= iDCoffset * ( dbPred0CrSum + dbPred0CrSum - (dbNchroma * iDCoffset) );
  dbPred1CbSquareSum -= iDCoffset * ( dbPred1CbSum + dbPred1CbSum - (dbNchroma * iDCoffset) );
  dbPred1CrSquareSum -= iDCoffset * ( dbPred1CrSum + dbPred1CrSum - (dbNchroma * iDCoffset) );

  dbPred0CbOrgCbSum  -= iDCoffset * ( dbPred0CbSum + dbOrgCbSum   - (dbNchroma * iDCoffset) );
  dbPred0CrOrgCrSum  -= iDCoffset * ( dbPred0CrSum + dbOrgCrSum   - (dbNchroma * iDCoffset) );
  dbPred1CbOrgCbSum  -= iDCoffset * ( dbPred1CbSum + dbOrgCbSum   - (dbNchroma * iDCoffset) );
  dbPred1CrOrgCrSum  -= iDCoffset * ( dbPred1CrSum + dbOrgCrSum   - (dbNchroma * iDCoffset) );
  dbPred0CbPred1CbSum -= iDCoffset * ( dbPred0CbSum + dbPred1CbSum   - (dbNchroma * iDCoffset) );
  dbPred0CrPred1CrSum -= iDCoffset * ( dbPred0CrSum + dbPred1CrSum   - (dbNchroma * iDCoffset) );

  dbPred0SquareSum += ( dbPred0CbSquareSum + dbPred0CrSquareSum);  // sum(pred0Y^2) + sum (pred0Cb^2) + sum ( pred0Cr^2)
  dbPred1SquareSum += ( dbPred1CbSquareSum + dbPred1CrSquareSum);  // sum(pred0Y^2) + sum (pred0Cb^2) + sum ( pred0Cr^2)
  dbPred0Pred1Sum  += ( dbPred0CbPred1CbSum + dbPred0CrPred1CrSum);// sum(pred0Y*pred1Y) + sum(pred0Cb*pred1Cb) + sum(pred0Cr*pred1Cr)
  dbPred0OrgSum    += ( dbPred0CbOrgCbSum + dbPred0CrOrgCrSum);    // sum(pred0Y* OrgY) + sum(pred0Cb * OrgCb) + sum(pred0Cr* OrgCr)
  dbPred1OrgSum    += ( dbPred1CbOrgCbSum + dbPred1CrOrgCrSum);    // sum(pred1Y* OrgY) + sum(pred1Cb * OrgCb) + sum(pred1Cr* OrgCr)

  Double dbA = (dbN * dbPred1SquareSum)  - (dbPred1Sum * dbPred1Sum);
  Double dbB = (dbPred0Sum * dbPred1Sum) - (dbN * dbPred0Pred1Sum);
  Double dbC = (dbPred1Sum * dbPred0Pred1Sum) -  (dbPred1SquareSum * dbPred0Sum);

  Double dbD = (dbN * dbPred0SquareSum) - (dbPred0Sum * dbPred0Sum);
  Double dbE = (dbPred0Pred1Sum * dbPred0Sum) - (dbPred1Sum * dbPred0SquareSum);
  Double dbF = (dbPred0SquareSum * dbPred1SquareSum) - (dbPred0Pred1Sum * dbPred0Pred1Sum);

  Double dbDet = (dbPred0SquareSum * dbA) + (dbPred0Pred1Sum * dbB) + (dbPred0Sum * dbC);

  if( dbDet > 0 )
  {
    dbScale0 = ((dbPred0OrgSum * dbA) + (dbPred1OrgSum * dbB) + (dbOrgSum * dbC))/dbDet;
    dbScale1 = ((dbPred0OrgSum * dbB) + (dbPred1OrgSum * dbD) + (dbOrgSum * dbE))/dbDet;
    iParam0 = ICROUND( (dbScale0 + dbScale1 - 1.0) * (1 <<  IC_SCALE_PREC   ) );
    iParam1 = ICROUND( (dbScale0 - dbScale1      ) * (1 << (IC_SCALE_PREC-1)) );

    dbScale0 = 0.5 * (1 + Double(iParam0 + (iParam1<<1)) / (1 << IC_SCALE_PREC));
    dbScale1 = 0.5 * (1 + Double(iParam0 - (iParam1<<1)) / (1 << IC_SCALE_PREC));
    dbOffset = ( dbOrgSum - (dbPred0Sum*dbScale0 + dbPred1Sum*dbScale1) ) / dbN;
    iParam2 = ( ICROUND(dbOffset) + iRound ) >> g_uiBitIncrement;
  }
  else
  {
	  iParam0=0;
	  iParam1=0;
	  iParam2=0;
  }

  rcIc->setIcParam(iParam0,iParam1,iParam2);
  rcIc->computeScaleOffset( REF_PIC_LIST_X  );

  // Scale only
  rcIc++;
  if (dbF > 0)
  {
    dbScale0 = ( (dbPred0OrgSum*dbPred1SquareSum) - (dbPred1OrgSum*dbPred0Pred1Sum) ) / dbF;
    dbScale1 = ( (dbPred1OrgSum*dbPred0SquareSum) - (dbPred0OrgSum*dbPred0Pred1Sum) ) / dbF;
    iParam0 = ICROUND( (dbScale0 + dbScale1 - 1.0) * (1 <<  IC_SCALE_PREC   ) );
    iParam1 = ICROUND( (dbScale0 - dbScale1      ) * (1 << (IC_SCALE_PREC-1)) );
  }
  else
  {
	  iParam0 = 0;
	  iParam1 = 0;
  }
  rcIc->setIcParam(iParam0, iParam1, 0);
  rcIc->computeScaleOffset( REF_PIC_LIST_X  );

  return;
}

UInt TEncSearch::xGetComponentBits(Int iVal)
{
  UInt uiLength = 1;
  UInt uiTemp   = ( iVal <= 0) ? (-iVal<<1)+1: (iVal<<1);

  assert ( uiTemp );

  while ( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }

  return uiLength;
}

UInt TEncSearch::xGetMvdBits(TComMv cMvd)
{
  return ( xGetComponentBits(cMvd.getHor()) + xGetComponentBits(cMvd.getVer()) );
}

UInt TEncSearch::xGetIcdBits(TComIc cIcd)
{
  Int aiParam[3];
  cIcd.getIcParam( aiParam[0], aiParam[1], aiParam[2] );
  return ( xGetComponentBits(aiParam[0]) + xGetComponentBits(aiParam[1]) + xGetComponentBits(aiParam[2]) );
}
#endif //#ifdef DCM_PBIC
>>>>>>> upstream/master
