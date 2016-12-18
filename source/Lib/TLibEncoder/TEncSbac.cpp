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

/** \file     TEncSbac.cpp
    \brief    SBAC encoder class
*/

#include "TEncTop.h"
#include "TEncSbac.h"
#include "TLibCommon/TComTU.h"

#include <map>
#include <algorithm>

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
#include "../TLibCommon/Debug.h"
#endif


//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TEncSbac::TEncSbac()
// new structure here
: m_pcBitIf                            ( NULL )
, m_pcBinIf                            ( NULL )
, m_numContextModels                   ( 0 )
, m_cCUSplitFlagSCModel                ( 1,             1,                      NUM_SPLIT_FLAG_CTX                   , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSkipFlagSCModel                 ( 1,             1,                      NUM_SKIP_FLAG_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMergeFlagExtSCModel             ( 1,             1,                      NUM_MERGE_FLAG_EXT_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMergeIdxExtSCModel              ( 1,             1,                      NUM_MERGE_IDX_EXT_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUPartSizeSCModel                 ( 1,             1,                      NUM_PART_SIZE_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUPredModeSCModel                 ( 1,             1,                      NUM_PRED_MODE_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUIntraPredSCModel                ( 1,             1,                      NUM_INTRA_PREDICT_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUChromaPredSCModel               ( 1,             1,                      NUM_CHROMA_PRED_CTX                  , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUDeltaQpSCModel                  ( 1,             1,                      NUM_DELTA_QP_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUInterDirSCModel                 ( 1,             1,                      NUM_INTER_DIR_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCURefPicSCModel                   ( 1,             1,                      NUM_REF_NO_CTX                       , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMvdSCModel                      ( 1,             1,                      NUM_MV_RES_CTX                       , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUQtCbfSCModel                    ( 1,             NUM_QT_CBF_CTX_SETS,    NUM_QT_CBF_CTX_PER_SET               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUTransSubdivFlagSCModel          ( 1,             1,                      NUM_TRANS_SUBDIV_FLAG_CTX            , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUQtRootCbfSCModel                ( 1,             1,                      NUM_QT_ROOT_CBF_CTX                  , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSigCoeffGroupSCModel            ( 1,             2,                      NUM_SIG_CG_FLAG_CTX                  , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSigSCModel                      ( 1,             1,                      NUM_SIG_FLAG_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCuCtxLastX                        ( 1,             NUM_CTX_LAST_FLAG_SETS, NUM_CTX_LAST_FLAG_XY                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCuCtxLastY                        ( 1,             NUM_CTX_LAST_FLAG_SETS, NUM_CTX_LAST_FLAG_XY                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUOneSCModel                      ( 1,             1,                      NUM_ONE_FLAG_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUAbsSCModel                      ( 1,             1,                      NUM_ABS_FLAG_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cMVPIdxSCModel                     ( 1,             1,                      NUM_MVP_IDX_CTX                      , m_contextModels + m_numContextModels, m_numContextModels)
, m_cSaoMergeSCModel                   ( 1,             1,                      NUM_SAO_MERGE_FLAG_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cSaoTypeIdxSCModel                 ( 1,             1,                      NUM_SAO_TYPE_IDX_CTX                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cTransformSkipSCModel              ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_TRANSFORMSKIP_FLAG_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_CUTransquantBypassFlagSCModel      ( 1,             1,                      NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX    , m_contextModels + m_numContextModels, m_numContextModels)
, m_explicitRdpcmFlagSCModel           ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_EXPLICIT_RDPCM_FLAG_CTX          , m_contextModels + m_numContextModels, m_numContextModels)
, m_explicitRdpcmDirSCModel            ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_EXPLICIT_RDPCM_DIR_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCrossComponentPredictionSCModel   ( 1,             1,                      NUM_CROSS_COMPONENT_PREDICTION_CTX   , m_contextModels + m_numContextModels, m_numContextModels)
, m_ChromaQpAdjFlagSCModel             ( 1,             1,                      NUM_CHROMA_QP_ADJ_FLAG_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_ChromaQpAdjIdcSCModel              ( 1,             1,                      NUM_CHROMA_QP_ADJ_IDC_CTX            , m_contextModels + m_numContextModels, m_numContextModels)
{
  assert( m_numContextModels <= MAX_NUM_CTX_MOD );
}

TEncSbac::~TEncSbac()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TEncSbac::resetEntropy           (const TComSlice *pSlice)
{
  Int  iQp              = pSlice->getSliceQp();
  SliceType eSliceType  = pSlice->getSliceType();

  SliceType encCABACTableIdx = pSlice->getEncCABACTableIdx();
  if (!pSlice->isIntra() && (encCABACTableIdx==B_SLICE || encCABACTableIdx==P_SLICE) && pSlice->getPPS()->getCabacInitPresentFlag())
  {
    eSliceType = encCABACTableIdx;
  }

  m_cCUSplitFlagSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_SPLIT_FLAG );
  m_cCUSkipFlagSCModel.initBuffer                 ( eSliceType, iQp, (UChar*)INIT_SKIP_FLAG );
  m_cCUMergeFlagExtSCModel.initBuffer             ( eSliceType, iQp, (UChar*)INIT_MERGE_FLAG_EXT);
  m_cCUMergeIdxExtSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_MERGE_IDX_EXT);
  m_cCUPartSizeSCModel.initBuffer                 ( eSliceType, iQp, (UChar*)INIT_PART_SIZE );
  m_cCUPredModeSCModel.initBuffer                 ( eSliceType, iQp, (UChar*)INIT_PRED_MODE );
  m_cCUIntraPredSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_INTRA_PRED_MODE );
  m_cCUChromaPredSCModel.initBuffer               ( eSliceType, iQp, (UChar*)INIT_CHROMA_PRED_MODE );
  m_cCUInterDirSCModel.initBuffer                 ( eSliceType, iQp, (UChar*)INIT_INTER_DIR );
  m_cCUMvdSCModel.initBuffer                      ( eSliceType, iQp, (UChar*)INIT_MVD );
  m_cCURefPicSCModel.initBuffer                   ( eSliceType, iQp, (UChar*)INIT_REF_PIC );
  m_cCUDeltaQpSCModel.initBuffer                  ( eSliceType, iQp, (UChar*)INIT_DQP );
  m_cCUQtCbfSCModel.initBuffer                    ( eSliceType, iQp, (UChar*)INIT_QT_CBF );
  m_cCUQtRootCbfSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_QT_ROOT_CBF );
  m_cCUSigCoeffGroupSCModel.initBuffer            ( eSliceType, iQp, (UChar*)INIT_SIG_CG_FLAG );
  m_cCUSigSCModel.initBuffer                      ( eSliceType, iQp, (UChar*)INIT_SIG_FLAG );
  m_cCuCtxLastX.initBuffer                        ( eSliceType, iQp, (UChar*)INIT_LAST );
  m_cCuCtxLastY.initBuffer                        ( eSliceType, iQp, (UChar*)INIT_LAST );
  m_cCUOneSCModel.initBuffer                      ( eSliceType, iQp, (UChar*)INIT_ONE_FLAG );
  m_cCUAbsSCModel.initBuffer                      ( eSliceType, iQp, (UChar*)INIT_ABS_FLAG );
  m_cMVPIdxSCModel.initBuffer                     ( eSliceType, iQp, (UChar*)INIT_MVP_IDX );
  m_cCUTransSubdivFlagSCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_TRANS_SUBDIV_FLAG );
  m_cSaoMergeSCModel.initBuffer                   ( eSliceType, iQp, (UChar*)INIT_SAO_MERGE_FLAG );
  m_cSaoTypeIdxSCModel.initBuffer                 ( eSliceType, iQp, (UChar*)INIT_SAO_TYPE_IDX );
  m_cTransformSkipSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_TRANSFORMSKIP_FLAG );
  m_CUTransquantBypassFlagSCModel.initBuffer      ( eSliceType, iQp, (UChar*)INIT_CU_TRANSQUANT_BYPASS_FLAG );
  m_explicitRdpcmFlagSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_EXPLICIT_RDPCM_FLAG);
  m_explicitRdpcmDirSCModel.initBuffer            ( eSliceType, iQp, (UChar*)INIT_EXPLICIT_RDPCM_DIR);
  m_cCrossComponentPredictionSCModel.initBuffer   ( eSliceType, iQp, (UChar*)INIT_CROSS_COMPONENT_PREDICTION  );
  m_ChromaQpAdjFlagSCModel.initBuffer             ( eSliceType, iQp, (UChar*)INIT_CHROMA_QP_ADJ_FLAG );
  m_ChromaQpAdjIdcSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_CHROMA_QP_ADJ_IDC );

  for (UInt statisticIndex = 0; statisticIndex < RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS ; statisticIndex++)
  {
    m_golombRiceAdaptationStatistics[statisticIndex] = 0;
  }

  m_pcBinIf->start();

  return;
}

/** The function does the following:
 * If current slice type is P/B then it determines the distance of initialisation type 1 and 2 from the current CABAC states and
 * stores the index of the closest table.  This index is used for the next P/B slice when cabac_init_present_flag is true.
 */
SliceType TEncSbac::determineCabacInitIdx(const TComSlice *pSlice)
{
  Int  qp              = pSlice->getSliceQp();

  if (!pSlice->isIntra())
  {
    SliceType aSliceTypeChoices[] = {B_SLICE, P_SLICE};

    UInt bestCost             = MAX_UINT;
    SliceType bestSliceType   = aSliceTypeChoices[0];
    for (UInt idx=0; idx<2; idx++)
    {
      UInt curCost          = 0;
      SliceType curSliceType  = aSliceTypeChoices[idx];

      curCost  = m_cCUSplitFlagSCModel.calcCost                ( curSliceType, qp, (UChar*)INIT_SPLIT_FLAG );
      curCost += m_cCUSkipFlagSCModel.calcCost                 ( curSliceType, qp, (UChar*)INIT_SKIP_FLAG );
      curCost += m_cCUMergeFlagExtSCModel.calcCost             ( curSliceType, qp, (UChar*)INIT_MERGE_FLAG_EXT);
      curCost += m_cCUMergeIdxExtSCModel.calcCost              ( curSliceType, qp, (UChar*)INIT_MERGE_IDX_EXT);
      curCost += m_cCUPartSizeSCModel.calcCost                 ( curSliceType, qp, (UChar*)INIT_PART_SIZE );
      curCost += m_cCUPredModeSCModel.calcCost                 ( curSliceType, qp, (UChar*)INIT_PRED_MODE );
      curCost += m_cCUIntraPredSCModel.calcCost                ( curSliceType, qp, (UChar*)INIT_INTRA_PRED_MODE );
      curCost += m_cCUChromaPredSCModel.calcCost               ( curSliceType, qp, (UChar*)INIT_CHROMA_PRED_MODE );
      curCost += m_cCUInterDirSCModel.calcCost                 ( curSliceType, qp, (UChar*)INIT_INTER_DIR );
      curCost += m_cCUMvdSCModel.calcCost                      ( curSliceType, qp, (UChar*)INIT_MVD );
      curCost += m_cCURefPicSCModel.calcCost                   ( curSliceType, qp, (UChar*)INIT_REF_PIC );
      curCost += m_cCUDeltaQpSCModel.calcCost                  ( curSliceType, qp, (UChar*)INIT_DQP );
      curCost += m_cCUQtCbfSCModel.calcCost                    ( curSliceType, qp, (UChar*)INIT_QT_CBF );
      curCost += m_cCUQtRootCbfSCModel.calcCost                ( curSliceType, qp, (UChar*)INIT_QT_ROOT_CBF );
      curCost += m_cCUSigCoeffGroupSCModel.calcCost            ( curSliceType, qp, (UChar*)INIT_SIG_CG_FLAG );
      curCost += m_cCUSigSCModel.calcCost                      ( curSliceType, qp, (UChar*)INIT_SIG_FLAG );
      curCost += m_cCuCtxLastX.calcCost                        ( curSliceType, qp, (UChar*)INIT_LAST );
      curCost += m_cCuCtxLastY.calcCost                        ( curSliceType, qp, (UChar*)INIT_LAST );
      curCost += m_cCUOneSCModel.calcCost                      ( curSliceType, qp, (UChar*)INIT_ONE_FLAG );
      curCost += m_cCUAbsSCModel.calcCost                      ( curSliceType, qp, (UChar*)INIT_ABS_FLAG );
      curCost += m_cMVPIdxSCModel.calcCost                     ( curSliceType, qp, (UChar*)INIT_MVP_IDX );
      curCost += m_cCUTransSubdivFlagSCModel.calcCost          ( curSliceType, qp, (UChar*)INIT_TRANS_SUBDIV_FLAG );
      curCost += m_cSaoMergeSCModel.calcCost                   ( curSliceType, qp, (UChar*)INIT_SAO_MERGE_FLAG );
      curCost += m_cSaoTypeIdxSCModel.calcCost                 ( curSliceType, qp, (UChar*)INIT_SAO_TYPE_IDX );
      curCost += m_cTransformSkipSCModel.calcCost              ( curSliceType, qp, (UChar*)INIT_TRANSFORMSKIP_FLAG );
      curCost += m_CUTransquantBypassFlagSCModel.calcCost      ( curSliceType, qp, (UChar*)INIT_CU_TRANSQUANT_BYPASS_FLAG );
      curCost += m_explicitRdpcmFlagSCModel.calcCost           ( curSliceType, qp, (UChar*)INIT_EXPLICIT_RDPCM_FLAG);
      curCost += m_explicitRdpcmDirSCModel.calcCost            ( curSliceType, qp, (UChar*)INIT_EXPLICIT_RDPCM_DIR);
      curCost += m_cCrossComponentPredictionSCModel.calcCost   ( curSliceType, qp, (UChar*)INIT_CROSS_COMPONENT_PREDICTION );
      curCost += m_ChromaQpAdjFlagSCModel.calcCost             ( curSliceType, qp, (UChar*)INIT_CHROMA_QP_ADJ_FLAG );
      curCost += m_ChromaQpAdjIdcSCModel.calcCost              ( curSliceType, qp, (UChar*)INIT_CHROMA_QP_ADJ_IDC );

      if (curCost < bestCost)
      {
        bestSliceType = curSliceType;
        bestCost      = curCost;
      }
    }
    return bestSliceType;
  }
  else
  {
    return I_SLICE;
  }
}

Void TEncSbac::codeVPS( const TComVPS* /*pcVPS*/ )
{
  assert (0);
  return;
}

Void TEncSbac::codeSPS( const TComSPS* /*pcSPS*/ )
{
  assert (0);
  return;
}

Void TEncSbac::codePPS( const TComPPS* /*pcPPS*/ )
{
  assert (0);
  return;
}

Void TEncSbac::codeSliceHeader( TComSlice* /*pcSlice*/ )
{
  assert (0);
  return;
}

Void TEncSbac::codeTilesWPPEntryPoint( TComSlice* /*pSlice*/ )
{
  assert (0);
  return;
}

Void TEncSbac::codeTerminatingBit( UInt uilsLast )
{
  m_pcBinIf->encodeBinTrm( uilsLast );
}

Void TEncSbac::codeSliceFinish()
{
  m_pcBinIf->finish();
}

Void TEncSbac::xWriteUnarySymbol( UInt uiSymbol, ContextModel* pcSCModel, Int iOffset )
{
  m_pcBinIf->encodeBin( uiSymbol ? 1 : 0, pcSCModel[0] );

  if( 0 == uiSymbol)
  {
    return;
  }

  while( uiSymbol-- )
  {
    m_pcBinIf->encodeBin( uiSymbol ? 1 : 0, pcSCModel[ iOffset ] );
  }

  return;
}

Void TEncSbac::xWriteUnaryMaxSymbol( UInt uiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol )
{
  if (uiMaxSymbol == 0)
  {
    return;
  }

  m_pcBinIf->encodeBin( uiSymbol ? 1 : 0, pcSCModel[ 0 ] );

  if ( uiSymbol == 0 )
  {
    return;
  }

  Bool bCodeLast = ( uiMaxSymbol > uiSymbol );

  while( --uiSymbol )
  {
    m_pcBinIf->encodeBin( 1, pcSCModel[ iOffset ] );
  }
  if( bCodeLast )
  {
    m_pcBinIf->encodeBin( 0, pcSCModel[ iOffset ] );
  }

  return;
}

Void TEncSbac::xWriteEpExGolomb( UInt uiSymbol, UInt uiCount )
{
  UInt bins = 0;
  Int numBins = 0;

  while( uiSymbol >= (UInt)(1<<uiCount) )
  {
    bins = 2 * bins + 1;
    numBins++;
    uiSymbol -= 1 << uiCount;
    uiCount  ++;
  }
  bins = 2 * bins + 0;
  numBins++;

  bins = (bins << uiCount) | uiSymbol;
  numBins += uiCount;

  assert( numBins <= 32 );
  m_pcBinIf->encodeBinsEP( bins, numBins );
}


/** Coding of coeff_abs_level_minus3
 * \param symbol                  value of coeff_abs_level_minus3
 * \param rParam                  reference to Rice parameter
 * \param useLimitedPrefixLength
 * \param maxLog2TrDynamicRange 
 */
Void TEncSbac::xWriteCoefRemainExGolomb ( UInt symbol, UInt &rParam, const Bool useLimitedPrefixLength, const Int maxLog2TrDynamicRange )
{
  Int codeNumber  = (Int)symbol;
  UInt length;

  if (codeNumber < (COEF_REMAIN_BIN_REDUCTION << rParam))
  {
    length = codeNumber>>rParam;
    m_pcBinIf->encodeBinsEP( (1<<(length+1))-2 , length+1);
    m_pcBinIf->encodeBinsEP((codeNumber%(1<<rParam)),rParam);
  }
  else if (useLimitedPrefixLength)
  {
    const UInt maximumPrefixLength = (32 - (COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange));

    UInt prefixLength = 0;
    UInt suffixLength = MAX_UINT;
    UInt codeValue    = (symbol >> rParam) - COEF_REMAIN_BIN_REDUCTION;

    if (codeValue >= ((1 << maximumPrefixLength) - 1))
    {
      prefixLength = maximumPrefixLength;
      suffixLength = maxLog2TrDynamicRange - rParam;
    }
    else
    {
      while (codeValue > ((2 << prefixLength) - 2))
      {
        prefixLength++;
      }

      suffixLength = prefixLength + 1; //+1 for the separator bit
    }

    const UInt suffix = codeValue - ((1 << prefixLength) - 1);

    const UInt totalPrefixLength = prefixLength + COEF_REMAIN_BIN_REDUCTION;
    const UInt prefix            = (1 << totalPrefixLength) - 1;
    const UInt rParamBitMask     = (1 << rParam) - 1;

    m_pcBinIf->encodeBinsEP(  prefix,                                        totalPrefixLength      ); //prefix
    m_pcBinIf->encodeBinsEP(((suffix << rParam) | (symbol & rParamBitMask)), (suffixLength + rParam)); //separator, suffix, and rParam bits
  }
  else
  {
    length = rParam;
    codeNumber  = codeNumber - ( COEF_REMAIN_BIN_REDUCTION << rParam);

    while (codeNumber >= (1<<length))
    {
      codeNumber -=  (1<<(length++));
    }

    m_pcBinIf->encodeBinsEP((1<<(COEF_REMAIN_BIN_REDUCTION+length+1-rParam))-2,COEF_REMAIN_BIN_REDUCTION+length+1-rParam);
    m_pcBinIf->encodeBinsEP(codeNumber,length);
  }
}

// SBAC RD
Void  TEncSbac::load ( const TEncSbac* pSrc)
{
  this->xCopyFrom(pSrc);
}

Void  TEncSbac::loadIntraDirMode( const TEncSbac* pSrc, const ChannelType chType )
{
  m_pcBinIf->copyState( pSrc->m_pcBinIf );
  if (isLuma(chType))
  {
    this->m_cCUIntraPredSCModel      .copyFrom( &pSrc->m_cCUIntraPredSCModel       );
  }
  else
  {
    this->m_cCUChromaPredSCModel     .copyFrom( &pSrc->m_cCUChromaPredSCModel      );
  }
}


Void  TEncSbac::store( TEncSbac* pDest) const
{
  pDest->xCopyFrom( this );
}


Void TEncSbac::xCopyFrom( const TEncSbac* pSrc )
{
  m_pcBinIf->copyState( pSrc->m_pcBinIf );
  xCopyContextsFrom(pSrc);
}

Void TEncSbac::codeMVPIdx ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  Int iSymbol = pcCU->getMVPIdx(eRefList, uiAbsPartIdx);
  Int iNum = AMVP_MAX_NUM_CANDS;

  xWriteUnaryMaxSymbol(iSymbol, m_cMVPIdxSCModel.get(0), 1, iNum-1);
}

Void TEncSbac::codePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  PartSize eSize         = pcCU->getPartitionSize( uiAbsPartIdx );
  const UInt log2DiffMaxMinCodingBlockSize = pcCU->getSlice()->getSPS()->getLog2DiffMaxMinCodingBlockSize();

  if ( pcCU->isIntra( uiAbsPartIdx ) )
  {
    if( uiDepth == log2DiffMaxMinCodingBlockSize )
    {
      m_pcBinIf->encodeBin( eSize == SIZE_2Nx2N? 1 : 0, m_cCUPartSizeSCModel.get( 0, 0, 0 ) );
    }
    return;
  }

  switch(eSize)
  {
    case SIZE_2Nx2N:
    {
      m_pcBinIf->encodeBin( 1, m_cCUPartSizeSCModel.get( 0, 0, 0) );
      break;
    }
    case SIZE_2NxN:
    case SIZE_2NxnU:
    case SIZE_2NxnD:
    {
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 0) );
      m_pcBinIf->encodeBin( 1, m_cCUPartSizeSCModel.get( 0, 0, 1) );
      if ( pcCU->getSlice()->getSPS()->getUseAMP() && uiDepth < log2DiffMaxMinCodingBlockSize )
      {
        if (eSize == SIZE_2NxN)
        {
          m_pcBinIf->encodeBin(1, m_cCUPartSizeSCModel.get( 0, 0, 3 ));
        }
        else
        {
          m_pcBinIf->encodeBin(0, m_cCUPartSizeSCModel.get( 0, 0, 3 ));
          m_pcBinIf->encodeBinEP((eSize == SIZE_2NxnU? 0: 1));
        }
      }
      break;
    }
    case SIZE_Nx2N:
    case SIZE_nLx2N:
    case SIZE_nRx2N:
    {
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 0) );
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 1) );

      if( uiDepth == log2DiffMaxMinCodingBlockSize && !( pcCU->getWidth(uiAbsPartIdx) == 8 && pcCU->getHeight(uiAbsPartIdx) == 8 ) )
      {
        m_pcBinIf->encodeBin( 1, m_cCUPartSizeSCModel.get( 0, 0, 2) );
      }

      if ( pcCU->getSlice()->getSPS()->getUseAMP() && uiDepth < log2DiffMaxMinCodingBlockSize )
      {
        if (eSize == SIZE_Nx2N)
        {
          m_pcBinIf->encodeBin(1, m_cCUPartSizeSCModel.get( 0, 0, 3 ));
        }
        else
        {
          m_pcBinIf->encodeBin(0, m_cCUPartSizeSCModel.get( 0, 0, 3 ));
          m_pcBinIf->encodeBinEP((eSize == SIZE_nLx2N? 0: 1));
        }
      }
      break;
    }
    case SIZE_NxN:
    {
      if( uiDepth == log2DiffMaxMinCodingBlockSize && !( pcCU->getWidth(uiAbsPartIdx) == 8 && pcCU->getHeight(uiAbsPartIdx) == 8 ) )
      {
        m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 0) );
        m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 1) );
        m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 2) );
      }
      break;
    }
    default:
    {
      assert(0);
      break;
    }
  }
}


/** code prediction mode
 * \param pcCU
 * \param uiAbsPartIdx
 * \returns Void
 */
Void TEncSbac::codePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  // get context function is here
  m_pcBinIf->encodeBin( pcCU->isIntra( uiAbsPartIdx ) ? 1 : 0, m_cCUPredModeSCModel.get( 0, 0, 0 ) );
}

Void TEncSbac::codeCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiSymbol = pcCU->getCUTransquantBypass(uiAbsPartIdx);
  m_pcBinIf->encodeBin( uiSymbol, m_CUTransquantBypassFlagSCModel.get( 0, 0, 0 ) );
}

/** code skip flag
 * \param pcCU
 * \param uiAbsPartIdx
 * \returns Void
 */
Void TEncSbac::codeSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  // get context function is here
  UInt uiSymbol = pcCU->isSkipped( uiAbsPartIdx ) ? 1 : 0;
  UInt uiCtxSkip = pcCU->getCtxSkipFlag( uiAbsPartIdx ) ;
  m_pcBinIf->encodeBin( uiSymbol, m_cCUSkipFlagSCModel.get( 0, 0, uiCtxSkip ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tSkipFlag" );
  DTRACE_CABAC_T( "\tuiCtxSkip: ");
  DTRACE_CABAC_V( uiCtxSkip );
  DTRACE_CABAC_T( "\tuiSymbol: ");
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\n");
}

/** code merge flag
 * \param pcCU
 * \param uiAbsPartIdx
 * \returns Void
 */
Void TEncSbac::codeMergeFlag( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  const UInt uiSymbol = pcCU->getMergeFlag( uiAbsPartIdx ) ? 1 : 0;
  m_pcBinIf->encodeBin( uiSymbol, *m_cCUMergeFlagExtSCModel.get( 0 ) );

  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tMergeFlag: " );
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\tAddress: " );
  DTRACE_CABAC_V( pcCU->getCtuRsAddr() );
  DTRACE_CABAC_T( "\tuiAbsPartIdx: " );
  DTRACE_CABAC_V( uiAbsPartIdx );
  DTRACE_CABAC_T( "\n" );
}

/** code merge index
 * \param pcCU
 * \param uiAbsPartIdx
 * \returns Void
 */
Void TEncSbac::codeMergeIndex( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiUnaryIdx = pcCU->getMergeIndex( uiAbsPartIdx );
  UInt uiNumCand = pcCU->getSlice()->getMaxNumMergeCand();
  if ( uiNumCand > 1 )
  {
    for( UInt ui = 0; ui < uiNumCand - 1; ++ui )
    {
      const UInt uiSymbol = ui == uiUnaryIdx ? 0 : 1;
      if ( ui==0 )
      {
        m_pcBinIf->encodeBin( uiSymbol, m_cCUMergeIdxExtSCModel.get( 0, 0, 0 ) );
      }
      else
      {
        m_pcBinIf->encodeBinEP( uiSymbol );
      }
      if( uiSymbol == 0 )
      {
        break;
      }
    }
  }
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tparseMergeIndex()" );
  DTRACE_CABAC_T( "\tuiMRGIdx= " );
  DTRACE_CABAC_V( pcCU->getMergeIndex( uiAbsPartIdx ) );
  DTRACE_CABAC_T( "\n" );
}

Void TEncSbac::codeSplitFlag   ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( uiDepth == pcCU->getSlice()->getSPS()->getLog2DiffMaxMinCodingBlockSize() )
  {
    return;
  }

  UInt uiCtx           = pcCU->getCtxSplitFlag( uiAbsPartIdx, uiDepth );
  UInt uiCurrSplitFlag = ( pcCU->getDepth( uiAbsPartIdx ) > uiDepth ) ? 1 : 0;

  assert( uiCtx < 3 );
  m_pcBinIf->encodeBin( uiCurrSplitFlag, m_cCUSplitFlagSCModel.get( 0, 0, uiCtx ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tSplitFlag\n" )
  return;
}

Void TEncSbac::codeTransformSubdivFlag( UInt uiSymbol, UInt uiCtx )
{
  m_pcBinIf->encodeBin( uiSymbol, m_cCUTransSubdivFlagSCModel.get( 0, 0, uiCtx ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseTransformSubdivFlag()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiSymbol )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\n" )
}


Void TEncSbac::codeIntraDirLumaAng( TComDataCU* pcCU, UInt absPartIdx, Bool isMultiple)
{
  UInt dir[4],j;
  Int preds[4][NUM_MOST_PROBABLE_MODES] = {{-1, -1, -1},{-1, -1, -1},{-1, -1, -1},{-1, -1, -1}};
  Int predIdx[4] ={ -1,-1,-1,-1};
  PartSize mode = pcCU->getPartitionSize( absPartIdx );
  UInt partNum = isMultiple?(mode==SIZE_NxN?4:1):1;
  UInt partOffset = ( pcCU->getPic()->getNumPartitionsInCtu() >> ( pcCU->getDepth(absPartIdx) << 1 ) ) >> 2;
  for (j=0;j<partNum;j++)
  {
    dir[j] = pcCU->getIntraDir( CHANNEL_TYPE_LUMA, absPartIdx+partOffset*j );
    pcCU->getIntraDirPredictor(absPartIdx+partOffset*j, preds[j], COMPONENT_Y);
    for(UInt i = 0; i < NUM_MOST_PROBABLE_MODES; i++)
    {
      if(dir[j] == preds[j][i])
      {
        predIdx[j] = i;
      }
    }
    m_pcBinIf->encodeBin((predIdx[j] != -1)? 1 : 0, m_cCUIntraPredSCModel.get( 0, 0, 0 ) );
  }
  for (j=0;j<partNum;j++)
  {
    if(predIdx[j] != -1)
    {
      m_pcBinIf->encodeBinEP( predIdx[j] ? 1 : 0 );
      if (predIdx[j])
      {
        m_pcBinIf->encodeBinEP( predIdx[j]-1 );
      }
    }
    else
    {
      if (preds[j][0] > preds[j][1])
      {
        std::swap(preds[j][0], preds[j][1]);
      }
      if (preds[j][0] > preds[j][2])
      {
        std::swap(preds[j][0], preds[j][2]);
      }
      if (preds[j][1] > preds[j][2])
      {
        std::swap(preds[j][1], preds[j][2]);
      }
      for(Int i = (Int(NUM_MOST_PROBABLE_MODES) - 1); i >= 0; i--)
      {
        dir[j] = dir[j] > preds[j][i] ? dir[j] - 1 : dir[j];
      }
      m_pcBinIf->encodeBinsEP( dir[j], 5 );
    }
  }
  return;
}

Void TEncSbac::codeIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiIntraDirChroma = pcCU->getIntraDir( CHANNEL_TYPE_CHROMA, uiAbsPartIdx );

  if( uiIntraDirChroma == DM_CHROMA_IDX )
  {
    m_pcBinIf->encodeBin( 0, m_cCUChromaPredSCModel.get( 0, 0, 0 ) );
  }
  else
  {
    m_pcBinIf->encodeBin( 1, m_cCUChromaPredSCModel.get( 0, 0, 0 ) );

    UInt uiAllowedChromaDir[ NUM_CHROMA_MODE ];
    pcCU->getAllowedChromaDir( uiAbsPartIdx, uiAllowedChromaDir );

    for( Int i = 0; i < NUM_CHROMA_MODE - 1; i++ )
    {
      if( uiIntraDirChroma == uiAllowedChromaDir[i] )
      {
        uiIntraDirChroma = i;
        break;
      }
    }

    m_pcBinIf->encodeBinsEP( uiIntraDirChroma, 2 );
  }

  return;
}


Void TEncSbac::codeInterDir( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  const UInt uiInterDir = pcCU->getInterDir( uiAbsPartIdx ) - 1;
  const UInt uiCtx      = pcCU->getCtxInterDir( uiAbsPartIdx );
  ContextModel *pCtx    = m_cCUInterDirSCModel.get( 0 );

  if (pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2Nx2N || pcCU->getHeight(uiAbsPartIdx) != 8 )
  {
    m_pcBinIf->encodeBin( uiInterDir == 2 ? 1 : 0, *( pCtx + uiCtx ) );
  }

  if (uiInterDir < 2)
  {
    m_pcBinIf->encodeBin( uiInterDir, *( pCtx + 4 ) );
  }

  return;
}

Void TEncSbac::codeRefFrmIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  Int iRefFrame = pcCU->getCUMvField( eRefList )->getRefIdx( uiAbsPartIdx );
  ContextModel *pCtx = m_cCURefPicSCModel.get( 0 );
  m_pcBinIf->encodeBin( ( iRefFrame == 0 ? 0 : 1 ), *pCtx );

  if( iRefFrame > 0 )
  {
    UInt uiRefNum = pcCU->getSlice()->getNumRefIdx( eRefList ) - 2;
    pCtx++;
    iRefFrame--;
    for( UInt ui = 0; ui < uiRefNum; ++ui )
    {
      const UInt uiSymbol = ui == iRefFrame ? 0 : 1;
      if( ui == 0 )
      {
        m_pcBinIf->encodeBin( uiSymbol, *pCtx );
      }
      else
      {
        m_pcBinIf->encodeBinEP( uiSymbol );
      }
      if( uiSymbol == 0 )
      {
        break;
      }
    }
  }
  return;
}

Void TEncSbac::codeMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  if(pcCU->getSlice()->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pcCU->getInterDir(uiAbsPartIdx)==3)
  {
    return;
  }

  const TComCUMvField* pcCUMvField = pcCU->getCUMvField( eRefList );
  const Int iHor = pcCUMvField->getMvd( uiAbsPartIdx ).getHor();
  const Int iVer = pcCUMvField->getMvd( uiAbsPartIdx ).getVer();
  ContextModel* pCtx = m_cCUMvdSCModel.get( 0 );

  m_pcBinIf->encodeBin( iHor != 0 ? 1 : 0, *pCtx );
  m_pcBinIf->encodeBin( iVer != 0 ? 1 : 0, *pCtx );

  const Bool bHorAbsGr0 = iHor != 0;
  const Bool bVerAbsGr0 = iVer != 0;
  const UInt uiHorAbs   = 0 > iHor ? -iHor : iHor;
  const UInt uiVerAbs   = 0 > iVer ? -iVer : iVer;
  pCtx++;

  if( bHorAbsGr0 )
  {
    m_pcBinIf->encodeBin( uiHorAbs > 1 ? 1 : 0, *pCtx );
  }

  if( bVerAbsGr0 )
  {
    m_pcBinIf->encodeBin( uiVerAbs > 1 ? 1 : 0, *pCtx );
  }

  if( bHorAbsGr0 )
  {
    if( uiHorAbs > 1 )
    {
      xWriteEpExGolomb( uiHorAbs-2, 1 );
    }

    m_pcBinIf->encodeBinEP( 0 > iHor ? 1 : 0 );
  }

  if( bVerAbsGr0 )
  {
    if( uiVerAbs > 1 )
    {
      xWriteEpExGolomb( uiVerAbs-2, 1 );
    }

    m_pcBinIf->encodeBinEP( 0 > iVer ? 1 : 0 );
  }

  return;
}

Void TEncSbac::codeCrossComponentPrediction( TComTU &rTu, ComponentID compID )
{
  TComDataCU *pcCU = rTu.getCU();

  if( isLuma(compID) || !pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() )
  {
    return;
  }

  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();

  if (!pcCU->isIntra(uiAbsPartIdx) || (pcCU->getIntraDir( CHANNEL_TYPE_CHROMA, uiAbsPartIdx ) == DM_CHROMA_IDX))
  {
    DTRACE_CABAC_VL( g_nSymbolCounter++ )
    DTRACE_CABAC_T("\tparseCrossComponentPrediction()")
    DTRACE_CABAC_T( "\tAddr=" )
    DTRACE_CABAC_V( compID )
    DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
    DTRACE_CABAC_V( uiAbsPartIdx )

    Int alpha = pcCU->getCrossComponentPredictionAlpha( uiAbsPartIdx, compID );
    ContextModel *pCtx = m_cCrossComponentPredictionSCModel.get(0, 0) + ((compID == COMPONENT_Cr) ? (NUM_CROSS_COMPONENT_PREDICTION_CTX >> 1) : 0);
    m_pcBinIf->encodeBin(((alpha != 0) ? 1 : 0), pCtx[0]);

    if (alpha != 0)
    {
      static const Int log2AbsAlphaMinus1Table[8] = { 0, 1, 1, 2, 2, 2, 3, 3 };
      assert(abs(alpha) <= 8);

      if (abs(alpha)>1)
      {
        m_pcBinIf->encodeBin(1, pCtx[1]);
        xWriteUnaryMaxSymbol( log2AbsAlphaMinus1Table[abs(alpha) - 1] - 1, (pCtx + 2), 1, 2 );
      }
      else
      {
        m_pcBinIf->encodeBin(0, pCtx[1]);
      }
      m_pcBinIf->encodeBin( ((alpha < 0) ? 1 : 0), pCtx[4] );
    }
    DTRACE_CABAC_T( "\tAlpha=" )
    DTRACE_CABAC_V( pcCU->getCrossComponentPredictionAlpha( uiAbsPartIdx, compID ) )
    DTRACE_CABAC_T( "\n" )
  }
}

Void TEncSbac::codeDeltaQP( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  Int iDQp  = pcCU->getQP( uiAbsPartIdx ) - pcCU->getRefQP( uiAbsPartIdx );

  Int qpBdOffsetY =  pcCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
  iDQp = (iDQp + 78 + qpBdOffsetY + (qpBdOffsetY/2)) % (52 + qpBdOffsetY) - 26 - (qpBdOffsetY/2);

  UInt uiAbsDQp = (UInt)((iDQp > 0)? iDQp  : (-iDQp));
  UInt TUValue = min((Int)uiAbsDQp, CU_DQP_TU_CMAX);
  xWriteUnaryMaxSymbol( TUValue, &m_cCUDeltaQpSCModel.get( 0, 0, 0 ), 1, CU_DQP_TU_CMAX);
  if( uiAbsDQp >= CU_DQP_TU_CMAX )
  {
    xWriteEpExGolomb( uiAbsDQp - CU_DQP_TU_CMAX, CU_DQP_EG_k );
  }

  if ( uiAbsDQp > 0)
  {
    UInt uiSign = (iDQp > 0 ? 0 : 1);
    m_pcBinIf->encodeBinEP(uiSign);
  }

  return;
}

/** code chroma qp adjustment, converting from the internal table representation
 * \returns Void
 */
Void TEncSbac::codeChromaQpAdjustment( TComDataCU* cu, UInt absPartIdx )
{
  Int internalIdc = cu->getChromaQpAdj( absPartIdx );
  Int chromaQpOffsetListLen = cu->getSlice()->getPPS()->getPpsRangeExtension().getChromaQpOffsetListLen();
  /* internal_idc == 0 => flag = 0
   * internal_idc > 1 => code idc value (if table size warrents) */
  m_pcBinIf->encodeBin( internalIdc > 0, m_ChromaQpAdjFlagSCModel.get( 0, 0, 0 ) );

  if (internalIdc > 0 && chromaQpOffsetListLen > 1)
  {
    xWriteUnaryMaxSymbol( internalIdc - 1, &m_ChromaQpAdjIdcSCModel.get( 0, 0, 0 ), 0, chromaQpOffsetListLen - 1 );
  }
}

Void TEncSbac::codeQtCbf( TComTU &rTu, const ComponentID compID, const Bool lowestLevel )
{
  TComDataCU* pcCU = rTu.getCU();

  const UInt absPartIdx   = rTu.GetAbsPartIdxTU(compID);
  const UInt TUDepth      = rTu.GetTransformDepthRel();
        UInt uiCtx        = pcCU->getCtxQtCbf( rTu, toChannelType(compID) );
  const UInt contextSet   = toChannelType(compID);

  const UInt width        = rTu.getRect(compID).width;
  const UInt height       = rTu.getRect(compID).height;
  const Bool canQuadSplit = (width >= (MIN_TU_SIZE * 2)) && (height >= (MIN_TU_SIZE * 2));

  //             Since the CBF for chroma is coded at the highest level possible, if sub-TUs are
  //             to be coded for a 4x8 chroma TU, their CBFs must be coded at the highest 4x8 level
  //             (i.e. where luma TUs are 8x8 rather than 4x4)
  //    ___ ___
  //   |   |   | <- 4 x (8x8 luma + 4x8 4:2:2 chroma)
  //   |___|___|    each quadrant has its own chroma CBF
  //   |   |   | _ _ _ _
  //   |___|___|        |
  //   <--16--->        V
  //                   _ _
  //                  |_|_| <- 4 x 4x4 luma + 1 x 4x8 4:2:2 chroma
  //                  |_|_|    no chroma CBF is coded - instead the parent CBF is inherited
  //                  <-8->    if sub-TUs are present, their CBFs had to be coded at the parent level

  const UInt lowestTUDepth = TUDepth + ((!lowestLevel && !canQuadSplit) ? 1 : 0); //unsplittable TUs inherit their parent's CBF

  if ((width != height) && (lowestLevel || !canQuadSplit)) //if sub-TUs are present
  {
    const UInt subTUDepth        = lowestTUDepth + 1;                      //if this is the lowest level of the TU-tree, the sub-TUs are directly below. Otherwise, this must be the level above the lowest level (as specified above)
    const UInt partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID) >> 1;

    for (UInt subTU = 0; subTU < 2; subTU++)
    {
      const UInt subTUAbsPartIdx = absPartIdx + (subTU * partIdxesPerSubTU);
      const UInt uiCbf           = pcCU->getCbf(subTUAbsPartIdx, compID, subTUDepth);

      m_pcBinIf->encodeBin(uiCbf, m_cCUQtCbfSCModel.get(0, contextSet, uiCtx));

      DTRACE_CABAC_VL( g_nSymbolCounter++ )
      DTRACE_CABAC_T( "\tparseQtCbf()" )
      DTRACE_CABAC_T( "\tsub-TU=" )
      DTRACE_CABAC_V( subTU )
      DTRACE_CABAC_T( "\tsymbol=" )
      DTRACE_CABAC_V( uiCbf )
      DTRACE_CABAC_T( "\tctx=" )
      DTRACE_CABAC_V( uiCtx )
      DTRACE_CABAC_T( "\tetype=" )
      DTRACE_CABAC_V( compID )
      DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
      DTRACE_CABAC_V( subTUAbsPartIdx )
      DTRACE_CABAC_T( "\n" )
    }
  }
  else
  {
    const UInt uiCbf = pcCU->getCbf( absPartIdx, compID, lowestTUDepth );
    m_pcBinIf->encodeBin( uiCbf , m_cCUQtCbfSCModel.get( 0, contextSet, uiCtx ) );


    DTRACE_CABAC_VL( g_nSymbolCounter++ )
    DTRACE_CABAC_T( "\tparseQtCbf()" )
    DTRACE_CABAC_T( "\tsymbol=" )
    DTRACE_CABAC_V( uiCbf )
    DTRACE_CABAC_T( "\tctx=" )
    DTRACE_CABAC_V( uiCtx )
    DTRACE_CABAC_T( "\tetype=" )
    DTRACE_CABAC_V( compID )
    DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
    DTRACE_CABAC_V( rTu.GetAbsPartIdxTU(compID) )
    DTRACE_CABAC_T( "\n" )
  }
}


Void TEncSbac::codeTransformSkipFlags (TComTU &rTu, ComponentID component )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();

  if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    return;
  }

  if (!TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(component), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize()))
  {
    return;
  }

  UInt useTransformSkip = pcCU->getTransformSkip( uiAbsPartIdx,component);
  m_pcBinIf->encodeBin( useTransformSkip, m_cTransformSkipSCModel.get( 0, toChannelType(component), 0 ) );

  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T("\tparseTransformSkip()");
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( useTransformSkip )
  DTRACE_CABAC_T( "\tAddr=" )
  DTRACE_CABAC_V( pcCU->getCtuRsAddr() )
  DTRACE_CABAC_T( "\tetype=" )
  DTRACE_CABAC_V( component )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( rTu.GetAbsPartIdxTU() )
  DTRACE_CABAC_T( "\n" )
}


/** Code I_PCM information.
 * \param pcCU pointer to CU
 * \param uiAbsPartIdx CU index
 * \returns Void
 */
Void TEncSbac::codeIPCMInfo( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiIPCM = (pcCU->getIPCMFlag(uiAbsPartIdx) == true)? 1 : 0;

  Bool writePCMSampleFlag = pcCU->getIPCMFlag(uiAbsPartIdx);

  m_pcBinIf->encodeBinTrm (uiIPCM);

  if (writePCMSampleFlag)
  {
    m_pcBinIf->encodePCMAlignBits();

    const UInt minCoeffSizeY = pcCU->getPic()->getMinCUWidth() * pcCU->getPic()->getMinCUHeight();
    const UInt offsetY       = minCoeffSizeY * uiAbsPartIdx;
    for (UInt ch=0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      const ComponentID compID = ComponentID(ch);
      const UInt offset = offsetY >> (pcCU->getPic()->getComponentScaleX(compID) + pcCU->getPic()->getComponentScaleY(compID));
      Pel * pPCMSample  = pcCU->getPCMSample(compID) + offset;
      const UInt width  = pcCU->getWidth (uiAbsPartIdx) >> pcCU->getPic()->getComponentScaleX(compID);
      const UInt height = pcCU->getHeight(uiAbsPartIdx) >> pcCU->getPic()->getComponentScaleY(compID);
      const UInt sampleBits = pcCU->getSlice()->getSPS()->getPCMBitDepth(toChannelType(compID));
      for (UInt y=0; y<height; y++)
      {
        for (UInt x=0; x<width; x++)
        {
          UInt sample = pPCMSample[x];
          m_pcBinIf->xWritePCMCode(sample, sampleBits);
        }
        pPCMSample += width;
      }
    }

    m_pcBinIf->resetBac();
  }
}

Void TEncSbac::codeQtRootCbf( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiCbf = pcCU->getQtRootCbf( uiAbsPartIdx );
  UInt uiCtx = 0;
  m_pcBinIf->encodeBin( uiCbf , m_cCUQtRootCbfSCModel.get( 0, 0, uiCtx ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseQtRootCbf()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiCbf )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )
}

Void TEncSbac::codeQtCbfZero( TComTU & rTu, const ChannelType chType )
{
  // this function is only used to estimate the bits when cbf is 0
  // and will never be called when writing the bistream. do not need to write log
  UInt uiCbf = 0;
  UInt uiCtx = rTu.getCU()->getCtxQtCbf( rTu, chType );

  m_pcBinIf->encodeBin( uiCbf , m_cCUQtCbfSCModel.get( 0, chType, uiCtx ) );
}

Void TEncSbac::codeQtRootCbfZero( )
{
  // this function is only used to estimate the bits when cbf is 0
  // and will never be called when writing the bistream. do not need to write log
  UInt uiCbf = 0;
  UInt uiCtx = 0;
  m_pcBinIf->encodeBin( uiCbf , m_cCUQtRootCbfSCModel.get( 0, 0, uiCtx ) );
}

/** Encode (X,Y) position of the last significant coefficient
 * \param uiPosX     X component of last coefficient
 * \param uiPosY     Y component of last coefficient
 * \param width      Block width
 * \param height     Block height
 * \param component  chroma component ID
 * \param uiScanIdx  scan type (zig-zag, hor, ver)
 * This method encodes the X and Y component within a block of the last significant coefficient.
 */
Void TEncSbac::codeLastSignificantXY( UInt uiPosX, UInt uiPosY, Int width, Int height, ComponentID component, UInt uiScanIdx )
{
  // swap
  if( uiScanIdx == SCAN_VER )
  {
    swap( uiPosX, uiPosY );
    swap( width,  height );
  }

  UInt uiCtxLast;
  UInt uiGroupIdxX    = g_uiGroupIdx[ uiPosX ];
  UInt uiGroupIdxY    = g_uiGroupIdx[ uiPosY ];

  ContextModel *pCtxX = m_cCuCtxLastX.get( 0, toChannelType(component) );
  ContextModel *pCtxY = m_cCuCtxLastY.get( 0, toChannelType(component) );

  Int blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY;
  getLastSignificantContextParameters(component, width, height, blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY);

  //------------------

  // posX

  for( uiCtxLast = 0; uiCtxLast < uiGroupIdxX; uiCtxLast++ )
  {
    m_pcBinIf->encodeBin( 1, *( pCtxX + blkSizeOffsetX + (uiCtxLast >>shiftX) ) );
  }
  if( uiGroupIdxX < g_uiGroupIdx[ width - 1 ])
  {
    m_pcBinIf->encodeBin( 0, *( pCtxX + blkSizeOffsetX + (uiCtxLast >>shiftX) ) );
  }

  // posY

  for( uiCtxLast = 0; uiCtxLast < uiGroupIdxY; uiCtxLast++ )
  {
    m_pcBinIf->encodeBin( 1, *( pCtxY + blkSizeOffsetY + (uiCtxLast >>shiftY) ) );
  }
  if( uiGroupIdxY < g_uiGroupIdx[ height - 1 ])
  {
    m_pcBinIf->encodeBin( 0, *( pCtxY + blkSizeOffsetY + (uiCtxLast >>shiftY) ) );
  }

  // EP-coded part

  if ( uiGroupIdxX > 3 )
  {
    UInt uiCount = ( uiGroupIdxX - 2 ) >> 1;
    uiPosX       = uiPosX - g_uiMinInGroup[ uiGroupIdxX ];
    for (Int i = uiCount - 1 ; i >= 0; i-- )
    {
      m_pcBinIf->encodeBinEP( ( uiPosX >> i ) & 1 );
    }
  }
  if ( uiGroupIdxY > 3 )
  {
    UInt uiCount = ( uiGroupIdxY - 2 ) >> 1;
    uiPosY       = uiPosY - g_uiMinInGroup[ uiGroupIdxY ];
    for ( Int i = uiCount - 1 ; i >= 0; i-- )
    {
      m_pcBinIf->encodeBinEP( ( uiPosY >> i ) & 1 );
    }
  }
}


Void TEncSbac::codeCoeffNxN( TComTU &rTu, TCoeff* pcCoef, const ComponentID compID )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU(compID);
  const TComRectangle &tuRect=rTu.getRect(compID);
  const UInt uiWidth=tuRect.width;
  const UInt uiHeight=tuRect.height;
  const TComSPS &sps=*(pcCU->getSlice()->getSPS());

  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseCoeffNxN()\teType=" )
  DTRACE_CABAC_V( compID )
  DTRACE_CABAC_T( "\twidth=" )
  DTRACE_CABAC_V( uiWidth )
  DTRACE_CABAC_T( "\theight=" )
  DTRACE_CABAC_V( uiHeight )
  DTRACE_CABAC_T( "\tdepth=" )
//  DTRACE_CABAC_V( rTu.GetTransformDepthTotalAdj(compID) )
  DTRACE_CABAC_V( rTu.GetTransformDepthTotal() )
  DTRACE_CABAC_T( "\tabspartidx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\ttoCU-X=" )
  DTRACE_CABAC_V( pcCU->getCUPelX() )
  DTRACE_CABAC_T( "\ttoCU-Y=" )
  DTRACE_CABAC_V( pcCU->getCUPelY() )
  DTRACE_CABAC_T( "\tCU-addr=" )
  DTRACE_CABAC_V(  pcCU->getCtuRsAddr() )
  DTRACE_CABAC_T( "\tinCU-X=" )
//  DTRACE_CABAC_V( g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ] )
  DTRACE_CABAC_V( g_auiRasterToPelX[ g_auiZscanToRaster[rTu.GetAbsPartIdxTU(compID)] ] )
  DTRACE_CABAC_T( "\tinCU-Y=" )
// DTRACE_CABAC_V( g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ] )
  DTRACE_CABAC_V( g_auiRasterToPelY[ g_auiZscanToRaster[rTu.GetAbsPartIdxTU(compID)] ] )
  DTRACE_CABAC_T( "\tpredmode=" )
  DTRACE_CABAC_V(  pcCU->getPredictionMode( uiAbsPartIdx ) )
  DTRACE_CABAC_T( "\n" )

  //--------------------------------------------------------------------------------------------------

  if( uiWidth > sps.getMaxTrSize() )
  {
    std::cerr << "ERROR: codeCoeffNxN was passed a TU with dimensions larger than the maximum allowed size" << std::endl;
    assert(false);
    exit(1);
  }

  // compute number of significant coefficients
  UInt uiNumSig = TEncEntropy::countNonZeroCoeffs(pcCoef, uiWidth * uiHeight);

  if ( uiNumSig == 0 )
  {
    std::cerr << "ERROR: codeCoeffNxN called for empty TU!" << std::endl;
    assert(false);
    exit(1);
  }

  //--------------------------------------------------------------------------------------------------

  //set parameters

  const ChannelType  chType            = toChannelType(compID);
  const UInt         uiLog2BlockWidth  = g_aucConvertToBit[ uiWidth  ] + 2;
  const UInt         uiLog2BlockHeight = g_aucConvertToBit[ uiHeight ] + 2;

  const ChannelType  channelType       = toChannelType(compID);
  const Bool         extendedPrecision = sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag();

  const Bool         alignCABACBeforeBypass = sps.getSpsRangeExtension().getCabacBypassAlignmentEnabledFlag();
  const Int          maxLog2TrDynamicRange  = sps.getMaxLog2TrDynamicRange(channelType);

  Bool beValid;

  {
    Int uiIntraMode = -1;
    const Bool       bIsLuma = isLuma(compID);
    Int isIntra = pcCU->isIntra(uiAbsPartIdx) ? 1 : 0;
    if ( isIntra )
    {
      uiIntraMode = pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx );

      const UInt partsPerMinCU = 1<<(2*(sps.getMaxTotalCUDepth() - sps.getLog2DiffMaxMinCodingBlockSize()));
      uiIntraMode = (uiIntraMode==DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, rTu.GetChromaFormat(), partsPerMinCU)) : uiIntraMode;
      uiIntraMode = ((rTu.GetChromaFormat() == CHROMA_422) && !bIsLuma) ? g_chroma422IntraAngleMappingTable[uiIntraMode] : uiIntraMode;
    }

    Int transformSkip = pcCU->getTransformSkip( uiAbsPartIdx,compID) ? 1 : 0;
    Bool rdpcm_lossy = ( transformSkip && isIntra && ( (uiIntraMode == HOR_IDX) || (uiIntraMode == VER_IDX) ) ) && pcCU->isRDPCMEnabled(uiAbsPartIdx);

    if ( (pcCU->getCUTransquantBypass(uiAbsPartIdx)) || rdpcm_lossy )
    {
      beValid = false;
      if ( (!pcCU->isIntra(uiAbsPartIdx)) && pcCU->isRDPCMEnabled(uiAbsPartIdx))
      {
        codeExplicitRdpcmMode( rTu, compID);
      }
    }
    else
    {
      beValid = pcCU->getSlice()->getPPS()->getSignHideFlag();
    }
  }

  //--------------------------------------------------------------------------------------------------

  if(pcCU->getSlice()->getPPS()->getUseTransformSkip())
  {
    codeTransformSkipFlags(rTu, compID);
    if(pcCU->getTransformSkip(uiAbsPartIdx, compID) && !pcCU->isIntra(uiAbsPartIdx) && pcCU->isRDPCMEnabled(uiAbsPartIdx))
    {
      //  This TU has coefficients and is transform skipped. Check whether is inter coded and if yes encode the explicit RDPCM mode
      codeExplicitRdpcmMode( rTu, compID);

      if(pcCU->getExplicitRdpcmMode(compID, uiAbsPartIdx) != RDPCM_OFF)
      {
        //  Sign data hiding is avoided for horizontal and vertical explicit RDPCM modes
        beValid = false;
      }
    }
  }

  //--------------------------------------------------------------------------------------------------

  const Bool  bUseGolombRiceParameterAdaptation = sps.getSpsRangeExtension().getPersistentRiceAdaptationEnabledFlag();
        UInt &currentGolombRiceStatistic        = m_golombRiceAdaptationStatistics[rTu.getGolombRiceStatisticsIndex(compID)];

  //select scans
  TUEntropyCodingParameters codingParameters;
  getTUEntropyCodingParameters(codingParameters, rTu, compID);

  //----- encode significance map -----

  // Find position of last coefficient
  Int scanPosLast = -1;
  Int posLast;


  UInt uiSigCoeffGroupFlag[ MLS_GRP_NUM ];

  memset( uiSigCoeffGroupFlag, 0, sizeof(UInt) * MLS_GRP_NUM );
  do
  {
    posLast = codingParameters.scan[ ++scanPosLast ];

    if( pcCoef[ posLast ] != 0 )
    {
      // get L1 sig map
      UInt uiPosY   = posLast >> uiLog2BlockWidth;
      UInt uiPosX   = posLast - ( uiPosY << uiLog2BlockWidth );

      UInt uiBlkIdx = (codingParameters.widthInGroups * (uiPosY >> MLS_CG_LOG2_HEIGHT)) + (uiPosX >> MLS_CG_LOG2_WIDTH);
      uiSigCoeffGroupFlag[ uiBlkIdx ] = 1;

      uiNumSig--;
    }
  } while ( uiNumSig > 0 );

  // Code position of last coefficient
  Int posLastY = posLast >> uiLog2BlockWidth;
  Int posLastX = posLast - ( posLastY << uiLog2BlockWidth );
  codeLastSignificantXY(posLastX, posLastY, uiWidth, uiHeight, compID, codingParameters.scanType);

  //===== code significance flag =====
  ContextModel * const baseCoeffGroupCtx = m_cCUSigCoeffGroupSCModel.get( 0, chType );
  ContextModel * const baseCtx = m_cCUSigSCModel.get( 0, 0 ) + getSignificanceMapContextOffset(compID);

  const Int  iLastScanSet  = scanPosLast >> MLS_CG_SIZE;

  UInt c1                  = 1;
  UInt uiGoRiceParam       = 0;
  Int  iScanPosSig         = scanPosLast;

  for( Int iSubSet = iLastScanSet; iSubSet >= 0; iSubSet-- )
  {
    Int numNonZero = 0;
    Int  iSubPos   = iSubSet << MLS_CG_SIZE;
    uiGoRiceParam  = currentGolombRiceStatistic / RExt__GOLOMB_RICE_INCREMENT_DIVISOR;
    Bool updateGolombRiceStatistics = bUseGolombRiceParameterAdaptation; //leave the statistics at 0 when not using the adaptation system
    UInt coeffSigns = 0;

    Int absCoeff[1 << MLS_CG_SIZE];

    Int lastNZPosInCG  = -1;
    Int firstNZPosInCG = 1 << MLS_CG_SIZE;

    Bool escapeDataPresentInGroup = false;

    if( iScanPosSig == scanPosLast )
    {
      absCoeff[ 0 ] = Int(abs( pcCoef[ posLast ] ));
      coeffSigns    = ( pcCoef[ posLast ] < 0 );
      numNonZero    = 1;
      lastNZPosInCG  = iScanPosSig;
      firstNZPosInCG = iScanPosSig;
      iScanPosSig--;
    }

    // encode significant_coeffgroup_flag
    Int iCGBlkPos = codingParameters.scanCG[ iSubSet ];
    Int iCGPosY   = iCGBlkPos / codingParameters.widthInGroups;
    Int iCGPosX   = iCGBlkPos - (iCGPosY * codingParameters.widthInGroups);

    if( iSubSet == iLastScanSet || iSubSet == 0)
    {
      uiSigCoeffGroupFlag[ iCGBlkPos ] = 1;
    }
    else
    {
      UInt uiSigCoeffGroup   = (uiSigCoeffGroupFlag[ iCGBlkPos ] != 0);
      UInt uiCtxSig  = TComTrQuant::getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, iCGPosX, iCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups );
      m_pcBinIf->encodeBin( uiSigCoeffGroup, baseCoeffGroupCtx[ uiCtxSig ] );
    }

    // encode significant_coeff_flag
    if( uiSigCoeffGroupFlag[ iCGBlkPos ] )
    {
      const Int patternSigCtx = TComTrQuant::calcPatternSigCtx(uiSigCoeffGroupFlag, iCGPosX, iCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups);

      UInt uiBlkPos, uiSig, uiCtxSig;
      for( ; iScanPosSig >= iSubPos; iScanPosSig-- )
      {
        uiBlkPos  = codingParameters.scan[ iScanPosSig ];
        uiSig     = (pcCoef[ uiBlkPos ] != 0);
        if( iScanPosSig > iSubPos || iSubSet == 0 || numNonZero )
        {
          uiCtxSig  = TComTrQuant::getSigCtxInc( patternSigCtx, codingParameters, iScanPosSig, uiLog2BlockWidth, uiLog2BlockHeight, chType );
          m_pcBinIf->encodeBin( uiSig, baseCtx[ uiCtxSig ] );
        }
        if( uiSig )
        {
          absCoeff[ numNonZero ] = Int(abs( pcCoef[ uiBlkPos ] ));
          coeffSigns = 2 * coeffSigns + ( pcCoef[ uiBlkPos ] < 0 );
          numNonZero++;
          if( lastNZPosInCG == -1 )
          {
            lastNZPosInCG = iScanPosSig;
          }
          firstNZPosInCG = iScanPosSig;
        }
      }
    }
    else
    {
      iScanPosSig = iSubPos - 1;
    }

    if( numNonZero > 0 )
    {
      Bool signHidden = ( lastNZPosInCG - firstNZPosInCG >= SBH_THRESHOLD );

      const UInt uiCtxSet = getContextSetIndex(compID, iSubSet, (c1 == 0));
      c1 = 1;

      ContextModel *baseCtxMod = m_cCUOneSCModel.get( 0, 0 ) + (NUM_ONE_FLAG_CTX_PER_SET * uiCtxSet);

      Int numC1Flag = min(numNonZero, C1FLAG_NUMBER);
      Int firstC2FlagIdx = -1;
      for( Int idx = 0; idx < numC1Flag; idx++ )
      {
        UInt uiSymbol = absCoeff[ idx ] > 1;
        m_pcBinIf->encodeBin( uiSymbol, baseCtxMod[c1] );
        if( uiSymbol )
        {
          c1 = 0;

          if (firstC2FlagIdx == -1)
          {
            firstC2FlagIdx = idx;
          }
          else //if a greater-than-one has been encountered already this group
          {
            escapeDataPresentInGroup = true;
          }
        }
        else if( (c1 < 3) && (c1 > 0) )
        {
          c1++;
        }
      }

      if (c1 == 0)
      {
        baseCtxMod = m_cCUAbsSCModel.get( 0, 0 ) + (NUM_ABS_FLAG_CTX_PER_SET * uiCtxSet);
        if ( firstC2FlagIdx != -1)
        {
          UInt symbol = absCoeff[ firstC2FlagIdx ] > 2;
          m_pcBinIf->encodeBin( symbol, baseCtxMod[0] );
          if (symbol != 0)
          {
            escapeDataPresentInGroup = true;
          }
        }
      }

      escapeDataPresentInGroup = escapeDataPresentInGroup || (numNonZero > C1FLAG_NUMBER);

      if (escapeDataPresentInGroup && alignCABACBeforeBypass)
      {
        m_pcBinIf->align();
      }

      if( beValid && signHidden )
      {
        m_pcBinIf->encodeBinsEP( (coeffSigns >> 1), numNonZero-1 );
      }
      else
      {
        m_pcBinIf->encodeBinsEP( coeffSigns, numNonZero );
      }

      Int iFirstCoeff2 = 1;
      if (escapeDataPresentInGroup)
      {
        for ( Int idx = 0; idx < numNonZero; idx++ )
        {
          UInt baseLevel  = (idx < C1FLAG_NUMBER)? (2 + iFirstCoeff2 ) : 1;

          if( absCoeff[ idx ] >= baseLevel)
          {
            const UInt escapeCodeValue = absCoeff[idx] - baseLevel;

            xWriteCoefRemainExGolomb( escapeCodeValue, uiGoRiceParam, extendedPrecision, maxLog2TrDynamicRange );

            if (absCoeff[idx] > (3 << uiGoRiceParam))
            {
              uiGoRiceParam = bUseGolombRiceParameterAdaptation ? (uiGoRiceParam + 1) : (std::min<UInt>((uiGoRiceParam + 1), 4));
            }

            if (updateGolombRiceStatistics)
            {
              const UInt initialGolombRiceParameter = currentGolombRiceStatistic / RExt__GOLOMB_RICE_INCREMENT_DIVISOR;

              if (escapeCodeValue >= (3 << initialGolombRiceParameter))
              {
                currentGolombRiceStatistic++;
              }
              else if (((escapeCodeValue * 2) < (1 << initialGolombRiceParameter)) && (currentGolombRiceStatistic > 0))
              {
                currentGolombRiceStatistic--;
              }

              updateGolombRiceStatistics = false;
            }
          }

          if(absCoeff[ idx ] >= 2)
          {
            iFirstCoeff2 = 0;
          }
        }
      }
    }
  }
#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  printSBACCoeffData(posLastX, posLastY, uiWidth, uiHeight, compID, uiAbsPartIdx, codingParameters.scanType, pcCoef, pcCU->getSlice()->getFinalized());
#endif

  return;
}

/** code SAO offset sign
 * \param code sign value
 */
Void TEncSbac::codeSAOSign( UInt code )
{
  m_pcBinIf->encodeBinEP( code );
}

Void TEncSbac::codeSaoMaxUvlc    ( UInt code, UInt maxSymbol )
{
  if (maxSymbol == 0)
  {
    return;
  }

  Int i;
  Bool bCodeLast = ( maxSymbol > code );

  if ( code == 0 )
  {
    m_pcBinIf->encodeBinEP( 0 );
  }
  else
  {
    m_pcBinIf->encodeBinEP( 1 );
    for ( i=0; i<code-1; i++ )
    {
      m_pcBinIf->encodeBinEP( 1 );
    }
    if( bCodeLast )
    {
      m_pcBinIf->encodeBinEP( 0 );
    }
  }
}

/** Code SAO EO class or BO band position
 */
Void TEncSbac::codeSaoUflc       ( UInt uiLength, UInt uiCode )
{
  m_pcBinIf->encodeBinsEP ( uiCode, uiLength );
}

/** Code SAO merge flags
 */
Void TEncSbac::codeSaoMerge       ( UInt uiCode )
{
  m_pcBinIf->encodeBin(((uiCode == 0) ? 0 : 1),  m_cSaoMergeSCModel.get( 0, 0, 0 ));
}

/** Code SAO type index
 */
Void TEncSbac::codeSaoTypeIdx       ( UInt uiCode)
{
  if (uiCode == 0)
  {
    m_pcBinIf->encodeBin( 0, m_cSaoTypeIdxSCModel.get( 0, 0, 0 ) );
  }
  else
  {
    m_pcBinIf->encodeBin( 1, m_cSaoTypeIdxSCModel.get( 0, 0, 0 ) );
    m_pcBinIf->encodeBinEP( uiCode == 1 ? 0 : 1 );
  }
}

Void TEncSbac::codeSAOOffsetParam(ComponentID compIdx, SAOOffset& ctbParam, Bool sliceEnabled, const Int channelBitDepth)
{
  UInt uiSymbol;
  if(!sliceEnabled)
  {
    assert(ctbParam.modeIdc == SAO_MODE_OFF);
    return;
  }
  const Bool bIsFirstCompOfChType = (getFirstComponentOfChannel(toChannelType(compIdx)) == compIdx);

  //type
  if(bIsFirstCompOfChType)
  {
    //sao_type_idx_luma or sao_type_idx_chroma
    if(ctbParam.modeIdc == SAO_MODE_OFF)
    {
      uiSymbol =0;
    }
    else if(ctbParam.typeIdc == SAO_TYPE_BO) //BO
    {
      uiSymbol = 1;
    }
    else
    {
      assert(ctbParam.typeIdc < SAO_TYPE_START_BO); //EO
      uiSymbol = 2;
    }
    codeSaoTypeIdx(uiSymbol);
  }

  if(ctbParam.modeIdc == SAO_MODE_NEW)
  {
    Int numClasses = (ctbParam.typeIdc == SAO_TYPE_BO)?4:NUM_SAO_EO_CLASSES;
    Int offset[4];
    Int k=0;
    for(Int i=0; i< numClasses; i++)
    {
      if(ctbParam.typeIdc != SAO_TYPE_BO && i == SAO_CLASS_EO_PLAIN)
      {
        continue;
      }
      Int classIdx = (ctbParam.typeIdc == SAO_TYPE_BO)?(  (ctbParam.typeAuxInfo+i)% NUM_SAO_BO_CLASSES   ):i;
      offset[k] = ctbParam.offset[classIdx];
      k++;
    }

    const Int  maxOffsetQVal = TComSampleAdaptiveOffset::getMaxOffsetQVal(channelBitDepth);
    for(Int i=0; i< 4; i++)
    {
      codeSaoMaxUvlc((offset[i]<0)?(-offset[i]):(offset[i]),  maxOffsetQVal ); //sao_offset_abs
    }


    if(ctbParam.typeIdc == SAO_TYPE_BO)
    {
      for(Int i=0; i< 4; i++)
      {
        if(offset[i] != 0)
        {
          codeSAOSign((offset[i]< 0)?1:0);
        }
      }

      codeSaoUflc(NUM_SAO_BO_CLASSES_LOG2, ctbParam.typeAuxInfo ); //sao_band_position
    }
    else //EO
    {
      if(bIsFirstCompOfChType)
      {
        assert(ctbParam.typeIdc - SAO_TYPE_START_EO >=0);
        codeSaoUflc(NUM_SAO_EO_TYPES_LOG2, ctbParam.typeIdc - SAO_TYPE_START_EO ); //sao_eo_class_luma or sao_eo_class_chroma
      }
    }

  }
}


Void TEncSbac::codeSAOBlkParam(SAOBlkParam& saoBlkParam, const BitDepths &bitDepths
                              , Bool* sliceEnabled
                              , Bool leftMergeAvail
                              , Bool aboveMergeAvail
                              , Bool onlyEstMergeInfo // = false
                              )
{

  Bool isLeftMerge = false;
  Bool isAboveMerge= false;

  if(leftMergeAvail)
  {
    isLeftMerge = ((saoBlkParam[COMPONENT_Y].modeIdc == SAO_MODE_MERGE) && (saoBlkParam[COMPONENT_Y].typeIdc == SAO_MERGE_LEFT));
    codeSaoMerge( isLeftMerge?1:0  ); //sao_merge_left_flag
  }

  if( aboveMergeAvail && !isLeftMerge)
  {
    isAboveMerge = ((saoBlkParam[COMPONENT_Y].modeIdc == SAO_MODE_MERGE) && (saoBlkParam[COMPONENT_Y].typeIdc == SAO_MERGE_ABOVE));
    codeSaoMerge( isAboveMerge?1:0  ); //sao_merge_left_flag
  }

  if(onlyEstMergeInfo)
  {
    return; //only for RDO
  }

  if(!isLeftMerge && !isAboveMerge) //not merge mode
  {
    for(Int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      codeSAOOffsetParam(ComponentID(compIdx), saoBlkParam[compIdx], sliceEnabled[compIdx], bitDepths.recon[toChannelType(ComponentID(compIdx))]);
    }
  }
}

/*!
 ****************************************************************************
 * \brief
 *   estimate bit cost for CBP, significant map and significant coefficients
 ****************************************************************************
 */
Void TEncSbac::estBit( estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, ChannelType chType )
{
  estCBFBit( pcEstBitsSbac );

  estSignificantCoeffGroupMapBit( pcEstBitsSbac, chType );

  // encode significance map
  estSignificantMapBit( pcEstBitsSbac, width, height, chType );

  // encode last significant position
  estLastSignificantPositionBit( pcEstBitsSbac, width, height, chType );

  // encode significant coefficients
  estSignificantCoefficientsBit( pcEstBitsSbac, chType );

  memcpy(pcEstBitsSbac->golombRiceAdaptationStatistics, m_golombRiceAdaptationStatistics, (sizeof(UInt) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS));
}

/*!
 ****************************************************************************
 * \brief
 *    estimate bit cost for each CBP bit
 ****************************************************************************
 */
Void TEncSbac::estCBFBit( estBitsSbacStruct* pcEstBitsSbac )
{
  ContextModel *pCtx = m_cCUQtCbfSCModel.get( 0 );

  for( UInt uiCtxInc = 0; uiCtxInc < (NUM_QT_CBF_CTX_SETS * NUM_QT_CBF_CTX_PER_SET); uiCtxInc++ )
  {
    pcEstBitsSbac->blockCbpBits[ uiCtxInc ][ 0 ] = pCtx[ uiCtxInc ].getEntropyBits( 0 );
    pcEstBitsSbac->blockCbpBits[ uiCtxInc ][ 1 ] = pCtx[ uiCtxInc ].getEntropyBits( 1 );
  }

  pCtx = m_cCUQtRootCbfSCModel.get( 0 );

  for( UInt uiCtxInc = 0; uiCtxInc < 4; uiCtxInc++ )
  {
    pcEstBitsSbac->blockRootCbpBits[ uiCtxInc ][ 0 ] = pCtx[ uiCtxInc ].getEntropyBits( 0 );
    pcEstBitsSbac->blockRootCbpBits[ uiCtxInc ][ 1 ] = pCtx[ uiCtxInc ].getEntropyBits( 1 );
  }
}


/*!
 ****************************************************************************
 * \brief
 *    estimate SAMBAC bit cost for significant coefficient group map
 ****************************************************************************
 */
Void TEncSbac::estSignificantCoeffGroupMapBit( estBitsSbacStruct* pcEstBitsSbac, ChannelType chType )
{
  Int firstCtx = 0, numCtx = NUM_SIG_CG_FLAG_CTX;

  for ( Int ctxIdx = firstCtx; ctxIdx < firstCtx + numCtx; ctxIdx++ )
  {
    for( UInt uiBin = 0; uiBin < 2; uiBin++ )
    {
      pcEstBitsSbac->significantCoeffGroupBits[ ctxIdx ][ uiBin ] = m_cCUSigCoeffGroupSCModel.get(  0, chType, ctxIdx ).getEntropyBits( uiBin );
    }
  }
}


/*!
 ****************************************************************************
 * \brief
 *    estimate SAMBAC bit cost for significant coefficient map
 ****************************************************************************
 */
Void TEncSbac::estSignificantMapBit( estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, ChannelType chType )
{
  //--------------------------------------------------------------------------------------------------

  //set up the number of channels and context variables

  const UInt firstComponent = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cb));
  const UInt lastComponent  = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cb));

  //----------------------------------------------------------

  Int firstCtx = MAX_INT;
  Int numCtx   = MAX_INT;

  if      ((width == 4) && (height == 4))
  {
    firstCtx = significanceMapContextSetStart[chType][CONTEXT_TYPE_4x4];
    numCtx   = significanceMapContextSetSize [chType][CONTEXT_TYPE_4x4];
  }
  else if ((width == 8) && (height == 8))
  {
    firstCtx = significanceMapContextSetStart[chType][CONTEXT_TYPE_8x8];
    numCtx   = significanceMapContextSetSize [chType][CONTEXT_TYPE_8x8];
  }
  else
  {
    firstCtx = significanceMapContextSetStart[chType][CONTEXT_TYPE_NxN];
    numCtx   = significanceMapContextSetSize [chType][CONTEXT_TYPE_NxN];
  }

  //--------------------------------------------------------------------------------------------------

  //fill the data for the significace map

  for (UInt component = firstComponent; component <= lastComponent; component++)
  {
    const UInt contextOffset = getSignificanceMapContextOffset(ComponentID(component));

    if (firstCtx > 0)
    {
      for( UInt bin = 0; bin < 2; bin++ ) //always get the DC
      {
        pcEstBitsSbac->significantBits[ contextOffset ][ bin ] = m_cCUSigSCModel.get( 0, 0, contextOffset ).getEntropyBits( bin );
      }
    }

    // This could be made optional, but would require this function to have knowledge of whether the
    // TU is transform-skipped or transquant-bypassed and whether the SPS flag is set
    for( UInt bin = 0; bin < 2; bin++ )
    {
      const Int ctxIdx = significanceMapContextSetStart[chType][CONTEXT_TYPE_SINGLE];
      pcEstBitsSbac->significantBits[ contextOffset + ctxIdx ][ bin ] = m_cCUSigSCModel.get( 0, 0, (contextOffset + ctxIdx) ).getEntropyBits( bin );
    }

    for ( Int ctxIdx = firstCtx; ctxIdx < firstCtx + numCtx; ctxIdx++ )
    {
      for( UInt uiBin = 0; uiBin < 2; uiBin++ )
      {
        pcEstBitsSbac->significantBits[ contextOffset + ctxIdx ][ uiBin ] = m_cCUSigSCModel.get(  0, 0, (contextOffset + ctxIdx) ).getEntropyBits( uiBin );
      }
    }
  }

  //--------------------------------------------------------------------------------------------------
}


/*!
 ****************************************************************************
 * \brief
 *    estimate bit cost of significant coefficient
 ****************************************************************************
 */

Void TEncSbac::estLastSignificantPositionBit( estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, ChannelType chType )
{
  //--------------------------------------------------------------------------------------------------.

  //set up the number of channels

  const UInt firstComponent = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cb));
  const UInt lastComponent  = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cb));

  //--------------------------------------------------------------------------------------------------

  //fill the data for the last-significant-coefficient position

  for (UInt componentIndex = firstComponent; componentIndex <= lastComponent; componentIndex++)
  {
    const ComponentID component = ComponentID(componentIndex);

    Int iBitsX = 0, iBitsY = 0;

    Int blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY;
    getLastSignificantContextParameters(ComponentID(component), width, height, blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY);

    Int ctx;

    const ChannelType channelType = toChannelType(ComponentID(component));

    ContextModel *const pCtxX = m_cCuCtxLastX.get( 0, channelType );
    ContextModel *const pCtxY = m_cCuCtxLastY.get( 0, channelType );
    Int          *const lastXBitsArray = pcEstBitsSbac->lastXBits[channelType];
    Int          *const lastYBitsArray = pcEstBitsSbac->lastYBits[channelType];

    //------------------------------------------------

    //X-coordinate

    for (ctx = 0; ctx < g_uiGroupIdx[ width - 1 ]; ctx++)
    {
      Int ctxOffset = blkSizeOffsetX + (ctx >>shiftX);
      lastXBitsArray[ ctx ] = iBitsX + pCtxX[ ctxOffset ].getEntropyBits( 0 );
      iBitsX += pCtxX[ ctxOffset ].getEntropyBits( 1 );
    }

    lastXBitsArray[ctx] = iBitsX;

    //------------------------------------------------

    //Y-coordinate

    for (ctx = 0; ctx < g_uiGroupIdx[ height - 1 ]; ctx++)
    {
      Int ctxOffset = blkSizeOffsetY + (ctx >>shiftY);
      lastYBitsArray[ ctx ] = iBitsY + pCtxY[ ctxOffset ].getEntropyBits( 0 );
      iBitsY += pCtxY[ ctxOffset ].getEntropyBits( 1 );
    }

    lastYBitsArray[ctx] = iBitsY;

  } //end of component loop

  //--------------------------------------------------------------------------------------------------
}


/*!
 ****************************************************************************
 * \brief
 *    estimate bit cost of significant coefficient
 ****************************************************************************
 */
Void TEncSbac::estSignificantCoefficientsBit( estBitsSbacStruct* pcEstBitsSbac, ChannelType chType )
{
  ContextModel *ctxOne = m_cCUOneSCModel.get(0, 0);
  ContextModel *ctxAbs = m_cCUAbsSCModel.get(0, 0);

  const UInt oneStartIndex = ((isLuma(chType)) ? (0)                     : (NUM_ONE_FLAG_CTX_LUMA));
  const UInt oneStopIndex  = ((isLuma(chType)) ? (NUM_ONE_FLAG_CTX_LUMA) : (NUM_ONE_FLAG_CTX));
  const UInt absStartIndex = ((isLuma(chType)) ? (0)                     : (NUM_ABS_FLAG_CTX_LUMA));
  const UInt absStopIndex  = ((isLuma(chType)) ? (NUM_ABS_FLAG_CTX_LUMA) : (NUM_ABS_FLAG_CTX));

  for (Int ctxIdx = oneStartIndex; ctxIdx < oneStopIndex; ctxIdx++)
  {
    pcEstBitsSbac->m_greaterOneBits[ ctxIdx ][ 0 ] = ctxOne[ ctxIdx ].getEntropyBits( 0 );
    pcEstBitsSbac->m_greaterOneBits[ ctxIdx ][ 1 ] = ctxOne[ ctxIdx ].getEntropyBits( 1 );
  }

  for (Int ctxIdx = absStartIndex; ctxIdx < absStopIndex; ctxIdx++)
  {
    pcEstBitsSbac->m_levelAbsBits[ ctxIdx ][ 0 ] = ctxAbs[ ctxIdx ].getEntropyBits( 0 );
    pcEstBitsSbac->m_levelAbsBits[ ctxIdx ][ 1 ] = ctxAbs[ ctxIdx ].getEntropyBits( 1 );
  }
}

/**
 - Initialize our context information from the nominated source.
 .
 \param pSrc From where to copy context information.
 */
Void TEncSbac::xCopyContextsFrom( const TEncSbac* pSrc )
{
  memcpy(m_contextModels, pSrc->m_contextModels, m_numContextModels*sizeof(m_contextModels[0]));
  memcpy(m_golombRiceAdaptationStatistics, pSrc->m_golombRiceAdaptationStatistics, (sizeof(UInt) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS));
}

Void  TEncSbac::loadContexts ( const TEncSbac* pSrc)
{
  xCopyContextsFrom(pSrc);
}

/** Performs CABAC encoding of the explicit RDPCM mode
 * \param rTu current TU data structure
 * \param compID component identifier
 */
Void TEncSbac::codeExplicitRdpcmMode( TComTU &rTu, const ComponentID compID )
{
  TComDataCU *cu = rTu.getCU();
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt absPartIdx   = rTu.GetAbsPartIdxTU(compID);
  const UInt tuHeight = g_aucConvertToBit[rect.height];
  const UInt tuWidth  = g_aucConvertToBit[rect.width];

  assert(tuHeight == tuWidth);
  assert(tuHeight < 4);

  UInt explicitRdpcmMode = cu->getExplicitRdpcmMode(compID, absPartIdx);

  if( explicitRdpcmMode == RDPCM_OFF )
  {
    m_pcBinIf->encodeBin (0, m_explicitRdpcmFlagSCModel.get (0, toChannelType(compID), 0));
  }
  else if( explicitRdpcmMode == RDPCM_HOR || explicitRdpcmMode == RDPCM_VER )
  {
    m_pcBinIf->encodeBin (1, m_explicitRdpcmFlagSCModel.get (0, toChannelType(compID), 0));
    if(explicitRdpcmMode == RDPCM_HOR)
    {
      m_pcBinIf->encodeBin ( 0, m_explicitRdpcmDirSCModel.get(0, toChannelType(compID), 0));
    }
    else
    {
      m_pcBinIf->encodeBin ( 1, m_explicitRdpcmDirSCModel.get(0, toChannelType(compID), 0));
    }
  }
  else
  {
    assert(0);
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

/** \file     TEncSbac.cpp
    \brief    SBAC encoder class
*/

#include "TEncTop.h"
#include "TEncSbac.h"

#if HHI_RQT
#include <map>
#endif

extern UChar  stateMappingTable[113];
extern Int entropyBits[128];

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TEncSbac::TEncSbac()
  // new structure here
  : m_cCUSplitFlagSCModel     ( 1,             1,               NUM_SPLIT_FLAG_CTX            )
  , m_cCUSkipFlagSCModel      ( 1,             1,               NUM_SKIP_FLAG_CTX             )
#if HHI_MRG
  , m_cCUMergeFlagSCModel     ( 1,             1,               NUM_MERGE_FLAG_CTX            )
  , m_cCUMergeIndexSCModel    ( 1,             1,               NUM_MERGE_INDEX_CTX           )
#endif
  , m_cCUAlfCtrlFlagSCModel   ( 1,             1,               NUM_ALF_CTRL_FLAG_CTX         )
  , m_cCUPartSizeSCModel      ( 1,             1,               NUM_PART_SIZE_CTX             )
  , m_cCUXPosiSCModel         ( 1,             1,               NUM_CU_X_POS_CTX              )
  , m_cCUYPosiSCModel         ( 1,             1,               NUM_CU_Y_POS_CTX              )
  , m_cCUPredModeSCModel      ( 1,             1,               NUM_PRED_MODE_CTX             )
  , m_cCUIntraPredSCModel     ( 1,             1,               NUM_ADI_CTX                   )
#if HHI_AIS
  , m_cCUIntraFiltFlagSCModel ( 1,             1,               NUM_ADI_FILT_CTX              )
#endif
  , m_cCUChromaPredSCModel    ( 1,             1,               NUM_CHROMA_PRED_CTX           )
  , m_cCUInterDirSCModel      ( 1,             1,               NUM_INTER_DIR_CTX             )
  , m_cCUMvdSCModel           ( 1,             2,               NUM_MV_RES_CTX                )
#ifdef DCM_PBIC
  , m_cCUIcdSCModel           ( 1,             3,               NUM_IC_RES_CTX                )
#endif
  , m_cCURefPicSCModel        ( 1,             1,               NUM_REF_NO_CTX                )
#ifdef QC_AMVRES
  , m_cCUMvResCModel       ( 1,             1,                 NUM_MV_RES_FALG_CTX )
#endif
#if HHI_RQT
  , m_cCUTransSubdivFlagSCModel( 1,          1,               NUM_TRANS_SUBDIV_FLAG_CTX )
#if HHI_RQT_ROOT
  , m_cCUQtRootCbfSCModel     ( 1,             1,               NUM_QT_ROOT_CBF_CTX   )
#endif
#endif
  , m_cCUTransIdxSCModel      ( 1,             1,               NUM_TRANS_IDX_CTX             )
  , m_cCUDeltaQpSCModel       ( 1,             1,               NUM_DELTA_QP_CTX              )
  , m_cCUCbfSCModel           ( 1,             2,               NUM_CBF_CTX                   )

#if HHI_RQT
  , m_cCUQtCbfSCModel       ( 1,             3,               NUM_QT_CBF_CTX        )
#endif

#if HHI_TRANSFORM_CODING
  , m_cCuCtxModSig            ( MAX_CU_DEPTH,  2,               NUM_SIG_FLAG_CTX              )
  , m_cCuCtxModLast           ( MAX_CU_DEPTH,  2,               NUM_LAST_FLAG_CTX             )
  , m_cCuCtxModAbsGreOne      ( 1,             2,               NUM_ABS_GREATER_ONE_CTX       )
  , m_cCuCtxModCoeffLevelM1   ( 1,             2,               NUM_COEFF_LEVEL_MINUS_ONE_CTX )
#else
  , m_cCUMapSCModel           ( MAX_CU_DEPTH,  2,               NUM_MAP_CTX                   )
  , m_cCULastSCModel          ( MAX_CU_DEPTH,  2,               NUM_LAST_CTX                  )
  , m_cCUOneSCModel           ( MAX_CU_DEPTH,  2,               NUM_ONE_CTX                   )
  , m_cCUAbsSCModel           ( MAX_CU_DEPTH,  2,               NUM_ABS_CTX                   )
#endif

  , m_cMVPIdxSCModel          ( 1,             1,               NUM_MVP_IDX_CTX               )
#ifdef DCM_PBIC
  , m_cICPIdxSCModel          ( 1,             1,               NUM_ICP_IDX_CTX               )
  , m_cZTreeMV0SCModel        ( 1,             1,               NUM_ZTREE_MV0_CTX             )
  , m_cZTreeMV1SCModel        ( 1,             1,               NUM_ZTREE_MV1_CTX             )
  , m_cZTreeMV2SCModel        ( 1,             1,               NUM_ZTREE_MV2_CTX             )
#endif
  , m_cCUROTindexSCModel      ( 1,             1,               NUM_ROT_IDX_CTX               )
  , m_cCUCIPflagCCModel       ( 1,             1,               NUM_CIP_FLAG_CTX              )
  , m_cALFFlagSCModel         ( 1,             1,               NUM_ALF_FLAG_CTX              )
  , m_cALFUvlcSCModel         ( 1,             1,               NUM_ALF_UVLC_CTX              )
  , m_cALFSvlcSCModel         ( 1,             1,               NUM_ALF_SVLC_CTX              )
#if HHI_ALF
  , m_cALFSplitFlagSCModel    ( 1,             1,               NUM_ALF_SPLITFLAG_CTX         )
#endif
#if PLANAR_INTRA
  , m_cPlanarIntraSCModel     ( 1,             1,               NUM_PLANAR_INTRA_CTX          )
#endif
{
  m_pcBitIf = 0;
  m_pcSlice = 0;
  m_pcBinIf = 0;

  m_uiCoeffCost = 0;
  m_bAlfCtrl = false;
  m_uiMaxAlfCtrlDepth = 0;
}

TEncSbac::~TEncSbac()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TEncSbac::resetEntropy           ()
{
  Int  iQp              = m_pcSlice->getSliceQp();
  SliceType eSliceType  = m_pcSlice->getSliceType();

  m_cCUSplitFlagSCModel.initBuffer    ( eSliceType, iQp, (Short*)INIT_SPLIT_FLAG );

  m_cCUSkipFlagSCModel.initBuffer     ( eSliceType, iQp, (Short*)INIT_SKIP_FLAG );
  m_cCUAlfCtrlFlagSCModel.initBuffer  ( eSliceType, iQp, (Short*)INIT_SKIP_FLAG );
#if HHI_MRG
  m_cCUMergeFlagSCModel.initBuffer    ( eSliceType, iQp, (Short*)INIT_MERGE_FLAG );
  m_cCUMergeIndexSCModel.initBuffer   ( eSliceType, iQp, (Short*)INIT_MERGE_INDEX );
#endif
  m_cCUPartSizeSCModel.initBuffer     ( eSliceType, iQp, (Short*)INIT_PART_SIZE );
  m_cCUXPosiSCModel.initBuffer        ( eSliceType, iQp, (Short*)INIT_CU_X_POS );
  m_cCUYPosiSCModel.initBuffer        ( eSliceType, iQp, (Short*)INIT_CU_Y_POS );
  m_cCUPredModeSCModel.initBuffer     ( eSliceType, iQp, (Short*)INIT_PRED_MODE );
  m_cCUIntraPredSCModel.initBuffer    ( eSliceType, iQp, (Short*)INIT_INTRA_PRED_MODE );
#if HHI_AIS
  m_cCUIntraFiltFlagSCModel.initBuffer( eSliceType, iQp, (Short*)INIT_INTRA_PRED_FILT );
#endif
  m_cCUChromaPredSCModel.initBuffer   ( eSliceType, iQp, (Short*)INIT_CHROMA_PRED_MODE );
  m_cCUInterDirSCModel.initBuffer     ( eSliceType, iQp, (Short*)INIT_INTER_DIR );
  m_cCUMvdSCModel.initBuffer          ( eSliceType, iQp, (Short*)INIT_MVD );
#ifdef DCM_PBIC
  m_cCUIcdSCModel.initBuffer          ( eSliceType, iQp, (Short*)INIT_ICD );
#endif
  m_cCURefPicSCModel.initBuffer       ( eSliceType, iQp, (Short*)INIT_REF_PIC );
#ifdef QC_AMVRES
  m_cCUMvResCModel.initBuffer         (eSliceType,  iQp, (Short*)INIT_MVRES_FLAG );
#endif
  m_cCUDeltaQpSCModel.initBuffer      ( eSliceType, iQp, (Short*)INIT_DQP );
  m_cCUCbfSCModel.initBuffer          ( eSliceType, iQp, (Short*)INIT_CBF );
#if HHI_RQT
  m_cCUQtCbfSCModel.initBuffer        ( eSliceType, iQp, (Short*)INIT_QT_CBF );
#if HHI_RQT_ROOT
  m_cCUQtRootCbfSCModel.initBuffer    ( eSliceType, iQp, (Short*)INIT_QT_ROOT_CBF );
#endif
#endif

#if HHI_TRANSFORM_CODING
  m_cCuCtxModSig.initBuffer           ( eSliceType, iQp, (Short*)INIT_SIG_FLAG );
  m_cCuCtxModLast.initBuffer          ( eSliceType, iQp, (Short*)INIT_LAST_FLAG );
  m_cCuCtxModAbsGreOne.initBuffer     ( eSliceType, iQp, (Short*)INIT_ABS_GREATER_ONE_FLAG );
  m_cCuCtxModCoeffLevelM1.initBuffer  ( eSliceType, iQp, (Short*)INIT_COEFF_LEVEL_MINUS_ONE_FLAG );
#else
  m_cCUMapSCModel.initBuffer          ( eSliceType, iQp, (Short*)INIT_SIGMAP );
  m_cCULastSCModel.initBuffer         ( eSliceType, iQp, (Short*)INIT_LAST_FLAG );
  m_cCUOneSCModel.initBuffer          ( eSliceType, iQp, (Short*)INIT_ONE_FLAG );
  m_cCUAbsSCModel.initBuffer          ( eSliceType, iQp, (Short*)INIT_TCOEFF_LEVEL );
#endif

  m_cMVPIdxSCModel.initBuffer         ( eSliceType, iQp, (Short*)INIT_MVP_IDX );
#ifdef DCM_PBIC
  m_cICPIdxSCModel.initBuffer         ( eSliceType, iQp, (Short*)INIT_ICP_IDX );
  m_cZTreeMV0SCModel.initBuffer       ( eSliceType, iQp, (Short*)INIT_ZTree_MV0 );
  m_cZTreeMV1SCModel.initBuffer       ( eSliceType, iQp, (Short*)INIT_ZTree_MV1 );
  m_cZTreeMV2SCModel.initBuffer       ( eSliceType, iQp, (Short*)INIT_ZTree_MV2 );
#endif
  m_cCUROTindexSCModel.initBuffer     ( eSliceType, iQp, (Short*)INIT_ROT_IDX );
  m_cCUCIPflagCCModel.initBuffer      ( eSliceType, iQp, (Short*)INIT_CIP_IDX );

#if HHI_ALF
  m_cALFSplitFlagSCModel.initBuffer   ( eSliceType, iQp, (Short*)INIT_ALF_SPLITFLAG );	
#endif
  
  m_cALFFlagSCModel.initBuffer        ( eSliceType, iQp, (Short*)INIT_ALF_FLAG );
  m_cALFUvlcSCModel.initBuffer        ( eSliceType, iQp, (Short*)INIT_ALF_UVLC );
  m_cALFSvlcSCModel.initBuffer        ( eSliceType, iQp, (Short*)INIT_ALF_SVLC );
#if HHI_RQT
  m_cCUTransSubdivFlagSCModel.initBuffer( eSliceType, iQp, (Short*)INIT_TRANS_SUBDIV_FLAG );
#endif
  m_cCUTransIdxSCModel.initBuffer     ( eSliceType, iQp, (Short*)INIT_TRANS_IDX );

#if PLANAR_INTRA
  m_cPlanarIntraSCModel.initBuffer    ( eSliceType, iQp, (Short*)INIT_PLANAR_INTRA );
#endif

  // new structure
  m_uiLastQp          = iQp;

  m_pcBinIf->start();

  return;
}

UInt TEncSbac::xGetCTXIdxFromWidth( Int iWidth )
{
  UInt uiCTXIdx;

  switch( iWidth )
  {
  case  2: uiCTXIdx = 6; break;
  case  4: uiCTXIdx = 5; break;
  case  8: uiCTXIdx = 4; break;
  case 16: uiCTXIdx = 3; break;
  case 32: uiCTXIdx = 2; break;
  case 64: uiCTXIdx = 1; break;
  default: uiCTXIdx = 0; break;
  }

  return uiCTXIdx;
}

Void TEncSbac::codeSPS( TComSPS* pcSPS )
{
  assert (0);
  return;
}

Void TEncSbac::codePPS( TComPPS* pcPPS )
{
  assert (0);
  return;
}

Void TEncSbac::codeSliceHeader( TComSlice* pcSlice )
{
  assert (0);
  return;
}

Void TEncSbac::codeTerminatingBit( UInt uilsLast )
{
  m_pcBinIf->encodeBinTrm( uilsLast );
}

Void TEncSbac::codeSliceFinish()
{
  m_pcBinIf->finish();
}



Void TEncSbac::xWriteUnarySymbol( UInt uiSymbol, ContextModel* pcSCModel, Int iOffset )
{
  m_pcBinIf->encodeBin( uiSymbol ? 1 : 0, pcSCModel[0] );

  if( 0 == uiSymbol)
  {
    return;
  }

  while( uiSymbol-- )
  {
    m_pcBinIf->encodeBin( uiSymbol ? 1 : 0, pcSCModel[ iOffset ] );
  }

  return;
}

Void TEncSbac::xWriteUnaryMaxSymbol( UInt uiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol )
{
  m_pcBinIf->encodeBin( uiSymbol ? 1 : 0, pcSCModel[ 0 ] );

  if ( uiSymbol == 0 )
  {
    return;
  }

  Bool bCodeLast = ( uiMaxSymbol > uiSymbol );

  while( --uiSymbol )
  {
    m_pcBinIf->encodeBin( 1, pcSCModel[ iOffset ] );
  }
  if( bCodeLast )
  {
    m_pcBinIf->encodeBin( 0, pcSCModel[ iOffset ] );
  }

  return;
}

Void TEncSbac::xWriteEpExGolomb( UInt uiSymbol, UInt uiCount )
{
  while( uiSymbol >= (UInt)(1<<uiCount) )
  {
    m_pcBinIf->encodeBinEP( 1 );
    uiSymbol -= 1<<uiCount;
    uiCount  ++;
  }
  m_pcBinIf->encodeBinEP( 0 );
  while( uiCount-- )
  {
    m_pcBinIf->encodeBinEP( (uiSymbol>>uiCount) & 1 );
  }

  return;
}

Void TEncSbac::xWriteExGolombLevel( UInt uiSymbol, ContextModel& rcSCModel  )
{
  if( uiSymbol )
  {
    m_pcBinIf->encodeBin( 1, rcSCModel );
    UInt uiCount = 0;
    Bool bNoExGo = (uiSymbol < 13);

    while( --uiSymbol && ++uiCount < 13 )
    {
      m_pcBinIf->encodeBin( 1, rcSCModel );
    }
    if( bNoExGo )
    {
      m_pcBinIf->encodeBin( 0, rcSCModel );
    }
    else
    {
      xWriteEpExGolomb( uiSymbol, 0 );
    }
  }
  else
  {
    m_pcBinIf->encodeBin( 0, rcSCModel );
  }

  return;
}


// SBAC RD
Void  TEncSbac::load ( TEncSbac* pScr)
{
  this->xCopyFrom(pScr);
}

Void  TEncSbac::store( TEncSbac* pDest)
{
  pDest->xCopyFrom( this );
}


Void TEncSbac::xCopyFrom( TEncSbac* pSrc )
{
  m_pcBinIf->copyState( pSrc->m_pcBinIf );

  this->m_uiCoeffCost        = pSrc->m_uiCoeffCost;
  this->m_uiLastQp           = pSrc->m_uiLastQp;

  this->m_cCUSplitFlagSCModel     .copyFrom( &pSrc->m_cCUSplitFlagSCModel     );
  this->m_cCUSkipFlagSCModel      .copyFrom( &pSrc->m_cCUSkipFlagSCModel      );
#if HHI_MRG
  this->m_cCUMergeFlagSCModel     .copyFrom( &pSrc->m_cCUMergeFlagSCModel     );
  this->m_cCUMergeIndexSCModel    .copyFrom( &pSrc->m_cCUMergeIndexSCModel    );
#endif
  this->m_cCUPartSizeSCModel      .copyFrom( &pSrc->m_cCUPartSizeSCModel      );
  this->m_cCUPredModeSCModel      .copyFrom( &pSrc->m_cCUPredModeSCModel      );
  this->m_cCUIntraPredSCModel     .copyFrom( &pSrc->m_cCUIntraPredSCModel     );
#if HHI_AIS
  this->m_cCUIntraFiltFlagSCModel .copyFrom( &pSrc->m_cCUIntraFiltFlagSCModel );
#endif
  this->m_cCUChromaPredSCModel.copyFrom( &pSrc->m_cCUChromaPredSCModel  );
  this->m_cCUDeltaQpSCModel   .copyFrom( &pSrc->m_cCUDeltaQpSCModel     );
  this->m_cCUInterDirSCModel  .copyFrom( &pSrc->m_cCUInterDirSCModel    );
  this->m_cCURefPicSCModel    .copyFrom( &pSrc->m_cCURefPicSCModel      );
  this->m_cCUMvdSCModel       .copyFrom( &pSrc->m_cCUMvdSCModel         );
#ifdef QC_AMVRES
  this->m_cCUMvResCModel       .copyFrom( &pSrc->m_cCUMvResCModel         );
#endif
#ifdef DCM_PBIC
  this->m_cCUIcdSCModel       .copyFrom( &pSrc->m_cCUIcdSCModel         );
#endif
  this->m_cCUCbfSCModel       .copyFrom( &pSrc->m_cCUCbfSCModel         );
#if HHI_RQT
  this->m_cCUQtCbfSCModel     .copyFrom( &pSrc->m_cCUQtCbfSCModel       );
  this->m_cCUTransSubdivFlagSCModel.copyFrom( &pSrc->m_cCUTransSubdivFlagSCModel );
#if HHI_RQT_ROOT
  this->m_cCUQtRootCbfSCModel .copyFrom( &pSrc->m_cCUQtRootCbfSCModel   );
#endif
#endif
  this->m_cCUTransIdxSCModel  .copyFrom( &pSrc->m_cCUTransIdxSCModel    );

#if HHI_TRANSFORM_CODING
  this->m_cCuCtxModSig         .copyFrom( &pSrc->m_cCuCtxModSig          );
  this->m_cCuCtxModLast        .copyFrom( &pSrc->m_cCuCtxModLast         );
  this->m_cCuCtxModAbsGreOne   .copyFrom( &pSrc->m_cCuCtxModAbsGreOne    );
  this->m_cCuCtxModCoeffLevelM1.copyFrom( &pSrc->m_cCuCtxModCoeffLevelM1 );
#else
  this->m_cCUMapSCModel        .copyFrom( &pSrc->m_cCUMapSCModel         );
  this->m_cCULastSCModel       .copyFrom( &pSrc->m_cCULastSCModel        );
  this->m_cCUOneSCModel        .copyFrom( &pSrc->m_cCUOneSCModel         );
  this->m_cCUAbsSCModel        .copyFrom( &pSrc->m_cCUAbsSCModel         );
#endif

  this->m_cMVPIdxSCModel      .copyFrom( &pSrc->m_cMVPIdxSCModel        );
#ifdef DCM_PBIC
  this->m_cICPIdxSCModel      .copyFrom( &pSrc->m_cICPIdxSCModel        );
  this->m_cZTreeMV0SCModel    .copyFrom( &pSrc->m_cZTreeMV0SCModel      );
  this->m_cZTreeMV1SCModel    .copyFrom( &pSrc->m_cZTreeMV1SCModel      );
  this->m_cZTreeMV2SCModel    .copyFrom( &pSrc->m_cZTreeMV2SCModel      );
#endif

  this->m_cCUROTindexSCModel  .copyFrom( &pSrc->m_cCUROTindexSCModel    );
  this->m_cCUCIPflagCCModel   .copyFrom( &pSrc->m_cCUCIPflagCCModel     );
  this->m_cCUXPosiSCModel     .copyFrom( &pSrc->m_cCUXPosiSCModel       );
  this->m_cCUYPosiSCModel     .copyFrom( &pSrc->m_cCUXPosiSCModel       );

#if HHI_ALF
  this->m_cCUAlfCtrlFlagSCModel .copyFrom( &pSrc->m_cCUAlfCtrlFlagSCModel );
  this->m_cALFFlagSCModel       .copyFrom( &pSrc->m_cALFFlagSCModel       );
  this->m_cALFSplitFlagSCModel  .copyFrom( &pSrc->m_cALFSplitFlagSCModel  );
#endif    
}

// CIP
Void TEncSbac::codeCIPflag( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

  Int CIPflag = pcCU->getCIPflag   ( uiAbsPartIdx );
  Int iCtx    = pcCU->getCtxCIPFlag( uiAbsPartIdx );

  m_pcBinIf->encodeBin( (CIPflag) ? 1 : 0, m_cCUCIPflagCCModel.get( 0, 0, iCtx ) );
}

Void TEncSbac::codeROTindex( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

  Int indexROT = pcCU->getROTindex( uiAbsPartIdx );
  Int dictSize = ROT_DICT;

  switch (dictSize)
  {
   case 9:
    {
      m_pcBinIf->encodeBin( indexROT> 0 ? 0 : 1 , m_cCUROTindexSCModel.get( 0, 0, 0 ) );
      if ( indexROT > 0 )
      {
        indexROT = indexROT-1;
        m_pcBinIf->encodeBin( (indexROT & 0x01),      m_cCUROTindexSCModel.get( 0, 0, 1 ) );
        m_pcBinIf->encodeBin( (indexROT & 0x02) >> 1, m_cCUROTindexSCModel.get( 0, 0, 2 ) );
        m_pcBinIf->encodeBin( (indexROT & 0x04) >> 2, m_cCUROTindexSCModel.get( 0, 0, 2 ) );
      }
    }
    break;
  case 4:
    {
      m_pcBinIf->encodeBin( (indexROT & 0x01),      m_cCUROTindexSCModel.get( 0, 0, 0 ) );
      m_pcBinIf->encodeBin( (indexROT & 0x02) >> 1, m_cCUROTindexSCModel.get( 0, 0, 1 ) );
    }
    break;
  case 2:
    {
      m_pcBinIf->encodeBin( indexROT> 0 ? 0 : 1 , m_cCUROTindexSCModel.get( 0, 0, 0 ) );
    }
    break;
  case 5:
    {
      m_pcBinIf->encodeBin( indexROT> 0 ? 0 : 1 , m_cCUROTindexSCModel.get( 0, 0, 0 ) );
      if ( indexROT > 0 )
      {
        indexROT = indexROT-1;
        m_pcBinIf->encodeBin( (indexROT & 0x01),      m_cCUROTindexSCModel.get( 0, 0, 1 ) );
        m_pcBinIf->encodeBin( (indexROT & 0x02) >> 1, m_cCUROTindexSCModel.get( 0, 0, 2 ) );
      }
    }
    break;
  case 1:
    {
    }
    break;
  }
  return;
}

Void TEncSbac::codeMVPIdx ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  Int iSymbol = pcCU->getMVPIdx(eRefList, uiAbsPartIdx);
  Int iNum    = pcCU->getMVPNum(eRefList, uiAbsPartIdx);

  xWriteUnaryMaxSymbol(iSymbol, m_cMVPIdxSCModel.get(0), 1, iNum-1);
}

#ifdef DCM_PBIC
Void TEncSbac::codeICPIdx ( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  Int iSymbol = pcCU->getICPIdx(uiAbsPartIdx);
  Int iNum    = pcCU->getICPNum(uiAbsPartIdx);

  xWriteUnaryMaxSymbol(iSymbol, m_cICPIdxSCModel.get(0), 1, iNum-1);
}
#endif

Void TEncSbac::codePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  PartSize eSize         = pcCU->getPartitionSize( uiAbsPartIdx );

  if ( pcCU->getSlice()->isInterB() && pcCU->isIntra( uiAbsPartIdx ) )
  {
    m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 0) );
#if HHI_RMP_SWITCH
    if( pcCU->getSlice()->getSPS()->getUseRMP() ||  pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
#endif
    {
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 1) );
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 2) );
    }
#if HHI_DISABLE_INTER_NxN_SPLIT
    if( pcCU->getWidth( uiAbsPartIdx ) == 8 )
    {
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 3) );
    }
#else
    m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 3) );
#endif
    m_pcBinIf->encodeBin( (eSize == SIZE_2Nx2N? 0 : 1), m_cCUPartSizeSCModel.get( 0, 0, 4) );
    return;
  }

  if ( pcCU->isIntra( uiAbsPartIdx ) )
  {
    m_pcBinIf->encodeBin( eSize == SIZE_2Nx2N? 1 : 0, m_cCUPartSizeSCModel.get( 0, 0, 0 ) );
    return;
  }

  switch(eSize)
  {
  case SIZE_2Nx2N:
    {
      m_pcBinIf->encodeBin( 1, m_cCUPartSizeSCModel.get( 0, 0, 0) );
      break;
    }
  case SIZE_2NxN:
  case SIZE_2NxnU:
  case SIZE_2NxnD:
    {
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 0) );
      m_pcBinIf->encodeBin( 1, m_cCUPartSizeSCModel.get( 0, 0, 1) );

#if HHI_RMP_SWITCH
      if (pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) && pcCU->getSlice()->getSPS()->getUseRMP() )
#else
      if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
#endif
      {
        if (eSize == SIZE_2NxN)
        {
          m_pcBinIf->encodeBin(1, m_cCUYPosiSCModel.get( 0, 0, 0 ));
        }
        else
        {
          m_pcBinIf->encodeBin(0, m_cCUYPosiSCModel.get( 0, 0, 0 ));
          m_pcBinIf->encodeBin((eSize == SIZE_2NxnU? 0: 1), m_cCUYPosiSCModel.get( 0, 0, 1 ));
        }
      }
#if HHI_RMP_SWITCH
      else if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) && !pcCU->getSlice()->getSPS()->getUseRMP() )
      {
        m_pcBinIf->encodeBin((eSize == SIZE_2NxnU? 0: 1), m_cCUYPosiSCModel.get( 0, 0, 1 ));
      }
#endif
      break;
    }
  case SIZE_Nx2N:
  case SIZE_nLx2N:
  case SIZE_nRx2N:
    {
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 0) );
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 1) );
      m_pcBinIf->encodeBin( 1, m_cCUPartSizeSCModel.get( 0, 0, 2) );

#if HHI_RMP_SWITCH
      if (pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) && pcCU->getSlice()->getSPS()->getUseRMP() )
#else
      if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
#endif
      {
        if (eSize == SIZE_Nx2N)
        {
          m_pcBinIf->encodeBin(1, m_cCUXPosiSCModel.get( 0, 0, 0 ));
        }
        else
        {
          m_pcBinIf->encodeBin(0, m_cCUXPosiSCModel.get( 0, 0, 0 ));
          m_pcBinIf->encodeBin((eSize == SIZE_nLx2N? 0: 1), m_cCUXPosiSCModel.get( 0, 0, 1 ));
        }
      }
#if HHI_RMP_SWITCH
      else if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) && !pcCU->getSlice()->getSPS()->getUseRMP() )
      {
        m_pcBinIf->encodeBin((eSize == SIZE_nLx2N? 0: 1), m_cCUXPosiSCModel.get( 0, 0, 1 ));
      }
#endif
      break;
    }
  case SIZE_NxN:
    {
#if HHI_DISABLE_INTER_NxN_SPLIT
      if( pcCU->getWidth( uiAbsPartIdx ) == 8 )
#endif
      {
        m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 0) );
#if HHI_RMP_SWITCH
        if( pcCU->getSlice()->getSPS()->getUseRMP() ||  pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
#endif
        {
          m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 1) );
          m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 2) );
        }

        if (pcCU->getSlice()->isInterB())
        {
          m_pcBinIf->encodeBin( 1, m_cCUPartSizeSCModel.get( 0, 0, 3) );
        }
      }
      break;
    }
  default:
    {
      assert(0);
    }
  }
}

Void TEncSbac::codePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  // get context function is here
  Int iPredMode = pcCU->getPredictionMode( uiAbsPartIdx );

#if HHI_MRG && !SAMSUNG_MRG_SKIP_DIRECT
  if ( !pcCU->getSlice()->getSPS()->getUseMRG() )
  {
    m_pcBinIf->encodeBin( iPredMode == MODE_SKIP ? 0 : 1, m_cCUPredModeSCModel.get( 0, 0, 0 ) );
  }
#else
  m_pcBinIf->encodeBin( iPredMode == MODE_SKIP ? 0 : 1, m_cCUPredModeSCModel.get( 0, 0, 0 ) );
#endif

  if (pcCU->getSlice()->isInterB() )
  {
    return;
  }

  if ( iPredMode != MODE_SKIP )
  {
    m_pcBinIf->encodeBin( iPredMode == MODE_INTER ? 0 : 1, m_cCUPredModeSCModel.get( 0, 0, 1 ) );
  }
}

#if HHI_ALF
Void TEncSbac::codeAlfCoeff( Int iCoeff, Int iLength, Int iPos)
{
  if ( iCoeff == 0 )
  {
    m_pcBinIf->encodeBin( 0, m_cALFSvlcSCModel.get( 0, 0, 0 ) );
  }
  else
  {
    m_pcBinIf->encodeBin( 1, m_cALFSvlcSCModel.get( 0, 0, 0 ) );

    // write sign
    if ( iCoeff > 0 )
    {
      m_pcBinIf->encodeBin( 0, m_cALFSvlcSCModel.get( 0, 0, 1 ) );
    }
    else
    {
      m_pcBinIf->encodeBin( 1, m_cALFSvlcSCModel.get( 0, 0, 1 ) );
      iCoeff = -iCoeff;
    }

    Int iM =4;
    if(iLength==3)
    {
      if(iPos == 0)
        iM = 4 ;
      else if(iPos == 1)
        iM = 1 ;
      else if(iPos == 2)
        iM = 2 ;
    }
    else if(iLength==5)
    {
      if(iPos == 0)
        iM = 3 ;
      else if(iPos == 1)
        iM = 5 ;
      else if(iPos == 2)
        iM = 1 ;
      else if(iPos == 3)
        iM = 3 ;
      else if(iPos == 4)
        iM = 2 ;
    }
    else if(iLength==7)
    {
      if(iPos == 0)
        iM = 3 ;
      else if(iPos == 1)
        iM = 4 ;
      else if(iPos == 2)
        iM = 5;
      else if(iPos == 3)
        iM = 1 ;
      else if(iPos == 4)
        iM = 3 ;
      else if(iPos == 5)
       iM = 3 ;
      else if(iPos == 6)
       iM = 2 ;
    }
    else if(iLength==9)
    {
      if(iPos == 0)
        iM = 2 ;
      else if(iPos == 1)
        iM = 4 ;
      else if(iPos == 2)
        iM = 4 ;
      else if(iPos == 3)
        iM = 5 ;
      else if(iPos == 4)
        iM = 1 ;
      else if(iPos == 5)
       iM = 3 ;
      else if(iPos == 6)
       iM = 3 ;
      else if(iPos == 7)
      iM = 3 ;
     else if(iPos == 8)
      iM = 2 ;
    }

    xWriteEpExGolomb( iCoeff , iM );
 }
}

Void TEncSbac::codeAlfDc( Int iDc )
{
  if ( iDc == 0 )
    {
      m_pcBinIf->encodeBin( 0, m_cALFSvlcSCModel.get( 0, 0, 0 ) );
    }
    else
    {
      m_pcBinIf->encodeBin( 1, m_cALFSvlcSCModel.get( 0, 0, 0 ) );

      // write sign
      if ( iDc > 0 )
      {
        m_pcBinIf->encodeBin( 0, m_cALFSvlcSCModel.get( 0, 0, 1 ) );
      }
      else
      {
        m_pcBinIf->encodeBin( 1, m_cALFSvlcSCModel.get( 0, 0, 1 ) );
        iDc = -iDc;
      }

      xWriteEpExGolomb( iDc , 9 );
    }
}


Void TEncSbac::codeAlfCtrlFlag( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  if (!m_bAlfCtrl)
    return;

  if( pcCU->getDepth(uiAbsPartIdx) > m_uiMaxAlfCtrlDepth && !pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, m_uiMaxAlfCtrlDepth))
  {
    return;
  }

  // get context function is here
  UInt uiSymbol = pcCU->getAlfCtrlFlag( uiAbsPartIdx ) ? 1 : 0;
  m_pcBinIf->encodeBin( uiSymbol, m_cCUAlfCtrlFlagSCModel.get( 0, 0, pcCU->getCtxAlfCtrlFlag( uiAbsPartIdx) ) );
}

Void TEncSbac::codeAlfQTCtrlFlag( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  if( pcCU->getDepth(uiAbsPartIdx) > m_uiMaxAlfCtrlDepth && !pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, m_uiMaxAlfCtrlDepth))
  {
    return;
  }

  // get context function is here
  UInt uiSymbol = pcCU->getAlfCtrlFlag( uiAbsPartIdx ) ? 1 : 0;
  m_pcBinIf->encodeBin( uiSymbol, m_cCUAlfCtrlFlagSCModel.get( 0, 0, 0 ) );
}

Void TEncSbac::codeAlfQTSplitFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiMaxDepth )
{
  //if( uiDepth >= uiMaxDepth )
  if( uiDepth >= g_uiMaxCUDepth - g_uiAddCUDepth ) // fix HS
    return;

  UInt uiSplitFlag =  ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) ? 1 : 0 ;
  //UInt uiCtx           = pcCU->getCtxSplitFlag( uiAbsPartIdx, uiDepth );
  m_pcBinIf->encodeBin( uiSplitFlag, m_cALFSplitFlagSCModel.get( 0, 0, 0 ) );
}
#else
Void TEncSbac::codeAlfCtrlFlag( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  if (!m_bAlfCtrl)
    return;

  if( pcCU->getDepth(uiAbsPartIdx) > m_uiMaxAlfCtrlDepth && !pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, m_uiMaxAlfCtrlDepth))
  {
    return;
  }

  // get context function is here
  UInt uiSymbol = pcCU->getAlfCtrlFlag( uiAbsPartIdx ) ? 1 : 0;

  m_pcBinIf->encodeBin( uiSymbol, m_cCUAlfCtrlFlagSCModel.get( 0, 0, pcCU->getCtxAlfCtrlFlag( uiAbsPartIdx) ) );
}
#endif

Void TEncSbac::codeAlfCtrlDepth()
{
  if (!m_bAlfCtrl)
    return;

  UInt uiDepth = m_uiMaxAlfCtrlDepth;
  xWriteUnaryMaxSymbol(uiDepth, m_cALFUvlcSCModel.get(0), 1, g_uiMaxCUDepth-1);
}

Void TEncSbac::codeSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  // get context function is here
  UInt uiSymbol = pcCU->isSkipped( uiAbsPartIdx ) ? 1 : 0;
  m_pcBinIf->encodeBin( uiSymbol, m_cCUSkipFlagSCModel.get( 0, 0, pcCU->getCtxSkipFlag( uiAbsPartIdx) ) );
}

#if HHI_MRG
Void TEncSbac::codeMergeFlag( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiSymbol = pcCU->getMergeFlag( uiAbsPartIdx ) ? 1 : 0;
  m_pcBinIf->encodeBin( uiSymbol, m_cCUMergeFlagSCModel.get( 0, 0, pcCU->getCtxMergeFlag( uiAbsPartIdx ) ) );
}

Void TEncSbac::codeMergeIndex( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiSymbol = pcCU->getMergeIndex( uiAbsPartIdx ) ? 1 : 0;
  m_pcBinIf->encodeBin( uiSymbol, m_cCUMergeIndexSCModel.get( 0, 0, pcCU->getCtxMergeIndex( uiAbsPartIdx ) ) );
}
#endif

Void TEncSbac::codeSplitFlag   ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
    return;

  UInt uiCtx           = pcCU->getCtxSplitFlag( uiAbsPartIdx, uiDepth );
  UInt uiCurrSplitFlag = ( pcCU->getDepth( uiAbsPartIdx ) > uiDepth ) ? 1 : 0;

  assert( uiCtx < 3 );
  m_pcBinIf->encodeBin( uiCurrSplitFlag, m_cCUSplitFlagSCModel.get( 0, 0, uiCtx ) );
#if HHI_RQT
  DTRACE_CABAC_V( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tSplitFlag\n" )
#endif
  return;
}

#if HHI_RQT
Void TEncSbac::codeTransformSubdivFlag( UInt uiSymbol, UInt uiCtx )
{
  m_pcBinIf->encodeBin( uiSymbol, m_cCUTransSubdivFlagSCModel.get( 0, 0, uiCtx ) );
  DTRACE_CABAC_V( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseTransformSubdivFlag()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiSymbol )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\n" )
}
#endif

Void TEncSbac::codeTransformIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiTrLevel = 0;

  UInt uiWidthInBit  = g_aucConvertToBit[pcCU->getWidth(uiAbsPartIdx)]+2;
  UInt uiTrSizeInBit = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxTrSize()]+2;
  uiTrLevel          = uiWidthInBit >= uiTrSizeInBit ? uiWidthInBit - uiTrSizeInBit : 0;

  UInt uiMinTrDepth = pcCU->getSlice()->getSPS()->getMinTrDepth() + uiTrLevel;
  UInt uiMaxTrDepth = pcCU->getSlice()->getSPS()->getMaxTrDepth() + uiTrLevel;

  if ( uiMinTrDepth == uiMaxTrDepth )
  {
    return;
  }

  UInt uiSymbol = pcCU->getTransformIdx(uiAbsPartIdx) - uiMinTrDepth;

  m_pcBinIf->encodeBin( uiSymbol ? 1 : 0, m_cCUTransIdxSCModel.get( 0, 0, pcCU->getCtxTransIdx( uiAbsPartIdx ) ) );

  if ( !uiSymbol )
  {
    return;
  }

  if (pcCU->getPartitionSize(uiAbsPartIdx) >= SIZE_2NxnU && pcCU->getPartitionSize(uiAbsPartIdx) <= SIZE_nRx2N && uiMinTrDepth == 0 && uiMaxTrDepth == 1 && uiSymbol)
  {
    return;
  }

  Int  iCount = 1;
  uiSymbol--;
  while( ++iCount <= (Int)( uiMaxTrDepth - uiMinTrDepth ) )
  {
    m_pcBinIf->encodeBin( uiSymbol ? 1 : 0, m_cCUTransIdxSCModel.get( 0, 0, 3 ) );
    if ( uiSymbol == 0 )
    {
      return;
    }
    uiSymbol--;
  }

  return;
}

#if PLANAR_INTRA
Void TEncSbac::xPutPlanarBins( Int n, Int cn )
{
  UInt tmp  = 1<<(n-4);
  UInt code = tmp+cn%tmp;
  UInt len  = 1+(n-4)+(cn>>(n-4));
  Int  ctr;

  for( ctr = len-1; ctr >= 0; ctr-- )
    m_pcBinIf->encodeBinEP( (code & (1 << ctr)) >> ctr );
}

Void TEncSbac::xCodePlanarDelta( TComDataCU* pcCU, UInt uiAbsPartIdx , Int iDelta )
{
  /* Planar quantization
  Y        qY              cW
  0-3   :  0,1,2,3         0-3
  4-15  :  4,6,8..14       4-9
  16-63 : 18,22,26..62    10-21
  64-.. : 68,76...        22-
  */
  Bool bDeltaNegative = iDelta < 0 ? true : false;
  UInt uiDeltaAbs     = abs(iDelta);

  if( uiDeltaAbs < 4 )
    xPutPlanarBins( 5, uiDeltaAbs );
  else if( uiDeltaAbs < 16 )
    xPutPlanarBins( 5, (uiDeltaAbs>>1)+2 );
  else if( uiDeltaAbs < 64)
    xPutPlanarBins( 5, (uiDeltaAbs>>2)+6 );
  else
    xPutPlanarBins( 5, (uiDeltaAbs>>3)+14 );

  if(uiDeltaAbs > 0)
    m_pcBinIf->encodeBinEP( bDeltaNegative ? 1 : 0 );

}

Void TEncSbac::codePlanarInfo( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  if (pcCU->isIntra( uiAbsPartIdx ))
  {
    UInt uiPlanar = pcCU->getPlanarInfo(uiAbsPartIdx, PLANAR_FLAG);

    m_pcBinIf->encodeBin( uiPlanar, m_cPlanarIntraSCModel.get( 0, 0, 0 ) );

    if ( uiPlanar )
    {
      // Planar delta for Y
      xCodePlanarDelta( pcCU, uiAbsPartIdx, pcCU->getPlanarInfo(uiAbsPartIdx, PLANAR_DELTAY) );

      // Planar delta for U and V
      Int  iPlanarDeltaU = pcCU->getPlanarInfo(uiAbsPartIdx, PLANAR_DELTAU);
      Int  iPlanarDeltaV = pcCU->getPlanarInfo(uiAbsPartIdx, PLANAR_DELTAV);

      m_pcBinIf->encodeBin( ( iPlanarDeltaU == 0 && iPlanarDeltaV == 0 ) ? 1 : 0, m_cPlanarIntraSCModel.get( 0, 0, 1 ) );

      if ( iPlanarDeltaU != 0 || iPlanarDeltaV != 0 )
      {
        xCodePlanarDelta( pcCU, uiAbsPartIdx, iPlanarDeltaU );
        xCodePlanarDelta( pcCU, uiAbsPartIdx, iPlanarDeltaV );
      }
    }
  }
}
#endif

#if ANG_INTRA
Void TEncSbac::codeIntraDirLumaAng( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiDir         = pcCU->getLumaIntraDir( uiAbsPartIdx );
  Int  iMostProbable = pcCU->getMostProbableIntraDirLuma( uiAbsPartIdx );

  if (uiDir == iMostProbable)
    m_pcBinIf->encodeBin( 1, m_cCUIntraPredSCModel.get( 0, 0, 0 ) );
  else{
    m_pcBinIf->encodeBin( 0, m_cCUIntraPredSCModel.get( 0, 0, 0 ) );
    uiDir = uiDir > iMostProbable ? uiDir - 1 : uiDir;
#if UNIFIED_DIRECTIONAL_INTRA
    Int iIntraIdx = pcCU->getIntraSizeIdx(uiAbsPartIdx);
    if ( g_aucIntraModeBitsAng[iIntraIdx] < 6 )
    {
      m_pcBinIf->encodeBin((uiDir & 0x01), m_cCUIntraPredSCModel.get(0, 0, 1));
      if ( g_aucIntraModeBitsAng[iIntraIdx] > 2 ) m_pcBinIf->encodeBin((uiDir & 0x02) >> 1, m_cCUIntraPredSCModel.get(0, 0, 1));
      if ( g_aucIntraModeBitsAng[iIntraIdx] > 3 ) m_pcBinIf->encodeBin((uiDir & 0x04) >> 2, m_cCUIntraPredSCModel.get(0, 0, 1));
      if ( g_aucIntraModeBitsAng[iIntraIdx] > 4 ) m_pcBinIf->encodeBin((uiDir & 0x08) >> 3, m_cCUIntraPredSCModel.get(0, 0, 1));
    }
    else
#endif
    if (uiDir < 31){ // uiDir is here 0...32, 5 bits for uiDir 0...30, 31 is an escape code for coding one more bit for 31 and 32
      m_pcBinIf->encodeBin((uiDir & 0x01),      m_cCUIntraPredSCModel.get(0, 0, 1));
      m_pcBinIf->encodeBin((uiDir & 0x02) >> 1, m_cCUIntraPredSCModel.get(0, 0, 1));
      m_pcBinIf->encodeBin((uiDir & 0x04) >> 2, m_cCUIntraPredSCModel.get(0, 0, 1));
      m_pcBinIf->encodeBin((uiDir & 0x08) >> 3, m_cCUIntraPredSCModel.get(0, 0, 1));
      m_pcBinIf->encodeBin((uiDir & 0x10) >> 4, m_cCUIntraPredSCModel.get(0, 0, 1));
    }
    else{
      m_pcBinIf->encodeBin(1, m_cCUIntraPredSCModel.get(0, 0, 1));
      m_pcBinIf->encodeBin(1, m_cCUIntraPredSCModel.get(0, 0, 1));
      m_pcBinIf->encodeBin(1, m_cCUIntraPredSCModel.get(0, 0, 1));
      m_pcBinIf->encodeBin(1, m_cCUIntraPredSCModel.get(0, 0, 1));
      m_pcBinIf->encodeBin(1, m_cCUIntraPredSCModel.get(0, 0, 1));
      m_pcBinIf->encodeBin((uiDir == 32) ? 1 : 0, m_cCUIntraPredSCModel.get(0, 0, 1));
    }
  }

  return;
}
#endif

Void TEncSbac::codeIntraDirLumaAdi( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  Int iIntraDirLuma = pcCU->convertIntraDirLumaAdi( pcCU, uiAbsPartIdx );
  Int iIntraIdx= pcCU->getIntraSizeIdx(uiAbsPartIdx);

  m_pcBinIf->encodeBin( iIntraDirLuma >= 0 ? 0 : 1, m_cCUIntraPredSCModel.get( 0, 0, 0 ) );

  if (iIntraDirLuma >= 0)
  {
    m_pcBinIf->encodeBin((iIntraDirLuma & 0x01), m_cCUIntraPredSCModel.get(0, 0, 1));

    m_pcBinIf->encodeBin((iIntraDirLuma & 0x02) >> 1, m_cCUIntraPredSCModel.get(0, 0, 1));

    if (g_aucIntraModeBits[iIntraIdx] >= 4)
    {
      m_pcBinIf->encodeBin((iIntraDirLuma & 0x04) >> 2, m_cCUIntraPredSCModel.get(0, 0, 1));

      if (g_aucIntraModeBits[iIntraIdx] >= 5)
      {
        m_pcBinIf->encodeBin((iIntraDirLuma & 0x08) >> 3,
          m_cCUIntraPredSCModel.get(0, 0, 1));

        if (g_aucIntraModeBits[iIntraIdx] >= 6)
        {
          m_pcBinIf->encodeBin((iIntraDirLuma & 0x10) >> 4,
            m_cCUIntraPredSCModel.get(0, 0, 1));
        }
      }
    }
  }
  return;
}

#if HHI_AIS
Void TEncSbac::codeIntraFiltFlagLumaAdi( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiSymbol = pcCU->getLumaIntraFiltFlag( uiAbsPartIdx );
#if ANG_INTRA
  UInt uiCtx    = pcCU->angIntraEnabledPredPart( uiAbsPartIdx ) ?  pcCU->getCtxIntraFiltFlagLumaAng( uiAbsPartIdx ) : pcCU->getCtxIntraFiltFlagLuma( uiAbsPartIdx );
#else
  UInt uiCtx    = pcCU->getCtxIntraFiltFlagLuma( uiAbsPartIdx );
#endif
  m_pcBinIf->encodeBin( uiSymbol, m_cCUIntraFiltFlagSCModel.get( 0, 0, uiCtx ) );

  return;
}
#endif

Void TEncSbac::codeIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiCtx            = pcCU->getCtxIntraDirChroma( uiAbsPartIdx );
  UInt uiIntraDirChroma = pcCU->getChromaIntraDir   ( uiAbsPartIdx );

  if ( 0 == uiIntraDirChroma )
  {
    m_pcBinIf->encodeBin( 0, m_cCUChromaPredSCModel.get( 0, 0, uiCtx ) );
  }
  else
  {
    m_pcBinIf->encodeBin( 1, m_cCUChromaPredSCModel.get( 0, 0, uiCtx ) );
    xWriteUnaryMaxSymbol( uiIntraDirChroma - 1, m_cCUChromaPredSCModel.get( 0, 0 ) + 3, 0, 3 );
  }

  return;
}

Void TEncSbac::codeInterDir( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiInterDir = pcCU->getInterDir   ( uiAbsPartIdx );
  UInt uiCtx      = pcCU->getCtxInterDir( uiAbsPartIdx );
  uiInterDir--;
  m_pcBinIf->encodeBin( ( uiInterDir == 2 ? 1 : 0 ), m_cCUInterDirSCModel.get( 0, 0, uiCtx ) );

  if ( uiInterDir < 2 )
  {
    m_pcBinIf->encodeBin( uiInterDir, m_cCUInterDirSCModel.get( 0, 0, 3 ) );
  }

  return;
}

Void TEncSbac::codeRefFrmIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  Int iRefFrame = pcCU->getCUMvField( eRefList )->getRefIdx( uiAbsPartIdx );

  UInt uiCtx = pcCU->getCtxRefIdx( uiAbsPartIdx, eRefList );

  m_pcBinIf->encodeBin( ( iRefFrame == 0 ? 0 : 1 ), m_cCURefPicSCModel.get( 0, 0, uiCtx ) );

  if ( iRefFrame > 0 )
  {
    xWriteUnaryMaxSymbol( iRefFrame - 1, &m_cCURefPicSCModel.get( 0, 0, 4 ), 1, pcCU->getSlice()->getNumRefIdx( eRefList )-2 );
  }
  return;
}

Void TEncSbac::codeMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  TComCUMvField* pcCUMvField = pcCU->getCUMvField( eRefList );
  Int iHor = pcCUMvField->getMvd( uiAbsPartIdx ).getHor();
  Int iVer = pcCUMvField->getMvd( uiAbsPartIdx ).getVer();

  UInt uiAbsPartIdxL, uiAbsPartIdxA;
  Int iHorPred, iVerPred;

  TComDataCU* pcCUL   = pcCU->getPULeft ( uiAbsPartIdxL, pcCU->getZorderIdxInCU() + uiAbsPartIdx );
  TComDataCU* pcCUA   = pcCU->getPUAbove( uiAbsPartIdxA, pcCU->getZorderIdxInCU() + uiAbsPartIdx );

  TComCUMvField* pcCUMvFieldL = ( pcCUL == NULL || pcCUL->isIntra( uiAbsPartIdxL ) ) ? NULL : pcCUL->getCUMvField( eRefList );
  TComCUMvField* pcCUMvFieldA = ( pcCUA == NULL || pcCUA->isIntra( uiAbsPartIdxA ) ) ? NULL : pcCUA->getCUMvField( eRefList );

#ifdef QC_AMVRES
  if(pcCU->getSlice()->getSPS()->getUseAMVRes())
  {
	  TComMv rcMv  = pcCUMvField->getMv ( uiAbsPartIdx );
	  Int iHor = pcCUMvField->getMvd( uiAbsPartIdx ).getHor();
	  Int iVer = pcCUMvField->getMvd( uiAbsPartIdx ).getVer();
	  Int iL =   ( (pcCUMvFieldL == NULL) ? 1 : (Int)(pcCUMvFieldL->getMVRes(uiAbsPartIdxL)));
	  Int iV =   ( (pcCUMvFieldA == NULL) ? 1 : (Int)(pcCUMvFieldA->getMVRes(uiAbsPartIdxA)));

	  xWriteMvResFlag(!rcMv.isHAM(),iL+iV);
	  if (!rcMv.isHAM()) 
	  {
		  iHorPred = ( (pcCUMvFieldL == NULL) ? 0 : (pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsHor()>>1) ) +
					 ( (pcCUMvFieldA == NULL) ? 0 : (pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsHor()>>1) );
		  iVerPred = ( (pcCUMvFieldL == NULL) ? 0 : (pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsVer()>>1) ) +
					 ( (pcCUMvFieldA == NULL) ? 0 : (pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsVer()>>1) );
	      xWriteMvd( iHor/2, iHorPred, 0 );
	      xWriteMvd( iVer/2, iVerPred, 1 );
		  return;
	  }
  }
#endif
 
  iHorPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsHor() ) +
       ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsHor() );
  iVerPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsVer() ) +
       ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsVer() );

  xWriteMvd( iHor, iHorPred, 0 );
  xWriteMvd( iVer, iVerPred, 1 );

  return;
}

#ifdef DCM_PBIC
Void TEncSbac::codeMvdIcd( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  Int iZeroPatt = 0;
  TComMv acMvd[2];
  Int iIcParam[3];
  ContextModel *pcCtxModel;
  TComZeroTree* pcZTree;

#ifdef QC_AMVRES
  // Determine and Code MV resolution flag (if necessary)
  Bool bMvResFlag[2] = {false, false};
  if ( (eRefList == REF_PIC_LIST_0) || (eRefList == REF_PIC_LIST_X) )
    bMvResFlag[REF_PIC_LIST_0] = xCodeMvResFlag( pcCU, uiAbsPartIdx, REF_PIC_LIST_0 );
  if ( (eRefList == REF_PIC_LIST_1) || (eRefList == REF_PIC_LIST_X) )
    bMvResFlag[REF_PIC_LIST_1] = xCodeMvResFlag( pcCU, uiAbsPartIdx, REF_PIC_LIST_1 );
#endif

  // Identify the non-zero components
  if (eRefList == REF_PIC_LIST_X)
  {
    acMvd[ REF_PIC_LIST_0 ] = pcCU->getCUMvField( REF_PIC_LIST_0 )->getMvd( uiAbsPartIdx );
#ifdef QC_AMVRES
    if (bMvResFlag[REF_PIC_LIST_0] == true)
      acMvd[REF_PIC_LIST_0].scale_down();
#endif
    acMvd[ REF_PIC_LIST_1 ] = pcCU->getCUMvField( REF_PIC_LIST_1 )->getMvd( uiAbsPartIdx );
#ifdef QC_AMVRES
    if (bMvResFlag[REF_PIC_LIST_1] == true)
      acMvd[REF_PIC_LIST_1].scale_down();
#endif
    iZeroPatt |= ( acMvd[REF_PIC_LIST_0].getHor() == 0 ) ? 0 : 1;
    iZeroPatt |= ( acMvd[REF_PIC_LIST_0].getVer() == 0 ) ? 0 : 2;
    iZeroPatt |= ( acMvd[REF_PIC_LIST_1].getHor() == 0 ) ? 0 : 4;
    iZeroPatt |= ( acMvd[REF_PIC_LIST_1].getVer() == 0 ) ? 0 : 8;

    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      pcCU->getCUIcField()->getIcd( uiAbsPartIdx ).getIcParam( iIcParam[0], iIcParam[1], iIcParam[2] );
      iZeroPatt |= ( iIcParam[0] == 0 ) ? 0 : 16;
      iZeroPatt |= ( iIcParam[1] == 0 ) ? 0 : 32;
      iZeroPatt |= ( iIcParam[2] == 0 ) ? 0 : 64;

      pcZTree    = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDICDBI);
      pcCtxModel = getZTreeCtx(IDX_ZTREE_MVDICDBI);
    }
    else
    {
      pcZTree    = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDBI);
      pcCtxModel = getZTreeCtx(IDX_ZTREE_MVDBI);
    }
  }
  else
  {
    acMvd[ eRefList ] = pcCU->getCUMvField( eRefList )->getMvd( uiAbsPartIdx );
#ifdef QC_AMVRES
    if (bMvResFlag[eRefList] == true)
      acMvd[eRefList].scale_down();
#endif
    iZeroPatt |= ( acMvd[eRefList].getHor() == 0 ) ? 0 : 1;
    iZeroPatt |= ( acMvd[eRefList].getVer() == 0 ) ? 0 : 2;

    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      pcCU->getCUIcField()->getIcd( uiAbsPartIdx ).getIcParam( iIcParam[0], iIcParam[1], iIcParam[2] );
      iZeroPatt |= ( iIcParam[0] == 0 ) ? 0 : 4;
      assert ( iIcParam[1] == 0 );
      iZeroPatt |= ( iIcParam[2] == 0 ) ? 0 : 8;

      pcZTree    = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDICDUNI);
      pcCtxModel = getZTreeCtx(IDX_ZTREE_MVDICDUNI);
    }
    else
    {
      pcZTree    = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDUNI);
      pcCtxModel = getZTreeCtx(IDX_ZTREE_MVDUNI);
    }
  }

  // Encode zeroflag and zerotree (if necessary)
  if (iZeroPatt == 0)
    m_pcBinIf->encodeBin( 1, *getZTreeCtx(IDX_ZEROFLAG) );
  else
  {
    m_pcBinIf->encodeBin( 0, *getZTreeCtx(IDX_ZEROFLAG) );
    pcZTree->updateVal(iZeroPatt);
    codeZTree( pcZTree, pcZTree->m_pcRoot, pcCtxModel );
  }

  //Encode the non-zero components
  if ( (eRefList == REF_PIC_LIST_X) || (eRefList == REF_PIC_LIST_0) )
  {
    xWriteMvdNZ( acMvd[REF_PIC_LIST_0].getHor(), 0 );
    xWriteMvdNZ( acMvd[REF_PIC_LIST_0].getVer(), 1 );
  }
  if ( (eRefList == REF_PIC_LIST_X) || (eRefList == REF_PIC_LIST_1) )
  {
    xWriteMvdNZ( acMvd[REF_PIC_LIST_1].getHor(), 0 );
    xWriteMvdNZ( acMvd[REF_PIC_LIST_1].getVer(), 1 );
  }

  if (pcCU->getSlice()->getSPS()->getUseIC())
  {
    xWriteIcdNZ( iIcParam[0], 0 );
    xWriteIcdNZ( iIcParam[1], 1 );
    xWriteIcdNZ( iIcParam[2], 2 );
  }

}

Void TEncSbac::codeZTree( TComZeroTree* pcZTree, TComZTNode* pcZTNode, ContextModel *pcCtxModel )
{
  Int iVal, iLval, iRval;

  if (pcZTNode->IsLeaf() == false)
  {
    iLval = pcZTNode->m_pcLeft->m_iVal;
    iRval = pcZTNode->m_pcRight->m_iVal;

    iVal = iLval & iRval;
    m_pcBinIf->encodeBin( iVal, *(pcCtxModel + 2*pcZTNode->m_id) );

    if (iVal == 0)
      m_pcBinIf->encodeBin( iLval, *(pcCtxModel + 2*pcZTNode->m_id + 1) );

    if (iLval != 0)
      codeZTree( pcZTree,  pcZTNode->m_pcLeft, pcCtxModel);
    if (iRval != 0)
      codeZTree( pcZTree, pcZTNode->m_pcRight, pcCtxModel);
  }
}

ContextModel* TEncSbac::getZTreeCtx( Int iIdx )
{
  switch (iIdx)
  {
  case IDX_ZEROFLAG:
    return &m_cZTreeMV0SCModel.get(0, 0, 0);
    break;
  case IDX_ZTREE_MVDICDUNI:
    return &m_cZTreeMV1SCModel.get(0, 0, 0);
    break;
  case IDX_ZTREE_MVDICDBI:
    return &m_cZTreeMV2SCModel.get(0, 0, 0);
    break;
  case IDX_ZTREE_MVDUNI:
    return &m_cZTreeMV1SCModel.get(0, 0, 2);
    break;
  case IDX_ZTREE_MVDBI:
    return &m_cZTreeMV2SCModel.get(0, 0, 2);
    break;
  default:
    printf("Wrong index to TEncSbac::getZTreeCtx()\n");
    exit(0);
    break;
  }
}
#endif

Void TEncSbac::codeDeltaQP( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  Int iDQp  = pcCU->getQP( uiAbsPartIdx ) - pcCU->getSlice()->getSliceQp();

  if ( iDQp == 0 )
  {
    m_pcBinIf->encodeBin( 0, m_cCUDeltaQpSCModel.get( 0, 0, 0 ) );
  }
  else
  {
    m_pcBinIf->encodeBin( 1, m_cCUDeltaQpSCModel.get( 0, 0, 0 ) );

    UInt uiDQp = (UInt)( iDQp > 0 ? ( 2 * iDQp - 2 ) : ( -2 * iDQp - 1 ) );
    xWriteUnarySymbol( uiDQp, &m_cCUDeltaQpSCModel.get( 0, 0, 2 ), 1 );
  }

  return;
}

Void TEncSbac::codeCbf( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth )
{
#if HHI_RQT
  if( pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() )
  {
#if HHI_RQT_INTRA
    return;
#else
    if( !pcCU->isIntra( uiAbsPartIdx ) )
    {
      return;
    }
#endif
  }
#endif
  UInt uiCbf = pcCU->getCbf   ( uiAbsPartIdx, eType, uiTrDepth );
  UInt uiCtx = pcCU->getCtxCbf( uiAbsPartIdx, eType, uiTrDepth );

  m_pcBinIf->encodeBin( uiCbf , m_cCUCbfSCModel.get( 0, eType == TEXT_LUMA ? 0 : 1, 3 - uiCtx ) );

  return;
}

#if HHI_RQT
Void TEncSbac::codeQtCbf( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth )
{
  UInt uiCbf = pcCU->getCbf     ( uiAbsPartIdx, eType, uiTrDepth );
  UInt uiCtx = pcCU->getCtxQtCbf( uiAbsPartIdx, eType, uiTrDepth );
  m_pcBinIf->encodeBin( uiCbf , m_cCUQtCbfSCModel.get( 0, eType ? eType - 1 : eType, uiCtx ) );
  DTRACE_CABAC_V( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseQtCbf()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiCbf )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\tetype=" )
  DTRACE_CABAC_V( eType )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )
}

UInt xCheckCoeffPlainCNoRecur( const TCoeff* pcCoef, UInt uiSize, UInt uiDepth )
{
  UInt uiNumofCoeff = 0;
  UInt ui = uiSize>>uiDepth;
  {
    UInt x, y;
    const TCoeff* pCeoff = pcCoef;
    for( y=0 ; y<ui ; y++ )
    {
      for( x=0 ; x<ui ; x++ )
      {
        if( pCeoff[x] != 0 )
        {
          uiNumofCoeff++;
        }
      }
      pCeoff += uiSize;
    }
  }
  return uiNumofCoeff;
}

#if HHI_RQT_ROOT
Void TEncSbac::codeQtRootCbf( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiCbf = pcCU->getQtRootCbf( uiAbsPartIdx );
  UInt uiCtx = pcCU->getCtxQtRootCbf( uiAbsPartIdx );
  m_pcBinIf->encodeBin( uiCbf , m_cCUQtRootCbfSCModel.get( 0, 0, uiCtx ) );
  DTRACE_CABAC_V( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseQtRootCbf()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiCbf )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )
}
#endif
#endif

Void TEncSbac::xCheckCoeff( TCoeff* pcCoef, UInt uiSize, UInt uiDepth, UInt& uiNumofCoeff, UInt& uiPart )
{
  UInt ui = uiSize>>uiDepth;
  if( uiPart == 0 )
  {
    if( ui <= 4 )
    {
      UInt x, y;
      TCoeff* pCeoff = pcCoef;
      for( y=0 ; y<ui ; y++ )
      {
        for( x=0 ; x<ui ; x++ )
        {
          if( pCeoff[x] != 0 )
          {
            uiNumofCoeff++;
          }
        }
        pCeoff += uiSize;
      }
    }
    else
    {
      xCheckCoeff( pcCoef,                            uiSize, uiDepth+1, uiNumofCoeff, uiPart ); uiPart++; //1st Part
      xCheckCoeff( pcCoef             + (ui>>1),      uiSize, uiDepth+1, uiNumofCoeff, uiPart ); uiPart++; //2nd Part
      xCheckCoeff( pcCoef + (ui>>1)*uiSize,           uiSize, uiDepth+1, uiNumofCoeff, uiPart ); uiPart++; //3rd Part
      xCheckCoeff( pcCoef + (ui>>1)*uiSize + (ui>>1), uiSize, uiDepth+1, uiNumofCoeff, uiPart );           //4th Part
    }
  }
  else
  {
    UInt x, y;
    TCoeff* pCeoff = pcCoef;
    for( y=0 ; y<ui ; y++ )
    {
      for( x=0 ; x<ui ; x++ )
      {
        if( pCeoff[x] != 0 )
        {
          uiNumofCoeff++;
        }
      }
      pCeoff += uiSize;
    }
  }
}

#if HHI_RQT
UInt xCheckCoeffPlainCNoRecur( const TCoeff* pcCoef, UInt uiSize, UInt uiDepth );
#endif

#if QC_MDDT
Void TEncSbac::codeCoeffNxN( TComDataCU* pcCU, TCoeff* pcCoef, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight, UInt uiDepth, TextType eTType, UInt uiMode, Bool bRD )
#else
Void TEncSbac::codeCoeffNxN( TComDataCU* pcCU, TCoeff* pcCoef, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight, UInt uiDepth, TextType eTType, Bool bRD )
#endif
{
#if HHI_RQT
  DTRACE_CABAC_V( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseCoeffNxN()\teType=" )
  DTRACE_CABAC_V( eTType )
  DTRACE_CABAC_T( "\twidth=" )
  DTRACE_CABAC_V( uiWidth )
  DTRACE_CABAC_T( "\theight=" )
  DTRACE_CABAC_V( uiHeight )
  DTRACE_CABAC_T( "\tdepth=" )
  DTRACE_CABAC_V( uiDepth )
  DTRACE_CABAC_T( "\tabspartidx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\ttoCU-X=" )
  DTRACE_CABAC_V( pcCU->getCUPelX() )
  DTRACE_CABAC_T( "\ttoCU-Y=" )
  DTRACE_CABAC_V( pcCU->getCUPelY() )
  DTRACE_CABAC_T( "\tCU-addr=" )
  DTRACE_CABAC_V(  pcCU->getAddr() )
  DTRACE_CABAC_T( "\tinCU-X=" )
  DTRACE_CABAC_V( g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ] )
  DTRACE_CABAC_T( "\tinCU-Y=" )
  DTRACE_CABAC_V( g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ] )
  DTRACE_CABAC_T( "\tpredmode=" )
  DTRACE_CABAC_V(  pcCU->getPredictionMode( uiAbsPartIdx ) )
  DTRACE_CABAC_T( "\n" )
#endif
#if HHI_TRANSFORM_CODING
  if( uiWidth > m_pcSlice->getSPS()->getMaxTrSize() )
  {
    uiWidth  = m_pcSlice->getSPS()->getMaxTrSize();
    uiHeight = m_pcSlice->getSPS()->getMaxTrSize();
  }

  UInt uiNumSig = 0;
  UInt uiCTXIdx = xGetCTXIdxFromWidth( uiWidth );

  // compute number of significant coefficients
  UInt  uiPart = 0;
  xCheckCoeff(pcCoef, uiWidth, 0, uiNumSig, uiPart );

  if ( bRD )
  {
    UInt uiTempDepth = uiDepth - pcCU->getDepth( uiAbsPartIdx );
    pcCU->setCbfSubParts( ( uiNumSig ? 1 : 0 ) << uiTempDepth, eTType, uiAbsPartIdx, uiDepth );
    codeCbf( pcCU, uiAbsPartIdx, eTType, uiTempDepth );
  }

  if ( uiNumSig == 0 )
    return;

  eTType = eTType == TEXT_LUMA ? TEXT_LUMA : ( eTType == TEXT_NONE ? TEXT_NONE : TEXT_CHROMA );

   //----- encode significance map -----
  const UInt   uiMaxNumCoeff     = uiWidth*uiHeight;
  const UInt   uiMaxNumCoeffM1   = uiMaxNumCoeff - 1;
  const UInt   uiLog2BlockSize   = g_aucConvertToBit[ uiWidth ] + 2;
  UInt         uiDownLeft        = 1;
  UInt         uiNumSigTopRight  = 0;
  UInt         uiNumSigBotLeft   = 0;

#if QC_MDDT//ADAPTIVE_SCAN
  const UInt*  pucScan;
  const UInt*  pucScanX;
  const UInt*  pucScanY;
  UInt *scanStats;
  int indexROT ;
  if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && (uiWidth == 4 || uiWidth == 8 /*|| uiWidth == 16 || uiWidth == 32 || uiWidth == 64*/))//!bRD &&  eTType == TEXT_LUMA)
  {
	indexROT = pcCU->getROTindex(uiAbsPartIdx);
	//int scan_index;
#if ROT_CHECK
    if(uiWidth == 4 && indexROT == 0)
#else
    if(uiWidth == 4)
#endif
    {
       UInt uiPredMode = g_aucIntra9Mode[uiMode];
       pucScan = scanOrder4x4[uiPredMode]; pucScanX = scanOrder4x4X[uiPredMode]; pucScanY = scanOrder4x4Y[uiPredMode];

	   if(g_bUpdateStats)
       {
         scanStats = scanStats4x4[uiPredMode]; update4x4Count[uiPredMode]++;
       }
    }
#if ROT_CHECK
    else if(uiWidth == 8 && indexROT == 0)
#else
    else if(uiWidth == 8)
#endif
    {
      UInt uiPredMode = ((1 << (pcCU->getIntraSizeIdx( uiAbsPartIdx ) + 1)) != uiWidth) ?  g_aucIntra9Mode[uiMode]: g_aucAngIntra9Mode[uiMode];
      pucScan = scanOrder8x8[uiPredMode]; pucScanX = scanOrder8x8X[uiPredMode]; pucScanY = scanOrder8x8Y[uiPredMode];

	   if(g_bUpdateStats)
      {
        scanStats = scanStats8x8[uiPredMode]; update8x8Count[uiPredMode]++;
      }
    }
	/*else if(uiWidth == 16)
	{
		scan_index = LUT16x16[indexROT][uiMode];
		pucScan = scanOrder16x16[scan_index]; pucScanX = scanOrder16x16X[scan_index]; pucScanY = scanOrder16x16Y[scan_index];

	   if(g_bUpdateStats)
		{
			scanStats = scanStats16x16[scan_index];
		}
    }
    else if(uiWidth == 32)
    {
		scan_index = LUT32x32[indexROT][uiMode];
		pucScan = scanOrder32x32[scan_index]; pucScanX = scanOrder32x32X[scan_index]; pucScanY = scanOrder32x32Y[scan_index];

	   if(g_bUpdateStats)
		{
			scanStats = scanStats32x32[scan_index];
		}
    }
    else if(uiWidth == 64)
    {
		scan_index = LUT64x64[indexROT][uiMode];
		pucScan = scanOrder64x64[scan_index]; pucScanX = scanOrder64x64X[scan_index]; pucScanY = scanOrder64x64Y[scan_index];

	   if(g_bUpdateStats)
		{
			scanStats = scanStats64x64[scan_index];
		}
    }*/
    else
    {
      //printf("uiWidth = %d is not supported!\n", uiWidth);
      //exit(1);
    }
  }
#endif

  for( UInt uiScanPos = 0; uiScanPos < uiMaxNumCoeffM1; uiScanPos++ )
  {
    UInt  uiBlkPos  = g_auiSigLastScan[ uiLog2BlockSize ][ uiDownLeft ][ uiScanPos ];
#if QC_MDDT
    if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && (uiWidth == 4 || uiWidth == 8 /*|| uiWidth == 16 || uiWidth == 32 || uiWidth == 64*/))//!bRD &&  eTType == TEXT_LUMA)
    {
#if ROT_CHECK
      if(/* uiMode<=8  && */ indexROT == 0) 
        uiBlkPos = pucScan[uiScanPos];
#else
        uiBlkPos = pucScan[uiScanPos];
#endif
    }
#endif
    UInt  uiPosY    = uiBlkPos >> uiLog2BlockSize;
    UInt  uiPosX    = uiBlkPos - ( uiPosY << uiLog2BlockSize );

    //===== code significance flag =====
    UInt  uiSig    = pcCoef[ uiBlkPos ] != 0 ? 1 : 0;
    UInt  uiCtxSig = TComTrQuant::getSigCtxInc( pcCoef, uiPosX, uiPosY, uiLog2BlockSize, uiWidth, ( uiDownLeft > 0 ) );
    m_pcBinIf->encodeBin( uiSig, m_cCuCtxModSig.get( uiCTXIdx, eTType, uiCtxSig ) );

    if( uiSig )
    {
#if QC_MDDT//ADAPTIVE_SCAN
#if ROT_CHECK
      if(g_bUpdateStats && pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && ((uiWidth == 4 && /* uiMode<=8 && */ indexROT == 0)|| (uiWidth == 8 && /* uiMode<=8 &&*/ indexROT == 0) /*|| uiWidth == 16 || uiWidth == 32 || uiWidth == 64*/))
#else
     if(g_bUpdateStats && pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && (uiWidth == 4 || uiWidth == 8))
#endif
      {
        scanStats[uiScanPos]++;
      }
#endif
      uiNumSig--;

      if( uiPosX > uiPosY )
      {
        uiNumSigTopRight++;
      }
      else if( uiPosY > uiPosX )
      {
        uiNumSigBotLeft ++;
      }

      //===== code last flag =====
      UInt  uiLast    = ( uiNumSig == 0 ) ? 1 : 0;
      UInt  uiCtxLast = TComTrQuant::getLastCtxInc( uiPosX, uiPosY, uiLog2BlockSize );
      m_pcBinIf->encodeBin( uiLast, m_cCuCtxModLast.get( uiCTXIdx, eTType, uiCtxLast ) );

      if( uiLast )
      {
        break;
      }
    }

    //===== update scan direction =====
#if !HHI_DISABLE_SCAN
    if( ( uiDownLeft == 1 && ( uiPosX == 0 || uiPosY == uiHeight - 1 ) ) ||
        ( uiDownLeft == 0 && ( uiPosY == 0 || uiPosX == uiWidth  - 1 ) )   )
    {
      uiDownLeft = ( uiNumSigTopRight >= uiNumSigBotLeft ? 1 : 0 );
    }
#else
		if( uiScanPos && 
			( ( uiDownLeft == 1 && ( uiPosX == 0 || uiPosY == uiHeight - 1 ) ) ||
        ( uiDownLeft == 0 && ( uiPosY == 0 || uiPosX == uiWidth  - 1 ) )   ) )
    {
      uiDownLeft = 1 - uiDownLeft;
    }
#endif
  }

  Int  c1, c2;
  UInt uiAbs;
  UInt uiSign;
  UInt uiPrevAbsGreOne     = 0;
  const UInt uiNumOfSets   = 4;
  const UInt uiNum4x4Blk   = max<UInt>( 1, uiMaxNumCoeff / 16 );

  if ( uiLog2BlockSize > 2 )
  {
    Bool bFirstBlock = true;

    for ( UInt uiSubBlk = 0; uiSubBlk < uiNum4x4Blk; uiSubBlk++ )
    {
      UInt uiCtxSet    = 0;
      UInt uiSubNumSig = 0;
      UInt uiSubLog2   = uiLog2BlockSize - 2;
      UInt uiSubPosX   = 0;
      UInt uiSubPosY   = 0;

      if ( uiSubLog2 > 1 )
      {
#if HHI_RQT
        uiSubPosX = g_auiFrameScanX[ uiSubLog2 - 1 ][ uiSubBlk ] * 4;
        uiSubPosY = g_auiFrameScanY[ uiSubLog2 - 1 ][ uiSubBlk ] * 4;
#else
        uiSubPosX = g_auiFrameScanX[ uiSubLog2 - 2 ][ uiSubBlk ] * 4;
        uiSubPosY = g_auiFrameScanY[ uiSubLog2 - 2 ][ uiSubBlk ] * 4;
#endif
      }
      else
      {
        uiSubPosX = ( uiSubBlk < 2      ) ? 0 : 1;
        uiSubPosY = ( uiSubBlk % 2 == 0 ) ? 0 : 1;
        uiSubPosX *= 4;
        uiSubPosY *= 4;
      }

      TCoeff* piCurr = &pcCoef[ uiSubPosX + uiSubPosY * uiWidth ];

      for ( UInt uiY = 0; uiY < 4; uiY++ )
      {
        for ( UInt uiX = 0; uiX < 4; uiX++ )
        {
          if( piCurr[ uiX ] )
          {
            uiSubNumSig++;
          }
        }
        piCurr += uiWidth;
      }

      if ( uiSubNumSig > 0 )
      {
        c1 = 1;
        c2 = 0;

        if ( bFirstBlock )
        {
          bFirstBlock = false;
          uiCtxSet = uiNumOfSets + 1;
        }
        else
        {
          uiCtxSet = uiPrevAbsGreOne / uiNumOfSets + 1;//( ( uiGreOne * uiEqRangeSize ) >> 8 ) + 1;
          uiPrevAbsGreOne = 0;
        }

        for ( UInt uiScanPos = 0; uiScanPos < 16; uiScanPos++ )
        {
#if HHI_RQT
          UInt  uiBlkPos  = g_auiFrameScanXY[ 1 ][ 15 - uiScanPos ];
#else
          UInt  uiBlkPos  = g_auiFrameScanXY[ 0 ][ 15 - uiScanPos ];
#endif
          UInt  uiPosY    = uiBlkPos >> 2;
          UInt  uiPosX    = uiBlkPos - ( uiPosY << 2 );
          UInt  uiIndex   = (uiSubPosY + uiPosY) * uiWidth + uiSubPosX + uiPosX;

          if( pcCoef[ uiIndex ]  )
          {
            if( pcCoef[ uiIndex ] > 0) { uiAbs = static_cast<UInt>( pcCoef[ uiIndex ]);  uiSign = 0; }
            else                       { uiAbs = static_cast<UInt>(-pcCoef[ uiIndex ]);  uiSign = 1; }

            UInt uiCtx    = min<UInt>(c1, 4);
            UInt uiSymbol = uiAbs > 1 ? 1 : 0;
            m_pcBinIf->encodeBin( uiSymbol, m_cCuCtxModAbsGreOne.get( 0, eTType, (uiCtxSet * 5) + uiCtx ) );

            if( uiSymbol )
            {
              uiCtx  = min<UInt>(c2, 4);
              uiAbs -= 2;
              c1     = 0;
              c2++;
              uiPrevAbsGreOne++;
              xWriteExGolombLevel( uiAbs, m_cCuCtxModCoeffLevelM1.get( 0, eTType, (uiCtxSet * 5) + uiCtx ) );
            }
            else if( c1 )
            {
              c1++;
            }
            m_pcBinIf->encodeBinEP( uiSign );
          }
        }
      }
    }
  }
  else
  {
    c1 = 1;
    c2 = 0;

    for ( UInt uiScanPos = 0; uiScanPos < uiWidth*uiHeight; uiScanPos++ )
    {
#if HHI_RQT
      UInt uiIndex = g_auiFrameScanXY[ (int)g_aucConvertToBit[ uiWidth ] + 1 ][ uiWidth*uiHeight - uiScanPos - 1 ];
#else
      UInt uiIndex = g_auiFrameScanXY[ (int)g_aucConvertToBit[ uiWidth ] ][ uiWidth*uiHeight - uiScanPos - 1 ];
#endif

      if( pcCoef[ uiIndex ]  )
      {
        if( pcCoef[ uiIndex ] > 0) { uiAbs = static_cast<UInt>( pcCoef[ uiIndex ]);  uiSign = 0; }
        else                       { uiAbs = static_cast<UInt>(-pcCoef[ uiIndex ]);  uiSign = 1; }

        UInt uiCtx    = min<UInt>(c1, 4);
        UInt uiSymbol = uiAbs > 1 ? 1 : 0;
        m_pcBinIf->encodeBin( uiSymbol, m_cCuCtxModAbsGreOne.get( 0, eTType, uiCtx ) );

        if( uiSymbol )
        {
          uiCtx  = min<UInt>(c2, 4);
          uiAbs -= 2;
          c1     = 0;
          c2++;
          xWriteExGolombLevel( uiAbs, m_cCuCtxModCoeffLevelM1.get( 0, eTType, uiCtx ) );
        }
        else if( c1 )
        {
          c1++;
        }
        m_pcBinIf->encodeBinEP( uiSign );
      }
    }
  }
  return;
#else
  UInt uiCTXIdx = xGetCTXIdxFromWidth( uiWidth );

  if( uiWidth > m_pcSlice->getSPS()->getMaxTrSize() )
  {
    uiWidth  = m_pcSlice->getSPS()->getMaxTrSize();
    uiHeight = m_pcSlice->getSPS()->getMaxTrSize();
  }
  UInt uiSize   = uiWidth*uiHeight;

	// point to coefficient
  TCoeff* piCoeff = pcCoef;
  UInt uiNumSig = 0;
  UInt ui;

	// compute number of significant coefficients
  UInt  uiPart = 0;
  xCheckCoeff(piCoeff, uiWidth, 0, uiNumSig, uiPart );
#if HHI_RQT
  assert( uiNumSig == xCheckCoeffPlainCNoRecur( piCoeff, uiWidth, 0 ) );
#endif

  if ( bRD )
  {
    UInt uiTempDepth = uiDepth - pcCU->getDepth( uiAbsPartIdx );
    pcCU->setCbfSubParts( ( uiNumSig ? 1 : 0 ) << uiTempDepth, eTType, uiAbsPartIdx, uiDepth );
    codeCbf( pcCU, uiAbsPartIdx, eTType, uiTempDepth );
  }

  if ( uiNumSig == 0 )
    return;

  UInt uiCodedSig = 0;
  UInt uiCtxSize  = 64;

  const Int* pos2ctx_map  = pos2ctx_map8x8;
  const Int* pos2ctx_last = pos2ctx_last8x8;
  if ( uiWidth < 8 )
  {
    pos2ctx_map  = pos2ctx_nomap;
    pos2ctx_last = pos2ctx_nomap;
    uiCtxSize    = 16;
  }

  eTType = eTType == TEXT_LUMA ? TEXT_LUMA : ( eTType == TEXT_NONE ? TEXT_NONE : TEXT_CHROMA );

  UInt uiSig, uiCtx, uiLast;
  uiLast = 0;
#if QC_MDDT == 0//ADAPTIVE_SCAN == 0
  uiSig = piCoeff[0] ? 1 : 0;

  m_pcBinIf->encodeBin( uiSig, m_cCUMapSCModel.get( uiCTXIdx, eTType, pos2ctx_map[ 0 ] ) );

  if( uiSig )
  {
    uiLast = (++uiCodedSig == uiNumSig ? 1 : 0);
    m_pcBinIf->encodeBin( uiLast, m_cCULastSCModel.get( uiCTXIdx, eTType, pos2ctx_last[ 0 ] ) );
  }
#endif
  // initialize scan
  const UInt*  pucScan;
  const UInt*  pucScanX;
  const UInt*  pucScanY;

  UInt uiConvBit = g_aucConvertToBit[ uiWidth ];
#if HHI_RQT
  pucScan  = g_auiFrameScanXY[ uiConvBit + 1 ]; 
  pucScanX = g_auiFrameScanX [ uiConvBit + 1 ];
  pucScanY = g_auiFrameScanY [ uiConvBit + 1 ];
#else
  switch(uiWidth)
  {
  case  2:
    {
      static UInt uiScanOrder [4] = {0, 1, 2, 3};
      static UInt uiScanOrderX[4] = {0, 1, 0, 1};
      static UInt uiScanOrderY[4] = {0, 0, 1, 1};
      pucScan  = &uiScanOrder [0];
      pucScanX = &uiScanOrderX[0];
      pucScanY = &uiScanOrderY[0];
      break;
    }
  case  4:
  case  8: pucScan = g_auiFrameScanXY[ uiConvBit ]; pucScanX = g_auiFrameScanX[uiConvBit]; pucScanY = g_auiFrameScanY[uiConvBit]; break;
  case 16:
  case 32:
  case 64:
  default: pucScan = g_auiFrameScanXY[ uiConvBit ]; pucScanX = g_auiFrameScanX[uiConvBit]; pucScanY = g_auiFrameScanY[uiConvBit]; break;
  }
#endif
#if QC_MDDT//ADAPTIVE_SCAN
  UInt *scanStats;
  int indexROT ;
  if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA)//!bRD &&  eTType == TEXT_LUMA)
  {
    UInt uiPredMode = g_aucIntra9Mode[uiMode];
	indexROT = pcCU->getROTindex(uiAbsPartIdx);
	int scan_index;
#if ROT_CHECK
    if(uiWidth == 4 && indexROT == 0)
#else
    if(uiWidth == 4)
#endif
    {
       UInt uiPredMode = g_aucIntra9Mode[uiMode];

       pucScan = scanOrder4x4[uiPredMode]; pucScanX = scanOrder4x4X[uiPredMode]; pucScanY = scanOrder4x4Y[uiPredMode];


	   if(g_bUpdateStats)
       {
         scanStats = scanStats4x4[uiPredMode]; update4x4Count[uiPredMode]++;
       }
    }
#if ROT_CHECK
    else if(uiWidth == 8 && indexROT == 0)
#else
    else if(uiWidth == 8)
#endif
    {
      UInt uiPredMode = ((1 << (pcCU->getIntraSizeIdx( uiAbsPartIdx ) + 1)) != uiWidth) ? g_aucIntra9Mode[uiMode] : g_aucAngIntra9Mode[uiMode];
      pucScan = scanOrder8x8[uiPredMode]; pucScanX = scanOrder8x8X[uiPredMode]; pucScanY = scanOrder8x8Y[uiPredMode];

	   if(g_bUpdateStats)
      {
        scanStats = scanStats8x8[uiPredMode]; update8x8Count[uiPredMode]++;
      }
    }
	else if(uiWidth == 16)
	{
		scan_index = LUT16x16[indexROT][uiMode];
		pucScan = scanOrder16x16[scan_index]; pucScanX = scanOrder16x16X[scan_index]; pucScanY = scanOrder16x16Y[scan_index];

	   if(g_bUpdateStats)
		{
			scanStats = scanStats16x16[scan_index];
		}
    }
    else if(uiWidth == 32)
    {
		scan_index = LUT32x32[indexROT][uiMode];
		pucScan = scanOrder32x32[scan_index]; pucScanX = scanOrder32x32X[scan_index]; pucScanY = scanOrder32x32Y[scan_index];

	   if(g_bUpdateStats)
		{
			scanStats = scanStats32x32[scan_index];
		}
    }
    else if(uiWidth == 64)
    {
		scan_index = LUT64x64[indexROT][uiMode];
		pucScan = scanOrder64x64[scan_index]; pucScanX = scanOrder64x64X[scan_index]; pucScanY = scanOrder64x64Y[scan_index];

	   if(g_bUpdateStats)
		{
			scanStats = scanStats64x64[scan_index];
		}
    }
    else
    {
      //printf("uiWidth = %d is not supported!\n", uiWidth);
      //exit(1);
    }
  }
#endif
	//----- encode significance map -----
  // DC is coded in the beginning
#if QC_MDDT//ADAPTIVE_SCAN
  for( ui = 0; ui < ( uiSize - 1 ) && !uiLast; ui++ ) // if last coeff is reached, it has to be significant
#else
  for( ui = 1; ui < ( uiSize - 1 ) && !uiLast; ui++ ) // if last coeff is reached, it has to be significant
#endif
	{
		uiSig = piCoeff[ pucScan[ ui ] ] ? 1 : 0;

#if HHI_RQT
    if (uiCtxSize < uiSize)
#else
    if (uiCtxSize != uiSize)
#endif
    {
      UInt uiXX, uiYY;
      uiXX = pucScanX[ui]/(uiWidth / 8);
      uiYY = pucScanY[ui]/(uiHeight / 8);

      uiCtx = g_auiAntiScan8[uiYY*8+uiXX];
    }
    else
      uiCtx = ui * uiCtxSize / uiSize;

#if QC_MDDT//ADAPTIVE_SCAN
#if ROT_CHECK
     if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && uiWidth == 8 /* && uiMode<=8*/ && indexROT == 0)//!bRD &&  eTType == TEXT_LUMA)
#else
     if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && uiWidth == 8)
#endif
     {
       assert(uiCtxSize == uiSize);
#ifdef QC_CTX
	   m_pcBinIf->encodeBin( uiSig, m_cCUMapSCModel.get( uiCTXIdx, eTType, pos2ctx_map[ uiCtx ] ) );
#else
       m_pcBinIf->encodeBin( uiSig, m_cCUMapSCModel.get( uiCTXIdx, eTType, pos2ctx_map[ g_auiAntiScan8[pucScan[ui]] ] ) );
#endif
     }
     else
#endif
		m_pcBinIf->encodeBin( uiSig, m_cCUMapSCModel.get( uiCTXIdx, eTType, pos2ctx_map[ uiCtx ] ) );

		if( uiSig )
		{
#if QC_MDDT//ADAPTIVE_SCAN
#if ROT_CHECK
      if(g_bUpdateStats && pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && ((uiWidth == 4 && /*uiMode<=8 &&*/ indexROT == 0)|| (uiWidth == 8 &&/*uiMode<=8 &&*/ indexROT == 0) || uiWidth == 16 || uiWidth == 32 || uiWidth == 64)) 
#else
      if(g_bUpdateStats && pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA )
#endif
      {
			  scanStats[ui]++;
      }
#endif
      uiCtx = ui * uiCtxSize / uiSize;
			uiLast = (++uiCodedSig == uiNumSig ? 1 : 0);
			m_pcBinIf->encodeBin( uiLast, m_cCULastSCModel.get( uiCTXIdx, eTType, pos2ctx_last[ uiCtx ] ) );

			if( uiLast )
			{
				break;
			}
		}
	}

  Int   c1 = 1;
  Int   c2 = 0;
  //----- encode significant coefficients -----
  ui++;
  while( (ui--) != 0 )
  {
    UInt  uiAbs, uiSign;
    Int   iCoeff = piCoeff[ pucScan[ ui ] ];

    if( iCoeff )
    {
      if( iCoeff > 0) { uiAbs = static_cast<UInt>( iCoeff);  uiSign = 0; }
      else            { uiAbs = static_cast<UInt>(-iCoeff);  uiSign = 1; }

      uiCtx    = Min (c1, 4);
      UInt uiSymbol = uiAbs > 1 ? 1 : 0;
      m_pcBinIf->encodeBin( uiSymbol, m_cCUOneSCModel.get( uiCTXIdx, eTType, uiCtx ) );

      if( uiSymbol )
      {
        uiCtx  = Min (c2,4);
        uiAbs -= 2;
        c1     = 0;
        c2++;
				xWriteExGolombLevel( uiAbs, m_cCUAbsSCModel.get( uiCTXIdx, eTType, uiCtx ) );
      }
      else if( c1 )
      {
        c1++;
      }
      m_pcBinIf->encodeBinEP( uiSign );
    }
  }
  return;
#endif
}

Void TEncSbac::xWriteMvd( Int iMvd, UInt uiAbsSum, UInt uiCtx )
{
  UInt uiLocalCtx = 0;
  if ( uiAbsSum >= 3 )
  {
    uiLocalCtx += ( uiAbsSum > 32 ) ? 2 : 1;
  }

  UInt uiSymbol = ( 0 == iMvd ) ? 0 : 1;
  m_pcBinIf->encodeBin( uiSymbol, m_cCUMvdSCModel.get( 0, uiCtx, uiLocalCtx ) );
  if ( 0 == uiSymbol )
  {
    return;
  }

  UInt uiSign = 0;
  if ( 0 > iMvd )
  {
    uiSign = 1;
    iMvd   = -iMvd;
  }
  xWriteExGolombMvd( iMvd-1, &m_cCUMvdSCModel.get( 0, uiCtx, 3 ), 3 );
  m_pcBinIf->encodeBinEP( uiSign );

  return;
}

#ifdef DCM_PBIC
Void TEncSbac::xWriteMvdNZ( Int iMvd, UInt uiCtx )
{
  if ( iMvd == 0 )
    return;

  UInt uiSign = 0;
  if ( 0 > iMvd )
  {
    uiSign = 1;
    iMvd   = -iMvd;
  }
  xWriteExGolombMvd( iMvd-1, &m_cCUMvdSCModel.get( 0, uiCtx, 3 ), 3 );
  m_pcBinIf->encodeBinEP( uiSign );

  return;
}

Void TEncSbac::xWriteIcdNZ( Int iIcd, UInt uiCtx )
{
  if ( iIcd == 0 )
    return;

  UInt uiSign = 0;
  if ( 0 > iIcd )
  {
    uiSign = 1;
    iIcd   = -iIcd;
  }
  xWriteExGolombIcd( iIcd-1, &m_cCUIcdSCModel.get( 0, uiCtx, 3 ), 3 );
  m_pcBinIf->encodeBinEP( uiSign );

  return;
}

Void  TEncSbac::xWriteExGolombIcd( UInt uiSymbol, ContextModel* pcSCModel, UInt uiMaxBin )
{
  if ( ! uiSymbol )
  {
    m_pcBinIf->encodeBin( 0, *pcSCModel );
    return;
  }

  m_pcBinIf->encodeBin( 1, *pcSCModel );

  Bool bNoExGo = ( uiSymbol < 8 );
  UInt uiCount = 1;
  pcSCModel++;

  while ( --uiSymbol && ++uiCount <= 8 )
  {
    m_pcBinIf->encodeBin( 1, *pcSCModel );
    if ( uiCount == 2 )
    {
      pcSCModel++;
    }
    if ( uiCount == uiMaxBin )
    {
      pcSCModel++;
    }
  }

  if ( bNoExGo )
  {
    m_pcBinIf->encodeBin( 0, *pcSCModel );
  }
  else
  {
    xWriteEpExGolomb( uiSymbol, 3 );
  }

  return;
}
#endif

Void  TEncSbac::xWriteExGolombMvd( UInt uiSymbol, ContextModel* pcSCModel, UInt uiMaxBin )
{
  if ( ! uiSymbol )
  {
    m_pcBinIf->encodeBin( 0, *pcSCModel );
    return;
  }

  m_pcBinIf->encodeBin( 1, *pcSCModel );

  Bool bNoExGo = ( uiSymbol < 8 );
  UInt uiCount = 1;
  pcSCModel++;

  while ( --uiSymbol && ++uiCount <= 8 )
  {
    m_pcBinIf->encodeBin( 1, *pcSCModel );
    if ( uiCount == 2 )
    {
      pcSCModel++;
    }
    if ( uiCount == uiMaxBin )
    {
      pcSCModel++;
    }
  }

  if ( bNoExGo )
  {
    m_pcBinIf->encodeBin( 0, *pcSCModel );
  }
  else
  {
    xWriteEpExGolomb( uiSymbol, 3 );
  }

  return;
}

Void TEncSbac::codeAlfFlag       ( UInt uiCode )
{
  UInt uiSymbol = ( ( uiCode == 0 ) ? 0 : 1 );
  m_pcBinIf->encodeBin( uiSymbol, m_cALFFlagSCModel.get( 0, 0, 0 ) );
}

#if TSB_ALF_HEADER
Void TEncSbac::codeAlfFlagNum( UInt uiCode, UInt minValue )
{
  UInt uiLength = 0;
  UInt maxValue = (minValue << (this->getMaxAlfCtrlDepth()*2));
  assert((uiCode>=minValue)&&(uiCode<=maxValue));
  UInt temp = maxValue - minValue;
  for(UInt i=0; i<32; i++)
  {
    if(temp&0x1)
    {
      uiLength = i+1;
    }
    temp = (temp >> 1);
  }
  UInt uiSymbol = uiCode - minValue;
  if(uiLength)
  {
    while( uiLength-- )
    {
      m_pcBinIf->encodeBinEP( (uiSymbol>>uiLength) & 0x1 );
    }
  }
}

Void TEncSbac::codeAlfCtrlFlag( UInt uiSymbol )
{
  m_pcBinIf->encodeBin( uiSymbol, m_cCUAlfCtrlFlagSCModel.get( 0, 0, 0) );
}
#endif

Void TEncSbac::codeAlfUvlc       ( UInt uiCode )
{
  Int i;

  if ( uiCode == 0 )
  {
    m_pcBinIf->encodeBin( 0, m_cALFUvlcSCModel.get( 0, 0, 0 ) );
  }
  else
  {
    m_pcBinIf->encodeBin( 1, m_cALFUvlcSCModel.get( 0, 0, 0 ) );
    for ( i=0; i<uiCode-1; i++ )
    {
      m_pcBinIf->encodeBin( 1, m_cALFUvlcSCModel.get( 0, 0, 1 ) );
    }
    m_pcBinIf->encodeBin( 0, m_cALFUvlcSCModel.get( 0, 0, 1 ) );
  }
}

Void TEncSbac::codeAlfSvlc       ( Int iCode )
{
  Int i;

  if ( iCode == 0 )
  {
    m_pcBinIf->encodeBin( 0, m_cALFSvlcSCModel.get( 0, 0, 0 ) );
  }
  else
  {
    m_pcBinIf->encodeBin( 1, m_cALFSvlcSCModel.get( 0, 0, 0 ) );

    // write sign
    if ( iCode > 0 )
    {
      m_pcBinIf->encodeBin( 0, m_cALFSvlcSCModel.get( 0, 0, 1 ) );
    }
    else
    {
      m_pcBinIf->encodeBin( 1, m_cALFSvlcSCModel.get( 0, 0, 1 ) );
      iCode = -iCode;
    }

    // write magnitude
    for ( i=0; i<iCode-1; i++ )
    {
      m_pcBinIf->encodeBin( 1, m_cALFSvlcSCModel.get( 0, 0, 2 ) );
    }
    m_pcBinIf->encodeBin( 0, m_cALFSvlcSCModel.get( 0, 0, 2 ) );
  }
}

/*!
 ****************************************************************************
 * \brief
 *   estimate bit cost for CBP, significant map and significant coefficients
 ****************************************************************************
 */
Void TEncSbac::estBit (estBitsSbacStruct* pcEstBitsSbac, UInt uiCTXIdx, TextType eTType)
{
  estCBFBit (pcEstBitsSbac, 0, eTType);

  // encode significance map
  estSignificantMapBit (pcEstBitsSbac, uiCTXIdx, eTType);

  // encode significant coefficients
  estSignificantCoefficientsBit (pcEstBitsSbac, uiCTXIdx, eTType);
}

/*!
 ****************************************************************************
 * \brief
 *    estimate bit cost for each CBP bit
 ****************************************************************************
 */
Void TEncSbac::estCBFBit (estBitsSbacStruct* pcEstBitsSbac, UInt uiCTXIdx, TextType eTType)
{
  Int ctx;
#if !BUGFIX85TMP
  Short cbp_bit;
#endif
  
  for ( ctx = 0; ctx <= 3; ctx++ )
  {
#if BUGFIX85TMP
    pcEstBitsSbac->blockCbpBits[ctx][0] = entropyBits[64];
    pcEstBitsSbac->blockCbpBits[ctx][1] = entropyBits[64];
#else
    cbp_bit = 0;
    pcEstBitsSbac->blockCbpBits[ctx][cbp_bit] = biari_no_bits (cbp_bit, m_cCUCbfSCModel.get( uiCTXIdx, eTType, ctx ));

    cbp_bit = 1;
    pcEstBitsSbac->blockCbpBits[ctx][cbp_bit] = biari_no_bits (cbp_bit, m_cCUCbfSCModel.get( uiCTXIdx, eTType, ctx ));
#endif
  }
}


/*!
 ****************************************************************************
 * \brief
 *    estimate SAMBAC bit cost for significant coefficient map
 ****************************************************************************
 */
Void TEncSbac::estSignificantMapBit (estBitsSbacStruct* pcEstBitsSbac, UInt uiCTXIdx, TextType eTType)
{
#if HHI_TRANSFORM_CODING
  Int    k;
  UShort sig, last;
  Int    k1 = 16;

  for ( k = 0; k < k1; k++ )
  {
    sig = 0;
    pcEstBitsSbac->significantBits[k][sig] = biari_no_bits (sig, m_cCuCtxModSig.get( uiCTXIdx, eTType, k ));

    sig = 1;
    pcEstBitsSbac->significantBits[k][sig] = biari_no_bits (sig, m_cCuCtxModSig.get( uiCTXIdx, eTType, k ));

    last = 0;
    pcEstBitsSbac->lastBits[k][last] = biari_no_bits (last, m_cCuCtxModLast.get( uiCTXIdx, eTType, k ));

    last = 1;
    pcEstBitsSbac->lastBits[k][last] = biari_no_bits (last, m_cCuCtxModLast.get( uiCTXIdx, eTType, k ));
  }
#else
  Int    k;
  UShort sig, last;
  Int    k1 = 15;

  const Int* pos2ctx_map  = pos2ctx_nomap;
  const Int* pos2ctx_last = pos2ctx_nomap;


  for ( k = 0; k < k1; k++ ) // if last coeff is reached, it has to be significant
  {
    sig = 0;
    pcEstBitsSbac->significantBits[pos2ctx_map[k]][sig] = biari_no_bits (sig, m_cCUMapSCModel.get( uiCTXIdx, eTType, pos2ctx_map[ k ] ));

    sig = 1;
    pcEstBitsSbac->significantBits[pos2ctx_map[k]][sig] = biari_no_bits (sig, m_cCUMapSCModel.get( uiCTXIdx, eTType, pos2ctx_map[ k ] ));

    last = 0;
    pcEstBitsSbac->lastBits[pos2ctx_last[k]][last] = biari_no_bits (last, m_cCULastSCModel.get( uiCTXIdx, eTType, pos2ctx_last[ k ] ));

    last = 1;
    pcEstBitsSbac->lastBits[pos2ctx_last[k]][last] = biari_no_bits (last, m_cCULastSCModel.get( uiCTXIdx, eTType, pos2ctx_last[ k ] ));
  }

  // if last coeff is reached, it has to be significant
  pcEstBitsSbac->significantBits[pos2ctx_map[k1]][0] = 0;
  pcEstBitsSbac->significantBits[pos2ctx_map[k1]][1] = 0;
  pcEstBitsSbac->lastBits[pos2ctx_last[k1]][0] = 0;
  pcEstBitsSbac->lastBits[pos2ctx_last[k1]][1] = 0;
#endif
}

/*!
 ****************************************************************************
 * \brief
 *    estimate bit cost of significant coefficient
 ****************************************************************************
 */
Void TEncSbac::estSignificantCoefficientsBit (estBitsSbacStruct* pcEstBitsSbac, UInt uiCTXIdx, TextType eTType)
{
#if HHI_TRANSFORM_CODING
  Int   ctx;
  uiCTXIdx = 0;

  for ( UInt ui = 0; ui < 6; ui++ )
  {
    for ( ctx = 0; ctx <= 4; ctx++ )
    {
      pcEstBitsSbac->greaterOneBits[ui][0][ctx][0] = biari_no_bits( 0, m_cCuCtxModAbsGreOne.get( uiCTXIdx, eTType, (ui * 5) + ctx ) );
      pcEstBitsSbac->greaterOneBits[ui][0][ctx][1] = biari_no_bits( 1, m_cCuCtxModAbsGreOne.get( uiCTXIdx, eTType, (ui * 5) + ctx ) );
    }

    for ( ctx = 0; ctx <= 4; ctx++ )
    {
      pcEstBitsSbac->greaterOneBits[ui][1][ctx][0] = biari_no_bits( 0, m_cCuCtxModCoeffLevelM1.get( uiCTXIdx, eTType, (ui * 5) + ctx ) );
      pcEstBitsSbac->greaterOneBits[ui][1][ctx][1] = biari_no_bits( 1, m_cCuCtxModCoeffLevelM1.get( uiCTXIdx, eTType, (ui * 5) + ctx ) );
    }
  }
#else
  Int   ctx;
  Short greater_one;

  for ( ctx = 0; ctx <= 4; ctx++ )
  {
    greater_one = 0;
    pcEstBitsSbac->greaterOneBits[0][ctx][greater_one] = biari_no_bits (greater_one, m_cCUOneSCModel.get( uiCTXIdx, eTType, ctx ));

    greater_one = 1;
    pcEstBitsSbac->greaterOneBits[0][ctx][greater_one] = biari_no_bits (greater_one, m_cCUOneSCModel.get( uiCTXIdx, eTType, ctx ));
  }

  for ( ctx = 0; ctx <= 4; ctx++ )
  {
    pcEstBitsSbac->greaterOneBits[1][ctx][0] = biari_no_bits(0, m_cCUAbsSCModel.get( uiCTXIdx, eTType, ctx ));

    pcEstBitsSbac->greaterOneState[ctx] = biari_state(0, m_cCUAbsSCModel.get( uiCTXIdx, eTType, ctx ));

    pcEstBitsSbac->greaterOneBits[1][ctx][1] = biari_no_bits(1, m_cCUAbsSCModel.get( uiCTXIdx, eTType, ctx ));
  }
#endif
}

Int TEncSbac::biari_no_bits (Short symbol, ContextModel& rcSCModel)
{
  Int ctx_state, estBits;

  symbol = (Short) (symbol != 0);

  ctx_state = (symbol == rcSCModel.getMps() ) ? 64 + rcSCModel.getState() : 63 - rcSCModel.getState();

  estBits = entropyBits[127-ctx_state];

  return estBits;
}

Int TEncSbac::biari_state (Short symbol, ContextModel& rcSCModel)
{
  Int ctx_state;

  symbol = (Short) (symbol != 0);

  ctx_state = ( symbol == rcSCModel.getMps() ) ? 64 + rcSCModel.getState() : 63 - rcSCModel.getState();

  return ctx_state;
}

#ifdef QC_AMVRES
Void TEncSbac::xWriteMvResFlag( Int iVal,Int Ctx_idx)
{
  // send flag
  m_pcBinIf->encodeBin( iVal, m_cCUMvResCModel.get( 0, 0, Ctx_idx ) );
}

#ifdef DCM_PBIC
Bool TEncSbac::xCodeMvResFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  Bool bMvResFlag = false;

  if(pcCU->getSlice()->getSPS()->getUseAMVRes())
  {
    bMvResFlag = !(pcCU->getCUMvField( eRefList )->getMv ( uiAbsPartIdx ).isHAM());

    UInt uiAbsPartIdxL, uiAbsPartIdxA;
    TComDataCU* pcCUL   = pcCU->getPULeft ( uiAbsPartIdxL, pcCU->getZorderIdxInCU() + uiAbsPartIdx );
    TComDataCU* pcCUA   = pcCU->getPUAbove( uiAbsPartIdxA, pcCU->getZorderIdxInCU() + uiAbsPartIdx );
    TComCUMvField* pcCUMvFieldL = ( pcCUL == NULL || pcCUL->isIntra( uiAbsPartIdxL ) ) ? NULL : pcCUL->getCUMvField( eRefList );
    TComCUMvField* pcCUMvFieldA = ( pcCUA == NULL || pcCUA->isIntra( uiAbsPartIdxA ) ) ? NULL : pcCUA->getCUMvField( eRefList );
    Int iL =   ( (pcCUMvFieldL == NULL) ? 1 : (Int)(pcCUMvFieldL->getMVRes(uiAbsPartIdxL)));
    Int iV =   ( (pcCUMvFieldA == NULL) ? 1 : (Int)(pcCUMvFieldA->getMVRes(uiAbsPartIdxA)));

    xWriteMvResFlag(bMvResFlag,iL+iV);
  }

  return bMvResFlag;
}
#endif
#endif


#ifdef QC_SIFO
Void TEncSbac::encodeSwitched_Filters(TComSlice* pcSlice,TComPrediction *m_cPrediction)
{
	assert(0);
  return;
}
#endif
>>>>>>> upstream/master
