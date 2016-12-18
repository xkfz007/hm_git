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

/** \file     TDecSbac.cpp
    \brief    Context-adaptive entropy decoder class
*/

#include "TDecSbac.h"
#include "TLibCommon/TComTU.h"
#include "TLibCommon/TComTrQuant.h"

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "TLibCommon/TComCodingStatistics.h"
//
#define RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(a) , a
#else
#define RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(a)
#endif

//! \ingroup TLibDecoder
//! \{

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
#include "../TLibCommon/Debug.h"
#endif


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TDecSbac::TDecSbac()
// new structure here
: m_pcBitstream                              ( 0 )
, m_pcTDecBinIf                              ( NULL )
, m_numContextModels                         ( 0 )
, m_cCUSplitFlagSCModel                      ( 1,             1,                      NUM_SPLIT_FLAG_CTX                   , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSkipFlagSCModel                       ( 1,             1,                      NUM_SKIP_FLAG_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMergeFlagExtSCModel                   ( 1,             1,                      NUM_MERGE_FLAG_EXT_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMergeIdxExtSCModel                    ( 1,             1,                      NUM_MERGE_IDX_EXT_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUPartSizeSCModel                       ( 1,             1,                      NUM_PART_SIZE_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUPredModeSCModel                       ( 1,             1,                      NUM_PRED_MODE_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUIntraPredSCModel                      ( 1,             1,                      NUM_INTRA_PREDICT_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUChromaPredSCModel                     ( 1,             1,                      NUM_CHROMA_PRED_CTX                  , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUDeltaQpSCModel                        ( 1,             1,                      NUM_DELTA_QP_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUInterDirSCModel                       ( 1,             1,                      NUM_INTER_DIR_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCURefPicSCModel                         ( 1,             1,                      NUM_REF_NO_CTX                       , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMvdSCModel                            ( 1,             1,                      NUM_MV_RES_CTX                       , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUQtCbfSCModel                          ( 1,             NUM_QT_CBF_CTX_SETS,    NUM_QT_CBF_CTX_PER_SET               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUTransSubdivFlagSCModel                ( 1,             1,                      NUM_TRANS_SUBDIV_FLAG_CTX            , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUQtRootCbfSCModel                      ( 1,             1,                      NUM_QT_ROOT_CBF_CTX                  , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSigCoeffGroupSCModel                  ( 1,             2,                      NUM_SIG_CG_FLAG_CTX                  , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSigSCModel                            ( 1,             1,                      NUM_SIG_FLAG_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCuCtxLastX                              ( 1,             NUM_CTX_LAST_FLAG_SETS, NUM_CTX_LAST_FLAG_XY                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCuCtxLastY                              ( 1,             NUM_CTX_LAST_FLAG_SETS, NUM_CTX_LAST_FLAG_XY                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUOneSCModel                            ( 1,             1,                      NUM_ONE_FLAG_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUAbsSCModel                            ( 1,             1,                      NUM_ABS_FLAG_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cMVPIdxSCModel                           ( 1,             1,                      NUM_MVP_IDX_CTX                      , m_contextModels + m_numContextModels, m_numContextModels)
, m_cSaoMergeSCModel                         ( 1,             1,                      NUM_SAO_MERGE_FLAG_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cSaoTypeIdxSCModel                       ( 1,             1,                      NUM_SAO_TYPE_IDX_CTX                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cTransformSkipSCModel                    ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_TRANSFORMSKIP_FLAG_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_CUTransquantBypassFlagSCModel            ( 1,             1,                      NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX    , m_contextModels + m_numContextModels, m_numContextModels)
, m_explicitRdpcmFlagSCModel                 ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_EXPLICIT_RDPCM_FLAG_CTX          , m_contextModels + m_numContextModels, m_numContextModels)
, m_explicitRdpcmDirSCModel                  ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_EXPLICIT_RDPCM_DIR_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCrossComponentPredictionSCModel         ( 1,             1,                      NUM_CROSS_COMPONENT_PREDICTION_CTX   , m_contextModels + m_numContextModels, m_numContextModels)
, m_ChromaQpAdjFlagSCModel                   ( 1,             1,                      NUM_CHROMA_QP_ADJ_FLAG_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_ChromaQpAdjIdcSCModel                    ( 1,             1,                      NUM_CHROMA_QP_ADJ_IDC_CTX            , m_contextModels + m_numContextModels, m_numContextModels)
{
  assert( m_numContextModels <= MAX_NUM_CTX_MOD );
}

TDecSbac::~TDecSbac()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TDecSbac::resetEntropy(TComSlice* pSlice)
{
  SliceType sliceType  = pSlice->getSliceType();
  Int       qp         = pSlice->getSliceQp();

  if (pSlice->getPPS()->getCabacInitPresentFlag() && pSlice->getCabacInitFlag())
  {
    switch (sliceType)
    {
    case P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = B_SLICE;
      break;
    case B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = P_SLICE;
      break;
    default     :           // should not occur
      assert(0);
      break;
    }
  }

  m_cCUSplitFlagSCModel.initBuffer                ( sliceType, qp, (UChar*)INIT_SPLIT_FLAG );
  m_cCUSkipFlagSCModel.initBuffer                 ( sliceType, qp, (UChar*)INIT_SKIP_FLAG );
  m_cCUMergeFlagExtSCModel.initBuffer             ( sliceType, qp, (UChar*)INIT_MERGE_FLAG_EXT );
  m_cCUMergeIdxExtSCModel.initBuffer              ( sliceType, qp, (UChar*)INIT_MERGE_IDX_EXT );
  m_cCUPartSizeSCModel.initBuffer                 ( sliceType, qp, (UChar*)INIT_PART_SIZE );
  m_cCUPredModeSCModel.initBuffer                 ( sliceType, qp, (UChar*)INIT_PRED_MODE );
  m_cCUIntraPredSCModel.initBuffer                ( sliceType, qp, (UChar*)INIT_INTRA_PRED_MODE );
  m_cCUChromaPredSCModel.initBuffer               ( sliceType, qp, (UChar*)INIT_CHROMA_PRED_MODE );
  m_cCUInterDirSCModel.initBuffer                 ( sliceType, qp, (UChar*)INIT_INTER_DIR );
  m_cCUMvdSCModel.initBuffer                      ( sliceType, qp, (UChar*)INIT_MVD );
  m_cCURefPicSCModel.initBuffer                   ( sliceType, qp, (UChar*)INIT_REF_PIC );
  m_cCUDeltaQpSCModel.initBuffer                  ( sliceType, qp, (UChar*)INIT_DQP );
  m_cCUQtCbfSCModel.initBuffer                    ( sliceType, qp, (UChar*)INIT_QT_CBF );
  m_cCUQtRootCbfSCModel.initBuffer                ( sliceType, qp, (UChar*)INIT_QT_ROOT_CBF );
  m_cCUSigCoeffGroupSCModel.initBuffer            ( sliceType, qp, (UChar*)INIT_SIG_CG_FLAG );
  m_cCUSigSCModel.initBuffer                      ( sliceType, qp, (UChar*)INIT_SIG_FLAG );
  m_cCuCtxLastX.initBuffer                        ( sliceType, qp, (UChar*)INIT_LAST );
  m_cCuCtxLastY.initBuffer                        ( sliceType, qp, (UChar*)INIT_LAST );
  m_cCUOneSCModel.initBuffer                      ( sliceType, qp, (UChar*)INIT_ONE_FLAG );
  m_cCUAbsSCModel.initBuffer                      ( sliceType, qp, (UChar*)INIT_ABS_FLAG );
  m_cMVPIdxSCModel.initBuffer                     ( sliceType, qp, (UChar*)INIT_MVP_IDX );
  m_cSaoMergeSCModel.initBuffer                   ( sliceType, qp, (UChar*)INIT_SAO_MERGE_FLAG );
  m_cSaoTypeIdxSCModel.initBuffer                 ( sliceType, qp, (UChar*)INIT_SAO_TYPE_IDX );
  m_cCUTransSubdivFlagSCModel.initBuffer          ( sliceType, qp, (UChar*)INIT_TRANS_SUBDIV_FLAG );
  m_cTransformSkipSCModel.initBuffer              ( sliceType, qp, (UChar*)INIT_TRANSFORMSKIP_FLAG );
  m_CUTransquantBypassFlagSCModel.initBuffer      ( sliceType, qp, (UChar*)INIT_CU_TRANSQUANT_BYPASS_FLAG );
  m_explicitRdpcmFlagSCModel.initBuffer           ( sliceType, qp, (UChar*)INIT_EXPLICIT_RDPCM_FLAG);
  m_explicitRdpcmDirSCModel.initBuffer            ( sliceType, qp, (UChar*)INIT_EXPLICIT_RDPCM_DIR);
  m_cCrossComponentPredictionSCModel.initBuffer   ( sliceType, qp, (UChar*)INIT_CROSS_COMPONENT_PREDICTION );
  m_ChromaQpAdjFlagSCModel.initBuffer             ( sliceType, qp, (UChar*)INIT_CHROMA_QP_ADJ_FLAG );
  m_ChromaQpAdjIdcSCModel.initBuffer              ( sliceType, qp, (UChar*)INIT_CHROMA_QP_ADJ_IDC );

  for (UInt statisticIndex = 0; statisticIndex < RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS ; statisticIndex++)
  {
    m_golombRiceAdaptationStatistics[statisticIndex] = 0;
  }

  m_pcTDecBinIf->start();
}

Void TDecSbac::parseTerminatingBit( UInt& ruiBit )
{
  m_pcTDecBinIf->decodeBinTrm( ruiBit );
  if ( ruiBit == 1 )
  {
    m_pcTDecBinIf->finish();

#if RExt__DECODER_DEBUG_BIT_STATISTICS
    TComCodingStatistics::IncrementStatisticEP(STATS__TRAILING_BITS, m_pcBitstream->readOutTrailingBits(),0);
#else
    m_pcBitstream->readOutTrailingBits();
#endif
  }
}

Void TDecSbac::parseRemainingBytes( Bool noTrailingBytesExpected )
{
  if (noTrailingBytesExpected)
  {
    const UInt numberOfRemainingSubstreamBytes=m_pcBitstream->getNumBitsLeft();
    assert (numberOfRemainingSubstreamBytes == 0);
  }
  else
  {
    while (m_pcBitstream->getNumBitsLeft())
    {
      UInt trailingNullByte=m_pcBitstream->readByte();
      if (trailingNullByte!=0)
      {
        printf("Trailing byte should be 0, but has value %02x\n", trailingNullByte);
        assert(trailingNullByte==0);
      }
    }
  }
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadUnaryMaxSymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadUnaryMaxSymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol )
#endif
{
  if (uiMaxSymbol == 0)
  {
    ruiSymbol = 0;
    return;
  }

  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );

  if( ruiSymbol == 0 || uiMaxSymbol == 1 )
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCont;

  do
  {
    m_pcTDecBinIf->decodeBin( uiCont, pcSCModel[ iOffset ] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    uiSymbol++;
  } while( uiCont && ( uiSymbol < uiMaxSymbol - 1 ) );

  if( uiCont && ( uiSymbol == uiMaxSymbol - 1 ) )
  {
    uiSymbol++;
  }

  ruiSymbol = uiSymbol;
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadEpExGolomb( UInt& ruiSymbol, UInt uiCount, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadEpExGolomb( UInt& ruiSymbol, UInt uiCount )
#endif
{
  UInt uiSymbol = 0;
  UInt uiBit = 1;

  while( uiBit )
  {
    m_pcTDecBinIf->decodeBinEP( uiBit RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    uiSymbol += uiBit << uiCount++;
  }

  if ( --uiCount )
  {
    UInt bins;
    m_pcTDecBinIf->decodeBinsEP( bins, uiCount RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    uiSymbol += bins;
  }

  ruiSymbol = uiSymbol;
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadUnarySymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadUnarySymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset )
#endif
{
  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));

  if( !ruiSymbol )
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCont;

  do
  {
    m_pcTDecBinIf->decodeBin( uiCont, pcSCModel[ iOffset ] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));
    uiSymbol++;
  } while( uiCont );

  ruiSymbol = uiSymbol;
}


/** Parsing of coeff_abs_level_remaing
 * \param rSymbol                 reference to coeff_abs_level_remaing
 * \param rParam                  reference to parameter
 * \param useLimitedPrefixLength
 * \param maxLog2TrDynamicRange
 */
#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadCoefRemainExGolomb ( UInt &rSymbol, UInt &rParam, const Bool useLimitedPrefixLength, const Int maxLog2TrDynamicRange, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadCoefRemainExGolomb ( UInt &rSymbol, UInt &rParam, const Bool useLimitedPrefixLength, const Int maxLog2TrDynamicRange )
#endif
{
  UInt prefix   = 0;
  UInt codeWord = 0;

  if (useLimitedPrefixLength)
  {
    const UInt longestPossiblePrefix = (32 - (COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange)) + COEF_REMAIN_BIN_REDUCTION;

    do
    {
      prefix++;
      m_pcTDecBinIf->decodeBinEP( codeWord RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    } while((codeWord != 0) && (prefix < longestPossiblePrefix));
  }
  else
  {
    do
    {
      prefix++;
      m_pcTDecBinIf->decodeBinEP( codeWord RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    } while( codeWord);
  }

  codeWord  = 1 - codeWord;
  prefix -= codeWord;
  codeWord=0;

  if (prefix < COEF_REMAIN_BIN_REDUCTION )
  {
    m_pcTDecBinIf->decodeBinsEP(codeWord,rParam RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));
    rSymbol = (prefix<<rParam) + codeWord;
  }
  else if (useLimitedPrefixLength)
  {
    const UInt maximumPrefixLength = (32 - (COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange));

    const UInt prefixLength = prefix - COEF_REMAIN_BIN_REDUCTION;
    const UInt suffixLength = (prefixLength == maximumPrefixLength) ? (maxLog2TrDynamicRange - rParam) : prefixLength;

    m_pcTDecBinIf->decodeBinsEP(codeWord, (suffixLength + rParam) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));

    rSymbol = codeWord + ((((1 << prefixLength) - 1) + COEF_REMAIN_BIN_REDUCTION) << rParam);
  }
  else
  {
    m_pcTDecBinIf->decodeBinsEP(codeWord,prefix-COEF_REMAIN_BIN_REDUCTION+rParam RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));
    rSymbol = (((1<<(prefix-COEF_REMAIN_BIN_REDUCTION))+COEF_REMAIN_BIN_REDUCTION-1)<<rParam)+codeWord;
  }
}


/** Parse I_PCM information.
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 *
 * If I_PCM flag indicates that the CU is I_PCM, parse its PCM alignment bits and codes.
 */
Void TDecSbac::parseIPCMInfo ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;

  m_pcTDecBinIf->decodeBinTrm(uiSymbol);

  if (uiSymbol == 1)
  {
    Bool bIpcmFlag = true;
    const TComSPS &sps=*(pcCU->getSlice()->getSPS());

    pcCU->setPartSizeSubParts  ( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts      ( sps.getMaxCUWidth()>>uiDepth, sps.getMaxCUHeight()>>uiDepth, uiAbsPartIdx, uiDepth );
    pcCU->setTrIdxSubParts     ( 0, uiAbsPartIdx, uiDepth );
    pcCU->setIPCMFlagSubParts  ( bIpcmFlag, uiAbsPartIdx, uiDepth );

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
          UInt sample;
          m_pcTDecBinIf->xReadPCMCode(sampleBits, sample);
          pPCMSample[x] = sample;
        }
        pPCMSample += width;
      }
    }

    m_pcTDecBinIf->start();
  }
}

Void TDecSbac::parseCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_CUTransquantBypassFlagSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__TQ_BYPASS_FLAG) );
  pcCU->setCUTransquantBypassSubParts(uiSymbol ? true : false, uiAbsPartIdx, uiDepth);
}

/** parse skip flag
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parseSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( pcCU->getSlice()->isIntra() )
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCtxSkip = pcCU->getCtxSkipFlag( uiAbsPartIdx );
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUSkipFlagSCModel.get( 0, 0, uiCtxSkip ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SKIP_FLAG) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tSkipFlag" );
  DTRACE_CABAC_T( "\tuiCtxSkip: ");
  DTRACE_CABAC_V( uiCtxSkip );
  DTRACE_CABAC_T( "\tuiSymbol: ");
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\n");

  if( uiSymbol )
  {
    pcCU->setSkipFlagSubParts( true,        uiAbsPartIdx, uiDepth );
    pcCU->setPredModeSubParts( MODE_INTER,  uiAbsPartIdx, uiDepth );
    pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts( pcCU->getSlice()->getSPS()->getMaxCUWidth()>>uiDepth, pcCU->getSlice()->getSPS()->getMaxCUHeight()>>uiDepth, uiAbsPartIdx, uiDepth );
    pcCU->setMergeFlagSubParts( true , uiAbsPartIdx, 0, uiDepth );
  }
}


/** parse merge flag
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \param uiPUIdx
 * \returns Void
 */
Void TDecSbac::parseMergeFlag ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, *m_cCUMergeFlagExtSCModel.get( 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MERGE_FLAG) );
  pcCU->setMergeFlagSubParts( uiSymbol ? true : false, uiAbsPartIdx, uiPUIdx, uiDepth );

  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tMergeFlag: " );
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\tAddress: " );
  DTRACE_CABAC_V( pcCU->getCtuRsAddr() );
  DTRACE_CABAC_T( "\tuiAbsPartIdx: " );
  DTRACE_CABAC_V( uiAbsPartIdx );
  DTRACE_CABAC_T( "\n" );
}

Void TDecSbac::parseMergeIndex ( TComDataCU* pcCU, UInt& ruiMergeIndex )
{
  UInt uiUnaryIdx = 0;
  UInt uiNumCand = pcCU->getSlice()->getMaxNumMergeCand();
  if ( uiNumCand > 1 )
  {
    for( ; uiUnaryIdx < uiNumCand - 1; ++uiUnaryIdx )
    {
      UInt uiSymbol = 0;
      if ( uiUnaryIdx==0 )
      {
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMergeIdxExtSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MERGE_INDEX) );
      }
      else
      {
        m_pcTDecBinIf->decodeBinEP( uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MERGE_INDEX) );
      }
      if( uiSymbol == 0 )
      {
        break;
      }
    }
  }
  ruiMergeIndex = uiUnaryIdx;

  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseMergeIndex()" )
  DTRACE_CABAC_T( "\tuiMRGIdx= " )
  DTRACE_CABAC_V( ruiMergeIndex )
  DTRACE_CABAC_T( "\n" )
}

Void TDecSbac::parseMVPIdx      ( Int& riMVPIdx )
{
  UInt uiSymbol;
  xReadUnaryMaxSymbol(uiSymbol, m_cMVPIdxSCModel.get(0), 1, AMVP_MAX_NUM_CANDS-1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVP_IDX) );
  riMVPIdx = uiSymbol;
}

Void TDecSbac::parseSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( uiDepth == pcCU->getSlice()->getSPS()->getLog2DiffMaxMinCodingBlockSize() )
  {
    pcCU->setDepthSubParts( uiDepth, uiAbsPartIdx );
    return;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__SPLIT_FLAG, g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()>>uiDepth]+2);
#endif

  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUSplitFlagSCModel.get( 0, 0, pcCU->getCtxSplitFlag( uiAbsPartIdx, uiDepth ) ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tSplitFlag\n" )
  pcCU->setDepthSubParts( uiDepth + uiSymbol, uiAbsPartIdx );

  return;
}

/** parse partition size
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parsePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol, uiMode = 0;
  PartSize eMode;
  const UChar cuWidth =UChar(pcCU->getSlice()->getSPS()->getMaxCUWidth()>>uiDepth);
  const UChar cuHeight=UChar(pcCU->getSlice()->getSPS()->getMaxCUHeight()>>uiDepth);
  const Int log2DiffMaxMinCodingBlockSize = pcCU->getSlice()->getSPS()->getLog2DiffMaxMinCodingBlockSize();

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__PART_SIZE, g_aucConvertToBit[cuWidth]+2);
#endif

  assert ( pcCU->getSlice()->getSPS()->getLog2DiffMaxMinCodingBlockSize() == log2DiffMaxMinCodingBlockSize);
  if ( pcCU->isIntra( uiAbsPartIdx ) )
  {
    uiSymbol = 1;
    if( uiDepth == log2DiffMaxMinCodingBlockSize )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 0) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
    }
    eMode = uiSymbol ? SIZE_2Nx2N : SIZE_NxN;
    UInt uiTrLevel = 0;
    UInt uiWidthInBit  = g_aucConvertToBit[pcCU->getWidth(uiAbsPartIdx)]+2;
    UInt uiTrSizeInBit = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxTrSize()]+2;
    uiTrLevel          = uiWidthInBit >= uiTrSizeInBit ? uiWidthInBit - uiTrSizeInBit : 0;
    if( eMode == SIZE_NxN )
    {
      pcCU->setTrIdxSubParts( 1+uiTrLevel, uiAbsPartIdx, uiDepth );
    }
    else
    {
      pcCU->setTrIdxSubParts( uiTrLevel, uiAbsPartIdx, uiDepth );
    }
  }
  else
  {
    UInt uiMaxNumBits = 2;

    if( uiDepth == log2DiffMaxMinCodingBlockSize && !( cuWidth == 8 && cuHeight == 8 ) )
    {
      uiMaxNumBits ++;
    }

    for ( UInt ui = 0; ui < uiMaxNumBits; ui++ )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, ui) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      if ( uiSymbol )
      {
        break;
      }
      uiMode++;
    }
    eMode = (PartSize) uiMode;
    if ( pcCU->getSlice()->getSPS()->getUseAMP() && uiDepth < log2DiffMaxMinCodingBlockSize )
    {
      if (eMode == SIZE_2NxN)
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 3 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype));
        if (uiSymbol == 0)
        {
          m_pcTDecBinIf->decodeBinEP(uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
          eMode = (uiSymbol == 0? SIZE_2NxnU : SIZE_2NxnD);
        }
      }
      else if (eMode == SIZE_Nx2N)
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 3 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
        if (uiSymbol == 0)
        {
          m_pcTDecBinIf->decodeBinEP(uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
          eMode = (uiSymbol == 0? SIZE_nLx2N : SIZE_nRx2N);
        }
      }
    }
  }
  pcCU->setPartSizeSubParts( eMode, uiAbsPartIdx, uiDepth );
  pcCU->setSizeSubParts( cuWidth, cuHeight, uiAbsPartIdx, uiDepth );
}


/** parse prediction mode
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parsePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( pcCU->getSlice()->isIntra() )
  {
    pcCU->setPredModeSubParts( MODE_INTRA, uiAbsPartIdx, uiDepth );
    return;
  }

  UInt uiSymbol;
  Int  iPredMode = MODE_INTER;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPredModeSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__PRED_MODE) );
  iPredMode += uiSymbol;
  pcCU->setPredModeSubParts( (PredMode)iPredMode, uiAbsPartIdx, uiDepth );
}


Void TDecSbac::parseIntraDirLumaAng  ( TComDataCU* pcCU, UInt absPartIdx, UInt depth )
{
  PartSize mode = pcCU->getPartitionSize( absPartIdx );
  UInt partNum = mode==SIZE_NxN?4:1;
  UInt partOffset = ( pcCU->getPic()->getNumPartitionsInCtu() >> ( pcCU->getDepth(absPartIdx) << 1 ) ) >> 2;
  UInt mpmPred[4],symbol;
  Int j,intraPredMode;
  if (mode==SIZE_NxN)
  {
    depth++;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__INTRA_DIR_ANG, g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()>>depth]+2, CHANNEL_TYPE_LUMA);
#endif
  for (j=0;j<partNum;j++)
  {
    m_pcTDecBinIf->decodeBin( symbol, m_cCUIntraPredSCModel.get( 0, 0, 0) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
    mpmPred[j] = symbol;
  }
  for (j=0;j<partNum;j++)
  {
    Int preds[NUM_MOST_PROBABLE_MODES] = {-1, -1, -1};
    pcCU->getIntraDirPredictor(absPartIdx+partOffset*j, preds, COMPONENT_Y);
    if (mpmPred[j])
    {
      m_pcTDecBinIf->decodeBinEP( symbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      if (symbol)
      {
        m_pcTDecBinIf->decodeBinEP( symbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
        symbol++;
      }
      intraPredMode = preds[symbol];
    }
    else
    {
      m_pcTDecBinIf->decodeBinsEP( symbol, 5 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      intraPredMode = symbol;

      //postponed sorting of MPMs (only in remaining branch)
      if (preds[0] > preds[1])
      {
        std::swap(preds[0], preds[1]);
      }
      if (preds[0] > preds[2])
      {
        std::swap(preds[0], preds[2]);
      }
      if (preds[1] > preds[2])
      {
        std::swap(preds[1], preds[2]);
      }
      for ( UInt i = 0; i < NUM_MOST_PROBABLE_MODES; i++ )
      {
        intraPredMode += ( intraPredMode >= preds[i] );
      }
    }
    pcCU->setIntraDirSubParts(CHANNEL_TYPE_LUMA, (UChar)intraPredMode, absPartIdx+partOffset*j, depth );
  }
}


Void TDecSbac::parseIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__INTRA_DIR_ANG, g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()>>uiDepth]+2, CHANNEL_TYPE_CHROMA);
#endif

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUChromaPredSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
  if( uiSymbol == 0 )
  {
    uiSymbol = DM_CHROMA_IDX;
  }
  else
  {
    UInt uiIPredMode;
    m_pcTDecBinIf->decodeBinsEP( uiIPredMode, 2 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
    UInt uiAllowedChromaDir[ NUM_CHROMA_MODE ];
    pcCU->getAllowedChromaDir( uiAbsPartIdx, uiAllowedChromaDir );
    uiSymbol = uiAllowedChromaDir[ uiIPredMode ];
  }

  pcCU->setIntraDirSubParts( CHANNEL_TYPE_CHROMA, uiSymbol, uiAbsPartIdx, uiDepth );
}


Void TDecSbac::parseInterDir( TComDataCU* pcCU, UInt& ruiInterDir, UInt uiAbsPartIdx )
{
  UInt uiSymbol;
  const UInt uiCtx = pcCU->getCtxInterDir( uiAbsPartIdx );
  ContextModel *pCtx = m_cCUInterDirSCModel.get( 0 );

  uiSymbol = 0;
  if (pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2Nx2N || pcCU->getHeight(uiAbsPartIdx) != 8 )
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, *( pCtx + uiCtx ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__INTER_DIR) );
  }

  if( uiSymbol )
  {
    uiSymbol = 2;
  }
  else
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, *( pCtx + 4 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__INTER_DIR) );
    assert(uiSymbol == 0 || uiSymbol == 1);
  }

  uiSymbol++;
  ruiInterDir = uiSymbol;
  return;
}

Void TDecSbac::parseRefFrmIdx( TComDataCU* pcCU, Int& riRefFrmIdx, RefPicList eRefList )
{
  UInt uiSymbol;

  ContextModel *pCtx = m_cCURefPicSCModel.get( 0 );
  m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__REF_FRM_IDX) );

  if( uiSymbol )
  {
    UInt uiRefNum = pcCU->getSlice()->getNumRefIdx( eRefList ) - 2;
    pCtx++;
    UInt ui;
    for( ui = 0; ui < uiRefNum; ++ui )
    {
      if( ui == 0 )
      {
        m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__REF_FRM_IDX) );
      }
      else
      {
        m_pcTDecBinIf->decodeBinEP( uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__REF_FRM_IDX) );
      }
      if( uiSymbol == 0 )
      {
        break;
      }
    }
    uiSymbol = ui + 1;
  }
  riRefFrmIdx = uiSymbol;

  return;
}

Void TDecSbac::parseMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList )
{
  UInt uiSymbol;
  UInt uiHorAbs;
  UInt uiVerAbs;
  UInt uiHorSign = 0;
  UInt uiVerSign = 0;
  ContextModel *pCtx = m_cCUMvdSCModel.get( 0 );

  if(pcCU->getSlice()->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pcCU->getInterDir(uiAbsPartIdx)==3)
  {
    uiHorAbs=0;
    uiVerAbs=0;
  }
  else
  {
    m_pcTDecBinIf->decodeBin( uiHorAbs, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );
    m_pcTDecBinIf->decodeBin( uiVerAbs, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );

    const Bool bHorAbsGr0 = uiHorAbs != 0;
    const Bool bVerAbsGr0 = uiVerAbs != 0;
    pCtx++;

    if( bHorAbsGr0 )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );
      uiHorAbs += uiSymbol;
    }

    if( bVerAbsGr0 )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );
      uiVerAbs += uiSymbol;
    }

    if( bHorAbsGr0 )
    {
      if( 2 == uiHorAbs )
      {
        xReadEpExGolomb( uiSymbol, 1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
        uiHorAbs += uiSymbol;
      }

      m_pcTDecBinIf->decodeBinEP( uiHorSign RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
    }

    if( bVerAbsGr0 )
    {
      if( 2 == uiVerAbs )
      {
        xReadEpExGolomb( uiSymbol, 1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
        uiVerAbs += uiSymbol;
      }

      m_pcTDecBinIf->decodeBinEP( uiVerSign RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
    }

  }

  const TComMv cMv( uiHorSign ? -Int( uiHorAbs ): uiHorAbs, uiVerSign ? -Int( uiVerAbs ) : uiVerAbs );
  pcCU->getCUMvField( eRefList )->setAllMvd( cMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiDepth, uiPartIdx );
  return;
}

Void TDecSbac::parseCrossComponentPrediction( TComTU &rTu, ComponentID compID )
{
  TComDataCU *pcCU = rTu.getCU();

  if( isLuma(compID) || !pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() )
  {
    return;
  }

  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();

  if (!pcCU->isIntra(uiAbsPartIdx) || (pcCU->getIntraDir( CHANNEL_TYPE_CHROMA, uiAbsPartIdx ) == DM_CHROMA_IDX))
  {
    Char alpha  = 0;
    UInt symbol = 0;

    DTRACE_CABAC_VL( g_nSymbolCounter++ )
    DTRACE_CABAC_T("\tparseCrossComponentPrediction()")
    DTRACE_CABAC_T( "\tAddr=" )
    DTRACE_CABAC_V( compID )
    DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
    DTRACE_CABAC_V( uiAbsPartIdx )
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__CROSS_COMPONENT_PREDICTION, (g_aucConvertToBit[rTu.getRect(compID).width] + 2), compID);
#endif
    ContextModel *pCtx = m_cCrossComponentPredictionSCModel.get(0, 0) + ((compID == COMPONENT_Cr) ? (NUM_CROSS_COMPONENT_PREDICTION_CTX >> 1) : 0);
    m_pcTDecBinIf->decodeBin( symbol, pCtx[0] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );

    if(symbol != 0)
    {
      // Cross-component prediction alpha is non-zero.
      UInt sign = 0;
      m_pcTDecBinIf->decodeBin( symbol, pCtx[1] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );

      if (symbol != 0)
      {
        // alpha is 2 (symbol=1), 4(symbol=2) or 8(symbol=3).
        // Read up to two more bits
        xReadUnaryMaxSymbol( symbol, (pCtx + 2), 1, 2 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
        symbol += 1;
      }
      m_pcTDecBinIf->decodeBin( sign, pCtx[4] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );

      alpha = (sign != 0) ? -(1 << symbol) : (1 << symbol);
    }
    DTRACE_CABAC_T( "\tAlpha=" )
    DTRACE_CABAC_V( alpha )
    DTRACE_CABAC_T( "\n" )

    pcCU->setCrossComponentPredictionAlphaPartRange( alpha, compID, uiAbsPartIdx, rTu.GetAbsPartIdxNumParts( compID ) );
  }
}

Void TDecSbac::parseTransformSubdivFlag( UInt& ruiSubdivFlag, UInt uiLog2TransformBlockSize )
{
  m_pcTDecBinIf->decodeBin( ruiSubdivFlag, m_cCUTransSubdivFlagSCModel.get( 0, 0, uiLog2TransformBlockSize )
      RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(TComCodingStatisticsClassType(STATS__CABAC_BITS__TRANSFORM_SUBDIV_FLAG, 5-uiLog2TransformBlockSize))
                          );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseTransformSubdivFlag()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( ruiSubdivFlag )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiLog2TransformBlockSize )
  DTRACE_CABAC_T( "\n" )
}

Void TDecSbac::parseQtRootCbf( UInt uiAbsPartIdx, UInt& uiQtRootCbf )
{
  UInt uiSymbol;
  const UInt uiCtx = 0;
  m_pcTDecBinIf->decodeBin( uiSymbol , m_cCUQtRootCbfSCModel.get( 0, 0, uiCtx ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__QT_ROOT_CBF) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseQtRootCbf()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiSymbol )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )

  uiQtRootCbf = uiSymbol;
}

Void TDecSbac::parseDeltaQP( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  Int qp;
  UInt uiDQp;
  Int  iDQp;

  UInt uiSymbol;

  xReadUnaryMaxSymbol (uiDQp,  &m_cCUDeltaQpSCModel.get( 0, 0, 0 ), 1, CU_DQP_TU_CMAX RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__DELTA_QP_EP) );

  if( uiDQp >= CU_DQP_TU_CMAX)
  {
    xReadEpExGolomb( uiSymbol, CU_DQP_EG_k RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__DELTA_QP_EP));
    uiDQp+=uiSymbol;
  }

  if ( uiDQp > 0 )
  {
    UInt uiSign;
    Int qpBdOffsetY = pcCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
    m_pcTDecBinIf->decodeBinEP(uiSign RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__DELTA_QP_EP));
    iDQp = uiDQp;
    if(uiSign)
    {
      iDQp = -iDQp;
    }
    qp = (((Int) pcCU->getRefQP( uiAbsPartIdx ) + iDQp + 52 + 2*qpBdOffsetY )%(52+qpBdOffsetY)) - qpBdOffsetY;
  }
  else
  {
    qp = pcCU->getRefQP(uiAbsPartIdx);
  }

  pcCU->setQPSubParts(qp, uiAbsPartIdx, uiDepth);
  pcCU->setCodedQP(qp);
}

/** parse chroma qp adjustment, converting to the internal table representation.
 * \returns Void
 */
Void TDecSbac::parseChromaQpAdjustment( TComDataCU* cu, UInt absPartIdx, UInt depth )
{
  UInt symbol;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__CHROMA_QP_ADJUSTMENT, g_aucConvertToBit[cu->getSlice()->getSPS()->getMaxCUWidth()>>depth]+2, CHANNEL_TYPE_CHROMA);
#endif

  Int chromaQpOffsetListLen = cu->getSlice()->getPPS()->getPpsRangeExtension().getChromaQpOffsetListLen();

  // cu_chroma_qp_offset_flag
  m_pcTDecBinIf->decodeBin( symbol, m_ChromaQpAdjFlagSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );

  if (symbol && chromaQpOffsetListLen > 1)
  {
    // cu_chroma_qp_offset_idx
    xReadUnaryMaxSymbol( symbol,  &m_ChromaQpAdjIdcSCModel.get( 0, 0, 0 ), 0, chromaQpOffsetListLen - 1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
    symbol++;
  }
  /* NB, symbol = 0 if outer flag is not set,
   *              1 if outer flag is set and there is no inner flag
   *              1+ otherwise */
  cu->setChromaQpAdjSubParts( symbol, absPartIdx, depth );
  cu->setCodedChromaQpAdj(symbol);
}

Void TDecSbac::parseQtCbf( TComTU &rTu, const ComponentID compID, const Bool lowestLevel )
{
  TComDataCU* pcCU = rTu.getCU();

  const UInt absPartIdx       = rTu.GetAbsPartIdxTU(compID);
  const UInt TUDepth          = rTu.GetTransformDepthRel();
  const UInt uiCtx            = pcCU->getCtxQtCbf( rTu, toChannelType(compID) );
  const UInt contextSet       = toChannelType(compID);

  const UInt width            = rTu.getRect(compID).width;
  const UInt height           = rTu.getRect(compID).height;
  const Bool canQuadSplit     = (width >= (MIN_TU_SIZE * 2)) && (height >= (MIN_TU_SIZE * 2));
  const UInt coveredPartIdxes = rTu.GetAbsPartIdxNumParts(compID);

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
        UInt lowestTUCBF   = 0;

  if ((width != height) && (lowestLevel || !canQuadSplit)) //if sub-TUs are present
  {
    const UInt subTUDepth        = lowestTUDepth + 1;
    const UInt partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID) >> 1;

    UInt combinedSubTUCBF = 0;

    for (UInt subTU = 0; subTU < 2; subTU++)
    {
      UInt uiCbf = MAX_UINT;
      m_pcTDecBinIf->decodeBin(uiCbf, m_cCUQtCbfSCModel.get(0, contextSet, uiCtx) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(TComCodingStatisticsClassType(STATS__CABAC_BITS__QT_CBF, g_aucConvertToBit[rTu.getRect(compID).width]+2, compID)));

      const UInt subTUAbsPartIdx = absPartIdx + (subTU * partIdxesPerSubTU);
      pcCU->setCbfPartRange((uiCbf << subTUDepth), compID, subTUAbsPartIdx, partIdxesPerSubTU);
      combinedSubTUCBF |= uiCbf;

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

    //propagate the sub-TU CBF up to the lowest TU level
    if (combinedSubTUCBF != 0)
    {
      pcCU->bitwiseOrCbfPartRange((combinedSubTUCBF << lowestTUDepth), compID, absPartIdx, coveredPartIdxes);
      lowestTUCBF = combinedSubTUCBF;
    }
  }
  else
  {
    UInt uiCbf = MAX_UINT;
    m_pcTDecBinIf->decodeBin(uiCbf, m_cCUQtCbfSCModel.get(0, contextSet, uiCtx) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(TComCodingStatisticsClassType(STATS__CABAC_BITS__QT_CBF, g_aucConvertToBit[rTu.getRect(compID).width]+2, compID)));

    pcCU->setCbfSubParts((uiCbf << lowestTUDepth), compID, absPartIdx, rTu.GetTransformDepthTotalAdj(compID));

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

    lowestTUCBF = uiCbf;
  }

  //propagate the lowest level CBF up to the current level
  if (lowestTUCBF != 0)
  {
    for (UInt depth = TUDepth; depth < lowestTUDepth; depth++)
    {
      pcCU->bitwiseOrCbfPartRange((lowestTUCBF << depth), compID, absPartIdx, coveredPartIdxes);
    }
  }
}


Void TDecSbac::parseTransformSkipFlags (TComTU &rTu, ComponentID component)
{
  TComDataCU* pcCU=rTu.getCU();
  UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU(component);

  if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    return;
  }

  if (!TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(component), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize()))
  {
    return;
  }

  UInt useTransformSkip;

  m_pcTDecBinIf->decodeBin( useTransformSkip , m_cTransformSkipSCModel.get( 0, toChannelType(component), 0 )
      RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(TComCodingStatisticsClassType(STATS__CABAC_BITS__TRANSFORM_SKIP_FLAGS, component))
                          );

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

  pcCU->setTransformSkipPartRange( useTransformSkip, component, uiAbsPartIdx, rTu.GetAbsPartIdxNumParts(component));
}


/** Parse (X,Y) position of the last significant coefficient
 * \param uiPosLastX reference to X component of last coefficient
 * \param uiPosLastY reference to Y component of last coefficient
 * \param width  Block width
 * \param height Block height
 * \param component chroma compinent ID
 * \param uiScanIdx scan type (zig-zag, hor, ver)
 *
 * This method decodes the X and Y component within a block of the last significant coefficient.
 */
Void TDecSbac::parseLastSignificantXY( UInt& uiPosLastX, UInt& uiPosLastY, Int width, Int height, ComponentID component, UInt uiScanIdx )
{
  UInt uiLast;

  ContextModel *pCtxX = m_cCuCtxLastX.get( 0, toChannelType(component) );
  ContextModel *pCtxY = m_cCuCtxLastY.get( 0, toChannelType(component) );

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__LAST_SIG_X_Y, g_aucConvertToBit[width]+2, component);
#endif


  if ( uiScanIdx == SCAN_VER )
  {
    swap( width, height );
  }

  Int blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY;
  getLastSignificantContextParameters(component, width, height, blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY);

  //------------------

  // posX

  for( uiPosLastX = 0; uiPosLastX < g_uiGroupIdx[ width - 1 ]; uiPosLastX++ )
  {
    m_pcTDecBinIf->decodeBin( uiLast, *( pCtxX + blkSizeOffsetX + (uiPosLastX >>shiftX) ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );

    if( !uiLast )
    {
      break;
    }
  }

  // posY

  for( uiPosLastY = 0; uiPosLastY < g_uiGroupIdx[ height - 1 ]; uiPosLastY++ )
  {
    m_pcTDecBinIf->decodeBin( uiLast, *( pCtxY + blkSizeOffsetY + (uiPosLastY >>shiftY)) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );

    if( !uiLast )
    {
      break;
    }
  }

  // EP-coded part

  if ( uiPosLastX > 3 )
  {
    UInt uiTemp  = 0;
    UInt uiCount = ( uiPosLastX - 2 ) >> 1;
    for ( Int i = uiCount - 1; i >= 0; i-- )
    {
      m_pcTDecBinIf->decodeBinEP( uiLast RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      uiTemp += uiLast << i;
    }
    uiPosLastX = g_uiMinInGroup[ uiPosLastX ] + uiTemp;
  }
  if ( uiPosLastY > 3 )
  {
    UInt uiTemp  = 0;
    UInt uiCount = ( uiPosLastY - 2 ) >> 1;
    for ( Int i = uiCount - 1; i >= 0; i-- )
    {
      m_pcTDecBinIf->decodeBinEP( uiLast RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      uiTemp += uiLast << i;
    }
    uiPosLastY = g_uiMinInGroup[ uiPosLastY ] + uiTemp;
  }

  if( uiScanIdx == SCAN_VER )
  {
    swap( uiPosLastX, uiPosLastY );
  }
}

Void TDecSbac::parseCoeffNxN(  TComTU &rTu, ComponentID compID )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU(compID);
  const TComRectangle &rRect=rTu.getRect(compID);
  const UInt uiWidth=rRect.width;
  const UInt uiHeight=rRect.height;
  TCoeff* pcCoef=(pcCU->getCoeff(compID)+rTu.getCoefficientOffset(compID));
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
//  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_V( rTu.GetAbsPartIdxTU(compID) )
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
    std::cerr << "ERROR: parseCoeffNxN was passed a TU with dimensions larger than the maximum allowed size" << std::endl;
    assert(false);
    exit(1);
  }

  //--------------------------------------------------------------------------------------------------

  //set parameters

  const ChannelType  chType            = toChannelType(compID);
  const UInt         uiLog2BlockWidth  = g_aucConvertToBit[ uiWidth  ] + 2;
  const UInt         uiLog2BlockHeight = g_aucConvertToBit[ uiHeight ] + 2;
  const UInt         uiMaxNumCoeff     = uiWidth * uiHeight;
  const UInt         uiMaxNumCoeffM1   = uiMaxNumCoeff - 1;

  const ChannelType  channelType       = toChannelType(compID);
  const Bool         extendedPrecision = sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag();

  const Bool         alignCABACBeforeBypass = sps.getSpsRangeExtension().getCabacBypassAlignmentEnabledFlag();
  const Int          maxLog2TrDynamicRange  = sps.getMaxLog2TrDynamicRange(channelType);

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatisticsClassType ctype_group(STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG, uiLog2BlockWidth, compID);
  TComCodingStatisticsClassType ctype_map(STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG, uiLog2BlockWidth, compID);
  TComCodingStatisticsClassType ctype_gt1(STATS__CABAC_BITS__GT1_FLAG, uiLog2BlockWidth, compID);
  TComCodingStatisticsClassType ctype_gt2(STATS__CABAC_BITS__GT2_FLAG, uiLog2BlockWidth, compID);
#endif

  Bool beValid;
  if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    beValid = false;
    if((!pcCU->isIntra(uiAbsPartIdx)) && pcCU->isRDPCMEnabled(uiAbsPartIdx))
    {
      parseExplicitRdpcmMode(rTu, compID);
    }
  }
  else
  {
    beValid = pcCU->getSlice()->getPPS()->getSignHideFlag();
  }

  UInt absSum = 0;

  //--------------------------------------------------------------------------------------------------

  if(pcCU->getSlice()->getPPS()->getUseTransformSkip())
  {
    parseTransformSkipFlags(rTu, compID);
    //  This TU has coefficients and is transform skipped. Check whether is inter coded and if yes decode the explicit RDPCM mode
    if(pcCU->getTransformSkip(uiAbsPartIdx, compID) && (!pcCU->isIntra(uiAbsPartIdx)) && pcCU->isRDPCMEnabled(uiAbsPartIdx) )
    {
      parseExplicitRdpcmMode(rTu, compID);
      if(pcCU->getExplicitRdpcmMode(compID, uiAbsPartIdx) != RDPCM_OFF)
      {
        //  Sign data hiding is avoided for horizontal and vertical RDPCM modes
        beValid = false;
      }
    }
  }

  Int uiIntraMode = -1;
  const Bool       bIsLuma = isLuma(compID);
  Int isIntra = pcCU->isIntra(uiAbsPartIdx) ? 1 : 0;
  if ( isIntra && pcCU->isRDPCMEnabled(uiAbsPartIdx) )
  {
    const UInt partsPerMinCU = 1<<(2*(sps.getMaxTotalCUDepth() - sps.getLog2DiffMaxMinCodingBlockSize()));
    uiIntraMode = pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx );
    uiIntraMode = (uiIntraMode==DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, rTu.GetChromaFormat(), partsPerMinCU)) : uiIntraMode;
    uiIntraMode = ((rTu.GetChromaFormat() == CHROMA_422) && !bIsLuma) ? g_chroma422IntraAngleMappingTable[uiIntraMode] : uiIntraMode;

    Bool transformSkip = pcCU->getTransformSkip( uiAbsPartIdx,compID);
    Bool rdpcm_lossy = ( transformSkip /*&& isIntra*/ && ( (uiIntraMode == HOR_IDX) || (uiIntraMode == VER_IDX) ) );
    if ( rdpcm_lossy )
    {
      beValid = false;
    }
  }

  //--------------------------------------------------------------------------------------------------

  const Bool  bUseGolombRiceParameterAdaptation = sps.getSpsRangeExtension().getPersistentRiceAdaptationEnabledFlag();
        UInt &currentGolombRiceStatistic        = m_golombRiceAdaptationStatistics[rTu.getGolombRiceStatisticsIndex(compID)];

  //select scans
  TUEntropyCodingParameters codingParameters;
  getTUEntropyCodingParameters(codingParameters, rTu, compID);

  //===== decode last significant =====
  UInt uiPosLastX, uiPosLastY;
  parseLastSignificantXY( uiPosLastX, uiPosLastY, uiWidth, uiHeight, compID, codingParameters.scanType );
  UInt uiBlkPosLast      = uiPosLastX + (uiPosLastY<<uiLog2BlockWidth);
  pcCoef[ uiBlkPosLast ] = 1;

  //===== decode significance flags =====
  UInt uiScanPosLast;
  for( uiScanPosLast = 0; uiScanPosLast < uiMaxNumCoeffM1; uiScanPosLast++ )
  {
    UInt uiBlkPos = codingParameters.scan[ uiScanPosLast ];
    if( uiBlkPosLast == uiBlkPos )
    {
      break;
    }
  }

  ContextModel * const baseCoeffGroupCtx = m_cCUSigCoeffGroupSCModel.get( 0, isChroma(chType) );
  ContextModel * const baseCtx = m_cCUSigSCModel.get( 0, 0 ) + getSignificanceMapContextOffset(compID);

  const Int  iLastScanSet  = uiScanPosLast >> MLS_CG_SIZE;
  UInt c1                  = 1;
  UInt uiGoRiceParam       = 0;


  UInt uiSigCoeffGroupFlag[ MLS_GRP_NUM ];
  memset( uiSigCoeffGroupFlag, 0, sizeof(UInt) * MLS_GRP_NUM );

  Int  iScanPosSig             = (Int) uiScanPosLast;
  for( Int iSubSet = iLastScanSet; iSubSet >= 0; iSubSet-- )
  {
    Int  iSubPos   = iSubSet << MLS_CG_SIZE;
    uiGoRiceParam  = currentGolombRiceStatistic / RExt__GOLOMB_RICE_INCREMENT_DIVISOR;
    Bool updateGolombRiceStatistics = bUseGolombRiceParameterAdaptation; //leave the statistics at 0 when not using the adaptation system
    Int numNonZero = 0;

    Int lastNZPosInCG  = -1;
    Int firstNZPosInCG = 1 << MLS_CG_SIZE;

    Bool escapeDataPresentInGroup = false;

    Int pos[1 << MLS_CG_SIZE];

    if( iScanPosSig == (Int) uiScanPosLast )
    {
      lastNZPosInCG  = iScanPosSig;
      firstNZPosInCG = iScanPosSig;
      iScanPosSig--;
      pos[ numNonZero ] = uiBlkPosLast;
      numNonZero = 1;
    }

    // decode significant_coeffgroup_flag
    Int iCGBlkPos = codingParameters.scanCG[ iSubSet ];
    Int iCGPosY   = iCGBlkPos / codingParameters.widthInGroups;
    Int iCGPosX   = iCGBlkPos - (iCGPosY * codingParameters.widthInGroups);

    if( iSubSet == iLastScanSet || iSubSet == 0)
    {
      uiSigCoeffGroupFlag[ iCGBlkPos ] = 1;
    }
    else
    {
      UInt uiSigCoeffGroup;
      UInt uiCtxSig  = TComTrQuant::getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, iCGPosX, iCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups );
      m_pcTDecBinIf->decodeBin( uiSigCoeffGroup, baseCoeffGroupCtx[ uiCtxSig ] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_group) );
      uiSigCoeffGroupFlag[ iCGBlkPos ] = uiSigCoeffGroup;
    }

    // decode significant_coeff_flag
    const Int patternSigCtx = TComTrQuant::calcPatternSigCtx(uiSigCoeffGroupFlag, iCGPosX, iCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups);

    UInt uiBlkPos, uiSig, uiCtxSig;
    for( ; iScanPosSig >= iSubPos; iScanPosSig-- )
    {
      uiBlkPos  = codingParameters.scan[ iScanPosSig ];
      uiSig     = 0;

      if( uiSigCoeffGroupFlag[ iCGBlkPos ] )
      {
        if( iScanPosSig > iSubPos || iSubSet == 0  || numNonZero )
        {
          uiCtxSig  = TComTrQuant::getSigCtxInc( patternSigCtx, codingParameters, iScanPosSig, uiLog2BlockWidth, uiLog2BlockHeight, chType );
          m_pcTDecBinIf->decodeBin( uiSig, baseCtx[ uiCtxSig ] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_map) );
        }
        else
        {
          uiSig = 1;
        }
      }
      pcCoef[ uiBlkPos ] = uiSig;
      if( uiSig )
      {
        pos[ numNonZero ] = uiBlkPos;
        numNonZero ++;
        if( lastNZPosInCG == -1 )
        {
          lastNZPosInCG = iScanPosSig;
        }
        firstNZPosInCG = iScanPosSig;
      }
    }

    if( numNonZero > 0 )
    {
      Bool signHidden = ( lastNZPosInCG - firstNZPosInCG >= SBH_THRESHOLD );

      absSum = 0;

      const UInt uiCtxSet = getContextSetIndex(compID, iSubSet, (c1 == 0));
      c1 = 1;
      UInt uiBin;

      ContextModel *baseCtxMod = m_cCUOneSCModel.get( 0, 0 ) + (NUM_ONE_FLAG_CTX_PER_SET * uiCtxSet);

      Int absCoeff[1 << MLS_CG_SIZE];

      for ( Int i = 0; i < numNonZero; i++)
      {
        absCoeff[i] = 1;
      }
      Int numC1Flag = min(numNonZero, C1FLAG_NUMBER);
      Int firstC2FlagIdx = -1;

      for( Int idx = 0; idx < numC1Flag; idx++ )
      {
        m_pcTDecBinIf->decodeBin( uiBin, baseCtxMod[c1] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_gt1) );
        if( uiBin == 1 )
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
        absCoeff[ idx ] = uiBin + 1;
      }

      if (c1 == 0)
      {
        baseCtxMod = m_cCUAbsSCModel.get( 0, 0 ) + (NUM_ABS_FLAG_CTX_PER_SET * uiCtxSet);
        if ( firstC2FlagIdx != -1)
        {
          m_pcTDecBinIf->decodeBin( uiBin, baseCtxMod[0] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_gt2) );
          absCoeff[ firstC2FlagIdx ] = uiBin + 2;
          if (uiBin != 0)
          {
            escapeDataPresentInGroup = true;
          }
        }
      }

      escapeDataPresentInGroup = escapeDataPresentInGroup || (numNonZero > C1FLAG_NUMBER);

      const Bool alignGroup = escapeDataPresentInGroup && alignCABACBeforeBypass;

#if RExt__DECODER_DEBUG_BIT_STATISTICS
      TComCodingStatisticsClassType ctype_signs((alignGroup ? STATS__CABAC_BITS__ALIGNED_SIGN_BIT    : STATS__CABAC_BITS__SIGN_BIT   ), uiLog2BlockWidth, compID);
      TComCodingStatisticsClassType ctype_escs ((alignGroup ? STATS__CABAC_BITS__ALIGNED_ESCAPE_BITS : STATS__CABAC_BITS__ESCAPE_BITS), uiLog2BlockWidth, compID);
#endif

      if (alignGroup)
      {
        m_pcTDecBinIf->align();
      }

      UInt coeffSigns;
      if ( signHidden && beValid )
      {
        m_pcTDecBinIf->decodeBinsEP( coeffSigns, numNonZero-1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_signs) );
        coeffSigns <<= 32 - (numNonZero-1);
      }
      else
      {
        m_pcTDecBinIf->decodeBinsEP( coeffSigns, numNonZero RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_signs) );
        coeffSigns <<= 32 - numNonZero;
      }

      Int iFirstCoeff2 = 1;
      if (escapeDataPresentInGroup)
      {
        for( Int idx = 0; idx < numNonZero; idx++ )
        {
          UInt baseLevel  = (idx < C1FLAG_NUMBER)? (2 + iFirstCoeff2) : 1;

          if( absCoeff[ idx ] == baseLevel)
          {
            UInt uiLevel;
            xReadCoefRemainExGolomb( uiLevel, uiGoRiceParam, extendedPrecision, maxLog2TrDynamicRange RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_escs) );

            absCoeff[ idx ] = uiLevel + baseLevel;

            if (absCoeff[idx] > (3 << uiGoRiceParam))
            {
              uiGoRiceParam = bUseGolombRiceParameterAdaptation ? (uiGoRiceParam + 1) : (std::min<UInt>((uiGoRiceParam + 1), 4));
            }

            if (updateGolombRiceStatistics)
            {
              const UInt initialGolombRiceParameter = currentGolombRiceStatistic / RExt__GOLOMB_RICE_INCREMENT_DIVISOR;

              if (uiLevel >= (3 << initialGolombRiceParameter))
              {
                currentGolombRiceStatistic++;
              }
              else if (((uiLevel * 2) < (1 << initialGolombRiceParameter)) && (currentGolombRiceStatistic > 0))
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

      for( Int idx = 0; idx < numNonZero; idx++ )
      {
        Int blkPos = pos[ idx ];
        // Signs applied later.
        pcCoef[ blkPos ] = absCoeff[ idx ];
        absSum += absCoeff[ idx ];

        if ( idx == numNonZero-1 && signHidden && beValid )
        {
          // Infer sign of 1st element.
          if (absSum&0x1)
          {
            pcCoef[ blkPos ] = -pcCoef[ blkPos ];
          }
        }
        else
        {
          Int sign = static_cast<Int>( coeffSigns ) >> 31;
          pcCoef[ blkPos ] = ( pcCoef[ blkPos ] ^ sign ) - sign;
          coeffSigns <<= 1;
        }
      }
    }
  }

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  printSBACCoeffData(uiPosLastX, uiPosLastY, uiWidth, uiHeight, compID, uiAbsPartIdx, codingParameters.scanType, pcCoef);
#endif

  return;
}

Void TDecSbac::parseSaoMaxUvlc ( UInt& val, UInt maxSymbol )
{
  if (maxSymbol == 0)
  {
    val = 0;
    return;
  }

  UInt code;
  Int  i;
  m_pcTDecBinIf->decodeBinEP( code RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
  if ( code == 0 )
  {
    val = 0;
    return;
  }

  i=1;
  while (1)
  {
    m_pcTDecBinIf->decodeBinEP( code RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
    if ( code == 0 )
    {
      break;
    }
    i++;
    if (i == maxSymbol)
    {
      break;
    }
  }

  val = i;
}

Void TDecSbac::parseSaoUflc (UInt uiLength, UInt&  riVal)
{
  m_pcTDecBinIf->decodeBinsEP ( riVal, uiLength RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
}

Void TDecSbac::parseSaoMerge (UInt&  ruiVal)
{
  UInt uiCode;
  m_pcTDecBinIf->decodeBin( uiCode, m_cSaoMergeSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
  ruiVal = (Int)uiCode;
}

Void TDecSbac::parseSaoTypeIdx (UInt&  ruiVal)
{
  UInt uiCode;
  m_pcTDecBinIf->decodeBin( uiCode, m_cSaoTypeIdxSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
  if (uiCode == 0)
  {
    ruiVal = 0;
  }
  else
  {
    m_pcTDecBinIf->decodeBinEP( uiCode RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
    if (uiCode == 0)
    {
      ruiVal = 1;
    }
    else
    {
      ruiVal = 2;
    }
  }
}

Void TDecSbac::parseSaoSign(UInt& val)
{
  m_pcTDecBinIf->decodeBinEP ( val RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
}

Void TDecSbac::parseSAOBlkParam (SAOBlkParam& saoBlkParam
                                , Bool* sliceEnabled
                                , Bool leftMergeAvail
                                , Bool aboveMergeAvail
                                , const BitDepths &bitDepths
                                )
{
  UInt uiSymbol;

  Bool isLeftMerge = false;
  Bool isAboveMerge= false;

  if(leftMergeAvail)
  {
    parseSaoMerge(uiSymbol); //sao_merge_left_flag
    isLeftMerge = (uiSymbol?true:false);
  }

  if( aboveMergeAvail && !isLeftMerge)
  {
    parseSaoMerge(uiSymbol); //sao_merge_up_flag
    isAboveMerge = (uiSymbol?true:false);
  }

  if(isLeftMerge || isAboveMerge) //merge mode
  {
    for (UInt componentIndex = 0; componentIndex < MAX_NUM_COMPONENT; componentIndex++)
    {
      saoBlkParam[componentIndex].modeIdc = (sliceEnabled[componentIndex]) ? SAO_MODE_MERGE : SAO_MODE_OFF;
      saoBlkParam[componentIndex].typeIdc = (isLeftMerge)?SAO_MERGE_LEFT:SAO_MERGE_ABOVE;
    }
  }
  else //new or off mode
  {
    for(Int compId=COMPONENT_Y; compId < MAX_NUM_COMPONENT; compId++)
    {
      const ComponentID compIdx=ComponentID(compId);
      const ComponentID firstCompOfChType = getFirstComponentOfChannel(toChannelType(compIdx));
      SAOOffset& ctbParam = saoBlkParam[compIdx];
#if O0043_BEST_EFFORT_DECODING
      const Int bitDepthOrig = bitDepths.stream[toChannelType(compIdx)];
      const Int forceBitDepthAdjust = bitDepthOrig - bitDepths.recon[toChannelType(compIdx)];
#else
      const Int bitDepthOrig = bitDepths.recon[toChannelType(compIdx)];
#endif
      const Int maxOffsetQVal=TComSampleAdaptiveOffset::getMaxOffsetQVal(bitDepthOrig);
      if(!sliceEnabled[compIdx])
      {
        //off
        ctbParam.modeIdc = SAO_MODE_OFF;
        continue;
      }

      //type
      if(compIdx == firstCompOfChType)
      {
        parseSaoTypeIdx(uiSymbol); //sao_type_idx_luma or sao_type_idx_chroma

        assert(uiSymbol ==0 || uiSymbol ==1 || uiSymbol ==2);

        if(uiSymbol ==0) //OFF
        {
          ctbParam.modeIdc = SAO_MODE_OFF;
        }
        else if(uiSymbol == 1) //BO
        {
          ctbParam.modeIdc = SAO_MODE_NEW;
          ctbParam.typeIdc = SAO_TYPE_START_BO;
        }
        else //2, EO
        {
          ctbParam.modeIdc = SAO_MODE_NEW;
          ctbParam.typeIdc = SAO_TYPE_START_EO;
        }

      }
      else //Cr, follow Cb SAO type
      {
        ctbParam.modeIdc = saoBlkParam[COMPONENT_Cb].modeIdc;
        ctbParam.typeIdc = saoBlkParam[COMPONENT_Cb].typeIdc;
      }

      if(ctbParam.modeIdc == SAO_MODE_NEW)
      {
        Int offset[4];
        for(Int i=0; i< 4; i++)
        {
          parseSaoMaxUvlc(uiSymbol, maxOffsetQVal ); //sao_offset_abs
          offset[i] = (Int)uiSymbol;
        }

        if(ctbParam.typeIdc == SAO_TYPE_START_BO)
        {
          for(Int i=0; i< 4; i++)
          {
            if(offset[i] != 0)
            {
              parseSaoSign(uiSymbol); //sao_offset_sign
              if(uiSymbol)
              {
#if O0043_BEST_EFFORT_DECODING
                offset[i] >>= forceBitDepthAdjust;
#endif
                offset[i] = -offset[i];
              }
            }
          }
          parseSaoUflc(NUM_SAO_BO_CLASSES_LOG2, uiSymbol ); //sao_band_position
          ctbParam.typeAuxInfo = uiSymbol;

          for(Int i=0; i<4; i++)
          {
            ctbParam.offset[(ctbParam.typeAuxInfo+i)%MAX_NUM_SAO_CLASSES] = offset[i];
          }

        }
        else //EO
        {
          ctbParam.typeAuxInfo = 0;

          if(firstCompOfChType == compIdx)
          {
            parseSaoUflc(NUM_SAO_EO_TYPES_LOG2, uiSymbol ); //sao_eo_class_luma or sao_eo_class_chroma
            ctbParam.typeIdc += uiSymbol;
          }
          else
          {
            ctbParam.typeIdc = saoBlkParam[firstCompOfChType].typeIdc;
          }
          ctbParam.offset[SAO_CLASS_EO_FULL_VALLEY] = offset[0];
          ctbParam.offset[SAO_CLASS_EO_HALF_VALLEY] = offset[1];
          ctbParam.offset[SAO_CLASS_EO_PLAIN      ] = 0;
          ctbParam.offset[SAO_CLASS_EO_HALF_PEAK  ] = -offset[2];
          ctbParam.offset[SAO_CLASS_EO_FULL_PEAK  ] = -offset[3];
        }
      }
    }
  }
}

/**
 - Initialize our contexts from the nominated source.
 .
 \param pSrc Contexts to be copied.
 */
Void TDecSbac::xCopyContextsFrom( const TDecSbac* pSrc )
{
  memcpy(m_contextModels, pSrc->m_contextModels, m_numContextModels*sizeof(m_contextModels[0]));
  memcpy(m_golombRiceAdaptationStatistics, pSrc->m_golombRiceAdaptationStatistics, (sizeof(UInt) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS));
}

Void TDecSbac::xCopyFrom( const TDecSbac* pSrc )
{
  m_pcTDecBinIf->copyState( pSrc->m_pcTDecBinIf );
  xCopyContextsFrom( pSrc );
}

Void TDecSbac::load ( const TDecSbac* pSrc )
{
  xCopyFrom(pSrc);
}

Void TDecSbac::loadContexts ( const TDecSbac* pSrc )
{
  xCopyContextsFrom(pSrc);
}

/** Performs CABAC decoding of the explicit RDPCM mode
 * \param rTu current TU data structure
 * \param compID component identifier
 */
Void TDecSbac::parseExplicitRdpcmMode( TComTU &rTu, ComponentID compID )
{
  TComDataCU* cu = rTu.getCU();
  const UInt absPartIdx=rTu.GetAbsPartIdxTU(compID);
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt tuHeight = g_aucConvertToBit[rect.height];
  const UInt tuWidth  = g_aucConvertToBit[rect.width];
  UInt code = 0;

  assert(tuHeight == tuWidth);

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__EXPLICIT_RDPCM_BITS, g_aucConvertToBit[cu->getSlice()->getSPS()->getMaxCUWidth()>>rTu.GetTransformDepthTotal()]+2);
#endif

  m_pcTDecBinIf->decodeBin(code, m_explicitRdpcmFlagSCModel.get (0, toChannelType(compID), 0) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype));

  if(code == 0)
  {
    cu->setExplicitRdpcmModePartRange( RDPCM_OFF, compID, absPartIdx, rTu.GetAbsPartIdxNumParts(compID));
  }
  else
  {
    m_pcTDecBinIf->decodeBin(code, m_explicitRdpcmDirSCModel.get (0, toChannelType(compID), 0) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype));
    if(code == 0)
    {
      cu->setExplicitRdpcmModePartRange( RDPCM_HOR, compID, absPartIdx, rTu.GetAbsPartIdxNumParts(compID));
    }
    else
    {
      cu->setExplicitRdpcmModePartRange( RDPCM_VER, compID, absPartIdx, rTu.GetAbsPartIdxNumParts(compID));
    }
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

/** \file     TDecSbac.cpp
    \brief    Context-adaptive entropy decoder class
*/

#include "TDecSbac.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TDecSbac::TDecSbac() 
    // new structure here
  : m_cCUSplitFlagSCModel     ( 1,             1,               NUM_SPLIT_FLAG_CTX            )
  , m_cCUSkipFlagSCModel      ( 1,             1,               NUM_SKIP_FLAG_CTX             )
  , m_cCUAlfCtrlFlagSCModel   ( 1,             1,               NUM_ALF_CTRL_FLAG_CTX         )
#if HHI_MRG
  , m_cCUMergeFlagSCModel     ( 1,             1,               NUM_MERGE_FLAG_CTX            )
  , m_cCUMergeIndexSCModel    ( 1,             1,               NUM_MERGE_INDEX_CTX           )
#endif
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
  , m_cCUMvResCModel       ( 1,             1,               NUM_MV_RES_FALG_CTX  )
#endif
#if HHI_RQT
  , m_cCUTransSubdivFlagSCModel( 1,            1,               NUM_TRANS_SUBDIV_FLAG_CTX     )
#if HHI_RQT_ROOT
  , m_cCUQtRootCbfSCModel     ( 1,             1,               NUM_QT_ROOT_CBF_CTX           )
#endif
#endif
  , m_cCUTransIdxSCModel      ( 1,             1,               NUM_TRANS_IDX_CTX             )
  , m_cCUDeltaQpSCModel       ( 1,             1,               NUM_DELTA_QP_CTX              )
  , m_cCUCbfSCModel           ( 1,             2,               NUM_CBF_CTX                   )
#if HHI_RQT
  , m_cCUQtCbfSCModel         ( 1,             3,               NUM_QT_CBF_CTX                )
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
  m_pcBitstream = 0;
  m_pcTDecBinIf = 0;

  m_bAlfCtrl = false;
  m_uiMaxAlfCtrlDepth = 0;
}

TDecSbac::~TDecSbac()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TDecSbac::resetEntropy          (TComSlice* pcSlice)
{
  Int  iQp              = pcSlice->getSliceQp();
  SliceType eSliceType  = pcSlice->getSliceType();

  m_cCUSplitFlagSCModel.initBuffer    ( eSliceType, iQp, (Short*)INIT_SPLIT_FLAG );

  m_cCUSkipFlagSCModel.initBuffer     ( eSliceType, iQp, (Short*)INIT_SKIP_FLAG );
#if HHI_MRG
  m_cCUMergeFlagSCModel.initBuffer    ( eSliceType, iQp, (Short*)INIT_MERGE_FLAG );
  m_cCUMergeIndexSCModel.initBuffer   ( eSliceType, iQp, (Short*)INIT_MERGE_INDEX );
#endif
  m_cCUAlfCtrlFlagSCModel.initBuffer  ( eSliceType, iQp, (Short*)INIT_SKIP_FLAG );
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
#ifdef QC_AMVRES
  m_cCUMvResCModel.initBuffer         (eSliceType,  iQp, (Short*)INIT_MVRES_FLAG );
#endif
#ifdef DCM_PBIC
  m_cCUIcdSCModel.initBuffer          ( eSliceType, iQp, (Short*)INIT_ICD );
#endif
  m_cCURefPicSCModel.initBuffer       ( eSliceType, iQp, (Short*)INIT_REF_PIC );

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

  m_uiLastDQpNonZero  = 0;

  // new structure
  m_uiLastQp          = iQp;

  m_pcTDecBinIf->start();
}

#if PLANAR_INTRA
// Temporary VLC function
UInt TDecSbac::xParsePlanarBins( )
{
  Bool bDone    = false;
  UInt uiZeroes = 0;
  UInt uiBit;
  UInt uiCodeword;

  while (!bDone)
  {
    m_pcTDecBinIf->decodeBinEP( uiBit );

    if ( uiBit )
    {
      m_pcTDecBinIf->decodeBinEP( uiCodeword );
      bDone = true;
    }
    else
      uiZeroes++;
  }

  return ( ( uiZeroes << 1 ) + uiCodeword );
}

Int TDecSbac::xParsePlanarDelta( TextType ttText )
{
  /* Planar quantization
  Y        qY              cW
  0-3   :  0,1,2,3         0-3
  4-15  :  4,6,8..14       4-9
  16-63 : 18,22,26..62    10-21
  64-.. : 68,76...        22-
  */
  UInt uiDeltaNegative = 0;
  Int  iDelta          = xParsePlanarBins();

  if( iDelta > 21 )
    iDelta = ( ( iDelta - 14 ) << 3 ) + 4;
  else if( iDelta > 9 )
    iDelta = ( ( iDelta - 6 ) << 2 ) + 2;
  else if( iDelta > 3 )
    iDelta = ( iDelta - 2 ) << 1;

  if( iDelta > 0 )
  {
    m_pcTDecBinIf->decodeBinEP( uiDeltaNegative );

    if( uiDeltaNegative )
      iDelta = -iDelta;
  }

  return iDelta;
}

Void TDecSbac::parsePlanarInfo( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cPlanarIntraSCModel.get( 0, 0, 0 ) );

  if ( uiSymbol )
  {
    Int iPlanarFlag   = 1;
    Int iPlanarDeltaY = xParsePlanarDelta( TEXT_LUMA );
    Int iPlanarDeltaU = 0;
    Int iPlanarDeltaV = 0;

    // Planar delta for U and V
     m_pcTDecBinIf->decodeBin( uiSymbol, m_cPlanarIntraSCModel.get( 0, 0, 1 ) );

    if ( !uiSymbol )
    {
      iPlanarDeltaU = xParsePlanarDelta( TEXT_CHROMA_U );
      iPlanarDeltaV = xParsePlanarDelta( TEXT_CHROMA_V );
    }

    pcCU->setPartSizeSubParts  ( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts      ( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );
    pcCU->setPlanarInfoSubParts( iPlanarFlag, iPlanarDeltaY, iPlanarDeltaU, iPlanarDeltaV, uiAbsPartIdx, uiDepth );
  }
}
#endif

Void TDecSbac::parseCIPflag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  Int  iCtx = pcCU->getCtxCIPFlag( uiAbsPartIdx );

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUCIPflagCCModel.get( 0, 0, iCtx ) );
  pcCU->setCIPflagSubParts( (UChar)uiSymbol, uiAbsPartIdx, uiDepth );
}

Void TDecSbac::parseROTindex  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  UInt indexROT = 0;
  Int   dictSize  = ROT_DICT;

  switch (dictSize)
  {
  case 9:
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUROTindexSCModel.get( 0, 0, 0) );
      if ( !uiSymbol )
      {
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUROTindexSCModel.get( 0, 0, 1 ) );
        indexROT  = uiSymbol;
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUROTindexSCModel.get( 0, 0, 2 ) );
        indexROT |= uiSymbol << 1;
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUROTindexSCModel.get( 0, 0, 2 ) );
        indexROT |= uiSymbol << 2;
        indexROT++;
      }
    }
    break;
  case 4:
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUROTindexSCModel.get( 0, 0, 0 ) );
      indexROT  = uiSymbol;
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUROTindexSCModel.get( 0, 0, 1 ) );
      indexROT |= uiSymbol << 1;
    }
    break;
  case 2:
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUROTindexSCModel.get( 0, 0, 0) );
      if ( !uiSymbol ) indexROT =1;
    }
    break;
  case 5:
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUROTindexSCModel.get( 0, 0, 0) );
      if ( !uiSymbol )
      {
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUROTindexSCModel.get( 0, 0, 1 ) );
        indexROT  = uiSymbol;
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUROTindexSCModel.get( 0, 0, 2 ) );
        indexROT |= uiSymbol << 1;
        indexROT++;
      }
    }
    break;
  case 1:
    {
    }
    break;
  }
  pcCU->setROTindexSubParts( indexROT, uiAbsPartIdx, uiDepth );

  return;
}


Void TDecSbac::parseTerminatingBit( UInt& ruiBit )
{
  m_pcTDecBinIf->decodeBinTrm( ruiBit );
}


Void TDecSbac::xReadUnaryMaxSymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol )
{
  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] );

  if (ruiSymbol == 0 || uiMaxSymbol == 1)
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCont;

  do
  {
    m_pcTDecBinIf->decodeBin( uiCont, pcSCModel[ iOffset ] );
    uiSymbol++;
  }
  while( uiCont && (uiSymbol < uiMaxSymbol-1) );

  if( uiCont && (uiSymbol == uiMaxSymbol-1) )
  {
    uiSymbol++;
  }

  ruiSymbol = uiSymbol;
}

Void TDecSbac::xReadMvd( Int& riMvdComp, UInt uiAbsSum, UInt uiCtx )
{
  UInt uiLocalCtx = 0;

  if( uiAbsSum >= 3)
  {
    uiLocalCtx += ( uiAbsSum > 32) ? 2 : 1;
  }

  riMvdComp = 0;

  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMvdSCModel.get( 0, uiCtx, uiLocalCtx ) );

  if (!uiSymbol)
  {
    return;
  }

  xReadExGolombMvd( uiSymbol, &m_cCUMvdSCModel.get( 0, uiCtx, 3 ), 3 );
  uiSymbol++;

  UInt uiSign;
  m_pcTDecBinIf->decodeBinEP( uiSign );

  riMvdComp = ( 0 != uiSign ) ? -(Int)uiSymbol : (Int)uiSymbol;

  return;
}
#ifdef QC_AMVRES
Void TDecSbac::xReadMvResFlag ( Int& riVal, UInt uiCtx )
{
  UInt uiSymbol = 0;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMvResCModel.get( 0, 0, uiCtx ) );
  riVal = (Int)uiSymbol;
}
#ifdef DCM_PBIC
Bool TDecSbac::xParseMvResFlag ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList )
{
  Int MVres=0;
  if( pcCU->getSlice()->getSPS()->getUseAMVRes() )
  {
    UInt uiAbsPartIdxL, uiAbsPartIdxA;
    TComDataCU* pcCUL   = pcCU->getPULeft ( uiAbsPartIdxL, pcCU->getZorderIdxInCU() + uiAbsPartIdx );
    TComDataCU* pcCUA   = pcCU->getPUAbove( uiAbsPartIdxA, pcCU->getZorderIdxInCU() + uiAbsPartIdx );
    TComCUMvField* pcCUMvFieldL = ( pcCUL == NULL || pcCUL->isIntra( uiAbsPartIdxL ) ) ? NULL : pcCUL->getCUMvField( eRefList );
    TComCUMvField* pcCUMvFieldA = ( pcCUA == NULL || pcCUA->isIntra( uiAbsPartIdxA ) ) ? NULL : pcCUA->getCUMvField( eRefList );
    Int iL =   ( (pcCUMvFieldL == NULL) ? 1 : (Int)(pcCUMvFieldL->getMVRes(uiAbsPartIdxL)));
    Int iV =   ( (pcCUMvFieldA == NULL) ? 1 : (Int)(pcCUMvFieldA->getMVRes(uiAbsPartIdxA)));
    xReadMvResFlag(MVres,iL+iV);
  }
  pcCU->getCUMvField( eRefList )->setAllMVRes( MVres>0, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
  return (MVres>0);
}
#endif
#endif


#ifdef DCM_PBIC
Void TDecSbac::xReadMvdNZ( Int& riMvdComp, UInt uiCtx )
{
  UInt uiSymbol;

  xReadExGolombMvd( uiSymbol, &m_cCUMvdSCModel.get( 0, uiCtx, 3 ), 3 );
  uiSymbol++;

  UInt uiSign;
  m_pcTDecBinIf->decodeBinEP( uiSign );

  riMvdComp = ( 0 != uiSign ) ? -(Int)uiSymbol : (Int)uiSymbol;

  return;
}

Void TDecSbac::xReadIcdNZ( Int& riIcdComp, UInt uiCtx )
{
  UInt uiSymbol;

  xReadExGolombIcd( uiSymbol, &m_cCUIcdSCModel.get( 0, uiCtx, 3 ), 3 );
  uiSymbol++;

  UInt uiSign;
  m_pcTDecBinIf->decodeBinEP( uiSign );

  riIcdComp = ( 0 != uiSign ) ? -(Int)uiSymbol : (Int)uiSymbol;

  return;
}

Void TDecSbac::xReadExGolombIcd( UInt& ruiSymbol, ContextModel* pcSCModel, UInt uiMaxBin )
{
  UInt uiSymbol;

  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] );

  if (!ruiSymbol) { return; }

  m_pcTDecBinIf->decodeBin( uiSymbol, pcSCModel[1] );

  ruiSymbol = 1;

  if (!uiSymbol)  { return; }

  pcSCModel += 2;
  UInt uiCount = 2;

  do
  {
    if( uiMaxBin == uiCount )
    {
      pcSCModel++;
    }
    m_pcTDecBinIf->decodeBin( uiSymbol, *pcSCModel );
    uiCount++;
  }
  while( uiSymbol && (uiCount != 8));

  ruiSymbol = uiCount-1;

  if( uiSymbol )
  {
    xReadEpExGolomb( uiSymbol, 3 );
    ruiSymbol += uiSymbol+1;
  }

  return;
}
#endif

Void TDecSbac::xReadExGolombMvd( UInt& ruiSymbol, ContextModel* pcSCModel, UInt uiMaxBin )
{
  UInt uiSymbol;

  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] );

  if (!ruiSymbol) { return; }

  m_pcTDecBinIf->decodeBin( uiSymbol, pcSCModel[1] );

  ruiSymbol = 1;

  if (!uiSymbol)  { return; }

  pcSCModel += 2;
  UInt uiCount = 2;

  do
  {
    if( uiMaxBin == uiCount )
    {
      pcSCModel++;
    }
    m_pcTDecBinIf->decodeBin( uiSymbol, *pcSCModel );
    uiCount++;
  }
  while( uiSymbol && (uiCount != 8));

  ruiSymbol = uiCount-1;

  if( uiSymbol )
  {
    xReadEpExGolomb( uiSymbol, 3 );
    ruiSymbol += uiSymbol+1;
  }

  return;
}

Void TDecSbac::xReadEpExGolomb( UInt& ruiSymbol, UInt uiCount )
{
  UInt uiSymbol = 0;
  UInt uiBit = 1;


  while( uiBit )
  {
    m_pcTDecBinIf->decodeBinEP( uiBit );
    uiSymbol += uiBit << uiCount++;
  }

  uiCount--;
  while( uiCount-- )
  {
    m_pcTDecBinIf->decodeBinEP( uiBit );
    uiSymbol += uiBit << uiCount;
  }

  ruiSymbol = uiSymbol;

  return;
}

Void TDecSbac::xReadUnarySymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset )
{
  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] );

  if (!ruiSymbol) { return; }

  UInt uiSymbol = 0;
  UInt uiCont;

  do
  {
    m_pcTDecBinIf->decodeBin( uiCont, pcSCModel[ iOffset ] );
    uiSymbol++;
  }
  while( uiCont );

  ruiSymbol = uiSymbol;
}

Void TDecSbac::xReadExGolombLevel( UInt& ruiSymbol, ContextModel& rcSCModel  )
{
  UInt uiSymbol;
  UInt uiCount = 0;
  do
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, rcSCModel );
    uiCount++;
  }
  while( uiSymbol && (uiCount != 13));

  ruiSymbol = uiCount-1;

  if( uiSymbol )
  {
    xReadEpExGolomb( uiSymbol, 0 );
    ruiSymbol += uiSymbol+1;
  }

  return;
}

Void TDecSbac::parseAlfCtrlDepth              ( UInt& ruiAlfCtrlDepth )
{
  UInt uiSymbol;
  xReadUnaryMaxSymbol(uiSymbol, m_cALFUvlcSCModel.get(0), 1, g_uiMaxCUDepth-1);
  ruiAlfCtrlDepth = uiSymbol;
}

#if HHI_ALF
Void TDecSbac::parseAlfCoeff( Int& riCoeff, Int iLength, Int iPos )
{
  UInt uiCode;
  Int  iSign;


  m_pcTDecBinIf->decodeBin( uiCode, m_cALFSvlcSCModel.get( 0, 0, 0 ) );

  if ( uiCode == 0 )
  {
    riCoeff = 0;
    return;
  }

  // read sign
  m_pcTDecBinIf->decodeBin( uiCode, m_cALFSvlcSCModel.get( 0, 0, 1 ) );

  if ( uiCode == 0 ) iSign =  1;
  else               iSign = -1;

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

  xReadEpExGolomb( uiCode, iM );

  riCoeff = uiCode*iSign;

}

Void TDecSbac::parseAlfDc( Int& riDc    )
{
  UInt uiCode;
  Int  iSign;


  m_pcTDecBinIf->decodeBin( uiCode, m_cALFSvlcSCModel.get( 0, 0, 0 ) );

  if ( uiCode == 0 )
  {
    riDc = 0;
    return;
  }

  // read sign
  m_pcTDecBinIf->decodeBin( uiCode, m_cALFSvlcSCModel.get( 0, 0, 1 ) );

  if ( uiCode == 0 ) iSign =  1;
  else               iSign = -1;


  xReadEpExGolomb( uiCode, 9 );

  riDc = uiCode*iSign;

}

Void TDecSbac::parseAlfCtrlFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if (!m_bAlfCtrl)
    return;

  if( uiDepth > m_uiMaxAlfCtrlDepth && !pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, m_uiMaxAlfCtrlDepth))
  {
    return;
  }

  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUAlfCtrlFlagSCModel.get( 0, 0, pcCU->getCtxAlfCtrlFlag( uiAbsPartIdx ) ) );

  if (uiDepth > m_uiMaxAlfCtrlDepth)
  {
    pcCU->setAlfCtrlFlagSubParts( uiSymbol, uiAbsPartIdx, m_uiMaxAlfCtrlDepth);
  }
  else
  {
    pcCU->setAlfCtrlFlagSubParts( uiSymbol, uiAbsPartIdx, uiDepth );
  }
}

Void TDecSbac::parseAlfQTCtrlFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if (!m_bAlfCtrl)
    return;

  if( uiDepth > m_uiMaxAlfCtrlDepth && !pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, m_uiMaxAlfCtrlDepth))
  {
    return;
  }

  UInt uiSymbol;
   m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUAlfCtrlFlagSCModel.get( 0, 0, 0 ) );
    pcCU->setAlfCtrlFlagSubParts( uiSymbol, uiAbsPartIdx, uiDepth );
  }

Void TDecSbac::parseAlfQTSplitFlag( TComDataCU* pcCU ,UInt uiAbsPartIdx, UInt uiDepth, UInt uiMaxDepth )
{
  //if( uiDepth >= uiMaxDepth )
  if( uiDepth >= g_uiMaxCUDepth-g_uiAddCUDepth ) // fix HS
   {
     pcCU->setDepthSubParts( uiDepth, uiAbsPartIdx );
     return;
   }

   UInt uiSymbol;
   m_pcTDecBinIf->decodeBin( uiSymbol, m_cALFSplitFlagSCModel.get( 0, 0, 0 ) );
   pcCU->setDepthSubParts( uiDepth + uiSymbol, uiAbsPartIdx );
   return;
}
#else
Void TDecSbac::parseAlfCtrlFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if (!m_bAlfCtrl)
    return;

  if( uiDepth > m_uiMaxAlfCtrlDepth && !pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, m_uiMaxAlfCtrlDepth))
  {
    return;
  }

  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUAlfCtrlFlagSCModel.get( 0, 0, pcCU->getCtxAlfCtrlFlag( uiAbsPartIdx ) ) );

  if (uiDepth > m_uiMaxAlfCtrlDepth)
  {
    pcCU->setAlfCtrlFlagSubParts( uiSymbol, uiAbsPartIdx, m_uiMaxAlfCtrlDepth);
  }
  else
  {
    pcCU->setAlfCtrlFlagSubParts( uiSymbol, uiAbsPartIdx, uiDepth );
  }
}
#endif

Void TDecSbac::parseSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
#if HHI_MRG && !SAMSUNG_MRG_SKIP_DIRECT
  if ( pcCU->getSlice()->getSPS()->getUseMRG() )
  {
    return;
  }
#endif

  if( pcCU->getSlice()->isIntra() )
  {
    return;
  }

  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUSkipFlagSCModel.get( 0, 0, pcCU->getCtxSkipFlag( uiAbsPartIdx ) ) );

  if( uiSymbol )
  {
    pcCU->setPredModeSubParts( MODE_SKIP,  uiAbsPartIdx, uiDepth );
    pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );

    TComMv cZeroMv(0,0);
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd    ( cZeroMv, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd    ( cZeroMv, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );

#ifdef DCM_PBIC
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      TComIc cDefaultIc;
      pcCU->getCUIcField()->setAllIcd ( cDefaultIc, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
    }
#endif

    pcCU->setTrIdxSubParts( 0, uiAbsPartIdx, uiDepth );
    pcCU->setCbfSubParts  ( 0, 0, 0, uiAbsPartIdx, uiDepth );

    if ( pcCU->getSlice()->isInterP() )
    {
      pcCU->setInterDirSubParts( 1, uiAbsPartIdx, 0, uiDepth );

      if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) > 0 )
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx(  0, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
      if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) > 0 )
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( NOT_VALID, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
    }
    else
    {
      pcCU->setInterDirSubParts( 3, uiAbsPartIdx, 0, uiDepth );

      if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) > 0 )
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx(  0, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
      if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) > 0 )
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( 0, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
    }
  }
}

#if HHI_MRG
#if HHI_MRG_PU
Void TDecSbac::parseMergeFlag ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx )
{
  UInt uiSymbol;
  UInt uiCtxIdx = pcCU->getCtxMergeFlag( uiAbsPartIdx );
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMergeFlagSCModel.get( 0, 0, uiCtxIdx ) );
  pcCU->setMergeFlagSubParts( uiSymbol ? true : false, uiAbsPartIdx, uiPUIdx, uiDepth );
}

Void TDecSbac::parseMergeIndex ( TComDataCU* pcCU, UInt& ruiMergeIndex, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  UInt uiCtxIdx = pcCU->getCtxMergeIndex( uiAbsPartIdx );
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMergeIndexSCModel.get( 0, 0, uiCtxIdx ) );
  ruiMergeIndex = uiSymbol;
}
#else
Void TDecSbac::parseMergeFlag ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMergeFlagSCModel.get( 0, 0, pcCU->getCtxMergeFlag( uiAbsPartIdx ) ) );
  pcCU->setMergeFlagSubParts( uiSymbol ? true : false, uiAbsPartIdx, uiDepth );
}

Void TDecSbac::parseMergeIndex ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMergeIndexSCModel.get( 0, 0, pcCU->getCtxMergeIndex( uiAbsPartIdx ) ) );
  pcCU->setMergeIndexSubParts( uiSymbol, uiAbsPartIdx, uiDepth );
}
#endif
#endif

Void TDecSbac::parseMVPIdx      ( TComDataCU* pcCU, Int& riMVPIdx, Int iMVPNum, UInt uiAbsPartIdx, UInt uiDepth, RefPicList eRefList )
{
  UInt uiSymbol;
  xReadUnaryMaxSymbol(uiSymbol, m_cMVPIdxSCModel.get(0), 1, iMVPNum-1);
  riMVPIdx = uiSymbol;
 }

#ifdef DCM_PBIC
Void TDecSbac::parseICPIdx      ( TComDataCU* pcCU, Int& riICPIdx, Int iICPNum, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  xReadUnaryMaxSymbol(uiSymbol, m_cICPIdxSCModel.get(0), 1, iICPNum-1);
  riICPIdx = uiSymbol;
}
#endif

Void TDecSbac::parseSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
  {
    pcCU->setDepthSubParts( uiDepth, uiAbsPartIdx );
    return;
  }

  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUSplitFlagSCModel.get( 0, 0, pcCU->getCtxSplitFlag( uiAbsPartIdx, uiDepth ) ) );
#if HHI_RQT
  DTRACE_CABAC_V( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tSplitFlag\n" )
#endif
  pcCU->setDepthSubParts( uiDepth + uiSymbol, uiAbsPartIdx );

  return;
}

Void TDecSbac::parsePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
#if HHI_MRG && !HHI_MRG_PU
  if ( pcCU->getMergeFlag( uiAbsPartIdx ) )
  {
    return;
  }
#endif

  if ( pcCU->isSkip( uiAbsPartIdx ) )
  {
    pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );
    return;
  }

  UInt uiSymbol, uiMode = 0;
  PartSize eMode;

  if ( pcCU->isIntra( uiAbsPartIdx ) )
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 0) );
    eMode = uiSymbol ? SIZE_2Nx2N : SIZE_NxN;
  }
  else
  {
#if HHI_RMP_SWITCH
    if ( !pcCU->getSlice()->getSPS()->getUseRMP() && !pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 0) );
      if( uiSymbol )
        uiMode = 0;
      else
        uiMode = 3;
    }
    else
#endif
    {
      UInt uiMaxNumBits = 3;
      for ( UInt ui = 0; ui < uiMaxNumBits; ui++ )
      {
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, ui) );
        if ( uiSymbol )
        {
          break;
        }
        uiMode++;
      }
    }
    eMode = (PartSize) uiMode;

    if (pcCU->getSlice()->isInterB() && uiMode == 3)
    {
#if HHI_DISABLE_INTER_NxN_SPLIT
      uiSymbol = 0;
      if( g_uiMaxCUWidth>>uiDepth == 8 )
#endif
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 3) );
      if (uiSymbol == 0)
      {
        pcCU->setPredModeSubParts( MODE_INTRA, uiAbsPartIdx, uiDepth );
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 4) );
        if (uiSymbol == 0)
          eMode = SIZE_2Nx2N;
      }
    }

#if HHI_RMP_SWITCH
    if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) && pcCU->getSlice()->getSPS()->getUseRMP() )
#else
    if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
#endif
    {
      if (eMode == SIZE_2NxN)
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUYPosiSCModel.get( 0, 0, 0 ));
        if (uiSymbol == 0)
        {
          m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUYPosiSCModel.get( 0, 0, 1 ));
          eMode = (uiSymbol == 0? SIZE_2NxnU : SIZE_2NxnD);
        }
      }
      else if (eMode == SIZE_Nx2N)
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUXPosiSCModel.get( 0, 0, 0 ));
        if (uiSymbol == 0)
        {
          m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUXPosiSCModel.get( 0, 0, 1 ));
          eMode = (uiSymbol == 0? SIZE_nLx2N : SIZE_nRx2N);
        }
      }
    }
#if HHI_RMP_SWITCH
    if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) && !pcCU->getSlice()->getSPS()->getUseRMP() )
    {
      if ( eMode == SIZE_2NxN )
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUYPosiSCModel.get( 0, 0, 1 ));
        eMode = (uiSymbol == 0? SIZE_2NxnU : SIZE_2NxnD);
      }
      else if ( eMode == SIZE_Nx2N )
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUXPosiSCModel.get( 0, 0, 1 ));
        eMode = (uiSymbol == 0? SIZE_nLx2N : SIZE_nRx2N);
      }
    }
#endif

  }

  pcCU->setPartSizeSubParts( eMode, uiAbsPartIdx, uiDepth );
  pcCU->setSizeSubParts( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );

  UInt uiTrLevel = 0;

  UInt uiWidthInBit  = g_aucConvertToBit[pcCU->getWidth(uiAbsPartIdx)]+2;
  UInt uiTrSizeInBit = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxTrSize()]+2;
  uiTrLevel          = uiWidthInBit >= uiTrSizeInBit ? uiWidthInBit - uiTrSizeInBit : 0;

  if( pcCU->getPredictionMode(uiAbsPartIdx) == MODE_INTRA )
  {
    if( pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_NxN )
    {
      pcCU->setTrIdxSubParts( 1+uiTrLevel, uiAbsPartIdx, uiDepth );
    }
    else
    {
      pcCU->setTrIdxSubParts( uiTrLevel, uiAbsPartIdx, uiDepth );
    }
  }
}

Void TDecSbac::parsePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
#if HHI_MRG && !HHI_MRG_PU
  if ( pcCU->getMergeFlag( uiAbsPartIdx ) )
  {
    pcCU->setPredModeSubParts( MODE_INTER, uiAbsPartIdx, uiDepth );
    return;
  }
#endif

  if( pcCU->getSlice()->isIntra() )
  {
    pcCU->setPredModeSubParts( MODE_INTRA, uiAbsPartIdx, uiDepth );
    return;
  }

  UInt uiSymbol;
  Int  iPredMode = MODE_INTER;

#if HHI_MRG && !SAMSUNG_MRG_SKIP_DIRECT
  if ( !pcCU->getSlice()->getSPS()->getUseMRG() )
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPredModeSCModel.get( 0, 0, 0 ) );
    if ( uiSymbol == 0 )
    {
      iPredMode = MODE_SKIP;
    }
  }
#else
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPredModeSCModel.get( 0, 0, 0 ) );
  if ( uiSymbol == 0 )
  {
    iPredMode = MODE_SKIP;
  }
#endif

  if ( pcCU->getSlice()->isInterB() )
  {
    pcCU->setPredModeSubParts( (PredMode)iPredMode, uiAbsPartIdx, uiDepth );
    return;
  }

  if ( iPredMode != MODE_SKIP )
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPredModeSCModel.get( 0, 0, 1 ) );
    iPredMode += uiSymbol;
  }

  pcCU->setPredModeSubParts( (PredMode)iPredMode, uiAbsPartIdx, uiDepth );
}

Void TDecSbac::parseIntraDirLuma  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  Int  uiIPredMode;

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 0) );

  if ( !uiSymbol )
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) );
    uiIPredMode  = uiSymbol;
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) );
    uiIPredMode |= uiSymbol << 1;

    // note: only 4 directions are allowed if PU size >= 16
    Int iPartWidth = pcCU->getWidth( uiAbsPartIdx );
    if ( pcCU->getPartitionSize( uiAbsPartIdx) == SIZE_NxN ) iPartWidth >>= 1;
    if ( iPartWidth <= 8 )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) );
      uiIPredMode |= uiSymbol << 2;
    }

    uiIPredMode  = pcCU->revertIntraDirLuma( uiAbsPartIdx, uiIPredMode );
  }
  else
  {
    uiIPredMode  = pcCU->revertIntraDirLuma( uiAbsPartIdx, -1 );
  }
  pcCU->setLumaIntraDirSubParts( (UChar)uiIPredMode, uiAbsPartIdx, uiDepth );

  return;
}

#if ANG_INTRA
Void TDecSbac::parseIntraDirLumaAng  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  Int  uiIPredMode;
  Int  iMostProbable = pcCU->getMostProbableIntraDirLuma( uiAbsPartIdx );

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 0) );

  if ( uiSymbol )
    uiIPredMode = iMostProbable;
  else{
#if UNIFIED_DIRECTIONAL_INTRA
    Int iIntraIdx = pcCU->getIntraSizeIdx(uiAbsPartIdx);
    if ( g_aucIntraModeBitsAng[iIntraIdx] < 6 )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) ); uiIPredMode  = uiSymbol;
      if ( g_aucIntraModeBitsAng[iIntraIdx] > 2 ) { m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) ); uiIPredMode |= uiSymbol << 1; }
      if ( g_aucIntraModeBitsAng[iIntraIdx] > 3 ) { m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) ); uiIPredMode |= uiSymbol << 2; }
      if ( g_aucIntraModeBitsAng[iIntraIdx] > 4 ) { m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) ); uiIPredMode |= uiSymbol << 3; }
    }
    else
    {
#endif
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) ); uiIPredMode  = uiSymbol;
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) ); uiIPredMode |= uiSymbol << 1;
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) ); uiIPredMode |= uiSymbol << 2;
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) ); uiIPredMode |= uiSymbol << 3;
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) ); uiIPredMode |= uiSymbol << 4;

    if (uiIPredMode == 31){ // Escape coding for the last two modes
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) );
      uiIPredMode = uiSymbol ? 32 : 31;
    }
#if UNIFIED_DIRECTIONAL_INTRA
    }
#endif

    if (uiIPredMode >= iMostProbable)
      uiIPredMode++;
  }

  pcCU->setLumaIntraDirSubParts( (UChar)uiIPredMode, uiAbsPartIdx, uiDepth );
}
#endif

Void TDecSbac::parseIntraDirLumaAdi  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  Int  uiIPredMode;
  Int iIntraIdx= pcCU->getIntraSizeIdx(uiAbsPartIdx);

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 0) );

  if ( !uiSymbol )
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) );
    uiIPredMode  = uiSymbol;
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) );
    uiIPredMode |= uiSymbol << 1;
    if (g_aucIntraModeBits[iIntraIdx]>=4)
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) );
      uiIPredMode |= uiSymbol << 2;
      if (g_aucIntraModeBits[iIntraIdx]>=5)
      {
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) );
        uiIPredMode |= uiSymbol << 3;
        if (g_aucIntraModeBits[iIntraIdx]>=6)
        {
           m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraPredSCModel.get( 0, 0, 1 ) );
           uiIPredMode |= uiSymbol << 4;
        }
      }
    }
    uiIPredMode = pcCU->revertIntraDirLumaAdi( pcCU,uiAbsPartIdx, (Int)( uiIPredMode  ) );
  }
  else
  {
    uiIPredMode  = pcCU->revertIntraDirLumaAdi( pcCU,uiAbsPartIdx, -1 );
  }
  pcCU->setLumaIntraDirSubParts( (UChar)uiIPredMode, uiAbsPartIdx, uiDepth );

  return;
}

#if HHI_AIS
Void TDecSbac::parseIntraFiltFlagLumaAdi  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;

#if ANG_INTRA
  UInt uiCtx = pcCU->angIntraEnabledPredPart( uiAbsPartIdx ) ?  pcCU->getCtxIntraFiltFlagLumaAng( uiAbsPartIdx ) : pcCU->getCtxIntraFiltFlagLuma( uiAbsPartIdx );
#else
  UInt uiCtx = pcCU->getCtxIntraFiltFlagLuma( uiAbsPartIdx );
#endif
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUIntraFiltFlagSCModel.get( 0, 0, uiCtx ) );

  pcCU->setLumaIntraFiltFlagSubParts( uiSymbol != 0, uiAbsPartIdx, uiDepth );
  return;
}
#endif

Void TDecSbac::parseIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUChromaPredSCModel.get( 0, 0, pcCU->getCtxIntraDirChroma( uiAbsPartIdx ) ) );

  if ( uiSymbol )
  {
    xReadUnaryMaxSymbol( uiSymbol, m_cCUChromaPredSCModel.get( 0, 0 ) + 3, 0, 3 );
    uiSymbol++;
  }

  pcCU->setChromIntraDirSubParts( uiSymbol, uiAbsPartIdx, uiDepth );

  return;
}

Void TDecSbac::parseInterDir( TComDataCU* pcCU, UInt& ruiInterDir, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  UInt uiCtx = pcCU->getCtxInterDir( uiAbsPartIdx );

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUInterDirSCModel.get( 0, 0, uiCtx ) );

  if ( uiSymbol )
  {
    uiSymbol = 2;
  }
  else
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUInterDirSCModel.get( 0, 0, 3 ) );
  }
  uiSymbol++;
  ruiInterDir = uiSymbol;
  return;
}

Void TDecSbac::parseRefFrmIdx( TComDataCU* pcCU, Int& riRefFrmIdx, UInt uiAbsPartIdx, UInt uiDepth, RefPicList eRefList )
{
  UInt uiSymbol;
  UInt uiCtx = pcCU->getCtxRefIdx( uiAbsPartIdx, eRefList );

  m_pcTDecBinIf->decodeBin ( uiSymbol, m_cCURefPicSCModel.get( 0, 0, uiCtx ) );
  if ( uiSymbol )
  {
    xReadUnaryMaxSymbol( uiSymbol, &m_cCURefPicSCModel.get( 0, 0, 4 ), 1, pcCU->getSlice()->getNumRefIdx( eRefList )-2 );

    uiSymbol++;
  }
  riRefFrmIdx = uiSymbol;
  return;
}
#ifdef QC_AMVRES
Void TDecSbac::parseMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList )
{  
  Int iHor, iVer;  
  UInt uiAbsPartIdxL, uiAbsPartIdxA; 
  Int iHorPred, iVerPred;
  Int MVres=0;
  TComDataCU* pcCUL   = pcCU->getPULeft ( uiAbsPartIdxL, pcCU->getZorderIdxInCU() + uiAbsPartIdx );
  TComDataCU* pcCUA   = pcCU->getPUAbove( uiAbsPartIdxA, pcCU->getZorderIdxInCU() + uiAbsPartIdx );

  TComCUMvField* pcCUMvFieldL = ( pcCUL == NULL || pcCUL->isIntra( uiAbsPartIdxL ) ) ? NULL : pcCUL->getCUMvField( eRefList );
  TComCUMvField* pcCUMvFieldA = ( pcCUA == NULL || pcCUA->isIntra( uiAbsPartIdxA ) ) ? NULL : pcCUA->getCUMvField( eRefList );

  if( pcCU->getSlice()->getSPS()->getUseAMVRes() )
  {
	  Int iL =   ( (pcCUMvFieldL == NULL) ? 1 : (Int)(pcCUMvFieldL->getMVRes(uiAbsPartIdxL)));
	  Int iV =   ( (pcCUMvFieldA == NULL) ? 1 : (Int)(pcCUMvFieldA->getMVRes(uiAbsPartIdxA)));  
	  xReadMvResFlag(MVres,iL+iV);
	  if (MVres)
	  {
		  iHorPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsHor()>>1 ) + 
					 ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsHor()>>1 );
		  iVerPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsVer()>>1 ) + 
					 ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsVer()>>1 );

		  xReadMvd( iHor, iHorPred, 0 );
		  xReadMvd( iVer, iVerPred, 1 );
		  iHor *=2;
		  iVer *=2;
	  } 
	  else
	  {
		  iHorPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsHor() ) + 
					 ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsHor() );
		  iVerPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsVer() ) + 
					 ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsVer() );
	  
		  xReadMvd( iHor, iHorPred, 0 );
		  xReadMvd( iVer, iVerPred, 1 );
	  }
	  pcCU->getCUMvField( eRefList )->setAllMVRes( MVres>0, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
  }
  else
  {
	  iHorPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsHor() ) + 
				 ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsHor() );
	  iVerPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsVer() ) + 
				 ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsVer() );
	  xReadMvd( iHor, iHorPred, 0 );
      xReadMvd( iVer, iVerPred, 1 );
	  pcCU->getCUMvField( eRefList )->setAllMVRes( 0, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
  }
  TComMv cTmpMv( 0, 0 );
  pcCU->getCUMvField( eRefList )->setAllMv( cTmpMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );

  TComMv cMv( iHor, iVer );
  pcCU->getCUMvField( eRefList )->setAllMvd( cMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );

  return;
}
#else
Void TDecSbac::parseMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList )
{
  Int iHor, iVer;
  UInt uiAbsPartIdxL, uiAbsPartIdxA;
  Int iHorPred, iVerPred;

  TComDataCU* pcCUL   = pcCU->getPULeft ( uiAbsPartIdxL, pcCU->getZorderIdxInCU() + uiAbsPartIdx );
  TComDataCU* pcCUA   = pcCU->getPUAbove( uiAbsPartIdxA, pcCU->getZorderIdxInCU() + uiAbsPartIdx );

  TComCUMvField* pcCUMvFieldL = ( pcCUL == NULL || pcCUL->isIntra( uiAbsPartIdxL ) ) ? NULL : pcCUL->getCUMvField( eRefList );
  TComCUMvField* pcCUMvFieldA = ( pcCUA == NULL || pcCUA->isIntra( uiAbsPartIdxA ) ) ? NULL : pcCUA->getCUMvField( eRefList );

  iHorPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsHor() ) +
       ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsHor() );
  iVerPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsVer() ) +
       ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsVer() );

  TComMv cTmpMv( 0, 0 );
  pcCU->getCUMvField( eRefList )->setAllMv( cTmpMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );

  xReadMvd( iHor, iHorPred, 0 );
  xReadMvd( iVer, iVerPred, 1 );

  // set mvd
  TComMv cMv( iHor, iVer );
  pcCU->getCUMvField( eRefList )->setAllMvd( cMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );

  return;
}
#endif

#ifdef DCM_PBIC
Void TDecSbac::parseMvdIcd( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList )
{
  Int iZeroPatt;
  Int aaiNZMv[2][2];
  Int aiNZIc[3];
  UInt uiZeroFlag;
  TComZeroTree* pcZTree;

#ifdef QC_AMVRES
  // Decode MV resolution flag (if necessary)
  Bool bMvResFlag[2] = {false, false};
  if ( (eRefList == REF_PIC_LIST_0) || (eRefList == REF_PIC_LIST_X) )
    bMvResFlag[REF_PIC_LIST_0] = xParseMvResFlag( pcCU, uiAbsPartIdx, uiPartIdx, uiDepth, REF_PIC_LIST_0 );
  if ( (eRefList == REF_PIC_LIST_1) || (eRefList == REF_PIC_LIST_X) )
    bMvResFlag[REF_PIC_LIST_1] = xParseMvResFlag( pcCU, uiAbsPartIdx, uiPartIdx, uiDepth, REF_PIC_LIST_1 );
#endif

  // Is any component non-zero?
  m_pcTDecBinIf->decodeBin( uiZeroFlag, *getZTreeCtx(IDX_ZEROFLAG) );
  if (uiZeroFlag)
  {
    TComMv cZeroMv;
    if (eRefList == REF_PIC_LIST_X)
    {
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cZeroMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( cZeroMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
    }
    else
      pcCU->getCUMvField(       eRefList )->setAllMvd( cZeroMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );

    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      TComIc cDefaultIc;
      pcCU->getCUIcField()->setAllIcd( cDefaultIc, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
    }
    return;
  }

  // Identify the non-zero components
  if (eRefList == REF_PIC_LIST_X)
  {
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      pcZTree   = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDICDBI);
      iZeroPatt = parseZTree( pcZTree, getZTreeCtx(IDX_ZTREE_MVDICDBI) );
    }
    else
    {
      pcZTree   = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDBI);
      iZeroPatt = parseZTree( pcZTree, getZTreeCtx(IDX_ZTREE_MVDBI) );
    }
    aaiNZMv[REF_PIC_LIST_0][0] = iZeroPatt & 1;
    aaiNZMv[REF_PIC_LIST_0][1] = iZeroPatt & 2;
    aaiNZMv[REF_PIC_LIST_1][0] = iZeroPatt & 4;
    aaiNZMv[REF_PIC_LIST_1][1] = iZeroPatt & 8;
    aiNZIc[0] = iZeroPatt & 16;
    aiNZIc[1] = iZeroPatt & 32;
    aiNZIc[2] = iZeroPatt & 64;
  }
  else
  {
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      pcZTree   = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDICDUNI);
      iZeroPatt = parseZTree( pcZTree, getZTreeCtx(IDX_ZTREE_MVDICDUNI) );
    }
    else
    {
      pcZTree   = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDUNI);
      iZeroPatt = parseZTree( pcZTree, getZTreeCtx(IDX_ZTREE_MVDUNI) );
    }
    aaiNZMv[eRefList  ][0] = iZeroPatt & 1;
    aaiNZMv[eRefList  ][1] = iZeroPatt & 2;
    aaiNZMv[eRefList^1][0] = 0;
    aaiNZMv[eRefList^1][1] = 0;
    aiNZIc[0] = iZeroPatt & 4;
    aiNZIc[1] = 0;
    aiNZIc[2] = iZeroPatt & 8;
  }

  //Decode the non-zero components
  TComMv cMvd0, cMvd1;
  Int iHor, iVer;

  iHor = iVer = 0;
  if (aaiNZMv[REF_PIC_LIST_0][0]) xReadMvdNZ(iHor, 0);
  if (aaiNZMv[REF_PIC_LIST_0][1]) xReadMvdNZ(iVer, 1);
#ifdef QC_AMVRES
  if (bMvResFlag[REF_PIC_LIST_0] == true)
  {
    iHor *= 2;
    iVer *= 2;
  }
#endif
  cMvd0.set( iHor, iVer );

  iHor = iVer = 0;
  if (aaiNZMv[REF_PIC_LIST_1][0]) xReadMvdNZ(iHor, 0);
  if (aaiNZMv[REF_PIC_LIST_1][1]) xReadMvdNZ(iVer, 1);
#ifdef QC_AMVRES
  if (bMvResFlag[REF_PIC_LIST_1] == true)
  {
    iHor *= 2;
    iVer *= 2;
  }
#endif
  cMvd1.set( iHor, iVer );

  if (eRefList == REF_PIC_LIST_X)
  {
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cMvd0, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( cMvd1, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
  }
  else if (eRefList == REF_PIC_LIST_1)
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( cMvd1, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
  else
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cMvd0, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );

  if (pcCU->getSlice()->getSPS()->getUseIC())
  {
    TComIc cIcd;
    Int iParam0, iParam1, iParam2;
    iParam0 = iParam1 = iParam2 = 0;
    if (aiNZIc[0]) xReadIcdNZ(iParam0, 0);
    if (aiNZIc[1]) xReadIcdNZ(iParam1, 1);
    if (aiNZIc[2]) xReadIcdNZ(iParam2, 2);
    cIcd.setIcParam( iParam0, iParam1, iParam2 );
    pcCU->getCUIcField()->setAllIcd( cIcd, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
  }
}

Int TDecSbac::parseZTree(TComZeroTree* pcZTree, ContextModel *pcCtxModel)
{
  Int iVal;
  Int iStack;
  Int iResult;
  Int iIsNotLeaf;
  UInt uiNodeBoth;
  UInt uiNodeLeft;
  UInt uiCtx;
  Int* piZTreeStructure = pcZTree->m_piStructure;
  Int* piLeafIdx        = pcZTree->m_piLeafIdx;

  iVal    = 1;
  iStack  = 0;
  iResult = 0;
  uiCtx   = 0;

  while (true)
  {
    STATE_1:
    iIsNotLeaf = *piZTreeStructure++;

    if (iIsNotLeaf == 0)
    {
      iResult += (1 << (*piLeafIdx));

      if (iStack == 0)
        break;

      piLeafIdx++;
      iVal = iStack & 1;
      iStack >>= 1;

      if (iVal == 0)
        goto STATE_0;
    }
    else
    {
      iStack <<= 1;

      m_pcTDecBinIf->decodeBin( uiNodeBoth, *(pcCtxModel+uiCtx) );
      if (uiNodeBoth)
        iStack++;
      else
      {
        m_pcTDecBinIf->decodeBin( uiNodeLeft, *(pcCtxModel+uiCtx+1) );
        if (uiNodeLeft == 0)
        {
          iStack++;
          uiCtx += 2;
          goto STATE_0;
        }
      }

      uiCtx += 2;
    }

    goto STATE_1;


    STATE_0:
    iIsNotLeaf = *piZTreeStructure++;

    if (iIsNotLeaf == 0)
    {
      if (iStack == 0)
        break;

      piLeafIdx++;
      iVal = iStack & 1;
      iStack >>= 1;

      if (iVal != 0)
        goto STATE_1;

    }
    else
    {
      iStack <<= 1;
      uiCtx += 2;
    }

    goto STATE_0;
  }

  return iResult;
}

ContextModel* TDecSbac::getZTreeCtx( Int iIdx )
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
    printf("Wrong index to TDecSbac::getZTreeCtx()\n");
    exit(0);
    break;
  }
}
#endif

#if HHI_RQT
Void TDecSbac::parseTransformSubdivFlag( UInt& ruiSubdivFlag, UInt uiLog2TransformBlockSize )
{
  m_pcTDecBinIf->decodeBin( ruiSubdivFlag, m_cCUTransSubdivFlagSCModel.get( 0, 0, uiLog2TransformBlockSize ) );
  DTRACE_CABAC_V( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseTransformSubdivFlag()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( ruiSubdivFlag )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiLog2TransformBlockSize )
  DTRACE_CABAC_T( "\n" )
}

#if HHI_RQT_ROOT
Void TDecSbac::parseQtRootCbf( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt& uiQtRootCbf )
{
  UInt uiSymbol;
  const UInt uiCtx = pcCU->getCtxQtRootCbf( uiAbsPartIdx );
  m_pcTDecBinIf->decodeBin( uiSymbol , m_cCUQtRootCbfSCModel.get( 0, 0, uiCtx ) );
  DTRACE_CABAC_V( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseQtRootCbf()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiSymbol )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )

  uiQtRootCbf = uiSymbol;
}
#endif
#endif

Void TDecSbac::parseTransformIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiTrLevel = 0;

  UInt uiWidthInBit  = g_aucConvertToBit[pcCU->getWidth(uiAbsPartIdx)]+2;
  UInt uiTrSizeInBit = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxTrSize()]+2;
  uiTrLevel          = uiWidthInBit >= uiTrSizeInBit ? uiWidthInBit - uiTrSizeInBit : 0;

  UInt uiMinTrDepth = pcCU->getSlice()->getSPS()->getMinTrDepth() + uiTrLevel;
  UInt uiMaxTrDepth = pcCU->getSlice()->getSPS()->getMaxTrDepth() + uiTrLevel;

  if ( uiMinTrDepth == uiMaxTrDepth )
  {
    pcCU->setTrIdxSubParts( uiMinTrDepth, uiAbsPartIdx, uiDepth );
    return;
  }

  UInt uiTrIdx;
  m_pcTDecBinIf->decodeBin( uiTrIdx, m_cCUTransIdxSCModel.get( 0, 0, pcCU->getCtxTransIdx( uiAbsPartIdx ) ) );

  if ( !uiTrIdx )
  {
    uiTrIdx = uiTrIdx + uiMinTrDepth;
    pcCU->setTrIdxSubParts( uiTrIdx, uiAbsPartIdx, uiDepth );
    return;
  }

  if (pcCU->getPartitionSize(uiAbsPartIdx) >= SIZE_2NxnU && pcCU->getPartitionSize(uiAbsPartIdx) <= SIZE_nRx2N && uiMinTrDepth == 0 && uiMaxTrDepth == 1)
  {
    uiTrIdx++;

    ///Maybe unnecessary///
    UInt      uiWidth      = pcCU->getWidth ( uiAbsPartIdx );
    while((uiWidth>>uiTrIdx) < (g_uiMaxCUWidth>>g_uiMaxCUDepth)) uiTrIdx--;
    ////////////////////////

    uiTrIdx = uiTrIdx + uiMinTrDepth;
    pcCU->setTrIdxSubParts( uiTrIdx, uiAbsPartIdx, uiDepth );
    return;
  }

  UInt uiSymbol;
  Int  iCount = 1;
  while( ++iCount <= (Int)( uiMaxTrDepth - uiMinTrDepth ) )
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUTransIdxSCModel.get( 0, 0, 3 ) );
    if ( uiSymbol == 0 )
    {
      uiTrIdx = uiTrIdx + uiMinTrDepth;
      pcCU->setTrIdxSubParts( uiTrIdx, uiAbsPartIdx, uiDepth );
      return;
    }
    uiTrIdx += uiSymbol;
  }

  uiTrIdx = uiTrIdx + uiMinTrDepth;

  pcCU->setTrIdxSubParts( uiTrIdx, uiAbsPartIdx, uiDepth );

  return;
}

Void TDecSbac::parseDeltaQP( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiDQp;
  Int  iDQp;

  m_pcTDecBinIf->decodeBin( uiDQp, m_cCUDeltaQpSCModel.get( 0, 0, 0 ) );

  if ( uiDQp == 0 )
  {
    uiDQp = pcCU->getSlice()->getSliceQp();
  }
  else
  {
    xReadUnarySymbol( uiDQp, &m_cCUDeltaQpSCModel.get( 0, 0, 2 ), 1 );
    iDQp = ( uiDQp + 2 ) / 2;

    if ( uiDQp & 1 )
    {
      iDQp = -iDQp;
    }
    uiDQp = pcCU->getSlice()->getSliceQp() + iDQp;
  }

  pcCU->setQPSubParts( uiDQp, uiAbsPartIdx, uiDepth );
}

Void TDecSbac::parseCbf( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth, UInt uiDepth )
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
  UInt uiSymbol;
  UInt uiCbf = pcCU->getCbf( uiAbsPartIdx, eType );
  UInt uiCtx = pcCU->getCtxCbf( uiAbsPartIdx, eType, uiTrDepth );

  m_pcTDecBinIf->decodeBin( uiSymbol , m_cCUCbfSCModel.get( 0, eType == TEXT_LUMA ? 0 : 1, 3 - uiCtx ) );
  pcCU->setCbfSubParts( uiCbf | ( uiSymbol << uiTrDepth ), eType, uiAbsPartIdx, uiDepth );

  if( !uiSymbol )
  {
    m_uiLastDQpNonZero = 0;
  }

  return;
}

#if HHI_RQT
Void TDecSbac::parseQtCbf( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth, UInt uiDepth )
{
  UInt uiSymbol;
  const UInt uiCtx = pcCU->getCtxQtCbf( uiAbsPartIdx, eType, uiTrDepth );
  m_pcTDecBinIf->decodeBin( uiSymbol , m_cCUQtCbfSCModel.get( 0, eType ? eType - 1: eType, uiCtx ) );

  DTRACE_CABAC_V( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseQtCbf()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiSymbol )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\tetype=" )
  DTRACE_CABAC_V( eType )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )

  pcCU->setCbfSubParts( uiSymbol << uiTrDepth, eType, uiAbsPartIdx, uiDepth );
}
#endif

Void TDecSbac::parseCoeffNxN( TComDataCU* pcCU, TCoeff* pcCoef, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight, UInt uiDepth, TextType eTType )
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
  if( uiWidth > pcCU->getSlice()->getSPS()->getMaxTrSize() )
  {
    uiWidth  = pcCU->getSlice()->getSPS()->getMaxTrSize();
    uiHeight = pcCU->getSlice()->getSPS()->getMaxTrSize();
  }

  UInt uiCTXIdx;

  switch(uiWidth)
  {
    case  2: uiCTXIdx = 6; break;
    case  4: uiCTXIdx = 5; break;
    case  8: uiCTXIdx = 4; break;
    case 16: uiCTXIdx = 3; break;
    case 32: uiCTXIdx = 2; break;
    case 64: uiCTXIdx = 1; break;
    default: uiCTXIdx = 0; break;
  }

  eTType = eTType == TEXT_LUMA ? TEXT_LUMA : ( eTType == TEXT_NONE ? TEXT_NONE : TEXT_CHROMA );

  //----- parse significance map -----
  const UInt  uiMaxNumCoeff     = uiWidth*uiHeight;
  const UInt  uiMaxNumCoeffM1   = uiMaxNumCoeff - 1;
  const UInt  uiLog2BlockSize   = g_aucConvertToBit[ uiWidth ] + 2;
  UInt        uiDownLeft        = 1;
  UInt        uiNumSigTopRight  = 0;
  UInt        uiNumSigBotLeft   = 0;
  bool        bLastReceived     = false;
#if QC_MDDT//ADAPTIVE_SCAN
  const UInt*  pucScan;
  const UInt*  pucScanX;
  const UInt*  pucScanY;
  UInt *scanStats;
  UInt uiMode;
	int indexROT;
  if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && (uiWidth == 4 || uiWidth == 8 /*|| uiWidth==16 || uiWidth==32 || uiWidth==64*/))
  {
    uiMode = pcCU->getLumaIntraDir(uiAbsPartIdx);
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

       scanStats = scanStats4x4[uiPredMode]; update4x4Count[uiPredMode]++;
    }
#if ROT_CHECK
    else if(uiWidth == 8 && indexROT == 0)
#else
    else if(uiWidth == 8)
#endif
    {
      UInt uiPredMode = ((1 << (pcCU->getIntraSizeIdx( uiAbsPartIdx ) + 1)) != uiWidth) ?  g_aucIntra9Mode[uiMode]: g_aucAngIntra9Mode[uiMode];
      pucScan = scanOrder8x8[uiPredMode]; pucScanX = scanOrder8x8X[uiPredMode]; pucScanY = scanOrder8x8Y[uiPredMode];
 
      scanStats = scanStats8x8[uiPredMode]; update8x8Count[uiPredMode]++;
    }
	/*else if(uiWidth == 16)
    {
	  scan_index = LUT16x16[indexROT][uiMode];
      pucScan = scanOrder16x16[scan_index]; pucScanX = scanOrder16x16X[scan_index]; pucScanY = scanOrder16x16Y[scan_index];
      scanStats = scanStats16x16[scan_index];
    }
    else if(uiWidth == 32)
    {
	  scan_index = LUT32x32[indexROT][uiMode];
      pucScan = scanOrder32x32[scan_index]; pucScanX = scanOrder32x32X[scan_index]; pucScanY = scanOrder32x32Y[scan_index];
      scanStats = scanStats32x32[scan_index];
    }
    else if(uiWidth == 64)
    {
	  scan_index = LUT64x64[indexROT][uiMode];
      pucScan = scanOrder64x64[scan_index]; pucScanX = scanOrder64x64X[scan_index]; pucScanY = scanOrder64x64Y[scan_index];
      scanStats = scanStats64x64[scan_index];
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
    if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && (uiWidth == 4 || uiWidth == 8 /*|| uiWidth==16 || uiWidth==32 || uiWidth==64*/))
    {
#if ROT_CHECK
      if(/*ipredmode<=8 && */indexROT == 0)
        uiBlkPos = pucScan[uiScanPos];
#else
        uiBlkPos = pucScan[uiScanPos];
#endif
    }
#endif
    UInt  uiPosY    = uiBlkPos >> uiLog2BlockSize;
    UInt  uiPosX    = uiBlkPos - ( uiPosY << uiLog2BlockSize );

    //===== code significance flag =====
    UInt  uiSig     = 0;
    UInt  uiCtxSig  = TComTrQuant::getSigCtxInc( pcCoef, uiPosX, uiPosY, uiLog2BlockSize, uiWidth, ( uiDownLeft > 0 ) );
    m_pcTDecBinIf->decodeBin( uiSig, m_cCuCtxModSig.get( uiCTXIdx, eTType, uiCtxSig ) );

    if( uiSig )
    {
#if QC_MDDT//ADAPTIVE_SCAN
#if ROT_CHECK
      if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA  && ((uiWidth == 4 && /*ipredmode<=8&&*/indexROT == 0)|| (uiWidth == 8 && /* ipredmode<=8 && */ indexROT == 0) /*|| uiWidth==16 || uiWidth==32 || uiWidth==64*/)) 
#else
     if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA  && (uiWidth == 4 || uiWidth == 8))
#endif
      {
        //scanStats[pucScan[ui]]++;
        scanStats[uiScanPos]++;
      }
#endif
      pcCoef[ uiBlkPos ] = 1;

      if( uiPosX > uiPosY )
      {
        uiNumSigTopRight++;
      }
      else if( uiPosY > uiPosX )
      {
        uiNumSigBotLeft ++;
      }

      //===== code last flag =====
      UInt  uiLast     = 0;
      UInt  uiCtxLast  = TComTrQuant::getLastCtxInc( uiPosX, uiPosY, uiLog2BlockSize );
      m_pcTDecBinIf->decodeBin( uiLast, m_cCuCtxModLast.get( uiCTXIdx, eTType, uiCtxLast ) );

      if( uiLast )
      {
        bLastReceived = true;
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
  if( !bLastReceived )
  {
    pcCoef[ uiMaxNumCoeffM1 ] = 1;
  }

  Int  c1, c2;
  UInt uiSign;
  UInt uiLevel;
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
          //uiCtxSet = ( ( uiGreOne * uiEqRangeSize ) >> 8 ) + 1;
          uiCtxSet = uiPrevAbsGreOne / uiNumOfSets + 1;
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

          uiLevel = pcCoef[ uiIndex ];

          if( uiLevel )
          {
            UInt uiCtx = min<UInt>(c1, 4);
            m_pcTDecBinIf->decodeBin( uiLevel, m_cCuCtxModAbsGreOne.get( 0, eTType, (uiCtxSet * 5) + uiCtx ) );

            if( uiLevel == 1 )
            {
              uiCtx = min<UInt>(c2, 4);
              c1    = 0;
              c2++;
              uiPrevAbsGreOne++;
              xReadExGolombLevel( uiLevel, m_cCuCtxModCoeffLevelM1.get( 0, eTType, (uiCtxSet * 5) + uiCtx ) );
              uiLevel += 2;
            }
            else if( c1 )
            {
              c1++;
              uiLevel++;
            }
            else
            {
              uiLevel++;
            }
            m_pcTDecBinIf->decodeBinEP( uiSign );
            pcCoef[ uiIndex ] = ( uiSign ? -(Int)uiLevel : (Int)uiLevel );
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

      uiLevel = pcCoef[ uiIndex ];

      if( uiLevel )
      {
        UInt uiCtx = min<UInt>(c1, 4);
        m_pcTDecBinIf->decodeBin( uiLevel, m_cCuCtxModAbsGreOne.get( 0, eTType, uiCtx ) );

        if( uiLevel == 1 )
        {
          uiCtx = min<UInt>(c2, 4);
          c1    = 0;
          c2++;
          xReadExGolombLevel( uiLevel, m_cCuCtxModCoeffLevelM1.get( 0, eTType, uiCtx ) );
          uiLevel += 2;
        }
        else if( c1 )
        {
          c1++;
          uiLevel++;
        }
        else
        {
          uiLevel++;
        }
        m_pcTDecBinIf->decodeBinEP( uiSign );
        pcCoef[ uiIndex ] = ( uiSign ? -(Int)uiLevel : (Int)uiLevel );
      }
    }
  }
  return;
#else
  UInt uiCTXIdx;

  switch(uiWidth)
  {
  case  2: uiCTXIdx = 6; break;
  case  4: uiCTXIdx = 5; break;
  case  8: uiCTXIdx = 4; break;
  case 16: uiCTXIdx = 3; break;
  case 32: uiCTXIdx = 2; break;
  case 64: uiCTXIdx = 1; break;
  default: uiCTXIdx = 0; break;
  }

  if( uiWidth > pcCU->getSlice()->getSPS()->getMaxTrSize() )
  {
    uiWidth  = pcCU->getSlice()->getSPS()->getMaxTrSize();
    uiHeight = pcCU->getSlice()->getSPS()->getMaxTrSize();
  }
  UInt uiSize   = uiWidth*uiHeight;

	// pre-compute shift from size
	UInt uiShift  = ( g_aucConvertToBit[ uiWidth ] + 2 ) + ( g_aucConvertToBit[ uiHeight ] + 2 );

  // point to coefficient
  TCoeff* piCoeff = pcCoef;
  UInt ui;
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

  UInt uiSig, uiLast, uiCtx, uiCtxOffst;
  uiLast = 0;

#if QC_MDDT == 0//ADAPTIVE_SCAN == 0
  m_pcTDecBinIf->decodeBin( uiSig, m_cCUMapSCModel.get( uiCTXIdx, eTType, pos2ctx_map[ 0 ] ) );

  piCoeff[0] = uiSig;

  if( uiSig )
    m_pcTDecBinIf->decodeBin( uiLast, m_cCULastSCModel.get( uiCTXIdx, eTType, pos2ctx_last[ 0 ] ) );
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
  case 64: pucScan = g_auiFrameScanXY[ uiConvBit ]; pucScanX = g_auiFrameScanX[uiConvBit]; pucScanY = g_auiFrameScanY[uiConvBit]; break;
  default: pucScan = g_auiFrameScanXY[ uiConvBit ]; pucScanX = g_auiFrameScanX[uiConvBit]; pucScanY = g_auiFrameScanY[uiConvBit]; break;
  }
#endif

#if QC_MDDT//ADAPTIVE_SCAN
  UInt *scanStats;
  UInt uiPredMode;
	int indexROT;

  UInt uiMode;
  if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA )
  {
    uiMode = pcCU->getLumaIntraDir(uiAbsPartIdx);
	indexROT = pcCU->getROTindex(uiAbsPartIdx);
	int scan_index;
#if ROT_CHECK
    if(uiWidth == 4 && indexROT == 0)
#else
    if(uiWidth == 4)
#endif
    {
       uiPredMode = g_aucIntra9Mode[uiMode];
       pucScan = scanOrder4x4[uiPredMode]; pucScanX = scanOrder4x4X[uiPredMode]; pucScanY = scanOrder4x4Y[uiPredMode];

       scanStats = scanStats4x4[uiPredMode]; update4x4Count[uiPredMode]++;
    }
#if ROT_CHECK
    else if(uiWidth == 8 && indexROT == 0)
#else
    else if(uiWidth == 8)
#endif
    {
      uiPredMode = ((1 << (pcCU->getIntraSizeIdx( uiAbsPartIdx ) + 1)) != uiWidth) ?  g_aucIntra9Mode[uiMode]: g_aucAngIntra9Mode[uiMode];
      pucScan = scanOrder8x8[uiPredMode]; pucScanX = scanOrder8x8X[uiPredMode]; pucScanY = scanOrder8x8Y[uiPredMode];
 
      scanStats = scanStats8x8[uiPredMode]; update8x8Count[uiPredMode]++;
    }
	else if(uiWidth == 16)
    {
	  scan_index = LUT16x16[indexROT][uiMode];
      pucScan = scanOrder16x16[scan_index]; pucScanX = scanOrder16x16X[scan_index]; pucScanY = scanOrder16x16Y[scan_index];
      scanStats = scanStats16x16[scan_index];
    }
    else if(uiWidth == 32)
    {
	  scan_index = LUT32x32[indexROT][uiMode];
      pucScan = scanOrder32x32[scan_index]; pucScanX = scanOrder32x32X[scan_index]; pucScanY = scanOrder32x32Y[scan_index];
      scanStats = scanStats32x32[scan_index];
    }
    else if(uiWidth == 64)
    {
	  scan_index = LUT64x64[indexROT][uiMode];
      pucScan = scanOrder64x64[scan_index]; pucScanX = scanOrder64x64X[scan_index]; pucScanY = scanOrder64x64Y[scan_index];
      scanStats = scanStats64x64[scan_index];
    }
    else
    {
      //printf("uiWidth = %d is not supported!\n", uiWidth);
      //exit(1);
    }
  }
#endif
	//----- parse significance map -----
  // DC is decoded in the beginning
	Int		uiScanXShift = g_aucConvertToBit[ uiWidth  >> 3 ] + 2;
	Int		uiScanYShift = g_aucConvertToBit[ uiHeight >> 3 ] + 2;
  UInt	uiXX, uiYY;

	ContextModel* pMapCCModel  = m_cCUMapSCModel.get ( uiCTXIdx, eTType );
	ContextModel* pLastCCModel = m_cCULastSCModel.get( uiCTXIdx, eTType );

#if QC_MDDT//ADAPTIVE_SCAN
	for ( ui=0, uiCtxOffst=0 ; ui<(uiSize-1) && !uiLast ; ui++, uiCtxOffst+=uiCtxSize ) // if last coeff is reached, it has to be significant
#else
    for ( ui=1, uiCtxOffst=uiCtxSize ; ui<(uiSize-1) && !uiLast ; ui++, uiCtxOffst+=uiCtxSize ) // if last coeff is reached, it has to be significant
#endif
	{
#if HHI_RQT
    if (uiCtxSize < uiSize)
#else
    if (uiCtxSize != uiSize)
#endif
    {
      uiXX  = pucScanX[ui] >> uiScanXShift;
      uiYY  = pucScanY[ui] >> uiScanYShift;
      uiCtx = g_auiAntiScan8[ ( uiYY << 3 )+uiXX];
    }
    else
		{
      uiCtx = uiCtxOffst >> uiShift;
		}

    // SBAC_SEP
#if QC_MDDT//ADAPTIVE_SCAN
#if ROT_CHECK
     if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && uiWidth == 8 && indexROT == 0)//!bRD &&  eTType == TEXT_LUMA)
#else
     if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && uiWidth == 8 )
#endif
     {
       assert(uiCtx == ui);

#ifdef QC_CTX
	   m_pcTDecBinIf->decodeBin( uiSig, pMapCCModel[ pos2ctx_map[ uiCtx ] ] );
#else
	   m_pcTDecBinIf->decodeBin( uiSig, pMapCCModel[ pos2ctx_map[ g_auiAntiScan8[pucScan[uiCtx] ]] ] );
#endif

     }
     else
#endif
		m_pcTDecBinIf->decodeBin( uiSig, pMapCCModel[ pos2ctx_map[ uiCtx ] ] );

		piCoeff[ pucScan[ui] ] = uiSig;

		if( uiSig )
		{
#if QC_MDDT//ADAPTIVE_SCAN
#if ROT_CHECK
         if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA  && ((uiWidth == 4 && indexROT == 0)|| (uiWidth == 8 && indexROT == 0) || uiWidth==16 || uiWidth==32 || uiWidth==64))
#else
          if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA )
#endif
          {
            //scanStats[pucScan[ui]]++;
            scanStats[ui]++;
          }
#endif
      uiCtx       = uiCtxOffst >> uiShift;

      // SBAC_SEP
			m_pcTDecBinIf->decodeBin( uiLast, pLastCCModel[ pos2ctx_last[ uiCtx ] ] );
			if( uiLast )
			{
				break;
			}
		}
	}

  //--- last coefficient must be significant if no last symbol was received ---
  if ( ui == uiSize - 1 )
  {
    piCoeff[ pucScan[ui] ] = 1;
  }

	ContextModel* pOnesCCModel = m_cCUOneSCModel.get( uiCTXIdx, eTType );
	ContextModel* pAbsCCModel  = m_cCUAbsSCModel.get( uiCTXIdx, eTType );

  Int   c1 = 1;
  Int   c2 = 0;

  ui++;
  while( (ui--) != 0 )
  {
    Int   iIndex  = pucScan[ui];
    UInt  uiCoeff = piCoeff[ iIndex ];

    if( uiCoeff )
    {
      uiCtx = Min (c1,4);

      // SBAC_SEP
      m_pcTDecBinIf->decodeBin( uiCoeff, pOnesCCModel[ uiCtx ] );
      if( 1 == uiCoeff )
      {
        uiCtx = Min (c2,4);
        // SBAC_SEP
				xReadExGolombLevel( uiCoeff, pAbsCCModel[ uiCtx ] );
        uiCoeff += 2;

        c1=0;
        c2++;
      }
      else if (c1)
      {
        uiCoeff++;
        c1++;
      }
      else
      {
        uiCoeff++;
      }

      UInt uiSign;
      m_pcTDecBinIf->decodeBinEP( uiSign );
      piCoeff[iIndex] = ( uiSign ? -(Int)uiCoeff : (Int)uiCoeff );
    }
  }
  return;
#endif
}

Void TDecSbac::parseAlfFlag (UInt& ruiVal)
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cALFFlagSCModel.get( 0, 0, 0 ) );

  ruiVal = uiSymbol;
}

#if TSB_ALF_HEADER
Void TDecSbac::parseAlfFlagNum( UInt& ruiVal, UInt minValue, UInt depth )
{
  UInt uiLength = 0;
  UInt maxValue = (minValue << (depth*2));
  UInt temp = maxValue - minValue;
  for(UInt i=0; i<32; i++)
  {
    if(temp&0x1)
    {
      uiLength = i+1;
    }
    temp = (temp >> 1);
  }
  ruiVal = 0;
  UInt uiBit;
  if(uiLength)
  {
    while( uiLength-- )
    {
      m_pcTDecBinIf->decodeBinEP( uiBit );
      ruiVal += uiBit << uiLength;
    }
  }
  else
  {
    ruiVal = 0;
  }
  ruiVal += minValue;
}

Void TDecSbac::parseAlfCtrlFlag( UInt &ruiAlfCtrlFlag )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUAlfCtrlFlagSCModel.get( 0, 0, 0 ) );
  ruiAlfCtrlFlag = uiSymbol;
}
#endif

Void TDecSbac::parseAlfUvlc (UInt& ruiVal)
{
  UInt uiCode;
  Int  i;

  m_pcTDecBinIf->decodeBin( uiCode, m_cALFUvlcSCModel.get( 0, 0, 0 ) );
  if ( uiCode == 0 )
  {
    ruiVal = 0;
    return;
  }

  i=1;
  while (1)
  {
    m_pcTDecBinIf->decodeBin( uiCode, m_cALFUvlcSCModel.get( 0, 0, 1 ) );
    if ( uiCode == 0 ) break;
    i++;
  }

  ruiVal = i;
}

Void TDecSbac::parseAlfSvlc (Int&  riVal)
{
  UInt uiCode;
  Int  iSign;
  Int  i;

  m_pcTDecBinIf->decodeBin( uiCode, m_cALFSvlcSCModel.get( 0, 0, 0 ) );

  if ( uiCode == 0 )
  {
    riVal = 0;
    return;
  }

  // read sign
  m_pcTDecBinIf->decodeBin( uiCode, m_cALFSvlcSCModel.get( 0, 0, 1 ) );

  if ( uiCode == 0 ) iSign =  1;
  else               iSign = -1;

  // read magnitude
  i=1;
  while (1)
  {
    m_pcTDecBinIf->decodeBin( uiCode, m_cALFSvlcSCModel.get( 0, 0, 2 ) );
    if ( uiCode == 0 ) break;
    i++;
  }

  riVal = i*iSign;
}

>>>>>>> upstream/master
