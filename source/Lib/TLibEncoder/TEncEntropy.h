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

/** \file     TEncEntropy.h
    \brief    entropy encoder class (header)
*/

#ifndef __TENCENTROPY__
#define __TENCENTROPY__

#include "TLibCommon/TComSlice.h"
#include "TLibCommon/TComDataCU.h"
#include "TLibCommon/TComBitStream.h"
#include "TLibCommon/ContextModel.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComSampleAdaptiveOffset.h"
#include "TLibCommon/TComChromaFormat.h"

class TEncSbac;
class TEncCavlc;
class SEI;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// entropy encoder pure class
class TEncEntropyIf
{
public:
  virtual Void  resetEntropy          (const TComSlice *pSlice)                = 0;
  virtual SliceType determineCabacInitIdx (const TComSlice *pSlice)                = 0;
  virtual Void  setBitstream          ( TComBitIf* p )  = 0;
  virtual Void  resetBits             ()                = 0;
  virtual UInt  getNumberOfWrittenBits()                = 0;

  virtual Void  codeVPS                 ( const TComVPS* pcVPS )                                      = 0;
  virtual Void  codeSPS                 ( const TComSPS* pcSPS )                                      = 0;
  virtual Void  codePPS                 ( const TComPPS* pcPPS )                                      = 0;
  virtual Void  codeSliceHeader         ( TComSlice* pcSlice )                                  = 0;

  virtual Void  codeTilesWPPEntryPoint  ( TComSlice* pSlice )     = 0;
  virtual Void  codeTerminatingBit      ( UInt uilsLast )                                       = 0;
  virtual Void  codeSliceFinish         ()                                                      = 0;
  virtual Void  codeMVPIdx ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList ) = 0;

public:
  virtual Void codeCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeSkipFlag      ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeMergeFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeMergeIndex    ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;

  virtual Void codePartSize      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;
  virtual Void codePredMode      ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;

  virtual Void codeIPCMInfo      ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;

  virtual Void codeTransformSubdivFlag( UInt uiSymbol, UInt uiCtx ) = 0;
  virtual Void codeQtCbf         ( TComTU &rTu, const ComponentID compID, const Bool lowestLevel ) = 0;
  virtual Void codeQtRootCbf     ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeQtCbfZero     ( TComTU &rTu, const ChannelType chType ) = 0;
  virtual Void codeQtRootCbfZero ( ) = 0;
  virtual Void codeIntraDirLumaAng( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool isMultiplePU ) = 0;

  virtual Void codeIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeInterDir      ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeRefFrmIdx     ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )      = 0;
  virtual Void codeMvd           ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )      = 0;

  virtual Void codeCrossComponentPrediction( TComTU &rTu, ComponentID compID ) = 0;

  virtual Void codeDeltaQP       ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeChromaQpAdjustment( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeCoeffNxN      ( TComTU &rTu, TCoeff* pcCoef, const ComponentID compID ) = 0;
  virtual Void codeTransformSkipFlags ( TComTU &rTu, ComponentID component ) = 0;
  virtual Void codeSAOBlkParam   (SAOBlkParam& saoBlkParam, const BitDepths &bitDepths, Bool* sliceEnabled, Bool leftMergeAvail, Bool aboveMergeAvail, Bool onlyEstMergeInfo = false)    =0;
  virtual Void estBit               (estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, ChannelType chType) = 0;

  virtual Void codeExplicitRdpcmMode ( TComTU &rTu, const ComponentID compID ) = 0;

  virtual ~TEncEntropyIf() {}
};

/// entropy encoder class
class TEncEntropy
{
public:
  Void    setEntropyCoder           ( TEncEntropyIf* e );
  Void    setBitstream              ( TComBitIf* p )          { m_pcEntropyCoderIf->setBitstream(p);  }
  Void    resetBits                 ()                        { m_pcEntropyCoderIf->resetBits();      }
  UInt    getNumberOfWrittenBits    ()                        { return m_pcEntropyCoderIf->getNumberOfWrittenBits(); }
  Void    resetEntropy              (const TComSlice *pSlice) { m_pcEntropyCoderIf->resetEntropy(pSlice);  }
  SliceType determineCabacInitIdx   (const TComSlice *pSlice) { return m_pcEntropyCoderIf->determineCabacInitIdx(pSlice); }

  Void    encodeSliceHeader         ( TComSlice* pcSlice );
  Void    encodeTilesWPPEntryPoint( TComSlice* pSlice );
  Void    encodeTerminatingBit      ( UInt uiIsLast );
  Void    encodeSliceFinish         ();
  TEncEntropyIf*      m_pcEntropyCoderIf;

public:
  Void encodeVPS               ( const TComVPS* pcVPS);
  // SPS
  Void encodeSPS               ( const TComSPS* pcSPS );
  Void encodePPS               ( const TComPPS* pcPPS );
  Void encodeSplitFlag         ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD = false );
  Void encodeCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodeSkipFlag          ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodePUWise       ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void encodeInterDirPU   ( TComDataCU* pcSubCU, UInt uiAbsPartIdx  );
  Void encodeRefFrmIdxPU  ( TComDataCU* pcSubCU, UInt uiAbsPartIdx, RefPicList eRefList );
  Void encodeMvdPU        ( TComDataCU* pcSubCU, UInt uiAbsPartIdx, RefPicList eRefList );
  Void encodeMVPIdxPU     ( TComDataCU* pcSubCU, UInt uiAbsPartIdx, RefPicList eRefList );
  Void encodeMergeFlag    ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void encodeMergeIndex   ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodePredMode          ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodePartSize          ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD = false );
  Void encodeIPCMInfo          ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodePredInfo          ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void encodeIntraDirModeLuma  ( TComDataCU* pcCU, UInt absPartIdx, Bool isMultiplePU = false );

  Void encodeIntraDirModeChroma( TComDataCU* pcCU, UInt uiAbsPartIdx );

  Void encodeTransformSubdivFlag( UInt uiSymbol, UInt uiCtx );
  Void encodeQtCbf             ( TComTU &rTu, const ComponentID compID, const Bool lowestLevel );

  Void encodeQtCbfZero         ( TComTU &rTu, const ChannelType chType );
  Void encodeQtRootCbfZero     ( );
  Void encodeQtRootCbf         ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void encodeQP                ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodeChromaQpAdjustment ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );

  Void encodeCrossComponentPrediction( TComTU &rTu, ComponentID compID );

private:
  Void xEncodeTransform        ( Bool& bCodeDQP, Bool& codeChromaQpAdj, TComTU &rTu );

public:
  Void encodeCoeff             ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool& bCodeDQP, Bool& codeChromaQpAdj );

  Void encodeCoeffNxN         ( TComTU &rTu, TCoeff* pcCoef, const ComponentID compID );

  Void estimateBit             ( estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, ChannelType chType );

  Void encodeSAOBlkParam(SAOBlkParam& saoBlkParam, const BitDepths &bitDepths, Bool* sliceEnabled, Bool leftMergeAvail, Bool aboveMergeAvail){m_pcEntropyCoderIf->codeSAOBlkParam(saoBlkParam, bitDepths, sliceEnabled, leftMergeAvail, aboveMergeAvail, false);}

  static Int countNonZeroCoeffs( TCoeff* pcCoef, UInt uiSize );

};// END CLASS DEFINITION TEncEntropy

//! \}

#endif // __TENCENTROPY__

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

/** \file     TEncEntropy.h
    \brief    entropy encoder class (header)
*/

#ifndef __TENCENTROPY__
#define __TENCENTROPY__

#include "../TLibCommon/TComSlice.h"
#include "../TLibCommon/TComDataCU.h"
#include "../TLibCommon/TComBitStream.h"
#include "../TLibCommon/ContextModel.h"
#include "../TLibCommon/TComPic.h"
#include "../TLibCommon/TComTrQuant.h"
#ifdef QC_SIFO
#include "../TLibCommon/TComPrediction.h"
#endif
class TEncSbac;
class TEncCavlc;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// entropy encoder pure class
class TEncEntropyIf
{
public:
  virtual Bool getAlfCtrl()                = 0;
  virtual UInt getMaxAlfCtrlDepth()                = 0;
  virtual Void setAlfCtrl(Bool bAlfCtrl)                = 0;
  virtual Void setMaxAlfCtrlDepth(UInt uiMaxAlfCtrlDepth)                = 0;

  virtual Void  resetEntropy          ()                = 0;
  virtual Void  setBitstream          ( TComBitIf* p )  = 0;
  virtual Void  setSlice              ( TComSlice* p )  = 0;
  virtual Void  resetBits             ()                = 0;
  virtual Void  resetCoeffCost        ()                = 0;
  virtual UInt  getNumberOfWrittenBits()                = 0;
  virtual UInt  getCoeffCost          ()                = 0;

  virtual Void  codeSPS                 ( TComSPS* pcSPS )                                      = 0;
  virtual Void  codePPS                 ( TComPPS* pcPPS )                                      = 0;
  virtual Void  codeSliceHeader         ( TComSlice* pcSlice )                                  = 0;
  virtual Void  codeTerminatingBit      ( UInt uilsLast )                                       = 0;
  virtual Void  codeSliceFinish         ()                                                      = 0;

  virtual Void codeAlfCtrlDepth() = 0;
  virtual Void codeMVPIdx ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList ) = 0;
#ifdef DCM_PBIC
  virtual Void codeICPIdx ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
#endif

public:
  virtual Void codeAlfCtrlFlag   ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;

#if HHI_ALF
  virtual Void codeAlfCoeff      ( Int iCoeff, Int iLength, Int iPos ) = 0;
  virtual Void codeAlfDc         ( Int iDc    ) = 0;
  virtual Void codeAlfQTCtrlFlag ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeAlfQTSplitFlag ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiMaxDepth ) = 0;
#endif

  virtual Void codeSkipFlag      ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
#if HHI_MRG
  virtual Void codeMergeFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeMergeIndex    ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
#endif
  virtual Void codeSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;

  virtual Void codePartSize      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;
  virtual Void codePredMode      ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
#if PLANAR_INTRA
  virtual Void codePlanarInfo    ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
#endif

#if HHI_RQT
  virtual Void codeTransformSubdivFlag( UInt uiSymbol, UInt uiCtx ) = 0;
  virtual Void codeQtCbf         ( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth ) = 0;
#if HHI_RQT_ROOT
  virtual Void codeQtRootCbf     ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
#endif
#endif
  virtual Void codeTransformIdx  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;
  virtual Void codeIntraDirLumaAdi( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
#if ANG_INTRA
  virtual Void codeIntraDirLumaAng( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
#endif

#if HHI_AIS
  virtual Void codeIntraFiltFlagLumaAdi( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
#endif

  virtual Void codeIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeInterDir      ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeRefFrmIdx     ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )      = 0;
  virtual Void codeMvd           ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )      = 0;
#ifdef DCM_PBIC
  virtual Void codeMvdIcd        ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )      = 0;
  virtual ContextModel* getZTreeCtx ( Int iIdx ) = 0;
#endif
  virtual Void codeDeltaQP       ( TComDataCU* pcCU, UInt uiAbsPartIdx ) = 0;
  virtual Void codeCbf           ( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth ) = 0;
#if QC_MDDT
  virtual Void codeCoeffNxN      ( TComDataCU* pcCU, TCoeff* pcCoef, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight, UInt uiDepth, TextType eTType, UInt uiMode, Bool bRD = false ) = 0;
#else
  virtual Void codeCoeffNxN      ( TComDataCU* pcCU, TCoeff* pcCoef, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight, UInt uiDepth, TextType eTType, Bool bRD = false ) = 0;
#endif
  virtual Void codeROTindex( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD ) = 0;
  virtual Void codeCIPflag ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD ) = 0;

  virtual Void codeAlfFlag          ( UInt uiCode ) = 0;
  virtual Void codeAlfUvlc          ( UInt uiCode ) = 0;
  virtual Void codeAlfSvlc          ( Int   iCode ) = 0;
#if TSB_ALF_HEADER
  virtual Void codeAlfFlagNum       ( UInt uiCode, UInt minValue ) = 0;
  virtual Void codeAlfCtrlFlag      ( UInt uiSymbol ) = 0;
#endif

  virtual Void estBit               (estBitsSbacStruct* pcEstBitsSbac, UInt uiCTXIdx, TextType eTType) = 0;
#ifdef QC_SIFO
  virtual Void encodeSwitched_Filters(TComSlice* pcSlice,TComPrediction *m_cPrediction) = 0;
#endif
};

/// entropy encoder class
class TEncEntropy
{
public:
  Void    setEntropyCoder           ( TEncEntropyIf* e, TComSlice* pcSlice );
  Void    setBitstream              ( TComBitIf* p )          { m_pcEntropyCoderIf->setBitstream(p);  }
  Void    resetBits                 ()                        { m_pcEntropyCoderIf->resetBits();      }
  Void    resetCoeffCost            ()                        { m_pcEntropyCoderIf->resetCoeffCost(); }
  UInt    getNumberOfWrittenBits    ()                        { return m_pcEntropyCoderIf->getNumberOfWrittenBits(); }
  UInt    getCoeffCost              ()                        { return  m_pcEntropyCoderIf->getCoeffCost(); }
  Void    resetEntropy              ()                        { m_pcEntropyCoderIf->resetEntropy();  }

  Void    encodeSliceHeader         ( TComSlice* pcSlice );
  Void    encodeTerminatingBit      ( UInt uiIsLast );
  Void    encodeSliceFinish         ();

  Void encodeAlfParam(ALFParam* pAlfParam);
  Void encodeMVPIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList, Bool bRD = false );
#ifdef DCM_PBIC
  Void encodeICPIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
#endif

  TEncEntropyIf*      m_pcEntropyCoderIf;

public:
  // SPS
  Void encodeSPS               ( TComSPS* pcSPS );
  Void encodePPS               ( TComPPS* pcPPS );
  Bool getAlfCtrl() {return m_pcEntropyCoderIf->getAlfCtrl();}
  UInt getMaxAlfCtrlDepth() {return m_pcEntropyCoderIf->getMaxAlfCtrlDepth();}
  Void setAlfCtrl(Bool bAlfCtrl) {m_pcEntropyCoderIf->setAlfCtrl(bAlfCtrl);}
  Void setMaxAlfCtrlDepth(UInt uiMaxAlfCtrlDepth) {m_pcEntropyCoderIf->setMaxAlfCtrlDepth(uiMaxAlfCtrlDepth);}

  Void encodeSplitFlag         ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD = false );
  Void encodeSkipFlag          ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
#if HHI_MRG
#if HHI_MRG_PU
  Void encodePUWise       ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodeMergeFlag    ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void encodeMergeIndex   ( TComDataCU* pcCU, UInt uiAbsPartIdx );

  Void encodeInterDirPU   ( TComDataCU* pcSubCU, UInt uiAbsPartIdx  );
  Void encodeRefFrmIdxPU  ( TComDataCU* pcSubCU, UInt uiAbsPartIdx, RefPicList eRefList );
  Void encodeMvdPU        ( TComDataCU* pcSubCU, UInt uiAbsPartIdx, RefPicList eRefList );
  Void encodeMVPIdxPU     ( TComDataCU* pcSubCU, UInt uiAbsPartIdx, RefPicList eRefList );
#ifdef DCM_PBIC
  Void encodeMvdIcdPU     ( TComDataCU* pcSubCU, UInt uiAbsPartIdx );
  Void encodeICPIdxPU     ( TComDataCU* pcSubCU, UInt uiAbsPartIdx );
#endif
#else
  Void encodeMergeFlag         ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodeMergeIndex        ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
#endif
#endif
#if HHI_ALF
  Void encodeAlfCtrlFlag       ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD , Bool bSeparateQt );
  Void encodeAlfQTSplitFlag    ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiMaxDepth, Bool bRD = false );
#else
  Void encodeAlfCtrlFlag       ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
#endif
#if TSB_ALF_HEADER
  Void encodeAlfCtrlParam      ( ALFParam *pAlfParam );
#endif
  Void encodePredMode          ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodePartSize          ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD = false );
#if PLANAR_INTRA
  Void encodePlanarInfo        ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
#endif

  Void encodePredInfo          ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodeIntraDirModeLuma  ( TComDataCU* pcCU, UInt uiAbsPartIdx );
#if HHI_AIS
  Void encodeIntraFiltFlagLuma ( TComDataCU* pcCU, UInt uiAbsPartIdx );
#endif

  Void encodeROTindex          ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD = false );
  Void encodeCIPflag           ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD = false );

#if HHI_MRG && !HHI_MRG_PU
  Void encodeMergeInfo         ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
#endif

  Void encodeIntraDirModeChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodeInterDir          ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );
  Void encodeRefFrmIdx         ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList, Bool bRD = false );
  Void encodeMvd               ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList, Bool bRD = false );
#ifdef DCM_PBIC
  Void encodeMvdIcd            ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList, Bool bRD = false );
#endif

  Void encodeTransformIdx      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD = false );
#if HHI_RQT
  Void encodeTransformSubdivFlag( UInt uiSymbol, UInt uiCtx );
  Void encodeQtCbf             ( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth );
#if HHI_RQT_ROOT
  Void encodeQtRootCbf         ( TComDataCU* pcCU, UInt uiAbsPartIdx );
#endif
#endif
  Void encodeCbf               ( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth, Bool bRD = false );
  Void encodeQP                ( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD = false );

#if HHI_RQT
private:
  Void xEncodeTransformSubdiv  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiInnerQuadIdx );
#endif
  Void xEncodeCoeff            ( TComDataCU* pcCU, TCoeff* pcCoeff, UInt uiAbsPartIdx, UInt uiDepth, UInt uiWidth, UInt uiHeight, UInt uiTrIdx, UInt uiCurrTrIdx, TextType eType, Bool bRD = false );
#if HHI_RQT
public:
#endif
  Void encodeCoeff             ( TComDataCU* pcCU                 , UInt uiAbsPartIdx, UInt uiDepth, UInt uiWidth, UInt uiHeight );
  Void encodeCoeff             ( TComDataCU* pcCU, TCoeff* pCoeff , UInt uiAbsPartIdx, UInt uiDepth, UInt uiWidth, UInt uiHeight, UInt uiMaxTrMode, UInt uiTrMode, TextType eType, Bool bRD = false );

  Void encodeCoeffNxN         ( TComDataCU* pcCU, TCoeff* pcCoeff, UInt uiAbsPartIdx, UInt uiTrWidth, UInt uiTrHeight, UInt uiDepth, TextType eType, Bool bRD = false );

  Void estimateBit             ( estBitsSbacStruct* pcEstBitsSbac, UInt uiWidth, TextType eTType);
#ifdef QC_SIFO
  Void encodeSwitched_Filters(TComSlice* pcSlice,TComPrediction *m_cPrediction);
#endif
#if QC_ALF
  Void codeAuxCountBit(ALFParam* pAlfParam, Int64* ruiRate);
  Void codeFiltCountBit(ALFParam* pAlfParam, Int64* ruiRate);
  Void codeAux (ALFParam* pAlfParam);
  Void codeFilt (ALFParam* pAlfParam);
  Int codeFilterCoeff(ALFParam* ALFp);
  Int writeFilterCodingParams(int minKStart, int maxScanVal, int kMinTab[]);
  Int writeFilterCoeffs(int sqrFiltLength, int filters_per_group, int pDepthInt[], 
                      int **FilterCoeff, int kMinTab[]);
  Int golombEncode(int coeff, int k);
  Int lengthGolomb(int coeffVal, int k);

#endif
};// END CLASS DEFINITION TEncEntropy


#endif // __TENCENTROPY__

>>>>>>> upstream/master
