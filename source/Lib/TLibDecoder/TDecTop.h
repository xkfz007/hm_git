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

/** \file     TDecTop.h
    \brief    decoder class (header)
*/

#ifndef __TDECTOP__
#define __TDECTOP__

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPicYuv.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComPrediction.h"
#include "TLibCommon/SEI.h"

#include "TDecGop.h"
#include "TDecEntropy.h"
#include "TDecSbac.h"
#include "TDecCAVLC.h"
#include "SEIread.h"
#define OUTPUT_FRM_LEN 1
class InputNALUnit;

//! \ingroup TLibDecoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// decoder class
class TDecTop
{
private:
  Int                     m_iMaxRefPicNum;

  NalUnitType             m_associatedIRAPType; ///< NAL unit type of the associated IRAP picture
  Int                     m_pocCRA;            ///< POC number of the latest CRA picture
  Int                     m_pocRandomAccess;   ///< POC number of the random access point (the first IDR or CRA picture)

  TComList<TComPic*>      m_cListPic;         //  Dynamic buffer
  ParameterSetManager     m_parameterSetManager;  // storage for parameter sets
  TComSlice*              m_apcSlicePilot;

  SEIMessages             m_SEIs; ///< List of SEI messages that have been received before the first slice and between slices, excluding prefix SEIs...

  // functional classes
  TComPrediction          m_cPrediction;
  TComTrQuant             m_cTrQuant;
  TDecGop                 m_cGopDecoder;
  TDecSlice               m_cSliceDecoder;
  TDecCu                  m_cCuDecoder;
  TDecEntropy             m_cEntropyDecoder;
  TDecCavlc               m_cCavlcDecoder;
  TDecSbac                m_cSbacDecoder;
  TDecBinCABAC            m_cBinCABAC;
  SEIReader               m_seiReader;
  TComLoopFilter          m_cLoopFilter;
  TComSampleAdaptiveOffset m_cSAO;

  Bool isSkipPictureForBLA(Int& iPOCLastDisplay);
  Bool isRandomAccessSkipPicture(Int& iSkipFrame,  Int& iPOCLastDisplay);
  TComPic*                m_pcPic;
  UInt                    m_uiSliceIdx;
  Int                     m_prevPOC;
  Int                     m_prevTid0POC;
  Bool                    m_bFirstSliceInPicture;
  Bool                    m_bFirstSliceInSequence;
  Bool                    m_prevSliceSkipped;
  Int                     m_skippedPOC;
  Bool                    m_bFirstSliceInBitstream;
  Int                     m_lastPOCNoOutputPriorPics;
  Bool                    m_isNoOutputPriorPics;
  Bool                    m_craNoRaslOutputFlag;    //value of variable NoRaslOutputFlag of the last CRA pic
#if O0043_BEST_EFFORT_DECODING
  UInt                    m_forceDecodeBitDepth;
#endif
  std::ostream           *m_pDecodedSEIOutputStream;

  Bool                    m_warningMessageSkipPicture;

  std::list<InputNALUnit*> m_prefixSEINALUs; /// Buffered up prefix SEI NAL Units.
public:
  TDecTop();
  virtual ~TDecTop();

  Void  create  ();
  Void  destroy ();

  Void setDecodedPictureHashSEIEnabled(Int enabled) { m_cGopDecoder.setDecodedPictureHashSEIEnabled(enabled); }

  Void  init();
  Bool  decode(InputNALUnit& nalu, Int& iSkipFrame, Int& iPOCLastDisplay);
  Void  deletePicBuffer();

  
  Void  executeLoopFilters(Int& poc, TComList<TComPic*>*& rpcListPic);
  Void  checkNoOutputPriorPics (TComList<TComPic*>* rpcListPic);

  Bool  getNoOutputPriorPicsFlag () { return m_isNoOutputPriorPics; }
  Void  setNoOutputPriorPicsFlag (Bool val) { m_isNoOutputPriorPics = val; }
  Void  setFirstSliceInPicture (bool val)  { m_bFirstSliceInPicture = val; }
  Bool  getFirstSliceInSequence ()         { return m_bFirstSliceInSequence; }
  Void  setFirstSliceInSequence (bool val) { m_bFirstSliceInSequence = val; }
#if O0043_BEST_EFFORT_DECODING
  Void  setForceDecodeBitDepth(UInt bitDepth) { m_forceDecodeBitDepth = bitDepth; }
#endif
  Void  setDecodedSEIMessageOutputStream(std::ostream *pOpStream) { m_pDecodedSEIOutputStream = pOpStream; }
  UInt  getNumberOfChecksumErrorsDetected() const { return m_cGopDecoder.getNumberOfChecksumErrorsDetected(); }
#if OUTPUT_FRM_LEN
  int frm_len;
#endif

protected:
  Void  xGetNewPicBuffer  (const TComSPS &sps, const TComPPS &pps, TComPic*& rpcPic, const UInt temporalLayer);
  Void  xCreateLostPicture (Int iLostPOC);

  Void      xActivateParameterSets();
  Bool      xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int iPOCLastDisplay);
  Void      xDecodeVPS(const std::vector<UChar> &naluData);
  Void      xDecodeSPS(const std::vector<UChar> &naluData);
  Void      xDecodePPS(const std::vector<UChar> &naluData);
  Void      xUpdatePreviousTid0POC( TComSlice *pSlice ) { if ((pSlice->getTLayer()==0) && (pSlice->isReferenceNalu() && (pSlice->getNalUnitType()!=NAL_UNIT_CODED_SLICE_RASL_R)&& (pSlice->getNalUnitType()!=NAL_UNIT_CODED_SLICE_RADL_R))) { m_prevTid0POC=pSlice->getPOC(); } }
  Void      xParsePrefixSEImessages();
  Void      xParsePrefixSEIsForUnknownVCLNal();

};// END CLASS DEFINITION TDecTop


//! \}

#endif // __TDECTOP__

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

/** \file     TDecTop.h
    \brief    decoder class (header)
*/

#ifndef __TDECTOP__
#define __TDECTOP__

#include "../TLibCommon/CommonDef.h"
#include "../TLibCommon/TComList.h"
#include "../TLibCommon/TComPicYuv.h"
#include "../TLibCommon/TComPic.h"
#include "../TLibCommon/TComTrQuant.h"

#include "TDecGop.h"
#include "TDecEntropy.h"
#include "TDecSbac.h"
#include "TDecCAVLC.h"

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// decoder class
class TDecTop
{
private:
  Int                     m_iGopSize;
  Bool                    m_bGopSizeSet;
  int                     m_iMaxRefPicNum;
  UInt                    m_uiBalancedCPUs;

  UInt                    m_uiValidPS;
  TComList<TComPic*>      m_cListPic;         //  Dynamic buffer
  TComSPS                 m_cSPS;
  TComPPS                 m_cPPS;
  TComSlice*              m_apcSlicePilot;

  // functional classes
  TComPrediction          m_cPrediction;
  TComTrQuant             m_cTrQuant;
  TDecGop                 m_cGopDecoder;
  TDecSlice               m_cSliceDecoder;
  TDecCu                  m_cCuDecoder;
  TDecEntropy             m_cEntropyDecoder;
  TDecCavlc               m_cCavlcDecoder;
  TDecSbac                m_cSbacDecoder;
  TDecBinCABAC            m_cBinCABAC;
  TDecBinMultiCABAC       m_cBinMultiCABAC;
  TDecBinPIPE             m_cBinPIPE;
  TDecBinMultiPIPE        m_cBinMultiPIPE;
  TDecV2V                 m_cBinV2VwLB;
  TComLoopFilter          m_cLoopFilter;
  TComAdaptiveLoopFilter  m_cAdaptiveLoopFilter;

public:
  TDecTop();
  virtual ~TDecTop();

  Void  create  ();
  Void  destroy ();

  Void  init();
  Void  decode ( Bool bEos, TComBitstream* pcBitstream, UInt& ruiPOC, TComList<TComPic*>*& rpcListPic );

  Void  deletePicBuffer();

  Void  setBalancedCPUs( UInt ui ) { m_uiBalancedCPUs = ui; }
  UInt  getBalancedCPUs() { return m_cSPS.getBalancedCPUs(); }

protected:
  Void  xGetNewPicBuffer  (TComSlice* pcSlice, TComPic*& rpcPic);
  Void  xUpdateGopSize    (TComSlice* pcSlice);

};// END CLASS DEFINITION TDecTop


#endif // __TDECTOP__

>>>>>>> upstream/master
