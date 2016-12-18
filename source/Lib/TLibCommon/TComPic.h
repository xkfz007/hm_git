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

/** \file     TComPic.h
    \brief    picture class (header)
*/

#ifndef __TCOMPIC__
#define __TCOMPIC__

// Include files
#include "CommonDef.h"
#include "TComPicSym.h"
#include "TComPicYuv.h"
#include "TComBitStream.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// picture class (symbol + YUV buffers)

class TComPic
{
public:
  typedef enum { PIC_YUV_ORG=0, PIC_YUV_REC=1, PIC_YUV_TRUE_ORG=2, NUM_PIC_YUV=3 } PIC_YUV_T;
     // TRUE_ORG is the input file without any pre-encoder colour space conversion (but with possible bit depth increment)
  TComPicYuv*   getPicYuvTrueOrg()        { return  m_apcPicYuv[PIC_YUV_TRUE_ORG]; }

private:
  UInt                  m_uiTLayer;               //  Temporal layer
  Bool                  m_bUsedByCurr;            //  Used by current picture
  Bool                  m_bIsLongTerm;            //  IS long term picture
  TComPicSym            m_picSym;                 //  Symbol
  TComPicYuv*           m_apcPicYuv[NUM_PIC_YUV];

  TComPicYuv*           m_pcPicYuvPred;           //  Prediction
  TComPicYuv*           m_pcPicYuvResi;           //  Residual
  Bool                  m_bReconstructed;
  Bool                  m_bNeededForOutput;
  UInt                  m_uiCurrSliceIdx;         // Index of current slice
  Bool                  m_bCheckLTMSB;

  Bool                  m_isTop;
  Bool                  m_isField;

  std::vector<std::vector<TComDataCU*> > m_vSliceCUDataLink;

  SEIMessages  m_SEIs; ///< Any SEI messages that have been received.  If !NULL we own the object.

public:
  TComPic();
  virtual ~TComPic();

  Void          create( const TComSPS &sps, const TComPPS &pps, const Bool bIsVirtual /*= false*/ );

  virtual Void  destroy();

  UInt          getTLayer() const               { return m_uiTLayer;   }
  Void          setTLayer( UInt uiTLayer ) { m_uiTLayer = uiTLayer; }

  Bool          getUsedByCurr() const            { return m_bUsedByCurr; }
  Void          setUsedByCurr( Bool bUsed ) { m_bUsedByCurr = bUsed; }
  Bool          getIsLongTerm() const            { return m_bIsLongTerm; }
  Void          setIsLongTerm( Bool lt ) { m_bIsLongTerm = lt; }
  Void          setCheckLTMSBPresent     (Bool b ) {m_bCheckLTMSB=b;}
  Bool          getCheckLTMSBPresent     () { return m_bCheckLTMSB;}

  TComPicSym*   getPicSym()           { return  &m_picSym;    }
  TComSlice*    getSlice(Int i)       { return  m_picSym.getSlice(i);  }
  Int           getPOC() const        { return  m_picSym.getSlice(m_uiCurrSliceIdx)->getPOC();  }
  TComDataCU*   getCtu( UInt ctuRsAddr )           { return  m_picSym.getCtu( ctuRsAddr ); }
  const TComDataCU* getCtu( UInt ctuRsAddr ) const { return  m_picSym.getCtu( ctuRsAddr ); }

  TComPicYuv*   getPicYuvOrg()        { return  m_apcPicYuv[PIC_YUV_ORG]; }
  TComPicYuv*   getPicYuvRec()        { return  m_apcPicYuv[PIC_YUV_REC]; }

  TComPicYuv*   getPicYuvPred()       { return  m_pcPicYuvPred; }
  TComPicYuv*   getPicYuvResi()       { return  m_pcPicYuvResi; }
  Void          setPicYuvPred( TComPicYuv* pcPicYuv )       { m_pcPicYuvPred = pcPicYuv; }
  Void          setPicYuvResi( TComPicYuv* pcPicYuv )       { m_pcPicYuvResi = pcPicYuv; }

  UInt          getNumberOfCtusInFrame() const     { return m_picSym.getNumberOfCtusInFrame(); }
  UInt          getNumPartInCtuWidth() const       { return m_picSym.getNumPartInCtuWidth();   }
  UInt          getNumPartInCtuHeight() const      { return m_picSym.getNumPartInCtuHeight();  }
  UInt          getNumPartitionsInCtu() const      { return m_picSym.getNumPartitionsInCtu();  }
  UInt          getFrameWidthInCtus() const        { return m_picSym.getFrameWidthInCtus();    }
  UInt          getFrameHeightInCtus() const       { return m_picSym.getFrameHeightInCtus();   }
  UInt          getMinCUWidth() const              { return m_picSym.getMinCUWidth();          }
  UInt          getMinCUHeight() const             { return m_picSym.getMinCUHeight();         }

  Int           getStride(const ComponentID id) const          { return m_apcPicYuv[PIC_YUV_REC]->getStride(id); }
  Int           getComponentScaleX(const ComponentID id) const    { return m_apcPicYuv[PIC_YUV_REC]->getComponentScaleX(id); }
  Int           getComponentScaleY(const ComponentID id) const    { return m_apcPicYuv[PIC_YUV_REC]->getComponentScaleY(id); }
  ChromaFormat  getChromaFormat() const                           { return m_apcPicYuv[PIC_YUV_REC]->getChromaFormat(); }
  Int           getNumberValidComponents() const                  { return m_apcPicYuv[PIC_YUV_REC]->getNumberValidComponents(); }

  Void          setReconMark (Bool b) { m_bReconstructed = b;     }
  Bool          getReconMark () const      { return m_bReconstructed;  }
  Void          setOutputMark (Bool b) { m_bNeededForOutput = b;     }
  Bool          getOutputMark () const      { return m_bNeededForOutput;  }

  Void          compressMotion();
  UInt          getCurrSliceIdx() const           { return m_uiCurrSliceIdx;                }
  Void          setCurrSliceIdx(UInt i)      { m_uiCurrSliceIdx = i;                   }
  UInt          getNumAllocatedSlice() const      {return m_picSym.getNumAllocatedSlice();}
  Void          allocateNewSlice()           {m_picSym.allocateNewSlice();         }
  Void          clearSliceBuffer()           {m_picSym.clearSliceBuffer();         }

  const Window& getConformanceWindow() const { return m_picSym.getSPS().getConformanceWindow(); }
  Window        getDefDisplayWindow() const  { return m_picSym.getSPS().getVuiParametersPresentFlag() ? m_picSym.getSPS().getVuiParameters()->getDefaultDisplayWindow() : Window(); }

  Bool          getSAOMergeAvailability(Int currAddr, Int mergeAddr);

  UInt          getSubstreamForCtuAddr(const UInt ctuAddr, const Bool bAddressInRaster, TComSlice *pcSlice);

  /* field coding parameters*/

   Void              setTopField(Bool b)                  {m_isTop = b;}
   Bool              isTopField()                         {return m_isTop;}
   Void              setField(Bool b)                     {m_isField = b;}
   Bool              isField()                            {return m_isField;}

  /** transfer ownership of seis to this picture */
  Void setSEIs(SEIMessages& seis) { m_SEIs = seis; }

  /**
   * return the current list of SEI messages associated with this picture.
   * Pointer is valid until this->destroy() is called */
  SEIMessages& getSEIs() { return m_SEIs; }

  /**
   * return the current list of SEI messages associated with this picture.
   * Pointer is valid until this->destroy() is called */
  const SEIMessages& getSEIs() const { return m_SEIs; }
#if 1//OUTPUT_FRM_LEN
  int pic_len;
#endif
};// END CLASS DEFINITION TComPic

//! \}

#endif // __TCOMPIC__
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

/** \file     TComPic.h
    \brief    picture class (header)
*/

#ifndef __TCOMPIC__
#define __TCOMPIC__

// Include files
#include "CommonDef.h"
#include "TComPicSym.h"
#include "TComPicYuv.h"
#include "TComBitStream.h"

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// picture class (symbol + YUV buffers)
class TComPic
{
private:
  TComPicSym*           m_apcPicSym;              //  Symbol

#if HHI_INTERP_FILTER
  TComPicYuv*           m_apcPicYuv[3];           //  Texture,  0:org / 1:rec / 2:rec-filtered
#else
  TComPicYuv*           m_apcPicYuv[2];           //  Texture,  0:org / 1:rec
#endif

  TComPicYuv*           m_pcPicYuvPred;           //  Prediction
  TComPicYuv*           m_pcPicYuvResi;           //  Residual
  Bool                  m_bReconstructed;

public:
  TComPic();
  virtual ~TComPic();

  Void          create( Int iWidth, Int iHeight, UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxDepth, Bool bIsVirtual = false );
  Void          destroy();

  TComPicSym*   getPicSym()           { return  m_apcPicSym;    }
  TComSlice*    getSlice()            { return  m_apcPicSym->getSlice();  }
  Int           getPOC()              { return  m_apcPicSym->getSlice()->getPOC();  }
  Bool          getDRBFlag()          { return  m_apcPicSym->getSlice()->getDRBFlag();  }
  Int           getERBIndex()         { return  m_apcPicSym->getSlice()->getERBIndex();  }
  TComDataCU*&  getCU( UInt uiCUAddr )  { return  m_apcPicSym->getCU( uiCUAddr ); }

  TComPicYuv*   getPicYuvOrg()        { return  m_apcPicYuv[0]; }
  TComPicYuv*   getPicYuvRec()        { return  m_apcPicYuv[1]; }

#if HHI_INTERP_FILTER
  TComPicYuv*   getPicYuvRecFilt()    { return  m_apcPicYuv[2]; }
#endif

  TComPicYuv*   getPicYuvPred()       { return  m_pcPicYuvPred; }
  TComPicYuv*   getPicYuvResi()       { return  m_pcPicYuvResi; }
  Void          setPicYuvPred( TComPicYuv* pcPicYuv )       { m_pcPicYuvPred = pcPicYuv; }
  Void          setPicYuvResi( TComPicYuv* pcPicYuv )       { m_pcPicYuvResi = pcPicYuv; }

  UInt          getNumCUsInFrame()      { return m_apcPicSym->getNumberOfCUsInFrame(); }
  UInt          getNumPartInWidth()     { return m_apcPicSym->getNumPartInWidth();     }
  UInt          getNumPartInHeight()    { return m_apcPicSym->getNumPartInHeight();    }
  UInt          getNumPartInCU()        { return m_apcPicSym->getNumPartition();       }
  UInt          getFrameWidthInCU()     { return m_apcPicSym->getFrameWidthInCU();     }
  UInt          getFrameHeightInCU()    { return m_apcPicSym->getFrameHeightInCU();    }
  UInt          getMinCUWidth()         { return m_apcPicSym->getMinCUWidth();         }
  UInt          getMinCUHeight()        { return m_apcPicSym->getMinCUHeight();        }

  UInt          getParPelX(UChar uhPartIdx) { return getParPelX(uhPartIdx); }
  UInt          getParPelY(UChar uhPartIdx) { return getParPelX(uhPartIdx); }

  Int           getStride()           { return m_apcPicYuv[1]->getStride(); }
  Int           getCStride()          { return m_apcPicYuv[1]->getCStride(); }

  Void          setReconMark (Bool b) { m_bReconstructed = b;     }
  Bool          getReconMark ()       { return m_bReconstructed;  }

};// END CLASS DEFINITION TComPic


#endif // __TCOMPIC__

>>>>>>> upstream/master
