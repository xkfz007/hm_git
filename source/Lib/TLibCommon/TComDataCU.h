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

/** \file     TComDataCU.h
    \brief    CU data structure (header)
    \todo     not all entities are documented
*/

#ifndef __TCOMDATACU__
#define __TCOMDATACU__

#include <algorithm>
#include <vector>

// Include files
#include "CommonDef.h"
#include "TComMotionInfo.h"
#include "TComSlice.h"
#include "TComRdCost.h"
#include "TComPattern.h"

//! \ingroup TLibCommon
//! \{

class TComTU; // forward declaration

static const UInt NUM_MOST_PROBABLE_MODES=3;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU data structure class
class TComDataCU
{
private:

  // -------------------------------------------------------------------------------------------------------------------
  // class pointers
  // -------------------------------------------------------------------------------------------------------------------

  TComPic*      m_pcPic;              ///< picture class pointer
  TComSlice*    m_pcSlice;            ///< slice header pointer

  // -------------------------------------------------------------------------------------------------------------------
  // CU description
  // -------------------------------------------------------------------------------------------------------------------

  UInt          m_ctuRsAddr;          ///< CTU (also known as LCU) address in a slice (Raster-scan address, as opposed to tile-scan/encoding order).
  UInt          m_absZIdxInCtu;       ///< absolute address in a CTU. It's Z scan order
  UInt          m_uiCUPelX;           ///< CU position in a pixel (X)
  UInt          m_uiCUPelY;           ///< CU position in a pixel (Y)
  UInt          m_uiNumPartition;     ///< total number of minimum partitions in a CU
  UChar*        m_puhWidth;           ///< array of widths
  UChar*        m_puhHeight;          ///< array of heights
  UChar*        m_puhDepth;           ///< array of depths
  Int           m_unitSize;           ///< size of a "minimum partition"

  // -------------------------------------------------------------------------------------------------------------------
  // CU data
  // -------------------------------------------------------------------------------------------------------------------

  Bool*          m_skipFlag;           ///< array of skip flags
  Char*          m_pePartSize;         ///< array of partition sizes
  Char*          m_pePredMode;         ///< array of prediction modes
  Char*          m_crossComponentPredictionAlpha[MAX_NUM_COMPONENT]; ///< array of cross-component prediction alpha values
  Bool*          m_CUTransquantBypass;   ///< array of cu_transquant_bypass flags
  Char*          m_phQP;               ///< array of QP values
  UChar*         m_ChromaQpAdj;        ///< array of chroma QP adjustments (indexed). when value = 0, cu_chroma_qp_offset_flag=0; when value>0, indicates cu_chroma_qp_offset_flag=1 and cu_chroma_qp_offset_idx=value-1
  UInt           m_codedChromaQpAdj;
  UChar*         m_puhTrIdx;           ///< array of transform indices
  UChar*         m_puhTransformSkip[MAX_NUM_COMPONENT];///< array of transform skipping flags
  UChar*         m_puhCbf[MAX_NUM_COMPONENT];          ///< array of coded block flags (CBF)
  TComCUMvField  m_acCUMvField[NUM_REF_PIC_LIST_01];    ///< array of motion vectors.
  TCoeff*        m_pcTrCoeff[MAX_NUM_COMPONENT];       ///< array of transform coefficient buffers (0->Y, 1->Cb, 2->Cr)
#if ADAPTIVE_QP_SELECTION
  TCoeff*        m_pcArlCoeff[MAX_NUM_COMPONENT];  // ARL coefficient buffer (0->Y, 1->Cb, 2->Cr)
  Bool           m_ArlCoeffIsAliasedAllocation;  ///< ARL coefficient buffer is an alias of the global buffer and must not be free()'d
#endif

  Pel*           m_pcIPCMSample[MAX_NUM_COMPONENT];    ///< PCM sample buffer (0->Y, 1->Cb, 2->Cr)

  // -------------------------------------------------------------------------------------------------------------------
  // neighbour access variables
  // -------------------------------------------------------------------------------------------------------------------

  TComDataCU*   m_pCtuAboveLeft;      ///< pointer of above-left CTU.
  TComDataCU*   m_pCtuAboveRight;     ///< pointer of above-right CTU.
  TComDataCU*   m_pCtuAbove;          ///< pointer of above CTU.
  TComDataCU*   m_pCtuLeft;           ///< pointer of left CTU
  TComDataCU*   m_apcCUColocated[NUM_REF_PIC_LIST_01];  ///< pointer of temporally colocated CU's for both directions
  TComMvField   m_cMvFieldA;          ///< motion vector of position A
  TComMvField   m_cMvFieldB;          ///< motion vector of position B
  TComMvField   m_cMvFieldC;          ///< motion vector of position C
  TComMv        m_cMvPred;            ///< motion vector predictor

  // -------------------------------------------------------------------------------------------------------------------
  // coding tool information
  // -------------------------------------------------------------------------------------------------------------------

  Bool*         m_pbMergeFlag;        ///< array of merge flags
  UChar*        m_puhMergeIndex;      ///< array of merge candidate indices
#if AMP_MRG
  Bool          m_bIsMergeAMP;
#endif
  UChar*        m_puhIntraDir[MAX_NUM_CHANNEL_TYPE]; // 0-> Luma, 1-> Chroma
  UChar*        m_puhInterDir;        ///< array of inter directions
  Char*         m_apiMVPIdx[NUM_REF_PIC_LIST_01];       ///< array of motion vector predictor candidates
  Char*         m_apiMVPNum[NUM_REF_PIC_LIST_01];       ///< array of number of possible motion vectors predictors
  Bool*         m_pbIPCMFlag;         ///< array of intra_pcm flags

  // -------------------------------------------------------------------------------------------------------------------
  // misc. variables
  // -------------------------------------------------------------------------------------------------------------------

  Bool          m_bDecSubCu;          ///< indicates decoder-mode
  Double        m_dTotalCost;         ///< sum of partition RD costs
  Distortion    m_uiTotalDistortion;  ///< sum of partition distortion
  UInt          m_uiTotalBits;        ///< sum of partition bits
  UInt          m_uiTotalBins;        ///< sum of partition bins
  Char          m_codedQP;
  UChar*        m_explicitRdpcmMode[MAX_NUM_COMPONENT]; ///< Stores the explicit RDPCM mode for all TUs belonging to this CU

protected:

  /// add possible motion vector predictor candidates
  Bool          xAddMVPCand           ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );
  Bool          xAddMVPCandOrder      ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );

  Void          deriveRightBottomIdx        ( UInt uiPartIdx, UInt& ruiPartIdxRB );
  Bool          xGetColMVP( RefPicList eRefPicList, Int ctuRsAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx );

  /// compute scaling factor from POC difference
  Int           xGetDistScaleFactor   ( Int iCurrPOC, Int iCurrRefPOC, Int iColPOC, Int iColRefPOC );

  Void xDeriveCenterIdx( UInt uiPartIdx, UInt& ruiPartIdxCenter );

public:
  TComDataCU();
  virtual ~TComDataCU();

  // -------------------------------------------------------------------------------------------------------------------
  // create / destroy / initialize / copy
  // -------------------------------------------------------------------------------------------------------------------

  Void          create                ( ChromaFormat chromaFormatIDC, UInt uiNumPartition, UInt uiWidth, UInt uiHeight, Bool bDecSubCu, Int unitSize
#if ADAPTIVE_QP_SELECTION
    , TCoeff *pParentARLBuffer = 0
#endif
    );
  Void          destroy               ();

  Void          initCtu               ( TComPic* pcPic, UInt ctuRsAddr );
  Void          initEstData           ( const UInt uiDepth, const Int qp, const Bool bTransquantBypass );
  Void          initSubCU             ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth, Int qp );
  Void          setOutsideCUPart      ( UInt uiAbsPartIdx, UInt uiDepth );

  Void          copySubCU             ( TComDataCU* pcCU, UInt uiPartUnitIdx );
  Void          copyInterPredInfoFrom ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList );
  Void          copyPartFrom          ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth );

  Void          copyToPic             ( UChar uiDepth );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for CU description
  // -------------------------------------------------------------------------------------------------------------------

  TComPic*        getPic              ()                        { return m_pcPic;           }
  const TComPic*  getPic              () const                  { return m_pcPic;           }
  TComSlice*       getSlice           ()                        { return m_pcSlice;         }
  const TComSlice* getSlice           () const                  { return m_pcSlice;         }
  UInt&         getCtuRsAddr          ()                        { return m_ctuRsAddr;       }
  UInt          getCtuRsAddr          () const                  { return m_ctuRsAddr;       }
  UInt          getZorderIdxInCtu     () const                  { return m_absZIdxInCtu;    }
  UInt          getCUPelX             () const                  { return m_uiCUPelX;        }
  UInt          getCUPelY             () const                  { return m_uiCUPelY;        }

  UChar*        getDepth              ()                        { return m_puhDepth;        }
  UChar         getDepth              ( UInt uiIdx ) const      { return m_puhDepth[uiIdx]; }
  Void          setDepth              ( UInt uiIdx, UChar  uh ) { m_puhDepth[uiIdx] = uh;   }

  Void          setDepthSubParts      ( UInt uiDepth, UInt uiAbsPartIdx );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for CU data
  // -------------------------------------------------------------------------------------------------------------------

  Char*         getPartitionSize      ()                        { return m_pePartSize;        }
  PartSize      getPartitionSize      ( UInt uiIdx )            { return static_cast<PartSize>( m_pePartSize[uiIdx] ); }
  Void          setPartitionSize      ( UInt uiIdx, PartSize uh){ m_pePartSize[uiIdx] = uh;   }
  Void          setPartSizeSubParts   ( PartSize eMode, UInt uiAbsPartIdx, UInt uiDepth );
  Void          setCUTransquantBypassSubParts( Bool flag, UInt uiAbsPartIdx, UInt uiDepth );

  Bool*         getSkipFlag            ()                        { return m_skipFlag;          }
  Bool          getSkipFlag            (UInt idx)                { return m_skipFlag[idx];     }
  Void          setSkipFlag           ( UInt idx, Bool skip)     { m_skipFlag[idx] = skip;   }
  Void          setSkipFlagSubParts   ( Bool skip, UInt absPartIdx, UInt depth );

  Char*         getPredictionMode     ()                        { return m_pePredMode;        }
  PredMode      getPredictionMode     ( UInt uiIdx )            { return static_cast<PredMode>( m_pePredMode[uiIdx] ); }
  Void          setPredictionMode     ( UInt uiIdx, PredMode uh){ m_pePredMode[uiIdx] = uh;   }
  Void          setPredModeSubParts   ( PredMode eMode, UInt uiAbsPartIdx, UInt uiDepth );

  Char*         getCrossComponentPredictionAlpha( ComponentID compID )             { return m_crossComponentPredictionAlpha[compID];         }
  Char          getCrossComponentPredictionAlpha( UInt uiIdx, ComponentID compID ) { return m_crossComponentPredictionAlpha[compID][uiIdx];  }

  Bool*         getCUTransquantBypass ()                        { return m_CUTransquantBypass;        }
  Bool          getCUTransquantBypass( UInt uiIdx )             { return m_CUTransquantBypass[uiIdx]; }

  UChar*        getWidth              ()                        { return m_puhWidth;          }
  UChar         getWidth              ( UInt uiIdx )            { return m_puhWidth[uiIdx];   }
  Void          setWidth              ( UInt uiIdx, UChar  uh ) { m_puhWidth[uiIdx] = uh;     }

  UChar*        getHeight             ()                        { return m_puhHeight;         }
  UChar         getHeight             ( UInt uiIdx )            { return m_puhHeight[uiIdx];  }
  Void          setHeight             ( UInt uiIdx, UChar  uh ) { m_puhHeight[uiIdx] = uh;    }

  Void          setSizeSubParts       ( UInt uiWidth, UInt uiHeight, UInt uiAbsPartIdx, UInt uiDepth );

  Char*         getQP                 ()                        { return m_phQP;              }
  Char          getQP                 ( UInt uiIdx ) const      { return m_phQP[uiIdx];       }
  Void          setQP                 ( UInt uiIdx, Char value ){ m_phQP[uiIdx] =  value;     }
  Void          setQPSubParts         ( Int qp,   UInt uiAbsPartIdx, UInt uiDepth );
  Int           getLastValidPartIdx   ( Int iAbsPartIdx );
  Char          getLastCodedQP        ( UInt uiAbsPartIdx );
  Void          setQPSubCUs           ( Int qp, UInt absPartIdx, UInt depth, Bool &foundNonZeroCbf );
  Void          setCodedQP            ( Char qp )               { m_codedQP = qp;             }
  Char          getCodedQP            ()                        { return m_codedQP;           }

  UChar*        getChromaQpAdj        ()                        { return m_ChromaQpAdj;       } ///< array of chroma QP adjustments (indexed). when value = 0, cu_chroma_qp_offset_flag=0; when value>0, indicates cu_chroma_qp_offset_flag=1 and cu_chroma_qp_offset_idx=value-1
  UChar         getChromaQpAdj        (Int idx)           const { return m_ChromaQpAdj[idx];  } ///< When value = 0, cu_chroma_qp_offset_flag=0; when value>0, indicates cu_chroma_qp_offset_flag=1 and cu_chroma_qp_offset_idx=value-1
  Void          setChromaQpAdj        (Int idx, UChar val)      { m_ChromaQpAdj[idx] = val;   } ///< When val = 0,   cu_chroma_qp_offset_flag=0; when val>0,   indicates cu_chroma_qp_offset_flag=1 and cu_chroma_qp_offset_idx=val-1
  Void          setChromaQpAdjSubParts( UChar val, Int absPartIdx, Int depth );
  Void          setCodedChromaQpAdj   ( Char qp )               { m_codedChromaQpAdj = qp;    }
  Char          getCodedChromaQpAdj   ()                        { return m_codedChromaQpAdj;  }

  Bool          isLosslessCoded       ( UInt absPartIdx );

  UChar*        getTransformIdx       ()                        { return m_puhTrIdx;          }
  UChar         getTransformIdx       ( UInt uiIdx )            { return m_puhTrIdx[uiIdx];   }
  Void          setTrIdxSubParts      ( UInt uiTrIdx, UInt uiAbsPartIdx, UInt uiDepth );

  UChar*        getTransformSkip      ( ComponentID compID )    { return m_puhTransformSkip[compID];}
  UChar         getTransformSkip      ( UInt uiIdx, ComponentID compID)    { return m_puhTransformSkip[compID][uiIdx];}
  Void          setTransformSkipSubParts  ( UInt useTransformSkip, ComponentID compID, UInt uiAbsPartIdx, UInt uiDepth);
  Void          setTransformSkipSubParts  ( const UInt useTransformSkip[MAX_NUM_COMPONENT], UInt uiAbsPartIdx, UInt uiDepth );

  UChar*        getExplicitRdpcmMode      ( ComponentID component ) { return m_explicitRdpcmMode[component]; }
  UChar         getExplicitRdpcmMode      ( ComponentID component, UInt partIdx ) {return m_explicitRdpcmMode[component][partIdx]; }
  Void          setExplicitRdpcmModePartRange ( UInt rdpcmMode, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes );

  Bool          isRDPCMEnabled         ( UInt uiAbsPartIdx )  { return getSlice()->getSPS()->getSpsRangeExtension().getRdpcmEnabledFlag(isIntra(uiAbsPartIdx) ? RDPCM_SIGNAL_IMPLICIT : RDPCM_SIGNAL_EXPLICIT); }

  Void          setCrossComponentPredictionAlphaPartRange    ( Char alphaValue, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes );
  Void          setTransformSkipPartRange                    ( UInt useTransformSkip, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes );

  UInt          getQuadtreeTULog2MinSizeInCU( UInt uiIdx );

  TComCUMvField* getCUMvField         ( RefPicList e )          { return  &m_acCUMvField[e];  }

  TCoeff*       getCoeff              (ComponentID component)   { return m_pcTrCoeff[component]; }

#if ADAPTIVE_QP_SELECTION
  TCoeff*       getArlCoeff           ( ComponentID component ) { return m_pcArlCoeff[component]; }
#endif
  Pel*          getPCMSample          ( ComponentID component ) { return m_pcIPCMSample[component]; }

  UChar         getCbf    ( UInt uiIdx, ComponentID eType )                  { return m_puhCbf[eType][uiIdx];  }
  UChar*        getCbf    ( ComponentID eType )                              { return m_puhCbf[eType];         }
  UChar         getCbf    ( UInt uiIdx, ComponentID eType, UInt uiTrDepth )  { return ( ( getCbf( uiIdx, eType ) >> uiTrDepth ) & 0x1 ); }
  Void          setCbf    ( UInt uiIdx, ComponentID eType, UChar uh )        { m_puhCbf[eType][uiIdx] = uh;    }
  Void          clearCbf  ( UInt uiIdx, ComponentID eType, UInt uiNumParts );
  UChar         getQtRootCbf          ( UInt uiIdx );

  Void          setCbfSubParts        ( const UInt uiCbf[MAX_NUM_COMPONENT],  UInt uiAbsPartIdx, UInt uiDepth           );
  Void          setCbfSubParts        ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiDepth                    );
  Void          setCbfSubParts        ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth    );

  Void          setCbfPartRange       ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes      );
  Void          bitwiseOrCbfPartRange ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes      );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for coding tool information
  // -------------------------------------------------------------------------------------------------------------------

  Bool*         getMergeFlag          ()                        { return m_pbMergeFlag;               }
  Bool          getMergeFlag          ( UInt uiIdx )            { return m_pbMergeFlag[uiIdx];        }
  Void          setMergeFlag          ( UInt uiIdx, Bool b )    { m_pbMergeFlag[uiIdx] = b;           }
  Void          setMergeFlagSubParts  ( Bool bMergeFlag, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );

  UChar*        getMergeIndex         ()                        { return m_puhMergeIndex;                         }
  UChar         getMergeIndex         ( UInt uiIdx )            { return m_puhMergeIndex[uiIdx];                  }
  Void          setMergeIndex         ( UInt uiIdx, UInt uiMergeIndex ) { m_puhMergeIndex[uiIdx] = uiMergeIndex;  }
  Void          setMergeIndexSubParts ( UInt uiMergeIndex, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  template <typename T>
  Void          setSubPart            ( T bParameter, T* pbBaseCtu, UInt uiCUAddr, UInt uiCUDepth, UInt uiPUIdx );

#if AMP_MRG
  Void          setMergeAMP( Bool b )      { m_bIsMergeAMP = b; }
  Bool          getMergeAMP( )             { return m_bIsMergeAMP; }
#endif

  UChar*        getIntraDir         ( const ChannelType channelType )                   const { return m_puhIntraDir[channelType];         }
  UChar         getIntraDir         ( const ChannelType channelType, const UInt uiIdx ) const { return m_puhIntraDir[channelType][uiIdx];  }

  Void          setIntraDirSubParts ( const ChannelType channelType,
                                      const UInt uiDir,
                                      const UInt uiAbsPartIdx,
                                      const UInt uiDepth );

  UChar*        getInterDir           ()                        { return m_puhInterDir;               }
  UChar         getInterDir           ( UInt uiIdx )            { return m_puhInterDir[uiIdx];        }
  Void          setInterDir           ( UInt uiIdx, UChar  uh ) { m_puhInterDir[uiIdx] = uh;          }
  Void          setInterDirSubParts   ( UInt uiDir,  UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Bool*         getIPCMFlag           ()                        { return m_pbIPCMFlag;               }
  Bool          getIPCMFlag           (UInt uiIdx )             { return m_pbIPCMFlag[uiIdx];        }
  Void          setIPCMFlag           (UInt uiIdx, Bool b )     { m_pbIPCMFlag[uiIdx] = b;           }
  Void          setIPCMFlagSubParts   (Bool bIpcmFlag, UInt uiAbsPartIdx, UInt uiDepth);

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for accessing partition information
  // -------------------------------------------------------------------------------------------------------------------

  Void          getPartIndexAndSize   ( UInt uiPartIdx, UInt& ruiPartAddr, Int& riWidth, Int& riHeight ); // This is for use by a leaf/sub CU object only, with no additional AbsPartIdx
  UChar         getNumPartitions      ( const UInt uiAbsPartIdx = 0 );
  Bool          isFirstAbsZorderIdxInDepth (UInt uiAbsPartIdx, UInt uiDepth);

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for motion vector
  // -------------------------------------------------------------------------------------------------------------------

  Void          getMvField            ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, TComMvField& rcMvField );

  Void          fillMvpCand           ( UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, AMVPInfo* pInfo );
  Bool          isDiffMER             ( Int xN, Int yN, Int xP, Int yP);
  Void          getPartPosition       ( UInt partIdx, Int& xP, Int& yP, Int& nPSW, Int& nPSH);

  Void          setMVPIdx             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPIdx)  { m_apiMVPIdx[eRefPicList][uiIdx] = iMVPIdx;  }
  Int           getMVPIdx             ( RefPicList eRefPicList, UInt uiIdx)               { return m_apiMVPIdx[eRefPicList][uiIdx];     }
  Char*         getMVPIdx             ( RefPicList eRefPicList )                          { return m_apiMVPIdx[eRefPicList];            }

  Void          setMVPNum             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPNum ) { m_apiMVPNum[eRefPicList][uiIdx] = iMVPNum;  }
  Int           getMVPNum             ( RefPicList eRefPicList, UInt uiIdx )              { return m_apiMVPNum[eRefPicList][uiIdx];     }
  Char*         getMVPNum             ( RefPicList eRefPicList )                          { return m_apiMVPNum[eRefPicList];            }

  Void          setMVPIdxSubParts     ( Int iMVPIdx, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Void          setMVPNumSubParts     ( Int iMVPNum, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );

  Void          clipMv                ( TComMv&     rcMv     );
  Void          getMvPredLeft         ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldA.getMv(); }
  Void          getMvPredAbove        ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldB.getMv(); }
  Void          getMvPredAboveRight   ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldC.getMv(); }

  Void          compressMV            ();

  // -------------------------------------------------------------------------------------------------------------------
  // utility functions for neighbouring information
  // -------------------------------------------------------------------------------------------------------------------

  TComDataCU*   getCtuLeft                  () { return m_pCtuLeft;       }
  TComDataCU*   getCtuAbove                 () { return m_pCtuAbove;      }
  TComDataCU*   getCtuAboveLeft             () { return m_pCtuAboveLeft;  }
  TComDataCU*   getCtuAboveRight            () { return m_pCtuAboveRight; }
  TComDataCU*   getCUColocated              ( RefPicList eRefPicList ) { return m_apcCUColocated[eRefPicList]; }
  Bool          CUIsFromSameSlice           ( const TComDataCU *pCU /* Can be NULL */) const { return ( pCU!=NULL && pCU->getSlice()->getSliceCurStartCtuTsAddr() == getSlice()->getSliceCurStartCtuTsAddr() ); }
  Bool          CUIsFromSameTile            ( const TComDataCU *pCU /* Can be NULL */) const;
  Bool          CUIsFromSameSliceAndTile    ( const TComDataCU *pCU /* Can be NULL */) const;
  Bool          CUIsFromSameSliceTileAndWavefrontRow( const TComDataCU *pCU /* Can be NULL */) const;
  Bool          isLastSubCUOfCtu(const UInt absPartIdx);


  TComDataCU*   getPULeft                   ( UInt&  uiLPartUnitIdx,
                                              UInt uiCurrPartUnitIdx,
                                              Bool bEnforceSliceRestriction=true,
                                              Bool bEnforceTileRestriction=true );
  TComDataCU*   getPUAbove                  ( UInt&  uiAPartUnitIdx,
                                              UInt uiCurrPartUnitIdx,
                                              Bool bEnforceSliceRestriction=true,
                                              Bool planarAtCTUBoundary = false,
                                              Bool bEnforceTileRestriction=true );
  TComDataCU*   getPUAboveLeft              ( UInt&  uiALPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction=true );

  TComDataCU*   getQpMinCuLeft              ( UInt&  uiLPartUnitIdx , UInt uiCurrAbsIdxInCtu );
  TComDataCU*   getQpMinCuAbove             ( UInt&  uiAPartUnitIdx , UInt uiCurrAbsIdxInCtu );
  Char          getRefQP                    ( UInt   uiCurrAbsIdxInCtu                       );

  /// returns CU and part index of the PU above the top row of the current uiCurrPartUnitIdx of the CU, at a horizontal offset (to the right) of uiPartUnitOffset (in parts)
  TComDataCU*   getPUAboveRight             ( UInt&  uiARPartUnitIdx, UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset = 1, Bool bEnforceSliceRestriction=true );
  /// returns CU and part index of the PU left of the lefthand column of the current uiCurrPartUnitIdx of the CU, at a vertical offset (below) of uiPartUnitOffset (in parts)
  TComDataCU*   getPUBelowLeft              ( UInt&  uiBLPartUnitIdx, UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset = 1, Bool bEnforceSliceRestriction=true );

  Void          deriveLeftRightTopIdx       ( UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT );
  Void          deriveLeftBottomIdx         ( UInt uiPartIdx, UInt& ruiPartIdxLB );

  Bool          hasEqualMotion              ( UInt uiAbsPartIdx, TComDataCU* pcCandCU, UInt uiCandAbsPartIdx );
  Void          getInterMergeCandidates       ( UInt uiAbsPartIdx, UInt uiPUIdx, TComMvField* pcMFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand, Int mrgCandIdx = -1 );

  Void          deriveLeftRightTopIdxGeneral  ( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT );
  Void          deriveLeftBottomIdxGeneral    ( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLB );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for modes
  // -------------------------------------------------------------------------------------------------------------------

  Bool          isIntra            ( UInt uiPartIdx )  const { return m_pePredMode[ uiPartIdx ] == MODE_INTRA;                                              }
  Bool          isInter            ( UInt uiPartIdx )  const { return m_pePredMode[ uiPartIdx ] == MODE_INTER;                                              }
  Bool          isSkipped          ( UInt uiPartIdx );                                                     ///< returns true, if the partiton is skipped
  Bool          isBipredRestriction( UInt puIdx );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for symbol prediction (most probable / mode conversion)
  // -------------------------------------------------------------------------------------------------------------------

  UInt          getIntraSizeIdx                 ( UInt uiAbsPartIdx                                       );

  Void          getAllowedChromaDir             ( UInt uiAbsPartIdx, UInt* uiModeList );
  Void          getIntraDirPredictor            ( UInt uiAbsPartIdx, Int uiIntraDirPred[NUM_MOST_PROBABLE_MODES], const ComponentID compID, Int* piMode = NULL );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for SBAC context
  // -------------------------------------------------------------------------------------------------------------------

  UInt          getCtxSplitFlag                 ( UInt   uiAbsPartIdx, UInt uiDepth                   );
  UInt          getCtxQtCbf                     ( TComTU &rTu, const ChannelType chType );

  UInt          getCtxSkipFlag                  ( UInt   uiAbsPartIdx                                 );
  UInt          getCtxInterDir                  ( UInt   uiAbsPartIdx                                 );

  UInt&         getTotalBins            ()                            { return m_uiTotalBins;                              }
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for RD cost storage
  // -------------------------------------------------------------------------------------------------------------------

  Double&       getTotalCost()                  { return m_dTotalCost;        }
  Distortion&   getTotalDistortion()            { return m_uiTotalDistortion; }
  UInt&         getTotalBits()                  { return m_uiTotalBits;       }
  UInt&         getTotalNumPart()               { return m_uiNumPartition;    }

  UInt          getCoefScanIdx(const UInt uiAbsPartIdx, const UInt uiWidth, const UInt uiHeight, const ComponentID compID) const ;

};

namespace RasterAddress
{
  /** Check whether 2 addresses point to the same column
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualCol( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    // addrA % numUnitsPerRow == addrB % numUnitsPerRow
    return (( addrA ^ addrB ) &  ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether 2 addresses point to the same row
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualRow( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    // addrA / numUnitsPerRow == addrB / numUnitsPerRow
    return (( addrA ^ addrB ) &~ ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether 2 addresses point to the same row or column
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualRowOrCol( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    return isEqualCol( addrA, addrB, numUnitsPerRow ) | isEqualRow( addrA, addrB, numUnitsPerRow );
  }

  /** Check whether one address points to the first column
   * \param addr           Address in raster scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isZeroCol( Int addr, Int numUnitsPerRow )
  {
    // addr % numUnitsPerRow == 0
    return ( addr & ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether one address points to the first row
   * \param addr           Address in raster scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isZeroRow( Int addr, Int numUnitsPerRow )
  {
    // addr / numUnitsPerRow == 0
    return ( addr &~ ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether one address points to a column whose index is smaller than a given value
   * \param addr           Address in raster scan order
   * \param val            Given column index value
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool lessThanCol( Int addr, Int val, Int numUnitsPerRow )
  {
    // addr % numUnitsPerRow < val
    return ( addr & ( numUnitsPerRow - 1 ) ) < val;
  }

  /** Check whether one address points to a row whose index is smaller than a given value
   * \param addr           Address in raster scan order
   * \param val            Given row index value
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool lessThanRow( Int addr, Int val, Int numUnitsPerRow )
  {
    // addr / numUnitsPerRow < val
    return addr < val * numUnitsPerRow;
  }
}

//! \}

#endif
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

/** \file     TComDataCU.h
    \brief    CU data structure (header)
    \todo     not all entities are documented
*/

#ifndef _TCOMDATACU_
#define _TCOMDATACU_

#include <assert.h>

// Include files
#include "CommonDef.h"
#include "TComMotionInfo.h"
#include "TComSlice.h"
#include "TComRdCost.h"
#include "TComPattern.h"

#ifdef DCM_PBIC 
#include "TComICInfo.h"
#include "TComZeroTree.h"
#endif

#if HHI_IMVP || HHI_RQT
#include <algorithm>
#include <vector>
#endif

// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if HHI_IMVP
class MvPredMeasure
{
public:
  TComMv getMVPred( const Int iMvCompX, const Int iMvCompY )  const
  {
    const Int iSize     = Int( m_aiYThreshold.size() )-1;
    Int       iXOrigin  = m_aiXOrigin [ iSize ];
    for(  int iLstIdx   = 0;  iLstIdx < iSize; iLstIdx++ )
    {
      if( iMvCompY <= m_aiYThreshold[ iLstIdx ] )
      {
        iXOrigin = m_aiXOrigin   [ iLstIdx ];
        break;
      }
    }
    TComMv MVPred;
    MVPred.setVer(m_iYOrigin);
    MVPred.setHor(iXOrigin);
    return MVPred;
  } 

  Int                       m_iYOrigin;
  std::vector<Int>          m_aiYThreshold;
  std::vector<Int>          m_aiXOrigin;
};

#endif

/// CU data structure class
class TComDataCU
{
private:

  // -------------------------------------------------------------------------------------------------------------------
  // class pointers
  // -------------------------------------------------------------------------------------------------------------------

  TComPic*      m_pcPic;              ///< picture class pointer
  TComSlice*    m_pcSlice;            ///< slice header pointer
  TComPattern*  m_pcPattern;          ///< neighbour access class pointer

  // -------------------------------------------------------------------------------------------------------------------
  // CU description
  // -------------------------------------------------------------------------------------------------------------------

  UInt          m_uiCUAddr;           ///< CU address in a slice
  UInt          m_uiAbsIdxInLCU;      ///< absolute address in a CU. It's Z scan order
  UInt          m_uiCUPelX;           ///< CU position in a pixel (X)
  UInt          m_uiCUPelY;           ///< CU position in a pixel (Y)
  UInt          m_uiNumPartition;     ///< total number of minimum partitions in a CU
  UChar*        m_puhWidth;           ///< array of widths
  UChar*        m_puhHeight;          ///< array of heights
  UChar*        m_puhDepth;           ///< array of depths

  // -------------------------------------------------------------------------------------------------------------------
  // CU data
  // -------------------------------------------------------------------------------------------------------------------

  PartSize*     m_pePartSize;         ///< array of partition sizes
  PredMode*     m_pePredMode;         ///< array of prediction modes
  UChar*        m_phQP;               ///< array of QP values
  UChar*        m_puhTrIdx;           ///< array of transform indices
  UChar*        m_puhCbf[3];          ///< array of coded block flags (CBF)
  TComCUMvField m_acCUMvField[2];     ///< array of motion vectors
  TCoeff*       m_pcTrCoeffY;         ///< transformed coefficient buffer (Y)
  TCoeff*       m_pcTrCoeffCb;        ///< transformed coefficient buffer (Cb)
  TCoeff*       m_pcTrCoeffCr;        ///< transformed coefficient buffer (Cr)

#ifdef DCM_PBIC
  TComCUIcField m_acCUIcField;        ///< array of IC parameter vectors
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // neighbour access variables
  // -------------------------------------------------------------------------------------------------------------------

  TComDataCU*   m_pcCUAboveLeft;      ///< pointer of above-left CU
  TComDataCU*   m_pcCUAboveRight;     ///< pointer of above-right CU
  TComDataCU*   m_pcCUAbove;          ///< pointer of above CU
  TComDataCU*   m_pcCULeft;           ///< pointer of left CU
  TComDataCU*   m_apcCUColocated[2];  ///< pointer of temporally colocated CU's for both directions
  TComMvField   m_cMvFieldA;          ///< motion vector of position A
  TComMvField   m_cMvFieldB;          ///< motion vector of position B
  TComMvField   m_cMvFieldC;          ///< motion vector of position C
  TComMv        m_cMvPred;            ///< motion vector predictor

  // -------------------------------------------------------------------------------------------------------------------
  // coding tool information
  // -------------------------------------------------------------------------------------------------------------------

#if HHI_MRG
  Bool*         m_pbMergeFlag;        ///< array of merge flags
  UChar*        m_puhMergeIndex;      ///< array of merge candidate indices
#endif
  UChar*        m_puhLumaIntraDir;    ///< array of intra directions (luma)
#if HHI_AIS
  Bool*         m_pbLumaIntraFiltFlag;///< BB: array of intra smoothing flags (luma)
#endif
  UChar*        m_puhChromaIntraDir;  ///< array of intra directions (chroma)
  UChar*        m_puhInterDir;        ///< array of inter directions
  Int*          m_apiMVPIdx[2];       ///< array of motion vector predictor candidates
  Int*          m_apiMVPNum[2];       ///< array of number of possible motion vectors predictors
  UChar*        m_pROTindex;          ///< array of ROT indices
  UChar*        m_pCIPflag;           ///< array of CIP flags
  UInt*         m_puiAlfCtrlFlag;     ///< array of ALF flags
  UInt*         m_puiTmpAlfCtrlFlag;  ///< temporal array of ALF flags
#if PLANAR_INTRA
  Int*          m_piPlanarInfo[4];
#endif

#ifdef DCM_PBIC
  Int*			m_piICPIdx;		///< array of IC predictor candidates
  Int*			m_piICPNum;		///< array of number of possible IC predictor candidates
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // misc. variables
  // -------------------------------------------------------------------------------------------------------------------

  Bool          m_bDecSubCu;          ///< indicates decoder-mode
  Double        m_dTotalCost;         ///< sum of partition RD costs
  UInt          m_uiTotalDistortion;  ///< sum of partition distortion
  UInt          m_uiTotalBits;        ///< sum of partition bits

protected:

  /// add possible motion vector predictor candidates
  Bool          xAddMVPCand           ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );

  /// remove redundant candidates
  Void          xUniqueMVPCand        ( AMVPInfo* pInfo );
#ifdef QC_AMVRES
#if HHI_IMVP
  Void          xUniqueMVPCand_one_fourth(AMVPInfo* pInfo, Int uiPartIdx);
#else
  Void          xUniqueMVPCand_one_fourth(AMVPInfo* pInfo);
#endif
#endif
  /// compute required bits to encode MVD (used in AMVP)
  UInt          xGetMvdBits           ( TComMv cMvd );
  UInt          xGetComponentBits     ( Int iVal );

#ifdef DCM_PBIC
  /// add possible IC predictor candidates
  Bool          xAddICPCand           ( AICPInfo* pInfo, UChar uhInterDir, Int* piRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );

  /// remove redundant candidates
  Void          xUniqueICPCand        ( AICPInfo* pInfo );

  /// compute required bits to encode ICD (used in AICP)
  UInt			    xGetIcdBits			      ( TComIc cIcd );
#endif

  /// compute scaling factor from POC difference
  Int           xGetDistScaleFactor   ( Int iCurrPOC, Int iCurrRefPOC, Int iColPOC, Int iColRefPOC );

  /// calculate all CBF's from coefficients
  Void          xCalcCuCbf            ( UChar* puhCbf, UInt uiTrDepth, UInt uiCbfDepth, UInt uiCuDepth );

public:
  TComDataCU();
  virtual ~TComDataCU();

  // -------------------------------------------------------------------------------------------------------------------
  // create / destroy / initialize / copy
  // -------------------------------------------------------------------------------------------------------------------

  Void          create                ( UInt uiNumPartition, UInt uiWidth, UInt uiHeight, Bool bDecSubCu );
  Void          destroy               ();

  Void          initCU                ( TComPic* pcPic, UInt uiCUAddr );
  Void          initEstData           ();
  Void          initSubCU             ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth );

  Void          copySubCU             ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth );
  Void          copyInterPredInfoFrom ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList );
  Void          copyInterPredInfoTo   ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList );
  Void          copyPartFrom          ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth );

  Void          copyToPic             ( UChar uiDepth );
  Void          copyToPic             ( UChar uiDepth, UInt uiPartIdx, UInt uiPartDepth );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for CU description
  // -------------------------------------------------------------------------------------------------------------------

  TComPic*      getPic                ()                        { return m_pcPic;           }
  TComSlice*    getSlice              ()                        { return m_pcSlice;         }
  UInt&         getAddr               ()                        { return m_uiCUAddr;        }
  UInt&         getZorderIdxInCU      ()                        { return m_uiAbsIdxInLCU; }
  UInt          getCUPelX             ()                        { return m_uiCUPelX;        }
  UInt          getCUPelY             ()                        { return m_uiCUPelY;        }
  TComPattern*  getPattern            ()                        { return m_pcPattern;       }

  UChar*        getDepth              ()                        { return m_puhDepth;        }
  UChar         getDepth              ( UInt uiIdx )            { return m_puhDepth[uiIdx]; }
  Void          setDepth              ( UInt uiIdx, UChar  uh ) { m_puhDepth[uiIdx] = uh;   }

  Void          setDepthSubParts      ( UInt uiDepth, UInt uiAbsPartIdx );
  Void          setIsDecoderCU        ( Bool bIsDecoder )       { m_bDecSubCu = bIsDecoder; }

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for CU data
  // -------------------------------------------------------------------------------------------------------------------

  PartSize*     getPartitionSize      ()                        { return m_pePartSize;        }
  PartSize      getPartitionSize      ( UInt uiIdx )            { return m_pePartSize[uiIdx]; }
  Void          setPartitionSize      ( UInt uiIdx, PartSize uh){ m_pePartSize[uiIdx] = uh;   }
  Void          setPartSizeSubParts   ( PartSize eMode, UInt uiAbsPartIdx, UInt uiDepth );

  PredMode*     getPredictionMode     ()                        { return m_pePredMode;        }
  PredMode      getPredictionMode     ( UInt uiIdx )            { return m_pePredMode[uiIdx]; }
  Void          setPredictionMode     ( UInt uiIdx, PredMode uh){ m_pePredMode[uiIdx] = uh;   }
  Void          setPredModeSubParts   ( PredMode eMode, UInt uiAbsPartIdx, UInt uiDepth );

  UChar*        getWidth              ()                        { return m_puhWidth;          }
  UChar         getWidth              ( UInt uiIdx )            { return m_puhWidth[uiIdx];   }
  Void          setWidth              ( UInt uiIdx, UChar  uh ) { m_puhWidth[uiIdx] = uh;     }

  UChar*        getHeight             ()                        { return m_puhHeight;         }
  UChar         getHeight             ( UInt uiIdx )            { return m_puhHeight[uiIdx];  }
  Void          setHeight             ( UInt uiIdx, UChar  uh ) { m_puhHeight[uiIdx] = uh;    }

  Void          setSizeSubParts       ( UInt uiWidth, UInt uiHeight, UInt uiAbsPartIdx, UInt uiDepth );

  UChar*        getQP                 ()                        { return m_phQP;              }
  UChar         getQP                 ( UInt uiIdx )            { return m_phQP[uiIdx];       }
  Void          setQP                 ( UInt uiIdx, UChar  uh ) { m_phQP[uiIdx] = uh;         }
  Void          setQPSubParts         ( UInt uiQP,   UInt uiAbsPartIdx, UInt uiDepth );

  UChar*        getTransformIdx       ()                        { return m_puhTrIdx;          }
  UChar         getTransformIdx       ( UInt uiIdx )            { return m_puhTrIdx[uiIdx];   }
  Void          setTransformIdx       ( UInt uiIdx, UChar  uh ) { m_puhTrIdx[uiIdx] = uh;     }
  Void          setTrIdxSubParts      ( UInt uiTrIdx, UInt uiAbsPartIdx, UInt uiDepth );

#if HHI_RQT_DEPTH || HHI_RQT_DISABLE_SUB
  UInt          getQuadtreeTULog2MinSizeInCU( UInt uiIdx );
#endif
#if HHI_RQT_FORCE_SPLIT_ACC2_PU || HHI_RQT_DISABLE_SUB
  UInt          getQuadtreeTULog2RootSizeInCU( UInt uiIdx );	
#endif

  TComCUMvField* getCUMvField         ( RefPicList e )          { return  &m_acCUMvField[e];  }

#ifdef DCM_PBIC 
  TComCUIcField* getCUIcField         ()                        { return  &m_acCUIcField;  }
#endif

  TCoeff*&      getCoeffY             ()                        { return m_pcTrCoeffY;        }
  TCoeff*&      getCoeffCb            ()                        { return m_pcTrCoeffCb;       }
  TCoeff*&      getCoeffCr            ()                        { return m_pcTrCoeffCr;       }

  UChar         getCbf    ( UInt uiIdx, TextType eType )                  { return m_puhCbf[g_aucConvertTxtTypeToIdx[eType]][uiIdx];  }
  UChar*        getCbf    ( TextType eType )                              { return m_puhCbf[g_aucConvertTxtTypeToIdx[eType]];         }
  UChar         getCbf    ( UInt uiIdx, TextType eType, UInt uiTrDepth )  { return ( ( getCbf( uiIdx, eType ) >> uiTrDepth ) & 0x1 ); }
  Void          setCbf    ( UInt uiIdx, TextType eType, UChar uh )        { m_puhCbf[g_aucConvertTxtTypeToIdx[eType]][uiIdx] = uh;    }
  Void          clearCbf  ( UInt uiIdx, TextType eType, UInt uiNumParts );
#if HHI_RQT_ROOT
  UChar         getQtRootCbf          ( UInt uiIdx )                      { return getCbf( uiIdx, TEXT_LUMA, 0 ) || getCbf( uiIdx, TEXT_CHROMA_U, 0 ) || getCbf( uiIdx, TEXT_CHROMA_V, 0 ); }
#endif

  Void          setCuCbfLuma          ( UInt uiAbsPartIdx, UInt uiLumaTrMode, UInt uiPartDepth = 0                      );
  Void          setCuCbfChroma        ( UInt uiAbsPartIdx, UInt uiChromaTrMode, UInt uiPartDepth = 0                    );
  Void          setCuCbfChromaUV      ( UInt uiAbsPartIdx, UInt uiChromaTrMode, TextType eTxt,  UInt uiPartDepth = 0    );
  Void          setCbfSubParts        ( UInt uiCbfY, UInt uiCbfU, UInt uiCbfV, UInt uiAbsPartIdx, UInt uiDepth          );
  Void          setCbfSubParts        ( UInt uiCbf, TextType eTType, UInt uiAbsPartIdx, UInt uiDepth                    );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for coding tool information
  // -------------------------------------------------------------------------------------------------------------------

#if HHI_MRG
  Bool*         getMergeFlag          ()                        { return m_pbMergeFlag;               }
  Bool          getMergeFlag          ( UInt uiIdx )            { return m_pbMergeFlag[uiIdx];        }
  Void          setMergeFlag          ( UInt uiIdx, Bool b )    { m_pbMergeFlag[uiIdx] = b;           }
#if HHI_MRG_PU
  Void          setMergeFlagSubParts  ( Bool bMergeFlag, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
#else
  Void          setMergeFlagSubParts  ( Bool bMergeFlag, UInt uiAbsPartIdx, UInt uiDepth );
#endif
  UChar*        getMergeIndex         ()                        { return m_puhMergeIndex;                         }
  UChar         getMergeIndex         ( UInt uiIdx )            { return m_puhMergeIndex[uiIdx];                  }
  Void          setMergeIndex         ( UInt uiIdx, UInt uiMergeIndex ) { m_puhMergeIndex[uiIdx] = uiMergeIndex;  }
#if HHI_MRG_PU
  Void          setMergeIndexSubParts ( UInt uiMergeIndex, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Void          setSubPartBool        ( Bool bParameter, Bool* pbBaseLCU, UInt uiCUAddr, UInt uiCUDepth, UInt uiPUIdx );
  Void          setSubPartUChar       ( UInt bParameter, UChar* pbBaseLCU, UInt uiCUAddr, UInt uiCUDepth, UInt uiPUIdx );
#else
  Void          setMergeIndexSubParts ( UInt uiMergeIndex, UInt uiAbsPartIdx, UInt uiDepth );
#endif
#endif

  UChar*        getLumaIntraDir       ()                        { return m_puhLumaIntraDir;           }
  UChar         getLumaIntraDir       ( UInt uiIdx )            { return m_puhLumaIntraDir[uiIdx];    }
  Void          setLumaIntraDir       ( UInt uiIdx, UChar  uh ) { m_puhLumaIntraDir[uiIdx] = uh;      }
  Void          setLumaIntraDirSubParts( UInt uiDir,  UInt uiAbsPartIdx, UInt uiDepth );

#if HHI_AIS
  Bool*         getLumaIntraFiltFlag  ()                        { return m_pbLumaIntraFiltFlag;       }
  Bool          getLumaIntraFiltFlag  ( UInt uiIdx )            { return m_pbLumaIntraFiltFlag[uiIdx];}
  Void          setLumaIntraFiltFlag  ( UInt uiIdx, Bool b )    { m_pbLumaIntraFiltFlag[uiIdx] = b;   }
  Void          setLumaIntraFiltFlagSubParts( Bool bFlag, UInt uiAbsPartIdx, UInt uiDepth );
#endif
  UChar*        getChromaIntraDir     ()                        { return m_puhChromaIntraDir;         }
  UChar         getChromaIntraDir     ( UInt uiIdx )            { return m_puhChromaIntraDir[uiIdx];  }
  Void          setChromaIntraDir     ( UInt uiIdx, UChar  uh ) { m_puhChromaIntraDir[uiIdx] = uh;    }
  Void          setChromIntraDirSubParts( UInt uiDir,  UInt uiAbsPartIdx, UInt uiDepth );

  UChar*        getInterDir           ()                        { return m_puhInterDir;               }
  UChar         getInterDir           ( UInt uiIdx )            { return m_puhInterDir[uiIdx];        }
  Void          setInterDir           ( UInt uiIdx, UChar  uh ) { m_puhInterDir[uiIdx] = uh;          }
  Void          setInterDirSubParts   ( UInt uiDir,  UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );

  UChar*        getROTindex           ()                        { return m_pROTindex;                 }
  UChar         getROTindex           ( UInt uiIdx )            { return m_pROTindex[uiIdx];          }
  Void          setROTindex           ( UInt uiIdx, UChar  uh ) { m_pROTindex[uiIdx] = uh;            }
  Void          setROTindexSubParts   ( UInt ROTindex, UInt uiAbsPartIdx, UInt uiDepth );

  UInt*         getAlfCtrlFlag        ()                        { return m_puiAlfCtrlFlag;            }
  UInt          getAlfCtrlFlag        ( UInt uiIdx )            { return m_puiAlfCtrlFlag[uiIdx];     }
  Void          setAlfCtrlFlag        ( UInt uiIdx, UInt uiFlag){ m_puiAlfCtrlFlag[uiIdx] = uiFlag;   }
  Void          setAlfCtrlFlagSubParts( UInt uiFlag, UInt uiAbsPartIdx, UInt uiDepth );

  Void          createTmpAlfCtrlFlag  ();
  Void          destroyTmpAlfCtrlFlag ();
  Void          copyAlfCtrlFlagToTmp  ();
  Void          copyAlfCtrlFlagFromTmp();

  UChar*        getCIPflag            ()                        { return m_pCIPflag;                  }
  UChar         getCIPflag            ( UInt uiIdx )            { return m_pCIPflag[uiIdx];           }
  Void          setCIPflag            ( UInt uiIdx, UChar  uh ) { m_pCIPflag[uiIdx] = uh;             }
  Void          setCIPflagSubParts    ( UChar CIPflag, UInt uiAbsPartIdx, UInt uiDepth );

#if ANG_INTRA
  Bool          angIntraEnabledPredPart( UInt uiAbsPartIdx );
#endif

#if PLANAR_INTRA
  Int           getPlanarInfo   ( UInt uiIdx, PlanarType pType )            { return m_piPlanarInfo[pType][uiIdx];  }
  Int*          getPlanarInfo   ( PlanarType pType )                        { return m_piPlanarInfo[pType];         }
  Void          setPlanarInfo   ( UInt uiIdx, PlanarType pType, Int i )     { m_piPlanarInfo[pType][uiIdx] = i;     }
  Void          setPlanarInfoSubParts( Int iPlanarFlag, Int iPlanarDeltaY, Int iPlanarDeltaU, Int iPlanarDeltaV, UInt uiAbsPartIdx, UInt uiDepth );
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for accessing partition information
  // -------------------------------------------------------------------------------------------------------------------

  Void          getPartIndexAndSize   ( UInt uiPartIdx, UInt& ruiPartAddr, Int& riWidth, Int& riHeight );
  UChar         getNumPartInter       ();
  Bool          isFirstAbsZorderIdxInDepth (UInt uiAbsPartIdx, UInt uiDepth);

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for motion vector
  // -------------------------------------------------------------------------------------------------------------------

  Void          getMvField            ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, TComMvField& rcMvField );
  
#if HHI_IMVP
  Void          getYThresXPredLists   ( UInt uiPartIdx, RefPicList  eRefPicList, Int iRefIdx, Int& riYPred, std::vector<Int>& racYThresList, std::vector<Int>& racXPredList );
  Void          getMvPredCandLstStd   ( RefPicList eRefPicList, UInt uiPartIdx, Int iRefIdx, std::vector<TComMv>& rcMvPredCandLst );
  Void          getMvPredCandLstExt   ( RefPicList eRefPicList,  UInt uiPartIdx, Int iRefIdx, std::vector<TComMv>& rcMvPredCandLst );
  Void          setYThresXPredLists   ( std::vector<TComMv>& rcMvPredCandLst, std::vector<Int>& rcYThresList, std::vector<Int>& rcXPredList );
  Int           getMvPredYCompInd     ( std::vector<TComMv>& rcMvPredCandLst );
  Bool          insertNeighbourMvs    ( RefPicList eRefPicList, Int iRefIdx, TComDataCU* pcCUNeighbour, UInt uiIdxNeighbour, std::vector <TComMv>& racNeighbourMvs, Bool bAvoidMultipleInsertion );
  Void          getMvPredYInd         (  RefPicList eRefPicList, UInt uiPartIdx, Int iRefIdx, Int& riMvPredY );
  Void          getMvPredXDep         ( RefPicList eRefPicList, UInt uiPartIdx, Int iRefIdx, const Int iMvY, Int& riMvPredX );
  Int           getMvPredXCompDep     ( const std::vector<Int>& rcYThresLst, const std::vector<Int>& rcXPredLst, const Int iYComponent );
  Void          insertCollocated      ( RefPicList eRefPicList, Int uiPartIdx, Int iRefIdx, std::vector <TComMv>& racNeighbourMvs);
#endif

  AMVP_MODE     getAMVPMode           ( UInt uiIdx );
  Void          fillMvpCand           ( UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, AMVPInfo* pInfo );
#ifdef QC_AMVRES
#if HHI_IMVP
  Bool          clearMVPCand_one_fourth        ( TComMv cMvd, AMVPInfo* pInfo, RefPicList eRefPicList, UInt uiPartIdx, Int iRefIdx );
#else
  Bool          clearMVPCand_one_fourth        ( TComMv cMvd, AMVPInfo* pInfo);
#endif
  Int           searchMVPIdx_one_fourth(TComMv cMv, AMVPInfo* pInfo);
#endif
#if HHI_IMVP
  Bool          clearMVPCand          ( TComMv cMvd, AMVPInfo* pInfo, RefPicList eRefPicList, UInt uiPartIdx, Int iRefIdx );
#else
  Bool          clearMVPCand          ( TComMv cMvd, AMVPInfo* pInfo );
#endif
  Int           searchMVPIdx          ( TComMv cMv,  AMVPInfo* pInfo );

  Void          setMVPIdx             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPIdx)  { m_apiMVPIdx[eRefPicList][uiIdx] = iMVPIdx;  }
  Int           getMVPIdx             ( RefPicList eRefPicList, UInt uiIdx)               { return m_apiMVPIdx[eRefPicList][uiIdx];     }
  Int*          getMVPIdx             ( RefPicList eRefPicList )                          { return m_apiMVPIdx[eRefPicList];            }

  Void          setMVPNum             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPNum ) { m_apiMVPNum[eRefPicList][uiIdx] = iMVPNum;  }
  Int           getMVPNum             ( RefPicList eRefPicList, UInt uiIdx )              { return m_apiMVPNum[eRefPicList][uiIdx];     }
  Int*          getMVPNum             ( RefPicList eRefPicList )                          { return m_apiMVPNum[eRefPicList];            }

  Void          setMVPIdxSubParts     ( Int iMVPIdx, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Void          setMVPNumSubParts     ( Int iMVPNum, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );

  Void          clipMv                ( TComMv&     rcMv     );
  Void          getMvPredLeft         ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldA.getMv(); }
  Void          getMvPredAbove        ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldB.getMv(); }
  Void          getMvPredAboveRight   ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldC.getMv(); }

#ifdef DCM_PBIC 
// -------------------------------------------------------------------------------------------------------------------
// member functions for IC data
// -------------------------------------------------------------------------------------------------------------------
  Void	getIc		( TComDataCU* pcCU, UInt uiAbsPartIdx, TComIc& rcIc );

  Void	fillICPCand		( UInt uiPartIdx, UInt uiPartAddr, AICPInfo* pInfo );
  Int 	searchICPIdx	( TComIc cIc,  AICPInfo* pInfo );

  Void	setICPIdx	    ( UInt uiIdx, Int iICPIdx)	  { m_piICPIdx[uiIdx] = iICPIdx;	}
  Int	  getICPIdx	    ( UInt uiIdx)  { return m_piICPIdx[uiIdx];		}
  Int*	getICPIdx  () { return m_piICPIdx;				}

  Void	setICPNum	    ( UInt uiIdx, Int iMVPNum )   { m_piICPNum[uiIdx] = iMVPNum;	}
  Int	  getICPNum       ( UInt uiIdx )                { return m_piICPNum[uiIdx];}
  Int*	getICPNum	    ()							  { return m_piICPNum;				}

  Void	setICPIdxSubParts( Int iICPIdx, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Void	setICPNumSubParts( Int iICPNum, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // utility functions for neighbouring information
  // -------------------------------------------------------------------------------------------------------------------

  TComDataCU*   getCULeft                   () { return m_pcCULeft;       }
  TComDataCU*   getCUAbove                  () { return m_pcCUAbove;      }
  TComDataCU*   getCUAboveLeft              () { return m_pcCUAboveLeft;  }
  TComDataCU*   getCUAboveRight             () { return m_pcCUAboveRight; }
  TComDataCU*   getCUColocated              ( RefPicList eRefPicList ) { return m_apcCUColocated[eRefPicList]; }

  TComDataCU*   getPULeft                   ( UInt&  uiLPartUnitIdx , UInt uiCurrPartUnitIdx );
  TComDataCU*   getPUAbove                  ( UInt&  uiAPartUnitIdx , UInt uiCurrPartUnitIdx );
  TComDataCU*   getPUAboveLeft              ( UInt&  uiALPartUnitIdx, UInt uiCurrPartUnitIdx );
  TComDataCU*   getPUAboveRight             ( UInt&  uiARPartUnitIdx, UInt uiCurrPartUnitIdx );

  TComDataCU*   getPUBelowLeft              ( UInt& uiBLPartUnitIdx, UInt uiCurrPartUnitIdx );
  TComDataCU*   getPUAboveRightAdi          ( UInt&  uiARPartUnitIdx, UInt uiPuWidth, UInt uiCurrPartUnitIdx );
  TComDataCU*   getPUBelowLeftAdi           ( UInt& uiBLPartUnitIdx, UInt uiPuHeight, UInt uiCurrPartUnitIdx );

  Void          deriveLeftRightTopIdx       ( PartSize eCUMode, UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT );
  Void          deriveLeftBottomIdx         ( PartSize eCUMode, UInt uiPartIdx, UInt& ruiPartIdxLB );

  Void          deriveLeftRightTopIdxAdi    ( UInt& ruiPartIdxLT, UInt& ruiPartIdxRT, UInt uiPartOffset, UInt uiPartDepth );
  Void          deriveLeftBottomIdxAdi      ( UInt& ruiPartIdxLB, UInt  uiPartOffset, UInt uiPartDepth );

#if HHI_MRG
#ifdef DCM_PBIC
  Void          getInterMergeCandidates     ( UInt uiAbsPartIdx, TComMvField cMFieldNeighbours[4],TComIc cIcNeighbours[2],UChar uhInterDirNeighbours[2], UInt& uiNeighbourInfos );
#else
  Void          getInterMergeCandidates     ( UInt uiAbsPartIdx, TComMvField cMFieldNeighbours[4], UChar uhInterDirNeighbours[2], UInt& uiNeighbourInfos );
#endif
#endif


  // -------------------------------------------------------------------------------------------------------------------
  // member functions for modes
  // -------------------------------------------------------------------------------------------------------------------

  Bool          isIntra   ( UInt uiPartIdx )  { return m_pePredMode[ uiPartIdx ] == MODE_INTRA; }
  Bool          isSkip    ( UInt uiPartIdx )  { return m_pePredMode[ uiPartIdx ] == MODE_SKIP;  } ///< SKIP+DIRECT
  Bool          isSkipped ( UInt uiPartIdx );                                                     ///< SKIP (no residual)

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for symbol prediction (most probable / mode conversion)
  // -------------------------------------------------------------------------------------------------------------------

  Int           getMostProbableIntraDirLuma     ( UInt uiAbsPartIdx                                       );
  Int           convertIntraDirLuma             ( UInt uiAbsPartIdx                                       );
  UInt          revertIntraDirLuma              ( UInt uiAbsPartIdx, Int  iIntraDirLuma                   );

  Int           getMostProbableIntraDirLumaAdi  ( UInt uiAbsPartIdx                                       );
  Int           convertIntraDirLumaAdi          ( TComDataCU* pcCU, UInt uiAbsPartIdx                     );
  UInt          revertIntraDirLumaAdi           ( TComDataCU* pcCU, UInt uiAbsPartIdx, Int iIntraDirLuma  );

  PartSize      getMostProbablePartSize         ( UInt uiAbsPartIdx                                       );

  Int           getMostProbablePredMode         ( UInt uiAbsPartIdx                                       );
  Int           convertPredMode                 ( UInt uiAbsPartIdx                                       );
  Int           revertPredMode                  ( UInt uiAbsPartIdx, Int   iPredMode                      );

  UInt          getIntraSizeIdx                 ( UInt uiAbsPartIdx                                       );
  Void          convertTransIdx                 ( UInt uiAbsPartIdx, UInt uiTrIdx, UInt& ruiLumaTrMode, UInt& ruiChromaTrMode );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for SBAC context
  // -------------------------------------------------------------------------------------------------------------------

  UInt          getCtxSplitFlag                 ( UInt   uiAbsPartIdx, UInt uiDepth                   );
  UInt          getCtxCbf                       ( UInt   uiAbsPartIdx, TextType eType, UInt uiTrDepth );
#if HHI_RQT
  UInt          getCtxQtCbf                     ( UInt   uiAbsPartIdx, TextType eType, UInt uiTrDepth );
#if HHI_RQT_ROOT
  UInt          getCtxQtRootCbf                 ( UInt   uiAbsPartIdx                                 );
#endif
#endif
  UInt          getCtxRefIdx                    ( UInt   uiAbsPartIdx, RefPicList eRefPicList         );
  UInt          getCtxSkipFlag                  ( UInt   uiAbsPartIdx                                 );
#if HHI_MRG
  UInt          getCtxMergeFlag                 ( UInt   uiAbsPartIdx                                 );
  UInt          getCtxMergeIndex                ( UInt   uiAbsPartIdx                                 );
#endif
  UInt          getCtxAlfCtrlFlag               ( UInt   uiAbsPartIdx                                 );
  UInt          getCtxPredMode                  ( UInt   uiAbsPartIdx                                 );
  UInt          getCtxInterDir                  ( UInt   uiAbsPartIdx                                 );
  UInt          getCtxTransIdx                  ( UInt   uiAbsPartIdx                                 );
  UInt          getCtxIntraDirChroma            ( UInt   uiAbsPartIdx                                 );
  UInt          getCtxCIPFlag                   ( UInt   uiAbsPartIdx                                 );
#if HHI_AIS
#if ANG_INTRA
  UInt          getCtxIntraFiltFlagLumaAng      ( UInt   uiAbsPartIdx                                 );
#endif
  UInt          getCtxIntraFiltFlagLuma         ( UInt   uiAbsPartIdx                                 );
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for RD cost storage
  // -------------------------------------------------------------------------------------------------------------------

  Double&       getTotalCost()                  { return m_dTotalCost;        }
  UInt&         getTotalDistortion()            { return m_uiTotalDistortion; }
  UInt&         getTotalBits()                  { return m_uiTotalBits;       }
  UInt&         getTotalNumPart()               { return m_uiNumPartition;    }
};

#endif

>>>>>>> upstream/master
