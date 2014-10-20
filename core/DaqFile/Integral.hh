/**
 * \file Integral.hh
 *
 * \ingroup DaqFile
 * 
 * \brief Class def header for a class Integral
 *
 * @author echurch
 */

/** \addtogroup DaqFile

    @{*/
#ifndef INTEGRAL_HH
#define INTEGRAL_HH


#include <fstream>
#include <vector>
#include <memory>

#include <iostream>
#include <stdint.h>

#include "TTimeStamp.h"

#include "KvpSet.h"
#include "Plexus.h"

namespace gov {namespace fnal {namespace uboone {namespace datatypes {
	class eventRecord;
	class crateHeader;
	class cardHeader;
	class crateData;
	class cardData;
	class channelData;
	class crateHeaderPMT;
	class cardHeaderPMT;
	class crateDataPMT;
	class cardDataPMT;
	class channelDataPMT;
      } } } }


//gov::fnal::uboone::datatypes::eventRecord;
//gov::fnal::uboone::datatypes::crateHeader;
//gov::fnal::uboone::datatypes::crateData;
//gov::fnal::uboone::datatypes::cardHeader;
//gov::fnal::uboone::datatypes::cardData;
/**
   \class Integral
   User defined class Integral ... these comments are used to generate
   doxygen documentation!
 */

// Steal merely the integrate(shard_ptr *sp) method from Nathaniel's Integral class and shove it in here.
// Put his implementations of integrateTPC,PMT() into the Integral.cc file.
// Inside integrate() we'll have at our disposal the individual PMT and TPC-wire data. Make integrate()
// write that out to arrays in the python session that called it. OpenCL can go to work on it from there.

// EC, 14-Sep-2014.
  // TPC data collection, in Integral_tpc.cpp
  // Integrate functions all return number of items integrated.

  struct ChannelCount {
    int crates;
    int cards;
    int channels;
    int mapped_channels;
    ChannelCount() :crates(0),cards(0),channels(0), mapped_channels(0) {};
    ChannelCount(int ncrate,int ncard,int nchannel,int nmapped) :crates(ncrate),cards(ncard),channels(nchannel), mapped_channels(nmapped) {};
    ChannelCount& operator+=(const ChannelCount& rhs) { crates+=rhs.crates; cards+=rhs.cards; channels+=rhs.channels; mapped_channels+=rhs.mapped_channels; return *this;}
  };


class Integral{

public:

  /// Default constructor
  Integral();
  Integral (gov::fnal::uboone::dispatcher::KvpSet& k){};
  /// Default destructor
  virtual ~Integral(){};


  //  using namespace gov::fnal::uboone;
  //  using namespace gov::fnal::uboone::datatypes;
  // Top-level things:

  void integrate(gov::fnal::uboone::datatypes::eventRecord& record);
  void integrateHeaders(const gov::fnal::uboone::datatypes::eventRecord& record);
  void checkAllChecksums(const gov::fnal::uboone::datatypes::eventRecord& record);
  TTimeStamp getTimeStamp(const gov::fnal::uboone::datatypes::eventRecord& record);

  void integrateTpcCrate(const gov::fnal::uboone::datatypes::crateHeader& crate_header,
                         gov::fnal::uboone::datatypes::crateData& crate_data);
  ChannelCount integrateTpcCard(int crate,
				const gov::fnal::uboone::datatypes::cardHeader* card_header,
				const gov::fnal::uboone::datatypes::cardData* card_data);
  ChannelCount integrateTpcWire(int crate, int card, int channel, const gov::fnal::uboone::datatypes::channelData& data);
  void getWireCalibration(int crate, int card, int chan, 
			  double& ped, double& pederr,
			  double& pedsig, double& pedsigerr,
			  double& gain, double& gainerr );
  // void getWireFromCrateCardChan(int icrate,int icard,int ichan, int& outWire, int& outPlane);
  void sumTpcCrates();
  void aggregateTpc();
  
  // PMT data collection, in Integral_pmt.cpp
  void integratePmtCrate(const gov::fnal::uboone::datatypes::crateHeader& crate_header,
                         const gov::fnal::uboone::datatypes::crateDataPMT& crate_data
                         );
  ChannelCount integratePmtCard(int crate,
				const gov::fnal::uboone::datatypes::cardHeaderPMT& card_header,
				const gov::fnal::uboone::datatypes::cardDataPMT& card_data );
  ChannelCount integratePmtChannel(int crate, int card, int channel, 
				   const gov::fnal::uboone::datatypes::channelDataPMT& data );
  void getPmtCalibration(int crate, int card, int chan, 
			 double& ped, double& pederr,
			 double& pedsig, double& pedsigerr,
			 double& gain, double& gainerr );
  void  getPmtFromCrateCardChan(int icrate,int icard,int ichan, int& outPmt, int& outGain, std::string& outSpecial);
                        
  void aggregatePmt();




  // TPC data collection, in Integral_tpc.cpp
  // Integrate functions all return number of items integrated.

  // Some things I might need from more than one place...
  
  bool     isPmtTrigger()    const { return (m_trigger_bits & 0xFF)!=0; } 
  bool     isExtTrigger()    const { return (m_trigger_bits & 0x100)!=0;}
  bool     isActiveTrigger() const { return (m_trigger_bits & 0x200)!=0;}
  bool     isBnbTrigger()    const { return (m_trigger_bits & 0x400)!=0;}
  bool     isNumiTrigger()   const { return (m_trigger_bits & 0x800)!=0;}
  bool     isVetoTrigger()   const { return (m_trigger_bits & 0x1000)!=0;}
  bool     isCalibTrigger()  const { return (m_trigger_bits & 0x2000)!=0;}
  

  // Utilities:
  template<typename M> void aggregateCccThing(M mold, bool expandWallClock = false);
  template<typename M> void aggregatePlaneThing(M mold, bool expandWallClock = false);

  void  findPulses(   bool inductionPlane,
                      std::vector<double>& waveform, double thresh, double shapetime,
                      std::vector<double>& outPulseheight,
                      std::vector<double>& outPulsetime  );


  uint32_t          m_file_cycle;

  // Statistics & state:
  //#ifndef __CINT__    
  //std::shared_ptr<gov::fnal::uboone::datatypes::eventRecord> m_current_eventRecord;
  //#endif

  gov::fnal::uboone::datatypes::eventRecord* m_current_eventRecord;

  // General properties of this event:
  uint32_t m_events_processed;
  uint32_t m_run;
  uint32_t m_subrun;
  uint32_t m_event;
  TTimeStamp m_time_of_first_event;
  TTimeStamp m_time_of_cur_event;
  ChannelCount m_channel_count_tpc;
  ChannelCount m_channel_count_pmt;

  // from trigger card:
  uint16_t  m_trigger_bits;
  uint32_t  m_trig_frame;
  uint32_t  m_trig_time_2MHz;
  uint32_t  m_trig_time_16MHz;
  uint32_t  m_trig_time_64MHz;

  // From PMT card header:
  uint32_t  m_pmt_event_frame;
  uint32_t  m_pmt_trig_frame; 
  uint32_t  m_pmt_trig_frame_mod16; 
  uint32_t  m_pmt_trig_sample_2MHz; 
  
  // Time of the start of the readout window, from looking at PMT data:
  uint32_t  m_pmt_readout_frame_mod8; 
  uint32_t  m_pmt_readout_frame;  
  uint32_t  m_pmt_readout_sample;// 16 MHz

  gov::fnal::uboone::dispatcher::KvpSet m_config;
  bool        m_fake_event_times;

  gov::fnal::uboone::online::Plexus m_plexus;

};

#endif
/** @} */ // end of doxygen group 

