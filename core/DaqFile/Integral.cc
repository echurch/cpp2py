#ifndef INTEGRAL_CC
#define INTEGRAL_CC


#include "Integral.hh"
#include "datatypes/eventRecord.h"
#include "datatypes/crateHeader.h"
#include "datatypes/cardDataPMT.h"
#include "datatypes_utilities/card_utilities.h"


#include <sys/file.h>


//#include "FormString.h"


using namespace gov::fnal::uboone::datatypes;


Integral::Integral(KvpSet& config)
  :  m_file(0)
  , m_file_cycle(0)
  , m_events_processed(0)
  , m_run(0)
  , m_subrun(0)
  , m_event(0)
     // Configurables:
  , m_config(config)
  , m_time_between_writes(30.0)
  , m_update_symlink(true)
  , m_update_existing_file(true)
  , m_current_filename_to_symlink("current.root")
  , m_output_directory("./")
  , m_do_threads(1)
  , m_fake_event_times(false)
{
  m_config = config;
  m_time_between_writes = m_config.getDouble("timeBetweenWrites",m_time_between_writes);
  m_output_directory    = m_config.getString("outputDirectory", m_output_directory);
  m_update_symlink      = m_config.getInt("updateSymlink",m_update_symlink);
  m_update_existing_file = m_config.getInt("updateExistingFile",m_update_existing_file);
  m_current_filename_to_symlink = m_config.getString("currentFilenameToSymlink",m_current_filename_to_symlink);
  m_file_cycle       = m_config.getInt("fileCycle",m_file_cycle);
  m_do_threads       = m_config.getInt("do_threads",m_do_threads);
  m_fake_event_times = m_config.getInt("fakeEventTimes",m_fake_event_times);

  // First look at plexusInterface and plexusConnection. If that fails, look at plexusConnection_fallback and plexusConnection_fallback.
  std::string plexSource;    
  std::string plexConnection;
  plexSource     = m_config.getString("plexusInterface_fallback","postgresql");
  plexConnection = m_config.getString("plexusConnection_fallback","host=localhost port=5432");

  if(plexSource=="postgresql") {
    m_plexus.buildFromPostgresql(plexConnection);
    if(!m_plexus.is_ok()) {
      std::cout << "Cannot connect to database using " << plexSource << " and " << plexConnection << std::endl;
    }
  }
  if(!m_plexus.is_ok() && config.has("plexusConnection_fallback")) {
    plexSource     = m_config.getString("plexusInterface_fallback","postgresql");
    plexConnection = m_config.getString("plexusConnection_fallback","host=localhost port=5432");
    if(plexSource=="postgresql") {
      m_plexus.buildFromPostgresql(plexConnection);
      if(!m_plexus.is_ok()) {
	std::cout << "Cannot connect to database using " << plexSource << " and " << plexConnection << std::endl;
      }
    }    
  }

  if(!m_plexus.is_ok()) {
    std::cout << "Reverting to hard-coded connections mapping, since no DB is working." << std::endl;
  }
  m_plexus.buildHardcoded();

}



void Integral::integrate(std::shared_ptr<gov::fnal::uboone::datatypes::eventRecord> record)
{
  if(!record) return; // null pointer
  m_current_eventRecord = record;
  
  dispatcher::Timer timer;
  int run =     record->getGlobalHeader().getRunNumber();
  int subrun =  record->getGlobalHeader().getSubrunNumber();
  m_event =     record->getGlobalHeader().getEventNumber();
  m_time_of_cur_event =  getTimeStamp(record);
  std::cout << "Integrating record run|subrun|event " << run << "|" << subrun << "|"
	    <<record->getGlobalHeader().getEventNumber() << m_time_of_cur_event.AsString() << std::endl;

  m_channel_count_tpc = ChannelCount();
  m_channel_count_pmt = ChannelCount();

  // First question: do we need to set ourselves up?
  if((m_run != (unsigned int)run) || (m_subrun != (unsigned int)subrun) || (m_file==0)) {
    // Finish out last run.

    // Reset everything.
    gDirectory = 0;
    m_last_write_time.Reset();
    m_obj_cache.clear();
    m_events_processed = 0;
    m_run = run;
    m_subrun = subrun;
    m_time_of_first_event = m_time_of_cur_event;
    const char* mode = "UPDATE";
    if(!m_update_existing_file) mode = "RECREATE";
  }
  

  // Do event-level things.
  integrateHeaders(record);

  // histogram to record unpacking errors.
  MInputDataError mi;
  TH1* hErrors = get_or_create<TH1>(mi);

  // have the record unpack itself.
  bool dataGood = true;;

  try {
    record->updateIOMode(IO_GRANULARITY_CARD);      // The business end of things.
  } 
  catch (std::runtime_error& e) {
    // Catch error unpacking crates into cards
    std::cout << "Could not unpack card data on run " << run << " subrun " << subrun << " event " << m_event << std::endl;

    dataGood = false;
  }
  
  if(dataGood){
    try {
      checkAllChecksums(record);
      record->updateIOMode(IO_GRANULARITY_CHANNEL);      // The business end of things.
    } catch (std::runtime_error& e) {
      // Catch error unpacking cards into channels
      std::cout << "Could not unpack channel data on run " << run << " subrun " << subrun << " event " << m_event << std::cout;

      dataGood = false;
    }
  }


  
  
  int ncrates = 0;
  bool have_tpc = false;
  bool have_pmt = false;
  boost::thread_group crate_threads;

  if(dataGood) {  
    // Loop through data.
   
    //get the seb map, and do a loop over all sebs/crates
    eventRecord::sebMap_t& seb_map = record->getSEBMap();
    eventRecord::sebMap_t::iterator seb_it;
    for( seb_it = seb_map.begin(); seb_it != seb_map.end(); seb_it++){
      ncrates++;
      //get the crateHeader/crateData objects
      const crateHeader& crate_header = seb_it->first;
      crateData& crate_data = seb_it->second;
      if(m_do_threads) crate_threads.create_thread(boost::bind(&Integral::integrateTpcCrate,this,crate_header,crate_data));
      else             integrateTpcCrate(crate_header,crate_data);
      have_tpc = true;
    } // loop seb/crate

    
    const eventRecord::sebMapPMT_t& pmt_map = record->getSEBPMTMap();
    eventRecord::sebMapPMT_t::const_iterator pmt_it;
    for( pmt_it = pmt_map.begin(); pmt_it != pmt_map.end(); pmt_it++){
      ncrates++;
      //get the crateHeader/crateData objects
      const crateHeader& crate_header = pmt_it->first;
      const crateDataPMT& crate_data = pmt_it->second;
      if(m_do_threads) crate_threads.create_thread(boost::bind(&Integral::integratePmtCrate,this,crate_header,crate_data));
      else             integratePmtCrate(crate_header,crate_data);
      have_pmt = true;
    } // loop seb/crate

  }
  
  
  
  // OK, now wait for all crates to finish processing.
  crate_threads.join_all();
  m_current_eventRecord.reset();
  

  double t = timer.Count();
  std::cout << "benchmark: Integrated " << m_channel_count_tpc.crates << " TPC crates, "
	  << m_channel_count_pmt.crates << " PMT crates, "
	  << m_channel_count_tpc.mapped_channels << " wires, " 
	  << m_channel_count_pmt.mapped_channels << " pmts in "
	    << timer.Count() << " s" << std::endl;


}

void Integral::checkAllChecksums(std::shared_ptr<datatypes::eventRecord> record)
{
  MChecksumErrors mce;
  TH1* hChecksum = get_or_create<TH1>(mce);
  
  // Should be called when IO mode has been moved to 'card' level.
  // PMT checksum not yet build for 'channel' level, and it's faster this way anyway.
  
  //get the seb map, and do a loop over all sebs/crates
  const eventRecord::sebMap_t& seb_map = record->getSEBMap();
  eventRecord::sebMap_t::const_iterator seb_it;
  for( seb_it = seb_map.begin(); seb_it != seb_map.end(); seb_it++){

    const crateHeader& crate_header = seb_it->first;
    const crateData& crate_data = seb_it->second;

    datatypes::crateData::cardMap_t::const_iterator card_it;
    const datatypes::crateData::cardMap_t& card_map = crate_data.getCardMap();
    for(card_it = card_map.begin(); card_it != card_map.end(); card_it++){
    
     
      long result = utilities::compareCheckSum(card_it->first, card_it->second);
      if(result!=0) {

        
      // logInfo << std::hex << "TPC checksum error " << (int)(crate_header.getCrateNumber()) << "|" <<card_it->first.getModule() << " "
      //   << " header: 0x" << (long)(card_it->first.getChecksum())
      //   << " body: 0x" << (long)(datatypes::utilities::getCheate_data     = pmt_it->second;
      }
    }
  } // loop seb/crate

  const eventRecord::sebMapPMT_t& pmt_map = record->getSEBPMTMap();
  eventRecord::sebMapPMT_t::const_iterator pmt_it;
  for( pmt_it = pmt_map.begin(); pmt_it != pmt_map.end(); pmt_it++){
    //get the crateHeader/crateData objects
    const crateHeader&  crate_header = pmt_it->first;
    const crateDataPMT& crate_data     = pmt_it->second;
    
    datatypes::crateDataPMT::cardMap_t::const_iterator card_it;
    const datatypes::crateDataPMT::cardMap_t& card_map = crate_data.getCardMap();
    for(card_it = card_map.begin(); card_it != card_map.end(); card_it++){
	
      long result = utilities::compareCheckSum(card_it->first, card_it->second);
      if(result!=0) {
	logInfo << std::hex << "PMT checksum error " << (int)(crate_header.getCrateNumber()) << "|" <<card_it->first.getModule() << " " 
		<< " header: 0x" << (long)(card_it->first.getChecksum())
		<< " body: 0x" << (long)(datatypes::utilities::getCheckSumFromDataBlock(card_it->second.getCardDataPtr(),card_it->second.getCardDataSize()))
		<< " diff: 0x" << -result;
      }
    }
  } // loop seb/crate

}


const TTimeStamp kJan_1_2012(2012,1,1,0,0,0,true); // Default date, start of stamping
const TTimeStamp kJan_1_2014(2014,1,1,0,0,0,true); // First date that can be considered valid
                               
TTimeStamp Integral::getTimeStamp(std::shared_ptr<gov::fnal::uboone::datatypes::eventRecord> record)
{
  if(m_fake_event_times) {
    TTimeStamp now;
    std::cout << "Faking event time " << now.AsString() << std::endl;
    return now;
  }
  // Find the first valid time stamp.
  TTimeStamp crateTime = kJan_1_2012;
  TTimeStamp gpsTime   = kJan_1_2012;
  
  
  // Get the time from a TPC crate.
  const eventRecord::sebMap_t& seb_map = record->getSEBMap();
  eventRecord::sebMap_t::const_iterator seb_it;
  for( seb_it = seb_map.begin(); seb_it != seb_map.end(); seb_it++){
    const crateHeader& ch = seb_it->first;
    TTimeStamp t = TTimeStamp(ch.getSebTimeSec(), ch.getSebTimeUsec()*1000 );
    if(t > kJan_1_2014) {
      crateTime = TTimeStamp(ch.getSebTimeSec(), ch.getSebTimeUsec()*1000 );
    }
  }
  // Get the time from the PMT crate.
  const eventRecord::sebMapPMT_t& seb_pmt_map = record->getSEBPMTMap();
  eventRecord::sebMapPMT_t::const_iterator pmt_it;
  for( pmt_it = seb_pmt_map.begin(); pmt_it != seb_pmt_map.end(); pmt_it++){
    const crateHeader& ch = pmt_it->first;
    TTimeStamp t = TTimeStamp(ch.getSebTimeSec(), ch.getSebTimeUsec()*1000 );
    if(t > kJan_1_2014) {
      crateTime = t;
    }
  }
  time_t daqSec = record->getGlobalHeader().getSeconds();
  int daqNanoSec = (record->getGlobalHeader().getMilliSeconds()*1000000)
    + (record->getGlobalHeader().getMicroSeconds()*1000) // FIXME: Not sure if right.
    + (record->getGlobalHeader().getNanoSeconds()); // FIXME: Not sure if right.

  gpsTime = TTimeStamp(daqSec + kJan_1_2012.GetSec(), daqNanoSec);
  // std::cout << "gps time: " << gpsTime.AsString() << std::endl;
  
  double delta = gpsTime - crateTime;
  TH1* htime = get_or_create<TH1>(MGpsTimeVsCrateTime().me());

  
  if(gpsTime.GetSec() > kJan_1_2014.GetSec()) {
    std::cout << "Returning gpsTime=" << gpsTime.AsString() << std::endl;
    return gpsTime;
  } //else

  std::cout << "Returning crateTime=" << crateTime.AsString() << std::endl;
  return crateTime;
}



void Integral::integrateHeaders(std::shared_ptr<gov::fnal::uboone::datatypes::eventRecord> record)
{
  // get trigger card data.
  triggerData* trigger = record->getTriggerDataPtr();
  m_trigger_bits = trigger->getTriggerBits();
  m_trig_frame      = trigger->getFrame();
  m_trig_time_2MHz  = trigger->getSampleNum_trigger_bits & bit) {
  
  m_trig_time_16MHz = trigger->getSampleNumber_16MHz();
  m_trig_time_64MHz = trigger->getSampleNumber_64MHz();
  

  for(int ibit=0;ibit<16;ibit++) {
    uint16_t bit = 1 << ibit;
    if(m_trigger_bits & bit) {
      
      // Correlation matrix.
      for(int jbit=0;jbit<16;jbit++) {
	uint16_t bit2 = 1 << jbit;
	//	  if((bit2!=bit) && (m_trigger_bits & bit2)) h_triggerflagcorr->Fill(ibit,jbit);
      }
    }
  }
}

			       }


void Integral::findPulses(  bool inductionPlane,
			    std::vector<double>& waveform, double thresh, double tau,
			    std::vector<double>& pulseheight,
			    std::vector<double>& pulsetime
			    )
{
  ///
  /// Given a waveform at pedestal ped, find all unipolar pulses above a threshold thresh and
  /// return them as a vector <pulseheight,pulsetime>
  ///
  /// NB changes the input waveform to the the shaped waveform!
  ///
  /// Good shaping times are ~10 ticks: that gives some smoothing, but recovers quicking in about ~3 ticks.
  //
  using std::vector;
  pulseheight.clear();
  pulsetime.clear();
  
  
  size_t nsamp = waveform.size();
  double last = waveform[0];
  waveform[0] = 0;

  // double Blast = 0;
  double B =0;
  //double Clast = 0;
  double Dlast = 0;
  double Dlast2 = 0;
  double tauOverTauPlusOne = tau/(tau+1.0);

  for(size_t j=1;j<nsamp;j++) {
    double val = waveform[j];
    double dA = val-last;
    
    // simulated CR differentiator + RC integrator shaping circuit.
    // The point here is to remove pedestal by differentiating, and integrating again but with smoothing to get a good signal for pulse-height.
    // This function acts as a simulated CR differentiator. This is good for step-function integration, but lousy for this...
    // double B = tau1 * (Blast + dA) /(tau1+1.0);
    // instead, I'll just use a pure differentiation. 

    B = dA;
    if(inductionPlane) {
      B=-B;        // invert: we're looking for negative slopes
      //if(B<0) B=0; // Clamp: we're looking ONLY for negative slopes. 
      // NO! Clamping here will drive the baseline wonky!
    } 
    //else {
    //  B=B;  // Don't invert or clamp: we're looking for positive slopes
    //  }
  
    // Now, simulate a shaping integrator RC circuit with time constant tau2. Feed differential as input.
    // B = tau * dC/dt + C
    // or 
    // B = tau * (C-Clast) + C
    //double C = (B + tau*Clast)/(tau+1.0);

    // Use it. Look for peaks.
    // Then scale up by tau+1
    //double D = C*(tau+1.0);
    double D = B + tauOverTauPlusOne*Dlast;
    
    waveform[j] = D; /// write it back out.
    double dD2 = Dlast-Dlast2;
    double dD = D-Dlast;
    // Find a place where slope goes from positive to negative -that's a peak.
    if((dD2>0) && (dD<=0) && (Dlast>thresh)) {
      pulseheight.push_back(Dlast);
      pulsetime  .push_back(j-1);    
    }
    last = val;
    // Clast = C;
    Dlast2= Dlast;
    Dlast = D;
  }
}

			  
/////////////////////////////////////
// PMTs
/////////////////////////////////////
void Integral::integratePmtCrate(const crateHeader& crate_header,
                                 const crateDataPMT& crate_data)
{
  /// Integrate the PMT data for this event record.
  int crate = crate_header.getCrateNumber();

  //now get the card map (for the current crate), and do a loop over all cards
  const crateDataPMT::cardMap_t&  card_map = crate_data.getCardMap();
  crateDataPMT::cardMap_t::const_iterator card_it;
  
  // Values to persist across calls:
  m_pmt_readout_frame  = 0xFFFFFFFF;
  m_pmt_readout_frame_mod8 = 0xFFFFFFFF;
  m_pmt_readout_sample = 0xFFFFFFFF;
  
  ChannelCount count(1,0,0,0);
  
  std::vector<int> which_cards;
  for(card_it = card_map.begin(); card_it != card_map.end(); card_it++){    
    const cardHeaderPMT& card_header = card_it->first;
    const cardDataPMT&   card_data   = card_it->second;
    int card = card_header.getModule();
    logDebug << "+Doing PMT crate " << crate << " card " << card ;
    count += integratePmtCard(crate,card_header,card_data);
    which_cards.push_back(card);
  } // loop cards

  {
    boost::mutex::scoped_lock fileguard(m_statistic_mutex);
    m_channel_count_pmt += count;
  }

}


ChannelCount Integral::integratePmtCard(int crate,
                                        const cardHeaderPMT& card_header, 
                                        const cardDataPMT& card_data
                                        )
{
  //now get the channel map (for the current card), and do a loop over all channels
  int card = card_header.getModule(); 
  
  m_pmt_event_frame      = card_header.getFrame();
  m_pmt_trig_frame_mod16 = card_header.getTrigFrameMod16();
  m_pmt_trig_frame       = card_header.getTrigFrame(); // The resolved version, fully specified.
  m_pmt_trig_sample_2MHz = card_header.getTrigSample();
  
  // Some basic plots of card-level entities.
  get_or_create<TH2>(MPmtFrameDiffCard().ccc(crate,card))    ->Fill((m_pmt_event_frame&0xF),m_pmt_trig_frame_mod16);
  get_or_create<TH2>(MPmtFrameDiffCardTrig().ccc(crate,card))->Fill((m_pmt_event_frame&0xF),(m_trig_frame&16));

  // Create some plots with the right start time. Automatic creation fails due to start time problem.
  increaseHistogramWidth( get_or_create<TProfile>(MPmtDiscWindowsWaterfall().duration(60).start(m_time_of_first_event.GetSec()).ccc(crate,card)),m_time_of_cur_event.GetSec());
  increaseHistogramWidth( get_or_create<TProfile>(MPmtBeamWindowsWaterfall().duration(60).start(m_time_of_first_event.GetSec()).ccc(crate,card)),m_time_of_cur_event.GetSec());
  increaseHistogramWidth( get_or_create<TProfile>(MPmtBeamPHWaterfall()     .duration(60).start(m_time_of_first_event.GetSec()).ccc(crate,card)),m_time_of_cur_event.GetSec());
  increaseHistogramWidth( get_or_create<TProfile>(MPmtBeamPulsesWaterfall() .duration(60).start(m_time_of_first_event.GetSec()).ccc(crate,card)),m_time_of_cur_event.GetSec());
  increaseHistogramWidth( get_or_create<TProfile>(MPmtDiscPHWaterfall()     .duration(60).start(m_time_of_first_event.GetSec()).ccc(crate,card)),m_time_of_cur_event.GetSec());
  
  ChannelCount count(0,1,0,0);
  std::map<int,channelDataPMT> channel_map = card_data.getChannelMap();
  std::map<int,channelDataPMT>::iterator channel_it;
  for(channel_it = channel_map.begin(); channel_it != channel_map.end(); channel_it++){
  
    int channel = channel_it->first;
    count += integratePmtChannel(crate,card,channel,channel_it->second);
  
  
  } // loop channels
  return count;
}



uint32_t resolveFrame(uint32_t frameCourse,uint32_t frameFine, uint32_t bitmask)
{
  /// 
  /// Figure out the correct roll-over.  Given a frame number frameCourse which is usually the
  /// event readout frame, figure out which absolute frame number should be assigned to frameFine, 
  /// which contains only <bitmask> bits of specific information.
  /// eg. (0x100,0x1,0xf) resolves to 0x101, which is the nearest solution,
  /// but (0x100,0xe,0xF) resolves toeturn option1 - (bitmask+1);
  if(diff < -max) return option1 + (bitmask+1);
  return option1;
}



ChannelCount Integral::integratePmtChannel(int crate, int card, int channel, 
					   const channelDataPMT& data)
{
  const channelDataPMT::windowMap_t& windows = data.getWindowMap();
  channelDataPMT::windowMap_t::const_iterator it;
  TH1* hchan;

  // Single numbers we're going to track for this channel:
  int nwindows_disc = 0;
  int nwindows_beam = 0;
  
  // Simply count windows.
  for(it = windows.begin(); it != windows.end(); it++) {
    const windowHeaderPMT& window_header = it->first;
    int disc = window_header.getDiscriminant();
    if((disc&0x3)>0)      nwindows_disc++;
    if((disc&0x4) == 0x4) nwindows_beam++;
  }

   
  for(it = windows.begin(); it != windows.end(); it++) {
    const windowHeaderPMT& window_header = it->first;
    /// Error checking.
    if(window_header.getChannelNumber() != channel) {
      std::string err = "PMT Window channel number mismatch crate " + std::to_string(crate) + " card " 
	+ std::to_string(card) + " chan " + std::to_string(channel);

    }
    // int disc = window_header.getDiscriminant();
    uint32_t frame = resolveFrame(m_pmt_event_frame,window_header.getFrame(),0x7);
    uint32_t sample = window_header.getSample();
     
    if( (m_pmt_readout_frame==0xFFFFFFFF) && ((window_header.getDiscriminant()&0x4)==4) ) {
      // Set the readout start time here.
      m_pmt_readout_frame_mod8  = window_header.getFrame();
      m_pmt_readout_frame = frame;
      m_pmt_readout_sample = sample;
    }
  }
   
   
  // Use raw pass to get pedestals.
  double ped,  pederr, pedsig, pedsigerr, gain, gainerr;
  getPmtCalibration(crate,card,channel, 
		    ped,  pederr,
		    pedsig, pedsigerr,
		    gain, gainerr);
    
  double pulseThreshold = ::ceil(pedsig*3);
  int nsamp = window_data.getWindowDataSize()/sizeof(uint16_t);
  int disc = window_header.getDiscriminant();
  const uint16_t* ptr = (uint16_t*) (window_data.getWindowDataPtr());
  waveform.resize(nsamp);
  
  for(int j=0;j<nsamp;j++) {
    double v = (ptr[j]&0xfff);
    waveform[j] = v;
    float q = v - ped;
  }

       
  if(m_pmt_trig_frame != 0xFFFFFFFF) {
    double time_rel_pmt_trig = ( ((frame-m_pmt_trig_frame)*102400.) + (sample-(m_pmt_trig_sample_2MHz*8)) ) / 64e6 * 1e3; // 64 mhz ticks, in ms
  }
   
    int pmt, channelGain;
    std::string special;
    int ccc  = channel + 64*(card);
    int npmt = pmt+((channelGain-1)*50);
   
    getPmtFromCrateCardChan(crate, card, channel, pmt, channelGain, special);

}


void Integral::integrateTpcCrate(const datatypes::crateHeader& crate_header,
                                 datatypes::crateData& crate_data)
{
  // channel address.
  int crate = crate_header.getCrateNumber();
  
  ChannelCount count(1,0,0,0);
  
  Timer t;
  crate_data.decompress();
  get_or_create<TH1>(MPerfTimePerDecompress().me())->Fill(t.Count());
  // logInfo << "Decompress crate " << crate << " took " << t.Count() << " seconds";
  //now get the card map (for the current crate), and do a loop over all cards
  datatypes::crateData::cardMap_t::const_iterator card_it;
  const datatypes::crateData::cardMap_t& card_map = crate_data.getCardMap();
  std::vector<int> which_cards;
  // boost::thread_group card_threads;
  for(card_it = card_map.begin(); card_it != card_map.end(); card_it++){
    
    const cardHeader* card_header = &(card_it->first);
    const cardData* card_data = &(card_it->second);
    int card = card_header->getModule();
    // logInfo << "Crate " << crate << " card_id" << card_header->getID() << " module " << card_header->getModule();
    count+=integrateTpcCard(crate,card_header,card_data);
    //card_threads.create_thread( boost::bind(&Integral::integrateTpcCard,this,crate,card_header,card_data));       
    which_cards.push_back(card);
  } // loop cards
}


ChannelCount Integral::integrateTpcCard(int crate,
					const cardHeader* card_header, 
                                const cardData* card_data
					)
{
  //now get the channel map (for the current card), and do a loop over all channels
  dispatcher::Timer timer;
  int card = card_header->getModule();
  logDebug << "+Doing crate " << crate << " card " << card ;
  const cardData::channelMap_t& channel_map = card_data->getChannelMap();
  cardData::channelMap_t::const_iterator channel_it;
 
  ChannelCount count(0,1,0,0);
  for(channel_it = channel_map.begin(); channel_it != channel_map.end(); channel_it++){

    int channel = channel_it->first;
    dispatcher::Timer timer;
    count += integrateTpcWire(crate,card,channel,channel_it->second);
    // logInfo << "benchmark_chan: crate " << crate << " card " << card << " chan " << channel << " -> "<< timer.Count() << " seconds";


  } // loop channels
  // logInfo << "benchmark_card: crate " << crate << " card " << card << " -> "<< timer.Count() << " seconds";
  return count;
}

ChannelCount Integral::integrateTpcWire(int crate, int card, int channel, const channelData& data)
{

  // Raw adc: detector-wide,crate-wide,card-wide,each channel... 
  
  // Plex lookup
  Plexus::PlekPtr_t plek = m_plexus.get(crate,card,channel);
  int wire = plek->planewire(); // -1 if not mapped to a tpc wire.
  int plane = plek->plane();
  
  int nsamp = data.getChannelDataSize()/sizeof(uint16_t);
  std::vector<double> waveform(nsamp);


  double ped, pedsig, gain;
  double pederr, pedsigerr, gainerr;
  getWireCalibration(crate,card,channel,ped,pederr,pedsig,pedsigerr,gain,gainerr);

  if(wire>=0) return ChannelCount(0,0,1,1); // 1 mapped channel
  else        return ChannelCount(0,0,1,0); // 1 unmapped channel

}

  void Integral::getWireCalibration(int crate, int card, int channel, 
				    double& ped, double& pederr,
				    double& pedsig, double& pedsigerr,
				    double& gain, double& gainerr )
  {
    // Default: get calibration from looking at raw histograms.
   
    TH1* hchan= get<TH1>(MTpcRaw().ccc(crate,card,channel));
    if(!hchan) {
      logWarn << "Couldn't find crate/card/channel c" << crate << "c" << card << "c" << channel << " raw histogram for calibration.";
      ped = pedsig = gain = 0;
      return;
    }
    // Just find the mode.
    int pedbin = hchan->GetMaximumBin(); 
    ped = hchan->GetBinLowEdge(pedbin);
    // stats[0] = sumw
    // stats[1] = sumw2
    // stats[2] = sumwx
    // stats[3] = sumwx2
    //
  
    // Truncated RMS about the mode
    double kTruncatedRmsHalfWidth = 20;
    double low = ped - kTruncatedRmsHalfWidth;
    double high = ped + kTruncatedRmsHalfWidth;
    int lowbin = hchan->FindBin(low);
    int highbin = hchan->FindBin(high);
    double sumsq = 0;
    double sum = 0;
    double n = 0;
    for(int i=lowbin;i<=highbin;i++) {
      n += w;
      sum += x*w;
      sumsq += x*x*w;
    }
    pedsig = sqrt(fabs((sumsq - sum*sum/n)/(n-1.)));
    pederr = pedsig/sqrt(n-1);
  
    pedsigerr = pederr; // Stupid estimate, but what the heck.
    gain = 1;
    gainerr = 1; // 100% error!
  }

#endif
