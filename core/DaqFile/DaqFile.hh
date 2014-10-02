/**
 * \file DaqFile.hh
 *
 * \ingroup DaqFile
 * 
 * \brief Class def header for a class DaqFile
 *
 * @author echurch
 */

/** \addtogroup DaqFile

    @{*/
#ifndef DAQFILE_HH
#define DAQFILE_HH

#include <fstream>
#include <vector>
#include <memory>

#include <iostream>
#include <stdint.h>

namespace gov {namespace fnal {namespace uboone {namespace datatypes {
	class eventRecord;
      } } } }


/**
   \class DaqFile
   User defined class DaqFile ... these comments are used to generate
   doxygen documentation!
 */
class DaqFile{

public:

  /// Default constructor
  DaqFile(){};
  DaqFile( const std::string& pathname );

  bool Good()          { return good; };
  bool ClosedCleanly() { return closedCleanly; };
  int  NumEvents()     { return nevents; }; /// only valid if ClosedCleanly() 
  int GetEventData(unsigned int entry, char* &outEventData, size_t &outEventSize);

  gov::fnal::uboone::datatypes::eventRecord const& GetEventObj     (int entry);
  gov::fnal::uboone::datatypes::eventRecord const& GetNextEventObj ();

#ifndef __CINT__  
  std::shared_ptr<gov::fnal::uboone::datatypes::eventRecord> GetEvent(int entry);
  std::shared_ptr<gov::fnal::uboone::datatypes::eventRecord> GetNextEvent();
#endif
  
private:
  bool      good;
  bool      closedCleanly;
  uint32_t  m_entry;
  uint32_t  nevents;
  uint32_t*  index_buffer; 
  std::ifstream ifs;
  /// Default destructor
  virtual ~DaqFile();

};

#endif
/** @} */ // end of doxygen group 

