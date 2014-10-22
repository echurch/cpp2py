

#ifndef DAQFILE_CC
#define DAQFILE_CC

#include "DaqFile.hh"


// Prevents ASSERT on boolean values not being 1 or zero
#define BOOST_DISABLE_ASSERTS 1

//#include "dispatcher/Logging.h"


// Hand-include eventRecord ...
#include "datatypes/eventRecord.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/stream.hpp>

using gov::fnal::uboone::datatypes::eventRecord ;

DaqFile::DaqFile( const std::string& pathname )
  : good(false)
  , closedCleanly(false)
  , m_entry(0)
  , nevents(-1)
  , index_buffer(0)
  , ifs(pathname,std::ios::binary)
{
  good = true;
  if( (!ifs.is_open()) || (!ifs.good()) ) {
    good = false;
    return;
  }
  // get length of file:
  ifs.seekg (0, ifs.end);
  uint64_t length = ifs.tellg();
  if(length < 6) {
    good = false;
    closedCleanly = false;
    return;
  }

  ifs.seekg(-6,ifs.end); // Go to position 6 bytes before end of file.
  uint8_t buff[10];
  ifs.read((char*) buff, 6);
  
  // Danger danger will robinson!  Endian totally unchecked!
  nevents = *(uint32_t*)(buff);
  std::cout << " nevents = " << std::dec << nevents << std::endl;
  uint16_t endOfFileMarker = *(uint16_t*)(buff+4);
  std::cout << " eof marker = 0x" << std::hex << endOfFileMarker << std::endl;
  ifs.clear();    
  ifs.seekg(0,ifs.beg);

  std::cout << std::dec << nevents << " events in file." << std::endl;
  closedCleanly = true;
  if(endOfFileMarker != 0xe0f0) {
    closedCleanly = false;
    nevents = 0;
    return;
  }

  uint64_t index_pos = 6 + sizeof(uint32_t)*nevents;
  ifs.clear();
  index_buffer = new uint32_t[nevents];
  ifs.seekg(-index_pos,ifs.end);
  ifs.read((char*)index_buffer,sizeof(uint32_t)*nevents);  
  ifs.seekg(0,ifs.beg);
}


DaqFile::~DaqFile()
{
  if(index_buffer) delete [] index_buffer;
}

eventRecord const& DaqFile::GetEventObj     (int entry)
{
  return (*(GetEvent(entry)));
}
eventRecord const& DaqFile::GetNextEventObj ()
{
  return (*(GetNextEvent()));
}

int DaqFile::GetEventData(unsigned int entry, char* &outEventData, size_t &outEventSize)
{
  // Skip in stages: this prevents integer overflow from skipping many GB at a time!
  outEventData = 0;
  outEventSize = 0;
  if(entry>=nevents) return 0;

  ifs.seekg(0,ifs.beg);
  for(unsigned int i=0;i<entry;i++) ifs.seekg(index_buffer[i],ifs.cur);
  ifs.clear();    
  outEventSize = index_buffer[entry];
  
  // Read it.
  outEventData = new char[outEventSize];
  ifs.read((char*)outEventData, outEventSize);
  
  return 1;
}

std::shared_ptr<eventRecord> DaqFile::GetEvent(int entry)
{
  m_entry = entry;
  char* data = 0;
  size_t size = 0;
  int res = GetEventData(entry, data,size);
  if(!res) return std::shared_ptr<eventRecord>(); // Return an empty pointer.

  //
  // Unpack a raw dispatcher message into a full-blown set of data.
  //

  std::shared_ptr<eventRecord> r; // null return pointer
  if(!data) return r;
  if(size==0) return r; // nothing to do; leave defaults.

  try {
    // ok, try to unpack the darned event.
    namespace io = boost::iostreams;
    io::basic_array_source<char> source(data,size);
    io::stream<io::basic_array_source <char> > input_stream(source);
    boost::archive::binary_iarchive ia(input_stream);

    r = std::shared_ptr<eventRecord>(new eventRecord);
    ia >> *r;
  
    if(data) delete [] data; data = 0;
    return r;
  }
  catch ( std::exception& e ) {
    std::cout << "Error unpacking event record from entry " << entry << ": " << e.what() << std::endl;
  }

  m_entry++;
  if(data) delete [] data; data = 0;
  return std::shared_ptr<eventRecord>(); // Return an empty pointer.
}

std::shared_ptr<eventRecord> DaqFile::GetNextEvent()
{
  // This version just attempts to read the file directly.
  // More useful if the file wasn't closed correctly.

  std::shared_ptr<eventRecord> r = std::shared_ptr<eventRecord>(new eventRecord);
  try {
    boost::archive::binary_iarchive ia(ifs); // declare a boost::archive::binary_iarchive object
    ia >> *r;  // read in from the archive into your eventRecord object
    return r;
  }
  catch ( std::exception& e ) {
    std::cout << "Error unpacking event record from entry " << m_entry << ": " << e.what() << std::endl;
  }

  m_entry++;
  return std::shared_ptr<eventRecord>(); // Return an empty pointer.
}

#endif
