#include "Plexus.h"
#include "dispatcher/Logging.h"
#include "FormString.h"

// #include <sqlite3.h>
#include "libpq-fe.h"

namespace gov
{
namespace fnal
{
namespace uboone
{
namespace online
{

Plexus::Plek::Plek()
  : _crate (-1)  
  , _card (-1)
  , _channel (-1)
  , _view ('-')
  , _plane (-1)
  , _planewire (-1)
  , _wirenum (-1)
  , _pmt (-1)
  , _gain (-1)
  , _special()
{
}



Plexus::Plexus()
 : m_ok(false)
 , m_nullplek(new Plek())                
{}
  
Plexus::~Plexus()
{}
  

int Plexus::ccc(int crate, int card, int channel)
{
  return (100*crate + card)*100 + channel;
}

Plexus::PlekPtr_t Plexus::get(int crate, int card, int channel)
{
  MapType_t::iterator it = m_ccc_to_plek.find(ccc(crate,card,channel));
  if(it!=m_ccc_to_plek.end()) return it->second;
  return m_nullplek;
}

void Plexus::insert(PlekPtr_t p)
{
  m_ccc_to_plek[ccc(p->crate(),p->card(),p->channel())] = p;
}


void Plexus::insert_tpc_channel(int crate, int card, char wireplane, int wirenum, int channel_id, int larsoft_channel, int larsoft_wirenum )
{
  (void)larsoft_wirenum; // unused variable. 
  
  Plek* p = new Plek;
  p->_crate = crate;
  p->_card = card;
  p->_channel = (channel_id % 64); // Hack. DB doesn't have a proper motherboard channel id.
  p->_view = wireplane;
  switch(p->_view) {
    case 'U' : p->_plane = 0; break;
    case 'V' : p->_plane = 1; break;
    case 'Y' : p->_plane = 2; break;
    default :  p->_plane = -1; break;
  }
  p->_planewire = wirenum;
  p->_wirenum = larsoft_channel;
  m_ccc_to_plek[ccc(p->_crate,p->_card,p->_channel)] = PlekPtr_t(p);
  
}

  
bool Plexus::buildFromPostgresql(const std::string& connection )
{
  m_ccc_to_plek.clear();
  
  PGconn *conn = PQconnectdb(connection.c_str());
  if(PQstatus(conn)!=CONNECTION_OK) {
    logError << "Couldn't open connection to postgresql interface at " << connection;
    PQfinish(conn);
    return false;
  }

  PGresult *res  = PQexec(conn, "BEGIN");
  if (PQresultStatus(res) != PGRES_COMMAND_OK) { 
      logError << "postgresql BEGIN failed";
       PQclear(res);
       PQfinish(conn);
       return false;
  }
  PQclear(res);
  
  res = PQexec(conn,
      "SELECT crate_id, slot, wireplane, wirenum, channel_id, larsoft_channel, larsoft_wirenum "
      " FROM channels NATURAL JOIN asics NATURAL JOIN motherboards NATURAL JOIN coldcables NATURAL JOIN motherboard_mapping NATURAL JOIN intermediateamplifiers NATURAL JOIN servicecables NATURAL JOIN servicecards NATURAL JOIN warmcables NATURAL JOIN ADCreceivers NATURAL JOIN crates NATURAL JOIN fecards "
  );
  if ((!res) || (PQresultStatus(res) != PGRES_TUPLES_OK))
  {
      logError << "SELECT command did not return tuples properly";
      PQclear(res);
      PQfinish(conn);
      return false;;
  }
  int num_records = PQntuples(res);
  for(int i=0;i<num_records;i++) {
    int crate_id = atoi(PQgetvalue(res, i, 0));
    int slot     = atoi(PQgetvalue(res, i, 1));
    char wireplane = *PQgetvalue(res, i, 2);
    int wirenum  = atoi(PQgetvalue(res, i, 3));
    int channel_id = atoi(PQgetvalue(res, i, 4));
    int motherboard_channel = channel_id % 64;
    int larsoft_channel = atoi(PQgetvalue(res, i, 5));
    int larsoft_wirenum = atoi(PQgetvalue(res, i, 6));
    logVerbose << "   crate_id " << crate_id 
            << "   slot " << slot 
            << "   wireplane " << wireplane 
            << "   wirenum " << wirenum 
            << "   channel_id " << channel_id 
            << "   motherboard_channel " << motherboard_channel;
    insert_tpc_channel(crate_id, slot, wireplane, wirenum, channel_id, larsoft_channel, larsoft_wirenum);
        //
    // Plek* p = new Plek;
    // p->crate = crate_id;
    // p->card = slot;
    // p->channel = motherboard_channel;
    // p->view = wireplane;
    // switch(p->view) {
    //   case 'U' : p->plane = 0; break;
    //   case 'V' : p->plane = 1; break;
    //   case 'Y' : p->plane = 2; break;
    //   default :  p->plane = -1; break;
    // }
    // p->planewire = wirenum;
    // p->wirenum = larsoft_channel;
    //m_ccc_to_plek[ccc(p->crate,p->card,p->channel)] = PlekPtr_t(p);
  }

  
  PQclear(res);
  PQfinish(conn);

  buildHardcodedPmt();
  
  m_ok = true;
  return m_ok;
}

bool Plexus::buildHardcoded( )
{
  m_ccc_to_plek.clear();
  
  buildHardcodedTpc();
  buildHardcodedPmt();
  m_ok = true;
  return m_ok;
}

bool Plexus::buildHardcodedTpc()
{
#include "hardcoded_tpc_plexus.inc"

  // // Custom algorithm. Incorrect! need to populate this with something more reasonable.
  // // However, it should correctly assign induction/collection planes.
  // // is it x,u,or v
  // // assume channels 0-31 are X
  // // assume odd channels 32-63 are u
  // // assume even channels 32-63 are v
  // outPlane =  -1;
  // outWire  = -1;
  // if(ichan>63) return; // 0-63
  // if(icard>15) return; // 0-16
  // if(icrate>8) return; // 0-8
  // if(ichan<32) {
  //   // Y plane
  //   outPlane = 2;
  //   // count up how many channels of x-outPlane we have to here.
  //   // Assume 16 cards per crate, each of which have 32 crate wires.
  //   int xwire = ((icrate*16)+icard)*32+ichan;
  //   if(xwire<=3456) {outWire = xwire+4798; return;}
  //   else {
  //     //ok, it's not an x-wire, it's an overflow from u/v.
  //     if(icrate==8) {
  //       int uwire = icard*32+ichan + 2304;
  //       if(uwire<2398) {outWire = uwire; return;}
  //     }
  //     if(icrate==9) {
  //       int vwire = icard*32+ichan + 2304;
  //       if(vwire<2398) { outWire = vwire +2399; return; }
  //     }
  //     return;
  //   }
  // } else if (ichan%2 == 1) {
  //   outPlane = 1;
  //   // count up how many channels of u-outPlane we have to here.
  //   // Assume 15 cards per crate, each of which have 8 U wires.
  //   int uwire = ((icrate*16)+icard)*16+((ichan-32)/2);
  //   if(uwire<2398) { outWire = uwire; return; };
  //   return;
  // } else {
  //   outPlane = 0;
  //   int vwire = ((icrate*16)+icard)*16+((ichan-32)/2);
  //   if(vwire<2398) { outWire = vwire + 2399; return; }
  //   return;
  // }
  // }
  
  return true;
}

bool Plexus::buildHardcodedPmt()
{
  int crate = 10;
  for(int icard=7;icard<16;icard++) {
    for(int ichan=0;ichan<64;ichan++) {
      Plek* p = new Plek;
      p->_crate = crate;
      p->_card = icard;
      p->_channel = ichan;
      if(ichan<30) {    
        if(icard>7) p->_gain = 1; // low gain
        else        p->_gain = 2; // high gain
        p->_pmt = ichan;
      } else {
        p->_special = std::string("special_").append(std::to_string(ichan));
      }
      m_ccc_to_plek[ccc(p->_crate,p->_card,p->_channel)] = PlekPtr_t(p);
    }
  }
  return true;
}



}}}} // namespace
