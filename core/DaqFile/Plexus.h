#ifndef PLEXUS_H
#define PLEXUS_H

#include <string>
#include <map>
#include <memory>

//
// A class to interface to the what-connects-to-what database.
//

namespace gov
{
namespace fnal
{
namespace uboone
{
namespace online2
{
  
class sqlite3;

class Plexus
{
public:
  class Plek {
  public:
    Plek();
    virtual ~Plek() {};
    int crate()   const {return _crate;};
    int card()    const {return _card;};
    int channel() const {return _channel;};
    
    bool isPmt() const { return (_pmt>=0); }
    bool isWire() const { return (_wirenum>=0); }

    char view()     const { return _view; }; // U,V,Y
    int  plane()    const { return _plane; };  // 0-2 plane id
    int  planewire()const { return _planewire; }; // number 0-4000ish of wire in the plane
    int  wirenum()  const { return _wirenum; }; // number to 8155 of the wire in the detector

    int  pmt()      const { return _pmt; };
    int  gain()     const { return _gain; }; // 1=low, 2=high, -1 for special or unmapped.
    const std::string& special() const {return _special;}; // name of special channel, "" for unmapped
    
    
    int  _crate;   // crate number
    int  _card;    // slot of readout card
    int  _channel; // 0-63 in motherboard  

    char _view; // U,V,Y
    int  _plane;  // 0-2 plane id
    int  _planewire; // number 0-4000ish of wire in the plane
    int  _wirenum; // number to 8155 of the wire in the detector

    int  _pmt;
    int  _gain; // 1=low, 2=high, -1 for special or unmapped.
    std::string _special; // name of special channel, "" for unmapped
    
  };

  /*  
#ifndef __CINT__  
  typedef std::shared_ptr<const Plek>    PlekPtr_t;
#endif
  */
  typedef const Plek&    PlekPtr_t;

  Plexus();          
  ~Plexus();
  
  bool buildFromPostgresql(const std::string& connection="host=localhost port=5432" );
  bool buildHardcoded();  
  bool is_ok() const { return m_ok; };
 
  PlekPtr_t get(int crate, int card, int channel);

protected:
  bool buildHardcodedTpc();
  bool buildHardcodedPmt();

  void insert(PlekPtr_t plek);
  void insert_tpc_channel(int crate, int card, char wireplane, int wirenum, int channel_id, int larsoft_channel, int larsoft_wirenum );
  
  int ccc(int crate,int card, int channel);
  typedef std::map<int,PlekPtr_t> MapType_t;
  MapType_t m_ccc_to_plek;
  bool m_ok = false;

  PlekPtr_t m_nullplek;  
};

        
}}}} // namespace

#endif /* end of include guard: PLEXUS_H */
