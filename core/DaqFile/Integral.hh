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

#include <iostream>

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

class Integral{

public:

  /// Default constructor
  Integral(){};

  /// Default destructor
  virtual ~Integral(){};

};

#endif
/** @} */ // end of doxygen group 

