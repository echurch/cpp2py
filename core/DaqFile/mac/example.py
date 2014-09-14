import sys
from ROOT import *

try:

    print "PyROOT recognized your class %s" % str(DaqFile)

except NameError:

    print "Failed importing DaqFile..."

sys.exit(0)

