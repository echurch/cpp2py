import os,sys,commands

# check if LITE_FMWK_BASEDIR is defined. If not, nothing to do.
if not 'LITE_FMWK_BASEDIR' in os.environ:
    sys.exit(1)

# read in template GNUmakefile
makefile=open('%s/config/template/TopMakefile.tmp' % os.environ['LITE_FMWK_BASEDIR'],'r').read()

fout=open('%s/GNUmakefile' % os.environ['LITE_FMWK_BASEDIR'],'w')
fout.write(makefile)
fout.close()
