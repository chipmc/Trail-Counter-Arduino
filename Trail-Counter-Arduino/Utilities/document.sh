#!/bin/bash

#
# embedXcode
# ----------------------------------
# Embedded Computing on Xcode
#
# Copyright © Rei VILO, 2010-2016
# http://embedxcode.weebly.com
# All rights reserved
#
# Last update: Jun 27, 2016 release 5.0.6
#

export PATH=$PATH:/Applications/Xcode.app/Contents/Developer/usr/bin
export PATH=$PATH:/usr/local/texlive/2015/bin/x86_64-darwin
export PATH=$PATH:/usr/local/texlive/2013/bin/x86_64-darwin
export PATH=$PATH:/usr/local/texlive/2012/bin/x86_64-darwin
export PATH=$PATH:/usr/local/texlive/2014/bin/x86_64-darwin
export PATH=$PATH:/usr/local/texlive/2015/bin/x86_64-darwin
export PATH=$PATH:/usr/local/texlive/2016/bin/x86_64-darwin
cd Builds/latex
make
cd ../..
