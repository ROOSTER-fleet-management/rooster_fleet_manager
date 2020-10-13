#!/bin/bash
# A script to generate documentation and copy the necessary files to the appropriate directories
# so that the html files can be hosted on github pages.

roscd rooster_fleet_manager
rosdoc_lite .

DIRECTORY=docs
if [ ! -d "$DIRECTORY" ]; then
    mkdir docs
fi

#copying built html files to docs folder to be viewed in browser
/bin/cp -r doc/html/python/* docs/
/bin/cp -r doc/html/msg/ docs/
/bin/cp -r doc/html/srv/ docs/