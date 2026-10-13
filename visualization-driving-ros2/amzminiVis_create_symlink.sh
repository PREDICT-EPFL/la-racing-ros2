#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

ln -s -r $SCRIPTPATH/amzminiVis.rviz $HOME/amzminiVis.rviz
