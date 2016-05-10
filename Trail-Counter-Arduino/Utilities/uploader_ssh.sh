#!/bin/bash
#set -e

# Based on
# Intel Edison linux: upload arduino via ethernet, wifi or network
# Labby.co.uk
# January 18, 2015
# http://labby.co.uk/wp-content/uploads/2015/01/clupload_linux.sh
#
# Last update: Apr 07, 2016 release 4.4.5

echo
echo "*** Upload SSH"

SSH_ADDRESS=$1
SSH_PASSWORD=$2
REMOTE_FOLDER=$3 # not used, for compatibility only
FILE_NAME=$4
OPTION=$5

if [ "$5" == "-debug" ]
then
    SSH_EXEC="gdbserver $SSH_ADDRESS:1234"
else
    SSH_EXEC="exec"
fi

CHECK="`ping -c 1 -W 1 $SSH_ADDRESS | grep "1 packets received" | wc -l`"
if [ "$CHECK" -eq "0" ]; then
    echo -e "ERROR\t$SSH_ADDRESS not available"
    exit 1
fi

HOST=$SSH_ADDRESS
BUILDS_PATH=Builds
UTILITIES_PATH=Utilities

NEW_SKETCH=/sketch/sketch.elf
OLD_SKETCH=$NEW_SKETCH.old

echo "Upload "$FILE_NAME" to "$SSH_ADDRESS
echo ""

echo "1/3 Preparing"
$UTILITIES_PATH/sshpass -p $SSH_PASSWORD ssh root@$HOST "if [ -f $NEW_SKETCH ] ; then mv -f $NEW_SKETCH $OLD_SKETCH ; fi"

echo "2/3 Uploading"
$UTILITIES_PATH/sshpass -p $SSH_PASSWORD scp $BUILDS_PATH/$FILE_NAME root@$SSH_ADDRESS:$NEW_SKETCH

echo "3/3 Running"
if [ "$5" == "-debug" ]
then
	SSH_EXEC="gdbserver $SSH_ADDRESS:1234"
    $UTILITIES_PATH/sshpass -p $SSH_PASSWORD ssh root@$HOST "chmod +x $NEW_SKETCH; killall -q -USR1 launcher.sh || true; killall -q clloader || true; killall -q sketch.elf || true; $SSH_EXEC $NEW_SKETCH"
else
    SSH_EXEC="exec"
    $UTILITIES_PATH/sshpass -p $SSH_PASSWORD ssh root@$HOST "chmod +x $NEW_SKETCH; killall -q -USR1 launcher.sh || true; killall -q clloader || true; killall -q sketch.elf || true; $SSH_EXEC $NEW_SKETCH /dev/pts/0 > /dev/null 2>&1 &"
fi

echo "*** Done"


