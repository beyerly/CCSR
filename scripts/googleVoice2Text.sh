#!/bin/bash

# This script continuously checks for the existence of the 'voiceFile'
# Once ccsr has recorded an audio snippet, it will write out this file to disk
# This script will send it to the Google Speech to text API, which returns a json file with text
# ./googleVoice2TextParse.py will parse this json file and write the first/best guess to stdout,
# which gets piped to nlpxCCSR.py. This script interprets the sentence. The voice file will be deleted, and the 
# polling for a new voice file resumes


# CCSR writes this file to disk:
voiceFile="/home/root/ccsr/data/voice.wav"    
logfile="./googleVoice2Text.log"
# fifo to nlpxCCSR.py
fifo="./text2nlpFifo"
rate=44100

if [ ! -f $fifo ] 
then
   mkfifo $fifo
fi

ps | grep ccsr > /dev/null
while [ $? -eq 0 ]; do
   echo "Waiting to start google voice to text service..." > $logfile
   echo $voiceFile > $logfile
   echo waiting > $logfile
   while [ ! -f $voiceFile ]; do
     # wait untill voice file appears
     sleep 1;
   done
   echo posting request to google specht2text > $logfile
   curl  -X POST \
   --data-binary @$voiceFile \
   --header 'Content-Type: audio/l16; rate=11025;' \
   'https://www.google.com/speech-api/v2/recognize?output=json&lang=en-us&key=AIzaSyCRl0iv1MMI-vMafGdFyGlH4A0b7aXUsgI' -k\
   -o text.json_pre
   # apparently google returns 2 json structures, the first one always empty. Delete this first structure:
   tail -n +2 text.json_pre > text.json
   # this script reads text.json and pipes single sentence to nlpxCCSR.py
   ./googleVoice2TextParse.py > $fifo
   rm $voiceFile;   
done
echo "CCSR is not running..." > $logfile
