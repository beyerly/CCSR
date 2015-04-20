#!/bin/bash

# start speech2text service and NLP service

cd scripts
./googleVoice2Text.sh > voice2text.log & 
python ./nlpxCCSR.py &
cd webIFCCSR
python ./webIFCCSR.py &
cd ../../
./ccsr
