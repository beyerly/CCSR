# Top level module for CCSR NLP. Opens a read-fifo and waits for googleSpeech2Text.sh to pipe in a string, and 
# runs the nlpxCCSR parser on it. ccsrNlpClass writes directly to the CCSR process through a fifo.

import sys
sys.path.insert(0, './nlpxCCSR')
from nlpx import ccsrNlpClass

import getopt 

fifo = './text2nlpFifo'
logfile = './nlpxCCSR.log'

loop = True

def main(argv):
   global loop
   try:
      opts, args = getopt.getopt(argv,"hn",["help","noloop"])
   except getopt.GetoptError:
      print 'nlp.py -h -l'
      sys.exit(2)
   for opt, arg in opts:
      if opt == '-h':
         print 'nlp.py'
         sys.exit()
      elif opt in ("-n"):
	 loop = False

if __name__ == "__main__":
   main(sys.argv[1:])


appID = 'T3H9JX-RQQ2273TJ9'        # Fill in yur own Wolfram AppID here
useFifos = True                    # Only set True if integrated with CCSR robot platform
s = ccsrNlpClass(useFifos, appID)

# log = open(logfile, 'w')
# log.writeline('starting nlpxCCSR')

while (1):
   # Open pipe from googleVoice2Text.sh
   f = open(fifo, 'r')
#   log.writeline(fifo + ' opened for reading')
   # Read sentence, this blocks untill googleVoice2Text.sh posts a sentence.
   line = f.readline()
   print line
#   log.writeline('parsing: ' + line)
   s.nlpParse(line)
   f.close()
   if not loop:
      break

# log.close()
