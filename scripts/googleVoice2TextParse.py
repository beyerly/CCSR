#!/usr/bin/python

import sys
import re
import json
import os
from pprint import pprint

statinfo = os.stat('text.json')
if (statinfo.st_size>0):
   json_data=open('text.json')
   data = json.load(json_data)
   print data["result"][0]["alternative"][0]["transcript"]
   json_data.close()
