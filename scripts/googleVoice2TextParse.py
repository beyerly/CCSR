#!/usr/bin/python

import sys
import re
import json
from pprint import pprint

json_data=open('text.json')

data = json.load(json_data)
print data["result"][0]["alternative"][0]["transcript"]
json_data.close()
