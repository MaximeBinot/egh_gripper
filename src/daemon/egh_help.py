#!/usr/bin/env python

import sys 
import re
import time

def print_hex(msg, title=""):
	if title != "":
		print("-"*60)
		print(title)
		print("-"*60)
	print('[ 0x'+', 0x'.join(format(x, '02x') for x in msg)+']')
	#print("\n")

def strip_control_characters(input):

    if input:

        import re

        # unicode invalid characters
        RE_XML_ILLEGAL = u'([\u0000-\u0008\u000b-\u000c\u000e-\u001f\ufffe-\uffff])' + \
                         u'|' + \
                         u'([%s-%s][^%s-%s])|([^%s-%s][%s-%s])|([%s-%s]$)|(^[%s-%s])' % \
                          (unichr(0xd800),unichr(0xdbff),unichr(0xdc00),unichr(0xdfff),
                           unichr(0xd800),unichr(0xdbff),unichr(0xdc00),unichr(0xdfff),
                           unichr(0xd800),unichr(0xdbff),unichr(0xdc00),unichr(0xdfff),
                           )
        input = re.sub(RE_XML_ILLEGAL, "", input)

        # ascii control characters
        input = re.sub(r"[\x01-\x1F\x7F]", "", input)

    return input

def wait_until(predicate, timeout=2, period=0.01, *args, **kwargs):
	mustend = time.time() + timeout
	while time.time() < mustend:
		if predicate(*args, **kwargs): return True
		time.sleep(period)
	#print ("wait_until: time out")
	return False 

def bitList2Int(bitList):
	out = 0
	count = 0
	for bit in bitList:
		out = out | (bit << count)
		count = count + 1
		#out = (out << 1) | bit
	return out




