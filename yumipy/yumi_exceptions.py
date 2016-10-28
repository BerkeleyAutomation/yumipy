'''
Exception classes for YuMi control
Author: Jacky Liang
'''

class YuMiCommException(Exception):

	def __init__(self, *args, **kwargs):
		Exception.__init__(self, *args, **kwargs)

class YuMiControlException(Exception):

	def __init__(self, req_packet, res, *args, **kwargs):
		Exception.__init__(self, *args, **kwargs)
		self.req_packet = req_packet
		self.res = res

	def __str__(self):
		return "Failed Request!\nReq: {0}\nRes: {1}".format(self.req_packet, self.res)