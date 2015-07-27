# Kivy.org must be installed to use this.  Due to monitor driver compatibility reasons windows computers worked best for my testing.  Testing was completed with a Planar Helium PCT2785 27" monitor.
# Everytime the touchscreen is pressed draw a random colored square and send the x & y percentages to the web server.

import kivy
import urllib.request
kivy.require('1.1.1')

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty
from kivy.vector import Vector
from kivy.clock import Clock
from kivy.core.window import Window
import json
from kivy.graphics import *
from kivy.config import Config
from random import randint
import datetime as dt

class RawCord(Widget):
	
	already_published = []
	def was_point_already_published(self,x,y):
		tmp_set = [x,y]
		if tmp_set in self.already_published:
			return True
		else:
			if  self.was_similar_point_already_published(x,y):
				return True
			else:
				self.already_published.append(tmp_set)
				return False
	
	def was_similar_point_already_published(self,x,y):
		tolerance = float(0.03)
		for tmp_set in self.already_published:
			if float(tmp_set[0]) > (float(x) * (1 - tolerance)) and float(tmp_set[0]) < (float(x) * (1 + tolerance)) and float(tmp_set[1]) > (float(y) * (1 - tolerance)) and float(tmp_set[1]) < (float(y) * (1 + tolerance)): 
				return True
				
		return False
		
	def on_touch_down(self, touch):
		with self.canvas:
			Color(float(randint(3,10)/10), float(randint(3,10)/10), float(randint(3,10)/10))
			#I've yet to figure out why but by touch.y values are 1 cm higher than the actual touch.  My screen is 33.5 cm tall.  Therefore I am subtracting 1 cm worth of pixels from touch.y
			adjusted_y = float(touch.y) - (1.0/33.5*float(windowsize_height()))
			
			square_size = 300
			
			x_per = float(touch.x) / float(windowsize_width())
			y_per = float(touch.y) / float(windowsize_height())
			
			if not self.was_point_already_published(x_per,y_per):
				Rectangle(pos=(touch.x - (square_size/2),adjusted_y - (square_size/2)), size=(square_size,square_size))
				with urllib.request.urlopen('http://ec2-52-25-236-123.us-west-2.compute.amazonaws.com/touch_api.php?touch_api_x=' + str(x_per) + '&touch_api_y=' + str(y_per) + '&touch_api_type=1') as response:
					html = response.read()
	
class RawCordApp(App):
	def build(self):
		g = RawCord()
		Window.size = (windowsize_width(), windowsize_height())
		Window.clearcolor = (1, 1, 1, 1)
		return g
		
def windowsize_width():
	return 1600
			
def windowsize_height():
	return 900
	
if __name__ == '__main__':
	Window.fullscreen = True
	RawCordApp().run()