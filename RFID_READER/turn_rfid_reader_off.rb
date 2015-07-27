# Script to stop Alien ALR-9680 from reading

# add the default relative library location to the search path
$:.unshift File.join(File.dirname(__FILE__),"..","lib")

require 'alienreader'
require 'alienconfig'
require 'uri'
require 'net/http'

begin
# grab various parameters out of a configuration file
	config = AlienConfig.new("config.dat")

# change "reader_ip_address" in the config.dat file to the IP address of your reader.
	ipaddress = config["reader_ip_address"]
	puts ipaddress
# create our reader 
	r = AlienReader.new

	if r.open(ipaddress)    

		r.tagtype ='16'
		r.autoaction = 'acquire'
		r.autostarttrigger='0 0'
		r.automode='on'
		puts 'Reading...'
		sleep 1
		puts "...Done!"
		
		r.automode='off'
		puts "OFF----------------------------------"
		
	# be nice. Close the connection to the reader.
		r.close
	end
rescue
	puts $!
end
