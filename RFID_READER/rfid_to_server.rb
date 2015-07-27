# Turn on automode so Alien-9680 continuously reads.  You can use any RFID reader but POST RFID tag's values to the server in a similar fashion.

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
		$i = 0
		while $i < 1 do
			r.tagtype ='16'
			r.autoaction = 'acquire'
			r.autostarttrigger='0 0'
			r.automode='on'
			puts 'Reading...'	
			sleep 1
			puts "...Done!"

			params = {"rfid_api_guid" => r.taglist}

			x = Net::HTTP.post_form(URI.parse('http://ec2-52-25-236-123.us-west-2.compute.amazonaws.com/rfidwrite/'), params)
			puts x.body
		end
		
		r.automode='off'
		puts "----------------------------------"
		
	# be nice. Close the connection to the reader.
		r.close
	end
rescue
	puts $!
end
