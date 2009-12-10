#include "ros/ros.h"

#include <theora_image_transport/packet.h>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>
#include <ogg/ogg.h>

#include <fstream>
#include <vector>

#define null 0

using namespace std;

class OggSaver
{
public:
  OggSaver(ros::NodeHandle &n, const char* filename)
   : nh_(n), fout(filename, std::ios::out|std::ios::binary)
  {
    if(ogg_stream_init(&os, 0) != 0)
    {
      ROS_ERROR("Unable to initialize ogg_stream_state structure");
      exit(1);
    }

    sub = nh_.subscribe("stream", 10, &OggSaver::processMsg, this);
  }

  ~OggSaver()
  {
    ogg_page op;
    if(ogg_stream_flush(&os, &op) != 0)
    {
      fout.write((char*)op.header, op.header_len);
      fout.write((char*)op.body, op.body_len);
    }
    fout.close();
  }

private:

  ros::NodeHandle &nh_;
  ogg_stream_state os;
  ofstream fout;
  ros::Subscriber sub;

  //When using this caller is responsible for deleting oggpacket.packet!!
  void msgToOggPacket(const theora_image_transport::packet &msg, ogg_packet &oggpacketOutput)
  {
    oggpacketOutput.bytes = msg.bytes;
    oggpacketOutput.b_o_s = msg.b_o_s;
    oggpacketOutput.e_o_s = msg.e_o_s;
    oggpacketOutput.granulepos = msg.granulepos;
    oggpacketOutput.packetno = msg.packetno;
    oggpacketOutput.packet = new unsigned char[msg.bytes];
    memcpy(oggpacketOutput.packet, &msg.blob[0], msg.bytes);

    //ROS_DEBUG("Received %d bytes in packet#%d and granule%d (and this is BOS: %d).", oggpacketOutput.bytes, oggpacketOutput.packetno, oggpacketOutput.granulepos, oggpacketOutput.b_o_s);
    /*unsigned int i = 0;
    for (int j = 0; j < msg.bytes; j++)
      i = i * 2 % 91 + oggpacketOutput.packet[j];
    ROS_DEBUG("Checksum is: %d", i);*/
  }

  void processMsg(const theora_image_transport::packetConstPtr& message)
  {
    const theora_image_transport::packet &pkt = *message;
    ogg_packet oggpacket;
    msgToOggPacket(pkt, oggpacket);

    if(ogg_stream_packetin(&os, &oggpacket))
    {
      ROS_ERROR("Error while adding packet to stream.");
      exit(2);
    }
    delete[] oggpacket.packet;

    ogg_page op;
    if(ogg_stream_pageout(&os, &op) != 0)
    {
      fout.write((char*)op.header, op.header_len);
      fout.write((char*)op.body, op.body_len);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "OggSaver", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  if(argc < 2) {
    cerr << "Usage: " << argv[0] << " stream:=/theora/image/stream outputFile" << endl;
    exit(3);
  }
  if (n.resolveName("stream") == "/stream") {
      ROS_WARN("ogg_saver: stream has not been remapped! Typical command-line usage:\n"
               "\t$ ./ogg_saver stream:=<theora stream topic> outputFile");
  }
  OggSaver saver(n, argv[1]);
  ros::spin();
  return 0;
}
