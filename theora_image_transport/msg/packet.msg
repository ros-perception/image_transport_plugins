# ROS message adaptation of the ogg_packet struct from libogg,
# see http://www.xiph.org/ogg/doc/libogg/ogg_packet.html.

uint8[] blob      # Packet data
int32 bytes       # Size of packet data in bytes [yes, this is just blob.size()]
int32 b_o_s       # Flag indicating whether this packet begins a logical bitstream
int32 e_o_s       # Flag indicating whether this packet ends a bitstream

int64 granulepos  # A number indicating the position of this packet in the decoded data
int64 packetno    # Sequential number of this packet in the ogg bitstream
