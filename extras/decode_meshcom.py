# Minimal Example of decoding MeshCom Lora Frames on port 1798
import socket
import binascii
from struct import *



def hex_dump(data):
    return ' '.join(f'{byte:02X}' for byte in data)

def calc_fcs(msg):
    fcs = 0
    for x in range(0,len(msg)):
        fcs = fcs + msg[x]
    
    # SWAP MSB/LSB
    fcs = ((fcs & 0xFF00) >> 8) | ((fcs & 0xFF) << 8 )
    
    #print("calc_fcs=" + hex(fcs))
    return fcs

def decode_meshinfo( val):
    server = False
    track = False
    app_offline = False
    mesh = False
    
    if(val & 0x08):
        server = True
        
    if(val & 0x4):
        track = True

    if(val & 0x2):
        app_offline = True
    
    if(val & 0x1):
        mesh = True
    
    print("-> server=" + str(server) + " track=" + str(track) + " app_offline=" + str(app_offline) + " mesh=" + str(mesh))

def get_callsign_and_path(aprs_string,payload_type):
    path_end = aprs_string.find('>')
    path = aprs_string[0:path_end]
    
    rest = aprs_string[path_end:]
    target_end = rest.find(chr(payload_type))
    
    target_callsign = rest[1:target_end]
    
    aprs_payload = rest[target_end:]
    
    source_end = path.find(',')
    if source_end != -1:
        source_callsign = path[0:source_end]
    else:
        source_callsign = path
        last_callsign_in_path = path
        
    last_callsign_end = path.rfind(',')
    if last_callsign_end != -1:
        last_callsign_in_path = path[last_callsign_end+1:]
    
    print("Path: " + path)
    print("Target: " + target_callsign)
    print("Source: " + source_callsign)
    print("LastInPath: " + last_callsign_in_path)
    print("APRS-Payload: " + aprs_payload)
    
    return [target_callsign, source_callsign, path, last_callsign_in_path, aprs_payload]    

def decode(msg):
    print("")
    
    msg_start = msg[0:6]
    msg_end = msg[-9:]
    
    calced_fcs = calc_fcs(msg[0:-6])
        
    # little-endian unpack
    [payload_type, message_id, max_hop_raw] = unpack('<BIB', msg_start)
    max_hop = max_hop_raw & 0x0F
    mesh_info = max_hop_raw >> 4
    
    print("DTI=" + hex(payload_type) + " MSGID=" + hex(message_id) + " HOP=" + str(max_hop) + " MESH=" + hex(mesh_info))
    decode_meshinfo(mesh_info)

    # http://www.aprs.org/doc/APRS101.PDF
    # APRS Data Type Identifiers
    # 0x3A = ':' = Message
    # 0x21 = '!' = Position without timestamp (no APRS messaging)
    # 0x40 = '@' = Position with timestamp (with APRS messaging)
    
    if((payload_type == 0x3A) or (payload_type == 0x21) or (payload_type == 0x40)):
        try:
            aprs_string = msg[6:-9].decode()
            print("APRS_MESSAGE=" + aprs_string)
            [target_callsign, source_callsign, path, last_callsign_in_path, aprs_payload] = get_callsign_and_path(aprs_string,payload_type)
            
            #if via_hf:
            #    self.add_to_mheard(last_callsign_in_path,self.sx.getRSSI(),self.sx.getSNR())
            
            
        except UnicodeError:
            print("*** UNICODE ERROR ***")
            aprs_string = "unicode_error"
        [zero, hardware_id, lora_mod, fcs, fw, lasthw, fw_subver, ending] = unpack('<BBBHBBBB', msg_end)
        fcs_ok = (calced_fcs == fcs)
        print("ZERO=" + str(zero) + " HWID=" + str(hardware_id) + " LORAMOD=" + str(lora_mod) + " FCS=" + hex(fcs) + "(" + str(fcs_ok) + ") FW=" + str(fw) + "." + chr(fw_subver) + " LASTHW=" + str(lasthw) + " END=" + hex(ending))
        

    elif(payload_type == 0x41):
        text = "ACK msgid: " + hex(message_id) + " (via hf: " + str(via_hf) + ")"
        return text
        
    elif(payload_type == 0x3C):
        print("loraAPRS")
        print(msg)
        return "loraAPRS"
    

def listen_udp(port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("0.0.0.0", port))
        print(f"Listening for UDP packets on port {port}...")
        
        while True:
            data, addr = sock.recvfrom(4096)  # Buffer size of 4096 bytes
            print(f"Received LoraRawFrame from {addr}:")
            print(hex_dump(data))
            decode(data)
            print("-" * 100)

if __name__ == "__main__":
    listen_udp(1798)

