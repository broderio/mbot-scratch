import re
from serial import Serial
from serial.tools.list_ports import comports

def main():
    ports = comports()

    # print("Available ports:")
    # for port, desc, hwid in sorted(ports):
    #     print("{}: {} [{}]".format(port, desc, hwid))
    
    for port, _, hwid in ports:
        if '303A:1001' in hwid or '10C4:EA60' in hwid or '303A:4001' in hwid:
            port_name = port
            break
    else:
        raise Exception('ESP32-S3 COM port not found')

    ser = Serial(port_name, 921600)
    
    macs = open('macs.txt', 'a')

    while True:
        try:
            line = ser.readline().decode('utf-8')
        except UnicodeDecodeError:
            continue
        
        mac_address_pattern = r"([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})"
        m = re.search(mac_address_pattern, line)
        if m:
            addr = m.group(0)
            macs.write(addr + '\n')
            macs.flush()
            
            print(addr)
            return
        
if __name__ == "__main__":
    main()